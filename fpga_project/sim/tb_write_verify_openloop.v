// tb_write_verify_openloop.v -- reproduce EXACTAMENTE el protocolo real del
// firmware: manda CMD_SDRAM_WR/CMD_SDRAM_RD a ciegas, SIN consultar 'busy'
// para nada -- confia en que el tiempo de una trama SPI (~6us a 8MHz) le
// alcanza de sobra a la SDRAM (que tarda <200ns por operacion). Para las
// lecturas replica el patron real: una lectura de "calentamiento" antes del
// loop, despues cada trama entrega la respuesta de la ANTERIOR (pipeline de
// un paso, ver protocolo SDRAM_RD).
`timescale 1ns/1ps
module tb_write_verify_openloop;
    localparam N = 4096;

    reg clk = 0;
    always #9.259 clk = ~clk;
    wire clk_sdram = ~clk;

    reg resetn = 0;
    reg wsrst = 0;

    reg         rd, wr, refresh, rd_burst;
    reg  [22:0] addr;
    reg  [7:0]  din;
    wire [7:0]  dout;
    wire [31:0] dout32;
    wire        burst_word_valid;
    wire        data_ready;
    wire        busy;

    wire [31:0] SDRAM_DQ;
    wire [10:0] SDRAM_A;
    wire [1:0]  SDRAM_BA;
    wire        SDRAM_nCS, SDRAM_nWE, SDRAM_nRAS, SDRAM_nCAS, SDRAM_CLK, SDRAM_CKE;
    wire [3:0]  SDRAM_DQM;

    sdram #(.FREQ(54_000_000), .T_RP(4'd2), .T_RCD(4'd2)) dut (
        .clk(clk), .clk_sdram(clk_sdram), .resetn(resetn),
        .rd(rd), .wr(wr), .refresh(refresh), .rd_burst(rd_burst),
        .addr(addr), .din(din), .dout(dout), .dout32(dout32),
        .burst_word_valid(burst_word_valid), .data_ready(data_ready), .busy(busy),
        .SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA),
        .SDRAM_nCS(SDRAM_nCS), .SDRAM_nWE(SDRAM_nWE), .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_DQM(SDRAM_DQM)
    );

    sdram_model model (
        .SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA),
        .SDRAM_nCS(SDRAM_nCS), .SDRAM_nWE(SDRAM_nWE), .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_DQM(SDRAM_DQM)
    );

    wire        ws_want_req;
    wire [22:0] ws_addr;
    wire        stream_valid;
    wire signed [7:0] l0,l1,l2,l3,l4,l5,l6,l7;
    reg         stream_pop = 1'b0;

    weight_stream u_ws (
        .clk(clk), .rst(wsrst), .base_addr(23'd0),
        .ws_want_req(ws_want_req), .ws_addr(ws_addr), .ws_issue(want_ws_op),
        .sdram_busy(busy), .sdram_dout32(dout32), .sdram_burst_word_valid(burst_word_valid),
        .data_valid(stream_valid),
        .lane0(l0), .lane1(l1), .lane2(l2), .lane3(l3),
        .lane4(l4), .lane5(l5), .lane6(l6), .lane7(l7),
        .pop(stream_pop)
    );

    localparam REFRESH_INTERVAL = 700;
    reg [9:0] refresh_ctr = 0;
    reg       refresh_pending = 0;

    reg pending_wr = 0, pending_rd = 0;
    reg [22:0] pending_addr;
    reg [7:0]  pending_data;

    wire want_spi_op  = !busy && (pending_wr || pending_rd);
    wire want_ws_op   = !busy && !want_spi_op && ws_want_req;
    wire want_refresh = !busy && !want_spi_op && !want_ws_op && refresh_pending;

    always @(posedge clk) begin
        if (refresh_ctr == REFRESH_INTERVAL) begin
            refresh_ctr <= 0;
            refresh_pending <= 1'b1;
        end else refresh_ctr <= refresh_ctr + 1;
        if (want_refresh) refresh_pending <= 1'b0;
    end

    always @(posedge clk) begin
        wr <= 1'b0; rd <= 1'b0; refresh <= 1'b0; rd_burst <= 1'b0;
        if (want_spi_op) begin
            addr <= pending_addr;
            if (pending_wr) begin din <= pending_data; wr <= 1'b1; pending_wr <= 1'b0; end
            else begin rd <= 1'b1; pending_rd <= 1'b0; end
        end else if (want_ws_op) begin
            addr <= ws_addr;
            rd_burst <= 1'b1;
        end else if (want_refresh) begin
            refresh <= 1'b1;
        end
    end

    // ---- "snapshot" de dout, igual que tx_snapshot en top_sdram_p3.v: una
    // trama SPI dura FRAME_CYCLES, y lo que "le llega" al firmware es el
    // valor de dout en el instante justo antes de que termine la trama
    // (CS volviendo a alto). No miramos busy para nada, EXACTO como hace
    // npu_xfer() de verdad. ----
    localparam FRAME_CYCLES = 324;   // 48 bits a 8MHz relativo a clk_sys ~54MHz

    // 'snapshot' se toma AL PRINCIPIO de la trama (antes de mandar el pedido
    // de ESTA trama) -- ese es el valor que "llega" en esta trama: el
    // resultado del pedido de la trama ANTERIOR, tal como tx_snapshot lo
    // congela en el hardware real (se actualiza en el hueco entre tramas,
    // se congela mientras la trama esta en curso).
    reg [7:0] snapshot;
    task spi_frame_write(input [22:0] a, input [7:0] d);
        begin
            @(negedge clk);
            pending_addr = a; pending_data = d; pending_wr = 1'b1;
            repeat (FRAME_CYCLES) @(posedge clk);
        end
    endtask

    task spi_frame_read(input [22:0] a);
        begin
            snapshot = dout;   // respuesta de la trama ANTERIOR
            @(negedge clk);
            pending_addr = a; pending_rd = 1'b1;
            repeat (FRAME_CYCLES) @(posedge clk);
        end
    endtask

    reg [7:0] expected [0:N-1];
    integer errors;
    integer i;
    integer next_addr;

    initial begin
        $display("=== tb_write_verify_openloop: protocolo real (sin consultar busy) ===");
        resetn = 0;
        rd=0; wr=0; refresh=0; rd_burst=0; addr=0; din=0;
        repeat (5) @(posedge clk);
        resetn = 1;
        wait (!busy);
        $display("[%0t] controlador listo", $time);

        // ---- escritura, igual que write_weights_to_sdram(): tramas CMD_SDRAM_WR
        // seguidas, sin mirar busy ----
        for (i = 0; i < N; i = i + 1) begin
            expected[i] = i[7:0] ^ 8'hA5;
            spi_frame_write(i[22:0], expected[i]);
        end
        $display("[%0t] escritura terminada", $time);

        // ---- verificacion, igual que verify_weights(): UNA lectura de
        // "calentamiento" antes del loop, despues cada trama entrega la
        // respuesta de la lectura ANTERIOR ----
        errors = 0;
        spi_frame_read(23'd0);   // calentamiento (direccion 0, respuesta descartada)
        for (i = 0; i < N; i = i + 1) begin
            next_addr = i + 1;
            if (i + 1 < N)
                spi_frame_read(next_addr[22:0]);   // pide la SIGUIENTE mientras llega la respuesta de esta
            else
                spi_frame_read(23'd0);              // ultima vuelta, direccion no importa
            if (snapshot !== expected[i]) begin
                errors = errors + 1;
                if (errors <= 20)
                    $display("  MAL addr=%0d esperado=%02h leido=%02h", i, expected[i], snapshot);
            end
        end
        $display("=== RESULTADO: %0d errores de %0d bytes ===", errors, N);
        if (errors == 0) $display("PASO"); else $display("FALLO");
        $finish;
    end

    initial begin
        #100000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
