// tb_write_verify_ws.v -- igual que tb_write_verify.v pero con weight_stream
// tambien conectado y el arbitro COMPLETO (SPI directo > streaming de pesos
// > refresco), igual que top_sdram_p3.v. Sin "primed": weight_stream puede
// arrancar a pedir apenas sale del reset, igual que en el hardware real que
// fallo la primera vez.
`timescale 1ns/1ps
module tb_write_verify_ws;
    localparam N = 512;

    reg clk = 0;
    always #9.259 clk = ~clk;
    wire clk_sdram = ~clk;

    reg resetn = 0;
    reg wsrst = 0;   // start_pulse simulado

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

    // ---- weight_stream ----
    wire        ws_want_req;
    wire [22:0] ws_addr;
    wire        stream_valid;
    wire signed [7:0] l0,l1,l2,l3,l4,l5,l6,l7;
    reg         stream_pop = 1'b0;   // nadie consume durante esta prueba (computo no corre)

    weight_stream u_ws (
        .clk(clk), .rst(wsrst), .base_addr(23'd0),
        .ws_want_req(ws_want_req), .ws_addr(ws_addr), .ws_issue(want_ws_op),
        .sdram_busy(busy), .sdram_dout32(dout32), .sdram_burst_word_valid(burst_word_valid),
        .data_valid(stream_valid),
        .lane0(l0), .lane1(l1), .lane2(l2), .lane3(l3),
        .lane4(l4), .lane5(l5), .lane6(l6), .lane7(l7),
        .pop(stream_pop)
    );

    // ---- refresco + arbitro COMPLETO, igual que top_sdram_p3.v ----
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

    // ---- modelo de referencia ----
    reg [7:0] expected [0:N-1];
    integer errors;
    integer i;

    // brecha entre comandos SPI consecutivos, igual que en el hardware real
    // (una trama de 48 bits a 8MHz tarda ~6us puros de shift + overhead del
    // driver del ESP32 -- medido en la practica ~16us/byte, ~859 ciclos de
    // clk_sys a 54MHz). Sin esto, weight_stream nunca tiene hueco real para
    // intercalar muchas rafagas entre escrituras, muy distinto del hardware.
    localparam SPI_GAP_CYCLES = 859;

    task do_write(input [22:0] a, input [7:0] d);
        begin
            wait (!busy);
            @(negedge clk);
            pending_addr = a; pending_data = d; pending_wr = 1'b1;
            wait (busy);
            wait (!busy);
            repeat (SPI_GAP_CYCLES) @(posedge clk);
        end
    endtask

    reg [7:0] read_result;
    task do_read(input [22:0] a);
        begin
            wait (!busy);
            @(negedge clk);
            pending_addr = a; pending_rd = 1'b1;
            wait (busy);
            wait (!busy);
            read_result = dout;
            repeat (SPI_GAP_CYCLES) @(posedge clk);
        end
    endtask

    initial begin
        $display("=== tb_write_verify_ws: con weight_stream activo (sin primed) ===");
        resetn = 0;
        rd=0; wr=0; refresh=0; rd_burst=0; addr=0; din=0;
        repeat (5) @(posedge clk);
        resetn = 1;
        wait (!busy);
        $display("[%0t] controlador listo", $time);

        for (i = 0; i < N; i = i + 1) begin
            expected[i] = i[7:0] ^ 8'hA5;
            if (i == 181) model.trace_en = 1'b1;
            if (i >= 181 && i <= 185) $display("[%0t] === escribiendo byte %0d = %02h ===", $time, i, expected[i]);
            do_write(i[22:0], expected[i]);
            if (i == 185) model.trace_en = 1'b0;
        end
        $display("[%0t] escritura terminada", $time);

        errors = 0;
        for (i = 0; i < N; i = i + 1) begin
            if (i == 182) model.trace_en = 1'b1;
            if (i >= 182 && i <= 185) $display("[%0t] === leyendo byte %0d ===", $time, i);
            do_read(i[22:0]);
            if (i == 185) model.trace_en = 1'b0;
            if (read_result !== expected[i]) begin
                errors = errors + 1;
                if (errors <= 20)
                    $display("  MAL addr=%0d esperado=%02h leido=%02h", i, expected[i], read_result);
            end
        end
        $display("=== RESULTADO: %0d errores de %0d bytes ===", errors, N);
        if (errors == 0) $display("PASO"); else $display("FALLO");
        $finish;
    end

    initial begin
        #40000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
