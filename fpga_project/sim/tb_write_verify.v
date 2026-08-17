// tb_write_verify.v -- reproduce en simulacion la secuencia de
// write_weights_to_sdram()+verify_weights() del firmware (escribir N bytes
// secuenciales, despues leerlos de vuelta y comparar), con refresco
// periodico igual que el arbitro real de top_sdram_p3.v. SIN weight_stream
// todavia -- el caso mas simple posible, para ver si el bug ya aparece aca.
`timescale 1ns/1ps
module tb_write_verify;
    localparam N = 4096;   // bytes a probar (cruza 4 filas de 1024 bytes)

    reg clk = 0;
    always #9.259 clk = ~clk;   // ~54 MHz
    wire clk_sdram = ~clk;

    reg resetn = 0;

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

    // ---- refresco periodico, mismo patron que top_sdram_p3.v ----
    localparam REFRESH_INTERVAL = 700;
    reg [9:0] refresh_ctr = 0;
    reg       refresh_pending = 0;
    wire      want_spi_op = !busy && (pending_wr || pending_rd);
    wire      want_refresh = !busy && !want_spi_op && refresh_pending;

    reg pending_wr = 0, pending_rd = 0;
    reg [22:0] pending_addr;
    reg [7:0]  pending_data;

    always @(posedge clk) begin
        if (refresh_ctr == REFRESH_INTERVAL) begin
            refresh_ctr <= 0;
            refresh_pending <= 1'b1;
        end else refresh_ctr <= refresh_ctr + 1;
        if (want_refresh) refresh_pending <= 1'b0;
    end

    always @(posedge clk) begin
        wr <= 1'b0; rd <= 1'b0; refresh <= 1'b0;
        if (want_spi_op) begin
            addr <= pending_addr;
            if (pending_wr) begin din <= pending_data; wr <= 1'b1; pending_wr <= 1'b0; end
            else begin rd <= 1'b1; pending_rd <= 1'b0; end
        end else if (want_refresh) begin
            refresh <= 1'b1;
        end
    end

    // ---- modelo de referencia (lo que "deberia" haber en cada direccion) ----
    reg [7:0] expected [0:N-1];
    integer errors;
    integer i;

    task do_write(input [22:0] a, input [7:0] d);
        begin
            wait (!busy);
            @(negedge clk);   // fijar a mitad de ciclo, lejos del flanco que mira el arbitro
            pending_addr = a; pending_data = d; pending_wr = 1'b1;
            wait (busy);      // el controlador lo tomo
            wait (!busy);     // termino
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
        end
    endtask

    initial begin
        $display("=== tb_write_verify: escribiendo y verificando %0d bytes ===", N);
        resetn = 0;
        rd = 0; wr = 0; refresh = 0; rd_burst = 0; addr = 0; din = 0;
        repeat (5) @(posedge clk);
        resetn = 1;

        // esperar a que el controlador salga de INIT/CONFIG
        wait (!busy);
        $display("[%0t] controlador listo (fin de INIT/CONFIG)", $time);

        // ---- escribir N bytes secuenciales, direccion 0..N-1 ----
        for (i = 0; i < N; i = i + 1) begin
            expected[i] = i[7:0] ^ 8'hA5;   // patron facil de reconocer
            do_write(i[22:0], expected[i]);
        end
        $display("[%0t] escritura terminada", $time);

        // ---- leer y comparar ----
        errors = 0;
        for (i = 0; i < N; i = i + 1) begin
            do_read(i[22:0]);
            if (read_result !== expected[i]) begin
                errors = errors + 1;
                if (errors <= 20)
                    $display("  MAL addr=%0d esperado=%02h leido=%02h", i, expected[i], read_result);
            end
        end

        $display("=== RESULTADO: %0d errores de %0d bytes ===", errors, N);
        if (errors == 0) $display("PASO");
        else $display("FALLO");
        $finish;
    end

    // watchdog
    initial begin
        #5000000;
        $display("TIMEOUT -- la simulacion no termino a tiempo");
        $finish;
    end
endmodule
