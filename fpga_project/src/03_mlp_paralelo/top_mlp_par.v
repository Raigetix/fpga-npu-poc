// top_mlp_par.v -- top-level de la version paralela (8 carriles MAC) de la
// PoC de inferencia. Reutiliza spi_slave.v sin cambios (PLL propia, ver
// pll_par.v). La entrada se carga en RAFAGA (5 bytes utiles por frame de 48
// bits) porque su tiempo de carga SI cuenta en la comparacion de velocidad
// contra el ESP32 (se repite en cada inferencia). Los PESOS, en cambio, se
// cargan de a uno por transaccion SPI (como en la version single-lane, ya
// probada confiable): la carga en rafaga de pesos (8 bancos, secuenciador de
// varios ciclos) resulto ser una fuente real de corrupcion intermitente e
// irreproducible entre resets (mismos datos, mismo bitstream, tasas de
// fallo de 0.4% a 78% segun la corrida) -- no se identifico la causa raiz
// exacta pese a variar la frecuencia de clk_sys, asi que se elimino esa
// complejidad en vez de perseguir una condicion de carrera esquiva. Como la
// carga de pesos es un costo unico al arrancar (no se repite en el loop de
// inferencia), no afecta el objetivo real (ganarle al ESP32 por inferencia).
//
// Protocolo (frame de 6 bytes = 48 bits, igual forma que top_mlp.v):
//   MOSI: [CMD(1)] [A_hi(1)] [A_lo(1)] [B_hi(1)] [B_lo(1)] [pad(1)]
//   MISO: [status(1): bit0=busy] [out0][out1][out2][out3][out4 o dbg]
//
//   CMD_NOP        = 0x00
//   CMD_LOAD_W     = 0x03  A[14:12]=carril(0..7) A[11:0]=direccion local
//                          B_lo=peso int8 (escritura directa, 1 SPI = 1 peso)
//   CMD_LOAD_BIAS  = 0x02  A=direccion de bias (0..228)  B_lo=bias int8
//   CMD_START_INFER= 0x04
//   CMD_DBG_RD     = 0x06  A[10:8]=capa(0=h1,1=h2,2=h3,3=start_count,4=input_mem,
//                          5=weight_bank[w_lane][w_ptr], estos 2 ultimos fijados
//                          por el CMD_SET_WTGT mas reciente) A[7:0]=indice
//   CMD_SET_WTGT   = 0x07  A[14:12]=carril(0..7) A[11:0]=direccion local
//                          (solo para apuntar CMD_DBG_RD sel=5, no escribe)
//   CMD_SET_ITGT   = 0x0A  A[7:0]=direccion base de entrada (arranca/reinicia
//                          el puntero de rafaga de entrada)
//   CMD_IBURST5    = 0x0B  usa los 5 bytes no-CMD del frame como payload:
//                          escribe input_mem[ptr..ptr+4] y avanza ptr en 5.
//
//   Los comandos de carga se ignoran mientras busy=1, o mientras una rafaga
//   de entrada anterior todavia se esta escribiendo (dura solo 5 ciclos de
//   clk_sys, nunca se solapa con el siguiente frame SPI en la practica).
module top_mlp_par (
    input  wire       clk,   // 27 MHz onboard, pin 4 (entrada cruda a la PLL)
    input  wire       sclk,  // SPI SCLK, desde ESP32
    input  wire       cs_n,  // SPI CS, activo en bajo, desde ESP32
    input  wire       mosi,  // SPI MOSI, desde ESP32
    output wire       miso,  // SPI MISO, hacia ESP32
    output wire [5:0] led    // debug visual: busy + bits bajos de out0, activos en bajo
);

    localparam [7:0] CMD_NOP      = 8'h00;
    localparam [7:0] CMD_LOAD_W   = 8'h03;
    localparam [7:0] CMD_LOAD_B   = 8'h02;
    localparam [7:0] CMD_START    = 8'h04;
    localparam [7:0] CMD_DBG_RD   = 8'h06;
    localparam [7:0] CMD_SET_WTGT = 8'h07;
    localparam [7:0] CMD_SET_ITGT = 8'h0A;
    localparam [7:0] CMD_IBURST5  = 8'h0B;

    wire clk_sys;
    wire pll_lock;
    pll_par u_pll (
        .clk_in (clk),
        .clk_out(clk_sys),
        .lock   (pll_lock)
    );

    wire [47:0] rx_data;
    wire        frame_done;
    reg  [47:0] tx_snapshot = 48'd0;

    spi_slave u_spi (
        .clk       (clk_sys),
        .sclk      (sclk),
        .cs_n      (cs_n),
        .mosi      (mosi),
        .miso      (miso),
        .tx_data   (tx_snapshot),
        .rx_data   (rx_data),
        .frame_done(frame_done)
    );

    reg frame_done_d1 = 1'b0;
    always @(posedge clk_sys) frame_done_d1 <= frame_done;

    wire [7:0]  cmd        = rx_data[47:40];
    wire [15:0] addr_field = rx_data[39:24];
    wire [15:0] data_field = rx_data[23:8];

    wire              busy;
    wire signed [7:0] out0, out1, out2, out3, out4;
    wire signed [7:0] dbg_rd_data;

    // Objetivo de debug de pesos (CMD_SET_WTGT), sin secuenciador: solo
    // apunta CMD_DBG_RD sel=5. La escritura real de pesos (CMD_LOAD_W) es
    // directa, 1 ciclo, sin puntero propio -- ver comentario de cabecera.
    reg [11:0]       w_ptr;
    reg [2:0]        w_lane;

    // ---- Secuenciador de rafaga de ENTRADA: 5 escrituras en 5 ciclos ----
    reg [2:0]        ib_step = 3'd0;
    reg signed [7:0] ib_bytes [0:4];
    reg [7:0]        i_ptr;

    reg        imac_en;
    reg [7:0]  imac_addr;
    reg signed [7:0] imac_data;

    // ---- Decodificador de comandos de 1 ciclo (los que no son rafaga) ----
    reg load_b_en, load_w_en, start_pulse, dbg_rd_en, set_wtgt, set_itgt;
    wire cmd_ok = frame_done_d1 && !busy && (ib_step == 3'd0);
    always @(*) begin
        load_b_en   = 1'b0;
        load_w_en   = 1'b0;
        start_pulse = 1'b0;
        dbg_rd_en   = 1'b0;
        set_wtgt    = 1'b0;
        set_itgt    = 1'b0;
        if (cmd_ok) begin
            case (cmd)
                CMD_LOAD_B:   load_b_en   = 1'b1;
                CMD_LOAD_W:   load_w_en   = 1'b1;
                CMD_START:    start_pulse = 1'b1;
                CMD_DBG_RD:   dbg_rd_en   = 1'b1;
                CMD_SET_WTGT: set_wtgt    = 1'b1;
                CMD_SET_ITGT: set_itgt    = 1'b1;
                default: ;
            endcase
        end
    end

    // w_ptr/w_lane solo se tocan aca (un unico driver).
    always @(posedge clk_sys) begin
        if (set_wtgt) begin
            w_lane <= addr_field[14:12];
            w_ptr  <= addr_field[11:0];
        end
    end

    // ---- Arranque y avance de la rafaga de entrada (mismo criterio) ----
    always @(posedge clk_sys) begin
        imac_en <= 1'b0;
        if (set_itgt) begin
            i_ptr <= addr_field[7:0];
        end else if (ib_step == 3'd0) begin
            if (cmd_ok && cmd == CMD_IBURST5) begin
                ib_bytes[0] <= rx_data[39:32];
                ib_bytes[1] <= rx_data[31:24];
                ib_bytes[2] <= rx_data[23:16];
                ib_bytes[3] <= rx_data[15:8];
                ib_bytes[4] <= rx_data[7:0];
                ib_step     <= 3'd1;
            end
        end else begin
            imac_en   <= 1'b1;
            imac_addr <= i_ptr + (ib_step - 3'd1);
            imac_data <= ib_bytes[ib_step - 3'd1];
            if (ib_step == 3'd5) begin
                i_ptr   <= i_ptr + 8'd5;
                ib_step <= 3'd0;
            end else begin
                ib_step <= ib_step + 3'd1;
            end
        end
    end

    reg dbg_mode;
    always @(posedge clk_sys) begin
        if (frame_done_d1) begin
            if (cmd == CMD_DBG_RD) dbg_mode <= 1'b1;
            else if (cmd == CMD_START) dbg_mode <= 1'b0;
        end
    end

    mlp_engine_par u_mlp (
        .clk              (clk_sys),
        .load_weight_en   (load_w_en),
        .load_weight_lane (addr_field[14:12]),
        .load_weight_addr (addr_field[11:0]),
        .load_weight_data (data_field[7:0]),
        .load_bias_en     (load_b_en),
        .load_bias_addr   (addr_field[7:0]),
        .load_bias_data   (data_field[7:0]),
        .load_input_en    (imac_en),
        .load_input_addr  (imac_addr),
        .load_input_data  (imac_data),
        .start            (start_pulse),
        .busy             (busy),
        .out0(out0), .out1(out1), .out2(out2), .out3(out3), .out4(out4),
        .dbg_rd_en   (dbg_rd_en),
        .dbg_rd_sel  (addr_field[10:8]),
        .dbg_rd_addr (addr_field[7:0]),
        .dbg_rd_data (dbg_rd_data),
        .dbg_w_lane  (w_lane),
        .dbg_w_addr  (w_ptr)
    );

    always @(posedge clk_sys) begin
        tx_snapshot <= {6'd0, pll_lock, busy, out0, out1, out2, out3, (dbg_mode ? dbg_rd_data : out4)};
    end

    assign led = ~{busy, out0[4:0]};

endmodule
