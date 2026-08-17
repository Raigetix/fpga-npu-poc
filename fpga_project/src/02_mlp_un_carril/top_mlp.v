// top_mlp.v -- top-level de la PoC de inferencia: carga de pesos/bias/entrada
// por SPI hacia mlp_engine (red 130->128->64->32->5), dispara la inferencia y
// devuelve el resultado. Reutiliza spi_slave.v tal cual (sin cambios) y el
// mismo patron de registro-espejo (tx_snapshot) aprendido en top.v para no
// exponer nada auto-referenciado directo al puerto variablemente indexado.
//
// Protocolo (frame fijo de 6 bytes = 48 bits, igual que top.v):
//   MOSI: [CMD(1)] [A_hi(1)] [A_lo(1)] [B_hi(1)] [B_lo(1)] [pad(1)]
//   MISO: [status(1): bit0=busy] [out0][out1][out2][out3][out4 o dbg]
//
//   CMD_NOP        = 0x00  (no hace nada, solo sirve para leer el estado)
//   CMD_LOAD_WEIGHT= 0x01  A=direccion de peso (0..27039)  B_lo=peso int8
//   CMD_LOAD_BIAS  = 0x02  A=direccion de bias (0..228)    B_lo=bias int8
//   CMD_LOAD_INPUT = 0x03  A=indice de entrada (0..129)    B_lo=valor int8
//   CMD_START_INFER= 0x04  dispara una pasada completa (ignorado si busy=1)
//   CMD_DBG_READ   = 0x06  A[9:8]=capa (0=h1,1=h2,2=h3) A[7:0]=indice de neurona
//                          -> el ultimo byte de la respuesta muestra ese valor
//                          en vez de out4, hasta el proximo comando
//
//   Los comandos de carga se ignoran mientras busy=1 (protege la memoria de
//   pesos durante una inferencia en curso). CMD_DBG_READ solo tiene sentido
//   leerlo cuando busy=0 (despues de que termino una inferencia, antes de
//   arrancar la siguiente): h1/h2/h3 mantienen el ultimo valor calculado.
module top_mlp (
    input  wire       clk,   // 27 MHz onboard, pin 4
    input  wire       sclk,  // SPI SCLK, desde ESP32
    input  wire       cs_n,  // SPI CS, activo en bajo, desde ESP32
    input  wire       mosi,  // SPI MOSI, desde ESP32
    output wire       miso,  // SPI MISO, hacia ESP32
    output wire [5:0] led    // debug visual: busy + bits bajos de out0, activos en bajo
);

    localparam [7:0] CMD_NOP     = 8'h00;
    localparam [7:0] CMD_LOAD_W  = 8'h01;
    localparam [7:0] CMD_LOAD_B  = 8'h02;
    localparam [7:0] CMD_LOAD_I  = 8'h03;
    localparam [7:0] CMD_START   = 8'h04;
    localparam [7:0] CMD_DBG_RD  = 8'h06;

    wire [47:0] rx_data;
    wire        frame_done;
    reg  [47:0] tx_snapshot = 48'd0;

    spi_slave u_spi (
        .clk       (clk),
        .sclk      (sclk),
        .cs_n      (cs_n),
        .mosi      (mosi),
        .miso      (miso),
        .tx_data   (tx_snapshot),
        .rx_data   (rx_data),
        .frame_done(frame_done)
    );

    reg frame_done_d1 = 1'b0;
    always @(posedge clk) frame_done_d1 <= frame_done;

    wire [7:0]  cmd        = rx_data[47:40];
    wire [15:0] addr_field = rx_data[39:24];
    wire [15:0] data_field = rx_data[23:8];

    wire              busy;
    wire signed [7:0] out0, out1, out2, out3, out4;
    wire signed [7:0] dbg_rd_data;

    reg load_w_en, load_b_en, load_i_en, start_pulse, dbg_rd_en;
    reg dbg_mode; // 1: el ultimo byte de la respuesta muestra dbg_rd_data en vez de out4
    always @(*) begin
        load_w_en   = 1'b0;
        load_b_en   = 1'b0;
        load_i_en   = 1'b0;
        start_pulse = 1'b0;
        dbg_rd_en   = 1'b0;
        if (frame_done_d1 && !busy) begin
            case (cmd)
                CMD_LOAD_W: load_w_en   = 1'b1;
                CMD_LOAD_B: load_b_en   = 1'b1;
                CMD_LOAD_I: load_i_en   = 1'b1;
                CMD_START:  start_pulse = 1'b1;
                CMD_DBG_RD: dbg_rd_en   = 1'b1;
                default: ; // CMD_NOP u otro: nada
            endcase
        end
    end

    always @(posedge clk) begin
        if (frame_done_d1) begin
            if (cmd == CMD_DBG_RD) dbg_mode <= 1'b1;
            else if (cmd == CMD_START) dbg_mode <= 1'b0; // al arrancar una inferencia, volver a mostrar out4
        end
    end

    mlp_engine u_mlp (
        .clk              (clk),
        .load_weight_en   (load_w_en),
        .load_weight_addr (addr_field[14:0]),
        .load_weight_data (data_field[7:0]),
        .load_bias_en     (load_b_en),
        .load_bias_addr   (addr_field[7:0]),
        .load_bias_data   (data_field[7:0]),
        .load_input_en    (load_i_en),
        .load_input_addr  (addr_field[7:0]),
        .load_input_data  (data_field[7:0]),
        .start            (start_pulse),
        .busy             (busy),
        .out0(out0), .out1(out1), .out2(out2), .out3(out3), .out4(out4),
        .dbg_rd_en   (dbg_rd_en),
        .dbg_rd_sel  (addr_field[9:8]),
        .dbg_rd_addr (addr_field[7:0]),
        .dbg_rd_data (dbg_rd_data)
    );

    always @(posedge clk) begin
        tx_snapshot <= {7'd0, busy, out0, out1, out2, out3, (dbg_mode ? dbg_rd_data : out4)};
    end

    assign led = ~{busy, out0[4:0]};

endmodule
