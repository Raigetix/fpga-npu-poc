// top_mlp_fast.v -- igual que top_mlp.v, pero la logica interna (spi_slave +
// mlp_engine) corre a 108MHz (generados por PLL a partir del oscilador de
// 27MHz) en vez de a los 27MHz crudos. El puerto 'clk' de este modulo sigue
// siendo el pin fisico de 27MHz (no se toca top.cst); 'clk_sys' es la senal
// interna de 108MHz que efectivamente usan spi_slave y mlp_engine.
//
// Protocolo: identico a top_mlp.v (ver ese archivo para el detalle).
module top_mlp_fast (
    input  wire       clk,   // 27 MHz onboard, pin 4 (entrada cruda a la PLL)
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

    wire clk_sys; // 108 MHz
    wire pll_lock;
    pll_108mhz u_pll (
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

    reg load_w_en, load_b_en, load_i_en, start_pulse, dbg_rd_en;
    reg dbg_mode;
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
                default: ;
            endcase
        end
    end

    always @(posedge clk_sys) begin
        if (frame_done_d1) begin
            if (cmd == CMD_DBG_RD) dbg_mode <= 1'b1;
            else if (cmd == CMD_START) dbg_mode <= 1'b0;
        end
    end

    mlp_engine u_mlp (
        .clk              (clk_sys),
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

    always @(posedge clk_sys) begin
        tx_snapshot <= {6'd0, pll_lock, busy, out0, out1, out2, out3, (dbg_mode ? dbg_rd_data : out4)};
    end

    assign led = ~{busy, out0[4:0]};

endmodule
