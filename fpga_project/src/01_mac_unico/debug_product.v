// Diagnostico temporal: devuelve el PRODUCTO de la ultima multiplicacion
// recibida (sin acumular nada), usando el mismo patron frame_done_d1 que
// top.v (necesario porque spi_slave ahora registra rx_data en dominio clk).
module top_debug_product (
    input  wire       clk,
    input  wire       sclk,
    input  wire       cs_n,
    input  wire       mosi,
    output wire       miso,
    output wire [5:0] led
);

    localparam [7:0] CMD_MAC = 8'h01;

    wire [47:0] rx_data;
    wire        frame_done;
    reg  [47:0] last_product = 48'd0;

    spi_slave u_spi (
        .clk       (clk),
        .sclk      (sclk),
        .cs_n      (cs_n),
        .mosi      (mosi),
        .miso      (miso),
        .tx_data   (last_product),
        .rx_data   (rx_data),
        .frame_done(frame_done)
    );

    reg frame_done_d1 = 1'b0;
    always @(posedge clk) frame_done_d1 <= frame_done;

    wire [7:0]         cmd  = rx_data[47:40];
    wire signed [15:0] op_a = rx_data[39:24];
    wire signed [15:0] op_b = rx_data[23:8];

    wire signed [17:0] op_a_ext = {{2{op_a[15]}}, op_a};
    wire signed [17:0] op_b_ext = {{2{op_b[15]}}, op_b};
    wire signed [35:0] product  = op_a_ext * op_b_ext;

    always @(posedge clk) begin
        if (frame_done_d1 && cmd == CMD_MAC)
            last_product <= {{12{product[35]}}, product};
    end

    assign led = ~last_product[5:0];

endmodule
