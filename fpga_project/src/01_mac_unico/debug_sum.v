// Diagnostico temporal: acumula (A+B) de cada MAC recibido (sin multiplicar),
// usando el patron ya confirmado que funciona (registro espejo + frame_done
// retrasado). A diferencia del +1 fijo, esto SI depende de los valores reales
// que manda el ESP32: 12+3=15, -7+20=13, 100+(-5)=95 -> total esperado 123.
module top_debug_sum (
    input  wire       clk,
    input  wire       sclk,
    input  wire       cs_n,
    input  wire       mosi,
    output wire       miso,
    output wire [5:0] led
);

    localparam [7:0] CMD_MAC   = 8'h01;
    localparam [7:0] CMD_RESET = 8'h02;

    wire [47:0] rx_data;
    wire        frame_done;
    reg  [47:0] accumulator = 48'd0;
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

    wire [7:0]         cmd  = rx_data[47:40];
    wire signed [15:0] op_a = rx_data[39:24];
    wire signed [15:0] op_b = rx_data[23:8];
    wire signed [16:0] sum  = op_a + op_b; // suma simple, sin pasar por el DSP

    always @(posedge clk) begin
        if (frame_done_d1) begin
            if (cmd == CMD_MAC)
                accumulator <= accumulator + {{31{sum[16]}}, sum};
            else if (cmd == CMD_RESET)
                accumulator <= 48'd0;
        end
        tx_snapshot <= accumulator;
    end

    assign led = ~accumulator[5:0];

endmodule
