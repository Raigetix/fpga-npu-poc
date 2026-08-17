// Diagnostico temporal: cuenta cuantos frames SPI completos (flancos de CS)
// detecta la FPGA, y lo muestra en los 6 LEDs (contador modulo 64).
// Sirve para saber si el problema esta en la deteccion de frames (CS/framing)
// o en otro lado, sin necesidad de instrumentacion externa.
module top_debug_framecount (
    input  wire clk,
    input  wire sclk,
    input  wire cs_n,
    input  wire mosi,
    output wire miso,
    output wire [5:0] led
);

    wire [47:0] rx_data;
    wire        frame_done;
    reg  [47:0] dummy_tx = 48'd0;

    spi_slave u_spi (
        .clk       (clk),
        .sclk      (sclk),
        .cs_n      (cs_n),
        .mosi      (mosi),
        .miso      (miso),
        .tx_data   (dummy_tx),
        .rx_data   (rx_data),
        .frame_done(frame_done)
    );

    reg [5:0] frame_count = 6'd0;
    always @(posedge clk) begin
        if (frame_done)
            frame_count <= frame_count + 1'b1;
    end

    assign led = ~frame_count; // activos en bajo

endmodule
