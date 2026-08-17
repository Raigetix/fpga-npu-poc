// Diagnostico temporal: acumula +1 (constante, NO el producto del DSP) en un
// registro de 48 bits cada vez que llega un MAC. Aisla si el problema es
// "registro ancho auto-referenciado leido por spi_slave" en general, o algo
// especifico de sumar el producto del multiplicador.
module top_debug_wideaccum (
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
    reg  [47:0] wide_acc = 48'd0;
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

    wire [7:0] cmd = rx_data[47:40];

    always @(posedge clk) begin
        if (frame_done_d1) begin
            if (cmd == CMD_MAC)
                wide_acc <= wide_acc + 48'd1;
            else if (cmd == CMD_RESET)
                wide_acc <= 48'd0;
        end
        tx_snapshot <= wide_acc;
    end

    assign led = ~wide_acc[5:0];

endmodule
