// Diagnostico temporal: acumula (OR bit a bit) todos los valores de CMD que la
// FPGA vio alguna vez en el byte de comando, y los muestra en los 6 LEDs.
// Como CMD_NOP=0x00, CMD_MAC=0x01 y CMD_RESET=0x02 solo usan los 2 bits mas
// bajos, si la captura de MOSI funciona bien el patron deberia estabilizarse
// en binario 000011 (0x01 | 0x02 = 0x03) despues de un ciclo de prueba.
module top_debug_cmdmask (
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

    reg [5:0] cmd_mask = 6'd0;
    always @(posedge clk) begin
        if (frame_done)
            cmd_mask <= cmd_mask | rx_data[45:40]; // bits bajos del byte CMD (rx_data[47:40])
    end

    assign led = ~cmd_mask; // activos en bajo

endmodule
