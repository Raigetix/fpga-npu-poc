// Diagnostico temporal: siempre devuelve la constante 0x0000_0000_1234 (+4660
// decimal) por MISO, sin importar los comandos que lleguen. Sirve para probar
// el camino de LECTURA (bit_idx -> miso) de forma aislada: si el ESP32 recibe
// exactamente 4660, esa parte funciona bien.
module top_debug_misoconst (
    input  wire clk,
    input  wire sclk,
    input  wire cs_n,
    input  wire mosi,
    output wire miso,
    output wire [5:0] led
);

    wire [47:0] rx_data;
    wire        frame_done;
    localparam [47:0] TEST_PATTERN = 48'h0000_0000_1234;

    spi_slave u_spi (
        .clk       (clk),
        .sclk      (sclk),
        .cs_n      (cs_n),
        .mosi      (mosi),
        .miso      (miso),
        .tx_data   (TEST_PATTERN),
        .rx_data   (rx_data),
        .frame_done(frame_done)
    );

    assign led = ~rx_data[5:0]; // debug extra: bits bajos de lo ultimo recibido

endmodule
