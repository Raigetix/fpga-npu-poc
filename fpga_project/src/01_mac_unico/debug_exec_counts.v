// Diagnostico temporal: reproduce EXACTAMENTE la logica de acumulacion de
// top.v, pero en vez de devolver el acumulador, devuelve por SPI cuantas
// veces se ejecuto cada rama del case (RESET vs MAC), empaquetado como
// reset_count*256 + mac_count (asi se lee directo como numero decimal en el
// "Recibido de la FPGA" del sketch, sin tocar el firmware). Por cada ciclo de
// prueba del ESP32 se espera reset_count+=1 y mac_count+=3, o sea que el
// valor deberia crecer de a 256+3=259 por ciclo (259, 518, 777, ...).
module top_debug_execcounts (
    input  wire       clk,
    input  wire       sclk,
    input  wire       cs_n,
    input  wire       mosi,
    output wire       miso,
    output wire [5:0] led
);

    localparam [7:0] CMD_NOP   = 8'h00;
    localparam [7:0] CMD_MAC   = 8'h01;
    localparam [7:0] CMD_RESET = 8'h02;

    wire [47:0] rx_data;
    wire        frame_done;
    reg  [47:0] accumulator = 48'd0;

    reg [7:0] mac_count   = 8'd0;
    reg [7:0] reset_count = 8'd0;

    // bits bajos del acumulador (deberian mostrar 0xFDA4 = 64932 en decimal
    // si el acumulador realmente llego a -604 como es esperable) + los
    // contadores de antes en los bytes bajos.
    wire [47:0] tx_debug = {16'd0, accumulator[15:0], reset_count, mac_count};

    spi_slave u_spi (
        .clk       (clk),
        .sclk      (sclk),
        .cs_n      (cs_n),
        .mosi      (mosi),
        .miso      (miso),
        .tx_data   (tx_debug),
        .rx_data   (rx_data),
        .frame_done(frame_done)
    );

    wire [7:0]         cmd  = rx_data[47:40];
    wire signed [15:0] op_a = rx_data[39:24];
    wire signed [15:0] op_b = rx_data[23:8];

    wire signed [17:0] op_a_ext = {{2{op_a[15]}}, op_a};
    wire signed [17:0] op_b_ext = {{2{op_b[15]}}, op_b};
    wire signed [35:0] product  = op_a_ext * op_b_ext;

    always @(posedge clk) begin
        if (frame_done) begin
            case (cmd)
                CMD_MAC: begin
                    accumulator <= accumulator + {{12{product[35]}}, product};
                    mac_count   <= mac_count + 1'b1;
                end
                CMD_RESET: begin
                    accumulator <= 48'd0;
                    reset_count <= reset_count + 1'b1;
                end
                default: accumulator <= accumulator;
            endcase
        end
    end

    assign led = ~{reset_count[2:0], mac_count[2:0]}; // activos en bajo, bonus visual

endmodule
