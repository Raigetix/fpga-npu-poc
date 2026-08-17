// fpga_NPU_poc — Prueba de concepto minima: 1 unidad MAC (multiplicar-acumular)
// sobre el multiplicador duro de 18x18 de la FPGA, controlada por SPI desde un
// ESP32-S3 (maestro). No es la NPU completa, es el ladrillo basico: probar que
// el ESP32 puede mandar operandos, la FPGA multiplica con el DSP real, y el
// resultado vuelve por SPI.
//
// Protocolo (ver spi_slave.v para el detalle de framing):
//   Frame fijo de 6 bytes (48 bits), full-duplex:
//     MOSI: [CMD(1)] [A_hi(1)] [A_lo(1)] [B_hi(1)] [B_lo(1)] [pad(1)]
//     MISO: acumulador de 48 bits (con signo), MSB primero
//   CMD_NOP=0x00 (no cambia el acumulador, solo sirve para leerlo)
//   CMD_MAC=0x01 (acumulador += A*B, con A y B signed de 16 bits)
//   CMD_RESET=0x02 (acumulador <= 0)
//
//   Latencia: el acumulador solo cambia justo despues de que un frame termina
//   (ver spi_slave.v), asi que la respuesta de un frame siempre refleja el
//   estado de UN frame atras. Por eso el sketch de ESP32 manda 1 frame NOP
//   extra despues del ultimo MAC antes de confiar en el resultado leido.
module top (
    input  wire       clk,   // 27 MHz onboard, pin 4
    input  wire       sclk,  // SPI SCLK, desde ESP32
    input  wire       cs_n,  // SPI CS, activo en bajo, desde ESP32
    input  wire       mosi,  // SPI MOSI, desde ESP32
    output wire       miso,  // SPI MISO, hacia ESP32
    output wire [5:0] led    // debug visual: parte del acumulador, activos en bajo
);

    localparam [7:0] CMD_NOP   = 8'h00;
    localparam [7:0] CMD_MAC   = 8'h01;
    localparam [7:0] CMD_RESET = 8'h02;

    wire [47:0] rx_data;
    wire        frame_done;
    reg  [47:0] accumulator = 48'd0; // registro interno, auto-referenciado (acc <= acc + producto)
    reg  [47:0] tx_snapshot = 48'd0; // copia simple de 'accumulator', sin auto-referencia: es lo
                                      // UNICO que se conecta directo al puerto de spi_slave. Un
                                      // registro auto-referenciado conectado DIRECTO a un puerto que
                                      // otro modulo lee con indice variable (bit_idx) se comprobo
                                      // en placa que da resultados incorrectos (quedaba en 0) con
                                      // este sintetizador; con un registro intermedio no auto-
                                      // referenciado en el medio, funciona bien.

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

    // frame_done retrasado 1 ciclo de 'clk': en el mismo ciclo en que
    // frame_done se activa, rx_data (registrado dentro de spi_slave) todavia
    // no tiene el valor nuevo (semantica no bloqueante) -- un ciclo despues si.
    reg frame_done_d1 = 1'b0;
    always @(posedge clk) frame_done_d1 <= frame_done;

    wire [7:0]         cmd  = rx_data[47:40];
    wire signed [15:0] op_a = rx_data[39:24];
    wire signed [15:0] op_b = rx_data[23:8];
    // rx_data[7:0]: byte de relleno, sin uso

    // Extension de signo de 16 a 18 bits: ancho real del multiplicador duro de Gowin
    wire signed [17:0] op_a_ext = {{2{op_a[15]}}, op_a};
    wire signed [17:0] op_b_ext = {{2{op_b[15]}}, op_b};
    wire signed [35:0] product  = op_a_ext * op_b_ext; // Gowin mapea esto al DSP MULT18X18

    // Etapa de pipeline explicita: registra el producto (y el comando, para
    // mantenerlo alineado) UN ciclo antes de sumarlo al acumulador. Sin esto,
    // el patron "acc <= acc + a*b" en un solo ciclo se probo en placa que da
    // resultados incorrectos (el acumulador quedaba pegado en 0) -- todo
    // indica que Gowin lo estaba fusionando con el acumulador INTERNO del
    // propio bloque DSP, que tiene su propia semantica de reset/enable
    // distinta a la nuestra. Cortando el camino combinacional en dos etapas
    // registradas, el sintetizador ya no puede hacer esa fusion.
    reg signed [35:0] product_reg   = 36'sd0;
    reg  [7:0]        cmd_reg       = 8'h00;
    reg               frame_done_d2 = 1'b0;
    always @(posedge clk) begin
        product_reg   <= product;
        cmd_reg       <= cmd;
        frame_done_d2 <= frame_done_d1;
    end

    always @(posedge clk) begin
        if (frame_done_d2) begin
            case (cmd_reg)
                CMD_MAC:   accumulator <= accumulator + {{12{product_reg[35]}}, product_reg};
                CMD_RESET: accumulator <= 48'd0;
                default:   accumulator <= accumulator; // CMD_NOP u otro: sin cambios
            endcase
        end
        tx_snapshot <= accumulator; // copia simple, todos los ciclos, sin condicion
    end

    assign led = ~accumulator[5:0]; // solo indicativo, activos en bajo

endmodule
