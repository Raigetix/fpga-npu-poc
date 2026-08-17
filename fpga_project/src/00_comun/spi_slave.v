// Esclavo SPI generico, modo 0 (CPOL=0, CPHA=0), frame de ancho fijo FRAME_BITS
// (asume FRAME_BITS <= 64 por el ancho de bit_idx). El maestro (ESP32) controla
// CS y SCLK; este modulo es full-duplex: mientras entra un frame nuevo por
// MOSI, sale por MISO el 'tx_data' vigente (ver nota de latencia en top.v).
module spi_slave #(
    parameter integer FRAME_BITS = 48
)(
    input  wire                   clk,        // reloj de sistema (27MHz), dominio "lento" del diseño
    input  wire                   sclk,       // SCLK del SPI, generado por el ESP32 (asincrono a clk)
    input  wire                   cs_n,       // chip select activo en bajo (asincrono a clk)
    input  wire                   mosi,
    output wire                   miso,
    input  wire [FRAME_BITS-1:0]  tx_data,    // valor a devolver, se lee en vivo bit a bit
    output reg  [FRAME_BITS-1:0]  rx_data = {FRAME_BITS{1'b0}}, // ultimo frame recibido, en dominio 'clk'
    output wire                   frame_done  // pulso de 1 ciclo de 'clk' por cada frame terminado
);

    // ---- Dominio SCLK: captura de MOSI, puramente sincrona (sin control
    // asincrono: nada que objetar aca) ----
    reg [FRAME_BITS-1:0] shift_in = {FRAME_BITS{1'b0}};
    always @(posedge sclk) begin
        if (!cs_n)
            shift_in <= {shift_in[FRAME_BITS-2:0], mosi}; // MSB primero
    end

    // ---- Dominio SCLK: indice de bit para MISO. En vez de precargar un
    // shift-register con un valor variable de forma asincrona (Gowin lo
    // desaconseja explicitamente: "async set and reset" mixto sobre datos no
    // constantes, riesgo real de que no funcione en la placa), el reset
    // asincrono de este contador es a una CONSTANTE fija (FRAME_BITS-1), que
    // si es valido. 'tx_data' se indexa de forma combinacional: como el
    // acumulador (en top.v) solo cambia justo despues de que un frame termina,
    // se mantiene estable durante todo el frame siguiente, asi que leerlo asi
    // es seguro (no hay "tearing" de bits a mitad de lectura).
    reg [5:0] bit_idx;
    always @(negedge sclk or posedge cs_n) begin
        if (cs_n)
            bit_idx <= FRAME_BITS - 1;
        else
            bit_idx <= bit_idx - 1'b1;
    end

    assign miso = tx_data[bit_idx];

    // ---- Cruce a dominio 'clk': mismo patron sincronizador+detector de flanco
    // que debounce.v/edge_detect.v del proyecto test_mult_18bits ----
    reg cs_sync0 = 1'b1, cs_sync1 = 1'b1, cs_sync2 = 1'b1;
    always @(posedge clk) begin
        cs_sync0 <= cs_n;
        cs_sync1 <= cs_sync0;
        cs_sync2 <= cs_sync1;
    end

    assign frame_done = cs_sync1 & ~cs_sync2; // flanco de subida de CS, ya sincronizado

    // rx_data SI se registra en el dominio 'clk': es la forma correcta de
    // cruzar shift_in (dominio sclk) hacia clk, en vez de leerlo combinacional
    // (eso generaba un camino de datos sclk->clk sin sincronizar, invisible
    // para el analisis de timing y con comportamiento real impredecible).
    // El consumidor (top.v) debe usar 'frame_done' retrasado 1 ciclo de clk
    // para leer cmd/operandos, ya que en el MISMO flanco en que frame_done se
    // activa, rx_data todavia no tiene el valor nuevo (semantica no bloqueante).
    always @(posedge clk) begin
        if (frame_done)
            rx_data <= shift_in;
    end

endmodule
