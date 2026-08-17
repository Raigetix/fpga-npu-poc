// mac_lane.v -- UN carril de multiplicar-acumular, con su PROPIO banco de
// pesos (memoria independiente = lectura independiente = paralelismo real).
// Reproduce EXACTAMENTE el mismo patron de pipeline ya validado en
// mlp_engine.v (producto del DSP registrado en un ciclo separado antes de
// sumarlo, para no toparnos otra vez con la fusion del ALU interno del DSP)
// -- la unica diferencia es que ahora hay 8 instancias de esto en paralelo
// (ver mlp_engine_par.v), cada una con su propio banco de pesos, compartiendo
// la misma entrada (broadcast) y el mismo control (acc_clear/acc_en) desde
// una unica FSM central.
module mac_lane #(
    parameter integer BANK_ABITS = 13,
    parameter integer BANK_DEPTH = 3392 // direcciones locales realmente usadas
)(
    input  wire                    clk,

    // ---- Carga del banco de pesos de ESTE carril ----
    input  wire                    load_en,
    input  wire [BANK_ABITS-1:0]   load_addr,
    input  wire signed [7:0]       load_data,

    // ---- Lectura durante el computo (direccion compartida entre los 8
    // carriles: todos estan siempre en la misma "ola"/entrada, cada uno
    // la aplica a SU banco) ----
    input  wire [BANK_ABITS-1:0]   rd_addr,
    input  wire signed [7:0]       actin_rdata, // entrada actual, broadcast

    // ---- Control del acumulador (pulsos desde la FSM central) ----
    input  wire                    acc_clear,
    input  wire                    acc_en,

    // ---- Post-procesamiento: bias de la neurona de ESTE carril, ya
    // presentado por la FSM central (broadcast, un carril por vez durante
    // la escritura de resultados) ----
    input  wire signed [7:0]       bias_rdata,
    output wire signed [7:0]       activated,

    // Espejo de solo-lectura de weight_rdata, para que la FSM central pueda
    // exponerlo por SPI en modo debug (verificar que lo que se cargo
    // realmente quedo guardado, sin pasar por el pipeline de MAC).
    output wire signed [7:0]       weight_dbg
);

    // ================= Banco de pesos =================
    // Un solo array (profundidad configurable por instancia via
    // BANK_DEPTH, para no desperdiciar BRAM en modelos chicos). Se probo
    // partirlo en 8 arrays chicos (uno por grupo de olas) para esquivar lo
    // que parecia un limite de bloque fisico de BRAM en Gowin (~1024/2048)
    // -- pero la corrupcion siguio apareciendo en la MISMA zona de
    // direcciones de siempre, ahora en el MEDIO de uno de los chunks
    // chicos, no en ningun limite logico mio. Eso mostro que Gowin estaba
    // empaquetando varios de esos chunks de vuelta en un mismo bloque
    // fisico (uso 46/46 BSRAM, el 100%), asi que partir la memoria en
    // Verilog no controla la organizacion fisica real. El arreglo que si
    // ataca la causa: darle a la lectura un ciclo extra de margen (2
    // registros en cascada en vez de 1), para que cualquier multiplexor de
    // seleccion entre bloques que arme el sintetizador tenga tiempo de
    // asentarse antes de usarse -- mismo patron que ya resolvio la fusion
    // del ALU del DSP en mlp_engine.v. mlp_engine_par*.v tiene un estado
    // extra en la FSM (S_WAIT_MEM2) para darle ese ciclo extra al computo;
    // el pipeline de debug ya tenia de sobra.
    reg signed [7:0] weight_bank [0:BANK_DEPTH-1];

    always @(posedge clk) if (load_en) weight_bank[load_addr] <= load_data;

    reg signed [7:0] weight_rdata;
    reg signed [7:0] weight_rdata2;
    always @(posedge clk) begin
        weight_rdata  <= weight_bank[rd_addr];
        weight_rdata2 <= weight_rdata;
    end
    assign weight_dbg = weight_rdata2;

    wire signed [17:0] w_ext = {{10{weight_rdata2[7]}}, weight_rdata2};
    wire signed [17:0] a_ext = {{10{actin_rdata[7]}}, actin_rdata};
    wire signed [35:0] mul_result = w_ext * a_ext; // Gowin mapea esto al DSP MULT18X18

    reg signed [31:0] mul_reg;
    always @(posedge clk) mul_reg <= mul_result[31:0]; // sin gate, igual que en mlp_engine.v

    reg signed [31:0] neuron_acc;
    always @(posedge clk) begin
        if (acc_clear)   neuron_acc <= 32'sd0;
        else if (acc_en) neuron_acc <= neuron_acc + mul_reg;
    end

    wire signed [31:0] biased = neuron_acc + {{24{bias_rdata[7]}}, bias_rdata};
    assign activated = (biased < 0) ? 8'sd0 :
                        (biased > 32'sd127) ? 8'sd127 :
                        biased[7:0];

endmodule
