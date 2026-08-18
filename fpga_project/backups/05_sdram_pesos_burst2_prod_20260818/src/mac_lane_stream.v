// mac_lane_stream.v -- version de mac_lane.v (03_mlp_paralelo/) SIN banco de
// pesos local: el peso de este carril llega ya leido y verificado desde
// weight_stream.v (Fase 3), en vez de vivir en un weight_bank de BRAM propio.
// El pipeline de multiplicar-acumular es EXACTAMENTE el mismo (mismo motivo
// para el producto del DSP registrado antes de sumarlo, ver mac_lane.v) --
// unico cambio real: donde antes habia una lectura de BRAM de 2 etapas
// (weight_rdata/weight_rdata2), ahora esas 2 etapas registran el valor que
// ya llego por weight_in, para no tocar el pipeline de computo que ya esta
// probado.
//
// Cuantizacion estandar (int8, escala+zero-point, ver README.md seccion
// Etapa 4): a diferencia de la version anterior, este modulo YA NO aplica
// ReLU/saturacion -- solo entrega el acumulador + bias SIN reescalar
// (`biased`, 32 bits con signo). El reescalado (multiplicar por el factor
// de la capa, desplazar, sumar el zero-point de salida, y recien ahi
// aplicar los limites de activacion) se hace UNA VEZ, compartido entre los
// 8 carriles, en mlp_engine_par_stream.v -- ahorra 7 multiplicadores (la
// escritura de resultados ya es secuencial, un carril por vez, asi que no
// hace falta que cada carril tenga su propia unidad de reescalado).
module mac_lane_stream (
    input  wire                    clk,

    // ---- Peso de este carril para el paso actual, ya verificado por
    // weight_stream.v. Se captura UNA SOLA VEZ, en el ciclo en que la FSM
    // confirma que el grupo es valido (w_load), en vez de ir copiando la
    // cabeza de la FIFO en todos los ciclos y confiar en que el valor
    // correcto este ahi justo cuando lo usa el multiplicador. Esa version
    // anterior perdia el peso de la ULTIMA iteracion de cada ola (llegaba
    // 0 al multiplicador), lo que se detecto escribiendo en cada grupo su
    // propio numero de grupo: con in_count=5 la ola 0 sumaba 0+1+2+3=6 en
    // vez de 0+1+2+3+4=10, y la ola 1 sumaba 5+6+7+8=26 en vez de 35 --
    // o sea, los 5 grupos SI se consumian (el alineamiento entre olas
    // quedaba intacto) pero el ultimo producto daba 0.
    input  wire signed [7:0]       weight_in,
    input  wire                    w_load,

    // ---- Entrada actual, broadcast entre los 8 carriles ----
    input  wire signed [7:0]       actin_rdata,

    // ---- Control del acumulador (pulsos desde la FSM central) ----
    input  wire                    acc_clear,
    input  wire                    acc_en,

    // ---- Bias YA CORREGIDO por el zero-point de entrada ("bias
    // efectivo", calculado del lado del host antes de cargarlo -- ver
    // README.md), 32 bits con signo (puede ser bastante mas grande que lo
    // que entra en 8 bits) ----
    input  wire signed [31:0]      bias_in,
    output wire signed [31:0]      biased
);

    reg signed [7:0] weight_rdata2;
    always @(posedge clk) begin
        if (w_load) weight_rdata2 <= weight_in;
    end

    wire signed [17:0] w_ext = {{10{weight_rdata2[7]}}, weight_rdata2};
    wire signed [17:0] a_ext = {{10{actin_rdata[7]}}, actin_rdata};
    wire signed [35:0] mul_result = w_ext * a_ext; // Gowin mapea esto al DSP MULT18X18

    // syn_keep: EVITAR que Gowin fusione multiplicador + acumulador dentro
    // de un DSP MULTADDALU18X18. Esa fusion es la causa raiz del bug que
    // costo toda una tanda de diagnostico: el reporte de sintesis de la
    // version que ANDABA bien (fase 3 sin cuantizacion, 50/50 correctas)
    // muestra "MULT9X9 x8" (multiplicador solo, acumulador en logica
    // normal), mientras que la version rota muestra "MULTADDALU18X18 x8"
    // (multiplicar-y-acumular fusionado). El macro fusionado tiene su
    // propia semantica de reset/enable -- exactamente la trampa que este
    // proyecto ya documento en la etapa 1 (ver README) -- y hacia que se
    // perdiera el ULTIMO producto de cada ola. Por eso ningun ajuste de
    // margen de ciclos servia: el acumulador no estaba en la logica que se
    // estaba ajustando, estaba adentro del DSP.
    (* syn_keep = 1 *) reg signed [31:0] mul_reg;
    always @(posedge clk) mul_reg <= mul_result[31:0];

    (* syn_keep = 1 *) reg signed [31:0] neuron_acc;
    always @(posedge clk) begin
        if (acc_clear)   neuron_acc <= 32'sd0;
        else if (acc_en) neuron_acc <= neuron_acc + mul_reg;
    end

    assign biased = neuron_acc + bias_in;

endmodule
