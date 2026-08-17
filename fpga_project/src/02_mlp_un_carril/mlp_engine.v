// mlp_engine.v -- motor de inferencia secuencial para una red feedforward de
// 4 capas (130 -> 128 -> 64 -> 32 -> 5), pesos/entrada/activaciones en int8,
// acumulador de 32 bits, ReLU + saturacion uniforme en todas las capas.
// Un solo MAC (el multiplicador duro de 18x18 de la FPGA), reutilizado
// secuencialmente para las ~27.040 multiplicaciones de una pasada completa.
//
// LECCION APLICADA DE fpga_NPU_poc/top.v: el primitivo que Gowin usa para nuestra
// multiplicacion es "MULTADDALU18X18" (trae ALU/acumulador incorporado). Un
// patron "acc <= acc + a*b" en UN solo ciclo se fusiona con ese acumulador
// interno y da resultados incorrectos. Por eso el producto del DSP (mul_reg)
// se REGISTRA en un ciclo separado antes de sumarlo al acumulador (S_MUL vs
// S_ACC), igual que tuvimos que hacer en top.v.
//
// MAPA DE MEMORIA DE PESOS (direccion plana; la carga por SPI usa esta MISMA
// numeracion, ver top_mlp.v):
//   Capa 1 (entrada 130 -> oculta1 128): direcciones     0 .. 16639
//   Capa 2 (oculta1 128 -> oculta2  64): direcciones 16640 .. 24831
//   Capa 3 (oculta2  64 -> oculta3  32): direcciones 24832 .. 26879
//   Capa 4 (oculta3  32 -> salida    5): direcciones 26880 .. 27039
//   Orden dentro de cada capa: para neurona n (0..out-1), entrada i (0..in-1):
//     direccion = base_capa + n*in_count + i   (por eso weight_addr solo
//     necesita incrementarse de a 1 en el orden en que recorremos los MACs,
//     sin necesidad de multiplicar para direccionar)
//
// MAPA DE MEMORIA DE BIAS (direccion plana):
//   Capa 1: 0..127   Capa 2: 128..191   Capa 3: 192..223   Capa 4: 224..228
module mlp_engine (
    input  wire clk,

    // ---- Puertos de carga (desde la FSM de comandos SPI en top_mlp.v) ----
    input  wire        load_weight_en,
    input  wire [14:0] load_weight_addr,
    input  wire signed [7:0] load_weight_data,

    input  wire        load_bias_en,
    input  wire [7:0]  load_bias_addr,
    input  wire signed [7:0] load_bias_data,

    input  wire        load_input_en,
    input  wire [7:0]  load_input_addr,
    input  wire signed [7:0] load_input_data,

    // ---- Control ----
    input  wire        start,   // pulso de 1 ciclo: dispara una inferencia completa
    output reg          busy,

    // ---- Resultado (5 neuronas de salida) ----
    output reg signed [7:0] out0,
    output reg signed [7:0] out1,
    output reg signed [7:0] out2,
    output reg signed [7:0] out3,
    output reg signed [7:0] out4,

    // ---- Lectura de diagnostico de capas ocultas (h1/h2/h3), disponible
    // apenas termina una inferencia (busy vuelve a 0), antes de arrancar la
    // siguiente. dbg_sel: 0=h1 1=h2 2=h3. Pipeline de 3 ciclos, mismo patron
    // ya probado en debug_weight_rw.v ----
    input  wire        dbg_rd_en,
    input  wire [1:0]  dbg_rd_sel,
    input  wire [7:0]  dbg_rd_addr,
    output reg signed [7:0] dbg_rd_data
);

    // ================= Memorias =================
    reg signed [7:0] weight_mem [0:27039];
    reg signed [7:0] bias_mem   [0:228];
    reg signed [7:0] input_mem  [0:129];
    reg signed [7:0] h1_mem     [0:127];
    reg signed [7:0] h2_mem     [0:63];
    reg signed [7:0] h3_mem     [0:31];

    always @(posedge clk) if (load_weight_en) weight_mem[load_weight_addr] <= load_weight_data;
    always @(posedge clk) if (load_bias_en)   bias_mem[load_bias_addr]     <= load_bias_data;
    always @(posedge clk) if (load_input_en)  input_mem[load_input_addr]   <= load_input_data;

    // ================= Configuracion por capa =================
    reg [1:0]  layer_idx;      // 0..3
    reg [7:0]  in_count, out_count;
    reg [14:0] layer_weight_base;
    reg [7:0]  layer_bias_base;

    always @(*) begin
        case (layer_idx)
            2'd0: begin in_count=8'd130; out_count=8'd128; layer_weight_base=15'd0;     layer_bias_base=8'd0;   end
            2'd1: begin in_count=8'd128; out_count=8'd64;  layer_weight_base=15'd16640; layer_bias_base=8'd128; end
            2'd2: begin in_count=8'd64;  out_count=8'd32;  layer_weight_base=15'd24832; layer_bias_base=8'd192; end
            default: begin in_count=8'd32; out_count=8'd5; layer_weight_base=15'd26880; layer_bias_base=8'd224; end
        endcase
    end

    // ================= Contadores de recorrido =================
    reg [7:0]  neuron_idx;
    reg [7:0]  input_idx;
    reg [14:0] weight_addr;
    reg [7:0]  bias_addr;
    reg signed [31:0] neuron_acc;

    // ================= Lectura de memorias (latencia 1 ciclo, estilo BRAM) ====
    reg signed [7:0] weight_rdata;
    always @(posedge clk) weight_rdata <= weight_mem[weight_addr];

    reg signed [7:0] bias_rdata;
    always @(posedge clk) bias_rdata <= bias_mem[bias_addr];

    // La "entrada" de una capa es el buffer de salida de la anterior (o
    // input_mem para la capa 1). Se selecciona segun layer_idx.
    reg signed [7:0] actin_rdata;
    always @(posedge clk) begin
        case (layer_idx)
            2'd0: actin_rdata <= input_mem[input_idx];
            2'd1: actin_rdata <= h1_mem[input_idx];
            2'd2: actin_rdata <= h2_mem[input_idx];
            default: actin_rdata <= h3_mem[input_idx];
        endcase
    end

    // ================= Multiplicador (DSP) + etapa de pipeline =================
    wire signed [17:0] w_ext = {{10{weight_rdata[7]}}, weight_rdata};
    wire signed [17:0] a_ext = {{10{actin_rdata[7]}}, actin_rdata};
    wire signed [35:0] mul_result = w_ext * a_ext; // Gowin mapea esto al DSP MULT18X18

    reg signed [31:0] mul_reg; // registro de pipeline: rompe la fusion con el ALU interno del DSP
    always @(posedge clk) mul_reg <= mul_result[31:0];

    // ================= Post-procesamiento: bias + ReLU + saturacion a int8 ====
    wire signed [31:0] biased = neuron_acc + {{24{bias_rdata[7]}}, bias_rdata};
    wire signed [7:0]  activated = (biased < 0) ? 8'sd0 :
                                    (biased > 32'sd127) ? 8'sd127 :
                                    biased[7:0];

    // ================= FSM =================
    localparam [3:0]
        S_IDLE        = 4'd0,
        S_LAYER_INIT  = 4'd1,
        S_NEURON_INIT = 4'd2,
        S_SET_ADDR    = 4'd3,
        S_WAIT_MEM    = 4'd4,
        S_MUL         = 4'd5,
        S_ACC         = 4'd6,
        S_BIAS_WAIT   = 4'd7,
        S_NEURON_DONE = 4'd8,
        S_LAYER_DONE  = 4'd9,
        S_ALL_DONE    = 4'd10;

    reg [3:0] state;

    // Contador de diagnostico: cuantas veces arranco una inferencia de
    // verdad (transicion S_IDLE -> S_LAYER_INIT). Si un "start" espurio
    // dispara una segunda pasada sin que el ESP32 lo haya pedido, esto lo
    // muestra (leible por CMD_DBG_RD con sel=3).
    reg [7:0] start_count = 8'd0;

    always @(posedge clk) begin
        case (state)
            S_IDLE: begin
                busy <= 1'b0;
                if (start) begin
                    busy        <= 1'b1;
                    layer_idx   <= 2'd0;
                    start_count <= start_count + 8'd1;
                    state       <= S_LAYER_INIT;
                end
            end

            S_LAYER_INIT: begin
                // in_count/out_count/layer_weight_base/layer_bias_base ya
                // estan actualizados (combinacional, dependen de layer_idx)
                neuron_idx  <= 8'd0;
                weight_addr <= layer_weight_base;
                bias_addr   <= layer_bias_base;
                state       <= S_NEURON_INIT;
            end

            S_NEURON_INIT: begin
                input_idx  <= 8'd0;
                neuron_acc <= 32'sd0;
                state      <= S_SET_ADDR;
            end

            S_SET_ADDR: begin
                // weight_addr e input_idx ya apuntan al par (neurona, entrada)
                // actual; este estado solo deja que el flanco de reloj dispare
                // la lectura registrada de las memorias.
                state <= S_WAIT_MEM;
            end

            S_WAIT_MEM: begin
                // weight_rdata / actin_rdata quedan validos en ESTE flanco.
                state <= S_MUL;
            end

            S_MUL: begin
                // mul_result (combinacional) ya es valido; se registra en
                // mul_reg en este mismo flanco.
                state <= S_ACC;
            end

            S_ACC: begin
                neuron_acc  <= neuron_acc + mul_reg; // usa el producto YA registrado
                // weight_addr SIEMPRE avanza (los pesos estan en orden continuo
                // neurona-mayor); si no se avanza tambien en la ULTIMA entrada
                // de la neurona, la neurona siguiente arranca leyendo el ultimo
                // peso de esta en vez del primero propio (bug encontrado en
                // pruebas: el desfasaje se acumulaba de a 1 por cada neurona).
                weight_addr <= weight_addr + 15'd1;
                if (input_idx == in_count - 8'd1) begin
                    bias_addr <= layer_bias_base + neuron_idx; // listo para leer el bias de esta neurona
                    state     <= S_BIAS_WAIT;
                end else begin
                    input_idx <= input_idx + 8'd1;
                    state     <= S_SET_ADDR;
                end
            end

            S_BIAS_WAIT: begin
                // bias_rdata queda valido en este flanco (por la direccion
                // fijada al final de S_ACC).
                state <= S_NEURON_DONE;
            end

            S_NEURON_DONE: begin
                // 'activated' (combinacional, usa neuron_acc + bias_rdata) ya
                // es valido en este ciclo.
                case (layer_idx)
                    2'd0: h1_mem[neuron_idx] <= activated;
                    2'd1: h2_mem[neuron_idx] <= activated;
                    2'd2: h3_mem[neuron_idx] <= activated;
                    default: begin
                        case (neuron_idx)
                            8'd0: out0 <= activated;
                            8'd1: out1 <= activated;
                            8'd2: out2 <= activated;
                            8'd3: out3 <= activated;
                            default: out4 <= activated;
                        endcase
                    end
                endcase

                if (neuron_idx == out_count - 8'd1) begin
                    state <= S_LAYER_DONE;
                end else begin
                    neuron_idx <= neuron_idx + 8'd1;
                    state      <= S_NEURON_INIT;
                end
            end

            S_LAYER_DONE: begin
                if (layer_idx == 2'd3) begin
                    state <= S_ALL_DONE;
                end else begin
                    layer_idx <= layer_idx + 2'd1;
                    state     <= S_LAYER_INIT;
                end
            end

            S_ALL_DONE: begin
                busy  <= 1'b0;
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end

    initial begin
        state = S_IDLE;
        busy  = 1'b0;
        out0 = 8'sd0; out1 = 8'sd0; out2 = 8'sd0; out3 = 8'sd0; out4 = 8'sd0;
        dbg_rd_data = 8'sd0;
    end

    // ================= Lectura de diagnostico h1/h2/h3 =================
    reg [1:0] dbg_sel_r;
    reg [7:0] dbg_addr_r;
    always @(posedge clk) begin
        if (dbg_rd_en) begin
            dbg_sel_r  <= dbg_rd_sel;
            dbg_addr_r <= dbg_rd_addr;
        end
    end

    reg signed [7:0] dbg_data_pre;
    always @(posedge clk) begin
        case (dbg_sel_r)
            2'd0: dbg_data_pre <= h1_mem[dbg_addr_r];
            2'd1: dbg_data_pre <= h2_mem[dbg_addr_r];
            2'd2: dbg_data_pre <= h3_mem[dbg_addr_r];
            default: dbg_data_pre <= start_count; // sel=3: contador de arranques
        endcase
    end

    always @(posedge clk) dbg_rd_data <= dbg_data_pre;

endmodule
