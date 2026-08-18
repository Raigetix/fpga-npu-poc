// mlp_engine_par_stream.v -- Etapa 4, Fase 3+: NPU generica de 8 carriles,
// COMPLETAMENTE configurable por SPI -- cantidad de capas, entradas,
// neuronas por capa, salidas, pesos, bias y parametros de cuantizacion,
// todo cargado en caliente sin recompilar el hardware. Limites fijados una
// unica vez, generosos, en la sintesis (MAX_LAYERS/MAX_INPUT_WIDTH/
// MAX_LAYER_WIDTH mas abajo); cualquier modelo que entre en esos limites
// se carga y corre sin volver a tocar la FPGA.
//
// Streaming de pesos: el barrido de pesos de la red COMPLETA (todas las
// capas, en el orden en que se procesan) es una unica secuencia monotona
// que weight_stream.v recorre sin saber nada de capas -- el host tiene que
// escribir los pesos en SDRAM en ESE mismo orden (capa 0 primero,
// contigua, despues capa 1, etc.). Ver weight_stream.v.
//
// Buffers de activacion: en vez de una memoria nombrada por capa (H1/H2/
// H3, que solo alcanza para una cantidad FIJA de capas), se usan DOS
// buffers de "ping-pong" (actbuf_a/actbuf_b) que se van alternando: la
// capa L lee de un buffer y escribe en el otro, sin importar cuantas capas
// haya en total. La entrada del usuario vive aparte (input_mem, mas ancha,
// para cubrir modelos con muchas entradas como MNIST) y la salida de la
// ULTIMA capa va a out_mem (se lee por SPI con el mismo mecanismo de
// debug que el resto de las memorias).
//
// CUANTIZACION ESTANDAR (int8, escala+zero-point, estilo TFLite -- ver
// README.md seccion Etapa 4): los pesos se cuantizan siempre simetricos
// (zero-point de peso = 0); la correccion por el zero-point de ENTRADA se
// precalcula del lado del host y se esconde en el bias ("bias efectivo").
// El reescalado de salida (multiplicar+desplazar+zero-point+limites) es
// por capa, en un unico lugar compartido entre los 8 carriles (estados
// S_REQUANT1/S_REQUANT2).
module mlp_engine_par_stream #(
    parameter integer QSHIFT = 20 // desplazamiento fijo del reescalado (ver mas abajo)
) (
    input  wire clk,
    input  wire rst, // reinicia weight_stream (pulso, ver top_sdram_p3.v)

    // ---- SDRAM: interfaz hacia el arbitro compartido del top-level ----
    output wire         ws_want_req,
    output wire [22:0]  ws_addr,
    output wire         ws_is_burst2,
    input  wire         ws_issue,
    input  wire         sdram_busy,
    input  wire [31:0]  sdram_dout32,
    input  wire [31:0]  sdram_dout32_a,
    input  wire [31:0]  sdram_dout32_b,
    input  wire [22:0]  weight_base_addr,

    // ---- Forma del modelo (configurable por SPI) ----
    input  wire        load_num_layers_en,
    input  wire [3:0]  load_num_layers_value,   // 1..8

    input  wire        load_layer_shape_en,
    input  wire [2:0]  load_layer_shape_layer,  // 0..7
    input  wire [1:0]  load_layer_shape_sel,     // 0=in_count 1=out_count 2=bias_base
    input  wire [15:0] load_layer_shape_value,

    // Bias EFECTIVO (32 bits, ya con la correccion del zero-point de
    // entrada aplicada del lado del host -- ver comentario de cabecera).
    input  wire        load_bias_en,
    input  wire [10:0] load_bias_addr,
    input  wire signed [31:0] load_bias_data,

    // Parametros de cuantizacion por capa. load_qparam_sel: 0=multiplicador
    // (32 bits, real_multiplier*2^QSHIFT) 1=zero_point de salida 2=minimo
    // de activacion 3=maximo de activacion.
    input  wire        load_qparam_en,
    input  wire [2:0]  load_qparam_layer,
    input  wire [1:0]  load_qparam_sel,
    input  wire signed [31:0] load_qparam_value,

    input  wire        load_input_en,
    input  wire [10:0] load_input_addr,
    input  wire signed [7:0] load_input_data,

    input  wire        start,
    output reg          busy,

    input  wire        dbg_rd_en,
    input  wire [2:0]  dbg_rd_sel,   // 0=actbuf_a 1=actbuf_b 3=start_count 4=input_mem 5=out_mem 6=bias_mem(8 bits bajos)
    input  wire [10:0] dbg_rd_addr,
    output reg signed [7:0] dbg_rd_data
);

    localparam integer NLANES = 8;

    // ---- Limites generosos, fijados una unica vez -- ver comentario de
    // cabecera. Cualquier modelo dentro de estos limites no necesita
    // recompilar nunca mas.
    localparam integer MAX_LAYERS      = 8;
    localparam integer MAX_INPUT_WIDTH = 1024;
    localparam integer MAX_LAYER_WIDTH = 256;
    localparam integer MAX_BIAS        = MAX_LAYERS * MAX_LAYER_WIDTH;

    reg signed [31:0] bias_mem   [0:MAX_BIAS-1];
    reg signed [7:0]  input_mem  [0:MAX_INPUT_WIDTH-1];
    reg signed [7:0]  actbuf_a   [0:MAX_LAYER_WIDTH-1];
    reg signed [7:0]  actbuf_b   [0:MAX_LAYER_WIDTH-1];
    reg signed [7:0]  out_mem    [0:MAX_LAYER_WIDTH-1];

    always @(posedge clk) if (load_bias_en)  bias_mem[load_bias_addr]   <= load_bias_data;
    always @(posedge clk) if (load_input_en) input_mem[load_input_addr] <= load_input_data;

    // ================= Forma del modelo (configurable por SPI) =================
    // Escritura: indices CONSTANTES en cada rama del case (no "array
    // [variable]"), asi que es segura sin importar el patron de la etapa 1.
    reg [3:0]  num_layers_reg = 4'd1;
    reg [10:0] in_count_reg   [0:MAX_LAYERS-1];
    reg [8:0]  out_count_reg  [0:MAX_LAYERS-1];
    reg [10:0] bias_base_reg  [0:MAX_LAYERS-1];

    always @(posedge clk) begin
        if (load_num_layers_en) num_layers_reg <= load_num_layers_value;
        if (load_layer_shape_en) begin
            case (load_layer_shape_layer)
                3'd0: case (load_layer_shape_sel) 2'd0: in_count_reg[0]<=load_layer_shape_value[10:0]; 2'd1: out_count_reg[0]<=load_layer_shape_value[8:0]; 2'd2: bias_base_reg[0]<=load_layer_shape_value[10:0]; default: ; endcase
                3'd1: case (load_layer_shape_sel) 2'd0: in_count_reg[1]<=load_layer_shape_value[10:0]; 2'd1: out_count_reg[1]<=load_layer_shape_value[8:0]; 2'd2: bias_base_reg[1]<=load_layer_shape_value[10:0]; default: ; endcase
                3'd2: case (load_layer_shape_sel) 2'd0: in_count_reg[2]<=load_layer_shape_value[10:0]; 2'd1: out_count_reg[2]<=load_layer_shape_value[8:0]; 2'd2: bias_base_reg[2]<=load_layer_shape_value[10:0]; default: ; endcase
                3'd3: case (load_layer_shape_sel) 2'd0: in_count_reg[3]<=load_layer_shape_value[10:0]; 2'd1: out_count_reg[3]<=load_layer_shape_value[8:0]; 2'd2: bias_base_reg[3]<=load_layer_shape_value[10:0]; default: ; endcase
                3'd4: case (load_layer_shape_sel) 2'd0: in_count_reg[4]<=load_layer_shape_value[10:0]; 2'd1: out_count_reg[4]<=load_layer_shape_value[8:0]; 2'd2: bias_base_reg[4]<=load_layer_shape_value[10:0]; default: ; endcase
                3'd5: case (load_layer_shape_sel) 2'd0: in_count_reg[5]<=load_layer_shape_value[10:0]; 2'd1: out_count_reg[5]<=load_layer_shape_value[8:0]; 2'd2: bias_base_reg[5]<=load_layer_shape_value[10:0]; default: ; endcase
                3'd6: case (load_layer_shape_sel) 2'd0: in_count_reg[6]<=load_layer_shape_value[10:0]; 2'd1: out_count_reg[6]<=load_layer_shape_value[8:0]; 2'd2: bias_base_reg[6]<=load_layer_shape_value[10:0]; default: ; endcase
                default: case (load_layer_shape_sel) 2'd0: in_count_reg[7]<=load_layer_shape_value[10:0]; 2'd1: out_count_reg[7]<=load_layer_shape_value[8:0]; 2'd2: bias_base_reg[7]<=load_layer_shape_value[10:0]; default: ; endcase
            endcase
        end
    end

    // Lectura: selector explicito por layer_idx (mismo motivo de siempre:
    // lectura de array por indice variable, patron a evitar segun la
    // etapa 1 de este proyecto).
    reg [10:0] in_count_sel;
    reg [8:0]  out_count_sel;
    reg [10:0] bias_base_sel;
    always @(*) begin
        case (layer_idx)
            3'd0: begin in_count_sel=in_count_reg[0]; out_count_sel=out_count_reg[0]; bias_base_sel=bias_base_reg[0]; end
            3'd1: begin in_count_sel=in_count_reg[1]; out_count_sel=out_count_reg[1]; bias_base_sel=bias_base_reg[1]; end
            3'd2: begin in_count_sel=in_count_reg[2]; out_count_sel=out_count_reg[2]; bias_base_sel=bias_base_reg[2]; end
            3'd3: begin in_count_sel=in_count_reg[3]; out_count_sel=out_count_reg[3]; bias_base_sel=bias_base_reg[3]; end
            3'd4: begin in_count_sel=in_count_reg[4]; out_count_sel=out_count_reg[4]; bias_base_sel=bias_base_reg[4]; end
            3'd5: begin in_count_sel=in_count_reg[5]; out_count_sel=out_count_reg[5]; bias_base_sel=bias_base_reg[5]; end
            3'd6: begin in_count_sel=in_count_reg[6]; out_count_sel=out_count_reg[6]; bias_base_sel=bias_base_reg[6]; end
            default: begin in_count_sel=in_count_reg[7]; out_count_sel=out_count_reg[7]; bias_base_sel=bias_base_reg[7]; end
        endcase
    end
    // Cantidad de "olas" (grupos de 8 neuronas de salida): derivada, no
    // hace falta que el host la mande.
    wire [5:0] num_waves_sel = (out_count_sel + 9'd7) >> 3;

    wire is_first_layer = (layer_idx == 3'd0);
    // Resta en el ancho completo (4 bits) ANTES de truncar a 3 -- num_layers
    // va de 1 a 8, "ultima capa" (num_layers-1) siempre da 0..7, entra
    // justo en 3 bits, pero hay que restar primero y truncar despues (al
    // reves se pisan dos errores que "casualmente" cancelan solo para
    // num_layers=8, fragil).
    wire [3:0] last_layer_idx = num_layers_reg - 4'd1;
    wire is_last_layer = (layer_idx == last_layer_idx[2:0]);

    // ================= Parametros de cuantizacion por capa =================
    reg signed [31:0] q_mult_reg [0:MAX_LAYERS-1];
    reg signed [7:0]  q_zp_reg   [0:MAX_LAYERS-1];
    reg signed [7:0]  q_min_reg  [0:MAX_LAYERS-1];
    reg signed [7:0]  q_max_reg  [0:MAX_LAYERS-1];
    always @(posedge clk) begin
        if (load_qparam_en) begin
            case (load_qparam_layer)
                3'd0: case (load_qparam_sel) 2'd0: q_mult_reg[0]<=load_qparam_value; 2'd1: q_zp_reg[0]<=load_qparam_value[7:0]; 2'd2: q_min_reg[0]<=load_qparam_value[7:0]; default: q_max_reg[0]<=load_qparam_value[7:0]; endcase
                3'd1: case (load_qparam_sel) 2'd0: q_mult_reg[1]<=load_qparam_value; 2'd1: q_zp_reg[1]<=load_qparam_value[7:0]; 2'd2: q_min_reg[1]<=load_qparam_value[7:0]; default: q_max_reg[1]<=load_qparam_value[7:0]; endcase
                3'd2: case (load_qparam_sel) 2'd0: q_mult_reg[2]<=load_qparam_value; 2'd1: q_zp_reg[2]<=load_qparam_value[7:0]; 2'd2: q_min_reg[2]<=load_qparam_value[7:0]; default: q_max_reg[2]<=load_qparam_value[7:0]; endcase
                3'd3: case (load_qparam_sel) 2'd0: q_mult_reg[3]<=load_qparam_value; 2'd1: q_zp_reg[3]<=load_qparam_value[7:0]; 2'd2: q_min_reg[3]<=load_qparam_value[7:0]; default: q_max_reg[3]<=load_qparam_value[7:0]; endcase
                3'd4: case (load_qparam_sel) 2'd0: q_mult_reg[4]<=load_qparam_value; 2'd1: q_zp_reg[4]<=load_qparam_value[7:0]; 2'd2: q_min_reg[4]<=load_qparam_value[7:0]; default: q_max_reg[4]<=load_qparam_value[7:0]; endcase
                3'd5: case (load_qparam_sel) 2'd0: q_mult_reg[5]<=load_qparam_value; 2'd1: q_zp_reg[5]<=load_qparam_value[7:0]; 2'd2: q_min_reg[5]<=load_qparam_value[7:0]; default: q_max_reg[5]<=load_qparam_value[7:0]; endcase
                3'd6: case (load_qparam_sel) 2'd0: q_mult_reg[6]<=load_qparam_value; 2'd1: q_zp_reg[6]<=load_qparam_value[7:0]; 2'd2: q_min_reg[6]<=load_qparam_value[7:0]; default: q_max_reg[6]<=load_qparam_value[7:0]; endcase
                default: case (load_qparam_sel) 2'd0: q_mult_reg[7]<=load_qparam_value; 2'd1: q_zp_reg[7]<=load_qparam_value[7:0]; 2'd2: q_min_reg[7]<=load_qparam_value[7:0]; default: q_max_reg[7]<=load_qparam_value[7:0]; endcase
            endcase
        end
    end

    reg signed [31:0] q_mult_sel;
    reg signed [7:0]  q_zp_sel, q_min_sel, q_max_sel;
    always @(*) begin
        case (layer_idx)
            3'd0: begin q_mult_sel=q_mult_reg[0]; q_zp_sel=q_zp_reg[0]; q_min_sel=q_min_reg[0]; q_max_sel=q_max_reg[0]; end
            3'd1: begin q_mult_sel=q_mult_reg[1]; q_zp_sel=q_zp_reg[1]; q_min_sel=q_min_reg[1]; q_max_sel=q_max_reg[1]; end
            3'd2: begin q_mult_sel=q_mult_reg[2]; q_zp_sel=q_zp_reg[2]; q_min_sel=q_min_reg[2]; q_max_sel=q_max_reg[2]; end
            3'd3: begin q_mult_sel=q_mult_reg[3]; q_zp_sel=q_zp_reg[3]; q_min_sel=q_min_reg[3]; q_max_sel=q_max_reg[3]; end
            3'd4: begin q_mult_sel=q_mult_reg[4]; q_zp_sel=q_zp_reg[4]; q_min_sel=q_min_reg[4]; q_max_sel=q_max_reg[4]; end
            3'd5: begin q_mult_sel=q_mult_reg[5]; q_zp_sel=q_zp_reg[5]; q_min_sel=q_min_reg[5]; q_max_sel=q_max_reg[5]; end
            3'd6: begin q_mult_sel=q_mult_reg[6]; q_zp_sel=q_zp_reg[6]; q_min_sel=q_min_reg[6]; q_max_sel=q_max_reg[6]; end
            default: begin q_mult_sel=q_mult_reg[7]; q_zp_sel=q_zp_reg[7]; q_min_sel=q_min_reg[7]; q_max_sel=q_max_reg[7]; end
        endcase
    end

    // ================= Contadores de recorrido =================
    reg [2:0]  layer_idx;
    reg [5:0]  wave_idx;
    reg [10:0] input_idx;
    reg [10:0] bias_addr;
    reg [2:0]  lane_ctr;

    // Entrada de la capa actual: la capa 0 lee de input_mem; las demas
    // alternan entre actbuf_a/actbuf_b segun la paridad de layer_idx (ver
    // comentario de cabecera).
    reg signed [7:0] actin_rdata;
    always @(posedge clk) begin
        if (is_first_layer)     actin_rdata <= input_mem[input_idx];
        else if (layer_idx[0])  actin_rdata <= actbuf_a[input_idx];
        else                    actin_rdata <= actbuf_b[input_idx];
    end

    reg signed [31:0] bias_rdata;
    always @(posedge clk) bias_rdata <= bias_mem[bias_addr];

    // ================= Motor de streaming de pesos =================
    wire               stream_valid;
    wire signed [7:0]  ws_lane0, ws_lane1, ws_lane2, ws_lane3, ws_lane4, ws_lane5, ws_lane6, ws_lane7;
    reg                stream_pop;

    weight_stream u_wstream (
        .clk         (clk),
        .rst         (rst),
        .base_addr   (weight_base_addr),
        .ws_want_req   (ws_want_req),
        .ws_addr       (ws_addr),
        .ws_is_burst2  (ws_is_burst2),
        .ws_issue      (ws_issue),
        .sdram_busy    (sdram_busy),
        .sdram_dout32  (sdram_dout32),
        .sdram_dout32_a(sdram_dout32_a),
        .sdram_dout32_b(sdram_dout32_b),
        .data_valid  (stream_valid),
        .lane0(ws_lane0), .lane1(ws_lane1), .lane2(ws_lane2), .lane3(ws_lane3),
        .lane4(ws_lane4), .lane5(ws_lane5), .lane6(ws_lane6), .lane7(ws_lane7),
        .pop         (stream_pop)
    );

    // ================= FSM =================
    localparam [4:0]
        S_IDLE       = 5'd0,
        S_LAYER_INIT = 5'd1,
        S_WAVE_INIT  = 5'd2,
        S_SET_ADDR   = 5'd3,
        S_WAIT_MEM   = 5'd4, // espera variable a stream_valid (SDRAM, no BRAM)
        S_WAIT_MEM2  = 5'd5,
        S_MUL        = 5'd6,
        S_ACC        = 5'd7,
        S_DRAIN      = 5'd16, // margen para que "aterrice" la ultima suma
        S_BIAS_ADDR  = 5'd8,
        S_BIAS_WAIT  = 5'd9,
        S_REQUANT1   = 5'd10,
        S_REQUANT2   = 5'd11,
        S_BIAS_WRITE = 5'd12,
        S_WAVE_DONE  = 5'd13,
        S_LAYER_DONE = 5'd14,
        S_ALL_DONE   = 5'd15;

    reg [4:0] state;
    // Contador del margen de "drenaje": Gowin fusiona el acumulador en un
    // DSP MULTADDALU18X18 (8 en el reporte, uno por carril) que tiene sus
    // propios registros internos, asi que el valor final aparece VARIOS
    // ciclos despues de lo que sugiere el RTL. Se detecto que faltaba
    // exactamente el ultimo producto de cada ola (con pesos = numero de
    // grupo, la ola 0 sumaba 0+1+2+3=6 en vez de 10, y la ola 1
    // 5+6+7+8=26 en vez de 35), pese a que los 5 grupos SI se consumian.
    // Pasar la captura de 2 a 3 ciclos no alcanzo; en vez de adivinar el
    // numero exacto se deja un margen holgado.
    reg [2:0] drain_ctr;
    wire acc_clear = (state == S_WAVE_INIT);
    wire acc_en    = (state == S_ACC);
    // Captura del peso: exactamente en el ciclo en que se confirma que el
    // grupo esta disponible (misma condicion con la que la FSM sale de
    // S_WAIT_MEM). Ver mac_lane_stream.v para por que se captura una sola
    // vez en vez de ir copiando la cabeza de la FIFO todos los ciclos.
    wire w_load = (state == S_WAIT_MEM) && stream_valid;

    reg [7:0] start_count = 8'd0;

    wire [7:0] neuron_global = {wave_idx[4:0], 3'b000} + {5'd0, lane_ctr};

    // ---- Reescalado: producto de 64 bits (biased de 32 bits * multiplicador
    // de 32 bits), desplazado QSHIFT bits, mas el zero-point de salida,
    // saturado a [q_min_sel, q_max_sel].
    // OJO -- el acumulador necesita UN CICLO MAS de margen del que sugiere
    // el Verilog: Gowin mapea `neuron_acc <= neuron_acc + mul_reg` a un DSP
    // MULTADDALU18X18 (8 de esos en el reporte, uno por carril) y ese macro
    // tiene registros internos propios, asi que el valor final aparece un
    // ciclo despues de lo que dice el RTL. Es la misma trampa que el ALU
    // interno del DSP ya habia tendido en la etapa 1 de este proyecto (ver
    // README) y se arregla igual: dar margen.
    //
    // Capturar aca en S_BIAS_WAIT (2 ciclos despues del ultimo S_ACC) hacia
    // que la ULTIMA suma de cada ola no llegara a tiempo -- se perdia
    // exactamente un producto por neurona, sistematicamente. Se detecto con
    // un test de "todos los pesos y entradas en 1", que daba in_count-1 en
    // vez de in_count para cualquier in_count (3->2, 5->4, 20->19). El
    // diseño anterior (mlp_engine_par.v, sin requantizacion) usaba el
    // acumulador recien 3 ciclos despues del ultimo S_ACC, por eso nunca
    // habia aparecido. Ahora se captura en S_REQUANT1 (3 ciclos despues).
    reg signed [63:0] requant_product;
    always @(posedge clk) begin
        if (state == S_REQUANT1) requant_product <= lane_biased_sel * q_mult_sel;
    end

    wire signed [63:0] requant_shifted = requant_product >>> QSHIFT;
    wire signed [63:0] requant_zp      = requant_shifted + {{56{q_zp_sel[7]}}, q_zp_sel};
    wire signed [63:0] requant_min_ext = {{56{q_min_sel[7]}}, q_min_sel};
    wire signed [63:0] requant_max_ext = {{56{q_max_sel[7]}}, q_max_sel};
    wire signed [7:0]  requant_clamped = (requant_zp < requant_min_ext) ? q_min_sel :
                                          (requant_zp > requant_max_ext) ? q_max_sel :
                                          requant_zp[7:0];
    // Corrido un ciclo junto con el producto (ver comentario de arriba):
    // el multiplicador ademas queda con un ciclo entero para si solo, que
    // es sano para el camino critico de 32x32 bits.
    reg signed [7:0] lane_final;
    always @(posedge clk) begin
        if (state == S_REQUANT2) lane_final <= requant_clamped;
    end

    always @(posedge clk) begin
        stream_pop <= 1'b0;

        case (state)
            S_IDLE: begin
                busy <= 1'b0;
                if (start) begin
                    busy        <= 1'b1;
                    layer_idx   <= 3'd0;
                    start_count <= start_count + 8'd1;
                    state       <= S_LAYER_INIT;
                end
            end

            S_LAYER_INIT: begin
                wave_idx <= 6'd0;
                state    <= S_WAVE_INIT;
            end

            S_WAVE_INIT: begin
                input_idx <= 11'd0;
                state     <= S_SET_ADDR;
            end

            S_SET_ADDR: state <= S_WAIT_MEM;

            S_WAIT_MEM: if (stream_valid) state <= S_WAIT_MEM2;

            S_WAIT_MEM2: state <= S_MUL;
            S_MUL:       state <= S_ACC;

            S_ACC: begin
                stream_pop <= 1'b1; // este grupo ya se uso, avanzar la FIFO
                if (input_idx == in_count_sel - 11'd1) begin
                    lane_ctr  <= 3'd0;
                    bias_addr <= bias_base_sel + {wave_idx, 3'd0};
                    drain_ctr <= 3'd0;
                    state     <= S_DRAIN;
                end else begin
                    input_idx <= input_idx + 11'd1;
                    state     <= S_SET_ADDR;
                end
            end

            // Margen holgado (6 ciclos) para que la ULTIMA suma de la ola
            // salga de los registros internos del DSP antes de leer el
            // acumulador -- ver comentario de drain_ctr arriba.
            S_DRAIN: begin
                if (drain_ctr == 3'd5) state <= S_BIAS_ADDR;
                else                   drain_ctr <= drain_ctr + 3'd1;
            end

            S_BIAS_ADDR: state <= S_BIAS_WAIT;
            S_BIAS_WAIT: state <= S_REQUANT1;
            S_REQUANT1:  state <= S_REQUANT2;
            S_REQUANT2:  state <= S_BIAS_WRITE;

            S_BIAS_WRITE: begin
                if ({1'b0, neuron_global} < out_count_sel) begin
                    if (is_last_layer) begin
                        out_mem[neuron_global] <= lane_final;
                    end else if (~layer_idx[0]) begin
                        actbuf_a[neuron_global] <= lane_final;
                    end else begin
                        actbuf_b[neuron_global] <= lane_final;
                    end
                end

                if (lane_ctr == NLANES - 1) begin
                    state <= S_WAVE_DONE;
                end else begin
                    lane_ctr  <= lane_ctr + 1'b1;
                    bias_addr <= bias_base_sel + {wave_idx, 3'd0} + {8'd0, lane_ctr} + 11'd1;
                    state     <= S_BIAS_ADDR;
                end
            end

            S_WAVE_DONE: begin
                if (wave_idx == num_waves_sel - 6'd1) begin
                    state <= S_LAYER_DONE;
                end else begin
                    wave_idx <= wave_idx + 1'b1;
                    state    <= S_WAVE_INIT;
                end
            end

            S_LAYER_DONE: begin
                if (is_last_layer) begin
                    state <= S_ALL_DONE;
                end else begin
                    layer_idx <= layer_idx + 3'd1;
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
        dbg_rd_data = 8'sd0;
    end

    // ================= 8 carriles MAC en paralelo (sin banco de pesos
    // local -- reciben su byte directo de weight_stream) =================
    wire signed [31:0] lane_biased [0:NLANES-1];

    mac_lane_stream u_lane0 (.clk(clk), .weight_in(ws_lane0), .w_load(w_load), .actin_rdata(actin_rdata), .acc_clear(acc_clear), .acc_en(acc_en), .bias_in(bias_rdata), .biased(lane_biased[0]));
    mac_lane_stream u_lane1 (.clk(clk), .weight_in(ws_lane1), .w_load(w_load), .actin_rdata(actin_rdata), .acc_clear(acc_clear), .acc_en(acc_en), .bias_in(bias_rdata), .biased(lane_biased[1]));
    mac_lane_stream u_lane2 (.clk(clk), .weight_in(ws_lane2), .w_load(w_load), .actin_rdata(actin_rdata), .acc_clear(acc_clear), .acc_en(acc_en), .bias_in(bias_rdata), .biased(lane_biased[2]));
    mac_lane_stream u_lane3 (.clk(clk), .weight_in(ws_lane3), .w_load(w_load), .actin_rdata(actin_rdata), .acc_clear(acc_clear), .acc_en(acc_en), .bias_in(bias_rdata), .biased(lane_biased[3]));
    mac_lane_stream u_lane4 (.clk(clk), .weight_in(ws_lane4), .w_load(w_load), .actin_rdata(actin_rdata), .acc_clear(acc_clear), .acc_en(acc_en), .bias_in(bias_rdata), .biased(lane_biased[4]));
    mac_lane_stream u_lane5 (.clk(clk), .weight_in(ws_lane5), .w_load(w_load), .actin_rdata(actin_rdata), .acc_clear(acc_clear), .acc_en(acc_en), .bias_in(bias_rdata), .biased(lane_biased[5]));
    mac_lane_stream u_lane6 (.clk(clk), .weight_in(ws_lane6), .w_load(w_load), .actin_rdata(actin_rdata), .acc_clear(acc_clear), .acc_en(acc_en), .bias_in(bias_rdata), .biased(lane_biased[6]));
    mac_lane_stream u_lane7 (.clk(clk), .weight_in(ws_lane7), .w_load(w_load), .actin_rdata(actin_rdata), .acc_clear(acc_clear), .acc_en(acc_en), .bias_in(bias_rdata), .biased(lane_biased[7]));

    // Mismo motivo de siempre: selector explicito en vez de array[variable]
    // (bug de la etapa 1 del proyecto).
    reg signed [31:0] lane_biased_sel;
    always @(*) begin
        case (lane_ctr)
            3'd0: lane_biased_sel = lane_biased[0];
            3'd1: lane_biased_sel = lane_biased[1];
            3'd2: lane_biased_sel = lane_biased[2];
            3'd3: lane_biased_sel = lane_biased[3];
            3'd4: lane_biased_sel = lane_biased[4];
            3'd5: lane_biased_sel = lane_biased[5];
            3'd6: lane_biased_sel = lane_biased[6];
            default: lane_biased_sel = lane_biased[7];
        endcase
    end

    // ================= Lectura de diagnostico =================
    reg [2:0]  dbg_sel_r;
    reg [10:0] dbg_addr_r;
    always @(posedge clk) begin
        if (dbg_rd_en) begin
            dbg_sel_r  <= dbg_rd_sel;
            dbg_addr_r <= dbg_rd_addr;
        end
    end

    reg signed [7:0] dbg_data_pre;
    always @(posedge clk) begin
        case (dbg_sel_r)
            3'd0: dbg_data_pre <= actbuf_a[dbg_addr_r[7:0]];
            3'd1: dbg_data_pre <= actbuf_b[dbg_addr_r[7:0]];
            3'd4: dbg_data_pre <= input_mem[dbg_addr_r];
            3'd5: dbg_data_pre <= out_mem[dbg_addr_r[7:0]];
            3'd6: dbg_data_pre <= bias_mem[dbg_addr_r][7:0]; // bias efectivo de 32 bits, solo los 8 bajos para debug
            default: dbg_data_pre <= start_count;
        endcase
    end

    always @(posedge clk) dbg_rd_data <= dbg_data_pre;

endmodule
