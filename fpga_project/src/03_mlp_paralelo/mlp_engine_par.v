// mlp_engine_par.v -- version paralela de mlp_engine.v: 8 carriles MAC
// trabajando a la vez (8 bancos de pesos independientes = 8 lecturas por
// ciclo en vez de 1), para que el computo baje ~8x. La entrada se sigue
// leyendo una vez por ciclo y se difunde (broadcast) a los 8 carriles, ya
// que todos procesan la MISMA entrada en un instante dado, cada uno con su
// propio peso.
//
// Las neuronas de una capa se procesan en "olas" de 8: la ola w cubre las
// neuronas [8w .. 8w+7]. El carril l calcula la neurona (8w+l).
//
// MAPA DE MEMORIA DE PESOS POR CARRIL (direccion LOCAL a cada banco; los 8
// bancos usan la MISMA numeracion, cada uno con sus propios valores):
//   Capa 1 (130->128, 16 olas): direcciones locales    0 .. 2079  (16*130)
//   Capa 2 (128->64,   8 olas): direcciones locales 2080 .. 3103  (8*128)
//   Capa 3 ( 64->32,   4 olas): direcciones locales 3104 .. 3359  (4*64)
//   Capa 4 ( 32-> 5,   1 ola ): direcciones locales 3360 .. 3391  (1*32)
//   Dentro de cada capa: para ola w (0..olas-1), entrada i (0..in-1):
//     direccion_local = base_capa + w*in_count + i
//   (Los carriles 5,6,7 en la unica ola de la capa 4 no corresponden a
//   ninguna neurona real -- sus pesos existen pero nunca se leen ni escriben
//   sus resultados.)
//
// MAPA DE BIAS: identico a mlp_engine.v (direccion plana 0..228, por
// neurona global = ola*8+carril), un solo banco compartido.
module mlp_engine_par (
    input  wire clk,

    // ---- Carga de UN carril del banco de pesos ----
    input  wire        load_weight_en,
    input  wire [2:0]  load_weight_lane,
    input  wire [11:0] load_weight_addr,
    input  wire signed [7:0] load_weight_data,

    input  wire        load_bias_en,
    input  wire [7:0]  load_bias_addr,
    input  wire signed [7:0] load_bias_data,

    input  wire        load_input_en,
    input  wire [7:0]  load_input_addr,
    input  wire signed [7:0] load_input_data,

    input  wire        start,
    output reg          busy,

    output reg signed [7:0] out0,
    output reg signed [7:0] out1,
    output reg signed [7:0] out2,
    output reg signed [7:0] out3,
    output reg signed [7:0] out4,

    input  wire        dbg_rd_en,
    input  wire [2:0]  dbg_rd_sel,   // 0=h1 1=h2 2=h3 3=start_count 4=input_mem 5=weight_bank[dbg_w_lane][dbg_w_addr] 6=bias_mem
    input  wire [7:0]  dbg_rd_addr,
    output reg signed [7:0] dbg_rd_data,

    // Objetivo del debug de pesos: se reutiliza el mismo w_lane/w_ptr que ya
    // fija CMD_SET_WTGT del lado del top-level (no hace falta un comando
    // nuevo). Solo importa cuando !busy (en compute manda weight_addr).
    input  wire [2:0]  dbg_w_lane,
    input  wire [11:0] dbg_w_addr
);

    localparam integer NLANES     = 8;
    // 3392 direcciones utiles por carril (0..3391). mac_lane.v las parte
    // internamente en 8 arrays chicos -- ver comentario alli.
    localparam integer BANK_ABITS = 12;

    // ================= Memorias compartidas (no bancadas) =================
    reg signed [7:0] bias_mem  [0:228];
    reg signed [7:0] input_mem [0:129];
    reg signed [7:0] h1_mem    [0:127];
    reg signed [7:0] h2_mem    [0:63];
    reg signed [7:0] h3_mem    [0:31];

    always @(posedge clk) if (load_bias_en)  bias_mem[load_bias_addr]   <= load_bias_data;
    always @(posedge clk) if (load_input_en) input_mem[load_input_addr] <= load_input_data;

    // ================= Configuracion por capa =================
    reg [1:0]  layer_idx;
    reg [7:0]  in_count, out_count;
    reg [4:0]  num_waves;
    reg [BANK_ABITS-1:0] layer_weight_base;
    reg [7:0]  layer_bias_base;

    always @(*) begin
        case (layer_idx)
            2'd0: begin in_count=8'd130; out_count=8'd128; num_waves=5'd16; layer_weight_base=12'd0;    layer_bias_base=8'd0;   end
            2'd1: begin in_count=8'd128; out_count=8'd64;  num_waves=5'd8;  layer_weight_base=12'd2080; layer_bias_base=8'd128; end
            2'd2: begin in_count=8'd64;  out_count=8'd32;  num_waves=5'd4;  layer_weight_base=12'd3104; layer_bias_base=8'd192; end
            default: begin in_count=8'd32; out_count=8'd5; num_waves=5'd1;  layer_weight_base=12'd3360; layer_bias_base=8'd224; end
        endcase
    end

    // ================= Contadores de recorrido =================
    reg [4:0] wave_idx;
    reg [7:0] input_idx;
    reg [BANK_ABITS-1:0] weight_addr; // compartido: los 8 carriles lo aplican cada uno a SU banco
    reg [7:0] bias_addr;
    reg [2:0] lane_ctr;

    // ================= Entrada de la capa actual (broadcast, 1 lectura/ciclo) ====
    reg signed [7:0] actin_rdata;
    always @(posedge clk) begin
        case (layer_idx)
            2'd0: actin_rdata <= input_mem[input_idx];
            2'd1: actin_rdata <= h1_mem[input_idx];
            2'd2: actin_rdata <= h2_mem[input_idx];
            default: actin_rdata <= h3_mem[input_idx];
        endcase
    end

    // ================= Bias de la neurona actual (broadcast, 1 lectura/ciclo) ====
    reg signed [7:0] bias_rdata;
    always @(posedge clk) bias_rdata <= bias_mem[bias_addr];

    // ================= FSM =================
    localparam [3:0]
        S_IDLE       = 4'd0,
        S_LAYER_INIT = 4'd1,
        S_WAVE_INIT  = 4'd2,
        S_SET_ADDR   = 4'd3,
        S_WAIT_MEM   = 4'd4,
        S_WAIT_MEM2  = 4'd5, // ciclo extra: mac_lane.v ahora tiene 2 registros
                              // en cascada en la lectura de pesos (ver alli)
        S_MUL        = 4'd6,
        S_ACC        = 4'd7,
        S_BIAS_ADDR  = 4'd8,
        S_BIAS_WAIT  = 4'd9,
        S_BIAS_WRITE = 4'd10,
        S_WAVE_DONE  = 4'd11,
        S_LAYER_DONE = 4'd12,
        S_ALL_DONE   = 4'd13;

    reg [3:0] state;
    wire acc_clear = (state == S_WAVE_INIT);
    wire acc_en    = (state == S_ACC);

    reg [7:0] start_count = 8'd0;

    wire [7:0] neuron_global = {wave_idx, 3'b000} + {5'd0, lane_ctr}; // wave_idx*8 + lane_ctr (concatenacion, no multiplicacion)

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
                wave_idx    <= 5'd0;
                weight_addr <= layer_weight_base;
                state       <= S_WAVE_INIT;
            end

            S_WAVE_INIT: begin
                input_idx <= 8'd0;
                state     <= S_SET_ADDR;
            end

            S_SET_ADDR:  state <= S_WAIT_MEM;
            S_WAIT_MEM:  state <= S_WAIT_MEM2;
            S_WAIT_MEM2: state <= S_MUL;
            S_MUL:       state <= S_ACC;

            S_ACC: begin
                weight_addr <= weight_addr + 1'b1; // continuo, igual razon que en mlp_engine.v
                if (input_idx == in_count - 8'd1) begin
                    lane_ctr  <= 3'd0;
                    bias_addr <= layer_bias_base + {wave_idx, 3'd0}; // wave_idx*8 + 0
                    state     <= S_BIAS_ADDR;
                end else begin
                    input_idx <= input_idx + 8'd1;
                    state     <= S_SET_ADDR;
                end
            end

            S_BIAS_ADDR: state <= S_BIAS_WAIT;
            S_BIAS_WAIT: state <= S_BIAS_WRITE;

            S_BIAS_WRITE: begin
                if (neuron_global < out_count) begin
                    case (layer_idx)
                        2'd0: h1_mem[neuron_global] <= lane_activated_sel;
                        2'd1: h2_mem[neuron_global] <= lane_activated_sel;
                        2'd2: h3_mem[neuron_global] <= lane_activated_sel;
                        default: begin
                            case (neuron_global)
                                8'd0: out0 <= lane_activated_sel;
                                8'd1: out1 <= lane_activated_sel;
                                8'd2: out2 <= lane_activated_sel;
                                8'd3: out3 <= lane_activated_sel;
                                default: out4 <= lane_activated_sel;
                            endcase
                        end
                    endcase
                end

                if (lane_ctr == NLANES - 1) begin
                    state <= S_WAVE_DONE;
                end else begin
                    lane_ctr  <= lane_ctr + 1'b1;
                    bias_addr <= layer_bias_base + {wave_idx, 3'd0} + {5'd0, lane_ctr} + 8'd1;
                    state     <= S_BIAS_ADDR;
                end
            end

            S_WAVE_DONE: begin
                if (wave_idx == num_waves - 1'b1) begin
                    state <= S_LAYER_DONE;
                end else begin
                    wave_idx <= wave_idx + 1'b1;
                    state    <= S_WAVE_INIT;
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

    // ================= 8 carriles MAC en paralelo =================
    // Decodificador de carril de destino, explicito (no shift variable):
    // el carril 0 (shift por 0) mostraba MUCHA mas corrupcion de carga que
    // el resto, de forma persistente y no convergente con reintentos, algo
    // que sobrevivio a 2 rediseños distintos de weight_bank (particionado
    // en chunks, pipeline de lectura de 2 etapas) -- apunta a que el
    // decodificador en si (unica pieza que sigue siendo distinta entre la
    // carga de pesos, que falla, y la de bias, que nunca fallo) tiene un
    // problema especifico con el caso "sin desplazamiento". Un case
    // explicito es logicamente identico pero evita el shifter variable.
    reg [NLANES-1:0] lane_load_en;
    always @(*) begin
        lane_load_en = 8'b0;
        if (load_weight_en) begin
            case (load_weight_lane)
                3'd0: lane_load_en = 8'b00000001;
                3'd1: lane_load_en = 8'b00000010;
                3'd2: lane_load_en = 8'b00000100;
                3'd3: lane_load_en = 8'b00001000;
                3'd4: lane_load_en = 8'b00010000;
                3'd5: lane_load_en = 8'b00100000;
                3'd6: lane_load_en = 8'b01000000;
                default: lane_load_en = 8'b10000000;
            endcase
        end
    end
    wire signed [7:0] lane_activated [0:NLANES-1];
    wire signed [7:0] lane_weight_dbg [0:NLANES-1];

    // Mismo motivo que el decodificador de arriba: "array[variable]" leido
    // directo (sin pasar por un case explicito) fue justo el bug que costo
    // encontrar en top.v en la etapa 1 de este proyecto (registro conectado
    // a un puerto indexado variable -> resultados mal en este toolchain).
    // lane_activated[lane_ctr] y lane_weight_dbg[dbg_w_lane] tenian
    // exactamente ese patron y nunca se habian tocado en los intentos
    // anteriores (todos se enfocaron en weight_bank). Selectores explicitos:
    reg signed [7:0] lane_activated_sel;
    always @(*) begin
        case (lane_ctr)
            3'd0: lane_activated_sel = lane_activated[0];
            3'd1: lane_activated_sel = lane_activated[1];
            3'd2: lane_activated_sel = lane_activated[2];
            3'd3: lane_activated_sel = lane_activated[3];
            3'd4: lane_activated_sel = lane_activated[4];
            3'd5: lane_activated_sel = lane_activated[5];
            3'd6: lane_activated_sel = lane_activated[6];
            default: lane_activated_sel = lane_activated[7];
        endcase
    end

    reg signed [7:0] lane_weight_dbg_sel;
    always @(*) begin
        case (dbg_w_lane)
            3'd0: lane_weight_dbg_sel = lane_weight_dbg[0];
            3'd1: lane_weight_dbg_sel = lane_weight_dbg[1];
            3'd2: lane_weight_dbg_sel = lane_weight_dbg[2];
            3'd3: lane_weight_dbg_sel = lane_weight_dbg[3];
            3'd4: lane_weight_dbg_sel = lane_weight_dbg[4];
            3'd5: lane_weight_dbg_sel = lane_weight_dbg[5];
            3'd6: lane_weight_dbg_sel = lane_weight_dbg[6];
            default: lane_weight_dbg_sel = lane_weight_dbg[7];
        endcase
    end

    // Mientras el motor esta ocupado, la direccion de lectura de pesos la
    // maneja la FSM (weight_addr, avanza durante el computo). Cuando esta
    // libre, se usa como direccion de debug (dbg_w_addr) para poder leer
    // weight_bank[*][dbg_w_addr] de los 8 carriles "gratis" en paralelo, y
    // despues elegir el carril deseado con dbg_w_lane.
    wire [BANK_ABITS-1:0] eff_weight_addr = busy ? weight_addr : dbg_w_addr;

    genvar gi;
    generate
        for (gi = 0; gi < NLANES; gi = gi + 1) begin : gen_lane
            mac_lane #(
                .BANK_ABITS(BANK_ABITS)
            ) u_lane (
                .clk        (clk),
                .load_en    (lane_load_en[gi]),
                .load_addr  (load_weight_addr),
                .load_data  (load_weight_data),
                .rd_addr    (eff_weight_addr),
                .actin_rdata(actin_rdata),
                .acc_clear  (acc_clear),
                .acc_en     (acc_en),
                .bias_rdata (bias_rdata),
                .activated  (lane_activated[gi]),
                .weight_dbg (lane_weight_dbg[gi])
            );
        end
    endgenerate

    // ================= Lectura de diagnostico h1/h2/h3/start_count/input_mem ====
    reg [2:0] dbg_sel_r;
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
            3'd0: dbg_data_pre <= h1_mem[dbg_addr_r];
            3'd1: dbg_data_pre <= h2_mem[dbg_addr_r];
            3'd2: dbg_data_pre <= h3_mem[dbg_addr_r];
            3'd4: dbg_data_pre <= input_mem[dbg_addr_r];
            3'd5: dbg_data_pre <= lane_weight_dbg_sel;
            3'd6: dbg_data_pre <= bias_mem[dbg_addr_r];
            default: dbg_data_pre <= start_count;
        endcase
    end

    always @(posedge clk) dbg_rd_data <= dbg_data_pre;

endmodule
