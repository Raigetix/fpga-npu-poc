// mlp_engine_par_s3.v -- variante de mlp_engine_par.v para "Situacion 3"
// (demo de las 3 arquitecturas, ver README): red 130->176->88->36->5
// (~41.7K pesos). El primer intento (130->192->96->40->5, ~47.4K pesos)
// resulto NO ENTRAR en los 46 bloques de BRAM disponibles -- la eficiencia
// real de empaquetado de Gowin para este tamano de banco (~7900 bits por
// bloque) fue peor que lo estimado a partir del modelo original (~9800
// bits/bloque). Este tamano se ajusto verificando el reporte de sintesis
// real, no a partir de una estimacion. Mismo patron de FSM/pipeline que
// mlp_engine_par.v/_s2.v -- solo cambia la tabla de capas y el tamano de
// las memorias.
//
// MAPA DE MEMORIA DE PESOS POR CARRIL:
//   Capa 0 (130->176, 22 olas): direcciones locales    0..2859  (22*130)
//   Capa 1 (176->88,  11 olas): direcciones locales 2860..4795  (11*176)
//   Capa 2 ( 88->36,   5 olas): direcciones locales 4796..5235  ( 5*88)
//   Capa 3 ( 36-> 5,   1 ola ): direcciones locales 5236..5271  ( 1*36)
// MAPA DE BIAS: direccion plana 0..304 (176+88+36+5).
module mlp_engine_par_s3 (
    input  wire clk,

    input  wire        load_weight_en,
    input  wire [2:0]  load_weight_lane,
    input  wire [12:0] load_weight_addr,
    input  wire signed [7:0] load_weight_data,

    input  wire        load_bias_en,
    input  wire [8:0]  load_bias_addr,
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
    input  wire [2:0]  dbg_rd_sel,
    input  wire [8:0]  dbg_rd_addr,
    output reg signed [7:0] dbg_rd_data,

    input  wire [2:0]  dbg_w_lane,
    input  wire [12:0] dbg_w_addr
);

    localparam integer NLANES     = 8;
    localparam integer BANK_ABITS = 13;

    reg signed [7:0] bias_mem  [0:304];
    reg signed [7:0] input_mem [0:129];
    reg signed [7:0] h1_mem    [0:175];
    reg signed [7:0] h2_mem    [0:87];
    reg signed [7:0] h3_mem    [0:35];

    always @(posedge clk) if (load_bias_en)  bias_mem[load_bias_addr]   <= load_bias_data;
    always @(posedge clk) if (load_input_en) input_mem[load_input_addr] <= load_input_data;

    reg [1:0]  layer_idx;
    reg [7:0]  in_count, out_count;
    reg [4:0]  num_waves;
    reg [BANK_ABITS-1:0] layer_weight_base;
    reg [8:0]  layer_bias_base;

    always @(*) begin
        case (layer_idx)
            2'd0: begin in_count=8'd130; out_count=8'd176; num_waves=5'd22; layer_weight_base=13'd0;    layer_bias_base=9'd0;   end
            2'd1: begin in_count=8'd176; out_count=8'd88;  num_waves=5'd11; layer_weight_base=13'd2860; layer_bias_base=9'd176; end
            2'd2: begin in_count=8'd88;  out_count=8'd36;  num_waves=5'd5;  layer_weight_base=13'd4796; layer_bias_base=9'd264; end
            default: begin in_count=8'd36; out_count=8'd5; num_waves=5'd1;  layer_weight_base=13'd5236; layer_bias_base=9'd300; end
        endcase
    end

    reg [4:0] wave_idx;
    reg [7:0] input_idx;
    reg [BANK_ABITS-1:0] weight_addr;
    reg [8:0] bias_addr;
    reg [2:0] lane_ctr;

    reg signed [7:0] actin_rdata;
    always @(posedge clk) begin
        case (layer_idx)
            2'd0: actin_rdata <= input_mem[input_idx];
            2'd1: actin_rdata <= h1_mem[input_idx];
            2'd2: actin_rdata <= h2_mem[input_idx];
            default: actin_rdata <= h3_mem[input_idx];
        endcase
    end

    reg signed [7:0] bias_rdata;
    always @(posedge clk) bias_rdata <= bias_mem[bias_addr];

    localparam [3:0]
        S_IDLE       = 4'd0,
        S_LAYER_INIT = 4'd1,
        S_WAVE_INIT  = 4'd2,
        S_SET_ADDR   = 4'd3,
        S_WAIT_MEM   = 4'd4,
        S_WAIT_MEM2  = 4'd5,
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

    reg [8:0] start_count = 9'd0;

    wire [7:0] neuron_global = {wave_idx, 3'b000} + {5'd0, lane_ctr};

    always @(posedge clk) begin
        case (state)
            S_IDLE: begin
                busy <= 1'b0;
                if (start) begin
                    busy        <= 1'b1;
                    layer_idx   <= 2'd0;
                    start_count <= start_count + 9'd1;
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
                weight_addr <= weight_addr + 1'b1;
                if (input_idx == in_count - 8'd1) begin
                    lane_ctr  <= 3'd0;
                    bias_addr <= layer_bias_base + {wave_idx, 3'd0};
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
                    bias_addr <= layer_bias_base + {wave_idx, 3'd0} + {5'd0, lane_ctr} + 9'd1;
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

    wire [BANK_ABITS-1:0] eff_weight_addr = busy ? weight_addr : dbg_w_addr;

    genvar gi;
    generate
        for (gi = 0; gi < NLANES; gi = gi + 1) begin : gen_lane
            mac_lane #(
                .BANK_ABITS(BANK_ABITS),
                .BANK_DEPTH(5272)
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

    reg [2:0] dbg_sel_r;
    reg [8:0] dbg_addr_r;
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
            default: dbg_data_pre <= start_count[7:0];
        endcase
    end

    always @(posedge clk) dbg_rd_data <= dbg_data_pre;

endmodule
