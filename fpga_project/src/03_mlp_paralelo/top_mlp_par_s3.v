// top_mlp_par_s3.v -- top-level de "Situacion 3" (demo de arquitecturas,
// ver README): red 130->192->96->40->5 (~47.4K pesos, ~87-88% de BRAM),
// el modelo mas grande que entra con margen de seguridad. Identico a
// top_mlp_par_s2.v salvo que instancia mlp_engine_par_s3 (mismos anchos
// ensanchados: pesos 13 bits, bias/debug 9 bits).
module top_mlp_par_s3 (
    input  wire       clk,
    input  wire       sclk,
    input  wire       cs_n,
    input  wire       mosi,
    output wire       miso,
    output wire [5:0] led
);

    localparam [7:0] CMD_NOP      = 8'h00;
    localparam [7:0] CMD_LOAD_W   = 8'h03;
    localparam [7:0] CMD_LOAD_B   = 8'h02;
    localparam [7:0] CMD_START    = 8'h04;
    localparam [7:0] CMD_DBG_RD   = 8'h06;
    localparam [7:0] CMD_SET_WTGT = 8'h07;
    localparam [7:0] CMD_SET_ITGT = 8'h0A;
    localparam [7:0] CMD_IBURST5  = 8'h0B;

    wire clk_sys;
    wire pll_lock;
    pll_par_54 u_pll (
        .clk_in (clk),
        .clk_out(clk_sys),
        .lock   (pll_lock)
    );

    wire [47:0] rx_data;
    wire        frame_done;
    reg  [47:0] tx_snapshot = 48'd0;

    spi_slave u_spi (
        .clk       (clk_sys),
        .sclk      (sclk),
        .cs_n      (cs_n),
        .mosi      (mosi),
        .miso      (miso),
        .tx_data   (tx_snapshot),
        .rx_data   (rx_data),
        .frame_done(frame_done)
    );

    reg frame_done_d1 = 1'b0;
    always @(posedge clk_sys) frame_done_d1 <= frame_done;

    wire [7:0]  cmd        = rx_data[47:40];
    wire [15:0] addr_field = rx_data[39:24];
    wire [15:0] data_field = rx_data[23:8];

    wire              busy;
    wire signed [7:0] out0, out1, out2, out3, out4;
    wire signed [7:0] dbg_rd_data;

    reg [12:0]       w_ptr;
    reg [2:0]        w_lane;

    reg [2:0]        ib_step = 3'd0;
    reg signed [7:0] ib_bytes [0:4];
    reg [7:0]        i_ptr;

    reg        imac_en;
    reg [7:0]  imac_addr;
    reg signed [7:0] imac_data;

    reg load_b_en, load_w_en, start_pulse, dbg_rd_en, set_wtgt, set_itgt;
    wire cmd_ok = frame_done_d1 && !busy && (ib_step == 3'd0);
    always @(*) begin
        load_b_en   = 1'b0;
        load_w_en   = 1'b0;
        start_pulse = 1'b0;
        dbg_rd_en   = 1'b0;
        set_wtgt    = 1'b0;
        set_itgt    = 1'b0;
        if (cmd_ok) begin
            case (cmd)
                CMD_LOAD_B:   load_b_en   = 1'b1;
                CMD_LOAD_W:   load_w_en   = 1'b1;
                CMD_START:    start_pulse = 1'b1;
                CMD_DBG_RD:   dbg_rd_en   = 1'b1;
                CMD_SET_WTGT: set_wtgt    = 1'b1;
                CMD_SET_ITGT: set_itgt    = 1'b1;
                default: ;
            endcase
        end
    end

    always @(posedge clk_sys) begin
        if (set_wtgt) begin
            w_lane <= addr_field[15:13];
            w_ptr  <= addr_field[12:0];
        end
    end

    always @(posedge clk_sys) begin
        imac_en <= 1'b0;
        if (set_itgt) begin
            i_ptr <= addr_field[7:0];
        end else if (ib_step == 3'd0) begin
            if (cmd_ok && cmd == CMD_IBURST5) begin
                ib_bytes[0] <= rx_data[39:32];
                ib_bytes[1] <= rx_data[31:24];
                ib_bytes[2] <= rx_data[23:16];
                ib_bytes[3] <= rx_data[15:8];
                ib_bytes[4] <= rx_data[7:0];
                ib_step     <= 3'd1;
            end
        end else begin
            imac_en   <= 1'b1;
            imac_addr <= i_ptr + (ib_step - 3'd1);
            imac_data <= ib_bytes[ib_step - 3'd1];
            if (ib_step == 3'd5) begin
                i_ptr   <= i_ptr + 8'd5;
                ib_step <= 3'd0;
            end else begin
                ib_step <= ib_step + 3'd1;
            end
        end
    end

    reg dbg_mode;
    always @(posedge clk_sys) begin
        if (frame_done_d1) begin
            if (cmd == CMD_DBG_RD) dbg_mode <= 1'b1;
            else if (cmd == CMD_START) dbg_mode <= 1'b0;
        end
    end

    mlp_engine_par_s3 u_mlp (
        .clk              (clk_sys),
        .load_weight_en   (load_w_en),
        .load_weight_lane (addr_field[15:13]),
        .load_weight_addr (addr_field[12:0]),
        .load_weight_data (data_field[7:0]),
        .load_bias_en     (load_b_en),
        .load_bias_addr   (addr_field[8:0]),
        .load_bias_data   (data_field[7:0]),
        .load_input_en    (imac_en),
        .load_input_addr  (imac_addr),
        .load_input_data  (imac_data),
        .start            (start_pulse),
        .busy             (busy),
        .out0(out0), .out1(out1), .out2(out2), .out3(out3), .out4(out4),
        .dbg_rd_en   (dbg_rd_en),
        .dbg_rd_sel  (addr_field[11:9]),
        .dbg_rd_addr (addr_field[8:0]),
        .dbg_rd_data (dbg_rd_data),
        .dbg_w_lane  (w_lane),
        .dbg_w_addr  (w_ptr)
    );

    always @(posedge clk_sys) begin
        tx_snapshot <= {6'd0, pll_lock, busy, out0, out1, out2, out3, (dbg_mode ? dbg_rd_data : out4)};
    end

    assign led = ~{busy, out0[4:0]};

endmodule
