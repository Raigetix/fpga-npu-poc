// Diagnostico temporal: aisla si la memoria de pesos (mismo tamano que en
// mlp_engine.v: 27040 x int8) guarda y devuelve el valor correcto en la
// direccion correcta, sin pasar por toda la FSM de inferencia.
//   CMD_LOAD (0x01): A=direccion, B_lo=valor -> escribe weight_mem[A]=B_lo
//   CMD_READ (0x05): A=direccion             -> en la SIGUIENTE lectura,
//                     el byte bajo de la respuesta es weight_mem[A]
module top_debug_wr (
    input  wire       clk,
    input  wire       sclk,
    input  wire       cs_n,
    input  wire       mosi,
    output wire       miso,
    output wire [5:0] led
);

    localparam [7:0] CMD_LOAD = 8'h01;
    localparam [7:0] CMD_READ = 8'h05;

    wire [47:0] rx_data;
    wire        frame_done;
    reg  [47:0] tx_snapshot = 48'd0;

    spi_slave u_spi (
        .clk(clk), .sclk(sclk), .cs_n(cs_n), .mosi(mosi), .miso(miso),
        .tx_data(tx_snapshot), .rx_data(rx_data), .frame_done(frame_done)
    );

    reg frame_done_d1 = 1'b0;
    always @(posedge clk) frame_done_d1 <= frame_done;

    wire [7:0]        cmd   = rx_data[47:40];
    wire [14:0]       addr  = rx_data[38:24];
    wire signed [7:0] wdata = rx_data[15:8];

    reg signed [7:0] weight_mem [0:27039];

    wire load_en = frame_done_d1 && (cmd == CMD_LOAD);
    always @(posedge clk) if (load_en) weight_mem[addr] <= wdata;

    // FSM de lectura: separa "capturar la direccion pedida" de "esperar a
    // que la memoria la refleje" de "capturar el dato", cada uno en su
    // propio ciclo, para no repetir el bug de muestrear un ciclo antes de
    // tiempo.
    localparam S_IDLE=2'd0, S_ARM=2'd1, S_WAIT=2'd2, S_CAPTURE=2'd3;
    reg [1:0] rstate = S_IDLE;
    reg [14:0] raddr;
    reg signed [7:0] rdata;
    reg signed [7:0] rdata_captured = 8'sd0;

    always @(posedge clk) rdata <= weight_mem[raddr];

    always @(posedge clk) begin
        case (rstate)
            S_IDLE: if (frame_done_d1 && cmd == CMD_READ) begin
                raddr  <= addr;
                rstate <= S_ARM;
            end
            S_ARM:     rstate <= S_WAIT;
            S_WAIT:    rstate <= S_CAPTURE;
            S_CAPTURE: begin
                rdata_captured <= rdata;
                rstate         <= S_IDLE;
            end
            default: rstate <= S_IDLE;
        endcase
    end

    always @(posedge clk) tx_snapshot <= {39'd0, rstate == S_IDLE, rdata_captured};

    assign led = ~rdata_captured[5:0];

endmodule
