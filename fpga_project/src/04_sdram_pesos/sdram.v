// sdram.v -- controlador de SDRAM para la SDRAM embebida del Tang Nano 20K
// (64Mbit, 32 bits de ancho, 2K filas x 256 columnas x 4 bancos). Adaptado
// de nand2mario/sdram-tang-nano-20k (Apache-2.0), usado tal cual con
// comentarios traducidos -- la maquina de estados en si no se toco, ya
// esta probada por ese proyecto.
//
// Es un controlador POR BYTE, sin rafaga nativa, de baja latencia:
//   - La lectura tarda 4 ciclos en tener el dato listo.
//   - Cada operacion (lectura o escritura) ocupa el controlador 5 ciclos
//     en total (no hay solapamiento entre operaciones).
//   - Todas las operaciones usan auto-precharge, asi que quien llama no
//     tiene que lidiar con activar/precargar filas a mano.
//   - Necesita refresco periodico (la DRAM pierde el dato si no) via el
//     pulso 'refresh': al menos 4096 refrescos cada 64ms, o sea una vez
//     cada ~15us como maximo.
//   - Necesita un reloj adicional desfasado 180 grados (clk_sdram) para
//     manejar el pin fisico SDRAM_CLK -- se genera con la salida CLKOUTP
//     de la PLL (ver pll_sdram.v).
module sdram
#(
    parameter         FREQ = 54_000_000,
    parameter         DATA_WIDTH = 32,
    parameter         ROW_WIDTH = 11,  // 2K filas
    parameter         COL_WIDTH = 8,   // 256 palabras por fila (1KB)
    parameter         BANK_WIDTH = 2,  // 4 bancos

    // Tiempos validos hasta 66.7MHz de reloj (periodo minimo 15ns).
    parameter [3:0]   CAS  = 4'd2,     // 2/3 ciclos, se fija en el registro de modo
    parameter [3:0]   T_WR = 4'd2,     // 2 ciclos, recuperacion de escritura
    parameter [3:0]   T_MRD= 4'd2,     // 2 ciclos, fijar registro de modo
    parameter [3:0]   T_RP = 4'd1,     // 15ns, precarga a activacion
    parameter [3:0]   T_RCD= 4'd1,     // 15ns, activacion a lectura/escritura
    parameter [3:0]   T_RC = 4'd4      // 60ns, refresco/activacion a refresco/activacion
)
(
    // ---- Lado SDRAM ----
    inout [DATA_WIDTH-1:0]      SDRAM_DQ,
    output reg [ROW_WIDTH-1:0]  SDRAM_A,
    output reg [BANK_WIDTH-1:0] SDRAM_BA,
    output            SDRAM_nCS,    // no es estrictamente necesario, siempre 0
    output reg        SDRAM_nWE,
    output reg        SDRAM_nRAS,
    output reg        SDRAM_nCAS,
    output            SDRAM_CLK,
    output            SDRAM_CKE,    // no es estrictamente necesario, siempre 1
    output reg  [3:0] SDRAM_DQM,

    // ---- Lado logica ----
    input             clk,
    input             clk_sdram,    // desfasado de clk (180 grados)
    input             resetn,
    input             rd,           // comando: leer
    input             wr,           // comando: escribir
    input             refresh,      // comando: auto-refresco
    // Comando: leer DOS palabras de 32 bits consecutivas (columna, columna+1
    // de la MISMA fila) en una sola operacion -- activa una vez, emite dos
    // comandos READ seguidos (auto-precharge solo en el segundo, para que
    // el chip precargue solo sin necesitar un comando ni espera aparte), en
    // vez de dos operaciones completas de activar-leer-precargar. OJO: si
    // 'addr' cae en la ULTIMA columna de la fila (256 columnas por fila),
    // la segunda palabra NO se puede leer asi (se saldria de la fila) --
    // quien llama tiene que evitar ese caso (ver weight_stream.v).
    input             rd_burst2,
    input      [22:0] addr,         // direccion de byte, capturada en el pulso rd/wr/rd_burst2
    input       [7:0] din,          // dato de entrada, capturado en el pulso wr
    output      [7:0] dout,         // dato de salida, disponible 4 ciclos despues de rd
                                     // queda guardado hasta la proxima lectura
    output [DATA_WIDTH-1:0] dout32, // salida de 32 bits completa (rd)
    output [DATA_WIDTH-1:0] dout32_a, // primera palabra de rd_burst2 (columna base)
    output [DATA_WIDTH-1:0] dout32_b, // segunda palabra de rd_burst2 (columna base+1)
    output reg        data_ready,   // disponible 6 ciclos despues de que wr se activa
    output reg        busy          // 0: listo para el proximo comando
);

reg dq_oen;         // 0 = salida activa
reg [DATA_WIDTH-1:0] dq_out;
assign SDRAM_DQ = dq_oen ? 32'bzzzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : dq_out;

// EXPERIMENTO -- las fallas con BURST_LEN=2 caian SOLO en los bytes 2-3
// (mitad alta de los 32 bits), nunca en 0-1. La SDRAM embebida son en
// realidad DOS memorias de 16 bits en el mismo encapsulado (confirmado en
// el testbench oficial de Gowin, SDRC_EMB/GW2AR: nDRAM=2, DQ_BITS=16) --
// si hay un desfasaje real de unos pocos ns entre esas dos mitades, un
// retardo de un ciclo ENTERO (lo que se probo antes) apunta a la ventana
// equivocada (la rafaga ya termino, el bus ya esta flotando). La
// herramienta correcta para un desfasaje de sub-ciclo es IODELAY (retardo
// programable por pin del IOB de Gowin, pasos de 0.025ns, 0-127) -- se
// prueba agregandolo SOLO a la mitad alta, con la cantidad de pasos como
// parametro para barrer empiricamente.
parameter [6:0] DQ_HI_DELAY_TAPS = 7'd50;   // EXPERIMENTO -- barrer 0..127
// EXPERIMENTO -- la falla que quedaba en rd_burst2 (byte0, casi cualquier
// direccion, sin patron periodico -- no es la carrera de refresco ya
// conocida) sugiere que el byte0 tambien tiene un desfasaje propio, mas
// chico que el de bytes 2-3, que solo se nota con el timing mas ajustado
// de la rafaga real (el chip se comporta distinto a mitad de rafaga que
// terminando una lectura simple). Se agrega un segundo IODELAY, mas chico,
// para la mitad baja.
parameter [6:0] DQ_LO_DELAY_TAPS = 7'd50;   // EXPERIMENTO -- barrer 0..127
wire [DATA_WIDTH-1:0] dq_in_raw = SDRAM_DQ;
wire [DATA_WIDTH-1:0] dq_in;
genvar gi;
generate
    for (gi = 0; gi < 16; gi = gi + 1) begin : gen_iodelay_lo
        IODELAY #(.C_STATIC_DLY(DQ_LO_DELAY_TAPS)) u_iodelay_lo (
            .DI(dq_in_raw[gi]), .DO(dq_in[gi]),
            .SDTAP(1'b0), .SETN(1'b0), .VALUE(1'b0)
        );
    end
    for (gi = 16; gi < 32; gi = gi + 1) begin : gen_iodelay_hi
        IODELAY #(.C_STATIC_DLY(DQ_HI_DELAY_TAPS)) u_iodelay (
            .DI(dq_in_raw[gi]), .DO(dq_in[gi]),
            .SDTAP(1'b0), .SETN(1'b0), .VALUE(1'b0)
        );
    end
endgenerate

reg [1:0] off;          // desplazamiento de byte dentro de la palabra de 32 bits
reg [7:0] dout_buf;
wire [7:0] next_dout =  off == 0 ? dq_in[7:0] :
                        off == 1 ? dq_in[15:8] :
                        off == 2 ? dq_in[23:16] : dq_in[31:24];
assign dout = data_ready ? next_dout : dout_buf;
// OJO: dout32 TIENE que estar latcheada igual que dout. Antes era
// "assign dout32 = dq_in", o sea el bus fisico EN VIVO, que solo es valido
// durante el ciclo exacto en que el chip lo maneja. Quien consume la
// palabra (weight_stream.v) la lee DESPUES de que la operacion termina
// (cuando busy baja), momento en el que el chip ya solto el bus -- asi
// capturaba a veces lo que quedaba flotando en vez del dato. Se notaba
// como unos pocos pesos corruptos por inferencia, en proporcion a cuantos
// se leian: modelos chicos (300-450 pesos) daban 20/20, y los grandes
// (27K-73K pesos) fallaban casi siempre. La fase 2 nunca lo vio porque
// usaba dout (de un byte), que si estaba latcheada.
reg [DATA_WIDTH-1:0] dout32_buf;
assign dout32 = dout32_buf;
reg [DATA_WIDTH-1:0] dout32_a_buf, dout32_b_buf;
assign dout32_a = dout32_a_buf;
assign dout32_b = dout32_b_buf;
assign SDRAM_CLK = clk_sdram;
assign SDRAM_CKE = 1'b1;
assign SDRAM_nCS = 1'b0;

reg [2:0] state;
localparam INIT = 3'd0;
localparam CONFIG = 3'd1;
localparam IDLE = 3'd2;
localparam READ = 3'd3;
localparam WRITE = 3'd4;
localparam REFRESH = 3'd5;
localparam RDBURST2 = 3'd6;

// RAS# CAS# WE#
localparam CMD_SetModeReg=3'b000;
localparam CMD_AutoRefresh=3'b001;
localparam CMD_PreCharge=3'b010;
localparam CMD_BankActivate=3'b011;
localparam CMD_Write=3'b100;
localparam CMD_Read=3'b101;
localparam CMD_NOP=3'b111;

localparam [2:0] BURST_LEN = 3'b001;    // longitud de rafaga 2 real -- EXPERIMENTO
localparam BURST_MODE = 1'b0;           // secuencial
localparam [10:0] MODE_REG = {4'b0, CAS[2:0], BURST_MODE, BURST_LEN};

reg cfg_now;
reg [3:0] cycle;
reg [7:0] din_buf;
reg [22:0] addr_buf;

always @(posedge clk) begin
    cycle <= cycle == 4'd15 ? 4'd15 : cycle + 4'd1;
    {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_NOP;
    casex ({state, cycle})
        // espera 200us al prender
        {INIT, 4'bxxxx} : if (cfg_now) begin
            state <= CONFIG;
            cycle <= 0;
        end

        // secuencia de configuracion
        {CONFIG, 4'd0} : begin
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_PreCharge;
            SDRAM_A[10] <= 1'b1;
        end
        {CONFIG, T_RP} : begin
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AutoRefresh;
        end
        {CONFIG, T_RP+T_RC} : begin
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AutoRefresh;
        end
        {CONFIG, T_RP+T_RC+T_RC} : begin
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_SetModeReg;
            SDRAM_A[10:0] <= MODE_REG;
        end
        {CONFIG, T_RP+T_RC+T_RC+T_MRD} : begin
            state <= IDLE;
            busy <= 1'b0;
        end

        {IDLE, 4'bxxxx}: if (rd | wr) begin
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_BankActivate;
            SDRAM_BA <= addr[ROW_WIDTH+COL_WIDTH+BANK_WIDTH-1+2 : ROW_WIDTH+COL_WIDTH+2];
            SDRAM_A <= addr[ROW_WIDTH+COL_WIDTH-1+2:COL_WIDTH+2];
            state <= rd ? READ : WRITE;
            addr_buf <= addr;
            if (wr) din_buf <= din;
            cycle <= 4'd1;
            busy <= 1'b1;
        end else if (rd_burst2) begin
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_BankActivate;
            SDRAM_BA <= addr[ROW_WIDTH+COL_WIDTH+BANK_WIDTH-1+2 : ROW_WIDTH+COL_WIDTH+2];
            SDRAM_A <= addr[ROW_WIDTH+COL_WIDTH-1+2:COL_WIDTH+2];
            state <= RDBURST2;
            addr_buf <= addr;
            cycle <= 4'd1;
            busy <= 1'b1;
        end else if (refresh) begin
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AutoRefresh;
            state <= REFRESH;
            cycle <= 4'd1;
            busy <= 1'b1;
        end

        {READ, T_RCD}: begin
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_Read;
            SDRAM_A[10] <= 1'b1;
            SDRAM_A[9:0] <= {1'b0, addr_buf[COL_WIDTH-1+2:2]};
            SDRAM_DQM <= 4'b0;
            off <= addr_buf[1:0];
        end
        {READ, T_RCD+CAS}: begin
            data_ready <= 1'b1;
        end
        {READ, T_RCD+CAS+4'd1}: begin
            data_ready <= 1'b0;
            dout_buf   <= next_dout;
            dout32_buf <= dq_in;   // misma ventana valida que dout_buf
            busy <= 0;
            state <= IDLE;
        end

        // EXPERIMENTO -- rafaga NATIVA de verdad (BURST_LEN=2 real en el
        // registro de modo): UN solo comando READ con auto-precharge, el
        // chip streamea las dos palabras solo. Version anterior (chained,
        // dos comandos separados) asumia BURST_LEN=1 -- con BURST_LEN=2
        // real activo, CADA uno de esos dos comandos dispararia su PROPIA
        // rafaga de 2, y el segundo comando interrumpiria a mitad de
        // camino la rafaga del primero (nunca deja completar el auto-
        // precharge) -- eso explicaba el ~59% de bytes mal en hardware.
        {RDBURST2, T_RCD}: begin
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_Read;
            SDRAM_A[10]  <= 1'b1;
            SDRAM_A[9:0] <= {2'b0, addr_buf[COL_WIDTH-1+2:2]};
            SDRAM_DQM    <= 4'b0;
        end
        {RDBURST2, T_RCD+CAS+4'd1}: begin
            dout32_a_buf <= dq_in;   // 1ra palabra de la rafaga (columna base)
        end
        // EXPERIMENTO: un ciclo mas de margen antes de capturar la 2da
        // palabra -- coincide con el arranque del auto-precharge interno,
        // puede necesitar mas asentamiento que la 1ra.
        {RDBURST2, T_RCD+CAS+4'd3}: begin
            dout32_b_buf <= dq_in;   // 2da palabra (columna+1), llega sola
            busy  <= 0;
            state <= IDLE;
        end

        {WRITE, T_RCD}: begin
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_Write;
            SDRAM_A[10] <= 1'b1;
            SDRAM_A[9:0] <= {1'b0, addr_buf[COL_WIDTH-1+2:2]};
            SDRAM_DQM <= addr_buf[1:0] == 2'd0 ? 4'b1110 :
                         addr_buf[1:0] == 2'd1 ? 4'b1101 :
                         addr_buf[1:0] == 2'd2 ? 4'b1011 : 4'b0111;
            off <= addr_buf[1:0];
            dq_out <= {din_buf,din_buf,din_buf,din_buf};
            dq_oen <= 1'b0;
        end
        {WRITE, T_RCD+4'd1}: begin
            SDRAM_DQM <= 4'b1111;
            dq_oen    <= 1'b1;
        end
        {WRITE, T_RCD+4'd1+T_WR+T_RP}: begin
            busy <= 0;
            state <= IDLE;
        end

        {REFRESH, T_RC}: begin
            state <= IDLE;
            busy <= 0;
        end
    endcase

    if (~resetn) begin
        busy <= 1'b1;
        dq_oen <= 1'b1;
        SDRAM_DQM <= 4'b0;
        state <= INIT;
    end
end

// pulso cfg_now despues del retardo de inicializacion (normalmente 200us)
reg  [14:0]   rst_cnt;
reg rst_done, rst_done_p1, cfg_busy;

always @(posedge clk) begin
    rst_done_p1 <= rst_done;
    cfg_now     <= rst_done & ~rst_done_p1;

    if (rst_cnt != FREQ / 1000 * 200 / 1000) begin
        rst_cnt  <= rst_cnt[14:0] + 1;
        rst_done <= 1'b0;
        cfg_busy <= 1'b1;
    end else begin
        rst_done <= 1'b1;
        cfg_busy <= 1'b0;
    end

    if (~resetn) begin
        rst_cnt  <= 15'd0;
        rst_done <= 1'b0;
        cfg_busy <= 1'b1;
    end
end

endmodule
