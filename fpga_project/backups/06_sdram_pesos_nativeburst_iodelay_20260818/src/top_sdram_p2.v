// top_sdram_p2.v -- Etapa 4, Fase 2: motor de precarga (SDRAM -> buffer de
// BRAM) con lectura VERIFICADA. Construido sobre la base ya probada de
// Fase 1 (sdram.v sin tocar, mismo arbitro de cola+refresco, mismo
// mecanismo de snapshot SPI congelado por trama) -- ver top_sdram_p1.v
// para el detalle de cada uno de esos tres bugs y por que estan resueltos
// asi.
//
// Fase 1 se investigo a fondo un bug de datos residual (~0.3-0.5% de las
// lecturas devuelven el valor de la lectura ANTERIOR, aislado, sin
// arrastre) que sobrevivio a descartar TODO lo controlable por software:
// direccion, comandos perdidos, timing de captura, margenes de precharge/
// activacion, frecuencia, fase de reloj, estancamientos internos -- ver
// README.md, seccion Etapa 4. Se confirmo ahi que una RELECTURA inmediata
// de la misma direccion es practicamente siempre correcta (el evento es
// aislado, no persistente). En vez de seguir persiguiendo la causa fisica
// exacta, esta fase DISEÑA ALREDEDOR: cada byte se lee hasta 3 veces
// (compara las primeras dos; si no coinciden, la tercera desempata por
// mayoria) antes de guardarlo en el buffer -- la probabilidad de que 2 de
// 3 lecturas independientes coincidan por casualidad en el mismo valor
// incorrecto es (0.005)^2 = 0.0025%, IRRELEVANTE frente al ~0.3-0.5%
// original.
//
// Protocolo (mismo frame de 48 bits):
//   CMD_NOP               = 0x00
//   CMD_SDRAM_SET_ADDR    = 0x01  A[15:0]=addr[15:0] B[6:0]=addr[22:16]
//                                   (puntero de ESCRITURA directa a SDRAM,
//                                   para armar un patron de prueba conocido
//                                   -- mismo comando que Fase 1)
//   CMD_SDRAM_WR          = 0x02  B_lo=byte a escribir en el puntero
//                                   actual; el puntero avanza en 1 despues
//   CMD_PREFETCH_SET_BASE = 0x10  A[15:0]=base[15:0] B[6:0]=base[22:16]
//                                   (direccion en SDRAM donde arranca la
//                                   precarga)
//   CMD_PREFETCH_SET_LEN  = 0x11  A[15:0]=cantidad de bytes a precargar
//                                   (<= BUF_DEPTH)
//   CMD_PREFETCH_START    = 0x12  dispara la precarga verificada; busy=1
//                                   hasta que termina TODO el rango
//   CMD_PREFETCH_SET_RDPTR= 0x14  A[15:0]=puntero de LECTURA del buffer
//   CMD_PREFETCH_RD_BYTE  = 0x13  lee buf_mem[rdptr], incrementa; el dato
//                                   aparece en el byte 1 de la respuesta
//
//   MISO: [status: bit0=busy bit1=pll_lock bit2=pf_done]
//         [ultimo byte leido del buffer]
//         [pf_retry_count: 16 bits] [pf_fail3_count: 16 bits]
//
//   pf_retry_count: cuantos bytes de la ULTIMA precarga necesitaron una
//     segunda lectura (las primeras dos no coincidieron) -- da una medida
//     empirica directa de la tasa de error real.
//   pf_fail3_count: cuantos bytes necesitaron las 3 lecturas Y las 3
//     salieron distintas entre si (no hay mayoria) -- deberia dar 0
//     siempre; si no, es señal de que el problema es mas severo de lo
//     medido en Fase 1.
module top_sdram_p2 #(
    parameter integer BUF_DEPTH  = 4096,
    parameter integer BUF_ABITS  = 12
) (
    input  wire       clk,   // 27 MHz onboard, pin 4
    input  wire       sclk,  // SPI SCLK, desde ESP32
    input  wire       cs_n,  // SPI CS, activo en bajo, desde ESP32
    input  wire       mosi,  // SPI MOSI, desde ESP32
    output wire       miso,  // SPI MISO, hacia ESP32
    output wire [5:0] led,   // debug visual

    // ---- SDRAM embebida: nombres magicos, sin IO_LOC ----
    output wire        O_sdram_clk,
    output wire        O_sdram_cke,
    output wire        O_sdram_cs_n,
    output wire        O_sdram_cas_n,
    output wire        O_sdram_ras_n,
    output wire        O_sdram_wen_n,
    inout  wire [31:0] IO_sdram_dq,
    output wire [10:0] O_sdram_addr,
    output wire [1:0]  O_sdram_ba,
    output wire [3:0]  O_sdram_dqm
);

    localparam [7:0] CMD_NOP               = 8'h00;
    localparam [7:0] CMD_SDRAM_SET_ADDR    = 8'h01;
    localparam [7:0] CMD_SDRAM_WR          = 8'h02;
    localparam [7:0] CMD_PREFETCH_RD_BYTE  = 8'h13;
    localparam [7:0] CMD_PREFETCH_SET_BASE = 8'h10;
    localparam [7:0] CMD_PREFETCH_SET_LEN  = 8'h11;
    localparam [7:0] CMD_PREFETCH_START    = 8'h12;
    localparam [7:0] CMD_PREFETCH_SET_RDPTR= 8'h14;

    wire clk_sys;
    wire clk_sdram;
    wire pll_lock;
    pll_sdram u_pll (
        .clk_in  (clk),
        .clk_out (clk_sys),
        .clk_sdram(clk_sdram),
        .lock    (pll_lock)
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

    // ================= Controlador de SDRAM (identico a Fase 1) =================
    wire        sdram_busy;
    wire        sdram_data_ready;
    wire [7:0]  sdram_dout;
    reg         sdram_resetn = 1'b0;
    always @(posedge clk_sys) sdram_resetn <= pll_lock;

    reg [22:0] sdram_target;
    reg [22:0] sdram_op_addr;
    reg [7:0]  sdram_op_data;
    reg        sdram_rd_pulse, sdram_wr_pulse, sdram_refresh_pulse;

    reg        pending_wr, pending_rd;
    reg [22:0] pending_addr;
    reg [7:0]  pending_data;

    wire cmd_ok = frame_done_d1;

    localparam REFRESH_INTERVAL = 700;
    reg [9:0] refresh_ctr = 10'd0;
    reg       refresh_pending = 1'b0;
    always @(posedge clk_sys) begin
        if (refresh_ctr == REFRESH_INTERVAL) begin
            refresh_ctr     <= 10'd0;
            refresh_pending <= 1'b1;
        end else begin
            refresh_ctr <= refresh_ctr + 10'd1;
        end
        if (want_refresh) refresh_pending <= 1'b0;
    end

    // ---- Arbitro: SPI directo (pending_wr/pending_rd) tiene prioridad
    // sobre el motor de precarga (pf_want_req), que a su vez tiene
    // prioridad sobre el refresco. pf_state (mas abajo) hace el papel de
    // "pendiente" para el motor de precarga -- NO hay un pf_pending_rd
    // separado escrito desde dos lugares (mismo cuidado de "un solo
    // driver por registro" que en Fase 1).
    wire want_spi_op  = !sdram_busy && (pending_wr || pending_rd);
    wire want_pf_op   = !sdram_busy && !want_spi_op && pf_want_req;
    wire want_refresh = !sdram_busy && !want_spi_op && !want_pf_op && refresh_pending;

    always @(posedge clk_sys) begin
        sdram_wr_pulse      <= 1'b0;
        sdram_rd_pulse      <= 1'b0;
        sdram_refresh_pulse <= 1'b0;

        if (want_spi_op) begin
            sdram_op_addr <= pending_addr;
            if (pending_wr) begin
                sdram_op_data  <= pending_data;
                sdram_wr_pulse <= 1'b1;
                pending_wr     <= 1'b0;
            end else begin
                sdram_rd_pulse <= 1'b1;
                pending_rd     <= 1'b0;
            end
        end else if (want_pf_op) begin
            sdram_op_addr  <= pf_addr;
            sdram_rd_pulse <= 1'b1;
        end else if (want_refresh) begin
            sdram_refresh_pulse <= 1'b1;
        end

        if (cmd_ok) begin
            case (cmd)
                CMD_SDRAM_SET_ADDR: sdram_target <= {data_field[6:0], addr_field[15:0]};
                CMD_SDRAM_WR: begin
                    pending_addr <= sdram_target;
                    pending_data <= data_field[7:0];
                    pending_wr   <= 1'b1;
                    sdram_target <= sdram_target + 23'd1;
                end
                CMD_PREFETCH_SET_BASE:  pf_base_reg <= {data_field[6:0], addr_field[15:0]};
                CMD_PREFETCH_SET_LEN:   pf_len_reg  <= addr_field[12:0];
                // CMD_PREFETCH_SET_RDPTR se manda SIEMPRE antes de cada
                // CMD_PREFETCH_RD_BYTE (ver sdram_p2_poc.ino), asi que
                // este ultimo NO auto-incrementa buf_rdptr: hacerlo era un
                // bug real -- buf_rd_out se actualiza en vivo cada ciclo
                // (no esta congelado como el resto del snapshot), asi que
                // para cuando la respuesta llegaba a leerse el puntero ya
                // habia avanzado al siguiente indice, reportando siempre
                // el byte SIGUIENTE al pedido (patron "corrido en uno",
                // identico al primer bug de Fase 1 pero en el camino de
                // lectura del buffer en vez del de la SDRAM).
                CMD_PREFETCH_SET_RDPTR: buf_rdptr   <= addr_field[BUF_ABITS-1:0];
                default: ;
            endcase
        end
    end

    sdram #(.FREQ(54_000_000), .T_RP(4'd2), .T_RCD(4'd2)) u_sdram (
        .clk       (clk_sys),
        .clk_sdram (clk_sdram),
        .resetn    (sdram_resetn),
        .rd        (sdram_rd_pulse),
        .wr        (sdram_wr_pulse),
        .refresh   (sdram_refresh_pulse),
        .addr      (sdram_op_addr),
        .din       (sdram_op_data),
        .dout      (sdram_dout),
        .dout32    (),
        .data_ready(sdram_data_ready),
        .busy      (sdram_busy),

        .SDRAM_DQ  (IO_sdram_dq),
        .SDRAM_A   (O_sdram_addr),
        .SDRAM_BA  (O_sdram_ba),
        .SDRAM_nCS (O_sdram_cs_n),
        .SDRAM_nWE (O_sdram_wen_n),
        .SDRAM_nRAS(O_sdram_ras_n),
        .SDRAM_nCAS(O_sdram_cas_n),
        .SDRAM_CLK (O_sdram_clk),
        .SDRAM_CKE (O_sdram_cke),
        .SDRAM_DQM (O_sdram_dqm)
    );

    // ================= Motor de precarga verificada =================
    // Unico bloque que escribe pf_state/pf_idx/pf_addr/pf_val1/pf_val2/
    // pf_running/pf_done/pf_retry_count/pf_fail3_count/buf_mem. El
    // arbitro de arriba solo LEE pf_want_req/pf_addr (nunca los escribe),
    // asi que no hay riesgo de "dos drivers" para ningun registro.
    localparam [2:0] PF_IDLE=3'd0, PF_REQ1=3'd1, PF_WAIT1=3'd2, PF_REQ2=3'd3,
                      PF_WAIT2=3'd4, PF_REQ3=3'd5, PF_WAIT3=3'd6, PF_NEXT=3'd7;
    reg [2:0]            pf_state = PF_IDLE;
    reg [22:0]            pf_base_reg = 23'd0;
    reg [12:0]            pf_len_reg  = 13'd0;
    reg [22:0]            pf_addr;
    reg [BUF_ABITS-1:0]   pf_idx;
    reg [12:0]            pf_len;
    reg [7:0]             pf_val1, pf_val2;
    reg                   pf_running = 1'b0;
    reg                   pf_done    = 1'b0;
    reg [15:0]            pf_retry_count = 16'd0;
    reg [15:0]            pf_fail3_count = 16'd0;
    // Entre que el arbitro dispara el pulso (want_pf_op) y que sdram.v
    // realmente sube su propio busy pasa 1 ciclo (mismo hueco que top_busy
    // tapa en Fase 1 para el reporte SPI). Si PF_WAITx mirara !sdram_busy
    // directo, podria salir en ese mismo ciclo -- ANTES de que la lectura
    // arrancara -- y capturar el dato VIEJO como si fuera el nuevo. Este
    // flag exige ver busy subir primero, y recien despues esperar a que
    // baje, antes de dar la lectura por terminada.
    reg                   pf_busy_seen = 1'b0;

    wire pf_want_req   = (pf_state == PF_REQ1) || (pf_state == PF_REQ2) || (pf_state == PF_REQ3);
    // Pulso directo, sin registro intermedio: cmd_ok/cmd ya son senales
    // combinacionales estables durante 1 ciclo (spi_slave.v), asi que no
    // hace falta (ni conviene) un pf_start_req propio -- ese intento
    // anterior tenia una limpieza automatica que se disparaba el MISMO
    // ciclo que el arranque, pisandolo antes de que la FSM llegara a
    // verlo (el comando de arranque quedaba ignorado en silencio).
    wire pf_start_pulse = cmd_ok && (cmd == CMD_PREFETCH_START);

    reg [7:0] buf_mem [0:BUF_DEPTH-1];
    reg [BUF_ABITS-1:0] buf_rdptr = {BUF_ABITS{1'b0}};
    reg [7:0]           buf_rd_out;

    always @(posedge clk_sys) begin
        case (pf_state)
            PF_IDLE: begin
                pf_done <= 1'b0;
                if (pf_start_pulse) begin
                    pf_addr        <= pf_base_reg;
                    pf_idx         <= {BUF_ABITS{1'b0}};
                    pf_len         <= pf_len_reg;
                    pf_running     <= 1'b1;
                    pf_retry_count <= 16'd0;
                    pf_fail3_count <= 16'd0;
                    pf_state       <= (pf_len_reg == 13'd0) ? PF_IDLE : PF_REQ1;
                end
            end
            PF_REQ1: if (want_pf_op) pf_state <= PF_WAIT1;
            PF_WAIT1: begin
                if (sdram_busy) pf_busy_seen <= 1'b1;
                if (pf_busy_seen && !sdram_busy) begin
                    pf_val1      <= sdram_dout;
                    pf_state     <= PF_REQ2;
                    pf_busy_seen <= 1'b0;
                end
            end
            PF_REQ2: if (want_pf_op) pf_state <= PF_WAIT2;
            PF_WAIT2: begin
                if (sdram_busy) pf_busy_seen <= 1'b1;
                if (pf_busy_seen && !sdram_busy) begin
                    pf_val2      <= sdram_dout;
                    pf_busy_seen <= 1'b0;
                    if (sdram_dout == pf_val1) begin
                        buf_mem[pf_idx] <= pf_val1;
                        pf_state        <= PF_NEXT;
                    end else begin
                        pf_retry_count <= pf_retry_count + 16'd1;
                        pf_state       <= PF_REQ3;
                    end
                end
            end
            PF_REQ3: if (want_pf_op) pf_state <= PF_WAIT3;
            PF_WAIT3: begin
                if (sdram_busy) pf_busy_seen <= 1'b1;
                if (pf_busy_seen && !sdram_busy) begin
                    pf_busy_seen <= 1'b0;
                    if (sdram_dout == pf_val1) begin
                        buf_mem[pf_idx] <= pf_val1;
                    end else if (sdram_dout == pf_val2) begin
                        buf_mem[pf_idx] <= pf_val2;
                    end else begin
                        buf_mem[pf_idx] <= sdram_dout; // las 3 distintas -- caso raro, se cuenta aparte
                        pf_fail3_count  <= pf_fail3_count + 16'd1;
                    end
                    pf_state <= PF_NEXT;
                end
            end
            PF_NEXT: begin
                if ({1'b0, pf_idx} + 13'd1 == pf_len) begin
                    pf_running <= 1'b0;
                    pf_done    <= 1'b1;
                    pf_state   <= PF_IDLE;
                end else begin
                    pf_idx   <= pf_idx + 1'b1;
                    pf_addr  <= pf_addr + 23'd1;
                    pf_state <= PF_REQ1;
                end
            end
            default: pf_state <= PF_IDLE;
        endcase
    end

    always @(posedge clk_sys) buf_rd_out <= buf_mem[buf_rdptr];

    // ---- Busy visto desde afuera: igual necesidad que en Fase 1 (cubrir
    // el hueco entre "se acepto un comando" y "sdram.v ya subio su propio
    // busy"), mas pf_running para que el ESP32 pueda esperar el CMD_
    // PREFETCH_START con el mismo idiom de siempre (sondear con CMD_NOP
    // hasta ver busy=0).
    wire top_busy = sdram_busy | pending_wr | pending_rd | sdram_wr_pulse
                   | sdram_rd_pulse | pf_running | pf_start_pulse;

    // ---- Snapshot SPI: mismo mecanismo de Fase 1 (congelado durante toda
    // la trama, actualizado solo en los huecos entre tramas via cs_sync3)
    // -- ver top_sdram_p1.v para la explicacion completa de por que esto
    // hace falta (spi_slave.v necesita que tx_data se mantenga constante
    // durante un frame completo).
    reg cs_sync0 = 1'b1, cs_sync1 = 1'b1, cs_sync2 = 1'b1, cs_sync3 = 1'b1;
    always @(posedge clk_sys) begin
        cs_sync0 <= cs_n;
        cs_sync1 <= cs_sync0;
        cs_sync2 <= cs_sync1;
        cs_sync3 <= cs_sync2;
    end

    // Layout por byte: byte0=status{bit0=busy,bit1=pll_lock,bit2=pf_done},
    // byte1=buf_rd_out, byte2-3=pf_retry_count, byte4-5=pf_fail3_count.
    always @(posedge clk_sys) begin
        if (cs_sync3)
            tx_snapshot <= {5'd0, pf_done, pll_lock, top_busy, buf_rd_out, pf_retry_count, pf_fail3_count};
    end

    assign led = ~{top_busy, pll_lock, pf_done, 3'b000};

endmodule
