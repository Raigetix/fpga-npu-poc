// weight_stream.v -- Etapa 4, Fase 4: motor de streaming de pesos desde
// SDRAM hacia los 8 carriles de computo, ahora usando el modo de RAFAGA
// MANUAL de sdram.v (ver ese archivo) en vez de lecturas de a una palabra.
//
// Por que el cambio (Fase 3 -> Fase 4): con lecturas de a una palabra
// verificadas por mayoria de 3, el streaming corria 4-6x mas lento que el
// computo de los 8 carriles (ver el comentario que tenia esta cabecera
// antes -- todavia vale la cuenta de fondo, solo cambia COMO se hace la
// verificacion). Cada lectura de palabra pagaba el costo completo de
// activar+leer+precargar (5 ciclos) por SOLO 4 pesos utiles. La rafaga
// manual de sdram.v activa la fila UNA vez y trae BURST_WORDS=8 palabras
// seguidas (32 pesos, 4 grupos completos) por apenas ~14 ciclos en vez de
// 8*5=40 -- el mismo costo fijo de activar/precargar ahora se reparte entre
// 8 palabras en vez de 1.
//
// Verificacion de datos: se mantiene la MISMA idea de mayoria de 3 que
// antes (bug de corrupcion intermitente ~0.3-0.5%, ver pll_sdram.v y
// top_sdram_p1.v/top_sdram_p2.v), pero aplicada a la RAFAGA COMPLETA en vez
// de palabra por palabra: se trae la rafaga dos veces y se comparan las 8
// palabras; si coinciden, lista. Si alguna difiere, se trae una tercera vez
// y se vota por mayoria posicion por posicion. Como el costo fijo de
// activar/precargar ya esta repartido entre 8 palabras, hacer esto 2-3
// veces por rafaga sigue siendo mucho mas rapido que el esquema anterior
// (ver la cuenta completa en el comentario de cabecera de sdram.v).
//
// Layout en SDRAM: identico a antes -- direccion = base + a*8 + l, para el
// paso local `a` (continuo a lo largo de TODAS las capas) y carril `l`
// (0..7). La rafaga de 8 palabras arranca en base+fetch_addr*8 y cubre 4
// pasos locales consecutivos (fetch_addr, +1, +2, +3) de una sola vez.
// ASUME que la rafaga no cruza un limite de fila de la SDRAM (256
// columnas) -- valido mientras la base de los pesos (WSTREAM_SET_BASE) sea
// multiplo de 1024 bytes, que es el caso hoy (siempre 0). Ver sdram.v.
module weight_stream (
    input  wire        clk,
    input  wire        rst,        // pulso: reinicia el puntero de fetch a 0 y vacia la FIFO
    input  wire [22:0] base_addr,

    // ---- Interfaz hacia el arbitro compartido (mismo patron que Fase 3:
    // este modulo solo EXPONE que quiere leer y que direccion, el arbitro
    // del top-level decide cuando servirlo y maneja sdram_rdburst_pulse/
    // sdram_op_addr, unico lugar que los escribe) ----
    output wire         ws_want_req,
    output wire [22:0]  ws_addr,
    input  wire         ws_issue,     // == want_ws_op del arbitro, combinacional, este mismo ciclo
    input  wire         sdram_busy,
    input  wire [31:0]  sdram_dout32,
    input  wire         sdram_burst_word_valid,  // pulso, una vez por cada palabra de la rafaga

    // ---- Interfaz hacia el consumidor (motor de computo) ----
    output wire               data_valid,   // hay un grupo listo en la cabeza de la FIFO
    output wire signed [7:0]  lane0, lane1, lane2, lane3, lane4, lane5, lane6, lane7,
    input  wire                pop           // consumir el grupo de la cabeza, avanzar a la FIFO
);

    localparam integer BURST_WORDS       = 8;
    localparam integer GROUPS_PER_BURST  = 4;   // 8 palabras = 4 grupos (lo+hi cada uno)

    localparam [3:0] WS_WAIT_ROOM = 4'd0,
                      WS_REQ_A    = 4'd1, WS_CAP_A = 4'd2,
                      WS_REQ_B    = 4'd3, WS_CAP_B = 4'd4,
                      WS_REQ_C    = 4'd5, WS_CAP_C = 4'd6,
                      WS_RESOLVE  = 4'd7,
                      WS_PUSH     = 4'd8;
    reg [3:0]  ws_state = WS_WAIT_ROOM;
    reg [15:0] fetch_addr = 16'd0;    // paso local del PRIMER grupo de la rafaga
    reg [3:0]  widx;                  // palabras capturadas en la rafaga actual (0..8)
    reg [2:0]  gidx;                  // grupo actual al empujar a la FIFO (0..3)
    reg        need_c;                // A y B no coincidieron, hace falta la tercera lectura
    // NO pedir nada hasta el primer START real: sin esto, el motor de fetch
    // arranca a leer SDRAM (rafagas reales, activar/precargar) apenas la
    // FPGA se configura, mucho antes de que el ESP32 escriba un solo peso
    // o mande START -- de diagnostico de una regresion real donde la carga
    // de pesos por SPI empezaba a fallar sistematicamente con este motor
    // activo. Una vez que se ve el primer START queda habilitado para
    // siempre (no se vuelve a apagar en resets posteriores).
    reg        primed = 1'b0;

    reg [31:0] bufA [0:BURST_WORDS-1];
    reg [31:0] bufB [0:BURST_WORDS-1];
    reg [31:0] bufC [0:BURST_WORDS-1];

    wire [22:0] burst_base = base_addr + {fetch_addr, 3'b000};
    assign ws_addr = burst_base;
    assign ws_want_req = (ws_state == WS_REQ_A) || (ws_state == WS_REQ_B) || (ws_state == WS_REQ_C);

    // ---- FIFO de 8 grupos (8 bytes cada uno) -- el doble que antes, para
    // que quepa una rafaga entera (4 grupos) mientras el consumidor todavia
    // tiene grupos de la rafaga anterior sin consumir (solapamiento real
    // entre el fetch de la proxima rafaga y el computo de la actual). ----
    localparam integer FIFO_DEPTH = 8;
    reg [63:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [2:0]  wr_ptr = 3'd0, rd_ptr = 3'd0;
    reg [3:0]  fifo_count = 4'd0;
    wire       fifo_full  = (fifo_count == FIFO_DEPTH[3:0]);
    wire       fifo_empty = (fifo_count == 4'd0);
    // no arrancar una rafaga nueva salvo que entren sus 4 grupos sin
    // desbordar -- asi el empuje de los 4 grupos nunca tiene que esperar
    // a mitad de camino por falta de lugar.
    wire       room_for_burst = (fifo_count <= FIFO_DEPTH[3:0] - GROUPS_PER_BURST[3:0]);

    assign data_valid = !fifo_empty;
    wire [63:0] fifo_head = fifo_mem[rd_ptr];
    assign lane0 = fifo_head[7:0];
    assign lane1 = fifo_head[15:8];
    assign lane2 = fifo_head[23:16];
    assign lane3 = fifo_head[31:24];
    assign lane4 = fifo_head[39:32];
    assign lane5 = fifo_head[47:40];
    assign lane6 = fifo_head[55:48];
    assign lane7 = fifo_head[63:56];

    // Unico bloque que escribe wr_ptr/rd_ptr/fifo_count/fifo_mem -- push
    // (motor de fetch, mas abajo) y pop (consumidor externo) conviven aca
    // para no repetir el error de "dos drivers" de la sesion.
    reg push_now = 1'b0; // pulso, fijado por el motor de fetch mas abajo
    reg [63:0] push_data;
    always @(posedge clk) begin
        if (push_now && pop) begin
            fifo_mem[wr_ptr] <= push_data;
            wr_ptr <= wr_ptr + 3'd1;
            rd_ptr <= rd_ptr + 3'd1;
            // fifo_count sin cambios (entra uno, sale uno)
        end else if (push_now) begin
            fifo_mem[wr_ptr] <= push_data;
            wr_ptr     <= wr_ptr + 3'd1;
            fifo_count <= fifo_count + 4'd1;
        end else if (pop && !fifo_empty) begin
            rd_ptr     <= rd_ptr + 3'd1;
            fifo_count <= fifo_count - 4'd1;
        end
        if (rst) begin
            wr_ptr <= 3'd0; rd_ptr <= 3'd0; fifo_count <= 4'd0;
        end
    end

    // ---- Motor de fetch: unico bloque que escribe ws_state/fetch_addr/
    // widx/gidx/need_c/bufA/bufB/bufC/push_now/push_data.
    integer k;
    always @(posedge clk) begin
        push_now <= 1'b0;

        case (ws_state)
            WS_WAIT_ROOM: if (primed && room_for_burst) begin
                widx     <= 4'd0;
                ws_state <= WS_REQ_A;
            end

            // ---- primera lectura de la rafaga ----
            WS_REQ_A: if (ws_issue) ws_state <= WS_CAP_A;
            WS_CAP_A: begin
                if (sdram_burst_word_valid) begin
                    bufA[widx] <= sdram_dout32;
                    widx       <= widx + 4'd1;
                end
                if (!sdram_busy && widx == BURST_WORDS[3:0]) begin
                    widx     <= 4'd0;
                    ws_state <= WS_REQ_B;
                end
            end

            // ---- segunda lectura, para comparar contra la primera ----
            WS_REQ_B: if (ws_issue) ws_state <= WS_CAP_B;
            WS_CAP_B: begin
                if (sdram_burst_word_valid) begin
                    bufB[widx] <= sdram_dout32;
                    widx       <= widx + 4'd1;
                end
                if (!sdram_busy && widx == BURST_WORDS[3:0]) begin
                    widx     <= 4'd0;
                    ws_state <= WS_RESOLVE;
                end
            end

            // ---- tercera lectura, solo si A y B no coincidieron en algo ----
            WS_REQ_C: if (ws_issue) ws_state <= WS_CAP_C;
            WS_CAP_C: begin
                if (sdram_burst_word_valid) begin
                    bufC[widx] <= sdram_dout32;
                    widx       <= widx + 4'd1;
                end
                if (!sdram_busy && widx == BURST_WORDS[3:0]) begin
                    gidx     <= 3'd0;
                    ws_state <= WS_PUSH;
                end
            end

            // ---- resolver: si A==B en las 8 palabras, listo; si no,
            // pedir la tercera lectura para desempatar ----
            WS_RESOLVE: begin
                need_c = 1'b0;
                for (k = 0; k < BURST_WORDS; k = k + 1)
                    if (bufA[k] != bufB[k]) need_c = 1'b1;
                if (need_c) begin
                    ws_state <= WS_REQ_C;
                end else begin
                    gidx     <= 3'd0;
                    ws_state <= WS_PUSH;
                end
            end

            // ---- empujar los 4 grupos de la rafaga a la FIFO, uno por
            // ciclo (ya se garantizo lugar en WS_WAIT_ROOM, asi que nunca
            // hace falta esperar aca en el medio) ----
            WS_PUSH: begin
                push_data <= word_pair(gidx);
                push_now  <= 1'b1;
                if (gidx == GROUPS_PER_BURST[2:0] - 3'd1) begin
                    fetch_addr <= fetch_addr + 16'd4;
                    ws_state   <= WS_WAIT_ROOM;
                end else begin
                    gidx <= gidx + 3'd1;
                end
            end

            default: ws_state <= WS_WAIT_ROOM;
        endcase

        if (rst) begin
            ws_state    <= WS_WAIT_ROOM;
            fetch_addr  <= 16'd0;
            widx        <= 4'd0;
            gidx        <= 3'd0;
            push_now    <= 1'b0;
            primed      <= 1'b1;
        end
    end

    // Palabra "lo" (carriles 0-3) y "hi" (carriles 4-7) del grupo local
    // `g` dentro de la rafaga actual: palabra 2*g y 2*g+1. Si hizo falta la
    // tercera lectura, mayoria de 3 posicion por posicion; si no, A vale
    // (ya se confirmo A==B en WS_RESOLVE).
    function [63:0] word_pair;
        input [2:0] g;
        reg [31:0] lo, hi;
        begin
            lo = majority(bufA[{g,1'b0}], bufB[{g,1'b0}], bufC[{g,1'b0}]);
            hi = majority(bufA[{g,1'b1}], bufB[{g,1'b1}], bufC[{g,1'b1}]);
            word_pair = {hi, lo};
        end
    endfunction

    // Mayoria de 3: si a==b, ese valor (c ni se necesito -- caso comun,
    // A y B ya coincidian). Si no, el que coincida con c; si ninguno
    // coincide con ninguno (rarisimo, nunca visto en Fase 3), c igual --
    // mejor un valor fresco que uno de dos que ya se sabe que no coinciden.
    function [31:0] majority;
        input [31:0] a, b, c;
        begin
            if (a == b) majority = a;
            else if (a == c) majority = a;
            else if (b == c) majority = b;
            else majority = c;
        end
    endfunction

endmodule
