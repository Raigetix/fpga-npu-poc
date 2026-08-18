// weight_stream.v -- Etapa 4, Fase 3: motor de streaming de pesos desde
// SDRAM hacia los 8 carriles de computo.
//
// HISTORIA DE LA VERIFICACION -- esta version pide UNA lectura por
// palabra y confia en lo que recibe; no vota por mayoria. La votacion
// (leer 2-3 veces, comparar) existio en este modulo desde la Fase 3
// original, como mitigacion de un bug de datos intermitente ~0.3-0.5%
// nunca resuelto en su causa fisica EN ESE MOMENTO. La causa raiz se
// encontro despues (ver docs/caracterizacion-frecuencia-sdram.md): es
// una carrera de temporizacion en la lectura que sigue justo despues de
// un refresco de la SDRAM (~97% de la corrupcion cae ahi). Un primer
// intento de hacer la votacion CONDICIONAL (solo verificar esa lectura
// puntual) DESDE ACA, con el arbitro solo pasando una bandera
// (`refresh_owed`), tenia una carrera real: la bandera se podia "gastar"
// en la relectura de OTRA verificacion en curso en vez de en la lectura
// que de verdad la necesitaba (ver commit "Intento de llevar la
// verificacion condicional a produccion" en
// feature/production-refresh-verify-fix -- causo una caida de precision
// de 97% a 72%, revertido).
//
// La version correcta mueve TODA la decision y ejecucion de la
// verificacion al arbitro (top_sdram_p3.v) -- este modulo vuelve a ser
// tan simple como en la Fase 2 (una lectura por palabra, listo) porque
// el arbitro le entrega 'busy'/'dout32' YA VERIFICADOS de forma
// transparente: nunca se entera si por debajo hubo 1, 2 o 3 lecturas
// reales de SDRAM. Ver top_sdram_p3.v para la maquina de verificacion.
//
// A diferencia de Fase 2 (precarga TODO el rango de una vez a un buffer
// grande), esto es streaming real: una FIFO chica (4 grupos) que se va
// llenando continuamente mientras el motor de computo consume, sin que el
// buffer necesite ver nunca el modelo completo -- asi el tamaño del modelo
// queda desacoplado de cuanta BRAM hay disponible.
//
// Cuenta de caudal (motivo de leer PALABRAS de 32 bits en vez de bytes):
// sdram.v YA trae la palabra de 32 bits completa en cada operacion (dq_in
// es de 32 bits) -- leyendo con dout32 en vez de dout, una sola operacion
// trae 4 pesos de una vez.
//
// Layout en SDRAM: para el paso local `a` (0,1,2,..., continuo a lo largo
// de TODAS las capas -- ver mlp_engine_par_stream.v, el barrido de pesos
// es una sola secuencia monotona sin saltos entre capas) y carril `l`
// (0..7): direccion_sdram = base_addr + a*8 + l. Los primeros 4 bytes de
// cada grupo de 8 (direccion base_addr+a*8+0..3) son los carriles 0-3
// (palabra "lo"); los siguientes 4 (+4..7) son los carriles 4-7 (palabra
// "hi"). base_addr debe ser multiplo de 4 (alineacion de palabra).
module weight_stream (
    input  wire        clk,
    input  wire        rst,        // pulso: reinicia el puntero de fetch a 0 y vacia la FIFO
    input  wire [22:0] base_addr,

    // ---- Interfaz hacia el arbitro compartido: este modulo solo EXPONE
    // que quiere leer y que direccion, el arbitro del top-level decide
    // cuando servirlo. 'sdram_busy'/'sdram_dout32' son la vista YA
    // VERIFICADA que entrega el arbitro (ver top_sdram_p3.v), no
    // necesariamente los cables crudos de sdram.v. ----
    output wire         ws_want_req,
    output wire [22:0]  ws_addr,
    output wire         ws_is_burst2, // pide rd_burst2 (lo+hi de una) en vez de rd simple
    input  wire         ws_issue,     // == want_ws_op del arbitro, combinacional, este mismo ciclo
    input  wire         sdram_busy,
    input  wire [31:0]  sdram_dout32,
    input  wire [31:0]  sdram_dout32_a, // palabra lo de rd_burst2 (ya verificada por el arbitro)
    input  wire [31:0]  sdram_dout32_b, // palabra hi de rd_burst2 (ya verificada por el arbitro)

    // ---- Interfaz hacia el consumidor (motor de computo) ----
    output wire               data_valid,   // hay un grupo listo en la cabeza de la FIFO
    output wire signed [7:0]  lane0, lane1, lane2, lane3, lane4, lane5, lane6, lane7,
    input  wire                pop           // consumir el grupo de la cabeza, avanzar a la FIFO
);

    localparam [2:0] WS_REQ1=3'd0, WS_WAIT1=3'd1, WS_NEXT_WORD=3'd2, WS_PUSH=3'd3,
                      WS_WAIT_BURST=3'd4;
    reg [2:0]  ws_state = WS_REQ1;
    reg        word_sel = 1'b0;      // 0=lo (carriles 0-3), 1=hi (carriles 4-7)
    reg [15:0] fetch_addr = 16'd0;   // paso local, se multiplica x8 para la direccion real
    reg [31:0] val_lo, val_hi;
    reg        busy_seen = 1'b0;     // mismo motivo que pf_busy_seen en Fase 2

    wire [22:0] group_base = base_addr + {fetch_addr, 3'b000};
    assign ws_addr = group_base + (word_sel ? 23'd4 : 23'd0);
    assign ws_want_req = (ws_state == WS_REQ1);

    // rd_burst2 trae lo+hi (columna base y columna+1 de la MISMA fila) en
    // una sola operacion -- ver sdram.v. Solo vale si lo NO cae en la
    // ULTIMA columna de la fila (columna 255): ahi hi se saldria a la fila
    // siguiente, que rd_burst2 no maneja (ver comentario de rd_burst2 en
    // sdram.v). Para ese caso puntual (1 de cada 128 grupos, como mucho)
    // se cae al camino viejo de dos lecturas simples, igual que antes.
    wire [7:0] lo_column     = group_base[9:2];
    wire       want_burst_now = (word_sel == 1'b0) && (lo_column != 8'hFF);
    assign ws_is_burst2 = want_burst_now;

    // ---- FIFO de 4 grupos (8 bytes cada uno) ----
    localparam integer FIFO_DEPTH = 4;
    reg [63:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [1:0]  wr_ptr = 2'd0, rd_ptr = 2'd0;
    reg [2:0]  fifo_count = 3'd0;
    wire       fifo_full  = (fifo_count == FIFO_DEPTH[2:0]);
    wire       fifo_empty = (fifo_count == 3'd0);

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
            wr_ptr <= wr_ptr + 2'd1;
            rd_ptr <= rd_ptr + 2'd1;
            // fifo_count sin cambios (entra uno, sale uno)
        end else if (push_now) begin
            fifo_mem[wr_ptr] <= push_data;
            wr_ptr     <= wr_ptr + 2'd1;
            fifo_count <= fifo_count + 3'd1;
        end else if (pop && !fifo_empty) begin
            rd_ptr     <= rd_ptr + 2'd1;
            fifo_count <= fifo_count - 3'd1;
        end
        if (rst) begin
            wr_ptr <= 2'd0; rd_ptr <= 2'd0; fifo_count <= 3'd0;
        end
    end

    // ---- Motor de fetch: unico bloque que escribe ws_state/word_sel/
    // fetch_addr/val_lo/val_hi/busy_seen/push_now/push_data.
    always @(posedge clk) begin
        push_now <= 1'b0;

        case (ws_state)
            WS_REQ1: if (ws_issue) begin
                busy_seen <= 1'b0;
                ws_state  <= want_burst_now ? WS_WAIT_BURST : WS_WAIT1;
            end
            WS_WAIT_BURST: begin
                if (sdram_busy) busy_seen <= 1'b1;
                if (busy_seen && !sdram_busy) begin
                    busy_seen <= 1'b0;
                    val_lo   <= sdram_dout32_a;
                    val_hi   <= sdram_dout32_b;
                    ws_state <= WS_PUSH;
                end
            end
            WS_WAIT1: begin
                if (sdram_busy) busy_seen <= 1'b1;
                if (busy_seen && !sdram_busy) begin
                    busy_seen <= 1'b0;
                    if (word_sel) val_hi <= sdram_dout32; else val_lo <= sdram_dout32;
                    ws_state <= WS_NEXT_WORD;
                end
            end
            WS_NEXT_WORD: begin
                if (word_sel) begin
                    word_sel <= 1'b0;
                    ws_state <= WS_PUSH;
                end else begin
                    word_sel <= 1'b1;
                    ws_state <= WS_REQ1;
                end
            end
            WS_PUSH: begin
                if (!fifo_full) begin
                    push_data   <= {val_hi, val_lo};
                    push_now    <= 1'b1;
                    fetch_addr  <= fetch_addr + 16'd1;
                    ws_state    <= WS_REQ1;
                end
                // si esta llena, se queda ESPERANDO en este mismo estado
                // hasta que el consumidor haga pop (fifo_full baje).
            end
            default: ws_state <= WS_REQ1;
        endcase

        if (rst) begin
            ws_state    <= WS_REQ1;
            word_sel    <= 1'b0;
            fetch_addr  <= 16'd0;
            busy_seen   <= 1'b0;
            push_now    <= 1'b0;
        end
    end

endmodule
