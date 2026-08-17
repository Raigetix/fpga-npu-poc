// weight_stream.v -- Etapa 4, Fase 3: motor de streaming de pesos desde
// SDRAM hacia los 8 carriles de computo, con verificacion por mayoria (ver
// top_sdram_p1.v/top_sdram_p2.v para el porque de la verificacion: bug de
// datos intermitente ~0.3-0.5%, nunca resuelto en su causa fisica exacta,
// mitigado por diseño en vez de perseguirlo mas).
//
// A diferencia de Fase 2 (precarga TODO el rango de una vez a un buffer
// grande), esto es streaming real: una FIFO chica (4 grupos) que se va
// llenando continuamente mientras el motor de computo consume, sin que el
// buffer necesite ver nunca el modelo completo -- asi el tamaño del modelo
// queda desacoplado de cuanta BRAM hay disponible.
//
// Cuenta de caudal (motivo de leer PALABRAS de 32 bits en vez de bytes):
// el computo de 8 carriles en paralelo consume 8 pesos nuevos cada 5 ciclos
// de clk_sys (~93ns a 54MHz). Una lectura verificada de un solo byte tarda
// ~200-300ns (2-3 lecturas reales de SDRAM) -- para juntar los 8 bytes de
// un paso de computo se necesitarian 8 lecturas de byte (~1.6-2.4us), 17-26x
// mas lento que el computo. sdram.v YA trae la palabra de 32 bits completa
// en cada operacion (dq_in es de 32 bits) y hoy se descartan 3 de cada 4
// bytes -- leyendo con dout32 en vez de dout, una sola operacion trae 4
// pesos (~4x de caudal), sin costo extra de latencia (el dato ya estaba
// disponible). Sigue sin alcanzar al computo (~4-6x mas lento en vez de
// 17-26x), pero es la mejora de mas impacto con menos cambio.
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

    // ---- Interfaz hacia el arbitro compartido (mismo patron que Fase 2:
    // este modulo solo EXPONE que quiere leer y que direccion, el arbitro
    // del top-level decide cuando servirlo y maneja sdram_rd_pulse/
    // sdram_op_addr, unico lugar que los escribe) ----
    output wire         ws_want_req,
    output wire [22:0]  ws_addr,
    input  wire         ws_issue,     // == want_ws_op del arbitro, combinacional, este mismo ciclo
    input  wire         sdram_busy,
    input  wire [31:0]  sdram_dout32,

    // ---- Interfaz hacia el consumidor (motor de computo) ----
    output wire               data_valid,   // hay un grupo listo en la cabeza de la FIFO
    output wire signed [7:0]  lane0, lane1, lane2, lane3, lane4, lane5, lane6, lane7,
    input  wire                pop           // consumir el grupo de la cabeza, avanzar a la FIFO
);

    localparam [3:0] WS_REQ1=4'd0, WS_WAIT1=4'd1, WS_REQ2=4'd2, WS_WAIT2=4'd3,
                      WS_REQ3=4'd4, WS_WAIT3=4'd5, WS_NEXT_WORD=4'd6, WS_PUSH=4'd7;
    reg [3:0]  ws_state = WS_REQ1;
    reg        word_sel = 1'b0;      // 0=lo (carriles 0-3), 1=hi (carriles 4-7)
    reg [15:0] fetch_addr = 16'd0;   // paso local, se multiplica x8 para la direccion real
    reg [31:0] val1, val2;
    reg [31:0] val_lo, val_hi;
    reg        busy_seen = 1'b0;     // mismo motivo que pf_busy_seen en Fase 2

    wire [22:0] group_base = base_addr + {fetch_addr, 3'b000};
    assign ws_addr = group_base + (word_sel ? 23'd4 : 23'd0);
    assign ws_want_req = (ws_state == WS_REQ1) || (ws_state == WS_REQ2) || (ws_state == WS_REQ3);

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
    // fetch_addr/val1/val2/val_lo/val_hi/busy_seen/push_now/push_data.
    always @(posedge clk) begin
        push_now <= 1'b0;

        case (ws_state)
            WS_REQ1: if (ws_issue) ws_state <= WS_WAIT1;
            WS_WAIT1: begin
                if (sdram_busy) busy_seen <= 1'b1;
                if (busy_seen && !sdram_busy) begin
                    val1      <= sdram_dout32;
                    ws_state  <= WS_REQ2;
                    busy_seen <= 1'b0;
                end
            end
            WS_REQ2: if (ws_issue) ws_state <= WS_WAIT2;
            WS_WAIT2: begin
                if (sdram_busy) busy_seen <= 1'b1;
                if (busy_seen && !sdram_busy) begin
                    val2      <= sdram_dout32;
                    busy_seen <= 1'b0;
                    if (sdram_dout32 == val1) begin
                        if (word_sel) val_hi <= val1; else val_lo <= val1;
                        ws_state <= WS_NEXT_WORD;
                    end else begin
                        ws_state <= WS_REQ3;
                    end
                end
            end
            WS_REQ3: if (ws_issue) ws_state <= WS_WAIT3;
            WS_WAIT3: begin
                if (sdram_busy) busy_seen <= 1'b1;
                if (busy_seen && !sdram_busy) begin
                    busy_seen <= 1'b0;
                    // mayoria de 3: si la tercera no coincide con ninguna
                    // de las dos primeras (caso rarisimo, nunca visto en
                    // Fase 2), se toma la tercera igual -- mejor un valor
                    // fresco que uno de dos que ya se sabe que no coinciden.
                    if (sdram_dout32 == val1) begin
                        if (word_sel) val_hi <= val1; else val_lo <= val1;
                    end else if (sdram_dout32 == val2) begin
                        if (word_sel) val_hi <= val2; else val_lo <= val2;
                    end else begin
                        if (word_sel) val_hi <= sdram_dout32; else val_lo <= sdram_dout32;
                    end
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
