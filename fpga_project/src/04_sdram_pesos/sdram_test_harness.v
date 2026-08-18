// sdram_test_harness.v -- arnes de estres/verificacion de la SDRAM
// embebida para la caracterizacion de frecuencia (ver plan_testing_sdram.md
// y top_sdram_freqtest.v para el protocolo SPI que lo maneja).
//
// Genera un patron pseudo-aleatorio reproducible con un LFSR de 32 bits
// (polinomio maximal x^32+x^22+x^2+x^1+1, tomando el byte bajo del estado
// en cada direccion), sembrado con cfg_seed_value -- el mismo seed
// reproduce la MISMA secuencia de bytes, asi que la verificacion no
// necesita guardar los valores escritos en ningun lado: solo reinicia el
// LFSR al mismo seed al empezar cada pasada y recalcula.
//
// Tres fases (ver protocolo en top_sdram_freqtest.v):
//   Fase A ("solo lectura repetida"): escribe el rango UNA vez, despues
//     hace start_reps pasadas de lectura+verificacion sobre el mismo dato
//     -- mide estres de LECTURA repetida / retencion.
//   Fase B ("escritura+lectura repetida"): por cada repeticion, reescribe
//     TODO el rango y lo verifica antes de pasar a la siguiente -- mide
//     estres de escritura repetida ademas de lectura.
//   Fase C ("rafaga de 2 palabras", nueva -- ver sdram.v rd_burst2):
//     escribe el rango UNA vez (byte a byte, camino ya probado), despues
//     hace start_reps pasadas de lectura usando rd_burst2 (2 palabras de
//     32 bits por pedido, 8 bytes) en vez de una lectura de byte por vez.
//     addr_max DEBE ser multiplo de 8 y no cruzar el limite de fila (256
//     columnas = 1024 bytes) para que la rafaga sea valida -- ver
//     docs/caracterizacion-frecuencia-sdram.md.
// Internamente las tres comparten la misma secuencia de estados: una
// pasada de escritura (S_WR_*, siempre byte a byte, sin cambios) seguida
// de una pasada de lectura (S_RD_* para A/B, S_RDB_* para C).
//
// Interfaz SDRAM: mismo patron want_req/issue/busy que weight_stream.v
// (el arbitro del top-level concede el ciclo via 'issue', combinacional).
//
// MARGIN_CYCLES (ver docs/caracterizacion-frecuencia-sdram.md): 0 =
// comportamiento original de la campaña de frecuencia. No se usa en la
// fase C (rafaga).
module sdram_test_harness #(
    parameter integer MARGIN_CYCLES = 0
) (
    input  wire        clk,
    input  wire        rst,

    input  wire        cfg_addr_max_en,
    input  wire [15:0] cfg_addr_max_value,
    input  wire        cfg_seed_en,
    input  wire [31:0] cfg_seed_value,
    input  wire        start_en,
    input  wire [1:0]  start_phase,   // 0=A 1=B 2=C(rafaga de 2)
    input  wire [14:0] start_reps,

    output wire         want_req,
    output wire [22:0]  op_addr,
    output wire         op_is_write,
    output wire         op_is_burst2,  // 1 = usar rd_burst2 en vez de rd (solo fase C)
    output wire [7:0]   op_wdata,
    input  wire         issue,
    input  wire         sdram_busy,
    input  wire [7:0]   sdram_dout,
    input  wire [31:0]  sdram_dout32_a,
    input  wire [31:0]  sdram_dout32_b,

    output wire        busy,
    output wire [1:0]  cur_phase,
    output wire [15:0] cur_rep,
    output wire [31:0] counter_a,
    output wire [31:0] counter_b,
    output wire [31:0] reads_total_a,
    output wire [31:0] reads_total_b,

    input  wire [7:0]  log_rd_idx,
    output wire [22:0] log_rd_addr,
    output wire [7:0]  log_rd_expect,
    output wire [7:0]  log_rd_actual,
    output wire [15:0] log_rd_pass
);

    localparam [3:0] S_IDLE=4'd0, S_WR_ISSUE=4'd1, S_WR_WAIT=4'd2, S_WR_MARGIN=4'd3,
                      S_RD_ISSUE=4'd4, S_RD_WAIT=4'd5, S_RD_MARGIN=4'd6,
                      S_REP_DONE=4'd7, S_DONE=4'd8,
                      S_RDB_ISSUE=4'd9, S_RDB_WAIT=4'd10;
    reg [3:0] state = S_IDLE;
    reg [3:0] pending_next_state;
    reg [7:0] margin_ctr;

    reg [15:0] addr_max_r = 16'd0;
    reg [31:0] seed_r     = 32'd0;
    always @(posedge clk) begin
        if (cfg_addr_max_en) addr_max_r <= cfg_addr_max_value;
        if (cfg_seed_en)     seed_r     <= cfg_seed_value;
    end
    wire [31:0] safe_seed  = (seed_r == 32'd0) ? 32'hDEADBEEF : seed_r;
    wire [22:0] addr_max23 = {7'd0, addr_max_r};

    reg [22:0] addr_ctr = 23'd0;
    reg [31:0] lfsr      = 32'hDEADBEEF;
    reg [14:0] rep_ctr   = 15'd0;
    reg [14:0] reps_r    = 15'd0;
    reg [1:0]  phase_r   = 2'd0;
    reg        busy_seen = 1'b0;

    function [31:0] lfsr_step;
        input [31:0] v;
        begin
            lfsr_step = {v[30:0], v[31]^v[21]^v[1]^v[0]};
        end
    endfunction

    wire [31:0] lfsr_next = lfsr_step(lfsr);

    // ---- Fase C: los 8 estados sucesivos del LFSR a partir de 'lfsr'
    // (posicion actual, byte 0 del grupo) -- uno por byte del grupo de 8,
    // mas el estado que sigue (para arrancar el proximo grupo). ----
    wire [31:0] lfsr_g1 = lfsr_step(lfsr);
    wire [31:0] lfsr_g2 = lfsr_step(lfsr_g1);
    wire [31:0] lfsr_g3 = lfsr_step(lfsr_g2);
    wire [31:0] lfsr_g4 = lfsr_step(lfsr_g3);
    wire [31:0] lfsr_g5 = lfsr_step(lfsr_g4);
    wire [31:0] lfsr_g6 = lfsr_step(lfsr_g5);
    wire [31:0] lfsr_g7 = lfsr_step(lfsr_g6);
    wire [31:0] lfsr_g8 = lfsr_step(lfsr_g7);  // arranque del proximo grupo

    wire [7:0] exp_b0 = lfsr[7:0];
    wire [7:0] exp_b1 = lfsr_g1[7:0];
    wire [7:0] exp_b2 = lfsr_g2[7:0];
    wire [7:0] exp_b3 = lfsr_g3[7:0];
    wire [7:0] exp_b4 = lfsr_g4[7:0];
    wire [7:0] exp_b5 = lfsr_g5[7:0];
    wire [7:0] exp_b6 = lfsr_g6[7:0];
    wire [7:0] exp_b7 = lfsr_g7[7:0];

    wire [7:0] got_b0 = sdram_dout32_a[7:0];
    wire [7:0] got_b1 = sdram_dout32_a[15:8];
    wire [7:0] got_b2 = sdram_dout32_a[23:16];
    wire [7:0] got_b3 = sdram_dout32_a[31:24];
    wire [7:0] got_b4 = sdram_dout32_b[7:0];
    wire [7:0] got_b5 = sdram_dout32_b[15:8];
    wire [7:0] got_b6 = sdram_dout32_b[23:16];
    wire [7:0] got_b7 = sdram_dout32_b[31:24];

    wire [7:0] bad_mask = {(got_b7!=exp_b7), (got_b6!=exp_b6), (got_b5!=exp_b5), (got_b4!=exp_b4),
                            (got_b3!=exp_b3), (got_b2!=exp_b2), (got_b1!=exp_b1), (got_b0!=exp_b0)};
    // cantidad de bytes malos en el grupo (0-8) -- suma de bits de bad_mask
    wire [3:0] bad_count = bad_mask[0]+bad_mask[1]+bad_mask[2]+bad_mask[3]+
                            bad_mask[4]+bad_mask[5]+bad_mask[6]+bad_mask[7];
    // primer byte malo encontrado (para el log) -- prioridad al de menor indice
    reg [2:0] first_bad_idx;
    reg [7:0] first_bad_exp, first_bad_got;
    always @(*) begin
        if (bad_mask[0])      begin first_bad_idx=3'd0; first_bad_exp=exp_b0; first_bad_got=got_b0; end
        else if (bad_mask[1]) begin first_bad_idx=3'd1; first_bad_exp=exp_b1; first_bad_got=got_b1; end
        else if (bad_mask[2]) begin first_bad_idx=3'd2; first_bad_exp=exp_b2; first_bad_got=got_b2; end
        else if (bad_mask[3]) begin first_bad_idx=3'd3; first_bad_exp=exp_b3; first_bad_got=got_b3; end
        else if (bad_mask[4]) begin first_bad_idx=3'd4; first_bad_exp=exp_b4; first_bad_got=got_b4; end
        else if (bad_mask[5]) begin first_bad_idx=3'd5; first_bad_exp=exp_b5; first_bad_got=got_b5; end
        else if (bad_mask[6]) begin first_bad_idx=3'd6; first_bad_exp=exp_b6; first_bad_got=got_b6; end
        else                   begin first_bad_idx=3'd7; first_bad_exp=exp_b7; first_bad_got=got_b7; end
    end

    assign want_req     = (state == S_WR_ISSUE) || (state == S_RD_ISSUE) || (state == S_RDB_ISSUE);
    assign op_addr       = addr_ctr;
    assign op_is_write    = (state == S_WR_ISSUE);
    assign op_is_burst2   = (state == S_RDB_ISSUE);
    assign op_wdata       = lfsr[7:0];

    wire rd_complete  = (state == S_RD_WAIT)  && busy_seen && !sdram_busy;
    wire rdb_complete = (state == S_RDB_WAIT) && busy_seen && !sdram_busy;
    wire [22:0] group_size = 23'd8;
    wire at_range_end  = (addr_ctr == addr_max23 - 23'd1);
    wire at_range_end8 = (addr_ctr + group_size >= addr_max23);

    // ---- FSM: unico bloque que escribe state/addr_ctr/lfsr/rep_ctr/
    // reps_r/phase_r/busy_seen/pending_next_state/margin_ctr ----
    always @(posedge clk) begin
        case (state)
            S_IDLE: if (start_en) begin
                phase_r   <= start_phase;
                reps_r    <= start_reps;
                rep_ctr   <= 15'd0;
                addr_ctr  <= 23'd0;
                lfsr      <= safe_seed;
                busy_seen <= 1'b0;
                state     <= S_WR_ISSUE;
            end

            S_WR_ISSUE: if (issue) begin
                busy_seen <= 1'b0;
                state     <= S_WR_WAIT;
            end
            S_WR_WAIT: begin
                if (sdram_busy) busy_seen <= 1'b1;
                if (busy_seen && !sdram_busy) begin
                    busy_seen <= 1'b0;
                    lfsr      <= lfsr_next;
                    if (at_range_end) begin
                        addr_ctr           <= 23'd0;
                        lfsr               <= safe_seed;   // reinicia la secuencia para la lectura
                        pending_next_state <= (phase_r == 2'd2) ? S_RDB_ISSUE : S_RD_ISSUE;
                    end else begin
                        addr_ctr           <= addr_ctr + 23'd1;
                        pending_next_state <= S_WR_ISSUE;
                    end
                    if (MARGIN_CYCLES == 0) begin
                        state <= at_range_end ? ((phase_r == 2'd2) ? S_RDB_ISSUE : S_RD_ISSUE) : S_WR_ISSUE;
                    end else begin
                        margin_ctr <= MARGIN_CYCLES[7:0] - 8'd1;
                        state      <= S_WR_MARGIN;
                    end
                end
            end
            S_WR_MARGIN: begin
                if (margin_ctr == 8'd0) state <= pending_next_state;
                else margin_ctr <= margin_ctr - 8'd1;
            end

            S_RD_ISSUE: if (issue) begin
                busy_seen <= 1'b0;
                state     <= S_RD_WAIT;
            end
            S_RD_WAIT: begin
                if (sdram_busy) busy_seen <= 1'b1;
                if (rd_complete) begin
                    busy_seen <= 1'b0;
                    lfsr      <= lfsr_next;
                    if (at_range_end) begin
                        addr_ctr           <= 23'd0;
                        pending_next_state <= S_REP_DONE;
                    end else begin
                        addr_ctr           <= addr_ctr + 23'd1;
                        pending_next_state <= S_RD_ISSUE;
                    end
                    if (MARGIN_CYCLES == 0) begin
                        state <= at_range_end ? S_REP_DONE : S_RD_ISSUE;
                    end else begin
                        margin_ctr <= MARGIN_CYCLES[7:0] - 8'd1;
                        state      <= S_RD_MARGIN;
                    end
                end
            end
            S_RD_MARGIN: begin
                if (margin_ctr == 8'd0) state <= pending_next_state;
                else margin_ctr <= margin_ctr - 8'd1;
            end

            // ---- Fase C: lectura por rafaga de 2 palabras (8 bytes) ----
            S_RDB_ISSUE: if (issue) begin
                busy_seen <= 1'b0;
                state     <= S_RDB_WAIT;
            end
            S_RDB_WAIT: begin
                if (sdram_busy) busy_seen <= 1'b1;
                if (rdb_complete) begin
                    busy_seen <= 1'b0;
                    lfsr      <= lfsr_g8;
                    if (at_range_end8) begin
                        addr_ctr <= 23'd0;
                        state    <= S_REP_DONE;
                    end else begin
                        addr_ctr <= addr_ctr + group_size;
                        state    <= S_RDB_ISSUE;
                    end
                end
            end

            S_REP_DONE: if (rep_ctr == reps_r - 15'd1) begin
                state <= S_DONE;
            end else begin
                rep_ctr <= rep_ctr + 15'd1;
                lfsr    <= safe_seed;
                // B rescribe, A y C no (C solo relee con burst2)
                state   <= (phase_r == 2'd1) ? S_WR_ISSUE :
                           (phase_r == 2'd2) ? S_RDB_ISSUE : S_RD_ISSUE;
            end

            S_DONE: state <= S_IDLE;

            default: state <= S_IDLE;
        endcase

        if (rst) begin
            state     <= S_IDLE;
            busy_seen <= 1'b0;
        end
    end

    // ---- Canalizacion de la verificacion (1 ciclo de latencia extra) --
    // EXPERIMENTO DE DEPURACION: al agregar restricciones reales de I/O
    // para la SDRAM (ver sdram_freqtest.sdc) aparecieron ~9ns de violacion
    // de setup en el camino pad->comparacion LFSR->contador de 32 bits,
    // TODAS con destino en la logica de este arnes (nunca en el registro
    // de captura de sdram.v en si) -- o sea, la comparacion+conteo en el
    // MISMO ciclo que llega el dato del pad era demasiado larga a 54MHz
    // con un retardo de entrada realista. Se separa en 2 pasos: capturar
    // el dato crudo (mismo costo que la captura de sdram.v, ya probada) en
    // el ciclo que completa la operacion, y hacer la comparacion/
    // codificacion/incremento del contador un ciclo despues, con datos ya
    // estables desde el arranque del ciclo.
    reg        chk_valid    = 1'b0;
    reg        chk_is_burst = 1'b0;
    reg [7:0]  chk_dout;
    reg [31:0] chk_dout32_a, chk_dout32_b;
    reg [31:0] chk_lfsr;
    reg [22:0] chk_addr;
    reg [14:0] chk_rep;
    reg        chk_phase_is_b;

    always @(posedge clk) begin
        chk_valid <= rd_complete || rdb_complete;
        if (rd_complete || rdb_complete) begin
            chk_is_burst   <= rdb_complete;
            chk_dout       <= sdram_dout;
            chk_dout32_a   <= sdram_dout32_a;
            chk_dout32_b   <= sdram_dout32_b;
            chk_lfsr       <= lfsr;
            chk_addr       <= addr_ctr;
            chk_rep        <= rep_ctr;
            chk_phase_is_b <= (phase_r == 2'd1);
        end
    end

    // Mismos 8 pasos del LFSR que exp_b0..b7/lfsr_g1..g8 mas arriba, pero
    // arrancando de chk_lfsr (el valor YA REGISTRADO, no el en vivo) --
    // asi esta comparacion no depende de nada que haya llegado este mismo
    // ciclo desde el pad.
    wire [31:0] chk_g1 = lfsr_step(chk_lfsr);
    wire [31:0] chk_g2 = lfsr_step(chk_g1);
    wire [31:0] chk_g3 = lfsr_step(chk_g2);
    wire [31:0] chk_g4 = lfsr_step(chk_g3);
    wire [31:0] chk_g5 = lfsr_step(chk_g4);
    wire [31:0] chk_g6 = lfsr_step(chk_g5);
    wire [31:0] chk_g7 = lfsr_step(chk_g6);

    wire [7:0] chk_exp_b0 = chk_lfsr[7:0];
    wire [7:0] chk_exp_b1 = chk_g1[7:0];
    wire [7:0] chk_exp_b2 = chk_g2[7:0];
    wire [7:0] chk_exp_b3 = chk_g3[7:0];
    wire [7:0] chk_exp_b4 = chk_g4[7:0];
    wire [7:0] chk_exp_b5 = chk_g5[7:0];
    wire [7:0] chk_exp_b6 = chk_g6[7:0];
    wire [7:0] chk_exp_b7 = chk_g7[7:0];

    wire [7:0] chk_got_b0 = chk_dout32_a[7:0];
    wire [7:0] chk_got_b1 = chk_dout32_a[15:8];
    wire [7:0] chk_got_b2 = chk_dout32_a[23:16];
    wire [7:0] chk_got_b3 = chk_dout32_a[31:24];
    wire [7:0] chk_got_b4 = chk_dout32_b[7:0];
    wire [7:0] chk_got_b5 = chk_dout32_b[15:8];
    wire [7:0] chk_got_b6 = chk_dout32_b[23:16];
    wire [7:0] chk_got_b7 = chk_dout32_b[31:24];

    wire [7:0] chk_bad_mask = {(chk_got_b7!=chk_exp_b7), (chk_got_b6!=chk_exp_b6),
                                (chk_got_b5!=chk_exp_b5), (chk_got_b4!=chk_exp_b4),
                                (chk_got_b3!=chk_exp_b3), (chk_got_b2!=chk_exp_b2),
                                (chk_got_b1!=chk_exp_b1), (chk_got_b0!=chk_exp_b0)};
    wire [3:0] chk_bad_count = chk_bad_mask[0]+chk_bad_mask[1]+chk_bad_mask[2]+chk_bad_mask[3]+
                                chk_bad_mask[4]+chk_bad_mask[5]+chk_bad_mask[6]+chk_bad_mask[7];
    reg [2:0] chk_first_bad_idx;
    reg [7:0] chk_first_bad_exp, chk_first_bad_got;
    always @(*) begin
        if (chk_bad_mask[0])      begin chk_first_bad_idx=3'd0; chk_first_bad_exp=chk_exp_b0; chk_first_bad_got=chk_got_b0; end
        else if (chk_bad_mask[1]) begin chk_first_bad_idx=3'd1; chk_first_bad_exp=chk_exp_b1; chk_first_bad_got=chk_got_b1; end
        else if (chk_bad_mask[2]) begin chk_first_bad_idx=3'd2; chk_first_bad_exp=chk_exp_b2; chk_first_bad_got=chk_got_b2; end
        else if (chk_bad_mask[3]) begin chk_first_bad_idx=3'd3; chk_first_bad_exp=chk_exp_b3; chk_first_bad_got=chk_got_b3; end
        else if (chk_bad_mask[4]) begin chk_first_bad_idx=3'd4; chk_first_bad_exp=chk_exp_b4; chk_first_bad_got=chk_got_b4; end
        else if (chk_bad_mask[5]) begin chk_first_bad_idx=3'd5; chk_first_bad_exp=chk_exp_b5; chk_first_bad_got=chk_got_b5; end
        else if (chk_bad_mask[6]) begin chk_first_bad_idx=3'd6; chk_first_bad_exp=chk_exp_b6; chk_first_bad_got=chk_got_b6; end
        else                       begin chk_first_bad_idx=3'd7; chk_first_bad_exp=chk_exp_b7; chk_first_bad_got=chk_got_b7; end
    end

    wire chk_mismatch = chk_valid && !chk_is_burst && (chk_dout != chk_lfsr[7:0]);

    // ---- Contadores de resultados + log de fallas: unico bloque que los
    // escribe. Se reinician al arrancar una fase nueva (start_en), no se
    // acumulan entre corridas distintas. Fase C reusa counter_a/
    // reads_total_a (nunca corren A y C en la misma sesion). ----
    reg [31:0] counter_a_r = 32'd0, counter_b_r = 32'd0;
    reg [31:0] reads_total_a_r = 32'd0, reads_total_b_r = 32'd0;
    reg [63:0] fail_log_mem [0:255];
    reg [7:0]  fail_log_wptr = 8'd0;

    always @(posedge clk) begin
        if (start_en) begin
            if (start_phase == 2'd1) begin
                counter_b_r     <= 32'd0;
                reads_total_b_r <= 32'd0;
            end else begin
                counter_a_r     <= 32'd0;
                reads_total_a_r <= 32'd0;
            end
        end else if (chk_valid && !chk_is_burst) begin
            if (!chk_phase_is_b) reads_total_a_r <= reads_total_a_r + 32'd1;
            else                  reads_total_b_r <= reads_total_b_r + 32'd1;
            if (chk_mismatch) begin
                if (!chk_phase_is_b) counter_a_r <= counter_a_r + 32'd1;
                else                  counter_b_r <= counter_b_r + 32'd1;
                fail_log_mem[fail_log_wptr] <= {9'd0, {1'b0, chk_rep}, chk_dout, chk_lfsr[7:0], chk_addr};
                fail_log_wptr <= fail_log_wptr + 8'd1;
            end
        end else if (chk_valid && chk_is_burst) begin
            reads_total_a_r <= reads_total_a_r + {28'd0, 4'd8};  // 8 bytes por grupo
            if (chk_bad_count != 4'd0) begin
                counter_a_r <= counter_a_r + {28'd0, chk_bad_count};
                fail_log_mem[fail_log_wptr] <= {9'd0, {1'b0, chk_rep}, chk_first_bad_got, chk_first_bad_exp,
                                                 chk_addr + {20'd0, chk_first_bad_idx}};
                fail_log_wptr <= fail_log_wptr + 8'd1;
            end
        end
    end

    assign busy          = (state != S_IDLE);
    assign cur_phase     = phase_r;
    assign cur_rep       = {1'b0, rep_ctr};
    assign counter_a     = counter_a_r;
    assign counter_b     = counter_b_r;
    assign reads_total_a = reads_total_a_r;
    assign reads_total_b = reads_total_b_r;

    // ---- Lectura del log: BRAM de lectura sincrona (1 ciclo de latencia,
    // insignificante frente al ritmo de una trama SPI) ----
    reg [63:0] log_rd_data;
    always @(posedge clk) log_rd_data <= fail_log_mem[log_rd_idx];
    assign log_rd_addr   = log_rd_data[22:0];
    assign log_rd_expect = log_rd_data[30:23];
    assign log_rd_actual = log_rd_data[38:31];
    assign log_rd_pass   = log_rd_data[54:39];

endmodule
