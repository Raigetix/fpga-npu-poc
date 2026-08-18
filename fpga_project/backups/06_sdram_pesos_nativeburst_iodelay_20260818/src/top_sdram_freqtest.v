// top_sdram_freqtest.v -- caracterizacion de frecuencia maxima segura de
// la SDRAM embebida (ver plan_testing_sdram.md). Standalone: no toca
// mlp_engine_par_stream.v ni weight_stream.v, reusa sdram.v y spi_slave.v
// tal cual.
//
// La frecuencia queda FIJA por bitstream (parametros del modulo, ver
// pll_sdram_static.v para el porque -- el chip solo tiene 2 PLLs fisicos,
// no alcanzan para 4 simultaneos + conmutacion en vivo). Los 4 candidatos
// se generan como 4 top-levels finitos (top_freqtest_27/36/45/54.v) que
// solo fijan los parametros de PLL e instancian este modulo.
//
// Protocolo (trama de 48 bits, mismo patron que top_sdram_p3.v):
//   CMD_NOP           = 0x00
//   CMD_TEST_CONFIG   = 0x11  A[15:0] = addr_max (cantidad de bytes del
//                                rango de prueba, empieza en direccion 0)
//   CMD_SET_SEED      = 0x12  valor de 32 bits (A+B) = semilla LFSR
//   CMD_TEST_START    = 0x13  A[1:0]=fase (0=A escribe una vez+lee N veces,
//                                1=B escribe+lee N veces, 2=C escribe una
//                                vez+lee N veces con rd_burst2, SIN
//                                verificacion post-refresco a proposito --
//                                addr_max debe ser multiplo de 8 y no pasar
//                                de 1024 para no cruzar fila) A[15:2]=repeticiones
//   CMD_TEST_RD       = 0x14  A[15:11]=sel(0=status 1=summary) A[10:0]=indice
//                                de byte, lectura secuencial de campos
//                                empaquetados (ver mas abajo), un byte por
//                                trama, en el byte 5 de la respuesta
//   CMD_TEST_LOG_RD   = 0x15  A[10:0]=indice de entrada del log (0-255)
//                                B[1:0]=sub-campo (0=addr[7:0] 1=addr[15:8]
//                                2={5'd0,addr[22:16]} 3=expected,
//                                4=actual 5=pasada[7:0] 6=pasada[15:8]),
//                                un byte por trama en el byte 5
//
// sel=0 (status, 4 bytes): [0]={6'd0,fase,busy} [1:2]=repeticion_actual
//   (16 bits, lo primero) [3]=0
// sel=1 (summary, 16 bytes): [0:3]=contador_A [4:7]=contador_B
//   [8:11]=lecturas_totales_A [12:15]=lecturas_totales_B (todos 32 bits,
//   byte menos significativo primero)
//
// MISO: [status: bit0=sdram_busy bit1=pll_lock bit2=test_busy] [xxxx]
//       [byte de debug de la ULTIMA lectura CMD_TEST_RD/CMD_TEST_LOG_RD]
module top_sdram_freqtest #(
    parameter IDIV_SEL  = 2,
    parameter FBDIV_SEL = 5,
    parameter ODIV_SEL  = 16,
    // Ciclos de margen extra entre operaciones consecutivas del arnes de
    // prueba (ver sdram_test_harness.v) -- 0 = comportamiento original de
    // la campaña de frecuencia (ver docs/caracterizacion-frecuencia-sdram.md).
    parameter integer MARGIN_CYCLES = 0
) (
    input  wire       clk,   // 27 MHz onboard, pin 4
    input  wire       sclk,
    input  wire       cs_n,
    input  wire       mosi,
    output wire       miso,
    output wire [5:0] led,

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

    localparam [7:0] CMD_NOP         = 8'h00;
    localparam [7:0] CMD_TEST_CONFIG = 8'h11;
    localparam [7:0] CMD_SET_SEED    = 8'h12;
    localparam [7:0] CMD_TEST_START  = 8'h13;
    localparam [7:0] CMD_TEST_RD     = 8'h14;
    localparam [7:0] CMD_TEST_LOG_RD = 8'h15;

    wire clk_sys;
    wire clk_sdram;
    wire pll_lock;
    pll_sdram_static #(.IDIV_SEL(IDIV_SEL), .FBDIV_SEL(FBDIV_SEL), .ODIV_SEL(ODIV_SEL)) u_pll (
        .clk_in   (clk),
        .clk_out  (clk_sys),
        .clk_sdram(clk_sdram),
        .lock     (pll_lock)
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
    wire [31:0] wide_value = {addr_field, data_field};

    wire cmd_ok = frame_done_d1 && !test_busy;

    // ================= SDRAM =================
    wire        sdram_busy;
    wire [31:0] sdram_dout32;
    reg         sdram_resetn = 1'b0;
    always @(posedge clk_sys) sdram_resetn <= pll_lock;

    wire        want_test_op;
    wire [22:0] test_op_addr;
    wire        test_op_is_write;
    wire        test_op_is_burst2;
    wire [7:0]  test_op_wdata;
    wire [7:0]  sdram_dout;
    wire [31:0] sdram_dout32_a, sdram_dout32_b;

    // Techo real: la SDRAM necesita >=4096 refrescos cada 64ms (~15.6us
    // maximo entre refrescos, ver sdram.v), que a 54MHz son ~844 ciclos.
    // Subido de 700 a 800 (margen de ~5% por debajo del techo, mas el
    // tiempo que puede demorar el arbitro en conceder el refresco si
    // justo hay una verificacion post-refresco en curso, ver mas abajo).
    localparam REFRESH_INTERVAL = 800;
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

    // ---- Verificacion por doble/triple lectura, SOLO para la lectura
    // que sigue justo despues de un refresco (ver
    // docs/caracterizacion-frecuencia-sdram.md -- ahi cae ~97% de las
    // fallas). Mismo algoritmo ya probado en weight_stream.v (produccion):
    // leer dos veces, si coinciden listo, si no leer una tercera y votar
    // por mayoria -- sin margen de espera, solo relectura. El resto de
    // las operaciones (~87 de cada 88) siguen a una sola pasada, rapidas,
    // sin este costo extra. No toca sdram.v (ver los 2 intentos previos
    // que fallaron por tocar el controlador o por dejar que un refresco
    // nuevo interrumpiera la verificacion a mitad de camino -- aca eso
    // esta bloqueado explicitamente, ver want_refresh mas abajo). ----
    reg refresh_owed = 1'b0;   // hubo un refresco concedido, todavia no
                                // "gastado" en la operacion de prueba siguiente

    // EXPERIMENTO DE DEPURACION -- con restricciones reales de I/O para la
    // SDRAM (ver sdram_freqtest.sdc), aparecio ~6ns de violacion de setup
    // en V_WAIT2/V_WAIT3: comparaban 'sdram_dout' (recien llegado del pad,
    // con retardo de entrada real) contra v_val1/v_val2 EN EL MISMO ciclo
    // que decidian el proximo estado -- mismo patron ya arreglado en
    // sdram_test_harness.v. Se agregan V_CMP2/V_CMP3: capturar el dato
    // crudo en V_WAIT2/V_WAIT3 (sin comparar todavia), comparar recien en
    // el ciclo siguiente con datos ya estables desde el arranque del
    // ciclo. Un ciclo mas de latencia en la verificacion (rara, solo tras
    // un refresco), sin cambiar la logica de mayoria en si.
    localparam [3:0] V_IDLE=4'd0, V_WAIT1=4'd1, V_ISSUE2=4'd2, V_WAIT2=4'd3,
                      V_CMP2=4'd4, V_ISSUE3=4'd5, V_WAIT3=4'd6, V_CMP3=4'd7,
                      V_DONE=4'd8;
    reg [3:0]  v_state = V_IDLE;
    reg        v_busy_seen = 1'b0;
    reg [7:0]  v_val1, v_val2, v_val3, v_final;
    reg [22:0] v_addr;
    reg        v_active = 1'b0;   // hay una verificacion post-refresco en curso

    wire want_test_grant = !sdram_busy && !v_active && want_test_op;
    wire want_refresh    = !sdram_busy && !v_active && !want_test_grant && refresh_pending;

    reg        sdram_rd_pulse, sdram_wr_pulse, sdram_refresh_pulse, sdram_rd_burst2_pulse;
    reg [22:0] sdram_op_addr;
    reg [7:0]  sdram_op_data;
    always @(posedge clk_sys) begin
        sdram_rd_pulse         <= 1'b0;
        sdram_wr_pulse         <= 1'b0;
        sdram_refresh_pulse    <= 1'b0;
        sdram_rd_burst2_pulse  <= 1'b0;

        if (v_state == V_IDLE) begin
            if (want_test_grant) begin
                sdram_op_addr <= test_op_addr;
                if (test_op_is_write) begin
                    sdram_op_data  <= test_op_wdata;
                    sdram_wr_pulse <= 1'b1;
                end else if (test_op_is_burst2) begin
                    // fase C (validacion aislada de rd_burst2): sin la
                    // verificacion post-refresco a proposito, para medir el
                    // comportamiento CRUDO de la rafaga (igual que se hizo
                    // primero con la lectura de a un byte, antes de agregar
                    // la verificacion condicional).
                    sdram_rd_burst2_pulse <= 1'b1;
                end else begin
                    sdram_rd_pulse <= 1'b1;
                    if (refresh_owed) begin
                        v_addr      <= test_op_addr;
                        v_busy_seen <= 1'b0;
                        v_active    <= 1'b1;
                        v_state     <= V_WAIT1;
                    end
                end
                refresh_owed <= 1'b0;   // se gasta al arrancar CUALQUIER operacion
            end else if (want_refresh) begin
                sdram_refresh_pulse <= 1'b1;
                refresh_owed        <= 1'b1;
            end
        end else begin
            case (v_state)
                V_WAIT1: begin
                    if (sdram_busy) v_busy_seen <= 1'b1;
                    if (v_busy_seen && !sdram_busy) begin
                        v_val1      <= sdram_dout;
                        v_busy_seen <= 1'b0;
                        v_state     <= V_ISSUE2;
                    end
                end
                V_ISSUE2: begin
                    sdram_op_addr  <= v_addr;
                    sdram_rd_pulse <= 1'b1;
                    v_state        <= V_WAIT2;
                end
                V_WAIT2: begin
                    if (sdram_busy) v_busy_seen <= 1'b1;
                    if (v_busy_seen && !sdram_busy) begin
                        v_val2      <= sdram_dout;
                        v_busy_seen <= 1'b0;
                        v_state     <= V_CMP2;
                    end
                end
                V_CMP2: begin
                    if (v_val2 == v_val1) begin
                        v_final <= v_val1;
                        v_state <= V_DONE;
                    end else begin
                        v_state <= V_ISSUE3;
                    end
                end
                V_ISSUE3: begin
                    sdram_op_addr  <= v_addr;
                    sdram_rd_pulse <= 1'b1;
                    v_state        <= V_WAIT3;
                end
                V_WAIT3: begin
                    if (sdram_busy) v_busy_seen <= 1'b1;
                    if (v_busy_seen && !sdram_busy) begin
                        v_val3      <= sdram_dout;
                        v_busy_seen <= 1'b0;
                        v_state     <= V_CMP3;
                    end
                end
                V_CMP3: begin
                    if (v_val3 == v_val1)      v_final <= v_val1;
                    else if (v_val3 == v_val2) v_final <= v_val2;
                    else                        v_final <= v_val3;
                    v_state <= V_DONE;
                end
                V_DONE: begin
                    v_active <= 1'b0;
                    v_state  <= V_IDLE;
                end
                default: v_state <= V_IDLE;
            endcase
        end
    end

    // Lo que ve el arnes de prueba: 'busy' se mantiene en alto durante
    // TODA la verificacion (no solo la primera lectura), y 'dout' entrega
    // el valor ya votado recien al terminar -- el arnes no se entera de
    // si hubo 1, 2 o 3 lecturas reales por debajo.
    wire       harness_sdram_busy = sdram_busy || v_active;
    wire [7:0] harness_sdram_dout = (v_state == V_DONE) ? v_final : sdram_dout;

    sdram #(.FREQ(54_000_000), .T_RP(4'd2), .T_RCD(4'd2)) u_sdram (
        .clk       (clk_sys),
        .clk_sdram (clk_sdram),
        .resetn    (sdram_resetn),
        .rd        (sdram_rd_pulse),
        .wr        (sdram_wr_pulse),
        .refresh   (sdram_refresh_pulse),
        .rd_burst2 (sdram_rd_burst2_pulse),
        .addr      (sdram_op_addr),
        .din       (sdram_op_data),
        .dout      (sdram_dout),
        .dout32    (sdram_dout32),
        .dout32_a  (sdram_dout32_a),
        .dout32_b  (sdram_dout32_b),
        .data_ready(),
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

    // ================= Configuracion del arnes de prueba =================
    reg  cfg_addr_max_en = 1'b0;
    reg  cfg_seed_en     = 1'b0;
    reg  start_en        = 1'b0;
    reg  [15:0] cfg_addr_max_value;
    reg  [31:0] cfg_seed_value;
    reg  [1:0]  start_phase;
    reg  [14:0] start_reps;

    always @(posedge clk_sys) begin
        cfg_addr_max_en <= 1'b0;
        cfg_seed_en     <= 1'b0;
        start_en        <= 1'b0;
        if (cmd_ok) begin
            case (cmd)
                CMD_TEST_CONFIG: begin
                    cfg_addr_max_value <= addr_field;
                    cfg_addr_max_en    <= 1'b1;
                end
                CMD_SET_SEED: begin
                    cfg_seed_value <= wide_value;
                    cfg_seed_en    <= 1'b1;
                end
                CMD_TEST_START: begin
                    // fase ahora ocupa 2 bits (0=A 1=B 2=C-rafaga) --
                    // repeticiones corridas 1 bit (14 bits, hasta 16383,
                    // de sobra).
                    start_phase <= addr_field[1:0];
                    start_reps  <= {1'b0, addr_field[15:2]};
                    start_en    <= 1'b1;
                end
                default: ;
            endcase
        end
    end

    wire        test_busy;
    wire [1:0]  test_cur_phase;
    wire [15:0] test_cur_rep;
    wire [31:0] test_counter_a, test_counter_b;
    wire [31:0] test_reads_total_a, test_reads_total_b;

    reg  [7:0]  log_rd_idx;
    wire [22:0] log_rd_addr;
    wire [7:0]  log_rd_expect, log_rd_actual;
    wire [15:0] log_rd_pass;

    sdram_test_harness #(.MARGIN_CYCLES(MARGIN_CYCLES)) u_harness (
        .clk(clk_sys),
        .rst(1'b0),

        .cfg_addr_max_en   (cfg_addr_max_en),
        .cfg_addr_max_value(cfg_addr_max_value),
        .cfg_seed_en       (cfg_seed_en),
        .cfg_seed_value    (cfg_seed_value),
        .start_en          (start_en),
        .start_phase       (start_phase),
        .start_reps        (start_reps),

        .want_req     (want_test_op),
        .op_addr      (test_op_addr),
        .op_is_write  (test_op_is_write),
        .op_is_burst2 (test_op_is_burst2),
        .op_wdata     (test_op_wdata),
        .issue        (want_test_grant),
        .sdram_busy   (harness_sdram_busy),
        .sdram_dout   (harness_sdram_dout),
        .sdram_dout32_a(sdram_dout32_a),
        .sdram_dout32_b(sdram_dout32_b),

        .busy         (test_busy),
        .cur_phase    (test_cur_phase),
        .cur_rep      (test_cur_rep),
        .counter_a    (test_counter_a),
        .counter_b    (test_counter_b),
        .reads_total_a(test_reads_total_a),
        .reads_total_b(test_reads_total_b),

        .log_rd_idx   (log_rd_idx),
        .log_rd_addr  (log_rd_addr),
        .log_rd_expect(log_rd_expect),
        .log_rd_actual(log_rd_actual),
        .log_rd_pass  (log_rd_pass)
    );

    // ---- Lectura de status/summary/log: un byte por trama, seleccionado
    // por el campo de indice de la propia trama (sin puntero con memoria,
    // asi el host puede pedir cualquier byte en cualquier orden) ----
    reg [7:0] test_rd_byte;
    always @(*) begin
        log_rd_idx = addr_field[7:0];
        case (cmd)
            CMD_TEST_RD: begin
                case (addr_field[15:11])
                    5'd0: // status
                        case (addr_field[10:0])
                            11'd0: test_rd_byte = {5'd0, test_cur_phase, test_busy};
                            11'd1: test_rd_byte = test_cur_rep[7:0];
                            11'd2: test_rd_byte = test_cur_rep[15:8];
                            default: test_rd_byte = 8'd0;
                        endcase
                    5'd1: // summary
                        case (addr_field[10:0])
                            11'd0:  test_rd_byte = test_counter_a[7:0];
                            11'd1:  test_rd_byte = test_counter_a[15:8];
                            11'd2:  test_rd_byte = test_counter_a[23:16];
                            11'd3:  test_rd_byte = test_counter_a[31:24];
                            11'd4:  test_rd_byte = test_counter_b[7:0];
                            11'd5:  test_rd_byte = test_counter_b[15:8];
                            11'd6:  test_rd_byte = test_counter_b[23:16];
                            11'd7:  test_rd_byte = test_counter_b[31:24];
                            11'd8:  test_rd_byte = test_reads_total_a[7:0];
                            11'd9:  test_rd_byte = test_reads_total_a[15:8];
                            11'd10: test_rd_byte = test_reads_total_a[23:16];
                            11'd11: test_rd_byte = test_reads_total_a[31:24];
                            11'd12: test_rd_byte = test_reads_total_b[7:0];
                            11'd13: test_rd_byte = test_reads_total_b[15:8];
                            11'd14: test_rd_byte = test_reads_total_b[23:16];
                            11'd15: test_rd_byte = test_reads_total_b[31:24];
                            default: test_rd_byte = 8'd0;
                        endcase
                    default: test_rd_byte = 8'd0;
                endcase
            end
            CMD_TEST_LOG_RD: begin
                log_rd_idx = addr_field[7:0];
                case (data_field[2:0])
                    3'd0: test_rd_byte = log_rd_addr[7:0];
                    3'd1: test_rd_byte = log_rd_addr[15:8];
                    3'd2: test_rd_byte = {5'd0, log_rd_addr[22:16]};
                    3'd3: test_rd_byte = log_rd_expect;
                    3'd4: test_rd_byte = log_rd_actual;
                    3'd5: test_rd_byte = log_rd_pass[7:0];
                    3'd6: test_rd_byte = log_rd_pass[15:8];
                    default: test_rd_byte = 8'd0;
                endcase
            end
            default: test_rd_byte = 8'd0;
        endcase
    end

    // ---- snapshot para MISO, mismo patron de "congelar mientras la trama
    // esta en curso" que el resto del proyecto ----
    reg cs_sync0 = 1'b1, cs_sync1 = 1'b1, cs_sync2 = 1'b1, cs_sync3 = 1'b1;
    always @(posedge clk_sys) begin
        cs_sync0 <= cs_n; cs_sync1 <= cs_sync0; cs_sync2 <= cs_sync1; cs_sync3 <= cs_sync2;
    end
    // OJO -- bug encontrado en hardware real: esta concatenacion sumaba
    // 40 bits (5+1+1+1+24+8), no los 48 de tx_snapshot -- Verilog la
    // extiende con ceros del lado del MSB al asignarla a un reg mas ancho,
    // asi que el byte0 completo (bits[47:40], de donde sale rx[0] del lado
    // del ESP32, MSB primero) quedaba siempre en 0. Se leia "PLL no
    // engancho" (bit1 de rx[0] siempre 0) con la FPGA andando perfecto --
    // era el ancho del paquete, no el PLL. Corregido: 32'd0 en vez de
    // 24'd0 para volver a completar 48 bits (8+32+8).
    always @(posedge clk_sys) begin
        if (cs_sync3)
            tx_snapshot <= {5'd0, test_busy, pll_lock, sdram_busy, 32'd0, test_rd_byte};
    end

    assign led = ~{test_busy, pll_lock, 4'b0000};

endmodule
