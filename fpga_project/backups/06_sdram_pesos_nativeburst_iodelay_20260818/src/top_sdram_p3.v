// top_sdram_p3.v -- Etapa 4, Fase 3+: NPU configurable por SPI de punta a
// punta -- cantidad de capas, ancho de cada capa, pesos (via SDRAM
// streaming, weight_stream.v), bias y parametros de cuantizacion, todo
// cargado en caliente sin recompilar. Ver mlp_engine_par_stream.v para el
// motor de computo generico y los limites (MAX_LAYERS/MAX_INPUT_WIDTH/
// MAX_LAYER_WIDTH) fijados una unica vez en la sintesis.
//
// Reutiliza sin cambios: sdram.v (controlador), pll_sdram.v (54MHz, fase
// corregida -- ver README.md seccion Etapa 4), spi_slave.v. El arbitro de
// SDRAM (SPI directo > streaming de pesos > refresco) sigue el MISMO patron
// ya probado en Fase 1/2: un unico bloque que escribe sdram_rd_pulse/
// sdram_wr_pulse/sdram_refresh_pulse/sdram_op_addr/sdram_op_data, todo lo
// demas solo LEE señales combinacionales que exponen "quiero leer" (nunca
// escriben esas señales, para no repetir el error de "dos drivers" ya
// encontrado y corregido varias veces esta sesion).
//
// Protocolo (mismo frame de 48 bits de siempre):
//   CMD_NOP               = 0x00
//   CMD_SDRAM_SET_ADDR    = 0x01  A[15:0]=addr[15:0] B[6:0]=addr[22:16]
//                                   (puntero de escritura directa a SDRAM,
//                                   para cargar los pesos ANTES de computar)
//   CMD_SDRAM_WR          = 0x02  B_lo=byte a escribir, puntero avanza en 1
//   CMD_WSTREAM_SET_BASE  = 0x03  A[15:0]=base[15:0] B[6:0]=base[22:16]
//                                   (direccion en SDRAM donde arrancan los
//                                   pesos para el streaming -- layout: para
//                                   el paso local a y carril l, direccion =
//                                   base + a*8 + l; ver weight_stream.v)
//   CMD_SET_NUM_LAYERS    = 0x04  A[3:0]=cantidad de capas (1..8)
//   CMD_SET_LAYER_SHAPE   = 0x05  A[2:0]=capa A[5:4]=parametro(0=entradas
//                                   1=salidas 2=direccion base de bias)
//                                   B[15:0]=valor
//   CMD_SET_BIAS_ADDR     = 0x06  A[15:0]=direccion (puntero de escritura
//                                   de bias, 0..2047)
//   CMD_BIAS_WR           = 0x07  valor de 32 bits (bias EFECTIVO, ya con
//                                   la correccion del zero-point de entrada
//                                   aplicada del lado del host -- ver
//                                   README.md seccion Etapa 4) en los bytes
//                                   1-4; el puntero avanza en 1 despues
//   CMD_SET_QPARAM        = 0x08  valor de 32 bits en los bytes 1-4, byte 5
//                                   = {2'b00, capa[2:0], parametro[1:0]}.
//                                   parametro: 0=multiplicador (entero =
//                                   round(escala_real*2^QSHIFT), ver
//                                   mlp_engine_par_stream.v) 1=zero_point de
//                                   salida 2=minimo de activacion 3=maximo
//                                   de activacion
//   CMD_START             = 0x09
//   CMD_DBG_RD            = 0x0A  A[13:11]=sel(0=actbuf_a 1=actbuf_b
//                                   3=start_count 4=input_mem 5=out_mem
//                                   6=bias_mem) A[10:0]=indice
//   CMD_SET_ITGT          = 0x0B  A[10:0]=direccion base de entrada
//   CMD_IBURST5           = 0x0C  5 bytes de payload, escribe input_mem y
//                                   avanza el puntero en 5
//
//   MISO: [status: bit0=busy bit1=pll_lock] [xxxx (reservado)]
//         [byte de debug de la ULTIMA lectura CMD_DBG_RD]
module top_sdram_p3 (
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

    localparam [7:0] CMD_NOP              = 8'h00;
    localparam [7:0] CMD_SDRAM_SET_ADDR   = 8'h01;
    localparam [7:0] CMD_SDRAM_WR         = 8'h02;
    localparam [7:0] CMD_WSTREAM_SET_BASE = 8'h03;
    localparam [7:0] CMD_SET_NUM_LAYERS   = 8'h04;
    localparam [7:0] CMD_SET_LAYER_SHAPE  = 8'h05;
    localparam [7:0] CMD_SET_BIAS_ADDR    = 8'h06;
    localparam [7:0] CMD_BIAS_WR          = 8'h07;
    localparam [7:0] CMD_SET_QPARAM       = 8'h08;
    localparam [7:0] CMD_START            = 8'h09;
    localparam [7:0] CMD_DBG_RD           = 8'h0A;
    localparam [7:0] CMD_SET_ITGT         = 8'h0B;
    localparam [7:0] CMD_IBURST5          = 8'h0C;
    // Lectura de vuelta de la SDRAM: lee el byte en el puntero actual y lo
    // deja en el byte 1 de la respuesta; el puntero avanza en 1. Sirve
    // para VERIFICAR los pesos despues de escribirlos -- el camino de
    // escritura no tiene ninguna comprobacion, y un byte mal escrito
    // queda corrupto de forma permanente (a diferencia de un error de
    // lectura, que la votacion por mayoria de weight_stream.v corrige).
    localparam [7:0] CMD_SDRAM_RD         = 8'h0D;

    wire clk_sys;
    wire clk_sdram;
    wire pll_lock;
    pll_sdram u_pll (
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

    // ---- Campo ancho, para CMD_BIAS_WR y CMD_SET_QPARAM (32 bits de
    // valor en los bytes 1-4, un byte de "cola" en el byte 5) ----
    wire signed [31:0] wide_value = {addr_field, data_field};
    wire [7:0]          tail_byte  = rx_data[7:0];

    // ================= Controlador de SDRAM (identico a Fase 1/2) =================
    wire        sdram_busy;
    wire [7:0]  sdram_dout;
    wire [31:0] sdram_dout32;
    reg         sdram_resetn = 1'b0;
    always @(posedge clk_sys) sdram_resetn <= pll_lock;

    reg [22:0] sdram_target;
    reg [22:0] sdram_op_addr;
    reg [7:0]  sdram_op_data;
    reg        sdram_rd_pulse, sdram_wr_pulse, sdram_refresh_pulse, sdram_rd_burst2_pulse;
    wire [31:0] sdram_dout32_a, sdram_dout32_b;

    reg        pending_wr, pending_rd;
    reg [22:0] pending_addr;
    reg [7:0]  pending_data;

    // Solo se acepta un comando SPI cuando el computo NO esta ocupado (evita
    // que una escritura directa a SDRAM se cuele en medio de una inferencia,
    // cosa que ademas no tendria sentido: la forma del modelo y los pesos
    // se cargan ANTES de arrancar, no durante). Mismo criterio que
    // top_mlp_par.v usa para ignorar comandos de carga mientras busy=1.
    wire cmd_ok = frame_done_d1 && !mlp_busy;

    // Techo real: la SDRAM necesita >=4096 refrescos cada 64ms (~15.6us
    // maximo entre refrescos, ver sdram.v), que a 54MHz son ~844 ciclos.
    // Subido de 700 a 800 (~5% de margen bajo ese techo), validado en
    // docs/caracterizacion-frecuencia-sdram.md junto con la verificacion
    // condicional de mas abajo.
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

    // ================= Verificacion por doble/triple lectura, SOLO en
    // la lectura de weight_stream que sigue justo despues de un refresco
    // (ver docs/caracterizacion-frecuencia-sdram.md -- ahi cae ~97% de la
    // corrupcion intermitente ya documentada). A DIFERENCIA de un primer
    // intento (ver weight_stream.v, encabezado, y el commit revertido en
    // feature/production-refresh-verify-fix), TODA la decision Y
    // ejecucion de la verificacion vive ACA, en un unico lugar -- no se
    // reparte una bandera entre el arbitro y weight_stream.v. Mientras
    // dura la verificacion (v_active), el arbitro bloquea TODO lo demas,
    // incluido un refresco nuevo -- asi la banderita nunca se puede
    // "gastar" en el lugar equivocado, que era la grieta del intento
    // anterior. weight_stream.v (y por lo tanto mlp_engine_par_stream.v)
    // ni se enteran: reciben 'busy'/'dout32' ya verificados, sea que por
    // debajo haya habido 1, 2 o 3 lecturas reales. =================
    reg        refresh_owed = 1'b0;   // hubo un refresco concedido, todavia
                                        // no "gastado" en la lectura siguiente
    // Con restricciones reales de I/O para la SDRAM (ver
    // docs/caracterizacion-frecuencia-sdram.md), el analisis estatico
    // mostro que comparar 'v_cur_sample' (recien llegado del pad) contra
    // v_val1/v_val2 EN EL MISMO ciclo que se decide el proximo estado
    // tenia poco margen a 54MHz -- mismo patron ya corregido en el arnes
    // de aislamiento (sdram_test_harness.v/top_sdram_freqtest.v). Se
    // agregan V_CMP2/V_CMP3: capturar el dato crudo en V_WAIT2/V_WAIT3
    // (sin comparar todavia), comparar recien un ciclo despues con datos
    // ya estables.
    localparam [3:0] V_IDLE=4'd0, V_WAIT1=4'd1, V_ISSUE2=4'd2, V_WAIT2=4'd3,
                      V_CMP2=4'd4, V_ISSUE3=4'd5, V_WAIT3=4'd6, V_CMP3=4'd7,
                      V_DONE=4'd8;
    reg [3:0]  v_state = V_IDLE;
    reg        v_busy_seen = 1'b0;
    // 64 bits para poder verificar tanto una lectura simple (32 bits, la
    // mitad alta siempre en 0) como un rd_burst2 (lo+hi, 64 bits) con el
    // mismo camino de comparacion/mayoria -- ver v_cur_sample mas abajo.
    reg [63:0] v_val1, v_val2, v_val3, v_final;
    reg [22:0] v_addr;
    reg        v_is_burst2;           // que tipo de operacion esta verificando
    reg        v_active = 1'b0;       // hay una verificacion post-refresco en curso

    // ---- Arbitro: SPI directo (carga de pesos a SDRAM) > streaming de
    // pesos (computo) > refresco. Mismo patron ya probado de Fase 1/2,
    // con !v_active agregado a los 3 para que nada se cuele mientras dura
    // una verificacion.
    wire want_spi_op  = !sdram_busy && !v_active && (pending_wr || pending_rd);
    wire want_ws_op   = !sdram_busy && !v_active && !want_spi_op && ws_want_req;
    wire want_refresh = !sdram_busy && !v_active && !want_spi_op && !want_ws_op && refresh_pending;

    wire        ws_want_req;
    wire [22:0] ws_addr;
    wire        ws_is_burst2;

    // Muestra actual, con el mismo ancho sea cual sea el tipo de operacion
    // que se esta verificando -- lo unico que cambia entre una lectura
    // simple y un rd_burst2 es de donde sale el dato, la maquina V_* de
    // abajo es identica para los dos casos.
    wire [63:0] v_cur_sample = v_is_burst2 ? {sdram_dout32_b, sdram_dout32_a}
                                            : {32'd0, sdram_dout32};

    always @(posedge clk_sys) begin
        sdram_wr_pulse       <= 1'b0;
        sdram_rd_pulse       <= 1'b0;
        sdram_rd_burst2_pulse<= 1'b0;
        sdram_refresh_pulse  <= 1'b0;

        if (v_state == V_IDLE) begin
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
            end else if (want_ws_op) begin
                sdram_op_addr <= ws_addr;
                if (ws_is_burst2) sdram_rd_burst2_pulse <= 1'b1;
                else              sdram_rd_pulse         <= 1'b1;
                if (refresh_owed) begin
                    v_addr       <= ws_addr;
                    v_is_burst2  <= ws_is_burst2;
                    v_busy_seen  <= 1'b0;
                    v_active     <= 1'b1;
                    v_state      <= V_WAIT1;
                end
                refresh_owed <= 1'b0;
            end else if (want_refresh) begin
                sdram_refresh_pulse <= 1'b1;
                refresh_owed        <= 1'b1;
            end
        end else begin
            case (v_state)
                V_WAIT1: begin
                    if (sdram_busy) v_busy_seen <= 1'b1;
                    if (v_busy_seen && !sdram_busy) begin
                        v_val1      <= v_cur_sample;
                        v_busy_seen <= 1'b0;
                        v_state     <= V_ISSUE2;
                    end
                end
                V_ISSUE2: begin
                    sdram_op_addr <= v_addr;
                    if (v_is_burst2) sdram_rd_burst2_pulse <= 1'b1;
                    else              sdram_rd_pulse         <= 1'b1;
                    v_state <= V_WAIT2;
                end
                V_WAIT2: begin
                    if (sdram_busy) v_busy_seen <= 1'b1;
                    if (v_busy_seen && !sdram_busy) begin
                        v_val2      <= v_cur_sample;
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
                    sdram_op_addr <= v_addr;
                    if (v_is_burst2) sdram_rd_burst2_pulse <= 1'b1;
                    else              sdram_rd_pulse         <= 1'b1;
                    v_state <= V_WAIT3;
                end
                V_WAIT3: begin
                    if (sdram_busy) v_busy_seen <= 1'b1;
                    if (v_busy_seen && !sdram_busy) begin
                        v_val3      <= v_cur_sample;
                        v_busy_seen <= 1'b0;
                        v_state     <= V_CMP3;
                    end
                end
                V_CMP3: begin
                    // mayoria de 3: si la tercera no coincide con
                    // ninguna de las dos primeras (rarisimo), se toma
                    // la tercera igual -- mejor un valor fresco que
                    // uno de dos que ya se sabe que no coinciden.
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

        if (cmd_ok) begin
            case (cmd)
                CMD_SDRAM_SET_ADDR: sdram_target <= {data_field[6:0], addr_field[15:0]};
                CMD_SDRAM_WR: begin
                    pending_addr <= sdram_target;
                    pending_data <= data_field[7:0];
                    pending_wr   <= 1'b1;
                    sdram_target <= sdram_target + 23'd1;
                end
                CMD_SDRAM_RD: begin
                    pending_addr <= sdram_target;
                    pending_rd   <= 1'b1;
                    sdram_target <= sdram_target + 23'd1;
                end
                CMD_WSTREAM_SET_BASE: weight_base_reg <= {data_field[6:0], addr_field[15:0]};
                default: ;
            endcase
        end
    end

    reg [22:0] weight_base_reg = 23'd0;

    // Lo que ve weight_stream.v (via mlp_engine_par_stream.v): 'busy' se
    // mantiene en alto durante TODA la verificacion (no solo la primera
    // lectura), y 'dout32' entrega el valor ya votado. Fuera de una
    // verificacion (v_active=0, el 97% del tiempo) esto es identico a
    // los cables crudos de sdram.v -- cero costo extra en el camino
    // rapido.
    wire        ws_sdram_busy     = sdram_busy || v_active;
    wire [31:0] ws_sdram_dout32   = (v_state == V_DONE && !v_is_burst2) ? v_final[31:0]  : sdram_dout32;
    wire [31:0] ws_sdram_dout32_a = (v_state == V_DONE &&  v_is_burst2) ? v_final[31:0]  : sdram_dout32_a;
    wire [31:0] ws_sdram_dout32_b = (v_state == V_DONE &&  v_is_burst2) ? v_final[63:32] : sdram_dout32_b;

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

    // ================= Decodificador de comandos del motor de computo =================
    reg load_numlayers_en, load_layershape_en, set_bias_addr_en, load_bias_en;
    reg load_qparam_en, start_pulse, dbg_rd_en, set_itgt;
    always @(*) begin
        load_numlayers_en  = 1'b0;
        load_layershape_en = 1'b0;
        set_bias_addr_en   = 1'b0;
        load_bias_en       = 1'b0;
        load_qparam_en     = 1'b0;
        start_pulse        = 1'b0;
        dbg_rd_en          = 1'b0;
        set_itgt           = 1'b0;
        if (cmd_ok) begin
            case (cmd)
                CMD_SET_NUM_LAYERS:  load_numlayers_en  = 1'b1;
                CMD_SET_LAYER_SHAPE: load_layershape_en = 1'b1;
                CMD_SET_BIAS_ADDR:   set_bias_addr_en   = 1'b1;
                CMD_BIAS_WR:         load_bias_en       = 1'b1;
                CMD_SET_QPARAM:      load_qparam_en     = 1'b1;
                CMD_START:           start_pulse        = 1'b1;
                CMD_DBG_RD:          dbg_rd_en          = 1'b1;
                CMD_SET_ITGT:        set_itgt           = 1'b1;
                default: ;
            endcase
        end
    end

    // ---- Puntero de escritura de bias (mismo patron "set + wr autoincr"
    // que la escritura directa a SDRAM de Fase 1) ----
    reg [10:0] bias_ptr;
    always @(posedge clk_sys) begin
        if (set_bias_addr_en) bias_ptr <= addr_field[10:0];
        else if (load_bias_en) bias_ptr <= bias_ptr + 11'd1;
    end

    // ---- Rafaga de entrada (mismo patron de siempre, puntero ensanchado
    // a 11 bits para cubrir hasta MAX_INPUT_WIDTH) ----
    reg [2:0]        ib_step = 3'd0;
    reg signed [7:0] ib_bytes [0:4];
    reg [10:0]       i_ptr;
    reg              imac_en;
    reg [10:0]       imac_addr;
    reg signed [7:0] imac_data;

    always @(posedge clk_sys) begin
        imac_en <= 1'b0;
        if (set_itgt) begin
            i_ptr <= addr_field[10:0];
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
            imac_addr <= i_ptr + {8'd0, (ib_step - 3'd1)};
            imac_data <= ib_bytes[ib_step - 3'd1];
            if (ib_step == 3'd5) begin
                i_ptr   <= i_ptr + 11'd5;
                ib_step <= 3'd0;
            end else begin
                ib_step <= ib_step + 3'd1;
            end
        end
    end

    wire              mlp_busy;
    wire signed [7:0] dbg_rd_data;

    mlp_engine_par_stream u_mlp (
        .clk (clk_sys),
        .rst (start_pulse),

        .ws_want_req   (ws_want_req),
        .ws_addr       (ws_addr),
        .ws_is_burst2  (ws_is_burst2),
        .ws_issue      (want_ws_op),
        .sdram_busy    (ws_sdram_busy),
        .sdram_dout32  (ws_sdram_dout32),
        .sdram_dout32_a(ws_sdram_dout32_a),
        .sdram_dout32_b(ws_sdram_dout32_b),
        .weight_base_addr(weight_base_reg),

        .load_num_layers_en   (load_numlayers_en),
        .load_num_layers_value(addr_field[3:0]),

        .load_layer_shape_en   (load_layershape_en),
        .load_layer_shape_layer(addr_field[2:0]),
        .load_layer_shape_sel  (addr_field[5:4]),
        .load_layer_shape_value(data_field),

        .load_bias_en   (load_bias_en),
        .load_bias_addr (bias_ptr),
        .load_bias_data (wide_value),

        .load_qparam_en   (load_qparam_en),
        .load_qparam_layer(tail_byte[4:2]),
        .load_qparam_sel  (tail_byte[1:0]),
        .load_qparam_value(wide_value),

        .load_input_en  (imac_en),
        .load_input_addr(imac_addr),
        .load_input_data(imac_data),

        .start (start_pulse),
        .busy  (mlp_busy),

        .dbg_rd_en  (dbg_rd_en),
        .dbg_rd_sel (addr_field[13:11]),
        .dbg_rd_addr(addr_field[10:0]),
        .dbg_rd_data(dbg_rd_data)
    );

    // bit2 del status = SDRAM ocupada (incluye lo pendiente en la cola),
    // para que el host pueda sondear las lecturas de verificacion sin
    // confundirlas con el busy del computo (bit0).
    wire sdram_busy_ext = sdram_busy | pending_wr | pending_rd | sdram_wr_pulse | sdram_rd_pulse;

    // ---- El snapshot se CONGELA mientras hay una trama SPI en curso ----
    // spi_slave.v lee tx_data bit a bit de forma combinacional, asi que el
    // valor tiene que mantenerse constante durante toda la trama (lo dice
    // su propio comentario). Sin esto, si una lectura de SDRAM termina a
    // mitad de trama, el byte sale "partido" -- se veia como bytes mal en
    // la verificacion de pesos que NO estaban realmente mal: la cuenta
    // variaba entre pasadas (43, 43, 54, 52) sobre datos que nadie tocaba,
    // y las inferencias daban 20/20 igual. Misma leccion que la fase 1.
    reg cs_sync0 = 1'b1, cs_sync1 = 1'b1, cs_sync2 = 1'b1, cs_sync3 = 1'b1;
    always @(posedge clk_sys) begin
        cs_sync0 <= cs_n;
        cs_sync1 <= cs_sync0;
        cs_sync2 <= cs_sync1;
        cs_sync3 <= cs_sync2;
    end

    // byte0=status, byte1=byte leido de SDRAM, byte5=byte de debug
    always @(posedge clk_sys) begin
        if (cs_sync3)
            tx_snapshot <= {5'd0, sdram_busy_ext, pll_lock, mlp_busy, sdram_dout, 24'd0, dbg_rd_data};
    end

    assign led = ~{mlp_busy, pll_lock, 4'b0000};

endmodule
