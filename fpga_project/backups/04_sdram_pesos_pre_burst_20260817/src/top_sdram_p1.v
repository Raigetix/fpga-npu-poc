// top_sdram_p1.v -- Etapa 4, Fase 1: leer y escribir la SDRAM embebida
// DIRECTO por SPI, sin ningun cache/buffer de por medio. Objetivo: probar
// que el controlador de SDRAM en si funciona antes de construir nada
// arriba (mismo metodo de "aislar antes de integrar" de toda la sesion).
//
// Los puertos O_sdram_*/IO_sdram_dq son "nombres magicos" que el
// compilador de Gowin conecta solo a los pines fisicos de la SDRAM
// embebida del GW2AR-18 -- NO llevan IO_LOC en el .cst (confirmado
// revisando 2 proyectos de referencia para esta placa, ninguno los
// constrain a mano).
//
// Protocolo (mismo formato de frame de 48 bits que el resto del proyecto):
//   CMD_NOP            = 0x00
//   CMD_SDRAM_SET_ADDR = 0x01  A[15:0]=direccion[15:0]  B[6:0]=direccion[22:16]
//                               (fija/reinicia el puntero, no lee ni escribe)
//   CMD_SDRAM_WR       = 0x02  B_lo=byte a escribir en el puntero actual;
//                               el puntero avanza en 1 despues
//   CMD_SDRAM_RD       = 0x03  dispara una lectura en el puntero actual;
//                               el puntero avanza en 1 despues. El dato
//                               aparece en el byte 1 de la respuesta SPI
//                               (queda guardado hasta la proxima lectura).
//
//   MISO: [status: bit0=sdram_busy bit1=pll_lock] [ultimo dato leido] [xxxx]
//
//   Los comandos se ignoran mientras sdram_busy=1 (una operacion tarda
//   ~5 ciclos de clk_sys en completarse) -- el ESP32 tiene que sondear con
//   CMD_NOP hasta ver busy=0 antes de mandar el siguiente comando real,
//   igual criterio que ya se usa en toda la sesion para busy de computo.
module top_sdram_p1 (
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

    localparam [7:0] CMD_NOP            = 8'h00;
    localparam [7:0] CMD_SDRAM_SET_ADDR = 8'h01;
    localparam [7:0] CMD_SDRAM_WR       = 8'h02;
    localparam [7:0] CMD_SDRAM_RD       = 8'h03;

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

    // ================= Controlador de SDRAM =================
    wire        sdram_busy;
    wire        sdram_data_ready;
    wire [7:0]  sdram_dout;
    reg         sdram_resetn = 1'b0;
    always @(posedge clk_sys) sdram_resetn <= pll_lock; // esperar lock antes de soltar el reset

    reg [22:0] sdram_target;    // puntero, fijado por CMD_SDRAM_SET_ADDR
    reg [22:0] sdram_op_addr;   // direccion presentada al controlador EN EL PULSO
    reg [7:0]  sdram_op_data;
    reg        sdram_rd_pulse, sdram_wr_pulse, sdram_refresh_pulse;

    // ---- Comando de usuario en cola (profundidad 1) ----
    // El comando SPI se acepta SIEMPRE que termina el frame, sin importar
    // si el controlador esta ocupado en ese instante (con un refresco, por
    // ejemplo) -- version anterior descartaba el comando si busy=1 justo
    // en ese momento, lo cual perdia bytes en silencio: el puntero no
    // avanzaba para ese byte, y todo lo que segia quedaba corrido una
    // posicion (exactamente el patron que se vio en la placa real: a
    // partir de cierto indice, cada valor era el que le tocaba al
    // siguiente). Ahora el comando queda pendiente y se emite apenas el
    // controlador se libera, sin perder nada.
    reg        pending_wr, pending_rd;
    reg [22:0] pending_addr;
    reg [7:0]  pending_data;

    wire cmd_ok = frame_done_d1;

    // ---- Refresco periodico: 4096 refrescos cada 64ms, o sea al menos
    // uno cada ~15us. A 54MHz eso son ~810 ciclos; se usa 700 (~13us) de
    // margen.
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

    // ---- Arbitro + aceptacion de comando, TODO en un unico bloque (para
    // no repetir el error de "dos drivers" de la misma sesion: pending_wr/
    // pending_rd no pueden tocarse desde dos always distintos). Orden:
    // primero se sirve lo pendiente (si el controlador esta libre),
    // despues se acepta el comando SPI nuevo -- si ambas cosas pasan el
    // mismo ciclo, la asignacion de "aceptar" queda ultima y gana (semantica
    // no bloqueante de Verilog), que es el comportamiento correcto: no se
    // pierde el comando nuevo aunque justo se haya liberado el pendiente
    // anterior en este mismo ciclo.
    wire want_user_op = !sdram_busy && (pending_wr || pending_rd);
    wire want_refresh  = !sdram_busy && !want_user_op && refresh_pending;

    always @(posedge clk_sys) begin
        sdram_wr_pulse      <= 1'b0;
        sdram_rd_pulse      <= 1'b0;
        sdram_refresh_pulse <= 1'b0;

        if (want_user_op) begin
            sdram_op_addr <= pending_addr;
            if (pending_wr) begin
                sdram_op_data  <= pending_data;
                sdram_wr_pulse <= 1'b1;
                pending_wr     <= 1'b0;
            end else begin
                sdram_rd_pulse <= 1'b1;
                pending_rd     <= 1'b0;
            end
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
                CMD_SDRAM_RD: begin
                    pending_addr <= sdram_target;
                    pending_rd   <= 1'b1;
                    sdram_target <= sdram_target + 23'd1;
                end
                default: ;
            endcase
        end
    end

    // ---- Instrumentacion (ronda 1, YA CONCLUIDA): contar comandos
    // CMD_SDRAM_RD aceptados vs. pulsos rd reales entregados a sdram.v.
    // Resultado: diff se mantuvo CONSTANTE en -1 en TODAS las lecturas,
    // fallen o no -- no hay ningun comando perdido ni ningun pulso
    // fantasma asociado a las corrupciones. Cada comando SI dispara
    // exactamente un pulso real. Se descarta esta hipotesis.
    //
    // ---- Instrumentacion (ronda 2): ya que el pulso SI se dispara 1:1,
    // el sospechoso que queda es la DIRECCION que se le pasa a ese pulso
    // (sdram_op_addr) -- capturamos que direccion se uso EN EL PULSO mas
    // reciente para poder compararla, del lado del ESP32, contra la
    // direccion que se esperaba para ese indice de lectura.
    reg [22:0] stat_last_rd_addr = 23'd0;
    always @(posedge clk_sys) begin
        if (sdram_rd_pulse) stat_last_rd_addr <= sdram_op_addr;
    end

    // ---- Instrumentacion (ronda 3): direccion y fase del reloj ya se
    // descartaron. Se vio en la placa real que UNA corrida entera de 256
    // lecturas tardo 531ms en vez de los ~20ms habituales (25x mas
    // lento), coincidiendo con una falla -- puede ser que sdram_busy se
    // quede en alto mucho mas tiempo del esperado en algun punto (un
    // atasco real del lado FPGA), o puede ser un problema del lado ESP32
    // (WiFi/interrupciones) sin relacion con el bug de datos. Este
    // contador mide cuantos ciclos de clk_sys estuvo en alto sdram_busy
    // en la ULTIMA operacion completada (lectura, escritura o refresco):
    // deberia dar ~5-6 ciclos normalmente. Si en el momento de una falla
    // este numero aparece gigante, confirma un atasco real en la FPGA.
    reg [31:0] busy_cyc_ctr        = 32'd0;
    reg [31:0] stat_last_busy_cyc  = 32'd0;
    reg        sdram_busy_d        = 1'b0;
    always @(posedge clk_sys) begin
        sdram_busy_d <= sdram_busy;
        if (sdram_busy) begin
            busy_cyc_ctr <= busy_cyc_ctr + 32'd1;
        end else begin
            busy_cyc_ctr <= 32'd0;
            if (sdram_busy_d) stat_last_busy_cyc <= busy_cyc_ctr;
        end
    end

    // T_RP=2: no cambio nada (hipotesis de refresco-vs-precharge
    // descartada por experimento). Direccion y dato viven en el MISMO
    // registro (tx_snapshot), capturados juntos por el mismo mecanismo de
    // congelado -- si la direccion SIEMPRE llega bien pero el dato no,
    // el problema no puede ser de reporte por SPI (ya se probo 3 veces
    // sin exito), tiene que ser la propia SDRAM no activando la fila a
    // tiempo. T_RCD (activacion->lectura) subido de 1 a 2 ciclos: con 1
    // ciclo son ~18.5ns a 54MHz contra un minimo tipico de chip de
    // ~15-20ns, margen casi nulo frente a variaciones de voltaje/
    // temperatura -- si la fila no termina de activarse a tiempo, el
    // chip puede devolver lo que haya quedado en los amplificadores de
    // sentido de la fila ANTERIOR, exactamente el patron observado.
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

    // ---- Busy "visto desde afuera" ----
    // sdram_busy crudo no alcanza para el bit de status: entre que el
    // arbitro decide emitir una operacion encolada (pending_rd/pending_wr
    // baja a 0, sale el pulso rd/wr) y que sdram.v realmente lo procesa y
    // sube su propio busy, pasan 1-2 ciclos de clk_sys donde NINGUNO de
    // los dos esta en 1. Si una trama SPI de sondeo cae justo en ese
    // hueco, el ESP32 ve busy=0 y da la lectura por terminada, pero el
    // dato que lee todavia es el de la lectura anterior (asi se vio en la
    // placa real: valor previo del patron, consistente, con una unica
    // lectura mal en ~0.5% de los casos). top_busy tapa ese hueco.
    wire top_busy = sdram_busy | pending_wr | pending_rd | sdram_wr_pulse | sdram_rd_pulse;

    // ---- OJO: spi_slave.v lee tx_data EN VIVO, bit a bit, de forma
    // combinacional (miso = tx_data[bit_idx]) -- su propio comentario dice
    // explicitamente que esto es seguro SOLO SI tx_data se mantiene
    // constante durante todo un frame, cambiando una unica vez cuando NO
    // hay ningun frame en curso. Actualizar tx_snapshot en CADA ciclo de
    // clk_sys (primera version) rompia eso: el snapshot podia cambiar a
    // mitad de un frame y "desgarrar" la lectura (status ya "listo" pero
    // dato todavia viejo). Pero actualizarlo solo al TERMINAR el frame que
    // disparo el comando (segundo intento) tambien esta mal, por el
    // motivo opuesto: en este modulo, a diferencia del resto del
    // proyecto, el efecto de un comando NO es inmediato (el arbitro tarda
    // varios ciclos en sacarlo de la cola y sdram.v tarda ~5-6 ciclos mas
    // en ejecutarlo) -- capturar el snapshot justo al terminar ESE mismo
    // frame lo deja congelado en el estado ANTERIOR al comando durante
    // todo el frame siguiente, resultando en un atraso de exactamente una
    // lectura, siempre (lo que se vio en la placa la segunda vez).
    //
    // Solucion: sincronizar cs_n a este dominio de reloj y actualizar
    // tx_snapshot en TODO ciclo donde no hay ningun frame en curso (cs_n
    // sincronizado en alto) -- se congela apenas empieza un frame nuevo
    // (nada lo cambia durante el frame, sin desgarro) pero se mantiene lo
    // mas fresco posible hasta ese instante (sin atraso estructural).
    //
    // Resto de carrera: esta cadena y la de spi_slave.v (que genera
    // frame_done a partir del mismo cs_n, de forma totalmente
    // independiente) llegan a "trama terminada" en ciclos ligeramente
    // distintos. Con solo 2 flip-flops, esta cadena declara "seguro
    // actualizar" ANTES de que pending_wr/pending_rd (que dependen de
    // frame_done_d1, un registro mas atras, mas el propio case que los
    // fija) lleguen a asentarse -- una ventana real de ~2 ciclos donde
    // top_busy todavia lee 0 aunque el comando ya este en camino. Se
    // agregan 2 flip-flops extra (4 en total) para que esta cadena
    // siempre quede in por lo menos tan atras como pending_wr/pending_rd,
    // cerrando la ventana con margen de sobra (unos 90ns extra a 54MHz,
    // insignificante frente a los ~6us que dura una trama SPI completa).
    reg cs_sync0 = 1'b1, cs_sync1 = 1'b1, cs_sync2 = 1'b1, cs_sync3 = 1'b1;
    always @(posedge clk_sys) begin
        cs_sync0 <= cs_n;
        cs_sync1 <= cs_sync0;
        cs_sync2 <= cs_sync1;
        cs_sync3 <= cs_sync2;
    end

    // Layout por byte: byte0=status, byte1=dout, byte2-5=stat_last_busy_cyc
    // (32 bits, MSB primero) -- direccion ya se confirmo siempre correcta,
    // asi que se reemplaza por el contador de ciclos ocupados de la ronda 3.
    always @(posedge clk_sys) begin
        if (cs_sync3)
            tx_snapshot <= {6'd0, pll_lock, top_busy, sdram_dout, stat_last_busy_cyc};
    end

    assign led = ~{top_busy, pll_lock, 4'b0000};

endmodule
