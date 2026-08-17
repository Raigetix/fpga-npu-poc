// debug_weight_par_direct.v -- diagnostico DIRECTO (sin SPI de por medio) de
// la estabilidad de lectura de weight_bank en mac_lane.v. Se agrego despues
// de que 4 arreglos distintos sobre la memoria/pipeline/decodificadores NO
// cambiaran el patron de corrupcion observado via SPI (ver README de esta
// carpeta) -- la idea es sacar del medio TODO lo que ya se descarto (SPI,
// CDC, la FSM de comandos, el mux de debug) y mirar el dato "en crudo".
//
// Carga un patron conocido (weight_bank[addr] = addr[7:0]) en TODO el banco
// de UN carril al arrancar, sin usar SPI para nada. Despues lee, en loop
// perpetuo, SIEMPRE la misma direccion (elegida por 'addr_sel': 0=direccion
// seguramente sana (100), 1=direccion sospechosa (1030, adentro de la zona
// donde se agrupo la corrupcion en TODAS las pruebas anteriores) y saca el
// valor leido directo por 8 pines GPIO, sin pasar por ningun comando ni
// protocolo.
//
// Si el valor en los pines "parpadea" sin que nadie vuelva a escribir nada
// (la carga termina una sola vez, al principio), confirma que el problema
// es de memoria/hardware, aislado de todo lo demas. Si se mantiene
// perfectamente estable, el problema esta en otro lado (SPI/CDC/FSM/mux).
module debug_weight_par_direct (
    input  wire       clk,       // 27 MHz onboard, pin 4
    input  wire        addr_sel,  // 0 = direccion segura (100), 1 = sospechosa (1030)
    output wire [7:0]  data_out,  // weight_dbg leido en vivo, sin parar
    output wire         heartbeat, // prueba de que el reloj interno sigue vivo
    output wire [5:0]  led
);

    wire clk_sys;
    wire pll_lock;
    pll_par u_pll (
        .clk_in (clk),
        .clk_out(clk_sys),
        .lock   (pll_lock)
    );

    // ---- Carga interna, una sola vez, sin SPI: weight_bank[addr]=addr[7:0] ----
    reg [11:0] load_ctr = 12'd0;
    reg        loading  = 1'b1;
    always @(posedge clk_sys) begin
        if (loading) begin
            if (load_ctr == 12'd3391) loading <= 1'b0;
            else load_ctr <= load_ctr + 1'b1;
        end
    end

    wire [11:0] test_addr = addr_sel ? 12'd1030 : 12'd100;

    wire signed [7:0] weight_dbg;
    mac_lane #(.BANK_ABITS(12)) u_lane (
        .clk        (clk_sys),
        .load_en    (loading),
        .load_addr  (load_ctr),
        .load_data  (load_ctr[7:0]),
        .rd_addr    (test_addr),
        .actin_rdata(8'sd0),
        .acc_clear  (1'b0),
        .acc_en     (1'b0),
        .bias_rdata (8'sd0),
        .activated  (),
        .weight_dbg (weight_dbg)
    );

    assign data_out = weight_dbg;

    reg [23:0] hb_ctr = 24'd0;
    always @(posedge clk_sys) hb_ctr <= hb_ctr + 1'b1;
    assign heartbeat = hb_ctr[23];

    assign led = ~{pll_lock, loading, addr_sel, 3'b000};

endmodule
