// pll_sdram.v -- PLL dedicada a la etapa 4 (SDRAM): genera dos salidas
// desde el oscilador de 27MHz de la placa:
//   clk_out  -- reloj principal (54MHz) para toda la logica de esta etapa
//               (SPI, controlador de SDRAM, FSM de computo).
//   clk_sdram -- el MISMO reloj pero desfasado 180 grados (CLKOUTP de la
//               PLL), necesario para manejar el pin fisico SDRAM_CLK -- sin
//               este desfasaje el retardo de pista de la placa hace que la
//               SDRAM vea los comandos en el momento equivocado. Patron
//               tomado de nand2mario/sdram-tang-nano-20k (Apache-2.0).
//
// OJO -- bug encontrado y corregido: el valor original copiado del
// proyecto de referencia, PSDA_SEL="1010", NO es 180 grados como decia
// este mismo comentario -- segun la tabla oficial de Gowin (UG286-2.0.2E,
// Tabla 5-7 "rPLL Phase Parameter Adjustment"), cada paso de PSDA_SEL son
// 22.5 grados (0000=0st, 0100=90, 1000=180, 1010=225, 1100=270...), asi
// que "1010" son en realidad 225 grados, un error de 45 grados (~2.3ns a
// 54MHz) respecto de la fase realmente buscada. Esto se descubrio
// investigando una corrupcion de datos intermitente (~0.3-0.5% de las
// lecturas, siempre devolvia el valor de la lectura anterior) que
// sobrevivio a: mas margen en T_RCD/T_RP, y a bajar clk_sys a la mitad de
// frecuencia (lo que descarta un simple problema de "poco tiempo" y
// apunta a un desalineamiento de fase, que no mejora con menos
// frecuencia). Corregido a "1000" (180 grados real).
//
// Parametros (formulas UG286):
//   PFD    = FCLKIN / (IDIV_SEL+1)                = 27 / 1  = 27 MHz
//   CLKOUT = FCLKIN * (FBDIV_SEL+1) / (IDIV_SEL+1) = 27*2/1 = 54 MHz
//   VCO    = CLKOUT * ODIV_SEL                     = 54*16  = 864 MHz
module pll_sdram (
    input  wire clk_in,    // 27 MHz, directo del oscilador de la placa
    output wire clk_out,   // 54 MHz
    output wire clk_sdram, // 54 MHz, desfasado 180 grados (de verdad, ahora)
    output wire lock
);

    wire gw_gnd = 1'b0;

    rPLL rpll_inst (
        .CLKOUT (clk_out),
        .LOCK   (lock),
        .CLKOUTP(clk_sdram),
        .CLKOUTD(),
        .CLKOUTD3(),
        .RESET  (gw_gnd),
        .RESET_P(gw_gnd),
        .CLKIN  (clk_in),
        .CLKFB  (gw_gnd),
        .FBDSEL ({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .IDSEL  ({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .ODSEL  ({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .PSDA   (4'b1000),
        .DUTYDA ({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .FDLY   ({gw_gnd,gw_gnd,gw_gnd,gw_gnd})
    );

    defparam rpll_inst.FCLKIN          = "27";
    defparam rpll_inst.DYN_IDIV_SEL    = "false";
    defparam rpll_inst.IDIV_SEL        = 0;
    defparam rpll_inst.DYN_FBDIV_SEL   = "false";
    defparam rpll_inst.FBDIV_SEL       = 1;
    defparam rpll_inst.DYN_ODIV_SEL    = "false";
    defparam rpll_inst.ODIV_SEL        = 16;
    defparam rpll_inst.PSDA_SEL        = "1000";
    defparam rpll_inst.DYN_DA_EN       = "false";
    defparam rpll_inst.DUTYDA_SEL      = "1000";
    defparam rpll_inst.CLKOUT_FT_DIR   = 1'b1;
    defparam rpll_inst.CLKOUTP_FT_DIR  = 1'b1;
    defparam rpll_inst.CLKOUT_DLY_STEP = 0;
    defparam rpll_inst.CLKOUTP_DLY_STEP = 0;
    defparam rpll_inst.CLKFB_SEL       = "internal";
    defparam rpll_inst.CLKOUT_BYPASS   = "false";
    defparam rpll_inst.CLKOUTP_BYPASS  = "false";
    defparam rpll_inst.CLKOUTD_BYPASS  = "false";
    defparam rpll_inst.DYN_SDIV_SEL    = 2;
    defparam rpll_inst.CLKOUTD_SRC     = "CLKOUT";
    defparam rpll_inst.CLKOUTD3_SRC    = "CLKOUT";

endmodule
