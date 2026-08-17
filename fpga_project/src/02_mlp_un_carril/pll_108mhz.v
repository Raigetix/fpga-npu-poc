// pll_108mhz.v -- multiplica el oscilador de 27MHz de la placa a 54MHz (x2)
// usando la PLL dura del chip (rPLL). NOTA: el nombre del archivo quedo de
// un primer intento a 108MHz (x4) que en la placa real daba muchas fallas de
// datos (corrupcion masiva en la capa 1) pese a que la sintesis reportaba
// timing limpio -- ver top.sdc para la explicacion (create_clock simple no
// modela el jitter propio de la PLL). Bajado a 54MHz como punto mas
// conservador, con margen de incertidumbre agregado en el SDC.
//
// Parametros (verificados contra las formulas oficiales de Gowin, UG286):
//   PFD    = FCLKIN / (IDIV_SEL+1)              = 27 / 1  = 27 MHz  (rango valido: 3-400 MHz)
//   CLKOUT = FCLKIN * (FBDIV_SEL+1) / (IDIV_SEL+1) = 27*2/1 = 54 MHz (rango valido: 4.6875-600 MHz)
//   VCO    = CLKOUT * ODIV_SEL                   = 54*16  = 864 MHz  (rango valido: 600-1200 MHz)
module pll_108mhz (
    input  wire clk_in,   // 27 MHz, directo del oscilador de la placa
    output wire clk_out,  // 54 MHz
    output wire lock
);

    wire gw_gnd = 1'b0;

    rPLL rpll_inst (
        .CLKOUT (clk_out),
        .LOCK   (lock),
        .CLKOUTP(),
        .CLKOUTD(),
        .CLKOUTD3(),
        .RESET  (gw_gnd),
        .RESET_P(gw_gnd),
        .CLKIN  (clk_in),
        .CLKFB  (gw_gnd),
        .FBDSEL ({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .IDSEL  ({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .ODSEL  ({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .PSDA   ({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
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
    defparam rpll_inst.PSDA_SEL        = "0000";
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
