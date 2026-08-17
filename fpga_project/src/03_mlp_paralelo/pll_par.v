// pll_par.v -- PLL dedicada a top_mlp_par.v, separada de pll_108mhz.v (que
// usa top_mlp_fast.v y no se toca para no arriesgar esa referencia que ya
// funciona).
//
// La corrupcion intermitente que se veia antes en la carga de pesos NO era
// timing/PLL -- se confirmo que era cableado de SPI (ver README del
// proyecto). Barrido de frecuencia hecho (27/54/108/216, doblando cada
// vez): 108MHz fue el ultimo paso limpio (0 violaciones, funciona en
// placa), 216MHz ya rompe (violaciones reales de timing, confirmado en
// placa). Se deja fijo en 108MHz, que es el techo real encontrado para
// ESTE modelo (130->128->64->32->5). VCO=864MHz.
//
// Parametros (formulas UG286):
//   PFD    = FCLKIN / (IDIV_SEL+1)                = 27 / 1  = 27 MHz  (rango 3-400 MHz)
//   CLKOUT = FCLKIN * (FBDIV_SEL+1) / (IDIV_SEL+1) = 27*4/1 = 108 MHz (rango 4.6875-600 MHz)
//   VCO    = CLKOUT * ODIV_SEL                     = 108*8  = 864 MHz (rango 600-1200 MHz)
module pll_par (
    input  wire clk_in,   // 27 MHz, directo del oscilador de la placa
    output wire clk_out,  // 108 MHz
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
    defparam rpll_inst.FBDIV_SEL       = 3;
    defparam rpll_inst.DYN_ODIV_SEL    = "false";
    defparam rpll_inst.ODIV_SEL        = 8;
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
