// pll_par_54.v -- PLL a 54MHz dedicada a las variantes de demo
// top_mlp_par_s1/s2/s3.v (ver README, seccion "3 situaciones"). No se
// reusa pll_par.v (108MHz, del modelo original) porque el bus de
// direcciones mas ancho (13 bits) de estas variantes ya no entra en el
// presupuesto de timing de 108MHz -- ver situaciones.sdc.
//
// Parametros (formulas UG286):
//   PFD    = FCLKIN / (IDIV_SEL+1)                = 27 / 1  = 27 MHz  (rango 3-400 MHz)
//   CLKOUT = FCLKIN * (FBDIV_SEL+1) / (IDIV_SEL+1) = 27*2/1 = 54 MHz  (rango 4.6875-600 MHz)
//   VCO    = CLKOUT * ODIV_SEL                     = 54*16  = 864 MHz (rango 600-1200 MHz)
module pll_par_54 (
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
