// pll_sdram_static.v -- version parametrizable de pll_sdram.v para la
// caracterizacion de frecuencia de la SDRAM (ver plan_testing_sdram.md).
//
// CAMBIO DE PLAN respecto del diseño original: se habia empezado con
// pll_sdram_multi.v, que instanciaba 4 rPLL + 2 DCS para conmutar de
// frecuencia EN VIVO sin glitches. La sintesis lo rechazo: el GW2AR-18C
// solo tiene 2 bloques de PLL fisicos en el chip (ERROR (PA2017), numero
// de PLL(4) excede el limite del dispositivo(2)) -- 4 PLLs simultaneos es
// imposible en este chip, sin importar como se conecten.
//
// La alternativa correcta seria reconfigurar UN solo rPLL dinamicamente
// (DYN_FBDIV_SEL/DYN_IDIV_SEL/DYN_ODIV_SEL="true", con FBDSEL/IDSEL/ODSEL
// como entradas en vivo), pero esa codificacion (ODSEL en particular usa
// una formula no obvia, ODSEL=128-ODIV0, y el protocolo de temporizado del
// reset/relock no esta claramente documentado en fuentes accesibles) tiene
// mas riesgo del que vale la pena para pruebas de hardware de horas sin
// supervision. En cambio: la frecuencia queda FIJA por PLL estatico (igual
// que pll_sdram.v de siempre, cero riesgo, ya probado en produccion), y
// "cambiar de frecuencia" pasa a ser "generar un bitstream distinto por
// frecuencia y reprogramar la FPGA entre corridas" -- mismo flujo de
// programmer_cli.exe ya usado toda la sesion, sin logica nueva de reloj.
//
// Los 4 candidatos (ver top_freqtest_27/36/45/54.v):
//   27 MHz: IDIV_SEL=2 FBDIV_SEL=2 ODIV_SEL=32  VCO=864MHz
//   36 MHz: IDIV_SEL=2 FBDIV_SEL=3 ODIV_SEL=16  VCO=576MHz
//   45 MHz: IDIV_SEL=2 FBDIV_SEL=4 ODIV_SEL=16  VCO=720MHz
//   54 MHz: IDIV_SEL=2 FBDIV_SEL=5 ODIV_SEL=16  VCO=864MHz (= produccion)
// PFD = 27/(IDIV_SEL+1) = 9MHz;  CLKOUT = PFD*(FBDIV_SEL+1);  VCO=CLKOUT*ODIV_SEL
// (rango valido de VCO en este chip: 500-1250MHz, confirmado por Gowin al
// fallar la sintesis original con 432MHz -- el 27MHz necesita ODIV_SEL=32,
// no 16, para no caer por debajo del piso)
module pll_sdram_static #(
    parameter IDIV_SEL  = 2,
    parameter FBDIV_SEL = 5,
    parameter ODIV_SEL  = 16
) (
    input  wire clk_in,    // 27 MHz, directo del oscilador de la placa
    output wire clk_out,
    output wire clk_sdram, // desfasado 180 grados (ver pll_sdram.v)
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
    defparam rpll_inst.IDIV_SEL        = IDIV_SEL;
    defparam rpll_inst.DYN_FBDIV_SEL   = "false";
    defparam rpll_inst.FBDIV_SEL       = FBDIV_SEL;
    defparam rpll_inst.DYN_ODIV_SEL    = "false";
    defparam rpll_inst.ODIV_SEL        = ODIV_SEL;
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
