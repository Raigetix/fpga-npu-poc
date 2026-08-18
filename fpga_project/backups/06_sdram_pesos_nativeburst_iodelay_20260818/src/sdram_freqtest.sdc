// sdram_freqtest.sdc -- constraints de reloj para los builds de
// caracterizacion de frecuencia (top_freqtest_27/36/45/54.v). clk_sys vive
// dentro de la instancia u_core (top_sdram_freqtest), no en el top-level
// directamente como en top_sdram_p3.v -- de ahi el prefijo de jerarquia.
//
// El periodo se deja fijo en el del candidato MAS RAPIDO (54MHz, el mismo
// valor que sdram_pesos.sdc usa para produccion): cerrar timing a 54MHz
// implica automaticamente que 27/36/45MHz tambien cierran (mismo camino
// combinacional, mas margen a menor frecuencia) -- no hace falta un sdc
// distinto por candidato.
create_clock -name clk -period 37.037 -waveform {0 18.518} [get_ports {clk}]
create_clock -name sclk -period 125 -waveform {0 62.5} [get_ports {sclk}]
set_clock_groups -asynchronous -group [get_clocks {clk}] -group [get_clocks {sclk}]

create_clock -name clk_sys -period 18.518 -waveform {0 9.259} [get_nets {u_core/clk_sys}]
set_clock_uncertainty -setup 0.5 -to [get_clocks {clk_sys}]
set_clock_uncertainty -hold  0.15 -to [get_clocks {clk_sys}]
set_clock_groups -asynchronous -group [get_clocks {clk_sys}] -group [get_clocks {sclk}]

// EXPERIMENTO -- constraints de I/O para la interfaz de la SDRAM embebida,
// que hasta ahora no tenia NINGUNA (ver comentario de sdram_pesos.cst: los
// pines son "nombres magicos" sin IO_LOC, y este .sdc nunca declaraba
// set_input_delay/set_output_delay para ellos). Sin esto, la herramienta
// de P&R no tiene ningun objetivo de timing para el camino externo
// (FPGA<->SDRAM embebida), asi que no hay garantia de que cierre igual de
// bien de una compilacion a otra -- hipotesis para la variabilidad
// observada entre builds identicos probando rd_burst2 nativo.
//
// clk_sdram: MISMO periodo que clk_sys, desfasado 180 grados (CLKOUTP de
// la PLL, ver pll_sdram.v) -- es el reloj que de verdad llega al pin
// O_sdram_clk, asi que las I/O delays de la SDRAM se miden contra este,
// no contra clk_sys.
create_clock -name clk_sdram -period 18.518 -waveform {9.259 18.518} [get_ports {O_sdram_clk}]
set_clock_groups -asynchronous -group [get_clocks {clk_sdram}] -group [get_clocks {sclk}]

// Valores basados en el testbench oficial de Gowin para esta misma SDRAM
// embebida (SDRC_EMB/GW2AR, geometria identica: 32 bits, 11 filas, 8
// columnas, 2 bancos), que la corre a ~166MHz -- clase de velocidad SDR
// SDRAM -6/-7 tipica: tAC~6ns, tOH~2.5ns (lectura, SDRAM->FPGA); tIS~1.5ns,
// tIH~0.8ns (escritura/comando, FPGA->SDRAM). Interconexion en el mismo
// encapsulado (SIP), asi que el retardo de pista se toma como ~0.
set_output_delay -clock [get_clocks {clk_sdram}] -max 1.5 [get_ports {O_sdram_addr[*] O_sdram_ba[*] O_sdram_cas_n O_sdram_ras_n O_sdram_wen_n O_sdram_dqm[*] O_sdram_cke}]
set_output_delay -clock [get_clocks {clk_sdram}] -min -0.8 [get_ports {O_sdram_addr[*] O_sdram_ba[*] O_sdram_cas_n O_sdram_ras_n O_sdram_wen_n O_sdram_dqm[*] O_sdram_cke}]
set_output_delay -clock [get_clocks {clk_sdram}] -max 1.5 [get_ports {IO_sdram_dq[*]}] -add_delay
set_output_delay -clock [get_clocks {clk_sdram}] -min -0.8 [get_ports {IO_sdram_dq[*]}] -add_delay
set_input_delay  -clock [get_clocks {clk_sdram}] -max 6.0 [get_ports {IO_sdram_dq[*]}] -add_delay
set_input_delay  -clock [get_clocks {clk_sdram}] -min 2.5 [get_ports {IO_sdram_dq[*]}] -add_delay
