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
