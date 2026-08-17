// situaciones.sdc -- constraint de clk_sys dedicada a top_mlp_par_s1/s2/s3.v
// (demo de las 3 arquitecturas, ver README). Estos 3 modelos usan un bus
// de direcciones de pesos mas ancho (13 bits, para pasar de 4096 a 8192
// direcciones/carril) que alarga el camino critico lo suficiente como
// para que 108MHz (el techo del modelo original de 12 bits) ya no
// alcance: STA mostraba -8ns de slack, 25 violaciones. Se usa 54MHz (ya
// probado reliable en el barrido de frecuencia del modelo original) en vez
// de tocar pll_par.v/top.sdc, que quedan reservados para top_mlp_par.v.
// El puerto 'clk' fisico y 'sclk' del ESP32 son los mismos de siempre
// (comparten top.cst, que no cambia entre variantes).
create_clock -name clk -period 37.037 -waveform {0 18.518} [get_ports {clk}]
create_clock -name sclk -period 125 -waveform {0 62.5} [get_ports {sclk}]
set_clock_groups -asynchronous -group [get_clocks {clk}] -group [get_clocks {sclk}]

create_clock -name clk_sys -period 18.518 -waveform {0 9.259} [get_nets {clk_sys}]
set_clock_uncertainty -setup 0.5 -to [get_clocks {clk_sys}]
set_clock_uncertainty -hold  0.15 -to [get_clocks {clk_sys}]
set_clock_groups -asynchronous -group [get_clocks {clk_sys}] -group [get_clocks {sclk}]
