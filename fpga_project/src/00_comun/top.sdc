create_clock -name clk -period 37.037 -waveform {0 18.518} [get_ports {clk}]
# 8MHz: el ESP32 puede correr mas lento sin problema, esto es cota superior
create_clock -name sclk -period 125 -waveform {0 62.5} [get_ports {sclk}]

# clk y sclk NO tienen relacion de fase entre si (sclk lo genera el ESP32,
# de forma completamente independiente al oscilador de la FPGA). Sin esto,
# el analisis estatico de timing puede evaluar los caminos que cruzan entre
# los dos dominios asumiendo una relacion de fase que en realidad no existe
# -- el cruce real se resuelve por los sincronizadores en RTL (ver
# spi_slave.v), no por timing de estas dos senales entre si.
set_clock_groups -asynchronous -group [get_clocks {clk}] -group [get_clocks {sclk}]

# Reloj interno "clk_sys": el nombre del net es igual en top_mlp_fast.v (PLL
# pll_108mhz, 27->54MHz) y en top_mlp_par.v (PLL pll_par) -- esta constraint
# se aplica a cualquiera de los dos que este activo como top_module (Gowin
# solo elabora el seleccionado). Si se vuelve a sintetizar top_mlp_fast.v
# hay que devolver esto a 18.518 (54MHz) o el STA quedara sub-restringido
# para ese diseno.
#
# VALOR ACTUAL: 108MHz (period 9.259), para top_mlp_par.v -- techo real
# encontrado con un barrido de frecuencia (27/54/108/216, doblando cada
# vez): 108MHz es el ultimo paso limpio (0 violaciones, funciona en placa),
# 216MHz ya rompe (confirmado con violaciones reales de timing Y
# corrupcion real en placa). Esto fue DESPUES de confirmar que la
# corrupcion intermitente que se veia antes en la carga de pesos era
# cableado de SPI, no timing (ver README del proyecto).
#
# Las variantes de demo mas grandes (top_mlp_par_s1/s2/s3.v, bus de
# direcciones de 13 bits) usan su PROPIA constraint dedicada a 54MHz (ver
# situaciones.sdc) porque el bus mas ancho ya no entra en el presupuesto de
# 108MHz -- no se toca este valor para no arriesgar el modelo probado.
create_clock -name clk_sys -period 9.259 -waveform {0 4.6296} [get_nets {clk_sys}]
set_clock_uncertainty -setup 0.5 -to [get_clocks {clk_sys}]
set_clock_uncertainty -hold  0.15 -to [get_clocks {clk_sys}]
set_clock_groups -asynchronous -group [get_clocks {clk_sys}] -group [get_clocks {sclk}]
