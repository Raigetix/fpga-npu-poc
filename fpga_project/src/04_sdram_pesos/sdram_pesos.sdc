create_clock -name clk -period 37.037 -waveform {0 18.518} [get_ports {clk}]
create_clock -name sclk -period 125 -waveform {0 62.5} [get_ports {sclk}]
set_clock_groups -asynchronous -group [get_clocks {clk}] -group [get_clocks {sclk}]

create_clock -name clk_sys -period 18.518 -waveform {0 9.259} [get_nets {clk_sys}]
set_clock_uncertainty -setup 0.5 -to [get_clocks {clk_sys}]
set_clock_uncertainty -hold  0.15 -to [get_clocks {clk_sys}]
set_clock_groups -asynchronous -group [get_clocks {clk_sys}] -group [get_clocks {sclk}]

// Restricciones de I/O para la SDRAM embebida (validadas en
// sdram_freqtest.sdc/docs/caracterizacion-frecuencia-sdram.md) -- ver ahi
// el detalle. clk_sdram: mismo periodo que clk_sys, desfasado 180 grados
// (el que de verdad llega a O_sdram_clk).
create_clock -name clk_sdram -period 18.518 -waveform {9.259 18.518} [get_ports {O_sdram_clk}]
set_clock_groups -asynchronous -group [get_clocks {clk_sdram}] -group [get_clocks {sclk}]

set_output_delay -clock [get_clocks {clk_sdram}] -max 1.5 [get_ports {O_sdram_addr[*] O_sdram_ba[*] O_sdram_cas_n O_sdram_ras_n O_sdram_wen_n O_sdram_dqm[*] O_sdram_cke}]
set_output_delay -clock [get_clocks {clk_sdram}] -min -0.8 [get_ports {O_sdram_addr[*] O_sdram_ba[*] O_sdram_cas_n O_sdram_ras_n O_sdram_wen_n O_sdram_dqm[*] O_sdram_cke}]
set_output_delay -clock [get_clocks {clk_sdram}] -max 1.5 [get_ports {IO_sdram_dq[*]}] -add_delay
set_output_delay -clock [get_clocks {clk_sdram}] -min -0.8 [get_ports {IO_sdram_dq[*]}] -add_delay
set_input_delay  -clock [get_clocks {clk_sdram}] -max 6.0 [get_ports {IO_sdram_dq[*]}] -add_delay
set_input_delay  -clock [get_clocks {clk_sdram}] -min 2.5 [get_ports {IO_sdram_dq[*]}] -add_delay
