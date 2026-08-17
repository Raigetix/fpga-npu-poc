create_clock -name clk -period 37.037 -waveform {0 18.518} [get_ports {clk}]
create_clock -name clk_sys -period 37.037 -waveform {0 18.518} [get_nets {clk_sys}]
set_clock_uncertainty -setup 0.5 -to [get_clocks {clk_sys}]
set_clock_uncertainty -hold  0.15 -to [get_clocks {clk_sys}]
