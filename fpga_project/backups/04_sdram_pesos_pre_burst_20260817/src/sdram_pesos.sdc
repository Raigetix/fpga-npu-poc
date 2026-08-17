create_clock -name clk -period 37.037 -waveform {0 18.518} [get_ports {clk}]
create_clock -name sclk -period 125 -waveform {0 62.5} [get_ports {sclk}]
set_clock_groups -asynchronous -group [get_clocks {clk}] -group [get_clocks {sclk}]

create_clock -name clk_sys -period 18.518 -waveform {0 9.259} [get_nets {clk_sys}]
set_clock_uncertainty -setup 0.5 -to [get_clocks {clk_sys}]
set_clock_uncertainty -hold  0.15 -to [get_clocks {clk_sys}]
set_clock_groups -asynchronous -group [get_clocks {clk_sys}] -group [get_clocks {sclk}]
