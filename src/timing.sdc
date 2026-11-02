# Constrain internal control logic to the board's 50 MHz clock.
# External ADC input/output timing still requires the ADC board timing data.
create_clock -name clk_50m -period 20.000 [get_ports {clk_50m}]
