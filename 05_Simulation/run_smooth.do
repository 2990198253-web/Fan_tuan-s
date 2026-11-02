# Can be launched from any ModelSim working directory.
cd [file dirname [file normalize [info script]]]
file mkdir smooth_results
transcript file smooth_results/modelsim_smooth.log
if {![file exists work]} {vlib work}
vmap work work
vlog -vlog01compat +acc {*}[glob ../01_Gowin/src/*.v] legacy_src/pendulum_v2_top.v tb_smooth_balance.v tb_smooth_top.v
vsim -onfinish stop -voptargs=+acc work.tb_smooth_balance
run -all
quit -sim
vsim -onfinish stop -voptargs=+acc work.tb_smooth_top
run -all
quit -f
