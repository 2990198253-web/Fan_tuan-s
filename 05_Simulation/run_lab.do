cd [file dirname [file normalize [info script]]]
file mkdir smooth_results
transcript file smooth_results/modelsim_lab.log
onerror {quit -code 1 -f}
if {![file exists work]} {vlib work}
vmap work work
vlog +incdir+../01_Gowin/src -vlog01compat +acc {*}[glob ../01_Gowin/src/*.v] {*}[glob golden/*.v] tb_lab_controller.v tb_lab_config.v tb_lab_uart.v tb_lab_top.v tb_energy_strategy.v tb_energy_supervisor.v tb_velocity_relay.v tb_relay_supervisor.v tb_precapture.v tb_catch_supervisor.v tb_capture_replay.v tb_zero_calibrator.v tb_reference_bridge.v tb_reference_equivalence.v tb_reference_top.v tb_reference_replay.v
vlog +incdir+../01_Gowin/src -vlog01compat +acc tb_locked_config.v
foreach top {tb_lab_controller tb_lab_config tb_lab_uart tb_lab_top tb_energy_strategy tb_energy_supervisor tb_velocity_relay tb_relay_supervisor tb_precapture tb_catch_supervisor tb_capture_replay tb_zero_calibrator tb_reference_bridge tb_reference_equivalence tb_reference_top tb_reference_replay tb_locked_config} {
    vsim -onfinish stop -voptargs=+acc work.$top
    run -all
    quit -sim
}
quit -f
