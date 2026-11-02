cd [file dirname [file normalize [info script]]]
file mkdir smooth_results
transcript file smooth_results/modelsim_auto.log
if {![file exists work]} {vlib work}
vmap work work
vlog -vlog01compat +acc {*}[glob ../01_Gowin/src/*.v] legacy_src/swing_up_supervisor_v6_reference.v tb_swing_up_supervisor.v tb_auto_swing_top.v tb_auto_motor_ports.v tb_measured_direction.v tb_arm_inertia.v tb_measured_12v.v tb_encoder_filter.v tb_encoder_filter_top.v tb_swing_trajectory.v tb_trajectory_inertia.v tb_trajectory_supervisor.v tb_trajectory_replay.v
vsim -onfinish stop -voptargs=+acc work.tb_swing_up_supervisor
run -all
quit -sim
vsim -onfinish stop -voptargs=+acc work.tb_auto_swing_top
run -all
quit -sim
vsim -onfinish stop -voptargs=+acc work.tb_auto_motor_ports
run -all
quit -sim
vsim -onfinish stop -voptargs=+acc work.tb_measured_direction
run -all
quit -sim
vsim -onfinish stop -voptargs=+acc work.tb_arm_inertia
run -all
quit -sim
vlog -vlog01compat +acc tb_impulse_swing.v tb_impulse_supervisor.v tb_impulse_inertia.v
foreach top {tb_measured_12v tb_encoder_filter tb_encoder_filter_top tb_swing_trajectory tb_trajectory_inertia tb_trajectory_supervisor tb_trajectory_replay tb_impulse_swing tb_impulse_supervisor tb_impulse_inertia} {
    vsim -onfinish stop -voptargs=+acc work.$top
    run -all
    quit -sim
}
quit -f
