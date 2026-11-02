#!/bin/sh
# Existing Ubuntu/WSL Icarus only; no installation or license changes.
set -eu
project=${1:-$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)}
cd "$project/05_Simulation"
mkdir -p smooth_results
set -- ../01_Gowin/src/*.v
if [ -d legacy_src ]; then set -- "$@" legacy_src/*.v; fi
tests='tb_integer_center tb_pendulum_v2 tb_control_safety tb_start_and_integral tb_position_pid tb_half_adc tb_balance_window5 tb_half_outer_integration tb_half_damping tb_half_integral tb_outer_integral tb_dual_integral_wiring tb_upright_toggle tb_response_limiter tb_motor_pwm_response tb_inhibit_recovery tb_derivative_rearm tb_stop_priority tb_speed_outer'
if [ -f tb_smooth_balance.v ]; then tests="$tests tb_smooth_balance"; fi
if [ -f tb_smooth_top.v ]; then tests="$tests tb_smooth_top"; fi
tests="$tests tb_swing_up_supervisor tb_auto_swing_top tb_auto_motor_ports tb_measured_direction tb_arm_inertia tb_measured_12v tb_encoder_filter tb_encoder_filter_top tb_swing_trajectory tb_trajectory_inertia tb_trajectory_supervisor tb_trajectory_replay"
tests="$tests tb_impulse_swing tb_impulse_supervisor tb_impulse_inertia"
for test in $tests; do
    iverilog -I ../01_Gowin/src -g2005 -Wall -s "$test" -o "smooth_results/$test.vvp" "$@" "$test.v" > "smooth_results/$test.compile.log" 2>&1 || { cat "smooth_results/$test.compile.log"; exit 1; }
    vvp "smooth_results/$test.vvp" > "smooth_results/$test.log" 2>&1 || { cat "smooth_results/$test.log"; exit 1; }
    cat "smooth_results/$test.log"
    if [ "$test" = tb_integer_center ]; then
        grep -q '^OLD_INTEGER_CENTER=764$' "smooth_results/$test.log"
    else
        grep -Eq '_PASS' "smooth_results/$test.log" || { echo "Missing PASS: $test"; exit 1; }
    fi
    if grep -Eiq 'FATAL|ERROR:' "smooth_results/$test.log"; then exit 1; fi
done
echo V6R5_ALL_REGRESSIONS_PASS
