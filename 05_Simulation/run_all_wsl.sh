#!/bin/sh
set -eu
cd "$(dirname "$0")"
sh run_regression_wsl.sh
sh run_lab_wsl.sh
sh run_r1_wsl.sh
sh run_lab4_wsl.sh
echo SWING_LAB4_R1_ALL_53_TESTS_PASS
