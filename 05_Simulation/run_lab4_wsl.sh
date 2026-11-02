#!/bin/sh
set -eu
cd "$(dirname "$0")"
mkdir -p smooth_results
for t in tb_zero_calibrator tb_reference_bridge tb_reference_equivalence tb_reference_top tb_reference_replay tb_locked_config; do
  iverilog -I ../01_Gowin/src -g2005 -Wall -s "$t" -o "smooth_results/$t.vvp" ../01_Gowin/src/*.v golden/*.v "$t.v" > "smooth_results/$t.compile.log" 2>&1 || { cat "smooth_results/$t.compile.log"; exit 1; }
  vvp "smooth_results/$t.vvp" > "smooth_results/$t.log" 2>&1 || { cat "smooth_results/$t.log"; exit 1; }
  cat "smooth_results/$t.log"
  grep -q '_PASS' "smooth_results/$t.log"
  if grep -Eq 'FATAL|ERROR:' "smooth_results/$t.log"; then exit 1; fi
done
echo LAB4_NEW_TESTS_PASS
