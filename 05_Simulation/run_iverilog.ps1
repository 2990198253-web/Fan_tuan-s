param([string]$IcarusBin = '')
$ErrorActionPreference = 'Stop'
if ($IcarusBin) {
    $compiler = Join-Path $IcarusBin 'iverilog.exe'
    $runner = Join-Path $IcarusBin 'vvp.exe'
} else {
    $compiler = (Get-Command iverilog -ErrorAction Stop).Source
    $runner = (Get-Command vvp -ErrorAction Stop).Source
}
$sourceDir = Join-Path $PSScriptRoot '..\01_Gowin\src'
$sources = @(Get-ChildItem -LiteralPath $sourceDir -Filter '*.v' | ForEach-Object { $_.FullName })
$legacyDir = Join-Path $PSScriptRoot 'legacy_src'
if (Test-Path -LiteralPath $legacyDir) {
    $sources += @(Get-ChildItem -LiteralPath $legacyDir -Filter '*.v' | ForEach-Object { $_.FullName })
}
$outputDir = Join-Path $PSScriptRoot 'icarus_results'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
Push-Location -LiteralPath $PSScriptRoot
try {
foreach ($test in @('tb_integer_center','tb_pendulum_v2','tb_control_safety','tb_start_and_integral','tb_position_pid','tb_half_adc','tb_balance_window5','tb_half_outer_integration','tb_half_damping','tb_half_integral','tb_outer_integral','tb_dual_integral_wiring','tb_upright_toggle','tb_response_limiter','tb_motor_pwm_response','tb_inhibit_recovery','tb_derivative_rearm','tb_stop_priority','tb_speed_outer','tb_smooth_balance','tb_smooth_top','tb_swing_up_supervisor','tb_auto_swing_top','tb_auto_motor_ports','tb_measured_direction','tb_arm_inertia','tb_measured_12v','tb_encoder_filter','tb_encoder_filter_top','tb_swing_trajectory','tb_trajectory_inertia','tb_trajectory_supervisor','tb_trajectory_replay','tb_impulse_swing','tb_impulse_supervisor','tb_impulse_inertia')) {
    $compiled = Join-Path $outputDir ($test + '.vvp')
    # Match Gowin's Verilog mode: the existing telemetry signal "sequence"
    # is a keyword only in SystemVerilog.
    & $compiler -g2005 -Wall -s $test -o $compiled @sources (Join-Path $PSScriptRoot ($test + '.v'))
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $test" }
    New-Item -ItemType Directory -Path (Join-Path $PSScriptRoot 'smooth_results') -Force | Out-Null
    $result = & $runner $compiled 2>&1
    $result | Tee-Object -FilePath (Join-Path $outputDir ($test + '.log'))
    if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $test" }
    $resultText = $result -join "`n"
    if ($resultText -notmatch '_PASS' -or $resultText -match 'FATAL|ERROR:') { throw "Missing PASS or runtime error: $test" }
}
Write-Host 'V6R5_ALL_REGRESSIONS_PASS'
} finally { Pop-Location }
