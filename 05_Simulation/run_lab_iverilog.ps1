param([string]$IcarusBin='')
$ErrorActionPreference='Stop'
$compiler=if($IcarusBin){Join-Path $IcarusBin 'iverilog.exe'}else{(Get-Command iverilog -ErrorAction Stop).Source}
$runner=if($IcarusBin){Join-Path $IcarusBin 'vvp.exe'}else{(Get-Command vvp -ErrorAction Stop).Source}
$sources=@(Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot '..\01_Gowin\src') -Filter '*.v'|ForEach-Object FullName)
$sources+=@(Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot 'golden') -Filter '*.v'|ForEach-Object FullName)
Push-Location -LiteralPath $PSScriptRoot
try{
    New-Item -ItemType Directory -Path smooth_results -Force|Out-Null
    foreach($test in @('tb_lab_controller','tb_lab_config','tb_lab_uart','tb_lab_top','tb_energy_strategy','tb_energy_supervisor','tb_velocity_relay','tb_relay_supervisor','tb_precapture','tb_catch_supervisor','tb_capture_replay','tb_zero_calibrator','tb_reference_bridge','tb_reference_equivalence','tb_reference_top','tb_reference_replay','tb_locked_config')){
        & $compiler -I ../01_Gowin/src -g2005 -Wall -s $test -o "smooth_results/$test.vvp" @sources "$test.v"
        if($LASTEXITCODE -ne 0){throw "Compile failed: $test"}
        $result=& $runner "smooth_results/$test.vvp" 2>&1
        $result|Tee-Object -FilePath "smooth_results/$test.log"
        if($LASTEXITCODE -ne 0 -or ($result -join "`n") -notmatch '_PASS' -or ($result -join "`n") -match 'FATAL|ERROR:'){throw "Test failed: $test"}
    }
    Write-Host 'SWING_LAB_REGRESSIONS_PASS'
}finally{Pop-Location}
