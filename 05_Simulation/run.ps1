$ErrorActionPreference = 'Stop'
Push-Location -LiteralPath $PSScriptRoot
try {
    & vsim -c -do run.do
    if ($LASTEXITCODE -ne 0) { throw "vsim exited with $LASTEXITCODE" }
    $result = Get-Content -LiteralPath 'smooth_results\modelsim_auto.log' -Raw
    if ($result -notmatch 'V6_AUTO_SWING_TOP_PASS') { throw 'Missing pass marker' }
    if ($result -match '(?m)^\s*(\*\* Error:|\*\* Fatal:|Error:)') {
        throw 'Simulator reported an error'
    }
    Write-Host 'V6_AUTO_SWING_TOP_PASS'
} finally {
    Pop-Location
}
