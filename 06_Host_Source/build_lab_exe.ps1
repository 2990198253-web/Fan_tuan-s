param([string]$PythonExe='python',[string]$BuildDir='')
$ErrorActionPreference='Stop'
if(-not $BuildDir){$BuildDir=Join-Path $env:TEMP ('SwingLab_'+[guid]::NewGuid().ToString('N'))}
$dist=Join-Path (Split-Path $PSScriptRoot -Parent) '03_PC_Monitor'
& $PythonExe -m PyInstaller --noconfirm --clean --onefile --windowed --noupx --name SwingLab4 --distpath $dist --workpath (Join-Path $BuildDir 'work') --specpath $BuildDir --paths $PSScriptRoot (Join-Path $PSScriptRoot 'swing_lab.py')
if($LASTEXITCODE -ne 0){throw 'SwingLab EXE build failed'}
