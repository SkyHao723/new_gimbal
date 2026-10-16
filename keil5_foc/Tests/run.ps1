param([string]$Gcc = 'D:\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$testExe = Join-Path $projectRoot 'Objects\test_control.exe'
New-Item -ItemType Directory -Path (Split-Path -Parent $testExe) -Force | Out-Null
& $Gcc -std=c99 -Wall -Wextra -Werror `
    -I (Join-Path $projectRoot 'User') `
    -I (Join-Path $projectRoot 'BSP') `
    -I (Join-Path $projectRoot 'Control') `
    (Join-Path $projectRoot 'Control\foc_math.c') `
    (Join-Path $projectRoot 'Control\encoder_tracking.c') `
    (Join-Path $projectRoot 'Control\trajectory.c') `
    (Join-Path $projectRoot 'Control\motor.c') `
    (Join-Path $PSScriptRoot 'test_control.c') -lm -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host test build failed.' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host tests failed.' }
