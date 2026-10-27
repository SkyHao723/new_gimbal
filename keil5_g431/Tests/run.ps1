param([string]$Gcc = 'D:\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$g431Root = Split-Path -Parent $PSScriptRoot
$sharedRoot = Join-Path (Split-Path -Parent $g431Root) 'keil5_foc'
$testExe = Join-Path $g431Root 'Objects\test_control.exe'
New-Item -ItemType Directory -Path (Split-Path -Parent $testExe) -Force | Out-Null
& $Gcc -std=c99 -Wall -Wextra -Werror `
    -I (Join-Path $g431Root 'User') `
    -I (Join-Path $g431Root 'BSP') `
    -I (Join-Path $sharedRoot 'Control') `
    (Join-Path $sharedRoot 'Control\foc_math.c') `
    (Join-Path $sharedRoot 'Control\encoder_tracking.c') `
    (Join-Path $sharedRoot 'Control\trajectory.c') `
    (Join-Path $sharedRoot 'Control\motor.c') `
    (Join-Path $sharedRoot 'Tests\test_control.c') -lm -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'G431 host test build failed.' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'G431 host tests failed.' }
$streamExe = Join-Path $g431Root 'Objects\test_can_commands.exe'
& $Gcc -std=c99 -Wall -Wextra -Werror `
    -I (Join-Path $g431Root 'BSP') `
    (Join-Path $g431Root 'BSP\can_commands.c') `
    (Join-Path $PSScriptRoot 'test_can_commands.c') -o $streamExe
if ($LASTEXITCODE -ne 0) { throw 'CAN command test build failed.' }
& $streamExe
if ($LASTEXITCODE -ne 0) { throw 'CAN command tests failed.' }
$limitExe = Join-Path $g431Root 'Objects\test_current_limit.exe'
& $Gcc -std=c99 -Wall -Wextra -Werror `
    -I (Join-Path $g431Root 'User') `
    (Join-Path $g431Root 'User\current_limit.c') `
    (Join-Path $PSScriptRoot 'test_current_limit.c') -lm -o $limitExe
if ($LASTEXITCODE -ne 0) { throw 'Current limit test build failed.' }
& $limitExe
if ($LASTEXITCODE -ne 0) { throw 'Current limit tests failed.' }
