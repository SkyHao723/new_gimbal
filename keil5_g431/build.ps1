param(
    [string]$KeilExe = '',
    [string]$BuildAlias = 'D:\dsh_build\keil5_g431'
)
$ErrorActionPreference = 'Stop'
if (-not $KeilExe) {
    $candidates = @($env:KEIL_UV4, 'D:\keil\UV4\UV4.exe', 'C:\Keil_v5\UV4\UV4.exe')
    $KeilExe = $candidates | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1
}
if (-not $KeilExe) { throw 'Specify -KeilExe with the full path to UV4.exe.' }
if ($BuildAlias -match '[^\x00-\x7F]') { throw 'BuildAlias must use an ASCII path.' }
$projectRoot = $PSScriptRoot
if (-not (Test-Path -LiteralPath $BuildAlias)) {
    $aliasParent = Split-Path -Parent $BuildAlias
    New-Item -ItemType Directory -Path $aliasParent -Force | Out-Null
    New-Item -ItemType Junction -Path $BuildAlias -Target $projectRoot | Out-Null
} else {
    $aliasItem = Get-Item -LiteralPath $BuildAlias
    if ($aliasItem.LinkType -ne 'Junction' -or [string]$aliasItem.Target -ne $projectRoot) {
        throw 'BuildAlias already exists and does not point to this project. Choose a different alias.'
    }
}
New-Item -ItemType Directory -Path (Join-Path $projectRoot 'Objects') -Force | Out-Null
$project = Join-Path $BuildAlias 'gimbal_g431.uvprojx'
$log = Join-Path $BuildAlias 'Objects\keil_build.log'
$arguments = '-r "{0}" -t G431CB_FOC -o "{1}"' -f $project, $log
$process = Start-Process -FilePath $KeilExe -ArgumentList $arguments -WindowStyle Hidden -PassThru -Wait
if (Test-Path -LiteralPath $log) { Get-Content -LiteralPath $log }
if ($process.ExitCode -ne 0) { throw "Keil build failed with exit code $($process.ExitCode)." }
Write-Output ('HEX: ' + (Join-Path $projectRoot 'Objects\gimbal_g431.hex'))
