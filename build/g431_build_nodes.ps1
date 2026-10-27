# Build one hex per CAN node id (default 1..3) without hand-editing board_config.h.
# ASCII only: Windows PowerShell 5.1 mis-decodes UTF-8 CJK in .ps1 files.
#
#   powershell -ExecutionPolicy Bypass -File build\g431_build_nodes.ps1
#   powershell -ExecutionPolicy Bypass -File build\g431_build_nodes.ps1 -Nodes 1,2,3,4
#
# Output: keil5_g431\Objects\gimbal_g431_node<N>.hex
# The board_config.h is restored to its original node id at the end (even on error).
param([int[]]$Nodes = @(1, 2, 3))

$ErrorActionPreference = 'Stop'

# ASCII junction to the real project directory (Chinese path breaks PS 5.1).
$root = 'D:\dsh_build\ng'
if (-not (Test-Path $root)) {
    throw "need the ASCII junction $root -> New_gimbal (mklink /J)"
}
$proj = Join-Path $root 'keil5_g431\gimbal_g431.uvprojx'
$cfg = Join-Path $root 'keil5_g431\BSP\board_config.h'
$out = Join-Path $root 'keil5_g431\Objects'
$uv = 'D:\keil\UV4\UV4.exe'
foreach ($p in @($proj, $cfg, $uv)) {
    if (-not (Test-Path $p)) { throw "missing: $p" }
}

$utf8 = New-Object System.Text.UTF8Encoding($false)
$original = [System.IO.File]::ReadAllText($cfg)

try {
    foreach ($n in $Nodes) {
        if ($n -lt 1 -or $n -gt 7) { throw "node id must be 1..7 (got $n)" }
        $patched = [regex]::Replace($original,
            '(?m)^#define\s+BOARD_CAN_NODE\s+\d+u',
            "#define BOARD_CAN_NODE ${n}u")
        if ($patched -notmatch "BOARD_CAN_NODE\s+${n}u") {
            throw "could not patch BOARD_CAN_NODE in $cfg"
        }
        [System.IO.File]::WriteAllText($cfg, $patched, $utf8)

        $log = Join-Path $out "build_node$n.log"
        if (Test-Path $log) { Remove-Item $log -Force }
        $proc = Start-Process -FilePath $uv -ArgumentList @('-b', $proj, '-o', $log) `
                              -Wait -PassThru -WindowStyle Hidden
        $text = ''
        if (Test-Path $log) { $text = [System.IO.File]::ReadAllText($log) }
        if ($text -notmatch '0 Error\(s\)') {
            throw "node $n build failed (UV4 exit $($proc.ExitCode)), see $log"
        }
        $hex = Join-Path $out "gimbal_g431_node$n.hex"
        Copy-Item (Join-Path $out 'gimbal_g431.hex') $hex -Force
        $size = (Get-Item $hex).Length
        Write-Output ("node {0}: cmd=0x{1:X3} reply=0x{2:X3} -> {3} ({4} bytes)" -f `
            $n, (0x200 + $n), (0x280 + $n), $hex, $size)
    }
}
finally {
    [System.IO.File]::WriteAllText($cfg, $original, $utf8)
    # Keep the generic gimbal_g431.hex pointing at node 1 so that any older
    # flash command keeps behaving as before.
    $n1 = Join-Path $out 'gimbal_g431_node1.hex'
    if (Test-Path $n1) { Copy-Item $n1 (Join-Path $out 'gimbal_g431.hex') -Force }
    Write-Output "board_config.h restored (gimbal_g431.hex = node 1)"
}
