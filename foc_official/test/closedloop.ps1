$ErrorActionPreference='Continue'
$port = New-Object System.IO.Ports.SerialPort COM8,115200,None,8,one

function TxWait($c, $ms){
  $port.Write($c + "`r`n")
  Start-Sleep -Milliseconds $ms
  $n = $port.BytesToRead
  if($n -gt 0){
    $b = New-Object byte[] $n
    $port.Read($b,0,$n) | Out-Null
    return [System.Text.Encoding]::ASCII.GetString($b)
  }
  return ""
}
function GetAng {
  $r = TxWait 'R' 250
  if($r -match 'angle=(-?[0-9]+\.[0-9]+)'){ return [double]$Matches[1] }
  return $null
}
function Clean($s){ return ($s -replace "`r`n", " ").Trim() }

$port.Open()
Start-Sleep -Milliseconds 800
$n = $port.BytesToRead
if($n){ $b = New-Object byte[] $n; $port.Read($b,0,$n) | Out-Null }

[void](TxWait 'D' 400)
[void](TxWait 'V3' 400)
Write-Output ("start: " + (Clean (TxWait '?' 400)))
Write-Output ""

$base = GetAng
Write-Output (">>> ENABLE  (base=" + $base.ToString("F4") + ")")
Write-Output ("    " + (Clean (TxWait 'E' 600)))
Start-Sleep -Milliseconds 1000
$a = GetAng
Write-Output ("    angle after enable = " + $a.ToString("F4"))
Write-Output ""

Write-Output "--- small steps (0.2 rad), hold 1.5s each ---"
$total = 0.0
foreach($i in 1..5){
  $tgt = [math]::Round($a + 0.2, 4)
  [void](TxWait ("T" + $tgt) 250)
  Start-Sleep -Milliseconds 1500
  $b2 = GetAng
  $d = $b2 - $a
  $total += $d
  Write-Output ("  step " + $i + ": moved=" + $d.ToString("F4") + "  err=" + ($tgt - $b2).ToString("F4"))
  $a = $b2
}
Write-Output ("  total moved = " + $total.ToString("F4") + "  (expected 1.0000)")
Write-Output ""

Write-Output "--- hold test: stay at current target 3s ---"
$holdStart = GetAng
Start-Sleep -Milliseconds 3000
$holdEnd = GetAng
Write-Output ("  drift over 3s = " + ($holdEnd - $holdStart).ToString("F4") + " rad")
Write-Output ""

Write-Output "--- step back -0.5 rad ---"
$tgt = [math]::Round($holdEnd - 0.5, 4)
[void](TxWait ("T" + $tgt) 250)
Start-Sleep -Milliseconds 2000
$b3 = GetAng
Write-Output ("  moved=" + ($b3 - $holdEnd).ToString("F4") + "  err=" + ($tgt - $b3).ToString("F4"))
Write-Output ""

Write-Output ">>> DISABLE"
Write-Output ("    " + (Clean (TxWait 'D' 500)))
Write-Output ("final: " + (Clean (TxWait '?' 400)))

$port.Close()
