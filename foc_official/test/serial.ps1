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

try{
  $port.Open()
  Start-Sleep -Milliseconds 800
  $n=$port.BytesToRead; if($n){ $b=New-Object byte[] $n; $port.Read($b,0,$n)|Out-Null }

  Write-Output "================================================================"
  Write-Output "  SERIAL RELIABILITY TEST -- 20 commands, count responses"
  Write-Output "================================================================"
  Write-Output ""

  # ---- 测试 1: 连续发 20 条 R（每条应返回 sensor angle=...）----
  $ok = 0
  for($i=1; $i -le 20; $i++){
    $r = TxWait 'R' 200
    if($r -match 'angle='){ $ok++ }
    else { Write-Output ("  #" + $i + " NO RESPONSE") }
  }
  Write-Output ("TEST 1 (20x R):    " + $ok + "/20  OK")
  Write-Output ""

  # ---- 测试 2: 连续发 20 条 ?（每条应返回 EN=...）----
  $ok2 = 0
  for($i=1; $i -le 20; $i++){
    $r = TxWait '?' 200
    if($r -match 'EN='){ $ok2++ }
    else { Write-Output ("  #" + $i + " NO RESPONSE") }
  }
  Write-Output ("TEST 2 (20x ?):    " + $ok2 + "/20  OK")
  Write-Output ""

  # ---- 测试 3: 快速连发 10 条（不加延时，考验缓冲）----
  for($i=1; $i -le 10; $i++){ $port.Write("R`r`n") }
  Start-Sleep -Milliseconds 900
  $n=$port.BytesToRead
  $fast = 0
  if($n -gt 0){
    $b=New-Object byte[] $n; $port.Read($b,0,$n)|Out-Null
    $txt = [System.Text.Encoding]::ASCII.GetString($b)
    $fast = ([regex]::Matches($txt,'angle=')).Count
  }
  Write-Output ("TEST 3 (burst 10x R, no delay): " + $fast + "/10  OK")
  Write-Output ""

  # ---- 测试 4: 验证参数命令真的生效 ----
  Write-Output "TEST 4: parameter commands actually take effect"
  [void](TxWait 'V2.5' 400)
  $r = TxWait '?' 400
  if($r -match 'Vlim=([0-9.]+)'){ Write-Output ("  set V2.5  -> status Vlim=" + $Matches[1]) }
  [void](TxWait 'V3' 400)
  $r = TxWait '?' 400
  if($r -match 'Vlim=([0-9.]+)'){ Write-Output ("  set V3    -> status Vlim=" + $Matches[1]) }
  $r = TxWait 'T1.234' 400
  Write-Output ("  set T1.234 -> " + ($r -replace "`r`n"," ").Trim())
  $r = TxWait '?' 400
  if($r -match 'tgt=([-0-9.]+)'){ Write-Output ("             status tgt=" + $Matches[1]) }

  Write-Output ""
  Write-Output "================================================================"
  if($ok -eq 20 -and $ok2 -eq 20 -and $fast -eq 10){
    Write-Output "  RESULT: SERIAL FIX CONFIRMED - 100% reliable"
  } else {
    Write-Output ("  RESULT: still losses  (R=" + $ok + "/20  ?=" + $ok2 + "/20  burst=" + $fast + "/10)")
  }
  Write-Output "================================================================"

}catch{ Write-Output ("ERR: " + $_.Exception.Message) }
finally{ try{$port.Close()}catch{} }
