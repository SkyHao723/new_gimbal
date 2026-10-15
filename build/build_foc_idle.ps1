# ============================================================================
# Build FOC_IDLE_TEST firmware (stage 2: FOC library integration, motor NOT driven)
#
# Usage:  powershell -ExecutionPolicy Bypass -File build\build_foc_idle.ps1
# Output: build\out_foc\foc_idle.hex
#
# NOTE: keep this file ASCII-only (PowerShell 5.1 mis-decodes UTF-8 CJK).
# ============================================================================
$ErrorActionPreference = 'Stop'

$ROOT = Split-Path -Parent $PSScriptRoot
$SRC  = Join-Path $ROOT 'app'
$LIB  = Join-Path $ROOT 'library'
$BLD  = Join-Path $ROOT 'build'
$OUT  = Join-Path $BLD  'out_foc'

# ---- locate toolchain ----
$GCC = $null
$cands = @()
if($env:ARM_GCC_BIN){ $cands += (Join-Path $env:ARM_GCC_BIN 'arm-none-eabi-gcc.exe') }
$cands += 'D:\program\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe'
foreach($dir in @('C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeIDE','D:\ST','C:\ST')){
  if(Test-Path $dir){
    $found = Get-ChildItem $dir -Recurse -Filter 'arm-none-eabi-gcc.exe' -ErrorAction SilentlyContinue |
             Select-Object -First 1
    if($found){ $cands += $found.FullName }
  }
}
foreach($c in $cands){ if($c -and (Test-Path $c)){ $GCC = $c; break } }
if(-not $GCC){
  $cmd = Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue
  if($cmd){ $GCC = $cmd.Source }
}
if(-not $GCC){
  throw "arm-none-eabi-gcc not found. Set env ARM_GCC_BIN or edit the candidate paths."
}

Write-Host "toolchain : $GCC"
Write-Host "repo root : $ROOT"
Write-Host ""

if(Test-Path $OUT){ Remove-Item $OUT -Recurse -Force }
New-Item -ItemType Directory -Force -Path $OUT | Out-Null

# NOTE: $SRC must come before $SRC\SimpleFOC so that the shim MyProject.h
#       in app/ wins over any reference copy.
$CFLAGS = @(
  '-mcpu=cortex-m3','-mthumb','-Os','-g0','-ffunction-sections','-fdata-sections',
  '-DUSE_STDPERIPH_DRIVER','-DSTM32F10X_MD','-w',
  "-I$SRC","-I$SRC\SimpleFOC","-I$SRC\Hardware",
  "-I$LIB\lib\inc","-I$LIB\startup","-I$BLD"
)

$sources = @(
  "$BLD\startup_gcc.s",
  "$BLD\syscalls.c",
  "$BLD\clock.c",
  "$SRC\foc_idle_test.c",
  "$SRC\encoder.c",
  # FOC library
  "$SRC\SimpleFOC\foc_utils.c",
  "$SRC\SimpleFOC\FOCMotor.c",
  "$SRC\SimpleFOC\BLDCMotor.c",
  "$SRC\SimpleFOC\pid.c",
  "$SRC\SimpleFOC\lowpass_filter.c",
  # Hardware
  "$SRC\Hardware\timer.c",
  "$SRC\Hardware\delay.c",
  # SPL
  "$LIB\startup\system_stm32f10x.c",
  "$LIB\lib\src\misc.c",
  "$LIB\lib\src\stm32f10x_rcc.c",
  "$LIB\lib\src\stm32f10x_gpio.c",
  "$LIB\lib\src\stm32f10x_tim.c",
  "$LIB\lib\src\stm32f10x_usart.c",
  "$LIB\lib\src\stm32f10x_flash.c",
  "$LIB\lib\src\stm32f10x_i2c.c",
  "$LIB\lib\src\stm32f10x_iwdg.c"
)

$objs = New-Object System.Collections.ArrayList
foreach($s in $sources){
  $name = [IO.Path]::GetFileNameWithoutExtension($s)
  $o = Join-Path $OUT "$name.o"
  Write-Host "CC  $name"
  & $GCC @CFLAGS -c $s -o $o
  if($LASTEXITCODE -ne 0){ throw "compile failed: $s" }
  [void]$objs.Add($o)
}

Write-Host ""
Write-Host "LD  foc_idle.elf"
$link = @(
  '-mcpu=cortex-m3','-mthumb','-w',
  '-T', "$BLD\stm32f103c8.ld",
  '-Wl,--gc-sections', '--specs=nano.specs',
  '-Wl,--start-group','-lc','-lm','-Wl,--end-group',
  '-u','_printf_float'
)
& $GCC @($objs + $link + @('-o', "$OUT\foc_idle.elf"))
if($LASTEXITCODE -ne 0){ throw "link failed" }

$BIN = Split-Path -Parent $GCC
& (Join-Path $BIN 'arm-none-eabi-objcopy.exe') -O ihex "$OUT\foc_idle.elf" "$OUT\foc_idle.hex"
& (Join-Path $BIN 'arm-none-eabi-size.exe') "$OUT\foc_idle.elf"

Write-Host ""
Write-Host "Artifacts:"
Get-ChildItem $OUT -File | Where-Object { $_.Extension -eq '.hex' } |
  ForEach-Object { "  $($_.Name)  $($_.Length) B" }
Write-Host ""
Write-Host "Flash (ST-Link):"
Write-Host "  STM32_Programmer_CLI -c port=SWD -w build\out_foc\foc_idle.hex -v -g 0x08000000"
