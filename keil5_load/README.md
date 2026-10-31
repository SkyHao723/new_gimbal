# 负载板固件（G431 + ICM-42670-P → CAN 遥测）

这块板是**早期的上位机板**（硬件与现在的上位机板同款），现在挂在 CAN 最末端，
只做一件事：**按 100 Hz 把自身 IMU 数据发到 CAN 总线上**，由上位机板（`keil5_host`）
接收、并且网页控制台（`tools/host_console/`）会显示成「负载板 IMU」卡片。

之前它被临时当作总线终端电阻使用（未通电，只靠板上的 120 Ω 给总线收尾）。

## 1. 硬件

| 项目 | 说明 |
|---|---|
| MCU | STM32G431CBT6 / 170 MHz |
| CAN | FDCAN1：**PB8 = RX, PB9 = TX**（AF9），TJA1042T，500 kbit/s，板上 120 Ω 终端 |
| IMU | ICM-42670-P，I2C 接 **PB6 = SCL, PB7 = SDA**（bit-bang，原因见下） |
| 串口 | PA9/PA10 → CH343P → USB（调试控制台，115200 8N1） |
| 指示 | PA5 LED（低电平点亮） |
| 供电 | 板子 5V + 3.3V 由系统 5V 母线供给（TJA1042 的 VCC 是 5V，VIO 是 3.3V） |

> **PB6 不能做硬件 I2C1_SCL**：G431 的 I2C1_SCL 只在 PA13/PA15/PB8，
> I2C1_SDA 在 PA14/PB7/PB9。本板 PB6/PB7 接 IMU，因此必须用 GPIO 开漏 bit-bang
> （`keil5_host/BSP/icm42670.c`，半位延时 2 µs ≈ 250 kHz，一次读 accel+gyro+temp 约 1.06 ms）。
> 同一原因也解释了为什么三块驱动板的 G431 一直"硬件 I2C 不通"。

## 2. CAN 帧格式（避开上位机已用的 0x200~0x207 / 0x280~0x287）

| CAN ID | 内容 | 频率 | 数据 |
|---|---|---|---|
| `0x2A0` | 加速度 | 100 Hz | `[0]=seq` `[1..6]=ax,ay,az`（int16 LE，单位 mg）`[7]=imu_ok` |
| `0x2A1` | 角速度 + 温度 | 100 Hz | `[0]=seq` `[1..6]=gx,gy,gz`（int16 LE，单位 0.1 dps）`[7]=温度`（int8，°C） |
| `0x2A2` | 状态 | 1 Hz | `[0..1]=rate_hz`（u16 LE）`[2]=telemetry_on` `[3]=imu_ok` `[4..7]=samples`（u32 LE） |

上位机收到后 `k` 命令打印：

```
[LOAD] seq=42 age=12ms frames=980 samples=490 rate=100Hz on=1 imu=1
[LOAD] acc[g] 0.004 -0.011 0.998  gyro[dps] -0.3 0.1 0.4  temp=28 C
```

## 3. 编译与烧录

```powershell
# 生成/更新 Keil 工程（从 host 工程派生，一般不用重跑）
D:\dsh_build\make_load_proj.ps1

# 编译
D:\keil\UV4\UV4.exe -b D:\dsh_build\ng\keil5_load\load_board.uvprojx -o D:\dsh_build\load_build.log
```

这块板用 **DAPLink（Horco CMSIS-DAP v2）**，Keil 走 CMSIS-DAP 会报
`Internal DLL Error`，所以改用 PlatformIO 自带的 OpenOCD：

```powershell
$ocd = 'C:\Users\SkyHao\.platformio\packages\tool-openocd\bin\openocd.exe'
$scripts = 'C:\Users\SkyHao\.platformio\packages\tool-openocd\openocd\scripts'
& $ocd -s $scripts -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg `
      -c "adapter speed 1000" `
      -c "program D:/dsh_build/ng/keil5_load/Objects/load_board.hex verify reset exit"
```

读 RAM 里的计数器（不接串口也能看板子状态）：

```powershell
& $ocd -s $scripts -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg `
      -c "adapter speed 1000" -c "init" -c "halt" `
      -c "mdw 0x2000001c 6" -c "resume" -c "shutdown"
```

## 4. 控制台命令（串口 115200）

| 命令 | 作用 |
|---|---|
| `h` / `?` | 帮助 |
| `s` | 状态一行（采样数 / 发送成败 / IMU 错误 / 速率） |
| `m` | 立即读一次 IMU 并打印 |
| `r<Hz>` | 设上报速率（10~500，默认 100） |
| `o<0\|1>` | 遥测开关 |
| `l` | 翻 LED |

## 5. 排障记录（2026-10-31，重要）

**现象**：负载板固件发送全部失败，`tx_fail` 狂涨、`PSR` 报 `LEC=5`（bit0 错误：
发了显性、读回隐性）、`TEC` 涨到 248 后进入 **bus-off**；同时把负载板接到总线后，
总线被拉到 **CANH=1.5 V / CANL=1.7 V**（正常空闲应各 ≈2.5 V），拔掉立刻恢复。

**判定过程**（固件侧逐层排除）：

1. `can_selftest[0..2]` = FDCAN **内部环回**：发送 1、自己收到 1、ID/长度/首字节都对 → 时钟、内核、引脚复用、过滤器全正常。
2. `can_selftest[11]` = FDCAN **外部环回**：同样通过 → MCU 的 TX/RX 通路正常。
3. 用 `LOAD_DISABLE_CAN=1` 编译一版**完全不碰 CAN** 的固件（FDCAN 不初始化、
   PB5/PB8/PB9 保持模拟输入）：接上总线后电压**仍然掉到 1.5/1.7 V** → 与固件无关。
4. 结论：**TJA1042 损坏**（总线引脚把线拉低）。用户换掉该芯片后，总线恢复 2.5 V。

**修复后实测**：`PSR=0x00000708`（无错，bus-off=0）、`ECR=0`、
`tx_acc = tx_gyro = samples = 414`、`tx_fail = 0`、`load_imu_errors = 0`；
上位机 RAM 里 `load` 结构：`valid=1`、`rate_hz=100`、`samples=1900`、
`frames=1944`、`acc_mg = (-322,-301,910)`、`temp=22 °C`、`last_ms` 与当前 tick 同步。

**教训**：
- 判断"某个节点是不是把总线弄坏"：拔掉它，看总线静态电压是否恢复 2.5 V；再把它的
  固件改成完全不初始化 CAN，如果仍然拉低，就是硬件。
- 只清 MOE / 关 FDCAN 并不会让收发器"脱离"总线 —— **收发器坏掉/没电时它照样会钳住总线**。

## 6. 诊断开关

| 宏（`User/load_config.h`） | 默认 | 作用 |
|---|---|---|
| `LOAD_DISABLE_CAN` | 0 | 1 = 完全不初始化 CAN（判"固件 vs 硬件"用） |
| `LOAD_RATE_HZ_DEFAULT` | 100 | 默认上报速率 |

`User/main.c` 里的 `can_selftest[12]`、`imu_ok`、`load_tx_fail` 等都是 `volatile`
全局，方便直接用 SWD/OpenOCD 读。

`load_config.h` 里记录的 STB 处理：本板 PB5 → TJA1042 STB，且设计上 STB 直接接地，
所以固件**不驱动** PB5（最初怀疑 standby，实际不是；芯片坏才是真因）。
