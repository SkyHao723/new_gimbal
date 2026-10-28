# 上位机板固件（G431 CAN 主机 / 点板固件）

这块板的定位是**三块驱动板的上位机**：一路 USB 串口连电脑，一路 CAN 当主机给三个电机驱动板发命令，
再挂一颗 ICM-42670-P 做姿态。当前只到"点板"阶段——把四个外设逐个验证出来。

- MCU：STM32G431CBT6，HSE 8 MHz → PLL → **170 MHz**（与驱动板同一份 `clock.c`）
- 构建产物：`keil5_host/Objects/host_bringup.hex`（`*.hex` 被 `.gitignore` 忽略，属正常）
- 源码：`keil5_host/User/main.c` + `keil5_host/BSP/*`

## 1. 引脚定义（用户给定）

| 引脚 | 功能 | 说明 |
| --- | --- | --- |
| PA9 / PA10 | USART1 TX / RX | 接 CH343P → USB 连电脑，**115200 8N1**，AF7 |
| PB8 / PB9 | FDCAN1 RX / TX | 接 TJA1042（CAN 主机），**500 kbit/s**，AF9 |
| PB6 / PB7 | I2C1 SCL / SDA | 接 ICM-42670-P，硬件 I2C，地址 0x68（或 0x69），AF4 |
| PA5 | 工作指示灯 | 代码默认**低电平点亮**（见 `HOST_LED_ACTIVE_LOW`，焊好实测后按需改） |
| PA2 / PA3 | USART2 TX / RX | 与 ESP32-S3 通讯，**暂缓**，只做了初始化（115200） |

CAN 参数与驱动板固件完全一致（PLLQ 85 MHz ÷2 ÷5 ÷(1+13+3) = 500 k，采样点 82.35%、经典帧、关自动重传），
所以两边能直接互通：

- 命令：`0x201` / `0x202` / `0x203` = 驱动板 1/2/3；`0x200` = 广播
- 回复：`0x281` / `0x282` / `0x283`

## 2. 编译与烧录

```powershell
# 编译（Keil 命令行；exit=0 表示无 error 无 warning）
& 'D:\keil\UV4\UV4.exe' -b 'D:\dsh_build\ng\keil5_host\host_bringup.uvprojx' -o 'D:\dsh_build\host_build.log'

# 烧录（新板建议先全片擦除，SWD 用 200 kHz —— 驱动板遇到过 1 MHz 下的伪故障）
$cli='D:\program\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
& $cli -c port=SWD mode=UR freq=200 -e all
& $cli -c port=SWD mode=UR freq=200 -d 'D:\dsh_build\ng\keil5_host\Objects\host_bringup.hex' -v -rst
```

编译环境：Keil ARMCC V5.06（工程沿用驱动板的设置：`STM32G431xx, USE_HAL_DRIVER`、MicroLIB、
`-O2`、ST-Link 烧录算法 `STM32G43x-4x_128`），HAL 源码与启动文件通过相对路径 `..\keil5_g431\` 复用，
不重复入库。

## 3. 上电期望输出（逐项判据）

想自动核对可以跑 `python tools/host_bringup_check.py --port COMx`。

```
===== G431 host board bring-up =====
SYSCLK=170000000 Hz  HCLK=170000000  PCLK1=170000000  PCLK2=170000000   <- 时钟 170 MHz
UART1 PA9/PA10 <-> CH343P @ 115200 8N1                                 <- 这行能出来就说明串口通了
I2C1 PB6/PB7 -> ICM-42670-P   FDCAN1 PB8/PB9 @ 500k   LED PA5
[UART2] PA2/PA3 initialised @ 115200 (ESP32 protocol deferred)
[I2C] scanning 0x08..0x77 ...
[I2C] 1 address(es): 0x68                                             <- 找到 IMU
[IMU] OK  addr=0x68  WHO_AM_I=0x67  I2C clock=100k                    <- WHO_AM_I 必须是 0x67
IMU acc[g] 0.00 0.00 1.00   gyro[dps] 0.0 0.0 0.0   temp 25.0 C        <- 平放时某一轴约 ±1 g
[CAN] init FDCAN1 500k ... OK  PSR=0x...                              <- CAN 初始化成功
[CAN] broadcast '?', collecting replies for 2 s (PSR=... ECR=0)
  [0x281] EN=OFF angle=...                                       <- 三块驱动板各自的回复
  [0x282] ...
  [0x283] ...
```

主循环：PA5 每 0.5 s 翻转一次（灯在闪 = 程序在跑）。

## 4. 串口命令

| 命令 | 作用 |
| --- | --- |
| `h` / `?` | 帮助 |
| `i` | 重扫 I2C 总线（换 IMU/改上拉后不用重新烧） |
| `m` | 读一次 IMU（加速度/角速度/温度） |
| `n` | 三块驱动板巡检（广播 `?`，收 2 s 回包，按 CAN ID 分组打印） |
| `c<N> <cmd>` | 给第 N 块驱动板发命令：`c1 T0.5`、`c2 M1`、`c0 V2.5`（`c0` = 广播） |
| `b` | 板级状态（时钟、CAN PSR/ECR/rx/busoff/txdrop、串口丢字节数、IMU 地址） |
| `l` | 手动翻一次灯 |

命令编码与驱动板 `can_commands.c` 完全一致：`E/D/?/R/H` 单字节；`T/V/L/M` = 操作码 + float32 小端。

## 5. 常见故障对照

| 现象 | 先查什么 |
| --- | --- |
| 串口完全没输出 | CH343P 的 3V3/GND、PA9/PA10 是否接反、USB 线是否只充电、波特率是否 115200 |
| 输出乱码 | 波特率不匹配（`HOST_CONSOLE_BAUD`）、PCLK2 不是 170 MHz |
| `SYSCLK` 不是 170000000 | HSE 8 MHz 晶振没起振（`clock_init()` 会退回 HSI，打印值会变） |
| `[I2C] no device found` | **本板是项目里第一次用硬件 I2C**（驱动板一直用软件 I2C 顶着）：先查 PB6/PB7 上拉（4.7 k 到 3V3）、IMU 供电、地址 0x68/0x69；代码已内置 100k→50k→10k 自动降速，能区分"速度问题"和"接线问题" |
| `[IMU] init failed: who_am_i` | 读到器件但不是 0x67：型号不是 ICM-42670-P，或电源/SPI 模式选择脚不对 |
| IMU 读到 0 / 全 0 | 初始化寄存器没写进去；用 `i` 重扫确认地址，再看 `imu_last_error()` 文本 |
| CAN `init FAILED` | PB8/PB9 与 TJA1042 的 RXD/TXD 是否交叉、TJA1042 的 5V 是否供电（驱动板 v1.1 经验：**收发器没 5V 时 CAN 一帧都发不出去**） |
| CAN 初始化 OK 但 `no reply` | CANH/CANL 是否反、两端 120 Ω 终端（量 CANH–CANL 约 60 Ω）、三块驱动板是否上电、ID 是否对（0x201..0x203） |
| PSR 显示 bus-off | 总线上只有自己（没人 ACK）。固件会在 500 ms 后自动重启，接上驱动板即恢复 |

## 6. 已知风险 / 待验证

1. **硬件 I2C 是第一次上板**：驱动板那边硬件 I2C 从未跑通（一直靠软件 I2C），所以 IMU 扫不到时要优先怀疑 I2C 配置与时序，而不是 IMU 本身。
2. **PA5 灯极性**先按"低电平点亮"写（沿用驱动板），实测若相反只改 `host_config.h` 里的 `HOST_LED_ACTIVE_LOW`。
3. **TJA1042 的 STB 脚**：这块板的引脚清单没提，代码默认 `HOST_CAN_DRIVE_STB_ON_PB5 = 0`（假设 STB 直接接地/下拉）。若 CAN 无输出且确认收发器有电，把该宏改 1 并确认 STB 实际接的是 PB5。
4. ESP32-S3 只做了 UART2 初始化，协议留空。
