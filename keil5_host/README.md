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
| PB6 / PB7 | GPIO I2C SCL / SDA | 接 ICM-42670-P，开漏软件 I2C，地址 0x68（或 0x69） |
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
GPIO-I2C PB6/PB7 -> ICM-42670-P   FDCAN1 PB8/PB9 @ 500k   LED PA5
[UART2] PA2/PA3 initialised @ 115200 (ESP32 protocol deferred)
[I2C] scanning 0x08..0x77 ...
[I2C] bus levels  pu=C0 pd=C0 clear=C0                                 <- PB6/PB7 空闲为高
[I2C] GPIO scan PB6=SCL PB7=SDA: 1 device(s), first 0x68; swapped: 0, first 0x00
[I2C] 1 address(es): 0x68                                             <- 找到 IMU
[IMU] OK  addr=0x68  WHO_AM_I=0x67  I2C clock=10k GPIO                 <- WHO_AM_I 必须是 0x67
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

### 4.1 有线串口终端的行为（2026-10-30 改进）

`USART1`（PA9/PA10 → CH343P → 电脑）是这块板的主控制口，现在按普通串口终端的方式工作：

| 操作 | 效果 |
| --- | --- |
| 输入可打印字符 | 立即回显（不再是"盲打"） |
| `Enter` | 发送命令行；空行只换行并重新给提示符 `> ` |
| `Backspace` / `DEL` | 退格删一个字符（同时回删屏幕上的字符） |
| `Ctrl-U` | 清空当前整行 |
| `Ctrl-C` | 放弃当前整行（显示 `^C`） |
| 方向键 / 功能键 | `ESC` 序列整段吞掉，不会把 `[A` 之类塞进命令 |
| TAB | 当空格 |
| 超长行（>95 字符） | 截断，不影响后续命令 |

- 每条命令执行完都会重新打印提示符 `> `，方便连续操作。
- 串口监视器请关掉"本地回显"（Arduino 监视器默认不回显，正好；PuTTY/SecureCRT 默认也是远程回显）。
- 接收是中断 + 256 B 环形缓冲，`b` 里的 `UART1 dropped` 一直是 0 就说明没丢字节；
  实测连发 8 条命令（不间隔）全部执行，`dropped=0`。


## 5. 常见故障对照

| 现象 | 先查什么 |
| --- | --- |
| 串口完全没输出 | CH343P 的 3V3/GND、PA9/PA10 是否接反、USB 线是否只充电、波特率是否 115200 |
| 输出乱码 | 波特率不匹配（`HOST_CONSOLE_BAUD`）、PCLK2 不是 170 MHz |
| `SYSCLK` 不是 170000000 | HSE 8 MHz 晶振没起振（`clock_init()` 会退回 HSI，打印值会变） |
| `[I2C] no device found` | 先看 `GPIO scan` 与 `bus levels`：确认 PB6/PB7 开漏、外部上拉（约 4.7–10 kΩ 到 3V3）、IMU 的 VDD/VDDIO 与 GND、AP_CS 拉高进入 I2C 模式、AP_AD0 选择 0x68/0x69。PB6 不支持本芯片的 I2C1_SCL 复用，所以固件使用 10 kHz GPIO 模拟 I2C |
| `[IMU] init failed` | 地址应答后 WHO_AM_I 应为 0x67；检查是否为 ICM-42670-P、AP_CS 是否拉高选择 I2C，以及 VDD/VDDIO |
| IMU 读到 0 / 全 0 | 初始化寄存器没写进去；用 `i` 重扫确认地址，再看 `imu_last_error()` 文本 |
| CAN `init FAILED` | PB8/PB9 与 TJA1042 的 RXD/TXD 是否交叉、TJA1042 的 5V 是否供电（驱动板 v1.1 经验：**收发器没 5V 时 CAN 一帧都发不出去**） |
| CAN 初始化 OK 但 `no reply` | CANH/CANL 是否反、两端 120 Ω 终端（量 CANH–CANL 约 60 Ω）、三块驱动板是否上电、ID 是否对（0x201..0x203） |
| PSR 显示 bus-off | 总线上只有自己（没人 ACK）。固件会在 500 ms 后自动重启，接上驱动板即恢复 |

## 6. 已知风险 / 待验证

1. STM32G431 的 PB6 没有 I2C1_SCL 复用功能，因此 PB6/PB7 必须使用开漏 GPIO 模拟 I2C；PB6 同时是 UCPD1_CC1，不用 USB-C/UCPD 时要关闭 dead-battery 下拉。
2. **PA5 灯极性**先按"低电平点亮"写（沿用驱动板），实测若相反只改 `host_config.h` 里的 `HOST_LED_ACTIVE_LOW`。
3. **TJA1042 的 STB 脚**：这块板的引脚清单没提，代码默认 `HOST_CAN_DRIVE_STB_ON_PB5 = 0`（假设 STB 直接接地/下拉）。若 CAN 无输出且确认收发器有电，把该宏改 1 并确认 STB 实际接的是 PB5。
4. ESP32-S3 只做了 UART2 初始化，协议留空。

## 7. 已排除的上板问题

### CAN 一有中断就整板死机（2026-10-31，已修）

- 现象：上电一切正常（横幅、`b`、`m` 都有输出），但一旦 CAN 总线上有活动（例如三块驱动板接好后回包），
  板子立刻**彻底失联**：串口再无任何输出、`uwTick` 停止增长。
- 定位：用 pyocd 暂停内核读 `PC=0x08000206`、`xPSR=0x01000025`（IPSR=0x25 → 异常号 37 → **IRQ21 = FDCAN1_IT0**）。
  `0x08000206` 正是启动文件里 `ADC1_2_IRQHandler` 那个 `b .` —— 也就是**启动文件的 weak 默认死循环**。
  `can_host_init()` 里只调了 `HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn)`，但本工程没有 `stm32g4xx_it.c`，
  从来没有定义 `FDCAN1_IT0_IRQHandler` → 第一个 CAN 中断就跳进 weak 默认处理。
- 修复：`BSP/can_host.c` 末尾补
  ```c
  void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&can); }
  void FDCAN1_IT1_IRQHandler(void) { can.Instance->IR = 0xFFFFFFFFu; }
  ```
- 教训：**EnableIRQ 之前必须确认同一工程里有同名 handler**（`nm` 里能看到 `T FDCAN1_IT0_IRQHandler`）；
  否则表现为"偶发死机"，且只有外设在特定时刻触发中断时才复现。
- 顺带改进：三个节点的回包原本按"同一 CAN ID 连续"分组，三块板同时回包时会互相打断成碎片；
  现在改为**每节点一个行缓冲**，按 `\n` 或 40 ms 静默成行输出，前缀 `[N1] / [N2] / [N3]`；
  主循环每轮 `can_poll_replies()`，所以 `c<N> <cmd>` 的回包也能异步打印，网页可以实时解析。

### ICM-42670-P I2C 无应答（2026-10-30，已在板验证）

- G431CBTx 的 PB6 复用表没有 `I2C1_SCL`，因此原硬件 I2C 配置不会在 PB6 输出时钟。已改为 PB6/PB7 开漏 GPIO、10 kHz 软件 I2C。
- PB6 同时是 `UCPD1_CC1`。实测原固件的 `PWR->CR3=0x8000`，`UCPD_DBDIS`（bit 14）未置位；UCPD dead-battery 下拉把 PB6 拉低。新固件初始化时调用 `HAL_PWREx_DisableUCPDDeadBattery()`；SWD 回读 `PWR->CR3=0xC000`。
- 修正传感器数据寄存器地址：温度 `0x09`、加速度 X `0x0B`、陀螺 X `0x11`。规格书页 52–54；原文件中的 `0x1D/0x2F/0x25` 会读错数据。
- 新固件在实机扫描到地址 `0x68`，`WHO_AM_I=0x67`；静止读数约 `0.001, -0.056, 1.017 g`、`-0.5, 0.0, 0.7 dps`、`25.8 °C`。外部上拉已由总线高电平读数确认。

### 首次上板的其他两个问题（2026-10-29，已修）

1. **上电进了系统 bootloader**：这块板的 `PB8` 既是 `FDCAN1_RX`，又和 `BOOT0` 引脚复用/受其影响，
   上电瞬间收发器把 PB8 拉高 → 芯片进 STM32 系统 bootloader，表现为"固件烧写成功但程序完全不跑"
   （SWD 读 `USART1->CR1=0x140D`、`BRR=0`，是 bootloader 的 8E1 配置；`GPIOA/PB 的 MODER` 是 bootloader 配的）。
   修法：写一次选项字节，让芯片忽略 BOOT0 引脚：
   ```powershell
   & $cli -c port=SWD mode=UR freq=200 -ob nSWBOOT0=0 nBOOT0=1
   ```
   （和驱动板同一个坑；写完后 SWD 读 `USART1->BRR == 0x5C4` 即说明跑的是本固件。）
2. **缺 `SysTick_Handler`**：驱动板工程里这个函数在 `BSP/board.c`，上位机板不编 `board.c`，
   于是第一次 1 ms tick 就落到启动文件的默认死循环：串口打印到一半停住、指示灯也不闪。
   已把 `void SysTick_Handler(void) { HAL_IncTick(); }` 放进 `BSP/host_board.c`。

自检办法（不依赖串口，用 SWD）：`ICSR(0xE000ED04)` 的 VECTACTIVE 应为 0；
`GPIOA->ODR(0x48000014)` 的 bit5 每 0.5 s 翻转一次；`USART1->BRR(0x4001380C) = 0x5C4`。

## 2026-10-30：IMU 一直读不到的真正原因（已修复）

三块驱动板一直"硬件 I2C 不通"、上位机板的 ICM-42670-P 也一个地址都不应答 —— 这次查清了，是**引脚复用**问题，不是焊接也不是芯片坏：

1. **PB6 在 STM32G431 上根本没有 I2C1 的复用功能**。G431 的 I2C1_SCL 只在 PA13 / PA15 / PB8，I2C1_SDA 只在 PA14 / PB7 / PB9；PB6 的 AF 是 TIM16_CH1N / TIM4_CH1 / TIM8_CH1 / USART1_TX，另有 **UCPD1_CC1** 功能。所以之前所有"把 PB6/PB7 配成 AF4 + 硬件 I2C1"的做法**注定失败**（外设收不到时钟，也驱动不了 SCL）。
2. PB6/PB7 作为 UCPD 引脚上电后带有**死电池下拉**（dead-battery pull-down，`PWR_CR3.UCPD1_DBDIS`），不清掉会把两根线往地拉。修复第一行就是 `HAL_PWREx_DisableUCPDDeadBattery()`。
3. 因此 IMU 改成 **GPIO 开漏 bit-bang I2C**（`BSP/icm42670.c`）：含 SCL 释放等待（防时钟拉伸）、超时、9 时钟总线解锁、START/STOP 完整时序。
4. 顺带修掉一个真 bug：**ICM-42670-P 的 bank0 寄存器地址用错了**（原来抄的是 ICM-42688 的 map）：
   - `TEMP_DATA1 = 0x09`（曾误写 0x1D）
   - `ACCEL_DATA_X1 = 0x0B`（曾误写 0x2F）
   - `GYRO_DATA_X1 = 0x11`（曾误写 0x25）
   `WHO_AM_I(0x75)=0x67`、`PWR_MGMT0(0x1F)`、`GYRO_CONFIG0(0x20)`、`ACCEL_CONFIG0(0x21)` 不变。

修复后实测（COM15，115200）：

```
[I2C] GPIO scan PB6=SCL PB7=SDA: 1 device(s), first 0x68; swapped: 0, first 0x00
[IMU] OK  addr=0x68  WHO_AM_I=0x67  I2C = GPIO bit-bang 250 kHz
IMU acc[g] 0.000 -0.056 1.022   gyro[dps] -0.5 -0.1 0.7   temp 25.7 C
[STRESS] 200/200 OK, total 211668 us, 1058 us/read, 944.9 Hz max
```

- 静态放置时 Z 轴 ≈ 1.02 g、X/Y ≈ 0、陀螺 ≈ 0，读数正确
- 一次完整读取（加速度 6B + 陀螺 6B + 温度 2B）≈ **1.06 ms**，理论上限 ~945 Hz，够做姿态解算
- 速度由 `BSP/icm42670.h` 的 `IMU_I2C_HALF_PERIOD_US` 控制：2 µs = 250 kHz；拉到不稳就加大（5 µs = 100 kHz，实测同样 200/200 通过）
- 新增 `t` 命令做压力测试（200 次读，报成功率/单次耗时/上限频率）

**对三块驱动板的启示**：它们的 AS5600 也接在 PB6/PB7 上，之前"硬件 I2C 不通、改用软件 I2C"其实是**唯一正确解**，不是将就。以后新板若想用硬件 I2C，请把 SCL/SDA 放到 **PB8/PB9**（或 PA13/PA15 + PA14）。


## 2026-10-30: two command inputs (CH343P USB + ESP32)

The host board accepts commands from **two** independent inputs. They are not
used at the same time; whichever side delivers a complete line first is
executed, and **the reply goes back only to the side that asked**. Boot banner
and asynchronous logs still go to both.

| Input | Pins | Driver | Notes |
| --- | --- | --- | --- |
| USB (CH343P) | PA9/PA10, USART1 | BSP/console.c | printf is routed here |
| ESP32-S3 | PA2/PA3, USART2 | BSP/esp_link.c | 115200 8N1, RX ring buffer |

Output routing API (in console.h): console_set_route(CONSOLE_OUT_USB),
CONSOLE_OUT_ESP, CONSOLE_OUT_BOTH; console_route() reads it back.
putc()/printf() follow the route, so a command handler never has to care
where the text goes.

### ESP lines must start with @

Lines arriving on UART2 are only executed when they start with @:

`
@m            read IMU
@b            board status
@c1 T0.5      send T0.5 to CAN node 1
`

Everything else is counted and dropped (ignored= in ). This is not
paranoia - during bring-up the ESP32 side was dumping a continuous ASCII
stream into PA3 at ~115200; without the @ gate those bytes were parsed as
commands (more than 1000 commands in two seconds) and the replies fed back
into the loop. The ESP32 firmware must therefore prefix every command with
@.

### Noise guard

esp_link_guard() runs once per main-loop pass. A real operator cannot type
more than ~1.5 kB/s, so if the measured rate exceeds that for a 500 ms window
the ESP input is **muted for 5 s** and a warning is printed on the USB side:

`
[ESP-LINK] noisy input (>3 kB/s) - ESP input muted for 5 s; check PA2/PA3 wiring
`

 shows the live counters: 
x=, ad=<lines>/<bytes>, ignored=,
dropped=, plus muted= / muted_drops=.

### e self-test

e prints the UART2 configuration and counters, then runs a pin-level test:
it takes PA2/PA3 over as GPIO, reads the idle levels, drives PA2 low and high
while watching PA3. Interpretation:

| Observation | Meaning |
| --- | --- |
| PA3 follows PA2 | TX and RX are wired together, or the ESP32 ties them |
| PA3 idles low | something is holding the line low / ESP32 TX stuck |
| PA3 idles high, does not follow | wiring looks sane |

Measured on the board with the ESP32 side spamming: idle PA2=1 PA3=1,
PA2 low -> PA3=1, PA2 high -> PA3=1, 
x growing ~15 kB/s, commands ESP=0
(the gate works), muted=1.

### Known state

* USB input path: verified (commands: USB= increments, UART1 dropped=0).
* ESP input path: framing + routing + guards verified, but **cannot be
  verified end-to-end until the ESP32 firmware sends @-prefixed lines** and
  the continuous junk stream on PA3 is explained (see e output).

### What the ESP line actually carries (2026-10-30)

e also prints [ESP-SAMPLE], the last printable run seen on PA3. On the
bench it reads:

`
[ESP-SAMPLE] framing_err=5 invalid header: 0xffffffffinvalid header: 0xffffffff...
`

That is the **ESP32 ROM bootloader complaining about an empty / invalid app
image** - the ESP32-S3 on this board has no valid firmware yet, so it reboots
in a loop and prints invalid header: 0xffffffff forever at 115200. Hence the
~12 kB/s of ASCII on PA3 (
x climbing, commands ESP=0 because nothing
starts with @).

So the ESP input cannot be validated end-to-end until the ESP32-S3 is flashed
with a bridge firmware that (a) stays quiet, (b) reads commands from its USB
CDC and writes @<cmd>\r\n to its UART TX at 115200, (c) optionally parses
the replies for its own display/UI.

Measured counters with the boot-looping ESP32 attached:

| Field | Value |
| --- | --- |
| 
x | climbing ≈ 12 kB/s (35430 -> 93049 in a few seconds) |
| ad lines/bytes | tiny (stream is valid 115200 ASCII) |
| raming_err | 1-5 (occasional glitch only) |
| ignored | climbing (no line starts with @) |
| commands ESP= | 0 - the gate prevents junk from running |
| muted | 1 (rate guard engaged) |


### ESP32-S3 side firmware

The ESP32-S3 bridge that feeds this input lives in `../esp32_bridge/`:

* `esp32_bridge.ino` - Arduino sketch: USB CDC (PC) <-> UART (G431, 115200),
  adds the `@` prefix, provides `!help/!stat/!ping/!probe/!raw/!reboot` and
  auto-detects which UART actually reaches this board.
* `esp32_bridge/README.md` - wiring, Arduino IDE settings, troubleshooting.

Expected exchange once it is flashed:

```
[esp32] G431 is alive: "EN=OFF angle=... dir=-1 zero=1.2270 Vbus=24.0 nFAULT=1"
```
