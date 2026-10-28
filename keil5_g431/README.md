# STM32G431CBT6 / DRV8316C / AS5600：Keil5 工程

> **2026-10-26 新 PCB 更新**：DRV 的 DRVOFF 已由用户直接接地，不再连接 PB1；MCU
> 当前由 ST-Link 3.3V 独立供电。已用独立 g431_bringup.uvprojx 完成 1.5V / 1.2s
> 开环转动，用户确认声音正常。接入 AS5600 后，慢速软件 I2C 与手动转轴反馈已验证，
> 现已定位：G431 的 PB6 没有 I2C1_SCL 复用，原 PB6/PB7 硬件 I2C 配置有误。
> 当前诊断固件上电默认软件 I2C；接入编码器后已完成 1.5V/1.2s 开环试转，
> 编码器记录约 -40.43°，通信零新增错误。按用户要求暂缓弱磁排查；未执行闭环，现已失能。
> 当前板上是 Bringup/main.c 诊断固件，默认不转。下文旧 CAN 固件的 PB1/DRVOFF 假设
> 不适用于这块新板，不能直接覆盖。详见
> [新板编码器验证](../docs/measurements/2026-10-26-g431-encoder/README.md)。

依据用户在 2026-10-16 提供的四张原理图建立。控制算法直接引用用户确认过的
`../keil5_foc/Control`，G431 外设使用本地 ST HAL / CMSIS，纯 C99，无 Arduino 依赖。
G4 没有 F1 的 SPL 库，不能直接替换 F103 的芯片宏后编译。

## 工程与构建

Keil5 打开 `gimbal_g431.uvprojx`，目标 `G431CB_FOC`：

- ARM Compiler 5.06 update 1，Cortex-M4 单精度 FPU，MicroLIB，C99。
- 设备包 `Keil.STM32G4xx_DFP.2.2.0`，本机已安装至 `D:\keil\ARM\PACK`。
- Flash `0x08000000 / 128KB`，RAM `0x20000000 / 32KB`，沿用设备包内存映射。
- 命令行 `./keil5_g431/build.ps1`；ASCII 联接 `D:\dsh_build\keil5_g431`。
- 产物 `Objects/gimbal_g431.hex`、`Objects/gimbal_g431.axf`。

复制工程时同时保留相邻的 `keil5_foc/Control`。G431 配置文件在本工程 `User`，
不读取 F103 的 `User/motor_config.h`。F103 的控制源码和参数没有因本次移植改变。

## 已落实的接线

| 信号 | G431 引脚 / 配置 | 与 F103 的区别 |
|---|---|---|
| PWM A/B/C | PA8 / PA9 / PA10，TIM1，AF6，中心对齐 25kHz；CH4 内部触发 ADC | G4 AF6，重新配置定时器 |
| nSLEEP | PB0，高=唤醒，**低=真正断电** | 本板 DRVOFF 已接地，nSLEEP 是唯一硬件关断手段 |
| DRVOFF | 原理图直连 PGND，PB1 空接 | 软件关断改为清 MOE **并拉低 nSLEEP**；PB1 不再有效 |
| nFAULT | PB2，低有效，EXTI2 | 下降沿立即关断，故障仍低时拒绝 E |
| AS5600 SCL / SDA | PB6 / PB7，开漏 GPIO 软件 I2C | G431 PB6 没有 I2C1_SCL；按实际插座接线位操作 |
| 编码器插座 CN1 | 1=SDA，2=SCL，3=GND，4=3.3V | 注意插头的实际针序 |
| CAN RX / TX | PB8 / PB9，FDCAN1，AF9 | 替代旧串口；两个 CAN 插座是同一总线 |
| CAN STB | PB5，低=正常，高=待机 | TJA1042T/3/1J，初始化后才退出待机 |
| 心跳 LED | PA5，低电平点亮 | 不使用蓝板 PC13 |
| SOA/SOB/SOC | PA0 / PA1 / PA2 → ADC1 注入组 IN1/IN2/IN3，TIM1_CC4 触发采样 | 新增相电流采样；PA2 不可再作串口 TX |
| SWD | PA13 / PA14 | 保留 |

原理图证据：[MCU](../docs/hardware/g431-2026-10-16/mcu.png)、
[CAN/电源](../docs/hardware/g431-2026-10-16/can-power.png)、
[插座](../docs/hardware/g431-2026-10-16/connectors.png)、
[驱动](../docs/hardware/g431-2026-10-16/driver.png)。

CPU 用图中的 8MHz HSE，经 `/2 ×85 /2` 得到 170MHz，Range 1 Boost、Flash 4 等待周期。
PLLQ=85MHz，FDCAN 再除以 2 后为 42.5MHz；100kHz 以下的 I2C 用 HSI16 供时钟。
TIM2 是原生 32 位 1MHz 微秒计时，不再用 F103 的 16 位计数溢出扩展。

## PB8 / BOOT0 的实际启动问题

板上 PB8 同时连接 BOOT0 和 CAN RXD。实机默认 `nSWBOOT0=1`，BOOT0 来自引脚；
CAN 收发器空闲时 RXD 为高，软件复位后实际进入系统 ROM，没有运行刚下载的 Flash 程序。

本轮已经将该板 `nSWBOOT0` 改为 **0**，原有 `nBOOT0=1`、`nBOOT1=1` 保留，
从 Flash 启动并让 PB8 作为普通 CAN 接收脚。CubeProgrammer 回读校验通过，随后已读到应用
诊断标志及 170MHz 运行状态。没有修改读保护、写保护、BOOT_LOCK 或 SWD。

其他同版板子首次下载，也需要在 Option Bytes 设置上述启动位。仅下载 HEX 不包含这项设置。
原配置备份：`../docs/measurements/2026-10-16-g431/option-bytes-before.log`。

## 控制参数与初次启动

速度 PI `0.6 / 5`，位置 P `25`（2026-10-28 由 40 降低，消除 ~57 Hz 极限环，见下文「抖动根因之二」），
速度低通 `0.02s`；规划最高速度 `6rad/s`，
加速度 `8rad/s²`，减速度 `4rad/s²`；母线配置 `24V`，默认电压请求 `4V`，
软件硬上限 `8V`。这些是用户确认过的 F103 参数；新板上的运动表现仍需实机验证。
`V` 命令只是**请求**：实际使用的电压还会被相电流限幅器压回去，`?` 会同时打印
请求值和当前允许值（见下一节）。上限从 5V 提到 8V 的前提是相电流采样可用。

G431 当前设置 `STARTUP_ROTATE_ONCE=0`，上电保持失能，收到 E 后才闭环保持当前位置。
原上电单圈流程仍在共享控制源码中，待新板方向和校准验证后可设为 1。
极对数 11、方向 +1、固定电角度零点 0.92rad 是原电机参数；换相线顺序或磁铁安装后须重新
确认零点。仅将文件移到 G431 不等于新板校准已经验证。本次没有使能电机或发送旋转目标。

运行失能改为**清 TIM1.MOE + 拉低 nSLEEP(PB0)**，故障不会被固件悄悄复位；DRVOFF
已硬接 PGND，只清 MOE 会把三相低侧全部导通（刹车），不是断电，所以必须同时拉低
nSLEEP。nFAULT 仅连接 PB2，不宣称已实现 TIM1_BKIN 硬件封波。电机仍是电压模式
控制，uq 是电压指令而不是实测电流；但 **2026-10-27 起加入了三路低侧相电流采样 +
运行时电压折叠 + I²t 预算**，见下一节。

## 相电流采样与限幅（2026-10-27）

目标：在保证绕组不过热的前提下把可用电压尽量提高。原来只有 `MOTOR_MAX_VOLTAGE`
一个静态电压上限（5V，靠猜），现在改成"实测相电流 → 电压预算"。

硬件链路（本地原理图 `../docs/hardware/g431-current-2026-10-20/page2-driver.png`）：

- DRV8316C 的 SOA/SOB/SOC（pin40/39/38）直连 PA0/PA1/PA2，**无 RC 滤波、无串阻**。
- GAIN(pin31) 与 OCP/SR(pin18) 接 PGND → 电流检测增益 0.15V/A，片上 OCP 为固定档位。
- VREF/ILIM(pin37) 接 AVDD 3.3V → 满量程约 ±11A，零点约 VREF/2 ≈ 2048 counts。

### 为什么"相电流 0.7A"时电源只显示 0.09A（2026-10-28 实测）

三相桥本质是降压（buck）变换器：相电流只在对应的占空比窗口里从母线取电，因此

    母线电流 ≈ 相电流 × (相电压 / 母线电压) + 板子静态电流

实测条件：板 1 把 A-B 两相锁在固定电压（不转、稳定直流），母线 24.48V，
三块板静态合计约 0.010A：

| 相间锁相电压 | 理论相电流 | 电源表实测 | 用上式反推的相电流 | 固件 `I pk` |
|---|---|---|---|---|
| 1.2 V | 0.32 A | 0.026 A | —（太小，被静态电流淹没） | ≈0.26 A |
| 3.0 V | 0.79 A | 0.090~0.092 A | 0.66 A | 0.68 / 0.54 A |

结论：

- `I pk` 是**单相电流峰值**，不是母线电流。正常巡航时相电压只有 1~3V，
  所以相电流可以是电源电流的 **5~10 倍**。web 上三块板 `I pk` 相加 ≈4A、而电源
  不到 1A，是正常的，不是读数错误。
- 增益与真实电流吻合到 ~10% 以内（反推 0.66A vs 固件 0.61~0.68A）。
- 三个通道的零点有 ±25 counts（±0.13A）级别的不一致，所以小电流（<0.3A）时
  A/B/C 不对称、基尔霍夫和不闭合；0.5A 以上误差就 <10%。

固件实现：

- `BSP/current_sense.c`：ADC1 注入组 3 通道（IN1/IN2/IN3），内核时钟 PLLP 170MHz/4
  = 42.5MHz，采样 12.5 周期 → 每通道 25 周期、三通道 1.76µs。
- 触发用 TIM1 CH4：`board_pwm()` 每次把 `CCR4` 设为 `max(CCR1..3)+160`，OC4 配成
  PWM mode 2，上升沿落在"三相全低侧"窗口起点后约 0.94µs。请求 8V 时低侧窗口仍有
  1540 counts（约 9µs），余量充足；窗口不足或 `CCR4` 超出 ARR 时不触发。
- 上电静止标定零点（驱动唤醒、MOE 关闭、无电流）：每个通道必须落在满量程 10%~90%，
  否则判定采样不可信。标定值可通过 `?` 的 `I` 行和 SWD 快照看到。
- `User/current_limit.c`（纯逻辑，可主机单测）：`allowed = 8V - (peak-1.6A)*10V/A`，
  再乘 `(1-I²t)`，clamp 到 `[1.5V, 8V]`；峰值到 2.6A 请求停机。
- `BSP/current_sense.c` 的中断里还有兜底：>2.6A 连续两次或 >4.5A 一次，直接
  `board_request_stop(BOARD_STOP_CURRENT)`，不依赖主循环。

安全退化：采样不可信（未标定 / 无新样本 / 低侧窗口 <300 counts / 触发被跳过）时，
限幅器进入 `blind`，电压请求被压到 **1.5V**，`?` 的 `I` 行显示 `ok=0`。
也就是"电流看不见就别想加大电压"，不会退化成无保护开环。

台架首次验证顺序（尚未执行）：

1. 接 24V + ST-Link，**不发 E**，先只跑 `python tools/g431_probe.py`：确认 `cs_valid=1`、
   三个 offset 都接近 2048、`cs_samples` 以约 25k/s 增长、`cs_peak`≈0。
2. offset ≈ 0 而不是 2048，说明 SOx 是单向输出，需要把零点模型改成 0 并重算增益；
   offset 完全不对则先查 VM、板上 5V 与 SOx 连线。
3. 确认采样可信后再发 `E` 和小 `T`，看 `?` 的 `I pk= allowed= req=` 是否跟随；
   先用短时间手动堵转验证折叠与停机生效，再谈提高电压上限。

## G431 闭环打通（2026-10-27）

实测结论，详见 [g431-闭环打通-2026-10-27.md](../docs/g431-闭环打通-2026-10-27.md)：

- 开环正弦换相连续转动：0.8 V 幅值、1 Hz 电频率、4 s → 位移 -2.009 rad，峰值 0.355 A。
- **极对数 11**（电机参数表里的 7 是错的）、**编码器方向 -1**、**电气零点 1.697 rad**，已写入 `User/motor_config.h`。
- 位置闭环：0.3 rad / 1.0 rad 阶跃稳态误差 ≤0.15°（0.0026 rad），3 rad 阶跃 ≤0.23°（0.0040 rad）。
- 电压上限扫描 2.5 / 4 / 6 / 8 V（3 rad 阶跃）：峰值相电流 0.77 / 0.37 / 0.35 / 0.30 A，全部 `code=0`、无跳闸。
  **上限越高电流越小**（位置环只取所需电压，低上限反而更容易长时间饱和）→ 防烧靠电流限幅，不是压电压上限。
- 三个 ADC 根因已修：① ADC12 时钟必须用 SYSCLK（PLLP 下 `ADCAL` 永不完成）；② `CFGR.JQDIS=1`
  是软件触发注入组的前提；③ **TIM1_CC4 直接触发在本板不产生转换，改 TIM1_TRGO（`CR2.MMS=7`）才工作**
  （诊断字段 `cs_trig` 稳定为 2）。
- nFAULT 上电会锁存为低 → 驱动永久睡眠、SOx 偏置从 1.72 V 掉到 0.6 V。现在 `board_driver_set(1)`
  先脉冲 nSLEEP 复位锁存，主循环空闲时每 500 ms 自恢复并重标零。
- **CAN 已打通**（接上外部 5V 后）：candleLight/gs_usb 500 kbit/s，`E/D/T/V/L/?` 全部可用。工具：
  `tools/g431_can.py`（单命令）、`tools/g431_can_move.py`（连续摆动 + 状态轮询 + CSV）、
  `tools/g431_can_step.py`（阶跃响应）。之前一帧都收不到就是 TJA1042 的 VCC 走板上 5V 而没有 5V。
- 连续运动 72 s（8 V 上限、±2.5 rad）：峰值 0.43 A、I²t 0、无跳闸。1 rad 阶跃在 2.5 V 与 8 V 下响应一致
  （都是 0.77 s 内进入 ±0.01 rad，超调 ≤0.27°）→ 常规运动 2.5 V 足够，8 V 只对高速/大加速度有意义。
- 限流链路已在硬件上验证：临时压低阈值后 `allowed` 按公式从 8 V 折到 7.30 V，峰值 0.31 A 越过 trip 时
  电机立即失能、电流归零；恢复 soft 1.6 / trip 2.6 后正常。
- **转速上限（2026-10-27 实测）**：`L` 命令本身允许到 20 rad/s，但轨迹巡航速度实际被
  `MOTOR_TARGET_SLEW` 限制（原来只有 6 rad/s，所以在上位机把速度设到 20 也没用）。
  现在 `MOTOR_TARGET_SLEW = 20.0f`，实测稳态转速：L=3/6/9/12/15/18 → 实际 3.00/6.00/9.00/11.95/14.92/17.85 rad/s；
  L=20 时实际约 18.7 rad/s，且峰值电流 1.96 A 触发 1.6 A 折叠（allowed 降到 4.43 V），
  即 **8 V 母线 + 1.6 A 折叠下约 18.7 rad/s（178 rpm）是物理上限**。
  抖动（`g431_jitter_probe.py cruise`，Vlim 8）：9 rad/s 11.4 mrad、12 rad/s 22.1 mrad、
  18 rad/s 158 mrad → **12 rad/s（≈115 rpm）以内仍然很平顺**，再往上纹波明显变大。
- 台架自测宏：`G431_SPIN_TEST` / `G431_ALIGN_TEST` / `G431_CLOSEDLOOP_TEST` / `G431_BENCH_CURRENT_TEST` /
  `G431_CURRENT_LOCK_TEST`，任一为 1 时自动 `G431_BENCH_MODE=1` 并跳过 CAN（否则 bus-off 会持续请求停机）。
  **交付固件必须全部置 0。**

## 抖动根因：AS5600 掉进 100 ms 轮询的低功耗模式（2026-10-27）

详见 [g431-抖动根因-AS5600低功耗.md](../docs/g431-抖动根因-AS5600低功耗.md)。

现象是"转动时有明显抖动、速度上不去、同一套增益有时稳有时不稳"。**不是调参问题，是编码器反馈变成了 10 Hz**：

- 固件侧每秒读 AS5600 401 次，但**原始角度每秒只变化 10.1 次**（`as5600_stats[]` 实测），相邻两次更新
  最小间隔 97 ms —— 传感器自己在 100 ms 轮询的低功耗模式（CONF `0x07` 的 PMOD=11=LPM3）。
- 于是 `delta/dt` 的速度估计变成"39 拍为 0、一拍跳 0.13 rad"，低通后是 5 rad/s 级尖刺；
  速度 PI 把 uq 顶到 2.5 V 饱和 → 10 Hz 极限环 → 用户感到的抖动，同时匀速只能到 ~1.3 rad/s。
- **只有给传感器断电重启才能退出该状态**：MCU 复位/重烧都不算数（传感器一直被 3.3 V 供电）。
  断电重启后 CONF 读回 `0x00`，`updates/s` 393~409，抖动从 74.5 mrad 降到 8.8 mrad（8.5 倍）。
- 固件现在做了两件事：
  1. `as5600_init()` 写 CONF 把 PMOD 置 00，并在运行中每 1024 次读取重写一遍（实测空闲 150 s 仍保持 394 updates/s）；
  2. **编码器停滞看门狗**：使能状态下若 `|ramp_target-angle| > 0.05 rad` 且原始角度 120 ms 未变化，
     立即停机并打印 `FAULT encoder stale (AS5600 low-power mode? power-cycle the sensor)`。
- SWD 诊断量（`nm` 取地址后 `-r32`）：`as5600_conf_log`（[15:8] 修前 CONF、[7:0] 修后）、
  `as5600_stats[6]`（更新次数 / 读取次数 / 最后原始值 / 最小间隔 µs / 最大间隔 µs / CONF 回读）。

## 抖动根因之二：位置环增益过高引起 ~57 Hz 极限环（2026-10-28）

详见 [g431-抖动-位置环极限环.md](../docs/g431-抖动-位置环极限环.md)。

现象：**转到位后静止保持时剧烈高频抖动**（不是转动中抖），约 100 Hz 声音级别，某一台电机更明显。
以前一直被误认为"某块板/某台电机的机械问题"，实测是三块板都会，只是 2 号电机最容易激发。

- 复现序列：`V2.5 → E → T(目标-2rad)` → 等 2.5s → `T(目标)`。板载黑匣子（`X` 命令，
  1200 拍 @398Hz = 3s，记录 angle / velocity / uq）抓到：角度在 ±55 mrad 之间每 4~6 拍一个来回，
  `uq` 有 30% 采样顶在 ±2.5V 饱和 → 位置/速度双环在采样率附近的极限环。
- 定位：把 `MOTOR_ANGLE_P` 从 **40 降到 25** 后，同一序列在 2.5V/8V 下 `angle_std` 从
  35~48 mrad 降到 **1~2 mrad**，1 rad/2 rad 阶跃稳态误差仍为 0.006~0.017°，匀速抖动 7.8/11.2 mrad
  与原来持平 → 只牺牲了很少的刚度就消除了振荡。
- 结论：**398 Hz 控制频率下 `MOTOR_ANGLE_P=40` 裕度不足**（尤其电机静摩擦/齿槽差异大时）。
  不要再往上加；4 号之后的板子沿用 25，需要更硬的位置环应先把控制频率提上去（软件 I2C 半周期 25µs
  是循环周期主项，`ENCODER_I2C_HALF_US` 可降到 5µs 把循环提到 ~1.9kHz 再试）。
- 排查工具（都留在仓库里）：
  - 固件 `X` 命令 = **黑匣子**：`{"node":N,"cmd":"X"}` 武装，3 秒后自动写满，
    SWD 读 `bench_trace` / `bench_trace_meta` 即可离线分析（`tools/g431_can_bridge.py` 与网页
    「抓抖动波形 (X)」按钮都已放行该命令）。
  - `D:\dsh_build` 下的分析脚本未入库：`osc_probe.py`（三块板对比保持抖动）、
    `trace_osc.py`（黑匣子统计：分块 pp / 频率 / uq 饱和率）、`hold_accuracy.py`（阶跃误差）、
    `soak_node.py`（往复热加载）。

## 两种运动模式：缓动 M0 / 直驱 M1（2026-10-28）

`T` 命令只是"目标在哪里"，**怎么走过去**由运动模式决定（`M0`/`M1`，掉电默认 M0）：

- **缓动 M0**（默认）：`trajectory_plan()` 生成 S 曲线，速度受 `MOTOR_TARGET_SLEW`
  （20 rad/s）、加速度受 `MOTOR_TARGET_ACCEL`（8）、减速度 `MOTOR_TARGET_DECEL`（4）限制，
  起停柔和、没有冲击。适合转大角度、演示、怕听到"砰"声的场合。
- **直驱 M1**：不做轨迹规划，`ramp_target` 直接等于 `target`，位置环
  `velocity_sp = clamp(MOTOR_ANGLE_P*(target-angle), ±velocity_limit)` 一步到位，
  到位速度快、干脆，但有冲击，大角度会瞬时拉到大电流。

实测（node3，3 号电机，Vlim 4 V，settle 判据 |err|<0.01 rad）：

| 位移 | 缓动 M0 | 直驱 M1 | M0 峰值电流 | M1 峰值电流 |
|---|---|---|---|---|
| 1 rad | 0.83~0.85 s | **0.20~0.61 s** | 0.1 A 级 | 0.5~1.0 A 级 |
| 3 rad | 1.45~1.54 s | **0.57~0.68 s** | 0.17 A | **1.77~1.92 A** |

注意 3 rad 直驱时峰值电流 1.9 A **已经超过 1.6 A 折叠阈值**，限流器会把 `allowed` 压下去
（这正是它该做的），所以直驱不是"更大扭矩"，只是"不给轨迹留缓冲"。云台瞄准这类
"小角度、要快"的场景用 M1，大幅度移动仍建议 M0。

实现位置：`keil5_foc/Control/motor.c` 的 `motor_step()`（`direct_mode` 分支）与
`motor_command()` 的 `M` 分支；`Motor.direct_mode` 字段默认 0，F103 工程不设置它，
行为与以前完全一致（`keil5_foc/Tests/test_control.c` 新增 `test_smooth_direct_mode()`
覆盖：M 命令解析、ramp 旁路、轨迹停靠、切回 M0 恢复规划、非法参数不改模式）。

网页上在「速度上限」下面有两个按钮「缓动 M0 / 直驱 M1」，状态栏会显示 `mode=SMOOTH|DIRECT`。

实测（node2，Vlim 4 V，`D:\dsh_build\mode_compare.py`，`R` 高速轮询）：

| 步长 | 模式 | 到位时间（|err|<0.01 rad） | 峰值 |vel| | 峰值相电流 |
| --- | --- | --- | --- | --- |
| 1.0 rad | M0 | 0.82 ~ 0.84 s | 2.3 ~ 2.5 rad/s | 0.12 ~ 0.13 A |
| 1.0 rad | M1 | 0.41 ~ 0.49 s | 3.9 ~ 14.3 rad/s | 1.1 ~ 1.7 A |
| 3.0 rad | M0 | 1.45 ~ 1.47 s | 3.8 ~ 4.5 rad/s | 0.13 ~ 0.16 A |
| 3.0 rad | M1 | 0.59 ~ 0.67 s | 18.9 ~ 19.2 rad/s | 1.6 ~ 1.7 A |

## `R` 命令坐标系修正（2026-10-28）

`R` 原来打印 `motor->sensor.angle`（传感器原始累积角），而 `?` 打印的是
`motor->angle = MOTOR_SENSOR_DIRECTION * sensor.angle`。**所以 `dir=-1` 的板（node1/node2）
两个命令的角度符号相反**，主机工具拿 `R` 的读数去和 `T` 的目标值比较时会差一个整段角度
（表现为"误差越跑越大"、"怎么都不收敛"）。现已改为 `R` 也打印方向化后的
`motor->angle`/`motor->velocity`，并在行尾加 `dir=N`，与 `?` 完全一致；`mech=` 仍是传感器
原始单圈角。F103 的 `dir=+1`，行为不变。

## CAN 协议

使用经典 CAN，标准 11 位 ID、500kbit/s，无 BRS。位时序：42.5MHz /5，
Sync=1、Seg1=13、Seg2=3，共 17TQ，SJW=3，采样点约 82.35%。这是自定义协议。

**节点 ID 分配**（`BSP/board_config.h` 的 `BOARD_CAN_NODE`）：

| 节点号 `BOARD_CAN_NODE` | 命令 ID | 回复 ID |
|---|---|---|
| 1（默认） | `0x201` | `0x281` |
| 2 | `0x202` | `0x282` |
| 3 | `0x203` | `0x283` |
| … 7 | `0x207` | `0x287` |
| 0（广播） | `0x200` | 无（所有节点都收） |

同一条总线上**每块板必须是不同节点号**，否则两块板会同时响应同一条命令、
并且在回复 ID 上互相仲裁冲突。广播 `0x200` 三块板都会收并且**都执行**（`D` 全体急停、`E/T/V/L` 会同时作用到所有板，慎用；广播没有回包）。

固件用两个精确匹配滤波器：本节点命令 ID + 广播 ID。改变节点号只需改宏后重编，
`build\g431_build_nodes.ps1` 可以一次生成 node1/2/3 三个 hex：

```powershell
powershell -ExecutionPolicy Bypass -File build\g431_build_nodes.ps1
# -> keil5_g431\Objects\gimbal_g431_node1.hex / node2 / node3
```

上位机工具都带 `--node`（默认 1）：`g431_can.py --node 2 ping`、
`g431_can_bridge.py --node 3`（网页只驱动那一块板）；`g431_can.py --node 0 cmd D` 是广播急停。

每条控制指令必须完整放在**一个 CAN 数据帧**中，数值使用 IEEE754 float32 小端。
这样不会把丢失一个分段后的角度字符串误当成另一个有效目标。

| 命令 | CAN payload | 含义 |
|---|---|---|
| E | `45`，DLC=1 | 当前位置使能；重复 E 不重置目标 |
| D | `44`，DLC=1 | 立即失能，不等待主循环处理队列 |
| Tπ | `54 DB 0F 49 40`，DLC=5 | 绝对机械角 π rad |
| V3 | `56 00 00 40 40`，DLC=5 | 电压上限 3V |
| L20 | `4C 00 00 A0 41`，DLC=5 | 速度上限 20rad/s（轨迹巡航也放到 20，见上文转速上限） |
| M1 | `4D 00 00 80 3F`，DLC=5 | 运动模式：0=缓动（轨迹规划，默认）1=直驱（不做规划，位置环直接拉到位） |
| ? | `3F`，DLC=1 | 查询状态 |
| R / H | `52` / `48`，各 DLC=1 | 编码器 / 帮助（`R` 已改成与 `?` 同一坐标系，行尾带 `dir=N`；`mech` 仍是传感器原始单圈角） |

不要把 `T3.1416` 或带换行的 `E\n` 直接作为 payload 发送。控制操作码沿用旧固件，
传输格式已经适配 CAN。Python 工具负责转换，无需手算浮点数。

回复是 ASCII 字节流，每帧 1～8 字节，按接收顺序拼接，以 CR/LF 分行。状态保留
`EN= angle= vel= tgt= ref= rv= uq= ui=`，并补充 CAN 丢弃字节数和 nFAULT 状态。
发送缓冲不阻塞 FOC；断线、拥塞时可能丢弃诊断文字，应重新查询完整状态。
本地发送不自动重试，bus-off 会关断驱动并停止 CAN，重新复位恢复。
接收溢出、队列溢出、错误长度、NaN/Inf 也会请求失能；D 始终优先。

## 三块板一条总线：接线与上电检查（2026-10-27）

三块完全一样的板子挂在同一条 CAN 上，除了上面的节点 ID，还要注意这些硬件项：

1. **终端电阻**：CANH–CANL 在**总线两端**各 120 Ω，中间节点不要接。断电后用万用表量
   任意两板的 CANH–CANL：两端都接好 ≈60 Ω；只有一端 ≈120 Ω；无穷大 = 一个都没接
   （短期能通，但 500kbit/s 下长线会随机出错）。原理图上 `R9` 是 0Ω 占位，**确认是否已装 120 Ω**。
2. **每块板的 TJA1042 都要 5 V**：本板 v1.1 的 5V 来自外部（DRV 内部 buck 已坏），
   `VCC` 没 5 V 时收发器完全没电、一帧都不会发出去（我们排查过整整一轮）。三块板各自要接 5 V。
3. **共地与共 24 V**：三块板共地是必须的；24 V 母线按 3 台电机的峰值电流留余量。
4. **每块板的 3.3 V 来源**：目前调试用的板子靠 ST-Link 的 3.3 V，正式装配要有自己的 1117 输出。
5. **线材**：CANH/CANL 双绞、支线（stub）尽量短（<0.3 m）、避开电机相线；
   总长 < 10 m 在 500kbit/s 下没问题。

上电顺序建议：先给三块板 5V/24V → 等 1 s（nFAULT 上电锁存自恢复路径会跑）→
用 `g431_can.py --node N ping` 逐块确认 `nFAULT=1 cs_valid=1 ok=1`，
三块都对了再发使能。

## 每块板都要单独标定 `zero` / `dir`（2026-10-28）

**同一个型号的电机，装在不同板上，电气零点与传感器方向都可能不同**（磁铁环手工粘、
三相线可任意插）。三块板实测结果：

| 节点 | 极对数 | 方向 | 零点 (rad) | 拟合残差 |
|---|---|---|---|---|
| node1 | 11 | −1 | 1.227 | 4.7° rms |
| node2 | 11 | −1 | 0.122 | 5.1° rms |
| node3 | 11 | **+1** | 0.487 | 3.7° rms |

板 2 曾沿用板 3 的 `dir=+1`，结果电压矢量压在转子磁极轴线上：`uq` 顶到限幅、
`pk` 1.07 A，转子一步不动（电流全变热）。**方向错的表现是"不动 + 发烫"，不是发抖。**

标定走"双向拉轴 + 叠加平均"：`bench_align_run()` 每行先从 `theta−0.9 rad` 平滑扫到
`theta` 取一次稳定角，再从 `theta+0.9 rad` 扫回来取一次，磁滞被平均掉；残差从
几十度电角降到 ~5°。拟合用 `tools/g431_fit_align.py`（输入 SWD dump 的
`bench_align_result[]`）。完整步骤、验收数据与注意事项见
`../docs/g431-三块板标定-2026-10-28.md`。

## 电脑调试

旧网页使用 Web Serial，不能直接连接这块仅引出 CAN 的板子。使用支持 python-can 的
USB-CAN 适配器及 `../tools/g431_can_controller.py`。ST-Link 本身不是 USB-CAN。

```powershell
python -m pip install "python-can>=4.5,<5" gs_usb libusb-package
# 本机实际识别为 candleLight，使用原生 USB，通道索引为 0
python tools/g431_can_controller.py --interface gs_usb --channel 0 --command "?"
# 其他支持 SLCAN 的适配器才使用 --interface slcan --channel COM8
```

工具默认为只查询状态；显式包含 E/T 的会话总时长上限 4.5 秒，结束/异常时尝试 D 并等待
失能应答，未收到确认会报错。查询期间没有完整回复也会报错，避免将打开适配器误当成通信成功。
Windows 下 gs_usb 使用 libusb-package 提供的 DLL；通道参数必须为非负整数。

本机 USB-CAN 的 VID/PID 为 `1D50:606F`，Windows 名称为 candleLight USB to CAN adapter，
序列号 `004700204346570B20363833`。目前没有枚举出 COM8，直接打开 COM8 也返回设备不存在；
该适配器应通过 gs_usb 使用，不能按旧 USB-TTL 的 COM8 参数操作。

没有 USB-CAN 时，可只读 SWD 诊断，不会发送使能命令：

```powershell
python tools/g431_probe.py --samples 2
```

探针从当前 MAP 解析 `g431_diagnostics` 的地址和大小（现在 24 words / 96 bytes），
读取时钟、心跳计数、实际驱动引脚状态、编码器错误、CAN 错误寄存器、CFSR/HFSR，
以及电流采样相关字段 `cs_valid / cs_samples / cs_trips / cs_blind / cs_peak /
cs_allowed / cs_thermal / cs_amp_mid`。脚本会校验结构体大小与它自己的字段表一致，
固件改了结构体却用旧脚本会直接报错而不是读错位。须使用与板上固件对应的 MAP。
阶段 4=主循环；0xE001=CAN 初始化失败；0xE002=处理器异常关断。

## 已完成与待完成的验证

2026-10-28 三块板：板 1、板 2、板 3 全部闭环正常（三块板同总线联调通过）；板 2 补焊功率级后
重新标定（`dir=−1/zero=0.122`），连续 10 次 1 rad 阶跃最大误差 0.0002 rad、40 s 往复
峰值 0.71 A 无跳闸、匀速抖动 9.6 mrad(1.5 rad/s) / 12.2 mrad(9 rad/s)，
参见 `../docs/g431-三块板标定-2026-10-28.md`。板 1 尚未写选项字节 `nSWBOOT0=0`。

2026-10-27 补测：G431 上已实测**开环转动、零点/方向/极对数标定、位置闭环、电压上限 2.5→8 V 扫描**，
全部通过；CAN 因缺外部 5V 仍不通。详见上文「G431 闭环打通（2026-10-27）」。

2026-10-27 相电流版完整 Rebuild：0 Error / 0 Warning，Code=20544、RO-data=1252、
RW-data=84、ZI-data=3636。

```powershell
./keil5_g431/Tests/run.ps1
./keil5_foc/Tests/run.ps1
```

通过既有 PWM、编码器解圈、速度 PI、294 组轨迹、故障和生命周期回归；增加默认上电失能
分支，以及 CAN 单帧 float32 往返精度、循环队列、队列满、D 优先、非法长度与 NaN/Inf 测试；
新增 `Tests/test_current_limit.c`：默认值必须与 `motor_config.h` 一致、折叠曲线、
停机锁存与计数、I²t 折减、峰值释放、失明时压到 1.5V、电压门限不越界。
CAN 的浮点转字符串在主循环执行，关中断仅覆盖队列取出，避免延长 nFAULT 响应。

ST-Link 识别实机为 Device ID `0x468`、128KB。SWD 恢复后，最终单帧 CAN 版本已下载、
Flash 校验成功并复位。HEX SHA256：
`875F92350A6940A130BE88C1033E0B0E13F9D0DD5ECED81F188962A279F46D2B`。

最终版运行快照确认：170MHz、阶段4、主循环计数增长、EN=0、nSLEEP=1、DRVOFF=1、
CFSR/HFSR=0。不过 **nFAULT=0，sensor_valid=0**，编码器错误累计。GPIOB 的采样中
SCL/SDA 为低；恢复总线会主动切换这两根线，因此寄存器快照不能代替接口电压及波形测量。
实测 24V 正常，SDA/SCL 约 0.1V，DRV 产生的 5V 只有约 1.7V；CAN 收发器 VCC
也取自这一路 5V。结合原理图审核记录，优先怀疑 Buck 输出端缺少或未焊接 `CBK 22uF/10V`
及 22~47uF 储能电容，或 47uH 电感/FB_BK 输出节点连接错误。5V 稳压恢复前不能判断
AS5600 损坏，也不能进行 CAN 或电机测试。

硬件复核顺序：断开 CAN VCC 负载后测电感后 5V；确认 SW_BK--47uH--5V、FB_BK 直接接
电感后输出、VSEL_BK(24脚)悬空、GND_BK 回功率地；补齐 22uF/10V + 22~47uF/10V。
5V 稳定后再接 CAN。若 AS5600 模块没有板载上拉，在 PB6/SCL、PB7/SDA 各加约 4.7kΩ
到 3.3V；当前固件使用开漏无内部上拉，不能依靠浮空线工作。

数据手册还说明：`nFAULT` 为低不一定代表芯片损坏。当前失能状态下固件将 `DRVOFF` 拉高；
DRV8316C 数据手册第 53 页明确指出，DRVOFF 独立关闭六个 MOSFET，并可能因此使 nFAULT
被拉低。除此之外，Buck 欠压（第 49 页）和 Buck 过流（第 52 页）也会拉低 nFAULT。
因此必须先把 5V Buck 稳定到 4.6~5.4V，再在保持 PWM/MOE 关闭的条件下复核 nFAULT。

在用户把 ST-Link 的 5V 线接到板上后，candleLight 在 500kbit/s 下已能稳定收发 `?` 和 `D`，
回复 ID/协议均正确，且 `EN=OFF`、`CAN txdrop=0`。这证明 CAN 连接和固件协议正常；但这一路
外接 5V 不能替代板上 Buck 的修复，不能在未确认电源隔离和防反灌前长期保持连接。
此时诊断仍为 `nFAULT=0`、`sensor_valid=0`、SDA/SCL 约 0.1V；尚未使能或旋转。
本轮没有发送 E/T，没有使能或旋转电机；软件构建与启动通过不等于PWM波形、校准或闭环运动通过。

原始启动快照：[bringup-snapshot.json](../docs/measurements/2026-10-16-g431/bringup-snapshot.json)。
最终版快照：[final-snapshot.json](../docs/measurements/2026-10-16-g431/final-snapshot.json)，
外设寄存器：[link-registers.log](../docs/measurements/2026-10-16-g431/link-registers.log)。
最终候选及参数备份：`D:\dsh_build\releases\g431-can-20261016`。
G431 原板 128KB Flash 备份：`D:\dsh_build\releases\g431-before-port-20261016.bin`。
F103 已确认版备份仍在 `D:\dsh_build\releases\gimbal-crisp-20261016`。

## 第三方来源

- ST STM32G4 HAL 1.2.7 / CMSIS Device 1.2.6：来自本机 STM32duino 包，已将所需文件复制进工程。
- ST ARMCC 启动文件：`STMicroelectronics/cmsis-device-g4` v1.2.4，栈改为 2KB。
- Arm CMSIS Core：本机 `ARM.CMSIS.5.7.0`。
- 许可见 `LICENSE-HAL-ST.txt`、`LICENSE-CMSIS-ST.txt`、`LICENSE-CMSIS-ARM.txt`；
  共享控制源码的 SimpleFOC 许可见 `LICENSE-SimpleFOC.txt`。
