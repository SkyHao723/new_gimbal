# STM32F103C8T6 电机控制：Keil5 标准库工程

2026-10-16，从 `../foc_official/src/main.cpp` 当前 PlatformIO 版本迁移。
这是 **纯 C99 + STM32F10x 标准外设库 V3.5.0** 的独立工程，包含本地 CMSIS、
启动文件和用到的 SPL 源码，不需要 Arduino、PlatformIO 或原厂旧 SimpleFOC C 库。
目标芯片仍是当前蓝板 **STM32F103C8T6**；G431 需要另行适配外设。

## 打开与编译

1. Keil µVision5 打开 `gimbal_foc.uvprojx`，选择 `F103C8_FOC`。
2. 使用 **ARM Compiler 5.06**、MicroLIB、C99，设备包为 `Keil.STM32F1xx_DFP.2.2.0`。
   以上配置已经写入工程；本机 Keil 安装位置为 `D:\keil`。
3. 按 F7 构建。输出为 `Objects/gimbal_foc.axf` 和 `Objects/gimbal_foc.hex`。
4. Debug/Utilities 配置使用 ST-Link；连接方式为 SWD，建议 1MHz。
   设备的 Flash 范围为 `0x08000000 + 64KB`，RAM 为 `0x20000000 + 20KB`。

中文路径可能影响旧 ARM 工具链。命令行构建脚本会创建 ASCII 目录联接，文件仍在本目录：

```powershell
.\build.ps1
# 其他安装位置/磁盘可指定：
.\build.ps1 -KeilExe C:\Keil_v5\UV4\UV4.exe -BuildAlias C:\work\gimbal_keil
```

本机已创建 `D:\dsh_build\keil5_foc` 联接，可以直接从这个目录打开工程。
整个 `keil5_foc` 文件夹可单独复制到其他计算机；编译器和设备包需要在 Keil 中安装。
本工程已验证 ARM Compiler 5；尚未验证直接切换 ARM Compiler 6。

## 接线

| 功能 | 引脚/外设 |
|---|---|
| 三相 PWM | PA8 / PA9 / PA10，TIM1 CH1/2/3，约 25kHz 中心对齐 |
| 驱动 EN | PB0，高电平使能 |
| USB-TTL | PA2 TX / PA3 RX，USART2，115200 8N1 |
| AS5600 | PB6 SCL / PB7 SDA，I2C1，100kHz，地址 0x36 |
| 调试 | PA13 / PA14，SWD；仅关闭 JTAG，保留 SWD |
| LED | PC13 |

## 保留的行为与参数

参数集中在 `User/motor_config.h`：

| 参数 | 值 |
|---|---|
| 极对数 / 编码器方向 | 11 / CW（+1） |
| 固定电角度零点 | 0.92 rad |
| 母线参数 / 初始电压上限 / 硬上限 | 24V / 3V / 5V |
| 速度 PI | P=0.2，I=5，电压斜率 1000V/s |
| 位置 P / 速度低通 | 20 / 0.05s |
| 位置目标斜坡 | 4 rad/s，且不超过用户的速度上限 |
| 速度上限 | 默认 20 rad/s，串口允许 0 < L <= 20 |
| 上电自检 | 正向一圈，4 rad/s，剩余角度 <=0.03 rad 时失能；10 秒超时失能 |

上电使用已有方向和零点，**不做正反扫角**。`STARTUP_ROTATE_ONCE=1` 时，
初始化后会自动转一圈，再失能；改为 `0` 则上电保持失能。
固定校准仅适用于当前电机、相线顺序和磁铁安装位置；改变这些条件后需要重新标定。

单圈结束由编码器的**实际累计机械角**判断，开环磁场角与实测角分别存储，避免把
电角周期当成机械一圈。普通位置控制使用多圈角度，支持大于 ±2π 的绝对目标。

## 串口与网页

| 命令 | 行为 |
|---|---|
| `E` | 从失能进入使能时，以当前位置为目标并清零 PI；重复 E 保留目标和 PI |
| `D` | 失能并取消上电自检；接收中断立即拉低 EN，随后主循环清空旧命令 |
| `T<n>` | 设置绝对机械角目标，单位 rad；在自检中收到时切回位置控制 |
| `V<n>` | 设置电压上限，最多 5V；非正值恢复 3V |
| `L<n>` | 设置速度上限，范围 0 < L <= 20 rad/s |
| `R` | 读取编码器累计角、滤波速度、单圈角 |
| `?` / `H` | 状态 / 帮助 |

命令以 CR 或 LF 结束。正常接收队列可缓存 15 条待处理命令；发送过长行时整行丢弃。
急停优先于普通队列；重新使能需要在急停应答后另外发送 E。
TX 使用 1024 字节中断缓冲，正常状态查询不会逐字节阻塞 FOC 循环。

状态保留 `EN=ON/OFF angle= vel= tgt= Vlim= Vbus= dir= zero=` 字段，可继续使用
`../tools/web_controller/index.html`。额外的 `pp=FIXED` 表示使用保存的极对数，
**本次启动没有重新执行极对数检测**；Vbus 是配置值，未接入母线电压采样。

## 移植实现与差异

- `Control/foc_math.c`：从官方 SimpleFOC 2.4.0 的 PI、65 点正弦表和逆 Park/Clarke +
  中点钳位 SVPWM 移植，保留 MIT 版权声明，见 `LICENSE-SimpleFOC.txt`。
- `Control/motor.c`：位置/速度控制、目标斜坡、固定校准、上电单圈状态和串口命令。
- `Control/encoder_tracking.c`：12 位角度解圈、速度差分和低通。
- `BSP/as5600.c`：按 STM32F1 两字节接收时序读取 0x0C/0x0D；每个等待点有 2ms 超时，
  失败时发 STOP、恢复总线。传感器读取失败会失能，需要重新 E 才恢复输出。
- `BSP/board.c`：SPL GPIO、TIM1、USART2、I2C 时钟；TIM2 提供 1MHz 的 32 位微秒计时，
  SysTick 提供毫秒计时。硬件故障异常会拉低 EN。

这次迁移保留控制参数，但不等于直接编译原 C++ 库：省去了未使用的控制模式和 Arduino
接口。速度反馈每次有效采样都更新时间，静止样本会使速度低通逐渐回到零，避免沿用旧速度；
目标斜坡也会服从 L 指令。非法数字、NaN、无穷和带尾随字符的参数会被拒绝。

采用电压力矩模式，未实现电流闭环或硬件过流采样；5V 限制是软件电压约束。

## 已完成的验证

2026-10-16，Keil µVision5 + ARMCC 5.06 完整 Rebuild：**0 Error(s), 0 Warning(s)**。

```text
Program Size: Code=13076 RO-data=556 RW-data=68 ZI-data=3836
Flash: 13700 bytes（Code + RO-data + RW-data）
RAM:    3904 bytes（RW-data + ZI-data，含 2KB 栈和 512B 堆）
```

构建日志：`Objects/keil_build.log`；链接映射：`Listings/gimbal_foc.map`。
离线测试可用本机 MinGW GCC 运行：

```powershell
.\Tests\run.ps1
# 或指定可用的主机 GCC：
.\Tests\run.ps1 -Gcc C:\tools\mingw64\bin\gcc.exe
```

测试通过：13,211 个 SVPWM 矢量对照（最大误差 0.000758V）、编码器正反两圈/计时回绕/
静止速度衰减、PI 饱和、运行时降压、上电单圈完成/超时、E 幂等、目标斜坡、参数校验、
D 失能及传感器故障处理。测试使用模拟角度输入，不代表实机噪声或位置跟随性能已经验证。

2026-10-16 用户已完成实机测试并反馈：无可感知噪音，停在指定位置时不抖动，
运动时仍有顿挫感。该稳定版本记录见 [稳定基线](../docs/稳定基线-2026-10-16.md)。
这是用户的实机观察；本轮自动化验证仍为编译和离线测试。
