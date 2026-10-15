# 方案 A：PlatformIO + 官方 SimpleFOC 库（已跑通）

> 日期：2026-10-15
> 状态：**编译通过、烧录成功、对齐成功、串口命令正常**
> 工程位置：`New_gimbal/foc_official/`

---

## 一、为什么选这条路

厂家 Keil 工程用的是 **2019 年的 SimpleFOC 移植版**，而你的工作区里
藏着一份**官方最新版完整库**（在 ESP32 项目的 PlatformIO 依赖里）：

```
常用测试源码-配合说明书使用\ESP32_SimpleFOC_BLE\ESP32_SimpleFOC_BLE\
  .pio\libdeps\lolin_d32\Simple FOC\
```

官方库**原生支持 STM32F1**：
```
src/drivers/hardware_specific/stm32/stm32_mcu.cpp        ← STM32 PWM 驱动
src/current_sense/hardware_specific/stm32/stm32f1/       ← F1 电流采样
src/sensors/MagneticSensorI2C.cpp  含 AS5600_I2C 配置
```

**而且官方自带 STM32F103(蓝板) 专用示例**：
```
examples/hardware_specific_examples/Bluepill_examples/
    magnetic_sensor/bluepill_position_control/     ← AS5600 闭环位置控制
    encoder/bluepill_position_control/             ← ABZ 编码器
```

**我们前面踩的坑（PWM 定时器、ADC 同步采样、对齐零点、看门狗干扰）
在官方库里都已经处理好**，所以直接用官方实现。

---

## 二、工程结构

```
New_gimbal/foc_official/
├── platformio.ini          工程配置（STM32F103C8 + Arduino + SimpleFOC）
├── patch_simplefoc.py      编译前自动修补 core 兼容问题
├── include/MyUart.h        寄存器直驱的极简串口
└── src/main.cpp            主程序（基于官方示例改写，只改引脚和参数）
```

## 三、引脚映射（与实物接线一致）

| 功能 | 引脚 | 说明 |
|---|---|---|
| PWM1/2/3 | **PA8 / PA9 / PA10** | TIM1_CH1/2/3 → 驱动板 IN1/IN2/IN3 |
| EN | **PB0** | 驱动板使能 |
| 串口 | **PA2(TX) / PA3(RX)** | USART2 |
| 编码器 | **PB6(SCL) / PB7(SDA)** | I2C1, AS5600 |
| SWD | PA13 / PA14 | 调试 |
| LED | PC13 | 心跳 |

**代码里只需要改两行**（对应官方 Bluepill 示例）：

```c
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PB0);   // 原: PB6,PB7,PB8,PB5
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);       // 原: MagneticSensorSPI
```

---

## 四、必须解决的两个兼容问题

### 问题 1：SimpleFOC 2.4.0 与新版 Arduino STM32 core 不兼容

**现象**
```
stm32_mcu.cpp:126: error: cannot convert 'TIM_HandleTypeDef*' to 'TIM_TypeDef*'
    enableTimerClock(handle);
```

**根因**
SimpleFOC 自己的 `enableTimerClock(TIM_HandleTypeDef*)` 被包在
`#if defined(__MBED__)` 里，在 STM32Duino 下**不参与编译**。
于是链接到了 Arduino core 的同名函数 `enableTimerClock(TIM_TypeDef*)`，
参数类型不匹配。

**修复**（一处）
```c
enableTimerClock(handle)  →  enableTimerClock(handle->Instance)
```
与 core 自己的用法一致（`timer.c:46  enableTimerClock(htim_base->Instance)`）。

**用 `extra_scripts` 自动打补丁**（见 `patch_simplefoc.py`），
这样重装库、清理构建都不会丢。

### 问题 2：Arduino 的 `Serial1` 在本 core 版本下无输出

**现象**
- `raw_puts(USART2, ...)` 直接写寄存器 → **正常输出**
- `Serial1.println(...)` → **完全无输出**

**结论**：`Serial1` 没有正确绑定到 USART2（本 core 版本 4.3.0 的问题）。

**修复**：自己写寄存器直驱的极简串口（`include/MyUart.h`），
不依赖 Arduino 的 `Serial` 类。更快也更可靠。

---

## 五、实测结果（固件已烧录运行）

```
=============================================
 SimpleFOC 官方库 / STM32F103C8T6 + DRV8316C
=============================================
PWM : PA8 / PA9 / PA10
EN  : PB0
I2C : PB6(SCL) / PB7(SDA)  AS5600
SysClk=72000000
---------------------------------------------
[1/5] 编码器 AS5600 @0x36 ... angle=4.133
[2/5] 驱动 PWM/EN ... OK
[3/5] 参数设置 ... OK
---------------------------------------------
[4/5] 安全参数
      Vbus      = 24.0 V
      电压上限  = 1.5 V  (约 0.4 A)
      对齐电压  = 1.0 V
      速度上限  = 20.0 rad/s
      极对数    = 7
---------------------------------------------
[5/5] motor.init() ... OK
开始对齐 (initFOC) —— 电机会轻微转动，属正常
---------------------------------------------
对齐结果: dir=CW  zero_electric_angle=3.494
电机已【失能】。确认安全后发 E 使能
```

**全部无错误**：时钟 72MHz、I2C 编码器、PWM 驱动、FOC 对齐一次成功。

命令接口验证：
```
> R
EN=OFF  angle=4.126  vel=-0.158  tgt=0.000  Vlim=1.5  Vbus=24.0  dir=1  zero=3.494

> V1.5
Vlim=1.5V (约 0.4A)
```

## 六、资源占用

```
RAM:   14.2% (2912 / 20480 bytes)
Flash: 83.9% (54960 / 65536 bytes)     ← 偏满，但可用
```

---

## 七、安全设计

| 措施 | 值 |
|---|---|
| 电压上限 | **1.5 V**（约 0.4A，额定 1.4A 的 29%） |
| 对齐电压 | 1.0 V |
| 速度上限 | 20 rad/s |
| initFOC 后状态 | **自动失能**，需发 `E` 才使能 |
| 急停 | 发 `D` 立即失能 |
| 在线调限压 | `V<n>`，超过 3V 会警告，超过 6V 自动夹住 |

## 八、串口命令

| 命令 | 作用 |
|---|---|
| `E` | 使能电机 |
| `D` | 失能（急停） |
| `T<n>` | 目标角度 (rad) |
| `V<n>` | 电压上限 (V) |
| `L<n>` | 速度上限 (rad/s) |
| `R` | 读编码器角度/速度 |
| `?` | 状态 |

## 九、构建与烧录

```powershell
cd New_gimbal/foc_official
python -m platformio run              # 编译
python -m platformio run -t upload    # 烧录（ST-Link）
python -m platformio device monitor   # 串口监视
```

首次编译会自动下载 STM32 平台 + Arduino 框架 + SimpleFOC 库（约 5 分钟）。

---

## 十、下一步（谨慎验证）

**当前电机是失能状态，固件也很保守。** 下一步验证建议：

1. **先确认接线**：驱动板 IN1/IN2/IN3 ↔ PA8/PA9/PA10，EN ↔ PB0，GND 共地
2. 发 `?` 确认 `EN=OFF`、`angle` 有合理读数
3. **用手转动电机轴**，发 `R` 看 angle 是否跟着变（验证编码器方向）
4. 发 `E` 使能，**手放在电源开关上**，观察是否仅轻微保持位置
5. 发 `T0.5` 给一个小目标角度，看电机是否平滑转过去
6. **全程用电流表监控**，预期电流 < 0.4A

**注意**：官方库默认 `motor.voltage_limit` 是 1.5V，若闭合成功且电流正常，
可逐步试探 `V2`、`V3`。**每次只加 0.5V，且必须在电流表监控下。**
