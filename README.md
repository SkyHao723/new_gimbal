# New Gimbal — 云台电机测试工程

基于 **STM32F103C8T6 + SimpleFOC Mini (DRV8316C) + AS5600** 的云台电机测试工程。
目标是先把**测试版**（开发板 + 厂家驱动小板）的流程打通，为后续 G431 自制板的正式版打基础。

> 状态：**电机已成功转动，空载电流 < 0.1 A**（厂家标称空载 0.1 A）

---

## 目录结构

```
New_gimbal/
├── app/                     测试版应用代码（当前使用）
│   ├── mini_test.c          ★ 主程序：六步换相 + 电流采样 + 过流保护
│   └── Hardware/
│       ├── timer.c/.h       （参考版本，当前构建未使用）
├── build/                   构建系统
│   ├── build_mini.ps1       ★ 一键构建脚本
│   ├── startup_gcc.s        GCC 启动文件
│   ├── stm32f103c8.ld       链接脚本（64K Flash / 20K RAM）
│   ├── syscalls.c           newlib 桩（让 printf 可用）
│   └── diag_common.c        RCC 时钟配置（72MHz）+ TIM4 中断
├── library/                 STM32F10x 标准外设库 (SPL v3.5.0)
│   ├── lib/inc, lib/src
│   └── startup/
├── reference/               参考工程（SimpleFOC 移植版，用于将来移 G431）
│   └── SimpleFOC_port/
│       ├── SimpleFOC/       BLDCMotor / FOCMotor / MagneticSensor / pid ...
│       ├── user/main.c      ★ 正式版主程序（已做安全重构）
│       └── Hardware/        timer.c (TIM1 PWM) / iic.c / usart.c / delay.c
├── foc_official/            ★ 方案A：PlatformIO + 官方 SimpleFOC 库（已跑通）
│   ├── platformio.ini
│   ├── patch_simplefoc.py   编译前自动修补 core 兼容问题
│   ├── include/MyUart.h     寄存器直驱串口
│   └── src/main.cpp         主程序
├── tools/web_controller/    ★ 浏览器控制器（Web Serial，单文件）
│   ├── index.html
│   └── README.md
└── docs/
    ├── 接线检查表.md
    └── 调试总结与安全指南.md    ★ 4 个关键 bug 的根因与修复 + 安全规程
```

---

## 快速开始

### 1. 接线

| 功能 | STM32F103 引脚 |
|---|---|
| PWM1 / PWM2 / PWM3 | **PA8 / PA9 / PA10**（TIM1） |
| EN（使能） | **PB0** |
| 串口 | **PA2(TX) / PA3(RX)**（USART2，115200） |
| 编码器 I2C | **PB6(SCL) / PB7(SDA)**（AS5600） |
| 电流采样 CS1/CS2/CS3 | **PA0 / PA1 / PB1** |
| SWD | PA13 / PA14 |
| GND | 全部共地 |

驱动板进 **24 V**；蓝板用 USB 或 5 V 脚（**绝不能进 24 V**）。

详见 `docs/接线检查表.md`

### 2. 构建

```powershell
powershell -ExecutionPolicy Bypass -File build\build_mini.ps1
```

产物：`build\out\mini.hex`

需要 `arm-none-eabi-gcc`。脚本会自动从 STM32CubeCLT 默认路径查找，
也可用环境变量指定：

```powershell
$env:ARM_GCC_BIN = 'D:\program\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin'
```

### 3. 烧录

```powershell
STM32_Programmer_CLI -c port=SWD -w build\out\mini.hex -v -g 0x08000000
```

### 4. 上电与测试

用串口助手（115200-8-N-1）连接，**按顺序**执行：

```
V2      电压上限 2V   (约 0.3A)
L1.5    过流阈值 1.5A
C1      打开过流保护
S60     每步 60ms（慢速）
R       开始旋转
```

预期：电机缓慢转动，电流约 **0.3 A**。
确认正常后再逐步试 `V3`、`V4`。

---

## 串口命令

| 命令 | 作用 |
|---|---|
| `?` | 状态（EN / 保护 / 电压 / 电流 / CCR） |
| `E` / `D` | 使能 / 失能（**急停**） |
| `V<x>` | 电压上限 (V) |
| `L<x>` | 过流阈值 (A) |
| `C0` / `C1` | 过流保护 关 / 开 |
| `S<x>` | 每步毫秒数（默认 20，越大越慢） |
| `R` | 开始旋转（带保护，自动停） |
| `Z` | 零电流测试（锁定转子，逐步加压测电流） |
| `I` | 读电流 |
| `K` | 重新标定电流零点 |
| `J<x>` | 电流标定系数 |

---

## 安全须知（重要）

### 电压上限与电流的关系

电机实测 **两相串联电阻约 6.5 Ω**，因此 `I ≈ V / 6.5`：

| voltage_limit | 电流 | 占额定 1.4A | 判定 |
|---|---|---|---|
| 2 V | 0.31 A | 22% | ✅ 很安全 |
| **5 V** | **0.77 A** | **55%** | ✅ 推荐 |
| 9 V | 1.38 A | 99% | ⚠️ 上限 |
| 10 V | 1.54 A | 110% | ❌ 超额 |
| 24 V | 3.7 A | 264% | ❌❌❌ |

### 绝对避免

- ❌ `voltage_limit` 超过 10 V
- ❌ 开环旋转时把每步毫秒数设得太小（磁场太快 → 转子**失步** → 等效堵转 → 大电流）
- ❌ 电机未固定就上电
- ❌ 电机烫手时继续通电

### 出现以下任一现象立即断电

- 电流 > 1.5 A
- 电机烫手
- 异响 / 抖动
- 电流突然飙升

> **一次真实事故**：`voltage_limit=12V` + 开环每步 5ms 导致失步，
> 电流冲到堵转值，**145 W 灌进 33 W 的电机**，电机瞬间烫手。
> 完整复盘见 `docs/调试总结与安全指南.md`。

---

## 已验证 / 待办

| 项目 | 状态 |
|---|---|
| PWM 输出 (TIM1, 25kHz) | ✅ 已验证 |
| 六步换相旋转 | ✅ 已验证（空载电流 < 0.1A） |
| 电流采样 + 零点标定 | ✅ 已验证（漂移 ±0.005A） |
| 过流硬切断保护 | ✅ 已验证 |
| 72 MHz 时钟 | ✅ 已验证 |
| AS5600 编码器读取 | ⚠️ 偶发读失败，已加 I2C 超时清理待重测 |
| 闭环 FOC（对齐 + 角度/速度环） | ⚠️ 待打通 |
| G431 正式版移植 | ⚠️ 待开始（参考 `reference/SimpleFOC_port`） |

---

## 参考

- SimpleFOC 官方：https://simplefoc.com/
- 原移植教程（loop222）：https://blog.csdn.net/loop222/article/details/120471390
- DRV8316C 数据手册：TI SLVSH07
- STM32F103 数据手册：ST DS5319

