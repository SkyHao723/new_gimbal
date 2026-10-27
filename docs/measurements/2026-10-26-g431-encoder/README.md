# G431 新 PCB：编码器与带电诊断（2026-10-26）

## 当前结论

已定位硬件 I2C 无法工作的确定原因：**STM32G431CBTx 的 PB6 没有 I2C1_SCL 复用功能**。之前工程沿用了 F103 的 PB6/PB7 硬件 I2C 配置；通用 GPIO_AF4_I2C1 宏存在，并不表示 PB6 支持这个功能。这不是改 I2C 时序或上拉就能解决的问题。

ST 官方 STM32_open_pin_data 中的芯片引脚与 GPIO 复用 XML 已保存到本目录。PB7 支持 I2C1_SDA；I2C1_SCL 可选 PA15 或 PB8。板上 PB8 已作 CAN RX，不能直接挪用；PA15 的改线方案也需要先核对板上实际连接。本次没有要求改线，PB6/PB7 使用开漏软件 I2C 已可读取。

来源：https://github.com/STMicroelectronics/STM32_open_pin_data ，路径 mcu/STM32G431C(6-8-B)Tx.xml 与 mcu/IP/GPIO-STM32G43x_gpio_v1_0_Modes.xml。原始下载时间为本次诊断日期，未伪造文档版本。

## 实际使能结果

用户授权后完成一次 **1.5V 电压指令、1200ms 开环动作**，实际记录位于 pwm-warning-test.json / .log：

| 指标 | 结果 |
|---|---:|
| 独立 SysTick 停机时间 | 1200ms |
| 编码器记录的净转动 | -40.43° |
| 测试新增通信错误 | 0 |
| 采集到的运动中 nFAULT | 全部为高，未报故障 |
| STATUS | 全部 0x33，MD=1 / ML=1 / MH=0 |
| 运动中幅值寄存器采样范围 | 977～986，原始数值、不是 mT |
| 结束输出状态 | nSLEEP=0、TIM1.MOE=0、active=0 |

驱动休眠、仅唤醒和实际 PWM 期间均有 ML 提示；这次低压短时试验没有发现 PWM 引发的新增通信错误或磁场状态突变。不能据此证明所有速度/负载下均无干扰。角度由编码器给出，未用独立机械仪表校验精度。

用户明确说明弱磁此前一直存在，要求暂缓排查。本次不要求拆解厂家预装磁铁，也不把 ML 当作本轮通信故障的根因。

## 已更新固件

- Bringup/main.c 上电默认软件 I2C，无需先从 SWD 切换；错误的 PB6 硬件 I2C 路径已移除，旧命令 5/8 被拒绝。
- 禁用未使用的 UCPD dead-battery 下拉，避免 PB6/UCPD1_CC1 的潜在交互。这是预防性配置，不是已证明的本次故障根因。
- 新增显式命令 9 / host spin15warn，只用于 1.5V、1.2s 开环诊断。允许 MD=1、ML=1、MH=0；仍要求新鲜角度通信，通信错误、磁铁丢失、MH、nFAULT 或超时均停机。普通运动命令仍保留严格磁场门限；没有用问题角度建立闭环。
- 当前仍是独立诊断程序，无 CAN 或位置闭环；DRVOFF 已接地，停机依赖 PB0/nSLEEP 与 TIM1.MOE。PB1 保持模拟输入。默认启动不转。
- 最终版本编译 0 Error / 0 Warning，下载校验成功。复位后直接记录 456 次成功读取、0 次通信错误，确认输出始终关闭，见 software-default-boot.json。

最新 HEX SHA256：82B50BB95E79255BA11CF632822FB2F9188B13DA551E1DB49442B6C5C6EFEF18

程序：keil5_g431/g431_bringup.uvprojx、keil5_g431/Bringup/main.c、tools/g431_bringup.py。普通旧 CAN 固件仍有 PB1/DRVOFF 与硬件 I2C 的旧板假设，不能直接刷入这块新 PCB。

## 历史记录和下一步

manual-movement.json / summary.json 保留此前失能手动转轴记录：18.556s 内净变化 164.795°，4505 次新增成功读取、0 次新增通信错误。其他 encoder-hardware-*、encoder-soft*、wake-comparison 等保留排查过程，不代表最终配置。

active-diagnosis-summary.json 汇总本次带电诊断。备份目录 D:/dsh_build/releases/g431-encoder-software-default-20261026；前一版备份仍保留。

## 使用 PB6/PB7 软件 I2C（2026-10-27）

按用户选择暂用 PB6=SCL、PB7=SDA 软件模拟 I2C，无需改板或加 PA15 跳线。诊断 bring-up 固件采用 25us 半周期软件总线，已经重新编译、下载并校验；复位后电机保持关闭。普通 G431 BSP 的 AS5600 驱动也已改为同样的 GPIO 开漏软件 I2C：初始化时恢复总线，运行时读取角度寄存器；读失败会关闭驱动、累计错误并恢复总线，不用旧角度继续控制。

G431 引脚信息表已确认 PB6 没有 I2C1_SCL 硬件复用，因此此接线下片上 I2C1 不能直接使用；用户当前选择暂不改线。诊断固件的实机软件读取有零通信错误记录。普通 FOC 软件 I2C 版本已成功编译（0 warning/error），但尚未上板运行或测量它的闭环刷新率：当前新 PCB 的 DRVOFF 接地，旧正常工程的 PB1/DRVOFF 控制假设不适用，不能将普通工程直接刷入该板。复位后实测 2.272s 内新增 551 次成功读取、0 次通信错误，约 243Hz，见 software-i2c-pb67-after-flash.json。

弱磁告警按用户要求暂缓。后续在安全的板卡接线条件下应测量普通 FOC 软件读取路径的实际循环频率和角度延迟，再决定是否改用具备硬件 I2C_SCL 的引脚；之后确认方向、极对数和电角度零点，再进入闭环验证。
