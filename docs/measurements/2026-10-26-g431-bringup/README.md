# G431 新 PCB 首次开环转动验证（2026-10-26）

## 实际接线与供电

- 用户确认：自制 STM32G431 PCB，仅接电机三相线、24V、ST-Link；没有 AS5600、TTL 或 USB-CAN。
- 与旧图的区别：DRV8316C 的 21 脚 DRVOFF 直接接地，不接 PB1。PB1 在本测试中设为模拟输入。
- 用户确认 MCU 由 ST-Link 的 3.3V 供电；本次没有验证板载 5V Buck 的电压、负载能力和长期可靠性。
- ST-Link SN：37FF71064E57343682CD1A43；Device ID=0x468，128KB Flash，探针报告目标电压 3.24V。

## 本次修改与程序

1. 全片回读确认原 Flash 是 0xFF，备份 `D:/dsh_build/releases/g431-before-bringup-20261026.bin`。
2. 创建独立 `keil5_g431/g431_bringup.uvprojx`，主程序 `keil5_g431/Bringup/main.c`；原 CAN/闭环主程序、F103 参数和共用控制源码没有修改。
3. Keil ARMCC 5.06：0 Error / 0 Warning；Code=5396、RO=660、RW=40、ZI=2144。下载及 Flash 校验成功。
4. 新板默认 nSWBOOT0=1，下载后复位未运行应用。改为 nSWBOOT0=0，原 nBOOT0=1、nBOOT1=1 保留，改为按选项字节从 Flash 启动；没有改变读写保护。详见 before.log、boot-option.log。
5. 最终 HEX SHA256：`5E0CEA59B2E1E63239111D98C8617A6AFE5F245F0C51653E8A07CAC0BE1029EA`，路径 `keil5_g431/ObjectsBringup/g431_bringup.hex`。

该固件不使用编码器、CAN、固定电角度零点或闭环位置控制。开环采用既有 SVPWM 数学实现，按 24V 母线配置和 11 极对数生成三相参考；未测实际相电压、电流或转速。

## 测试步骤与结果

- 默认上电不转：PB0/nSLEEP=0、TIM1.MOE=0。先通过 SWD 请求唤醒，PWM 始终关闭；唤醒后 nFAULT 连续两次读取为 1，170MHz 主循环正常、CFSR/HFSR=0。
- 再执行**仅一次** 1.5V 电压指令开环输出，最大机械参考速度 1rad/s；前 150ms 缓升电压，随后缓升旋转速度，最后 150ms 降低电压。
- 记录 started_ms=17495、定时停止 ended_ms=18695，严格为 1200ms；pulses=1、updates=138071，运行期间所采 nFAULT=1，无处理器异常。
- **用户现场反馈：有明显转动，声音正常。** 这验证了该接线与本次工况下能够驱动电机；没有编码器，因此不能依据合成角度断言实际转角、转速或是否精确跟随。
- 未执行 2V 候选，没有追加旋转、负载测试或电流闭环验证。

## 停机与适用边界

每次输出由板端 SysTick 在 1200ms 截止时拉低 nSLEEP、清 TIM1.MOE；nFAULT 中断和主循环也能停止。独立看门狗约 0.5s 标称超时复位到默认不转。正常完成后额外发送停止命令并读取实际寄存器：nSLEEP=0、MOE=0、active=0。final.json 再次确认已经停止。休眠时读到 nFAULT=0，不应当作运行期间发生了故障。

当前板上是独立短时验证固件，**重新上电不会自动旋转**。nSLEEP 关断适用于本次确认的 MCU 独立供电条件；若以后改回 DRV 派生电源给 MCU 供电，必须重新核对供电和停机机制。旧 CAN 固件的 PB1/DRVOFF 停机假设与这块新板不同，不能不适配就覆盖下载。

## 复查工具

只读状态：

```powershell
python tools/g431_bringup.py check --output docs/measurements/2026-10-26-g431-bringup/check.json
```

`tools/g431_bringup.py` 仅操作此带身份标记的验证固件。每次输出需显式唤醒与请求，不能由开机自动触发。正常测试输出、唤醒、停机和下载记录保存在同目录 JSON / LOG。wake.json 保留了早期 SWD 写后自动校验与请求消费之间的竞争错误；该次只有唤醒/停止，没有旋转。随后采用稳定请求/应答字段修正，见 wake-v2.json 和 spin15.json。
