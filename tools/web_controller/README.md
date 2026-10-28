# 云台电机 Web 控制器

单文件浏览器的电机调试面板。两种连接方式：

| 方式 | 适用板子 | 链路 |
|---|---|---|
| **CAN 桥**（推荐） | G431 正式版（`keil5_g431`） | 浏览器 → WebSocket → `tools/g431_can_bridge.py` → candleLight → FDCAN |
| **Web Serial** | F103 测试版（`keil5_foc`、`foc_official`） | 浏览器 → Web Serial → USB-TTL → USART |

页面本身不区分板子，命令集是同一套（`E D T V L R ?`），所以两种后端可以共用。

---

## 1. 用 CAN 桥驱动 G431（推荐）

G431 板上唯一的上级链路是 FDCAN（TJA1042 → candleLight），固件里没有任何 UART，
所以浏览器不能直连。用一个本地小桥把 WebSocket 翻译成 CAN 帧：

```powershell
# 1) 插上 candleLight，确认板上 5V 已接（TJA1042 的 VCC 来自 5V，缺了它一帧都通不了）
python tools/g431_can_bridge.py --nodes 1,2,3     # 三块板；单块用 --nodes 1

# 2) 浏览器打开桥自己托管的页面
#    http://127.0.0.1:8766/
```

**三块板怎么用**：页面连上桥后，「节点」下拉框会自动出现（桥上报的节点列表决定），
选哪块就控制哪块；下面的「三块板总览」表格同时显示 node1/2/3 的使能、角度、目标、
误差、电压上限、电流峰值，点表格里任意一行也能切换节点。选「广播 (所有板)」时命令发到
`0x200`，三块板都会执行但都不回包（适合急停 `D`）。

桥做三件事：

1. 在 `127.0.0.1:8766` 提供本页面，并注入 `window.__G431_BRIDGE__`；
2. 在 `127.0.0.1:8765` 收页面发来的 JSON 命令 `{"node":2,"cmd":"T1.5"}`，编码成 CAN 帧
   发到 `0x200+node`；`{"node":"all"}` 依次发给所有节点；
3. 把各板 `0x280+node` 回来的 ASCII 文本带上节点号转给页面：`{"node":2,"text":"..."}`。

页面检测到注入的标记就自动走 WebSocket，否则退回原来的 Web Serial，所以同一个
`index.html` 两种板子都能用（Web Serial 模式下没有节点概念，隐藏选择器）。

参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--channel` | 0 | candleLight 通道号 |
| `--bitrate` | 500000 | 与固件一致，不要改 |
| `--nodes` | `1,2,3` | 桥监听的节点号（命令 `0x200+n` / 回复 `0x280+n`），逗号分隔 |
| `--node` | — | 单块板的快捷写法，等价 `--nodes N` |
| `--ws-port` / `--http-port` | 8765 / 8766 | 端口冲突时改这里 |
| `--no-http` | 关 | 只用别的静态服务器托管页面时打开 |

依赖：`python-can`、`websockets`、`libusb-package`（`pip install python-can websockets libusb-package`）。
candleLight 需要 libusb 后端，脚本里已经做了 pyusb 的 monkeypatch，不用另装 Zadig。

---

## 2. 用 Web Serial 驱动 F103（旧方式）

双击 `index.html`（或用任意静态服务器打开）→ 选 COM 口 → 115200 8N1。
需要 Chrome/Edge（`navigator.serial`）。

---

## 3. 命令集

| 指令 | 作用 |
|---|---|
| `E` | 使能并保持当前位置；之后发 `T<角度>` 才会移动 |
| `D` | 失能 / 急停（G431 上会真正拉低 nSLEEP 断电） |
| `T<角度>` | 目标角度 (rad)，如 `T0.5` |
| `V<电压>` | 电压上限 (V)，如 `V2.5`（G431 硬上限 8 V，F103 为 5 V） |
| `L<速度>` | 速度上限 (rad/s) |
| `M0` / `M1` | 运动模式：`M0` 缓动（轨迹规划，默认）/ `M1` 直驱（不做缓启缓停，位置环直接拉到位） |
| `X` | 抓 3 秒黑匣子波形（角度/速度/uq 存 RAM，事后用 ST-Link 读） |
| `R` | 读编码器（角度 / 速度 / 机械角） |
| `?` | 状态：EN / angle / vel / tgt / Vlim / Vbus / dir / zero / mode，G431 还会多一行电流保护信息 |

每个命令之间要留一点间隔（桥里固定 35 ms，串口模式页面自己串行化），
板子主循环每轮只取一条命令。

---

## 4. 安全提示

- 使能前确认电机被固定、手放在断电开关上；
- 第一次上电先用 `V2.5` 这类低电压上限试方向；
- G431 的状态行里 `I pk` 是实测峰值相电流，`allowed` 是限流器当前允许的电压上限，
  `trips` 是累计跳闸次数；
- 看到 `FAULT overcurrent` 说明真的过流了，先查机械阻力再抬限制。

---

## 5. 排错

| 现象 | 原因 |
|---|---|
| 页面显示"未连接"、点连接报连不上 | 桥没启动，或端口被占用（`Get-NetTCPConnection -LocalPort 8765`） |
| 桥启动报 `No backend available` | `libusb-package` 没装，`pip install libusb-package` |
| 命令发出去没回应 | 检查 candleLight 是否插好、CANH/CANL 是否接反、**板上 5V 是否接** |
| 状态停在 `EN=OFF` 且 `nFAULT=0` | 驱动器报故障（母线欠压/过流锁存），板子空闲时会自动重试唤醒 |
