# ESP32-S3 ↔ G431 上位机板 命令桥

ESP32 作为"透明桥"，把电脑 USB 串口上打的命令转发给 G431 上位机板，再把 G431 的回复打回电脑。

```
PC 终端  <-- USB CDC -->  ESP32  <-- UART 115200 8N1 -->  G431 (PA2/PA3)
```

文件：`esp32_bridge.ino`（Arduino IDE 打开即可）

---

## 1. 接线

| 信号 | ESP32 | G431 上位机板 |
| --- | --- | --- |
| ESP32 TX → G431 RX | TX | **PA3**（USART2_RX） |
| ESP32 RX ← G431 TX | RX | **PA2**（USART2_TX） |
| 地 | GND | GND |

- 两块板各自供电时**不要再互连 3V3/5V**，只共地。
- 一定要注意 TX↔RX 交叉：ESP 的 TX 接 PA3，ESP 的 RX 接 PA2。
- 板子上的 `invalid header: 0xffffffff` 刷屏就是 ESP32 ROM 在报"flash 里没有有效程序"，刷完本固件后这条流就消失了。

---

## 2. Arduino IDE 设置

| 项 | 值 |
| --- | --- |
| Board | `ESP32S3 Dev Module`（按实际模块选） |
| **USB CDC On Boot** | **Enabled**（这样 `Serial` 才是原生 USB，UART0 才能留给 G431） |
| Upload Mode | 按你板子的下载口选（USB-OTG 或 UART0） |
| Flash Size / PSRAM | 默认即可 |
| 串口监视器波特率 | 任意（CDC 不看波特率），建议 115200 |

编译产物直接烧进去，监视器里应看到：

```
[esp32] bridge 1.0.0 booting
[esp32] chip=ESP32-S3  USB CDC on boot=1  free heap=...
[esp32] G431 link: UART0  TX=-1 RX=-1  115200 8N1
[esp32] G431 is alive: "EN=OFF angle=... dir=-1 zero=1.2270 ..."
[esp32] ready - type !help for commands
```

若最后一行变成 `no reply from G431 ...`，会自动跑一遍 `!probe`，逐个 UART 试。

---

## 3. 怎么用

在串口监视器里直接打 G431 的命令（**不用**自己加 `@`，桥会自动加）：

```
?            状态
m            读 IMU
b            板级状态
c1 T0.5      给 1 号驱动板发 T0.5
c0 V2.5      广播电压上限
```

ESP32 本地命令（`!` 开头，不会转发）：

| 命令 | 作用 |
| --- | --- |
| `!help` | 帮助 |
| `!stat` | 链路统计（收发字节/命令数/静默时长） |
| `!ping` | 发 `@?` 并测往返时间 |
| `!probe` | 把 UART0/1/2 都试一遍，找哪个口能通 G431 |
| `!raw on` / `!raw off` | G431 回包按十六进制显示（查波特率/噪声用） |
| `!reboot` | 重启 ESP32 |

---

## 4. 与 G431 固件的协议约定

G431 那边的规则（见 `../keil5_host/README.md`）：

1. ESP 通道**只执行 `@` 开头的行**——这是防止 TX/RX 短接或对方刷屏时自激；
2. 波特率必须 **115200 8N1**；
3. ESP 通道输入速率超过 3 kB/s 会被 G431 判为噪声并**静音 5 秒**，所以不要用 ESP 连续高速刷命令；
4. 回复只回到发起方：从 ESP 发的命令，回复只从 ESP 通道出来（即本桥会打回电脑）。

---

## 5. 常见问题

### 5.1 USB 口不稳定 / 换一次插就换 COM 号

原生 USB CDC 的口在 Windows 上确实会随枚举变化（COM16 → COM14 之类），排查顺序：

1. **线材和接触**：换短一点的优质 USB 线，直插主板后置口，别经过 hub。
2. **USB CDC On Boot**：必须是 `Enabled`；如果是 `Disabled`，`Serial` 会被当成 UART0，既看不到输出也会和 G431 抢 UART0。
3. **供电**：电机一转就掉线，多半是 5V 跌落——ESP32 单独供电，或加大电容。
4. **别开"下载模式"**：BOOT 键按住/IO0 被拉低会进 ROM，表现为端口反复消失。
5. **换一路调试**：G431 自己有 CH343P（USB 直连 G431），日常调试优先用它；ESP 桥用于"最终形态"联调。
6. 监视器里若一直刷 `invalid header: 0xffffffff`，说明**还没刷本固件**（那是 ROM bootloader，不是 USB 问题）。

### 5.2 打命令没反应

- 先 `!stat`：`link=UP` 还是 `DOWN`？
- `!ping` 超时 → `!probe` 看哪个 UART 通；
- `!raw on` 后看 G431 有没有回字节（有字节但乱码 → 波特率/接线问题）。

### 5.3 提示 `[bridge] no G431 link ... dropped`

说明开机时 G431 没应答，桥已把 `g431` 置空。确认 G431 已上电、PA2/PA3 接对，然后 `!probe`。链路恢复后桥也会每 5 秒自动重试（`link recovered` 日志）。

### 5.4 编译报错 `setTxTimeoutMs was not declared`

老版本 ESP32 core 没有这个方法：把文件顶部 `#define BRIDGE_SET_USB_TIMEOUT 1` 改成 `0`。

### 5.5 板载 USB 桥芯片占着 UART0

部分 ESP32 板子用 UART0 接板载 USB 转串口芯片。这种情况下把文件顶部 `G431_UART_NUM` 改成 `1`（或 `2`），并把 `G431_TX_PIN` / `G431_RX_PIN` 改成你实际接线的两个 GPIO。

### 5.6 串口日志里出现 E (115159) uart: uart_read_bytes(...): uart driver error

这是 ESP-IDF 的驱动提示：在 UART 驱动被 end() 之后又有一次读操作落到同一 UART 上。早期的桥接了
link_watchdog 里会 end() 后再 egin()，中间的读就会打出这条错误。现在已改为**驱动只安装、从不
end()**，这条日志不会再出现。看到历史日志可以直接忽略，不影响功能。
