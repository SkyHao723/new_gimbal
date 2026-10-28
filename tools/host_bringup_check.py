#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上位机板（G431 CAN 主机）点板结果自动核对

用法：
    python tools/host_bringup_check.py                    # 自动找 CH343 串口，读 8 秒
    python tools/host_bringup_check.py --port COM12
    python tools/host_bringup_check.py --seconds 12 --cmd n   # 先发 n（驱动板巡检）再读

它只做一件事：把串口上电日志按"点板清单"逐项判 PASS/FAIL/SKIP，省得人工一行行对。
不改固件、不占 CAN 适配器（只开串口）。
"""

import argparse
import re
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("需要 pyserial：python -m pip install pyserial")

# CH343/CH340 常见 VID；上位机板用的是 CH343P
CANDIDATE_VIDS = {0x1A86}

BANNER = "G431 host board bring-up"


def pick_port(explicit):
    if explicit:
        return explicit
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid in CANDIDATE_VIDS:
            return p.device
    for p in ports:
        if "CH34" in (p.description or "") or "USB-SERIAL" in (p.description or "").upper():
            return p.device
    return None


def collect(port, baud, seconds, cmd):
    text = []
    with serial.Serial(port, baud, timeout=0.2) as ser:
        ser.reset_input_buffer()
        if cmd:
            ser.write((cmd + "\r\n").encode())
            ser.flush()
        deadline = time.time() + seconds
        while time.time() < deadline:
            chunk = ser.read(256)
            if chunk:
                piece = chunk.decode("utf-8", "replace")
                sys.stdout.write(piece)
                sys.stdout.flush()
                text.append(piece)
        if cmd:
            ser.write(b"b\r\n")
            ser.flush()
            time.sleep(0.4)
            tail = ser.read(4096).decode("utf-8", "replace")
            sys.stdout.write(tail)
            text.append(tail)
    return "".join(text)


def judge(log):
    checks = []

    def add(name, ok, detail=""):
        checks.append((name, "PASS" if ok else "FAIL", detail))

    add("串口有输出 / 固件在跑", BANNER in log, "")

    m = re.search(r"SYSCLK=(\d+)", log)
    if m:
        hz = int(m.group(1))
        add("时钟 = 170 MHz", hz == 170000000, f"实测 {hz} Hz")
    else:
        add("时钟 = 170 MHz", False, "没打印 SYSCLK")

    m = re.search(r"UART1 PA9/PA10 .*?@ (\d+)", log)
    add("控制台串口 115200", bool(m) and m.group(1) == "115200",
        f"{m.group(1)}" if m else "没打印串口行")

    if "[I2C] no device found" in log:
        add("I2C 扫描到器件", False, "总线空：查上拉/IMU 供电/硬件 I2C 时序")
    else:
        m = re.search(r"\[I2C\] (\d+) address\(es\):(.*)", log)
        if m:
            add("I2C 扫描到器件", int(m.group(1)) >= 1, m.group(2).strip())
        else:
            add("I2C 扫描到器件", False, "没打印扫描结果")

    m = re.search(r"WHO_AM_I=0x([0-9A-Fa-f]{2})", log)
    if m:
        add("IMU WHO_AM_I = 0x67", m.group(1).upper() == "67", f"0x{m.group(1).upper()}")
    else:
        add("IMU WHO_AM_I = 0x67", False, "IMU 没初始化成功")

    m = re.search(r"IMU acc\[g\] ([\-0-9.]+) ([\-0-9.]+) ([\-0-9.]+)", log)
    if m:
        vals = [abs(float(x)) for x in m.groups()]
        # 平放时应该有一个轴接近 1 g
        add("IMU 加速度读数合理", max(vals) > 0.5, "轴幅值 " + " ".join(f"{v:.2f}" for v in vals))
    else:
        add("IMU 加速度读数合理", False, "没读到 IMU 数据")

    add("CAN 初始化成功", "[CAN] init FDCAN1 500k ... OK" in log, "")

    replies = re.findall(r"\[0x28([1-7])\]", log)
    nodes = sorted({int(r) for r in replies})
    if "[CAN] broadcast '?'" not in log:
        add("驱动板回包", False, "没做广播巡检（固件没跑到 CAN 或上电日志被截断）")
    else:
        add("驱动板回包", len(nodes) > 0,
            ("节点 " + ",".join(str(n) for n in nodes)) if nodes else "无回包：查 CANH/CANL/120Ω/驱动板供电")

    m = re.search(r"CAN PSR=0x([0-9A-Fa-f]+)", log)
    if m:
        psr = int(m.group(1), 16)
        lec = psr & 0x7
        add("CAN 总线状态", lec != 7, f"PSR=0x{m.group(1)} LEC={lec}" + ("（bus-off：总线没人 ACK）" if lec == 7 else ""))

    print("\n==== 点板结果 ====")
    width = max(len(c[0]) for c in checks) + 2
    for name, verdict, detail in checks:
        print(f"{name:<{width}} {verdict:<5} {detail}")
    failed = [c for c in checks if c[1] == "FAIL"]
    print(f"\n{len(checks) - len(failed)}/{len(checks)} 通过" + ("" if failed else "，点板完成 ✅"))
    return 0 if not failed else 1


def main():
    ap = argparse.ArgumentParser(description="上位机板点板结果核对")
    ap.add_argument("--port", help="串口号，如 COM12；缺省自动找 CH343")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=8.0, help="采集时长（上电日志约 3~4 秒）")
    ap.add_argument("--cmd", help="采集前先发一条命令，如 n（驱动板巡检）")
    args = ap.parse_args()

    port = pick_port(args.port)
    if not port:
        sys.exit("没找到串口，用 --port COMx 指定")
    print(f"# 打开 {port} @ {args.baud}，采集 {args.seconds} 秒\n")
    log = collect(port, args.baud, args.seconds, args.cmd)
    return judge(log)


if __name__ == "__main__":
    sys.exit(main())
