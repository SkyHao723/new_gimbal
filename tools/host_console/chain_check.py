"""上位机板整链路自检：USB 串口 -> host -> CAN -> 三块驱动板。

用法：
    python chain_check.py [--port COM15] [--move 0.3] [--vlim 2.5] [--no-move]

做的事：
  1. b   —— 板级状态（时钟 / CAN PSR / IMU）
  2. m   —— IMU 读数
  3. n   —— 广播巡检，看三块驱动板是否都应答
  4. c<N> ? —— 逐板确认 dir / zero / Vlim / EN
  5. 逐板：V<vlim> → E → 读当前角 → T(角+move) → 等 1.6 s → 读回角度，算误差
  6. c0 D  —— 全部失能
所有原始输出都打印出来，方便对照网页里应该看到什么。
"""
import argparse
import re
import sys
import time

import serial


class Link:
    def __init__(self, port, baud=115200):
        self.s = serial.Serial(port, baud, timeout=0.2)
        time.sleep(0.4)
        self.s.reset_input_buffer()

    def send(self, cmd, wait=0.6, quiet=False):
        if not quiet:
            print("\n>>> " + cmd)
        self.s.write((cmd + "\r").encode())
        end = time.time() + wait
        buf = b""
        while time.time() < end:
            chunk = self.s.read(4096)
            if chunk:
                buf += chunk
                end = max(end, time.time() + 0.12)   # 有数据就多等一点
        text = buf.decode("utf-8", "replace")
        # 去掉回显与提示符，便于阅读
        lines = []
        for raw in text.replace("\r\n", "\n").split("\n"):
            line = raw.strip()
            if not line or line == ">":
                continue
            if line == cmd:
                continue
            lines.append(line)
        if not quiet:
            for line in lines:
                print("    " + line)
        return "\n".join(lines)

    def angle_of(self, resp):
        m = re.search(r"\bangle=(-?\d+(?:\.\d+)?)", resp)
        return float(m.group(1)) if m else None

    def close(self):
        self.s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM15")
    ap.add_argument("--move", type=float, default=0.3)
    ap.add_argument("--vlim", type=float, default=2.5)
    ap.add_argument("--no-move", action="store_true")
    args = ap.parse_args()

    lk = Link(args.port)
    try:
        print("========== 1) 板级状态 ==========")
        lk.send("b", 1.0)

        print("\n========== 2) IMU ==========")
        lk.send("m", 0.8)

        print("\n========== 3) 广播巡检三块驱动板 ==========")
        scan = lk.send("n", 2.8)
        found = sorted(set(re.findall(r"\[0x28([1-7])\]", scan)))
        print("\n  -> 应答节点: " + (", ".join(f"0x28{n}" for n in found) if found else "无"))

        print("\n========== 4) 逐板状态 ==========")
        state = {}
        for node in (1, 2, 3):
            resp = lk.send(f"c{node} ?", 0.7)
            state[node] = resp
            for line in resp.split("\n"):
                if "EN=" in line:
                    en = "ON" if "EN=ON" in line else "OFF"
                    z = re.search(r"zero=(-?\d+(?:\.\d+)?)", line)
                    d = re.search(r"\bdir=(-?\d)", line)
                    v = re.search(r"Vlim=(\d+(?:\.\d+)?)", line)
                    print(f"  节点{node}: EN={en} dir={d.group(1) if d else '?'} "
                          f"zero={z.group(1) if z else '?'} Vlim={v.group(1) if v else '?'}")

        if args.no_move:
            print("\n(跳过运动测试)")
            return 0

        print(f"\n========== 5) 运动测试：Vlim={args.vlim} V，每板 +{args.move} rad ==========")
        lk.send(f"c0 V{args.vlim}", 0.6)
        lk.send("c0 E", 0.8)

        for node in (1, 2, 3):
            resp = lk.send(f"c{node} ?", 0.7)
            a0 = lk.angle_of(resp)
            if a0 is None:
                print(f"  节点{node}: 读不到角度，跳过")
                continue
            target = a0 + args.move
            lk.send(f"c{node} T{target:.4f}", 0.5)
            time.sleep(1.2)
            resp2 = lk.send(f"c{node} ?", 0.7)
            a1 = lk.angle_of(resp2)
            if a1 is None:
                print(f"  节点{node}: 运动后读不到角度")
                continue
            err = a1 - target
            print(f"  节点{node}: {a0:+.4f} -> {a1:+.4f} (目标 {target:+.4f}) "
                  f"误差 {err:+.4f} rad = {err * 57.2958:+.2f}°  "
                  f"{'OK' if abs(err) < 0.02 else '偏差大'}")

        print("\n========== 6) 全部失能 ==========")
        lk.send("c0 D", 0.8)
        lk.send("c0 ?", 1.2)
    finally:
        lk.close()
    print("\n完成。串口已释放，可以在浏览器里连 COM 口了。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
