#!/usr/bin/env python3
"""Measure G431 hold/move jitter over CAN and dump a CSV.

Usage:
    python tools/g431_can_jitter.py            # hold 3 s + move 1.5 rad
    python tools/g431_can_jitter.py --volts 2.5 --amp 1.5 --out run.csv
"""

from __future__ import annotations

import argparse
import csv
import re
import statistics
import struct
import sys
import time

try:
    import can
except ImportError:
    sys.exit("python-can is missing: pip install python-can")

try:
    import libusb_package
    import usb.core

    _orig_find = usb.core.find

    def _find_with_backend(*args, **kwargs):
        kwargs.setdefault("backend", libusb_package.get_libusb1_backend())
        return _orig_find(*args, **kwargs)

    usb.core.find = _find_with_backend
except ImportError:
    pass

CMD_ID = 0x201
REPLY_ID = 0x281

STATUS_RE = re.compile(
    r"EN=(ON|OFF)\s+angle=(-?\d+\.\d+)\s+vel=(-?\d+\.\d+)\s+tgt=(-?\d+\.\d+)"
)


def send(bus: can.BusABC, text: str) -> None:
    op = text[0]
    if op in "TVL":
        data = bytes([ord(op)]) + struct.pack("<f", float(text[1:]))
    else:
        data = bytes([ord(op)])
    bus.send(can.Message(arbitration_id=CMD_ID, data=data, is_extended_id=False))


def query(bus: can.BusABC, timeout: float = 0.5) -> dict | None:
    """Send '?' and parse the first status line that comes back."""
    send(bus, "?")
    deadline = time.monotonic() + timeout
    chunks: list[str] = []
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)
        if msg is None or msg.arbitration_id != REPLY_ID:
            continue
        chunks.append(msg.data.decode("ascii", "replace"))
        match = STATUS_RE.search("".join(chunks))
        if match:
            return {
                "en": match.group(1),
                "angle": float(match.group(2)),
                "vel": float(match.group(3)),
                "tgt": float(match.group(4)),
            }
    return None


def sample(bus: can.BusABC, seconds: float) -> list[tuple[float, float, float]]:
    rows: list[tuple[float, float, float]] = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        state = query(bus)
        if state is not None:
            rows.append((time.monotonic(), state["angle"], state["vel"]))
    return rows


def stats(rows: list[tuple[float, float, float]], label: str) -> dict:
    angles = [r[1] for r in rows]
    vels = [r[2] for r in rows]
    if len(angles) < 4:
        print(f"{label}: too few samples ({len(angles)})")
        return {}
    dvel = [abs(vels[i + 1] - vels[i]) for i in range(len(vels) - 1)]
    out = {
        "label": label,
        "n": len(angles),
        "angle_std_mdeg": statistics.pstdev(angles) * 1e3 * 180 / 3.141592653589793,
        "angle_pp_mdeg": (max(angles) - min(angles)) * 1e3 * 180 / 3.141592653589793,
        "vel_std": statistics.pstdev(vels),
        "vel_pp": max(vels) - min(vels),
        "vel_step_p95": sorted(dvel)[int(len(dvel) * 0.95)],
    }
    print(f"{label}: n={out['n']}  angle std={out['angle_std_mdeg']:.2f} m°  "
          f"pp={out['angle_pp_mdeg']:.2f} m°  vel std={out['vel_std']:.3f} rad/s  "
          f"vel pp={out['vel_pp']:.3f}  d(vel) p95={out['vel_step_p95']:.3f}")
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--volts", type=float, default=2.5)
    parser.add_argument("--amp", type=float, default=1.5, help="move amplitude in rad")
    parser.add_argument("--hold", type=float, default=3.0, help="hold capture seconds")
    parser.add_argument("--move", type=float, default=2.0, help="move capture seconds")
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    bus = can.Bus(interface="gs_usb", channel=args.channel,
                  index=args.channel, bitrate=500000)
    rows: list[tuple[str, float, float]] = []
    try:
        for attempt in range(3):
            send(bus, "D")
            time.sleep(0.4)
            state = query(bus)
            if state is not None:
                break
        else:
            print("no reply from the board")
            return 1
        start = state["angle"]
        print(f"start angle={start:.4f}")

        send(bus, f"V{args.volts}")
        time.sleep(0.15)
        send(bus, "E")
        time.sleep(0.6)
        hold = sample(bus, args.hold)
        hold_stats = stats(hold, "hold ")
        rows += [("hold", r[1], r[2]) for r in hold]

        target = start + args.amp
        send(bus, f"T{target:.5f}")
        move = sample(bus, args.move)
        move_stats = stats(move, "move ")
        rows += [("move", r[1], r[2]) for r in move]

        end = query(bus)
        if end is not None:
            print(f"end   angle={end['angle']:.4f}  (target {target:.4f}, "
                  f"err={end['angle'] - target:+.4f})")

        send(bus, "D")
        time.sleep(0.2)

        if args.out:
            with open(args.out, "w", newline="", encoding="utf-8") as fh:
                writer = csv.writer(fh)
                writer.writerow(["phase", "angle", "velocity"])
                writer.writerows(rows)
            print(f"wrote {args.out} ({len(rows)} rows)")
        return 0
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
