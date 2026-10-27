"""Step-response capture for the G431 gimbal over CAN.

Commands one position step, then polls `?` as fast as the CAN link allows and
records angle vs time, so the effect of the voltage ceiling on rise time and
overshoot can be compared (e.g. 2.5 V vs 8 V).

Usage:
  python g431_can_step.py --volts 2.5 --step 1.0
  python g431_can_step.py --volts 8   --step 1.0
"""
import argparse
import re
import time

try:
    import libusb_package
    import usb.core

    _backend = libusb_package.get_libusb1_backend()
    if _backend is not None:
        _orig_find = usb.core.find

        def _find_with_backend(*args, **kwargs):
            kwargs.setdefault("backend", _backend)
            return _orig_find(*args, **kwargs)

        usb.core.find = _find_with_backend
except Exception:
    pass

import can
import struct

CMD_ID = 0x201
REPLY_ID = 0x281
ANGLE = re.compile(r"angle=(-?\d+\.?\d*)")
PK = re.compile(r"pk=(-?\d+\.?\d*)A")


def send(bus, text):
    op = text[0]
    if op in ("T", "V", "L"):
        payload = bytes([ord(op)]) + struct.pack("<f", float(text[1:]))
    else:
        payload = bytes([ord(op)])
    bus.send(can.Message(arbitration_id=CMD_ID, data=payload, is_extended_id=False))


def poll(bus, wait=0.12):
    """Ask for status and return (angle, peak_current) or None."""
    send(bus, "?")
    deadline = time.monotonic() + wait
    chunks = []
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None:
            break
        if msg.arbitration_id == REPLY_ID:
            chunks.append(bytes(msg.data))
            if b"trips=" in b"".join(chunks[-3:]):
                break
    text = b"".join(chunks).decode("ascii", "replace")
    a = ANGLE.search(text)
    if not a:
        return None
    p = PK.search(text)
    return float(a.group(1)), (float(p.group(1)) if p else float("nan"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--volts", type=float, default=8.0)
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--hold", type=float, default=2.0)
    args = ap.parse_args()

    bus = can.Bus(interface="gs_usb", channel=0, index=0, bitrate=500000)
    try:
        send(bus, "D")
        time.sleep(1.0)
        while bus.recv(timeout=0.05):
            pass
        send(bus, "V%.3f" % args.volts)
        time.sleep(0.4)
        send(bus, "E")
        time.sleep(0.8)
        while bus.recv(timeout=0.05):
            pass
        base = poll(bus)
        if base is None:
            raise SystemExit("no status reply")
        start = base[0]
        target = start + args.step
        print("start angle=%.4f target=%.4f ceiling=%.2f V" % (start, target, args.volts))

        send(bus, "T%.4f" % target)
        t0 = time.monotonic()
        samples = []
        while True:
            dt = time.monotonic() - t0
            if dt > args.hold:
                break
            got = poll(bus)
            if got is not None:
                samples.append((dt, got[0] - target, got[1]))
        send(bus, "D")
        peak_over = max(s[1] for s in samples)
        peak_cur = max(s[2] for s in samples if s[2] == s[2])
        settle = None
        for dt, err, _ in reversed(samples):
            if abs(err) > 0.01:
                settle = dt
                break
        print("samples=%d  max overshoot=%+.4f rad  peak current=%.3f A" % (
            len(samples), peak_over, peak_cur))
        print("settled (|err|<0.01 rad) by %.2f s" % (settle if settle else 0.0))
        for dt, err, pk in samples:
            print("  t=%5.2f err=%+8.4f pk=%.3f" % (dt, err, pk))
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
