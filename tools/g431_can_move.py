"""Continuous-motion + thermal soak test for the G431 gimbal over CAN.

Holds one CAN bus open, enables the motor, sets the voltage/velocity limits, then
swings the target back and forth while polling `?` so every sample carries the
phase-current peak, the limiter's allowed voltage and the I2t thermal integrator.
Writes a CSV next to the script's --out argument (or stdout only).

Usage:
  python g431_can_move.py --volts 8 --amp 0.5 --cycles 45 --period 2.0
  python g431_can_move.py --volts 4 --amp 0.3 --cycles 20

Aborts immediately (and disables the motor) if the ADC protection reports a trip
or if the peak current passes --abort-amps.
"""
import argparse
import csv
import re
import struct
import time
from pathlib import Path

try:  # pyusb cannot always locate libusb-1.0.dll on Windows.
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

CMD_ID = 0x201
REPLY_ID = 0x281

NUM = r"(-?\d+\.?\d*)"
FIELDS = {
    "angle": re.compile(r"angle=" + NUM),
    "vel": re.compile(r"vel=" + NUM),
    "tgt": re.compile(r"tgt=" + NUM),
    "vlim": re.compile(r"Vlim=" + NUM),
    "enabled": re.compile(r"EN=(ON|OFF)"),
    "pk": re.compile(r"pk=" + NUM + r"A"),
    "allowed": re.compile(r"allowed=" + NUM + r"V"),
    "req": re.compile(r"req=" + NUM + r"V"),
    "thermal": re.compile(r"th=" + NUM),
    "samples": re.compile(r"smp=(\d+)"),
    "trips": re.compile(r"trips=(\d+)"),
    "ok": re.compile(r"ok=(\d)"),
    "fault": re.compile(r"nFAULT=(\d)"),
}


def open_bus(bitrate=500000, channel=0):
    return can.Bus(interface="gs_usb", channel=channel, index=channel, bitrate=bitrate)


def send(bus, text):
    op = text[0]
    if op in ("T", "V", "L"):
        payload = bytes([ord(op)]) + struct.pack("<f", float(text[1:]))
    else:
        payload = bytes([ord(op)])
    bus.send(can.Message(arbitration_id=CMD_ID, data=payload, is_extended_id=False))


def collect(bus, seconds=0.5):
    deadline = time.monotonic() + seconds
    chunks = []
    while True:
        left = deadline - time.monotonic()
        if left <= 0:
            break
        msg = bus.recv(timeout=left)
        if msg is None:
            break
        if msg.arbitration_id == REPLY_ID:
            chunks.append(bytes(msg.data))
            # A status reply ends with the current-limiter line.
            if b"trips=" in b"".join(chunks[-3:]):
                break
    return b"".join(chunks).decode("ascii", "replace")


def parse(text):
    out = {}
    for name, rx in FIELDS.items():
        m = rx.search(text)
        if not m:
            continue
        if name == "enabled":
            out[name] = m.group(1) == "ON"
        elif name in ("samples", "trips", "ok", "fault"):
            out[name] = int(m.group(1))
        else:
            out[name] = float(m.group(1))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--volts", type=float, default=8.0)
    ap.add_argument("--velocity-limit", type=float, default=20.0)
    ap.add_argument("--amp", type=float, default=0.5, help="swing amplitude, rad")
    ap.add_argument("--cycles", type=int, default=45)
    ap.add_argument("--period", type=float, default=2.0, help="seconds per swing")
    ap.add_argument("--abort-amps", type=float, default=2.2)
    ap.add_argument("--out", default="docs/measurements/2026-10-27-g431-thermal/run.csv")
    args = ap.parse_args()

    bus = open_bus()
    rows = []
    try:
        send(bus, "?")
        first = parse(collect(bus))
        if not first:
            raise SystemExit("no status reply: check 5V, bus termination and wiring")
        print("start:", first)
        base = first.get("angle", 0.0)

        # D clears the stop flags but also makes the board discard whatever is
        # still queued, so drain the FAULT spam it prints before sending more.
        send(bus, "D")
        collect(bus, 1.0)
        send(bus, "V%.3f" % args.volts)
        collect(bus, 0.5)
        send(bus, "L%.3f" % args.velocity_limit)
        collect(bus, 0.5)
        send(bus, "E")
        text = collect(bus, 1.0)
        st = parse(text)
        if "EN ON" in text:
            st["enabled"] = True
        print("enable:", st, repr(text[:120]))
        if not st.get("enabled"):
            raise SystemExit("enable refused: %r" % (text[:200],))

        t0 = time.monotonic()
        aborted = None
        for i in range(args.cycles):
            target = base + (args.amp if i % 2 == 0 else -args.amp)
            send(bus, "T%.4f" % target)
            time.sleep(args.period)
            send(bus, "?")
            st = parse(collect(bus))
            st["t"] = round(time.monotonic() - t0, 3)
            st["idx"] = i
            st["target_cmd"] = target
            rows.append(st)
            print(
                "t=%6.2f i=%2d tgt=%7.3f ang=%7.3f pk=%.3f allowed=%.2f th=%.3f trips=%d"
                % (
                    st["t"], i, target, st.get("angle", float("nan")),
                    st.get("pk", float("nan")), st.get("allowed", float("nan")),
                    st.get("thermal", float("nan")), st.get("trips", -1),
                )
            )
            if st.get("trips", 0) or st.get("pk", 0.0) > args.abort_amps:
                aborted = st
                break
        send(bus, "D")
        final = parse(collect(bus, 0.5))
        print("final:", final)
        print("aborted:", aborted is not None)

        if rows:
            out = Path(args.out)
            out.parent.mkdir(parents=True, exist_ok=True)
            keys = sorted({k for r in rows for k in r})
            with out.open("w", newline="", encoding="utf-8") as fh:
                w = csv.DictWriter(fh, fieldnames=keys)
                w.writeheader()
                w.writerows(rows)
            print("wrote", out)
            peaks = [r.get("pk", 0.0) for r in rows]
            thermal = [r.get("thermal", 0.0) for r in rows]
            print(
                "peak current over run: %.3f A (limit %.2f), max I2t %.4f, "
                "max allowed fold %.2f V" % (
                    max(peaks), args.abort_amps, max(thermal),
                    min(r.get("allowed", 99.0) for r in rows),
                )
            )
    finally:
        try:
            send(bus, "D")
        except Exception:
            pass
        bus.shutdown()


if __name__ == "__main__":
    main()
