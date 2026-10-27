#!/usr/bin/env python3
"""Quantify the G431 gimbal jitter over CAN, without touching the firmware.

The board only reports state over CAN, so the measurement is host side:
poll the short ``R`` command (sensor angle + velocity) as fast as the bus and
USB adapter allow, then reduce the samples to a few numbers that can be
compared between firmware builds / parameter sets.

Two modes are provided because they stress different things:

``cruise``  command a long move so the trajectory saturates at a constant
            velocity, then look at the angle residual after removing the best
            linear fit.  A smooth motor gives a small residual; cogging,
            velocity quantisation and loop oscillation show up as a bigger
            residual and as peaks in its spectrum.

``step``    command a single position step from standstill and watch the error
            settle.  Reports overshoot, oscillation amplitude in the tail and
            the peak phase current.  This is the "it hunts after a move"
            symptom the operator feels.

Usage::

    python g431_jitter_probe.py cruise --volts 2.5 --vel 1.5 --seconds 6
    python g431_jitter_probe.py step   --volts 2.5 --step 0.3 --seconds 4
    python g431_jitter_probe.py cruise --repeat 3 --tag baseline

Every run prints one ``RESULT`` line plus a compact report; ``--csv`` writes the
raw samples so a run can be re-analysed later.
"""
from __future__ import annotations

import argparse
import math
import os
import statistics
import struct
import sys
import time

import can
import usb.core
import libusb_package

# pyusb on this machine cannot find libusb-1.0.dll on its own; python-can runs
# before we can call set_backend(), so patch the lookup once here.
_orig_find = usb.core.find


def _find_with_backend(*args, **kwargs):
    kwargs.setdefault("backend", libusb_package.get_libusb1_backend())
    return _orig_find(*args, **kwargs)


usb.core.find = _find_with_backend

CMD_ID = 0x201
REPLY_ID = 0x281
SINGLE_BYTE = "ED?RH"
VALUE_OPS = "TVL"

_ANGLE_RE = __import__("re").compile(r"angle=([-+]?[0-9.]+)")
_VEL_RE = __import__("re").compile(r"vel=([-+]?[0-9.]+)")
_PK_RE = __import__("re").compile(r"I pk=([-+]?[0-9.]+)")


def open_bus(channel: int = 0, bitrate: int = 500000) -> can.BusABC:
    return can.Bus(interface="gs_usb", channel=channel, index=channel,
                   bitrate=bitrate)


def encode(text: str) -> bytes:
    body = text.strip()
    op = body[0]
    if op in SINGLE_BYTE:
        return bytes([ord(op)])
    if op in VALUE_OPS:
        return bytes([ord(op)]) + struct.pack("<f", float(body[1:]))
    raise ValueError("unsupported command %r" % body)


def send(bus: can.BusABC, text: str) -> None:
    bus.send(can.Message(arbitration_id=CMD_ID, is_extended_id=False,
                         data=encode(text)))


def _drain(bus: can.BusABC) -> None:
    while bus.recv(timeout=0.0) is not None:
        pass


def ask(bus: can.BusABC, text: str, timeout: float = 0.05) -> str:
    """Send one command and return the concatenated ASCII reply."""
    _drain(bus)
    send(bus, text)
    out = ""
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        msg = bus.recv(timeout=0.004)
        if msg is None:
            if "\n" in out:
                break
            continue
        if msg.arbitration_id != REPLY_ID:
            continue
        out += msg.data.decode("ascii", "replace")
        if "\n" in out:
            break
    return out


def sample_sensor(bus: can.BusABC) -> tuple[float, float, float]:
    """One fast poll; returns (t, angle, velocity)."""
    t = time.perf_counter()
    text = ask(bus, "R", timeout=0.03)
    m = _ANGLE_RE.search(text)
    v = _VEL_RE.search(text)
    if not m:
        return t, float("nan"), float("nan")
    ang = float(m.group(1))
    vel = float(v.group(1)) if v else float("nan")
    return t, ang, vel


def unwrap(angles: list[float]) -> list[float]:
    """Remove the 2*pi wraps the sensor has when the motor turns."""
    out = [angles[0]]
    for a in angles[1:]:
        d = a - out[-1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        out.append(out[-1] + d)
    return out


def linear_fit_residual(times: list[float], values: list[float]):
    """Least squares slope/intercept and the residual list."""
    n = len(values)
    mt = sum(times) / n
    mv = sum(values) / n
    num = sum((t - mt) * (v - mv) for t, v in zip(times, values))
    den = sum((t - mt) ** 2 for t in times)
    slope = num / den if den > 0 else 0.0
    intercept = mv - slope * mt
    resid = [v - (intercept + slope * t) for t, v in zip(times, values)]
    return slope, intercept, resid


def spectrum_peak(times: list[float], values: list[float], top: int = 3):
    """Crude DFT peak finder (the scan is short and unevenly sampled)."""
    n = len(values)
    if n < 16 or not times:
        return []
    span = times[-1] - times[0]
    if span <= 0:
        return []
    # Resample onto a uniform grid so a plain DFT is valid.  The grid is kept
    # small on purpose: this runs on every probe, and the scan is only a few
    # dozen Hz wide, so a coarse grid is plenty.
    grid = 1024
    dt = span / (grid - 1)
    uniform = []
    j = 0
    for i in range(grid):
        target = times[0] + i * dt
        while j + 1 < n and times[j + 1] < target:
            j += 1
        uniform.append(values[j])
    mean = sum(uniform) / grid
    uniform = [u - mean for u in uniform]
    best = []
    for k in range(2, grid // 2, 2):
        re = im = 0.0
        w = 2.0 * math.pi * k / grid
        for i, u in enumerate(uniform):
            re += u * math.cos(w * i)
            im -= u * math.sin(w * i)
        best.append((math.hypot(re, im) / grid, k / span))
    best.sort(reverse=True)
    return [(round(f, 2), round(a, 5)) for a, f in best[:top]]


def summarise(name: str, times: list[float], angles: list[float],
              vels: list[float], csv_path: str | None = None) -> dict:
    if len(times) < 4:
        print("RESULT %s: not enough samples (%d)" % (name, len(times)))
        return {}
    t0 = times[0]
    times = [t - t0 for t in times]
    angles = unwrap(angles)
    slope, _, resid = linear_fit_residual(times, angles)
    resid_mrad = [r * 1000.0 for r in resid]
    scan_hz = (len(times) - 1) / times[-1] if times[-1] > 0 else 0.0
    good_vel = [v for v in vels if not math.isnan(v)]

    result = {
        "name": name,
        "n": len(times),
        "scan_hz": scan_hz,
        "speed": slope,
        "angle_jitter_std_mrad": statistics.pstdev(resid_mrad),
        "angle_jitter_pp_mrad": max(resid_mrad) - min(resid_mrad),
        "vel_std": statistics.pstdev(good_vel) if len(good_vel) > 1 else float("nan"),
        "vel_pp": (max(good_vel) - min(good_vel)) if good_vel else float("nan"),
        "peaks": spectrum_peak(times, resid),
    }
    print("RESULT %s n=%d scan=%.1fHz speed=%+.4f rad/s "
          "jitter_std=%.2f mrad jitter_pp=%.2f mrad vel_std=%.4f vel_pp=%.4f"
          % (name, result["n"], scan_hz, slope, result["angle_jitter_std_mrad"],
             result["angle_jitter_pp_mrad"], result["vel_std"], result["vel_pp"]))
    if result["peaks"]:
        print("        residual peaks (Hz, amp): %s"
              % ", ".join("%.2f/%.5f" % p for p in result["peaks"]))
    if csv_path:
        with open(csv_path, "w", encoding="utf-8") as fh:
            fh.write("t,angle,velocity\n")
            for t, a, v in zip(times, angles, vels):
                fh.write("%.6f,%.6f,%.6f\n" % (t, a, v))
        print("        csv -> %s" % csv_path)
    return result


def ensure_enabled(bus: can.BusABC, volts: float) -> float:
    send(bus, "V%.3f" % volts)
    time.sleep(0.2)
    send(bus, "E")
    time.sleep(0.9)
    text = ask(bus, "?")
    m = _ANGLE_RE.search(text)
    return float(m.group(1)) if m else float("nan")


def run_cruise(bus: can.BusABC, args) -> dict:
    start = ensure_enabled(bus, args.volts)
    if math.isnan(start):
        print("RESULT cruise: no reply after enable")
        return {}
    send(bus, "L%.3f" % args.vel)
    time.sleep(0.2)
    send(bus, "T%.4f" % (start + 200.0))  # far target -> cruise at velocity limit
    times, angles, vels = [], [], []
    deadline = time.perf_counter() + args.seconds + args.warmup
    warm = time.perf_counter() + args.warmup
    while time.perf_counter() < deadline:
        t, a, v = sample_sensor(bus)
        if math.isnan(a):
            continue
        if t >= warm:
            times.append(t)
            angles.append(a)
            vels.append(v)
    send(bus, "D")
    return summarise("cruise(vlim=%.2f,vel=%.2f)" % (args.volts, args.vel),
                     times, angles, vels, args.csv)


def run_step(bus: can.BusABC, args) -> dict:
    start = ensure_enabled(bus, args.volts)
    if math.isnan(start):
        print("RESULT step: no reply after enable")
        return {}
    target = start + args.step
    times, angles, vels = [], [], []
    send(bus, "T%.4f" % target)
    deadline = time.perf_counter() + args.seconds
    while time.perf_counter() < deadline:
        t, a, v = sample_sensor(bus)
        if math.isnan(a):
            continue
        times.append(t)
        angles.append(a)
        vels.append(v)
    pk_text = ask(bus, "?")
    pk = _PK_RE.search(pk_text)
    peak_current = float(pk.group(1)) if pk else float("nan")
    send(bus, "D")
    result = summarise("step(%.2f rad)" % args.step, times, angles, vels, args.csv)
    if result:
        angles_u = unwrap(angles)
        err = [target - a for a in angles_u]
        tail_n = max(3, len(err) // 4)
        tail = err[-tail_n:]
        result["peak_current"] = peak_current
        result["tail_err_max"] = max(abs(e) for e in tail)
        result["tail_err_span"] = max(tail) - min(tail)
        settle = None
        for i in range(len(err)):
            if all(abs(e) < args.settle_band for e in err[i:]):
                settle = times[i] - times[0]
                break
        result["settle_s"] = settle
        print("        step: peak_current=%.3fA tail_err_max=%.4f rad "
              "tail_span=%.4f rad settle(<%.3f)=%s s"
              % (peak_current, result["tail_err_max"], result["tail_err_span"],
                 args.settle_band,
                 "%.2f" % settle if settle is not None else "never"))
    return result


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("cruise", "step"))
    ap.add_argument("--volts", type=float, default=2.5,
                    help="voltage limit request (default 2.5)")
    ap.add_argument("--vel", type=float, default=1.5,
                    help="cruise velocity limit rad/s (default 1.5)")
    ap.add_argument("--step", type=float, default=0.3,
                    help="step size rad (default 0.3)")
    ap.add_argument("--seconds", type=float, default=6.0,
                    help="recorded window (default 6 s, step default 4 s)")
    ap.add_argument("--warmup", type=float, default=0.6,
                    help="cruise: skip the acceleration ramp (default 0.6 s)")
    ap.add_argument("--settle-band", type=float, default=0.01,
                    help="step: |err| band that counts as settled (default 0.01 rad)")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--tag", default="")
    ap.add_argument("--csv", default=None)
    args = ap.parse_args(argv)
    if args.mode == "step" and args.seconds == 6.0:
        args.seconds = 4.0

    bus = open_bus()
    try:
        for i in range(args.repeat):
            if args.tag:
                print("--- run %d/%d  tag=%s ---" % (i + 1, args.repeat, args.tag))
            csv = None
            if args.csv:
                root, ext = os.path.splitext(args.csv)
                csv = "%s_%d%s" % (root, i + 1, ext) if args.repeat > 1 else args.csv
            sub = argparse.Namespace(**vars(args))
            sub.csv = csv
            if args.mode == "cruise":
                run_cruise(bus, sub)
            else:
                run_step(bus, sub)
            if i + 1 < args.repeat:
                time.sleep(1.0)
    finally:
        try:
            send(bus, "D")
        except Exception:
            pass
        time.sleep(0.1)
        bus.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
