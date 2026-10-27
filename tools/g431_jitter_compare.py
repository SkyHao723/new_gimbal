"""Compare every capture_*.txt produced by jtune.ps1 on the band that matters.

The operator feels the 3-50 Hz component of the position error, not the bulk
lag, so each variant is scored on the band-limited residual instead of the raw
trend-removed std.

    python tools/g431_jitter_compare.py --dir D:\\dsh_build
"""

from __future__ import annotations

import argparse
import glob
import re
import struct
from pathlib import Path

import numpy as np

WORD_RE = re.compile(r"^\s*0x[0-9A-Fa-f]+\s*:\s*((?:[0-9A-Fa-f]{8}\s*)+)$")


def read_text(path: Path) -> str:
    raw = path.read_bytes()
    if raw[:2] in (b"\xff\xfe", b"\xfe\xff"):
        return raw.decode("utf-16")
    return raw[3:].decode("utf-8", "replace") if raw[:3] == b"\xef\xbb\xbf" else raw.decode("utf-8", "replace")


def floats(path: Path) -> np.ndarray:
    words = []
    for line in read_text(path).splitlines():
        found = WORD_RE.match(line)
        if found:
            words.extend(int(token, 16) for token in found.group(1).split())
    if not words:
        return np.zeros(0)
    return np.array(struct.unpack(f"<{len(words)}f", struct.pack(f"<{len(words)}I", *words)))


def band(signal: np.ndarray, rate: float, low: float, high: float) -> np.ndarray:
    centered = signal - signal.mean()
    spectrum = np.fft.rfft(centered)
    freqs = np.fft.rfftfreq(len(centered), 1.0 / rate)
    spectrum[(freqs < low) | (freqs > high)] = 0.0
    return np.fft.irfft(spectrum, n=len(centered))


def score(tag: str, capture: Path, meta: Path) -> str:
    m = floats(meta)
    n = int(m[1])
    seconds = float(m[8]) if len(m) > 8 else 0.0
    if n < 16 or seconds <= 0:
        return f"{tag:16s} no data"
    data = floats(capture)[: n * 3].reshape(n, 3)
    rate = n / seconds
    t = np.arange(n) / rate
    angle, velocity, uq = data[:, 0], data[:, 1], data[:, 2]
    skip = min(0.6, 0.25 * t[-1])
    keep = t >= skip
    t, angle, velocity, uq = t[keep], angle[keep], velocity[keep], uq[keep]
    slope, intercept = np.polyfit(t, angle, 1)
    residual = angle - (slope * t + intercept)
    pitch = band(residual, rate, 3.0, 50.0)
    vel_band = band(velocity, rate, 3.0, 50.0)
    uq_band = band(uq, rate, 3.0, 50.0)
    return (f"{tag:16s} speed={slope:.3f} ripple_std={residual.std() * 1e3:6.2f}mrad "
            f"jit3-50={pitch.std() * 1e3:6.2f}mrad({np.ptp(pitch) * 1e3:6.2f}pp) "
            f"veljit={vel_band.std():.3f} uqjit={uq_band.std():.4f}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default=r"D:\dsh_build")
    args = parser.parse_args()
    folder = Path(args.dir)
    rows = []
    for capture in sorted(folder.glob("capture_*.txt")):
        tag = capture.stem.replace("capture_", "")
        meta = folder / f"meta_{tag}.txt"
        if not meta.exists():
            continue
        rows.append((tag, score(tag, capture, meta)))
    for _, line in rows:
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
