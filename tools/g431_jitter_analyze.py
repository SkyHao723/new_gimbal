#!/usr/bin/env python3
"""Parse a bench_capture RAM dump made with STM32_Programmer_CLI -r32 and report jitter.

    python tools/g431_jitter_analyze.py --capture capture.txt --meta meta.txt --seconds 1.5

The dump lines look like
    0x200003CC : 00000000 4415C000 40200000 BFDF8184
16 bytes per line, little endian words. bench_capture[] holds (angle, velocity, uq)
triples in sample order; bench_capture_meta[8] holds the run metadata.
"""

from __future__ import annotations

import argparse
import re
import struct
from pathlib import Path

import numpy as np

WORD_RE = re.compile(r"^\s*0x[0-9A-Fa-f]+\s*:\s*((?:[0-9A-Fa-f]{8}\s*)+)$")


def read_text(path: Path) -> str:
    """PowerShell's `>` writes UTF-16, python writes UTF-8; accept both."""
    raw = path.read_bytes()
    if raw[:2] in (b"\xff\xfe", b"\xfe\xff"):
        return raw.decode("utf-16")
    if raw[:3] == b"\xef\xbb\xbf":
        return raw[3:].decode("utf-8", "replace")
    return raw.decode("utf-8", "replace")


def parse_words(path: Path) -> list[int]:
    words: list[int] = []
    for line in read_text(path).splitlines():
        match = WORD_RE.match(line)
        if not match:
            continue
        for token in match.group(1).split():
            words.append(int(token, 16))
    return words


def as_floats(words: list[int]) -> np.ndarray:
    return np.array(struct.unpack(f"<{len(words)}f", struct.pack(f"<{len(words)}I", *words)),
                    dtype=np.float64)


def peaks(signal: np.ndarray, rate: float, count: int = 5) -> list[tuple[float, float]]:
    centered = signal - signal.mean()
    window = np.hanning(len(centered))
    spectrum = np.abs(np.fft.rfft(centered * window))
    freqs = np.fft.rfftfreq(len(centered), 1.0 / rate)
    if len(spectrum) < 3:
        return []
    spectrum[0] = 0.0
    order = np.argsort(spectrum)[::-1]
    picked: list[tuple[float, float]] = []
    for index in order:
        freq = freqs[index]
        if any(abs(freq - f) < 2.0 for f, _ in picked):
            continue
        picked.append((float(freq), float(spectrum[index])))
        if len(picked) >= count:
            break
    return picked


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", required=True)
    parser.add_argument("--meta", required=True)
    parser.add_argument("--seconds", type=float, required=True,
                        help="capture_ms/1000 used when the run finished on time")
    parser.add_argument("--skip", type=float, default=0.35,
                        help="seconds to drop from the start (acceleration phase)")
    args = parser.parse_args()

    meta = as_floats(parse_words(Path(args.meta)))
    print(f"meta: code={meta[0]:.0f} n={meta[1]:.0f} vlim={meta[2]:.2f}V "
          f"target={meta[3]:.4f} start={meta[4]:.4f} end={meta[5]:.4f} "
          f"worst={meta[6]:.3f}A trips={meta[7]:.0f}"
          + (f" duration={meta[8]:.4f}s" if len(meta) > 8 else ""))
    n = int(meta[1])
    if n < 8:
        print("not enough samples")
        return 1
    seconds = float(meta[8]) if len(meta) > 8 and meta[8] > 0 else args.seconds
    rate = n / seconds
    print(f"samples={n} over {seconds:.4f}s -> loop rate {rate:.1f} Hz")

    data = as_floats(parse_words(Path(args.capture)))[: n * 3].reshape(n, 3)
    angle, velocity, uq = data[:, 0], data[:, 1], data[:, 2]
    skip = int(args.skip * rate)
    if skip >= n - 8:
        skip = 0
    angle, velocity, uq = angle[skip:], velocity[skip:], uq[skip:]
    t = np.arange(len(angle)) / rate

    # position ripple: remove the constant-velocity trend
    slope, intercept = np.polyfit(t, angle, 1)
    residual = angle - (slope * t + intercept)
    print(f"\nconstant-velocity section: {len(angle)} samples, "
          f"mean speed {slope:+.3f} rad/s")
    print(f"angle ripple : std={residual.std() * 1e3:.3f} mrad "
          f"({residual.std() * 180 / np.pi * 1e3:.2f} mdeg) "
          f"pp={np.ptp(residual) * 180 / np.pi * 1e3:.2f} mdeg")
    print(f"velocity     : mean={velocity.mean():+.3f} std={velocity.std():.3f} "
          f"pp={np.ptp(velocity):.3f} rad/s")
    print(f"uq           : mean={uq.mean():+.4f} std={uq.std():.4f} "
          f"pp={np.ptp(uq):.4f} V")

    print("\ntop spectral peaks (Hz, amplitude):")
    for name, signal in (("angle", residual), ("velocity", velocity), ("uq", uq)):
        line = ", ".join(f"{freq:.1f}Hz" for freq, _ in peaks(signal, rate))
        print(f"  {name:9s}: {line}")

    # copy of the velocity sequence so it can be plotted later
    out = Path(args.capture).with_suffix(".csv")
    np.savetxt(out, np.column_stack([t, angle, velocity, uq]), delimiter=",",
               header="t,angle,velocity,uq", comments="")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
