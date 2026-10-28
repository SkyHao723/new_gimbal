"""Fit pole pairs, sensor direction and electrical zero from an align-test dump.

The board's align self-test (``G431_ALIGN_TEST = 1`` in ``User/motor_config.h``)
pulls the rotor onto a d-axis at a commanded electrical angle theta and records
the mechanical angle it settled at.  The firmware does two pulls per row, one
swinging the field up from ``theta - BENCH_ALIGN_SWING`` and one swinging it down
from ``theta + BENCH_ALIGN_SWING``: friction makes the rotor stop short of the
target by a hysteresis angle whose sign follows the approach direction, so the
two measurements straddle the true angle and their mean is nearly bias free.

A single pull is only good for a few tens of electrical degrees - that is why
board 2 first looked "unable to move" (the wrong direction put the vector on the
rotor flux axis).

Usage
-----
    # read 4 rows of bench_align_result[] over SWD (128 bytes = 4 x 8 floats)
    STM32_Programmer_CLI -c port=SWD mode=HOTPLUG freq=1000 \
        -r32 <bench_align_result addr> 128 > align_raw.txt

    python tools/g431_fit_align.py align_raw.txt

Get the symbol address from the map/ELF, e.g.
    arm-none-eabi-nm Objects/gimbal_g431.axf | findstr bench_align_result

Then copy the best-fit ``CALIB_ZERO_ELECTRIC`` / ``CALIB_SENSOR_DIRECTION`` into
``User/calib.h`` for that ``BOARD_CAN_NODE``.

Row layout (see User/bench_current.h):
    [0] mechanical angle before the pull
    [1] mechanical angle after the swing-up pull
    [2] delta = [1] - [0]
    [3] worst |current|, A
    [4] result code (0 = ok)
    [5] nFAULT after enable
    [6] mechanical angle after the swing-down pull   (older builds: sample count)
    [7] commanded electrical angle theta
"""

import math
import re
import struct
import sys

TWO_PI = 2.0 * math.pi
LINE = re.compile(r"^\s*0x[0-9A-Fa-f]+\s*:\s*((?:[0-9A-Fa-f]{8}\s*)+)$")


def load_rows(path):
    words = []
    for line in open(path, encoding="ascii", errors="replace").read().splitlines():
        m = LINE.match(line)
        if m:
            words.extend(struct.unpack("<f", struct.pack("<I", int(w, 16)))[0]
                         for w in m.group(1).split())
    rows = [words[i * 8:(i + 1) * 8] for i in range(len(words) // 8)]
    return [r for r in rows if r[4] == 0.0]


def wrap(angle):
    return (angle + math.pi) % TWO_PI - math.pi


def circ_mean(angles):
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    rows = load_rows(argv[1])
    if not rows:
        print("no successful rows in %s (all result codes non-zero?)" % argv[1])
        return 1
    print("rows read: %d" % len(rows))
    for i, r in enumerate(rows):
        print("  row%d  up=%.5f  down=%.5f  delta=%+.5f  worst=%.3fA  theta=%.4f"
              % (i, r[1], r[6], r[2], r[3], r[7]))

    scored = []
    for pp in range(1, 25):
        for direction in (1, -1):
            obs = []
            for r in rows:
                for m in (r[1], r[6]):
                    if m != 0.0:
                        obs.append(wrap(direction * pp * m - r[7]))
            if len(obs) < 4:
                continue
            mu = circ_mean(obs)
            dev = max(abs(wrap(z - mu)) for z in obs)
            rms = math.sqrt(sum(wrap(z - mu) ** 2 for z in obs) / len(obs))
            scored.append((dev, rms, pp, direction, mu))
    scored.sort()
    print("fit (max deviation over all rows and both pull directions):")
    for dev, rms, pp, direction, mu in scored[:5]:
        print("  pp=%2d dir=%+d zero=%.4f rad  max_dev=%.2f deg  rms=%.2f deg"
              % (pp, direction, mu, math.degrees(dev), math.degrees(rms)))

    dev, rms, pp, direction, mu = scored[0]
    print("\nbest: CALIB_ZERO_ELECTRIC = %.3f  CALIB_SENSOR_DIRECTION = %d"
          % (mu, direction))
    print("per-row zero with that fit:")
    for i, r in enumerate(rows):
        zu = wrap(direction * pp * r[1] - r[7])
        zd = wrap(direction * pp * r[6] - r[7]) if r[6] else zu
        print("  row%d  up=%+.4f  down=%+.4f  mean=%+.4f  (hysteresis %.2f deg)"
              % (i, zu, zd, wrap(circ_mean([zu, zd])),
                 math.degrees(abs(wrap(zu - zd)))))
    print("\nResidual spread is dominated by the hand-glued magnet ring (a few "
          "tenths of a degree mechanical maps to several degrees electrical); "
          "cos(5 deg) is 0.996, so it costs under half a percent of torque.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
