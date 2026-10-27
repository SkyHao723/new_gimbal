"""Read the running G431 diagnostic snapshot over SWD without enabling drive."""
import argparse
import json
from pathlib import Path
import re
import struct
import subprocess
import time


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=root/'keil5_g431/Listings/gimbal_g431.map')
    parser.add_argument('--cli', default='D:/program/STM32CubeCLT_1.21.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe')
    parser.add_argument('--serial', help='optional ST-Link serial number')
    parser.add_argument('--samples', type=int, default=2)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.samples <= 10:
        parser.error('1 <= samples <= 10 required')
    match = re.search(r'\bg431_diagnostics\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)\b',
                      args.map.read_text(encoding='utf-8', errors='replace'))
    if not match:
        parser.error('diagnostic symbol not found; build the matching G431 firmware first')
    address = int(match[1], 16)
    size = int(match[2])
    connection = ['-c', 'port=SWD', 'mode=HOTPLUG', 'freq=1000']
    if args.serial:
        connection.append('sn='+args.serial)
    # Order must match volatile G431Diagnostics in keil5_g431/User/diagnostics.h.
    keys = ['magic', 'stage', 'core_clock', 'uptime_ms', 'loops', 'enabled',
            'sleep_high', 'off_high', 'fault_high', 'sensor_valid', 'sensor_errors',
            'can_psr', 'can_ecr', 'tx_dropped', 'angle', 'velocity',
            'cs_valid', 'cs_samples', 'cs_trips', 'cs_blind',
            'cs_peak', 'cs_allowed', 'cs_thermal', 'cs_amp_mid',
            'cs_trig', 'cs_probe_a', 'cs_probe_b', 'cs_probe_c']
    if size != 4 * len(keys):
        parser.error('G431Diagnostics is %d bytes but this script expects %d; '
                     'rebuild the firmware and update the key list together'
                     % (size, 4 * len(keys)))
    float_index = (14, 15, 20, 21, 22, 23)
    snapshots, logs = [], []
    for _ in range(args.samples):
        result = subprocess.run([args.cli, *connection, '-r32', hex(address), str(size),
                                 '-r32', '0xE000ED28', '8'],
                                capture_output=True, text=True, errors='replace', timeout=15)
        logs.append(result.stdout + result.stderr)
        if result.returncode != 0 or not re.search(r'Device ID\s*:\s*0x468\b', result.stdout):
            raise RuntimeError(result.stdout + result.stderr)
        memory = {}
        for line in result.stdout.splitlines():
            row = re.fullmatch(r'0x([\dA-Fa-f]+)\s*:\s*((?:[\dA-Fa-f]{8}\s*)+)', line.strip())
            if row:
                for i, word in enumerate(row[2].split()):
                    memory[int(row[1], 16) + 4*i] = int(word, 16)
        words = [memory[address+4*i] for i in range(len(keys))]
        if words[0] != 0x47343331:
            raise RuntimeError('snapshot magic mismatch: firmware/map mismatch or application did not boot')
        snapshot = dict(zip(keys, words))
        for i in float_index:
            snapshot[keys[i]] = struct.unpack('<f', struct.pack('<I', words[i]))[0]
        snapshot['cfsr'] = memory[0xE000ED28]
        snapshot['hfsr'] = memory[0xE000ED2c]
        snapshots.append(snapshot)
        print(json.dumps(snapshot), flush=True)
        time.sleep(0.1)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(snapshots, indent=2), encoding='utf-8')
        args.output.with_suffix('.log').write_text('\n'.join(logs), encoding='utf-8')


if __name__ == '__main__':
    main()
