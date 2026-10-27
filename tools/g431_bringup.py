"""Operate only the separate DRVOFF-grounded, time-bounded bring-up image."""
import argparse
import json
from pathlib import Path
import re
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
CLI = 'D:/program/STM32CubeCLT_1.21.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe'
FIELDS = ('magic request ack stage clock_hz uptime_ms loops sleep_high fault_high '
          'moe active stop_reason started_ms ended_ms pulses updates gpiob_idr '
          'cfsr hfsr uq electrical_angle pwm_period heartbeat').split()
ENCODER_FIELDS = ('sensor_ok sensor_reads sensor_errors raw magnet_status magnet_errors '
                  'sensor_ms sensor_angle sensor_velocity motion_start_angle motion_end_angle '
                  'encoder_counts').split()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['check', 'pullupcheck', 'softcheck', 'slowcheck', 'wake', 'spin15', 'spin20', 'spin15warn', 'stop'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=2, help='2..30 snapshots for check')
    parser.add_argument('--interval', type=float, default=0.1, help='0.01..1 seconds between check snapshots')
    args = parser.parse_args()
    if args.action in ('pullupcheck', 'slowcheck'):
        parser.error('G431 PB6 does not support I2C1_SCL; use software I2C or reroute SCL to a supported pin')
    if not 2 <= args.samples <= 30:
        parser.error('2 <= samples <= 30 required')
    if not 0.01 <= args.interval <= 1:
        parser.error('0.01 <= interval <= 1 required')
    mapping = (ROOT/'keil5_g431/ListingsBringup/g431_bringup.map').read_text(errors='replace')
    match = re.search(r'\bbringup_diag\s+(0x[0-9a-fA-F]+)\s+Data\s+(92|140|152)\b', mapping)
    if not match:
        raise RuntimeError('build matching bring-up firmware first')
    address = int(match[1], 16)
    size = int(match[2])
    fields = FIELDS + (ENCODER_FIELDS if size >= 140 else [])
    if size == 152:
        fields += ['magnet_agc', 'magnet_magnitude', 'magnet_data_ms']
    logs, samples = [], []

    def call(*params):
        result = subprocess.run([CLI, '-c', 'port=SWD', 'mode=HOTPLUG', 'freq=1000',
                                 *params], capture_output=True, text=True, errors='replace', timeout=8)
        logs.append(result.stdout+result.stderr)
        if result.returncode or not re.search(r'Device ID\s*:\s*0x468\b', result.stdout):
            raise RuntimeError(result.stdout+result.stderr)
        return result.stdout

    def read():
        output = call('-r32', hex(address), str(size), '-r32', '0x40012c44', '4',
                      '-r32', '0x48000414', '4')
        memory = {}
        for line in output.splitlines():
            m = re.fullmatch(r'0x([\dA-Fa-f]+)\s*:\s*((?:[\dA-Fa-f]{8}\s*)+)', line.strip())
            if m:
                for i, word in enumerate(m[2].split()):
                    memory[int(m[1], 16)+i*4] = int(word, 16)
        words = [memory[address+i*4] for i in range(size//4)]
        if words[0] != {92: 0x42555031, 140: 0x42555032, 152: 0x42555033}[size]:
            raise RuntimeError('firmware identity mismatch; no command sent')
        sample = dict(zip(fields, words))
        for key in ['uq', 'electrical_angle', 'sensor_angle', 'sensor_velocity',
                    'motion_start_angle', 'motion_end_angle']:
            if key not in sample:
                continue
            sample[key] = struct.unpack('<f', struct.pack('<I', sample[key]))[0]
        if sample.get('encoder_counts', 0) >= 0x80000000:
            sample['encoder_counts'] -= 0x100000000
        sample['live_moe'] = bool(memory[0x40012c44] & 0x8000)
        sample['live_sleep'] = bool(memory[0x48000414] & 1)
        samples.append(sample)
        print(json.dumps(sample), flush=True)
        return sample

    def request(value):
        call('-w32', hex(address+4), str(value))

    initial = read()
    try:
        if args.action in ['wake', 'spin15', 'spin20', 'spin15warn']:
            if initial['active'] or initial['live_moe']:
                raise RuntimeError('unexpected active drive; refusing another test')
            if initial['clock_hz'] != 170000000 or initial['cfsr'] or initial['hfsr']:
                raise RuntimeError('clock or processor fault')
            request(1)
            time.sleep(0.15)
            awake = read()
            if awake['stage'] != 3 or not awake['live_sleep'] or awake['live_moe']:
                raise RuntimeError('driver wake check failed')
            time.sleep(0.1)
            awake = read()
            if not awake['fault_high']:
                raise RuntimeError('nFAULT remains LOW with PWM off; no spin command sent')
            if args.action.startswith('spin'):
                warning_test = args.action == 'spin15warn'
                if warning_test and (size != 152 or awake['magnet_status'] & 0x38 not in (0x20, 0x30)):
                    raise RuntimeError('warning diagnostic requires detected magnet without MH')
                if not warning_test and size >= 140 and (not awake['sensor_ok'] or awake['magnet_status'] & 0x38 != 0x20):
                    raise RuntimeError('encoder/magnet not healthy; no spin command sent')
                before = awake['pulses']
                request(9 if warning_test else (2 if args.action == 'spin15' else 4))
                deadline = time.monotonic()+1.8
                while time.monotonic() < deadline:
                    read()
                    time.sleep(0.08)
                final = read()
                if (final['pulses'] != before+1 or final['active'] or final['live_moe'] or
                        final['live_sleep'] or final['stop_reason'] != 1):
                    raise RuntimeError('pulse did not complete normally; see saved diagnostic log')
                if size >= 140:
                    if (final['sensor_errors'] != awake['sensor_errors'] or
                            (not warning_test and (final['magnet_errors'] != awake['magnet_errors'] or not final['sensor_ok'])) or
                            (warning_test and final['magnet_status'] & 0x38 not in (0x20, 0x30))):
                        raise RuntimeError('encoder fault during test')
                    print('MEASURED_MOVE_RAD', final['motion_end_angle']-final['motion_start_angle'])
                else:
                    print('PULSE_COMPLETE: output sequence only; physical rotation requires observation.')
        elif args.action == 'stop':
            request(3)
        elif args.action in ['check', 'pullupcheck', 'softcheck', 'slowcheck']:
            if args.action in ['pullupcheck', 'softcheck', 'slowcheck']:
                if size < 140 or initial['active'] or initial['live_moe']:
                    raise RuntimeError('pullup diagnostic requires disabled encoder image')
                request({'pullupcheck': 5, 'softcheck': 6, 'slowcheck': 8}[args.action])
            for _ in range(args.samples-2):
                time.sleep(args.interval)
                read()
    finally:
        try:
            if args.action != 'check':
                request(3)
                time.sleep(0.03)
            final = read()
            if final['active'] or final['live_moe'] or final['live_sleep']:
                raise RuntimeError('STOP NOT CONFIRMED: remove 24V power')
            print('STOP_CONFIRMED: nSLEEP=0, MOE=0, active=0')
        finally:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(samples, indent=2), encoding='utf-8')
            args.output.with_suffix('.log').write_text('\n'.join(logs), encoding='utf-8')

if __name__ == '__main__':
    main()
