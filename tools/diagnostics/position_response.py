"""Short, bounded position response capture on the connected test motor."""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('label')
    parser.add_argument('--port', default='COM8')
    parser.add_argument('--target', type=float, default=math.pi)
    parser.add_argument('--seconds', type=float, default=4.5)
    parser.add_argument('--flash', type=Path)
    parser.add_argument('--output', type=Path, default=Path('D:/dsh_build/releases/position-response'))
    args = parser.parse_args()
    if not math.isfinite(args.target) or not 1.0 <= args.seconds <= 4.5:
        parser.error('finite target and 1 <= seconds <= 4.5 required')
    args.output.mkdir(parents=True, exist_ok=True)
    samples, lines = [], []
    port = serial.Serial(args.port, 115200, timeout=0.08, write_timeout=0.2)

    def send(command):
        port.write((command+'\r\n').encode('ascii'))

    def status():
        send('?')
        deadline = time.monotonic()+0.3
        while time.monotonic() < deadline:
            line = port.readline().decode('ascii', errors='replace').strip()
            if line:
                lines.append(line)
            if 'FAULT' in line:
                raise RuntimeError(line)
            if line.startswith('EN='):
                fields = dict(re.findall(r'(\w+)=([^\s]+)', line))
                result = {k: float(v) for k, v in fields.items()
                          if re.fullmatch(r'[-+\d.eE]+', v) and k != 'EN'}
                result['enabled'] = fields['EN']
                return result
        raise RuntimeError('status timeout')

    try:
        send('D')
        time.sleep(0.08)
        assert status()['enabled'] == 'OFF'
        if args.flash:
            cli = 'D:/program/STM32CubeCLT_1.21.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe'
            result = subprocess.run([cli, '-c', 'port=SWD', 'mode=UR', 'freq=1000',
                                     '-w', str(args.flash.resolve()), '-v', '-rst'],
                                    timeout=25, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, errors='replace')
            (args.output/(args.label+'-flash.log')).write_text(result.stdout, encoding='utf-8')
            send('D')
            time.sleep(0.1)
            assert result.returncode == 0, result.stdout
            assert status()['enabled'] == 'OFF'
            print('Flash verified')
        send('V3')
        send('L20')
        time.sleep(0.05)
        initial = status()
        if abs(args.target-initial['angle']) > 6.5:
            raise RuntimeError('requested displacement exceeds one-turn test bound')
        send('E')
        assert status()['enabled'] == 'ON'
        send(f'T{args.target:.6f}')
        begin = time.monotonic()
        while time.monotonic()-begin < args.seconds:
            sample = status()
            sample['t'] = time.monotonic()-begin
            samples.append(sample)
            if sample['enabled'] != 'ON' or abs(sample['angle']-args.target) > abs(initial['angle']-args.target)+0.5:
                raise RuntimeError('motor stopped or excessive displacement')
            time.sleep(0.012)
        tail = [s['angle'] for s in samples if s['t'] >= args.seconds-1.0]
        print(json.dumps(dict(label=args.label, initial=initial['angle'], target=args.target,
                              samples=len(samples),
                              tail_span_deg=math.degrees(max(tail)-min(tail)),
                              tail_error_deg=math.degrees(samples[-1]['angle']-args.target))))
    finally:
        try:
            send('D')
            time.sleep(0.08)
            final = status()
            print('FINAL', json.dumps(final))
            assert final['enabled'] == 'OFF'
        finally:
            port.close()
            (args.output/(args.label+'.json')).write_text(json.dumps(samples, indent=2), encoding='utf-8')
            (args.output/(args.label+'.log')).write_text('\n'.join(lines), encoding='utf-8')


if __name__ == '__main__':
    main()
