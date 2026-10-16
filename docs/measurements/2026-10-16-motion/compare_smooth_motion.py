import argparse
import json
import math
import re
import subprocess
import time
from pathlib import Path
import serial

parser = argparse.ArgumentParser()
parser.add_argument('label')
parser.add_argument('--flash')
parser.add_argument('--step', type=float, default=0.2)
args = parser.parse_args()
if not 0.0<args.step<=0.5:
    parser.error('--step must be between 0 and 0.5 rad')
out = Path('D:/dsh_build/releases/motion-comparison')
out.mkdir(parents=True, exist_ok=True)
records = []
raw = []
port = serial.Serial('COM8', 115200, timeout=0.08, write_timeout=0.3)

def send(command):
    port.write((command+'\r\n').encode('ascii'))

def status():
    send('?')
    until = time.monotonic()+0.3
    while time.monotonic()<until:
        line = port.readline().decode('ascii', errors='replace').strip()
        if line:
            raw.append(line)
        if 'FAULT' in line:
            raise RuntimeError(line)
        match = re.search(r'EN=(ON|OFF)\s+angle=([-\d.]+)\s+vel=([-\d.]+)', line)
        if match:
            return dict(enabled=match[1], angle=float(match[2]), velocity=float(match[3]))
    raise RuntimeError('No status reply')

try:
    send('D')
    time.sleep(0.08)
    assert status()['enabled']=='OFF'
    if args.flash:
        cli = 'D:/program/STM32CubeCLT_1.21.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe'
        result = subprocess.run([cli,'-c','port=SWD','mode=UR','freq=1000',
                                 '-w',args.flash,'-v','-rst'],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=25, text=True, errors='replace')
        (out/(args.label+'-flash.log')).write_text(result.stdout,encoding='utf-8')
        print(result.stdout)
        send('D')
        time.sleep(0.1)
        if result.returncode:
            raise RuntimeError('Flash failed')
        assert status()['enabled']=='OFF'
    send('V3')
    time.sleep(0.04)
    send('E')
    start = status()
    assert start['enabled']=='ON'
    origin = start['angle']
    energized = time.monotonic()
    for phase, target in [('outward',origin+args.step), ('return',origin)]:
        send(f'T{target:.4f}')
        begin = time.monotonic()
        while time.monotonic()-begin<1.15:
            sample = status()
            sample.update(t=time.monotonic()-begin,phase=phase,target=target)
            records.append(sample)
            if sample['enabled']!='ON' or abs(sample['angle']-origin)>args.step+0.4:
                raise RuntimeError('Stopped or excessive displacement')
            if time.monotonic()-energized>3.0:
                raise RuntimeError('Energized test timeout')
            time.sleep(0.012)
    send('D')
    time.sleep(0.08)
    assert status()['enabled']=='OFF'
    for phase, direction in [('outward',1),('return',-1)]:
        points = [s for s in records if s['phase']==phase]
        projected = [direction*(s['angle']-s['target']) for s in points]
        tail = [s['angle'] for s in points if s['t']>0.8]
        travel = sum(abs(b['angle']-a['angle']) for a,b in zip(points,points[1:]))
        print(json.dumps(dict(label=args.label,phase=phase,samples=len(points),
                              overshoot_deg=math.degrees(max(0,max(projected))),
                              tail_span_deg=math.degrees(max(tail)-min(tail)),
                              final_error_deg=math.degrees(points[-1]['angle']-points[-1]['target']),
                              total_travel_deg=math.degrees(travel))))
finally:
    try:
        send('D')
        time.sleep(0.08)
        final = status()
        print('FINAL',json.dumps(final))
        assert final['enabled']=='OFF'
    finally:
        port.close()
        (out/(args.label+'.json')).write_text(json.dumps(records,indent=2),encoding='utf-8')
        (out/(args.label+'.log')).write_text('\n'.join(raw),encoding='utf-8')
