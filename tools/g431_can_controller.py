"""Send single-frame commands to the G431 board using a python-can adapter."""
import argparse
import math
import struct
import sys
import time


def encode_command(text):
    text = text.strip()
    if text in ('E', 'D', '?', 'R', 'H'):
        return text.encode('ascii')
    if not text or text[0] not in 'TVL':
        raise ValueError('use E, D, ?, R, H, T<rad>, V<volt> or L<rad/s>')
    value = float(text[1:])
    if not math.isfinite(value) or abs(value) > 1000000:
        raise ValueError('finite numeric value within +/-1000000 required')
    return text[0].encode('ascii') + struct.pack('<f', value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True, help='python-can interface, e.g. slcan, pcan, gs_usb')
    parser.add_argument('--channel', required=True, help='adapter channel, e.g. COM8, PCAN_USBBUS1 or gs_usb index 0')
    parser.add_argument('--bitrate', type=int, default=500000)
    parser.add_argument('--tx-id', type=lambda x: int(x, 0), default=0x201)
    parser.add_argument('--rx-id', type=lambda x: int(x, 0), default=0x281)
    parser.add_argument('--command', action='append', help='repeat for a sequence; default is a read-only ?')
    parser.add_argument('--seconds', type=float, default=2.5, help='total session duration, max 4.5 seconds')
    args = parser.parse_args()
    if not 0.1 <= args.seconds <= 4.5 or not all(0 <= x <= 0x7ff for x in [args.tx_id, args.rx_id]):
        parser.error('0.1 <= seconds <= 4.5 and standard 11-bit IDs required')
    commands = args.command or ['?']
    try:
        payloads = [encode_command(c) for c in commands]
    except (ValueError, OverflowError) as error:
        parser.error(str(error))
    # Motion sessions always attempt D, including after a send/receive exception.
    motion = any(data[0] in (ord('E'), ord('T')) for data in payloads)
    try:
        import can
    except ImportError:
        parser.error('install dependency: python -m pip install "python-can>=4.5,<5"')
    channel = args.channel
    if args.interface == 'gs_usb':
        try:
            channel = int(channel)
            if channel < 0:
                raise ValueError()
        except ValueError:
            parser.error('gs_usb channel must be a nonnegative adapter index, e.g. --channel 0')
        if sys.platform == 'win32':
            # Load the packaged libusb DLL before gs_usb asks PyUSB for its backend.
            try:
                import libusb_package
                if libusb_package.get_libusb1_backend() is None:
                    parser.error('libusb backend unavailable')
            except ImportError:
                parser.error('install Windows USB dependencies: python -m pip install gs_usb libusb-package')
    bus = can.Bus(interface=args.interface, channel=channel, bitrate=args.bitrate)
    pending = bytearray()
    off_confirmed = False
    reply_lines = 0
    stop_error = None

    def send(data, timeout=0.15):
        bus.send(can.Message(arbitration_id=args.tx_id, is_extended_id=False, data=data), timeout=timeout)

    def receive(timeout):
        nonlocal off_confirmed, reply_lines
        message = bus.recv(timeout)
        if message is None or message.is_extended_id or message.is_error_frame or message.arbitration_id != args.rx_id:
            return
        pending.extend(message.data)
        while b'\n' in pending:
            line, _, rest = pending.partition(b'\n')
            pending[:] = rest
            text = line.decode('ascii', errors='replace').strip()
            if text:
                reply_lines += 1
                print(text, flush=True)
            if text.startswith('EN=OFF') or text.startswith('EN OFF'):
                off_confirmed = True
        if len(pending) > 2048:
            pending.clear()

    try:
        deadline = time.monotonic() + args.seconds
        for data in payloads:
            if time.monotonic() >= deadline:
                raise TimeoutError('command sequence exceeded the bounded session duration')
            send(data, min(0.15, deadline - time.monotonic()))
            receive(0.01)
        poll_at = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            receive(min(0.02, max(0, deadline - time.monotonic())))
            if time.monotonic() >= poll_at:
                send(b'?')
                poll_at = time.monotonic() + 0.2
        if not reply_lines:
            raise TimeoutError('no complete reply from the G431; check CAN bitrate, wiring and termination')
    finally:
        try:
            if motion:
                off_confirmed = False
                pending.clear()
                send(b'D')
                stop_deadline = time.monotonic() + 0.5
                while time.monotonic() < stop_deadline and not off_confirmed:
                    receive(0.02)
                if not off_confirmed:
                    stop_error = 'D sent, but no EN OFF confirmation; check the board and power switch'
        except Exception as error:
            stop_error = f'could not confirm motor stop: {error}'
        finally:
            bus.shutdown()
        if stop_error:
            print(stop_error, file=sys.stderr)
            raise RuntimeError(stop_error)


if __name__ == '__main__':
    main()
