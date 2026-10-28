"""Talk to the G431 gimbal over CAN (candleLight / gs_usb, 500 kbit/s).

Wire format (see keil5_g431/BSP/can_commands.c):
  host -> board, standard id 0x201, classic CAN
      'E' | '?' | 'R' | 'H'            single byte opcodes
      'D'                              discard queued commands
      'T' | 'V' | 'L' + float32 LE     target / voltage limit / velocity limit
  board -> host, standard id 0x281, ASCII text in 8-byte chunks (printf stream)

Usage:
  python g431_can.py ping                 # send ? and print the status line
  python g431_can.py cmd E
  python g431_can.py cmd T 1.5
  python g431_can.py cmd V 4
  python g431_can.py cmd L 10
  python g431_can.py cmd D
  python g431_can.py listen 5             # just print frames for 5 seconds
"""
import argparse
import os
import struct
import sys
import time

try:  # pyusb cannot always locate libusb-1.0.dll on Windows; ship our own backend.
    import libusb_package
    import usb.core

    _backend = libusb_package.get_libusb1_backend()
    if _backend is not None:
        _orig_find = usb.core.find

        def _find_with_backend(*args, **kwargs):
            kwargs.setdefault("backend", _backend)
            return _orig_find(*args, **kwargs)

        usb.core.find = _find_with_backend
except Exception:  # pragma: no cover - convenience for Windows hosts only
    pass

import can

CMD_ID = 0x201
REPLY_ID = 0x281


def set_node(node):
    """Pick the CAN IDs of one board.

    node 1..7 -> command 0x201..0x207, reply 0x281..0x287 (default node 1).
    node 0    -> broadcast command 0x200, no reply expected (use for 'D' all-stop).
    """
    global CMD_ID, REPLY_ID
    if node == 0:
        CMD_ID, REPLY_ID = 0x200, -1
    else:
        CMD_ID, REPLY_ID = 0x200 + node, 0x280 + node


def open_bus(bitrate=500000, channel=0):
    return can.Bus(interface="gs_usb", channel=channel, index=channel, bitrate=bitrate)


def send(bus, text):
    """Send one command frame; numeric opcodes are encoded as float32 LE."""
    if not text:
        raise SystemExit("empty command")
    op = text[0]
    if op in ("T", "V", "L", "M"):
        value = float(text[1:])
        payload = bytes([ord(op)]) + struct.pack("<f", value)
    else:
        payload = bytes([ord(op)])
    bus.send(can.Message(arbitration_id=CMD_ID, data=payload, is_extended_id=False))


def collect(bus, seconds=0.4):
    deadline = time.monotonic() + seconds
    chunks = []
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None:
            break
        if msg.arbitration_id == REPLY_ID:
            chunks.append(bytes(msg.data))
    return b"".join(chunks).decode("ascii", "replace")


def main():
    parser = argparse.ArgumentParser(description="G431 gimbal CAN helper")
    parser.add_argument("--bitrate", type=int, default=500000)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--node", type=int, default=1,
                        help="board node id 1..7 (0 = broadcast, no reply expected)")
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("ping")
    p_cmd = sub.add_parser("cmd")
    p_cmd.add_argument("text", nargs="+")
    p_listen = sub.add_parser("listen")
    p_listen.add_argument("seconds", type=float, nargs="?", default=3.0)
    args = parser.parse_args()

    set_node(args.node)
    bus = open_bus(args.bitrate, args.channel)
    try:
        if args.action == "listen":
            sys.stdout.write(collect(bus, args.seconds))
            sys.stdout.flush()
            return
        text = "".join(args.text) if args.action == "cmd" else "?"
        send(bus, text)
        if REPLY_ID < 0:
            print(f"(broadcast 0x{CMD_ID:03X} sent: {text!r}; no reply expected)")
            return
        reply = collect(bus)
        if not reply:
            print(f"(no reply to {text!r} from node {args.node} "
                  f"[cmd 0x{CMD_ID:03X}]; is the board powered and the bus "
                  f"terminated?)")
        else:
            print(reply)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
