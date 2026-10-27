#!/usr/bin/env python3
"""G431 CAN <-> WebSocket bridge.

The web controller page (tools/web_controller/index.html) speaks Web Serial,
but the G431 board has no UART: its only host link is FDCAN.  This bridge
serves the page over HTTP and relays its text commands to the board over CAN,
so the same page can drive the G431 with no firmware or hardware change.

    python tools/g431_can_bridge.py
    -> open http://127.0.0.1:8766/  and press "连接"

Wire format it speaks (see keil5_g431/BSP/can_commands.c):
    0x201  E / D / ? / R / H        one byte
    0x201  T / V / L                one opcode + float32 little endian
    0x281  reply                   ASCII text (the firmware printf stream)
"""

from __future__ import annotations

import argparse
import asyncio
import os
import struct
import sys
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

try:
    import can
except ImportError:  # pragma: no cover
    sys.exit("python-can is missing: pip install python-can")

# candleLight needs an explicit libusb-1.0 backend; pyusb cannot find the DLL
# on this machine by itself (same workaround as tools/g431_can.py).
try:
    import libusb_package
    import usb.core

    _orig_find = usb.core.find

    def _find_with_backend(*args, **kwargs):
        kwargs.setdefault("backend", libusb_package.get_libusb1_backend())
        return _orig_find(*args, **kwargs)

    usb.core.find = _find_with_backend
except ImportError:
    pass

try:
    from websockets.asyncio.server import serve as ws_serve
except ImportError:  # pragma: no cover - older websockets
    from websockets.server import serve as ws_serve

CMD_ID = 0x201
REPLY_ID = 0x281
DEFAULT_BITRATE = 500_000
# One CAN frame per board main-loop iteration; the firmware queue is 40 deep and
# the loop also services FOC, so keep a small gap instead of flooding it.
CMD_GAP_S = 0.035
# A 12-bit encoder step at 25 kHz PWM still fits comfortably inside this gap.
PAGE_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web_controller")
INJECT_TEMPLATE = '<script>window.__G431_BRIDGE__={ws:"ws://127.0.0.1:%d"};</script>'

SINGLE_BYTE = "ED?RH"
VALUE_OPS = "TVL"


def encode_command(text: str) -> bytes:
    """Turn one page command into the exact frame the firmware expects."""
    if not text:
        raise ValueError("empty command")
    op = text[0].upper()
    if op in SINGLE_BYTE:
        return bytes([ord(op)])
    if op in VALUE_OPS:
        value = float(text[1:])
        return bytes([ord(op)]) + struct.pack("<f", value)
    raise ValueError(f"unsupported command: {text!r}")


class Bridge:
    def __init__(self, bus: can.BusABC) -> None:
        self.bus = bus
        self.clients: set = set()
        self.loop: asyncio.AbstractEventLoop | None = None
        self.queue: asyncio.Queue[str] = asyncio.Queue()
        self._thread: threading.Thread | None = None

    # ---- CAN -> browser -------------------------------------------------
    def start_can_reader(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop

        def pump() -> None:
            while True:
                try:
                    msg = self.bus.recv(timeout=0.2)
                except Exception:  # bus closed / adapter removed
                    return
                if msg is None or msg.arbitration_id != REPLY_ID:
                    continue
                text = msg.data.decode("ascii", "replace")
                loop.call_soon_threadsafe(self.queue.put_nowait, text)

        self._thread = threading.Thread(target=pump, name="can-rx", daemon=True)
        self._thread.start()

    async def broadcast_loop(self) -> None:
        while True:
            text = await self.queue.get()
            for ws in list(self.clients):
                try:
                    await ws.send(text)
                except Exception:
                    self.clients.discard(ws)


BRIDGE: Bridge | None = None


async def handle_client(ws, *_args) -> None:
    bridge = BRIDGE
    assert bridge is not None
    bridge.clients.add(ws)
    peer = getattr(ws, "remote_address", None)
    print(f"[ws] client connected {peer} (total {len(bridge.clients)})", flush=True)
    try:
        async for message in ws:
            if isinstance(message, (bytes, bytearray)):
                message = message.decode("ascii", "replace")
            for line in str(message).replace("\r", "\n").split("\n"):
                line = line.strip()
                if not line:
                    continue
                try:
                    frame = encode_command(line)
                except (ValueError, IndexError) as exc:
                    await ws.send(f"\r\nERR {exc}\r\n")
                    continue
                try:
                    bridge.bus.send(
                        can.Message(
                            arbitration_id=CMD_ID,
                            data=frame,
                            is_extended_id=False,
                        )
                    )
                except Exception as exc:  # bus error: tell the page, keep going
                    await ws.send(f"\r\nERR can: {exc}\r\n")
                    continue
                await asyncio.sleep(CMD_GAP_S)
    except Exception:
        pass
    finally:
        bridge.clients.discard(ws)
        print(f"[ws] client gone {peer} (total {len(bridge.clients)})", flush=True)


def make_http_handler(ws_port: int):
    inject = (INJECT_TEMPLATE % ws_port).encode()

    class PageHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=PAGE_ROOT, **kwargs)

        def do_GET(self):  # noqa: N802 - http.server API
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                try:
                    with open(os.path.join(PAGE_ROOT, "index.html"), "rb") as fh:
                        body = fh.read()
                except OSError:
                    self.send_error(404)
                    return
                if b"</head>" in body:
                    body = body.replace(b"</head>", inject + b"</head>", 1)
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            super().do_GET()

        def log_message(self, *_args):
            pass

    return PageHandler


def start_http(port: int, ws_port: int) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer(("127.0.0.1", port), make_http_handler(ws_port))
    thread = threading.Thread(target=server.serve_forever, name="http", daemon=True)
    thread.start()
    return server


async def run(args, bus: can.BusABC) -> None:
    global BRIDGE
    BRIDGE = Bridge(bus)
    loop = asyncio.get_running_loop()
    BRIDGE.start_can_reader(loop)
    asyncio.create_task(BRIDGE.broadcast_loop())

    print(f"[can] channel={args.channel} bitrate={args.bitrate} "
          f"cmd=0x{CMD_ID:03X} reply=0x{REPLY_ID:03X}", flush=True)
    if not args.no_http:
        print(f"[web] http://127.0.0.1:{args.http_port}/", flush=True)
    print(f"[ws ] ws://127.0.0.1:{args.ws_port}", flush=True)
    async with ws_serve(handle_client, "127.0.0.1", args.ws_port):
        await asyncio.Future()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", type=int, default=0, help="candleLight channel (default 0)")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE)
    parser.add_argument("--ws-port", type=int, default=8765)
    parser.add_argument("--http-port", type=int, default=8766)
    parser.add_argument("--no-http", action="store_true", help="only run the websocket bridge")
    args = parser.parse_args()

    bus = can.Bus(
        interface="gs_usb",
        channel=args.channel,
        index=args.channel,
        bitrate=args.bitrate,
    )
    if not args.no_http:
        start_http(args.http_port, args.ws_port)
    try:
        asyncio.run(run(args, bus))
    except KeyboardInterrupt:
        print("\n[bridge] stopped", flush=True)
    finally:
        bus.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
