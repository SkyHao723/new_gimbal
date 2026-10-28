#!/usr/bin/env python3
"""G431 CAN <-> WebSocket bridge (single board or a bus of several boards).

The web controller page (tools/web_controller/index.html) speaks Web Serial, but
the G431 board has no UART: its only host link is FDCAN.  This bridge serves the
page over HTTP and relays its commands to the boards over CAN, so the same page
can drive three boards independently (`--nodes 1,2,3`).

    python tools/g431_can_bridge.py --nodes 1,2,3
    -> open http://127.0.0.1:8766/  and press "连接"

Wire format it speaks (see keil5_g431/BSP/can_commands.c):
    0x200+n  E / D / ? / R / H        one byte
    0x200+n  T / V / L                one opcode + float32 little endian
    0x280+n  reply                    ASCII text (the firmware printf stream)
    0x200    broadcast command        no reply

Browser protocol (JSON text frames):
    <- {"nodes": [1, 2, 3]}                    on connect
    <- {"node": 2, "text": "EN=ON angle=..."}  one reply chunk from board 2
    <- {"error": "..."}                        bridge / CAN error
    -> {"node": 2, "cmd": "T1.5"}              command to board 2
    -> {"node": "all", "cmd": "?"}             command to every board
    -> {"node": 0, "cmd": "D"}                 broadcast (0x200, nobody replies)
Plain text frames are still accepted and go to the first configured node, so an
old client keeps working.
"""

from __future__ import annotations

import argparse
import asyncio
import json
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

DEFAULT_BITRATE = 500_000
BROADCAST_ID = 0x200
# One CAN frame per board main-loop iteration; the firmware queue is 40 deep and
# the loop also services FOC, so keep a small gap instead of flooding it.
CMD_GAP_S = 0.035
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
    def __init__(self, bus: can.BusABC, nodes: list[int]) -> None:
        self.bus = bus
        self.nodes = nodes
        self.default_node = nodes[0] if nodes else 1
        self.reply_to_node = {0x280 + n: n for n in nodes}
        self.clients: set = set()
        self.loop: asyncio.AbstractEventLoop | None = None
        self.queue: asyncio.Queue[dict] = asyncio.Queue()
        self._thread: threading.Thread | None = None

    def command_id(self, node: int) -> int:
        return BROADCAST_ID if node == 0 else 0x200 + node

    # ---- CAN -> browser -------------------------------------------------
    def start_can_reader(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop

        def pump() -> None:
            while True:
                try:
                    msg = self.bus.recv(timeout=0.2)
                except Exception:  # bus closed / adapter removed
                    return
                if msg is None:
                    continue
                node = self.reply_to_node.get(msg.arbitration_id)
                if node is None:
                    continue
                self.loop.call_soon_threadsafe(
                    self.queue.put_nowait,
                    {"node": node, "text": msg.data.decode("ascii", "replace")},
                )

        self._thread = threading.Thread(target=pump, name="can-rx", daemon=True)
        self._thread.start()

    async def broadcast_loop(self) -> None:
        while True:
            payload = await self.queue.get()
            text = json.dumps(payload)
            for ws in list(self.clients):
                try:
                    await ws.send(text)
                except Exception:
                    self.clients.discard(ws)

    # ---- browser -> CAN -------------------------------------------------
    async def dispatch(self, ws, message: str) -> None:
        message = message.strip()
        if not message:
            return
        target = self.default_node
        cmd = message
        if message.startswith("{"):
            try:
                payload = json.loads(message)
            except json.JSONDecodeError as exc:
                await ws.send(json.dumps({"error": f"bad json: {exc}"}))
                return
            if not isinstance(payload, dict) or "cmd" not in payload:
                await ws.send(json.dumps({"error": "missing cmd"}))
                return
            cmd = str(payload.get("cmd", "")).strip()
            target = payload.get("node", self.default_node)

        if not cmd:
            return
        try:
            frame = encode_command(cmd)
        except (ValueError, IndexError) as exc:
            await ws.send(json.dumps({"error": str(exc)}))
            return

        if target == "all":
            targets = list(self.nodes)
        else:
            try:
                targets = [int(target)]
            except (TypeError, ValueError):
                await ws.send(json.dumps({"error": f"bad node {target!r}"}))
                return
            if targets[0] != 0 and targets[0] not in self.nodes:
                # still allow it: the board may be on the bus but not monitored
                pass

        for node in targets:
            try:
                self.bus.send(
                    can.Message(
                        arbitration_id=self.command_id(node),
                        data=frame,
                        is_extended_id=False,
                    )
                )
            except Exception as exc:  # bus error: tell the page, keep going
                await ws.send(json.dumps({"error": f"can: {exc}"}))
                return
            await asyncio.sleep(CMD_GAP_S)


BRIDGE: Bridge | None = None


async def handle_client(ws, *_args) -> None:
    bridge = BRIDGE
    assert bridge is not None
    bridge.clients.add(ws)
    peer = getattr(ws, "remote_address", None)
    print(f"[ws] client connected {peer} (total {len(bridge.clients)})", flush=True)
    try:
        await ws.send(json.dumps({"nodes": bridge.nodes}))
        async for message in ws:
            if isinstance(message, (bytes, bytearray)):
                message = message.decode("ascii", "replace")
            for line in str(message).replace("\r", "\n").split("\n"):
                await bridge.dispatch(ws, line)
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


async def run(args, bus: can.BusABC, nodes: list[int]) -> None:
    global BRIDGE
    BRIDGE = Bridge(bus, nodes)
    loop = asyncio.get_running_loop()
    BRIDGE.start_can_reader(loop)
    asyncio.create_task(BRIDGE.broadcast_loop())

    ids = ", ".join(f"0x{0x200 + n:03X}/0x{0x280 + n:03X}" for n in nodes)
    print(f"[can] channel={args.channel} bitrate={args.bitrate} nodes={nodes} ({ids})",
          flush=True)
    if not args.no_http:
        print(f"[web] http://127.0.0.1:{args.http_port}/", flush=True)
    print(f"[ws ] ws://127.0.0.1:{args.ws_port}", flush=True)
    async with ws_serve(handle_client, "127.0.0.1", args.ws_port):
        await asyncio.Future()


def parse_nodes(value: str) -> list[int]:
    nodes = []
    for part in value.replace(" ", "").split(","):
        if not part:
            continue
        node = int(part, 0)
        if not 1 <= node <= 7:
            raise argparse.ArgumentTypeError("node ids must be 1..7")
        if node not in nodes:
            nodes.append(node)
    if not nodes:
        raise argparse.ArgumentTypeError("at least one node id is required")
    return nodes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", type=int, default=0, help="candleLight channel (default 0)")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE)
    parser.add_argument("--nodes", type=parse_nodes, default=[1, 2, 3],
                        help="comma separated board node ids, e.g. 1,2,3 (default 1,2,3)")
    parser.add_argument("--node", type=parse_nodes, default=None,
                        help="single board id (shortcut for --nodes N)")
    parser.add_argument("--ws-port", type=int, default=8765)
    parser.add_argument("--http-port", type=int, default=8766)
    parser.add_argument("--no-http", action="store_true", help="only run the websocket bridge")
    args = parser.parse_args()

    nodes = args.node if args.node else args.nodes
    bus = can.Bus(
        interface="gs_usb",
        channel=args.channel,
        index=args.channel,
        bitrate=args.bitrate,
    )
    if not args.no_http:
        start_http(args.http_port, args.ws_port)
    try:
        asyncio.run(run(args, bus, nodes))
    except KeyboardInterrupt:
        print("\n[bridge] stopped", flush=True)
    finally:
        bus.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
