"""
Teleop server — WebCodecs (VP8) + WebSocket video streaming.

- Subscribes to simulator ZeroMQ PUB socket (JPEG frames).
- Encodes frames to VP8 via PyAV and broadcasts binary packets over WebSocket.
- WASD commands are received as JSON text over the same WebSocket.

Binary packet protocol (server → client):
  byte 0      : 1 = keyframe, 0 = delta frame
  bytes 1–4   : pts as uint32 big-endian
  bytes 5+    : raw VP8 bitstream

Run (from project root):
    python -m teleop_server.main
"""

import asyncio
import json
import pathlib
import struct
from fractions import Fraction
from io import BytesIO

import av
import numpy as np
import zmq
import zmq.asyncio
from aiohttp import web
from PIL import Image

SIMULATOR_ADDR = "tcp://localhost:5555"
CMD_ADDR = "tcp://localhost:5556"
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8090
STATIC_DIR = pathlib.Path(__file__).parent.parent / "static"
TARGET_FPS = 30
WIDTH, HEIGHT = 1280, 720


# ---------------------------------------------------------------------------
# Frame store — fan-out to per-subscriber queues
# ---------------------------------------------------------------------------

class FrameStore:
    def __init__(self):
        self._subscribers: set[asyncio.Queue] = set()

    async def put(self, jpeg: bytes) -> None:
        for q in self._subscribers:
            if q.full():
                try:
                    q.get_nowait()      # drop oldest to make room
                except asyncio.QueueEmpty:
                    pass
            q.put_nowait(jpeg)

    def subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=2)
        self._subscribers.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue) -> None:
        self._subscribers.discard(q)


# ---------------------------------------------------------------------------
# ZeroMQ subscriber
# ---------------------------------------------------------------------------

async def zmq_receiver(store: FrameStore) -> None:
    ctx = zmq.asyncio.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 2)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.connect(SIMULATOR_ADDR)
    print(f"[teleop] subscribed to simulator at {SIMULATOR_ADDR}")

    while True:
        try:
            data: bytes = await sock.recv()
            await store.put(data[8:])       # skip 8-byte timestamp header
        except Exception as exc:
            print(f"[teleop] recv error: {exc}")
            await asyncio.sleep(0.01)


# ---------------------------------------------------------------------------
# VP8 encoder — encodes JPEG frames and broadcasts to WebSocket clients
# ---------------------------------------------------------------------------

def _make_vp8_encoder() -> av.CodecContext:
    codec = av.CodecContext.create("libvpx", "w")
    codec.width = WIDTH
    codec.height = HEIGHT
    codec.pix_fmt = "yuv420p"
    codec.framerate = TARGET_FPS
    codec.time_base = Fraction(1, TARGET_FPS)
    codec.bit_rate = 2_000_000
    codec.options = {
        "deadline": "realtime",
        "cpu-used": "8",
        "lag-in-frames": "0",
        "error-resilient": "1",
    }
    codec.open()
    return codec


async def encoder_task(store: FrameStore, app: web.Application) -> None:
    codec = _make_vp8_encoder()
    queue = store.subscribe()
    pts = 0

    try:
        while True:
            jpeg = await queue.get()
            img = np.array(Image.open(BytesIO(jpeg)))

            av_frame = av.VideoFrame.from_ndarray(img, format="rgb24")
            av_frame = av_frame.reformat(format="yuv420p")
            av_frame.pts = pts
            av_frame.time_base = Fraction(1, TARGET_FPS)
            pts += 1

            for pkt in codec.encode(av_frame):
                is_key = bool(pkt.is_keyframe)
                header = struct.pack(">BI", int(is_key), pkt.pts or 0)
                msg = header + bytes(pkt)

                dead: set[web.WebSocketResponse] = set()
                for ws in list(app["ws_clients"]):
                    try:
                        await ws.send_bytes(msg)
                    except Exception:
                        dead.add(ws)
                app["ws_clients"] -= dead

    except asyncio.CancelledError:
        pass
    finally:
        store.unsubscribe(queue)


# ---------------------------------------------------------------------------
# HTTP handlers
# ---------------------------------------------------------------------------

async def index_handler(request: web.Request) -> web.Response:
    return web.FileResponse(STATIC_DIR / "index.html")


async def client_js_handler(request: web.Request) -> web.Response:
    return web.FileResponse(STATIC_DIR / "client.js")


# ---------------------------------------------------------------------------
# WebSocket: binary VP8 video (server→client) + JSON commands (client→server)
# ---------------------------------------------------------------------------

async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    import aiohttp

    ws = web.WebSocketResponse()
    await ws.prepare(request)
    request.app["ws_clients"].add(ws)
    print("[ws] client connected")

    cmd_sock = request.app["cmd_sock"]

    async for msg in ws:
        if msg.type == aiohttp.WSMsgType.TEXT:
            data = json.loads(msg.data)
            if data.get("type") == "move":
                try:
                    cmd_sock.send_json(
                        {"dx": data["dx"], "dy": data["dy"]},
                        zmq.NOBLOCK,
                    )
                except zmq.Again:
                    pass
        elif msg.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.ERROR):
            break

    request.app["ws_clients"].discard(ws)
    print("[ws] client disconnected")
    return ws


# ---------------------------------------------------------------------------
# App lifecycle
# ---------------------------------------------------------------------------

async def on_startup(app: web.Application) -> None:
    app["ws_clients"] = set()

    store = FrameStore()
    app["zmq_task"] = asyncio.create_task(zmq_receiver(store))
    app["encoder_task"] = asyncio.create_task(encoder_task(store, app))

    cmd_ctx = zmq.asyncio.Context()
    cmd_sock = cmd_ctx.socket(zmq.PUSH)
    cmd_sock.setsockopt(zmq.SNDHWM, 1)
    cmd_sock.connect(CMD_ADDR)
    app["cmd_sock"] = cmd_sock
    print(f"[teleop] listening on http://{SERVER_HOST}:{SERVER_PORT}")


async def on_shutdown(app: web.Application) -> None:
    for key in ("zmq_task", "encoder_task"):
        app[key].cancel()
        try:
            await app[key]
        except asyncio.CancelledError:
            pass
    app["cmd_sock"].close()


def main() -> None:
    app = web.Application()
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/ws", websocket_handler)
    app.router.add_get("/", index_handler)
    app.router.add_get("/client.js", client_js_handler)
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)


if __name__ == "__main__":
    main()
