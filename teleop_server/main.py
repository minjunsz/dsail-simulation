"""
Teleop server process.

- Subscribes to the simulator ZeroMQ PUB socket and caches the latest JPEG frame.
- Serves a browser client over HTTP.
- Streams video as MJPEG over HTTP (/stream).
- Handles WASD commands over WebSocket (/ws).

Run (from project root):
    python -m teleop_server.main
"""

import asyncio
import json
import pathlib

import zmq.asyncio
from aiohttp import web

SIMULATOR_ADDR = "tcp://localhost:5555"
CMD_ADDR = "tcp://localhost:5556"
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8090
STATIC_DIR = pathlib.Path(__file__).parent.parent / "static"


# ---------------------------------------------------------------------------
# Frame store — asyncio.Condition for frame-triggered MJPEG delivery
# ---------------------------------------------------------------------------

class FrameStore:
    def __init__(self):
        self.jpeg: bytes | None = None
        self._cond = asyncio.Condition()

    async def put(self, jpeg: bytes) -> None:
        async with self._cond:
            self.jpeg = jpeg
            self._cond.notify_all()

    async def get_next(self) -> bytes:
        async with self._cond:
            await self._cond.wait()
            return self.jpeg


# ---------------------------------------------------------------------------
# ZeroMQ subscriber (runs as a background asyncio task)
# ---------------------------------------------------------------------------

async def zmq_receiver(store: FrameStore) -> None:
    ctx = zmq.asyncio.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 2)          # keep only the latest frames
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.connect(SIMULATOR_ADDR)
    print(f"[teleop] subscribed to simulator at {SIMULATOR_ADDR}")

    while True:
        try:
            data: bytes = await sock.recv()
            jpeg_bytes = data[8:]            # skip 8-byte timestamp header
            await store.put(jpeg_bytes)
        except Exception as exc:
            print(f"[teleop] recv error: {exc}")
            await asyncio.sleep(0.01)


# ---------------------------------------------------------------------------
# HTTP handlers
# ---------------------------------------------------------------------------

async def index_handler(request: web.Request) -> web.Response:
    return web.FileResponse(STATIC_DIR / "index.html")


async def client_js_handler(request: web.Request) -> web.Response:
    return web.FileResponse(STATIC_DIR / "client.js")


async def mjpeg_handler(request: web.Request) -> web.StreamResponse:
    store: FrameStore = request.app["frame_store"]
    response = web.StreamResponse(
        headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame"}
    )
    await response.prepare(request)

    try:
        while True:
            jpeg = await store.get_next()
            if jpeg is None:
                continue
            payload = (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + jpeg
                + b"\r\n"
            )
            await response.write(payload)
    except (ConnectionResetError, asyncio.CancelledError):
        pass

    return response


# ---------------------------------------------------------------------------
# WebSocket command handler (WASD only — no WebRTC signaling)
# ---------------------------------------------------------------------------

async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    import aiohttp
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    print("[cmd] client connected")

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

    print("[cmd] client disconnected")
    return ws


# ---------------------------------------------------------------------------
# App lifecycle
# ---------------------------------------------------------------------------

async def on_startup(app: web.Application) -> None:
    store = FrameStore()
    app["frame_store"] = store
    app["zmq_task"] = asyncio.create_task(zmq_receiver(store))

    cmd_ctx = zmq.asyncio.Context()
    cmd_sock = cmd_ctx.socket(zmq.PUSH)
    cmd_sock.setsockopt(zmq.SNDHWM, 1)
    cmd_sock.connect(CMD_ADDR)
    app["cmd_sock"] = cmd_sock
    print(f"[teleop] listening on http://{SERVER_HOST}:{SERVER_PORT}")


async def on_shutdown(app: web.Application) -> None:
    app["zmq_task"].cancel()
    try:
        await app["zmq_task"]
    except asyncio.CancelledError:
        pass
    app["cmd_sock"].close()


def main() -> None:
    app = web.Application()
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/stream", mjpeg_handler)
    app.router.add_get("/ws", websocket_handler)
    app.router.add_get("/", index_handler)
    app.router.add_get("/client.js", client_js_handler)
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)


if __name__ == "__main__":
    main()
