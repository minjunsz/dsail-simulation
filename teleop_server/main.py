"""
Teleop server process.

- Subscribes to the simulator ZeroMQ PUB socket and caches the latest frame.
- Serves a browser client over HTTP.
- Handles WebRTC signaling over WebSocket (/ws) and streams video via aiortc.

Run (from project root):
    python -m teleop_server.main
"""

import asyncio
import json
import pathlib
import struct
from io import BytesIO

import aiohttp
import numpy as np
import zmq.asyncio
from aiohttp import web
from PIL import Image

import av
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack

SIMULATOR_ADDR = "tcp://localhost:5555"
CMD_ADDR = "tcp://localhost:5556"
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8090
STATIC_DIR = pathlib.Path(__file__).parent.parent / "static"


# ---------------------------------------------------------------------------
# Frame store — a simple container; Python GIL makes single-ref assignment safe
# ---------------------------------------------------------------------------

class LatestFrame:
    frame: np.ndarray | None = None


# ---------------------------------------------------------------------------
# WebRTC video track fed from the frame store
# ---------------------------------------------------------------------------

class SimulatorVideoTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, store: LatestFrame):
        super().__init__()
        self._store = store

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        img = self._store.frame
        if img is None:
            # Send a black frame until the simulator connects
            img = np.zeros((720, 1280, 3), dtype=np.uint8)

        frame = av.VideoFrame.from_ndarray(img, format="rgb24")
        frame.pts = pts
        frame.time_base = time_base
        return frame


# ---------------------------------------------------------------------------
# ZeroMQ subscriber (runs as a background asyncio task)
# ---------------------------------------------------------------------------

async def zmq_receiver(store: LatestFrame) -> None:
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
            store.frame = np.array(Image.open(BytesIO(jpeg_bytes)))
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


# ---------------------------------------------------------------------------
# WebSocket signaling handler
# ---------------------------------------------------------------------------

async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    print("[signaling] client connected")

    store: LatestFrame = request.app["frame_store"]
    cmd_sock = request.app["cmd_sock"]
    pc = RTCPeerConnection()

    @pc.on("connectionstatechange")
    async def on_connection_state_change():
        print(f"[webrtc] state → {pc.connectionState}")
        if pc.connectionState in ("failed", "closed"):
            await pc.close()

    pc.addTrack(SimulatorVideoTrack(store))

    # Wait for ICE gathering to complete so we can send a self-contained offer
    # (on localhost, host candidates are collected almost instantly)
    ice_done = asyncio.Event()

    @pc.on("icegatheringstatechange")
    async def on_ice_gathering_state_change():
        if pc.iceGatheringState == "complete":
            ice_done.set()

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)
    await asyncio.wait_for(ice_done.wait(), timeout=10.0)

    await ws.send_json({"type": "offer", "sdp": pc.localDescription.sdp})
    print("[signaling] offer sent")

    async for msg in ws:
        if msg.type == aiohttp.WSMsgType.TEXT:
            data = json.loads(msg.data)
            if data["type"] == "answer":
                await pc.setRemoteDescription(
                    RTCSessionDescription(sdp=data["sdp"], type="answer")
                )
                print("[signaling] answer received — WebRTC handshake complete")
            elif data["type"] == "move":
                try:
                    cmd_sock.send_json(
                        {"dx": data["dx"], "dy": data["dy"]},
                        zmq.NOBLOCK,
                    )
                except zmq.Again:
                    pass
        elif msg.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.ERROR):
            break

    await pc.close()
    print("[signaling] client disconnected")
    return ws


# ---------------------------------------------------------------------------
# App lifecycle
# ---------------------------------------------------------------------------

async def on_startup(app: web.Application) -> None:
    store = LatestFrame()
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
    app.router.add_get("/ws", websocket_handler)
    app.router.add_get("/", index_handler)
    app.router.add_get("/client.js", client_js_handler)
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)


if __name__ == "__main__":
    main()
