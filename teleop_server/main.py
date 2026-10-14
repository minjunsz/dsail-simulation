"""
Teleop server process.

- Subscribes to the simulator ZeroMQ PUB socket and caches the latest frame.
- Serves a browser client over HTTP.
- Handles WebRTC signaling over WebSocket (/ws) and streams video via aiortc.

Run (from project root):
    python -m teleop_server.main
"""

import argparse
import asyncio
import json
import pathlib
from io import BytesIO

import aiohttp
import numpy as np
import zmq.asyncio
from aiohttp import web
from PIL import Image

import av
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack

from common.protocol import USER_COUNT, is_user_id, move_command, unpack_frame

STATIC_DIR = pathlib.Path(__file__).parent.parent / "static"


# ---------------------------------------------------------------------------
# Frame store — a simple container; Python GIL makes single-ref assignment safe
# ---------------------------------------------------------------------------


class LatestFrame:
    def __init__(self) -> None:
        self.frame: np.ndarray | None = None


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


async def zmq_receiver(stores: list[LatestFrame], simulator_addr: str) -> None:
    ctx = zmq.asyncio.Context()
    sock = ctx.socket(zmq.SUB)
    # Keep a small queue containing both users. CONFLATE would retain only the
    # globally newest message and could permanently starve one user's stream.
    sock.setsockopt(zmq.RCVHWM, USER_COUNT * 2)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.connect(simulator_addr)
    print(f"[teleop] subscribed to simulator at {simulator_addr}")

    try:
        while True:
            frame = unpack_frame(await sock.recv())
            if frame is None or not is_user_id(frame[0]):
                continue

            user_id, _, jpeg_bytes = frame
            try:
                stores[user_id - 1].frame = np.array(
                    Image.open(BytesIO(jpeg_bytes)).convert("RGB")
                )
            except Exception as exc:
                print(f"[teleop] frame decode error for user {user_id}: {exc}")
    finally:
        sock.close()
        ctx.term()


# ---------------------------------------------------------------------------
# HTTP handlers
# ---------------------------------------------------------------------------


async def index_handler(request: web.Request) -> web.Response:
    return web.FileResponse(STATIC_DIR / "index.html")


async def client_js_handler(request: web.Request) -> web.Response:
    return web.FileResponse(
        STATIC_DIR / "client.js",
        headers={"Cache-Control": "no-store"},
    )


# ---------------------------------------------------------------------------
# WebSocket signaling handler
# ---------------------------------------------------------------------------


async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    available_user_ids: list[int] = request.app["available_user_ids"]
    if not available_user_ids:
        print("[signaling] client rejected: no user slot left")
        await ws.send_json({"type": "no_slot", "message": "No user slot left"})
        # Keep the connection open so the browser can display the terminal
        # state instead of entering its automatic reconnect loop.
        async for _ in ws:
            pass
        return ws

    # No await occurs between checking and popping, so asyncio handlers assign
    # the two slots atomically and in connection order.
    user_id = available_user_ids.pop(0)
    print(f"[signaling] client connected → user {user_id}")

    stores: list[LatestFrame] = request.app["frame_stores"]
    cmd_sock = request.app["cmd_sock"]
    pc = RTCPeerConnection()

    @pc.on("connectionstatechange")
    async def on_connection_state_change():
        print(f"[webrtc] user {user_id} state → {pc.connectionState}")
        if pc.connectionState == "failed":
            await pc.close()
            await ws.close(code=1011, message=b"WebRTC connection failed")

    pc.addTrack(SimulatorVideoTrack(stores[user_id - 1]))

    try:
        await ws.send_json({"type": "assigned", "avatar_id": user_id})

        # Wait for ICE gathering to complete so we can send a self-contained
        # offer (host candidates are collected almost instantly on localhost).
        ice_done = asyncio.Event()

        @pc.on("icegatheringstatechange")
        async def on_ice_gathering_state_change():
            if pc.iceGatheringState == "complete":
                ice_done.set()

        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await asyncio.wait_for(ice_done.wait(), timeout=10.0)

        await ws.send_json({"type": "offer", "sdp": pc.localDescription.sdp})
        print(f"[signaling] offer sent for user {user_id}")

        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                data = json.loads(msg.data)
                if data["type"] == "answer":
                    await pc.setRemoteDescription(
                        RTCSessionDescription(sdp=data["sdp"], type="answer")
                    )
                    print(
                        f"[signaling] user {user_id} answer received — "
                        "WebRTC handshake complete"
                    )
                elif data["type"] == "move":
                    try:
                        cmd_sock.send_json(
                            move_command(user_id, data["vx"], data["vy"], data.get("wz", 0)),
                            zmq.NOBLOCK,
                        )
                    except zmq.Again:
                        pass
            elif msg.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.ERROR):
                break
    finally:
        try:
            cmd_sock.send_json(move_command(user_id), zmq.NOBLOCK)
        except zmq.Again:
            pass
        await pc.close()
        available_user_ids.append(user_id)
        available_user_ids.sort()
        print(f"[signaling] client disconnected; user {user_id} released")

    return ws


# ---------------------------------------------------------------------------
# App lifecycle
# ---------------------------------------------------------------------------


async def on_startup(app: web.Application) -> None:
    args = app["cli_args"]
    simulator_addr = f"tcp://localhost:{args.sub_port}"
    cmd_addr = f"tcp://localhost:{args.cmd_port}"
    server_host = args.host
    server_port = args.http_port

    stores = [LatestFrame() for _ in range(USER_COUNT)]
    app["frame_stores"] = stores
    app["available_user_ids"] = list(range(1, USER_COUNT + 1))
    app["zmq_task"] = asyncio.create_task(zmq_receiver(stores, simulator_addr))

    cmd_ctx = zmq.Context()
    cmd_sock = cmd_ctx.socket(zmq.PUSH)
    cmd_sock.setsockopt(zmq.SNDHWM, 100)
    cmd_sock.connect(cmd_addr)
    app["cmd_sock"] = cmd_sock
    print(f"[teleop] listening on http://{server_host}:{server_port}")


async def on_shutdown(app: web.Application) -> None:
    app["zmq_task"].cancel()
    try:
        await app["zmq_task"]
    except asyncio.CancelledError:
        pass
    app["cmd_sock"].close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sub-port",
        type=int,
        default=5555,
        help="Simulator ZMQ PUB port to subscribe to",
    )
    parser.add_argument(
        "--cmd-port",
        type=int,
        default=5556,
        help="Simulator ZMQ PULL port for commands",
    )
    parser.add_argument("--http-port", type=int, default=8090, help="HTTP server port")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="HTTP server host")
    args = parser.parse_args()

    app = web.Application()
    app["cli_args"] = args
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/ws", websocket_handler)
    app.router.add_get("/", index_handler)
    app.router.add_get("/client.js", client_js_handler)
    web.run_app(app, host=args.host, port=args.http_port)


if __name__ == "__main__":
    main()
