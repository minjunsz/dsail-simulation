"""The glue: one websocket server wiring the simulator to the voice chat.

    world  (voicechat_demo/genesis_world.py)  -- avatars, movement, pictures
    room   (voicechat/room.py)                -- who is here, voices, who hears what

Per tick it does exactly this:

    world.step()
    room.move(pid, *world.position(pid))     <- the whole connection, one line
    send each client its camera frame

and when someone types a line:

    speech = room.say(pid, text)
    for listener, gain in room.audience(speech): send them speech.pcm at `gain`

Run (from the repo root, on a machine with a GPU):

    uv run python -m voicechat_demo.main --port 8181
"""

import argparse
import asyncio
import json
import os
import struct

from aiohttp import WSMsgType, web

from voicechat import Room, TTSEngine, encode_speech

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_MODEL = os.path.join(ROOT, "assets", "voices", "en_US-libritts_r-medium.onnx")
DEFAULT_GLB = os.path.join(ROOT, "assets", "meshes", "penguin_plush.glb")
WEB_DIR = os.path.join(HERE, "static")
LIB_JS = os.path.join(ROOT, "voicechat", "static", "voicechat.js")

TICK_HZ = 20
TAG_FRAME = 0          # [0][jpeg]                    -- this file's own format
TAG_SPEECH = 1         # see voicechat/protocol.py    -- the voice chat's format
MAX_FRAME = 32 * 1024 * 1024


class Hub:
    def __init__(self, world, room):
        self.world = world
        self.room = room
        self.sockets = {}        # websocket -> participant id

    async def send_json(self, ws, obj):
        try:
            await ws.send_str(json.dumps(obj))
        except Exception:
            self.sockets.pop(ws, None)

    async def send_bytes(self, ws, payload):
        try:
            await ws.send_bytes(payload)
        except Exception:
            self.sockets.pop(ws, None)

    async def broadcast_people(self):
        people = self.room.snapshot()
        await asyncio.gather(*[self.send_json(ws, {"t": "people", "people": people})
                               for ws in list(self.sockets)])

    async def deliver(self, speech):
        """Push one utterance to everyone close enough to hear it."""
        by_id = {pid: ws for ws, pid in self.sockets.items()}
        for listener, gain in self.room.audience(speech):
            ws = by_id.get(listener.id)
            if ws is not None:
                await self.send_bytes(ws, encode_speech(speech, gain))


async def ws_handler(request):
    hub = request.app["hub"]
    ws = web.WebSocketResponse(max_msg_size=MAX_FRAME)
    await ws.prepare(request)
    me = None

    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            data = json.loads(msg.data)
            kind = data.get("t")

            if kind == "join" and me is None:
                me = hub.room.join(data.get("name"))
                if not hub.world.add(me.id):                  # pool full
                    hub.room.leave(me.id)
                    await hub.send_json(ws, {"t": "full"})
                    me = None
                    continue
                hub.sockets[ws] = me.id
                await hub.send_json(ws, {"t": "welcome",
                                         "you": {"id": me.id, "name": me.name,
                                                 "voice": me.voice}})
                await hub.broadcast_people()

            elif kind == "keys" and me is not None:
                hub.world.set_keys(me.id, data.get("keys") or {})

            elif kind == "say" and me is not None:
                text = (data.get("text") or "").strip()
                if text:
                    await asyncio.gather(*[
                        hub.send_json(peer, {"t": "said", "speaker_id": me.id,
                                             "speaker_name": me.name, "text": text})
                        for peer in list(hub.sockets)])
                    # TTS is blocking CPU work -- keep it off the event loop.
                    speech = await asyncio.to_thread(hub.room.say, me.id, text)
                    if speech:
                        await hub.deliver(speech)
    finally:
        hub.sockets.pop(ws, None)
        if me is not None:
            hub.world.remove(me.id)
            hub.room.leave(me.id)
            await hub.broadcast_people()
    return ws


async def tick_loop(app):
    """Advance the simulation, tell the room where everyone is, ship the frames."""
    hub = app["hub"]
    period = 1.0 / TICK_HZ
    try:
        while True:
            await asyncio.sleep(period)
            if not hub.sockets:
                continue

            hub.world.step()
            for ws, pid in list(hub.sockets.items()):
                hub.room.move(pid, *hub.world.position(pid))   # <-- the seam
            await hub.broadcast_people()

            for ws, pid in list(hub.sockets.items()):
                jpeg = hub.world.view(pid)
                if jpeg:
                    await hub.send_bytes(ws, struct.pack("B", TAG_FRAME) + jpeg)
    except asyncio.CancelledError:
        pass


def make_app(world, model_path=DEFAULT_MODEL, web_dir=WEB_DIR):
    engine = TTSEngine(model_path)
    print(f"[tts]   {os.path.basename(model_path)}  "
          f"{engine.sample_rate} Hz, {engine.voice_count} voices", flush=True)

    app = web.Application()
    app["hub"] = Hub(world, Room(engine))

    async def index(request):
        return web.FileResponse(os.path.join(web_dir, "client.html"),
                                headers={"Content-Type": "text/html; charset=utf-8"})

    async def library_js(request):
        return web.FileResponse(LIB_JS, headers={"Content-Type": "text/javascript"})

    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/voicechat.js", library_js)   # the library's browser half
    app.router.add_static("/static/", web_dir)

    async def _start(a):
        a["tick"] = asyncio.create_task(tick_loop(a))

    async def _stop(a):
        a["tick"].cancel()
        await asyncio.gather(a["tick"], return_exceptions=True)

    app.on_startup.append(_start)
    app.on_cleanup.append(_stop)
    return app


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8181)
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--mesh", default=DEFAULT_GLB)
    ap.add_argument("--cpu", action="store_true", help="Genesis on CPU (no rendering)")
    args = ap.parse_args()

    from voicechat_demo.genesis_world import GenesisWorld
    world = GenesisWorld(args.mesh, gpu=not args.cpu)

    print(f"[open]  http://localhost:{args.port}   "
          f"(open it in several tabs for several penguins)", flush=True)
    web.run_app(make_app(world, args.model), host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
