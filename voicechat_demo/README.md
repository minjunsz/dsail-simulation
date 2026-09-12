# voicechat_demo

A standalone demo of `voicechat/`, kept separate so the library can be read
without it. **It is meant to be deleted** once the voice chat is wired into
`simulator/` + `teleop_server/` — nothing in `voicechat/` imports it.

It exists to answer one question: *where exactly do I call this from?*

## Run

```bash
uv pip install -r requirements.txt
./assets/voices/download.sh
uv run python -m voicechat_demo.main --port 8181
```

Open the URL in two or three browser tabs — one tab is one penguin. Arrow keys
walk, type a line and press Enter. Needs a GPU, same as `simulator/main.py`.

## What is in here

| file | |
|---|---|
| `genesis_world.py` | a Genesis scene: ground plane, a pool of penguin avatars, one chase camera each |
| `main.py` | the glue — an aiohttp server that steps the world, streams frames and carries the voice chat |
| `static/client.html`, `static/client.js` | the demo page |

## The only thing worth copying

`main.py` connects the two halves in one line, in the tick loop:

```python
world.step()
room.move(pid, *world.position(pid))      # <- the entire seam
```

and when someone submits text:

```python
speech = await asyncio.to_thread(room.say, pid, text)   # blocking, ~0.1 s
for listener, gain in room.audience(speech):
    await send_bytes(sock_of[listener.id], encode_speech(speech, gain))
```

## Wiring it into this repo instead

`teleop_server/main.py` already has everything needed:

- it assigns each client a `robot_id` on connect → that is the participant id,
  so `room.join()` / `room.leave()` go where robot ids are taken and released
- it already holds a websocket per client → speech frames ride on it, no new
  connection
- `simulator/main.py` already reads `robot.base_link.get_pos()` for the chase
  camera → publishing it alongside the JPEG (widening `FRAME_HEADER` from
  `<BQ` to `<BQfff`) is what lets `room.move()` do distance falloff

Skipping the last point is fine to start with; without positions everyone
simply hears everyone at full volume.

## Avatar pool

Genesis compiles the scene in `build()`, so entities cannot be added
afterwards. `POOL` penguins are created up front and unused ones are parked
below the floor. That caps concurrent participants at `POOL` (8).
