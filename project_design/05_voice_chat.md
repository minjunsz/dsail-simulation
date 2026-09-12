# 05 — Multi-user text-to-speech voice chat

## Goal

Several people in one scene, each typing text that the others hear as speech,
in a voice of their own, quieter the further away the speaker is.

## Shape

Three pieces, and the point of the design is that the first two do not know
about each other:

```
voicechat/                     the voice half — no simulator anywhere in it
voicechat_demo/genesis_world.py    a simulator half — disposable example
voicechat_demo/main.py         the glue; the two meet in one line
```

```python
room.move(pid, *world.position(pid))
```

Verified by import graph: `voicechat/room.py` and `voicechat/protocol.py`
import only the standard library, `voicechat/tts.py` imports only Piper, and
nothing under `voicechat/` imports Genesis. The package imports and runs fine
in an environment where `genesis` is not installed.

## Interface the simulator must satisfy

```python
world.add(participant_id)        # someone joined -> give them an avatar
world.remove(participant_id)     # someone left
world.position(participant_id)   # -> (x, y, z)
```

`position()` is the only one the voice chat actually reads. It is used for one
thing: how loud each speaker is for each listener (full volume within 2 units,
silent past 12).

## TTS

Piper, `en_US-libritts_r-medium`: one 75 MB file holding 904 speakers, so each
joiner gets an audibly different voice without loading more models. CPU only,
~0.1 s per sentence, so it runs next to the simulator without touching the GPU.
Synthesis blocks, so it is called through `asyncio.to_thread`.

Three things were measured and fixed while building this:

- onnxruntime sizes its thread pool to every core on the machine. Inside an
  8-CPU SLURM allocation that produced ~90 failed-affinity warnings per start,
  and it was also slower: 8 threads take 0.12 s per sentence, 48 take 1.42 s.
  The session is now built explicitly, capped at 8 threads.
- `PiperVoice.load()` creates its own session before any of that can be set,
  so the session is constructed directly and handed to `PiperVoice(...)`.
- The first synthesis costs ~1.5 s of warm-up. It is now paid at startup
  instead of by the first person who talks (first utterance: 2.5 s -> 0.22 s).

## Wire format

Speech travels as one binary websocket frame:

```
[1 byte tag = 1][4 byte header length, big endian][UTF-8 JSON header][mono int16 PCM]
```

The header carries speaker, text, sample rate and that listener's gain.
`voicechat/protocol.py` and `voicechat/static/voicechat.js` implement the two
ends and must change together. Other tags are passed through to the
application, so one socket can also carry video or state.

## Verified

- Library alone, no server and no simulator: distinct voices, distance falloff.
- Three browsers in one room against Genesis on an L40: video streaming,
  arrow-key movement propagating through `room.move()`, all three receiving the
  same utterance, first-utterance latency 0.22 s and 0.07-0.17 s thereafter,
  no errors.

## Not done

- Distance falloff needs avatar positions. `voicechat_demo` has them directly;
  wiring it into `teleop_server` needs the simulator to publish positions
  alongside the JPEG frames (`FRAME_HEADER` `<BQ` -> `<BQfff`). Without it
  everyone hears everyone at full volume, which is a reasonable first step.
- Synthesis is serialized by a lock, so simultaneous speakers queue (~0.1 s
  each). Fine for a handful of people; a worker pool would be needed for many.
- Audio is sent as a whole clip rather than streamed, so a sentence is heard
  after it is fully synthesized.
- No rate limiting on submitted text.
