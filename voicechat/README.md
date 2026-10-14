# voicechat

Multi-user text-to-speech chat for a shared scene. Each participant types a
line, it comes out as speech in their own voice, and everybody close enough
hears it — quieter the further away they are.

**This package does not know what a simulator is.** `room.py` and
`protocol.py` import nothing but the standard library; `tts.py` imports only
Piper. There is no Genesis, torch or ZeroMQ anywhere in here, so it drops into
whatever process you already run.

## Install

```bash
uv pip install piper-tts          # the only new dependency
./assets/voices/download.sh       # 75 MB voice model, 904 speakers
```

## Use it

```python
from voicechat import Room, TTSEngine

room = Room(TTSEngine("assets/voices/en_US-libritts_r-medium.onnx"))

alice = room.join("alice")            # -> Participant(id, name, voice, x, y, z)
room.move(alice.id, x=3.0, y=1.0)     # whenever your simulator moves that avatar

speech = room.say(alice.id, "hello")  # -> Speech(pcm, sample_rate, speaker, x, y, z)
for listener, gain in room.audience(speech):
    send_to(listener, speech.pcm, gain)
```

That is the whole API.

| call | in | out |
|---|---|---|
| `room.join(name)` | a display name | `Participant`, with a voice already assigned |
| `room.move(id, x, y, z)` | a position | nothing — it just records it |
| `room.say(id, text)` | who + what they typed | `Speech`, or `None` if there was nothing to say |
| `room.audience(speech)` | a `Speech` | `[(listener, gain), ...]`, gain 1.0 up close to 0.0 at 12 units |
| `tts.synth(text, voice)` | text + voice number | `bytes` — mono 16-bit PCM |

`speech.pcm` is mono 16-bit PCM at 22050 Hz.

## What you have to provide

Positions. That is it. Call `room.move()` whenever an avatar moves — the voice
chat uses it for exactly one thing, deciding how loud each speaker is for each
listener. If you do not care about distance, never call it and everyone hears
everyone at full volume.

## The browser half

`static/voicechat.js` is the other half of this library: it owns the
websocket, plays the audio it receives at the right volume, and knows nothing
about rendering.

```js
const chat = new VoiceChat({ onSaid: (id, name, text) => showBubble(name, text) });
chat.connect("alice");
chat.say("hello");
chat.send({ t: "move", ... });          // your own messages pass straight through
```

Binary frames tagged `1` are speech and are handled internally; every other tag
is handed to your `onBinary(tag, payload)` callback, so you can share one socket
with your own video or state traffic.

## Notes

- **Voices.** Assigned automatically on join. The model holds 904 speakers and
  joiners are spread across them, so two people never sound alike. English.
- **Speed.** ~0.1 s of CPU per sentence. `synth()` blocks, so call it off the
  event loop (`asyncio.to_thread`). No GPU.
- **Distance.** Full volume within 2 units, silent past 12 — `NEAR` and `FAR`
  at the top of `room.py`.
- **Swapping the engine.** Rewrite `tts.py`, keep `sample_rate`, `voice_count`
  and `synth(text, voice) -> bytes`. Nothing else imports Piper.
- **`protocol.py` and `static/voicechat.js` are a pair.** They define the same
  frame layout on each side; change one and you must change the other.
