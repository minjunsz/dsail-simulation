"""Multi-user text-to-speech voice chat for a shared space.

    from voicechat import TTSEngine, Room

    room = Room(TTSEngine("assets/voices/en_US-libritts_r-medium.onnx"))
    alice = room.join("alice")          # each joiner gets a different voice
    bob = room.join("bob")
    room.move(alice.id, x=1.0, y=0.0)   # your simulator tells us where they are

    speech = room.say(alice.id, "hello everyone")
    for listener, gain in room.audience(speech):
        send_to(listener, speech.pcm, gain)   # gain fades with distance
"""

from .protocol import TAG_SPEECH, decode_speech, encode_speech
from .room import FAR, NEAR, Participant, Room, Speech, hearing_gain
from .tts import TTSEngine

__all__ = [
    "TTSEngine",
    "Room", "Participant", "Speech", "hearing_gain", "NEAR", "FAR",
    "encode_speech", "decode_speech", "TAG_SPEECH",
]
