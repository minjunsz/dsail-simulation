"""How a Speech travels over the network.

Binary frame sent server -> client:

    [1 byte tag][4 byte header length, big endian][UTF-8 JSON header][raw PCM]

The header carries who spoke, the text, the sample rate and the listener's gain;
the body is mono int16 PCM. Everything else on the socket is plain JSON text.

Only server.py imports this. If you move the audio over your own transport,
delete this file.
"""

import json
import struct

TAG_SPEECH = 1
_HEAD = ">BI"
_HEAD_SIZE = struct.calcsize(_HEAD)


def encode_speech(speech, gain=1.0):
    """in : a Speech + how loud it should be for this listener -> out: bytes"""
    header = json.dumps({
        "speaker_id": speech.speaker_id,
        "speaker_name": speech.speaker_name,
        "text": speech.text,
        "sample_rate": speech.sample_rate,
        "gain": round(gain, 4),
        "x": speech.x, "y": speech.y, "z": speech.z,
    }).encode("utf-8")
    return struct.pack(_HEAD, TAG_SPEECH, len(header)) + header + speech.pcm


def decode_speech(frame):
    """in : bytes -> out: (header dict, pcm bytes)"""
    tag, size = struct.unpack(_HEAD, frame[:_HEAD_SIZE])
    if tag != TAG_SPEECH:
        raise ValueError(f"unknown frame tag {tag}")
    head = json.loads(frame[_HEAD_SIZE:_HEAD_SIZE + size].decode("utf-8"))
    return head, frame[_HEAD_SIZE + size:]
