"""Wire protocol shared by simulator, teleop server and agent backends.

Avatar ids
----------
One id space covers every avatar in the scene:
  users   1 .. USER_COUNT          (browser clients, assigned by the teleop server)
  agents  AGENT_ID_BASE + board id (one presenter per poster board)

Simulator → teleop server (ZMQ PUB)
-----------------------------------
Frame: FRAME_HEADER (avatar id, timestamp ms) + JPEG bytes, one per user camera.

Teleop server → simulator (ZMQ PUSH)
------------------------------------
Move: {"avatar_id": int, "vx": ±1, "vy": ±1, "wz": ±1}
vx/vy are forward/left in the avatar's own frame, wz is yaw (counter-clockwise).
"""

import struct

USER_COUNT = 2
AGENT_ID_BASE = 100

FRAME_HEADER = struct.Struct("<BQ")  # one-byte avatar id + timestamp (ms)


def agent_avatar_id(board_id):
    return AGENT_ID_BASE + int(board_id)


def is_user_id(avatar_id):
    return 1 <= avatar_id <= USER_COUNT


def pack_frame(avatar_id, ts_ms, jpeg_bytes):
    return FRAME_HEADER.pack(avatar_id, ts_ms) + jpeg_bytes


def unpack_frame(data):
    """(avatar_id, ts_ms, jpeg_bytes), or None for a malformed message."""
    if len(data) <= FRAME_HEADER.size:
        return None
    avatar_id, ts_ms = FRAME_HEADER.unpack_from(data)
    return avatar_id, ts_ms, data[FRAME_HEADER.size:]


def move_command(avatar_id, vx=0.0, vy=0.0, wz=0.0):
    return {"avatar_id": int(avatar_id), "vx": vx, "vy": vy, "wz": wz}


def parse_move(msg):
    """(avatar_id, (vx, vy, wz)) clipped to [-1, 1], or None for a malformed message."""
    try:
        avatar_id = int(msg["avatar_id"])
        values = tuple(float(msg.get(key, 0.0)) for key in ("vx", "vy", "wz"))
    except (KeyError, TypeError, ValueError):
        return None
    return avatar_id, tuple(max(-1.0, min(1.0, v)) for v in values)
