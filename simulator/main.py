"""
Genesis simulator process.

Initializes a headless scene with a ground plane, renders at TARGET_FPS,
and publishes JPEG-compressed frames over ZeroMQ PUB socket.

Run:
    python -m simulator.main
"""

import argparse
import math
import struct
import time
from io import BytesIO

import torch
import zmq
from PIL import Image

import genesis as gs

TARGET_FPS = 30
CAM_RES = (1280, 720)
MOVE_SPEED = 0.05
ROT_SPEED = 0.05


def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    return torch.stack(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pub-port", type=int, default=5555, help="ZMQ PUB port")
    parser.add_argument(
        "--cmd-port", type=int, default=5556, help="ZMQ PULL (command) port"
    )
    args = parser.parse_args()

    publish_addr = f"tcp://*:{args.pub_port}"
    cmd_addr = f"tcp://*:{args.cmd_port}"

    # --- Genesis setup ---
    gs.init(backend=gs.gpu)

    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    box = scene.add_entity(gs.morphs.Box(size=(0.3, 0.3, 0.3), pos=(0.0, 0.0, 0.5)))
    cam = scene.add_sensor(
        gs.sensors.RasterizerCameraOptions(
            res=CAM_RES,
            pos=(0.0, 0.2, 3),
            lookat=(0.0, 0.0, 0.0),
            fov=30,
            entity_idx=box.idx,
        )
    )
    scene.build()

    # --- ZeroMQ setup ---
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 2)  # drop old frames; always send latest
    sock.bind(publish_addr)
    print(f"[simulator] publishing frames on {publish_addr}")

    cmd_sock = ctx.socket(zmq.PULL)
    cmd_sock.setsockopt(zmq.RCVHWM, 1)
    cmd_sock.bind(cmd_addr)
    print(f"[simulator] receiving commands on {cmd_addr}")

    interval = 1.0 / TARGET_FPS

    while True:
        t0 = time.perf_counter()

        # 1. 명령 polling
        try:
            msg = cmd_sock.recv_json(zmq.NOBLOCK)
            pos = box.get_pos()
            pos[0] += msg["dx"] * MOVE_SPEED
            pos[1] += msg["dy"] * MOVE_SPEED
            box.set_pos(pos)
            dz = msg.get("dz", 0)
            if dz != 0:
                quat = box.get_quat()
                half = math.cos(ROT_SPEED * dz / 2)
                sin_half = math.sin(ROT_SPEED * dz / 2)
                qz = torch.tensor([half, 0.0, 0.0, sin_half], device=quat.device)
                quat = quat_mult(qz, quat)
                norm = torch.norm(quat)
                if norm > 0:
                    quat = quat / norm
                box.set_quat(quat)
        except zmq.Again:
            pass  # 명령 없으면 현 위치 유지

        # 2. physics step
        scene.step()
        rgb = cam.read().rgb.cpu().numpy()

        # JPEG compress
        buf = BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=85)
        jpeg_bytes = buf.getvalue()

        # Message: 8-byte little-endian timestamp (ms) + JPEG bytes
        ts_ms = int(time.time() * 1000)
        try:
            sock.send(struct.pack("<Q", ts_ms) + jpeg_bytes, zmq.NOBLOCK)
        except zmq.Again:
            pass  # subscriber too slow; drop frame

        elapsed = time.perf_counter() - t0
        sleep_time = interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
