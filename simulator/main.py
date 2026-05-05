"""
Genesis simulator process.

Initializes a headless scene with a ground plane, renders at TARGET_FPS,
and publishes JPEG-compressed frames over ZeroMQ PUB socket.

Run:
    python -m simulator.main
"""

import argparse
import struct
import time
from io import BytesIO

import zmq
from PIL import Image

import genesis as gs

TARGET_FPS = 30
CAM_RES = (1280, 720)
MOVE_SPEED = 0.05


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
    cam = scene.add_camera(
        res=CAM_RES,
        pos=(3.5, 0.0, 2.5),
        lookat=(0.0, 0.0, 0.5),
        fov=30,
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
        except zmq.Again:
            pass  # 명령 없으면 현 위치 유지

        # 2. physics step
        scene.step()
        rgb, *_ = (
            cam.render()
        )  # returns (rgb, depth, seg, normal); rgb is np.ndarray (H, W, 3) uint8

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
