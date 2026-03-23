"""
Genesis simulator process.

Initializes a headless scene with a ground plane, renders at TARGET_FPS,
and publishes JPEG-compressed frames over ZeroMQ PUB socket.

Run:
    python -m simulator.main
"""

import struct
import time
from io import BytesIO

import zmq
from PIL import Image

import genesis as gs

PUBLISH_ADDR = "tcp://*:5555"
TARGET_FPS = 30
CAM_RES = (1280, 720)


def main():
    # --- Genesis setup ---
    gs.init(backend=gs.gpu)

    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
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
    sock.bind(PUBLISH_ADDR)
    print(f"[simulator] publishing frames on {PUBLISH_ADDR}")

    interval = 1.0 / TARGET_FPS

    while True:
        t0 = time.perf_counter()

        scene.step()
        rgb, *_ = cam.render()  # returns (rgb, depth, seg, normal); rgb is np.ndarray (H, W, 3) uint8

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
