"""
Isaac Sim simulator process.

Initializes a headless scene with a ground plane and a kinematic box,
renders at TARGET_FPS, and publishes JPEG-compressed frames over ZeroMQ PUB socket.
Receives WASD movement commands over ZeroMQ PULL socket.

Run:
    CUDA_VISIBLE_DEVICES=2 uv run python -m simulator.main
"""

import struct
import time
from io import BytesIO

# SimulationApp MUST be created before any omni.* imports
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "renderer": "RasterizationLighting"})

import numpy as np
import zmq
from PIL import Image

from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid
import omni.replicator.core as rep

PUBLISH_ADDR = "tcp://*:5555"
CMD_ADDR = "tcp://*:5556"
TARGET_FPS = 30
CAM_RES = (1280, 720)
MOVE_SPEED = 0.05


def main():
    # --- Isaac Sim setup ---
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    box = world.scene.add(
        VisualCuboid(
            prim_path="/World/box",
            name="box",
            position=np.array([0.0, 0.0, 0.5]),
            size=0.3,
        )
    )

    world.reset()

    # --- Camera / Replicator setup ---
    camera = rep.create.camera(
        position=(3.5, 0.0, 2.5),
        look_at=(0.0, 0.0, 0.5),
        focal_length=18.0,
    )
    render_product = rep.create.render_product(camera, resolution=CAM_RES)
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach([render_product])

    # Warm up the renderer (first few frames can be black)
    for _ in range(10):
        world.step(render=True)

    # --- ZeroMQ setup ---
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 2)  # drop old frames; always send latest
    sock.bind(PUBLISH_ADDR)
    print(f"[simulator] publishing frames on {PUBLISH_ADDR}")

    cmd_sock = ctx.socket(zmq.PULL)
    cmd_sock.setsockopt(zmq.RCVHWM, 1)
    cmd_sock.bind(CMD_ADDR)
    print(f"[simulator] receiving commands on {CMD_ADDR}")

    interval = 1.0 / TARGET_FPS
    latest_jpeg: bytes | None = None

    try:
        while simulation_app.is_running():
            t0 = time.perf_counter()

            # 1. 명령 polling
            try:
                msg = cmd_sock.recv_json(zmq.NOBLOCK)
                position, orientation = box.get_world_pose()
                position[0] += msg["dx"] * MOVE_SPEED
                position[1] += msg["dy"] * MOVE_SPEED
                box.set_world_pose(position=position, orientation=orientation)
            except zmq.Again:
                pass  # 명령 없으면 현 위치 유지

            # 2. Physics + render step
            world.step(render=True)
            rep.orchestrator.step(rt_subframes=1)

            # 3. Capture frame (RGBA → RGB)
            # In Isaac Sim 5.x, get_data() returns np.ndarray directly (H, W, 4) RGBA
            rgb_data = rgb_annotator.get_data()
            if rgb_data is not None and rgb_data.size > 0:
                frame = rgb_data[:, :, :3]  # RGBA → RGB

                buf = BytesIO()
                Image.fromarray(frame).save(buf, format="JPEG", quality=85)
                latest_jpeg = buf.getvalue()

            if latest_jpeg is not None:
                ts_ms = int(time.time() * 1000)
                try:
                    sock.send(struct.pack("<Q", ts_ms) + latest_jpeg, zmq.NOBLOCK)
                except zmq.Again:
                    pass  # subscriber too slow; drop frame

            elapsed = time.perf_counter() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    except Exception as e:
        print(f"[simulator] exception: {e}")
        import traceback
        traceback.print_exc()
        raise
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
