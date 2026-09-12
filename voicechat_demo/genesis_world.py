"""The Genesis side. THIS IS THE HALF YOU OWN -- replace it with your own scene.

It holds the scene, the penguin avatars and one camera per participant. The
voice chat never sees any of it; the only things it asks for are:

    world.add(pid)          someone joined  -> give them an avatar
    world.remove(pid)       someone left
    world.position(pid)     -> (x, y, z)    <- this is all the voice chat needs

`set_keys` / `step` / `view` are just this demo's way of moving the penguins and
producing pictures. Your simulator will do those differently, and that is fine.

Genesis needs a GPU, same as simulator/main.py.
"""

import io
import math

import numpy as np
import trimesh
from PIL import Image

import genesis as gs

# --- penguin mesh ------------------------------------------------------------
# These four values were tuned against real renders of this exact mesh; they are
# the reason the penguin stands upright and looks where it walks. Re-tune them
# if you swap the model.
CHAR_SCALE = 0.5                       # mesh is 2.43 units tall -> ~1.2 units
CHAR_ZUP = False                       # the glb is Y-up; Genesis converts it
CHAR_BASE_QUAT = (1.0, 0.0, 0.0, 0.0)  # upright is baked into the geometry
HEADING_OFFSET = math.radians(-301.0)  # rotation that puts the beak on +x

# --- world -------------------------------------------------------------------
POOL = 8                # entities must exist before build(), so pre-make a pool
PARKED = (0.0, 0.0, -50.0)   # where unused penguins wait, out of every shot
HALF_W, HALF_D = 9.0, 6.0    # walkable area, world units
SPEED = 3.5                  # units per second
TICK_HZ = 20
CAM_BACK, CAM_UP = 4.2, 2.6  # chase camera offset


def _feet_offset(glb_path):
    """How high to lift the model so it stands on the floor.

    get_verts() is empty for a collision-less visual mesh, so measure it from
    the file instead. Genesis turns the Y-up glb into Z-up, so world z is the
    file's y.
    """
    mesh = trimesh.load(glb_path, force="mesh")
    return float(-(mesh.vertices[:, 1].min() * CHAR_SCALE))


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2)


def _facing(heading):
    c, s = math.cos(heading / 2), math.sin(heading / 2)
    return _qmul((c, 0.0, 0.0, s), CHAR_BASE_QUAT)


class GenesisWorld:
    def __init__(self, penguin_glb, pool=POOL, res=(640, 400), gpu=True):
        self.zoff = _feet_offset(penguin_glb)
        self.slots = {}          # participant id -> slot index
        self.free = list(range(pool))
        self.state = {}          # slot -> dict(x, y, heading, keys)

        gs.init(backend=gs.gpu if gpu else gs.cpu)
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=1.0 / 60.0),
            vis_options=gs.options.VisOptions(
                ambient_light=(0.55, 0.55, 0.55),
                lights=[{"type": "directional", "dir": (-0.3, 1.0, -0.7),
                         "color": (1.0, 1.0, 1.0), "intensity": 5.0}],
            ),
            show_viewer=False,
            renderer=gs.renderers.Rasterizer(),
        )
        self.scene.add_entity(gs.morphs.Plane())

        self.avatars, self.cams = [], []
        for _ in range(pool):
            self.avatars.append(self.scene.add_entity(
                gs.morphs.Mesh(file=penguin_glb, scale=CHAR_SCALE, pos=PARKED,
                               euler=(0.0, 0.0, 0.0), collision=False,
                               file_meshes_are_zup=CHAR_ZUP)))
            self.cams.append(self.scene.add_camera(
                res=res, pos=(0.0, -CAM_BACK, CAM_UP), lookat=(0.0, 0.0, 0.8),
                fov=48, GUI=False))

        self.scene.build()
        for entity in self.avatars:                       # park the whole pool
            entity.set_pos(PARKED, zero_velocity=True)
        print(f"[world] genesis ready, {pool} avatar slots, feet z={self.zoff:.3f}",
              flush=True)

    # --- what the voice chat uses --------------------------------------------
    def add(self, pid):
        """Give this participant an avatar. Returns False if the pool is full."""
        if not self.free:
            return False
        slot = self.free.pop(0)
        self.slots[pid] = slot
        self.state[slot] = {"x": np.random.uniform(-2.5, 2.5),
                            "y": np.random.uniform(-2.0, 2.0),
                            "heading": 0.0,
                            "keys": {"up": False, "down": False,
                                     "left": False, "right": False}}
        return True

    def remove(self, pid):
        slot = self.slots.pop(pid, None)
        if slot is None:
            return
        self.avatars[slot].set_pos(PARKED, zero_velocity=True)
        self.state.pop(slot, None)
        self.free.append(slot)

    def position(self, pid):
        """-> (x, y, z). The only thing the voice chat reads from this class."""
        slot = self.slots.get(pid)
        if slot is None:
            return (0.0, 0.0, 0.0)
        s = self.state[slot]
        return (s["x"], s["y"], 0.0)

    # --- this demo's movement and pictures -----------------------------------
    def set_keys(self, pid, keys):
        slot = self.slots.get(pid)
        if slot is not None:
            self.state[slot]["keys"] = {k: bool(keys.get(k)) for k in
                                        ("up", "down", "left", "right")}

    def step(self):
        dt = 1.0 / TICK_HZ
        for slot, s in self.state.items():
            k = s["keys"]
            dx = (1 if k["right"] else 0) - (1 if k["left"] else 0)
            dy = (1 if k["up"] else 0) - (1 if k["down"] else 0)
            if dx or dy:
                mag = math.hypot(dx, dy)
                s["x"] = min(max(s["x"] + dx / mag * SPEED * dt, -HALF_W), HALF_W)
                s["y"] = min(max(s["y"] + dy / mag * SPEED * dt, -HALF_D), HALF_D)
                s["heading"] = math.atan2(dy / mag, dx / mag) + HEADING_OFFSET

        self.scene.step()

        # A free rigid body drifts and tips, so re-place and re-orient every tick.
        for slot, s in self.state.items():
            self.avatars[slot].set_pos((s["x"], s["y"], self.zoff), zero_velocity=True)
            self.avatars[slot].set_quat(_facing(s["heading"]))
            self.cams[slot].set_pose(
                pos=(s["x"], s["y"] - CAM_BACK, CAM_UP),
                lookat=(s["x"], s["y"], 0.8),
            )

    def view(self, pid):
        """-> JPEG bytes of what this participant sees, or None."""
        slot = self.slots.get(pid)
        if slot is None:
            return None
        out = self.cams[slot].render()
        rgb = out[0] if isinstance(out, tuple) else out
        buf = io.BytesIO()
        Image.fromarray(np.asarray(rgb, dtype=np.uint8)).save(buf, "JPEG", quality=72)
        return buf.getvalue()
