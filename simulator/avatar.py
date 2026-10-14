"""Avatars: cylinder bodies with a heading bar, moved kinematically by commands.

Avatar holds the body (pose, latest command, collision footprint); User adds
the TPS camera streamed to its browser; Agent adds the poster board it presents
and a per-view highlight tint.
Agent behaviour (listening, answering) lives in the agent backend process, which
drives the body through the same move commands as a user.
"""

import os
from io import BytesIO

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import genesis as gs

from common.hall_layout import agent_pose, avatar_mesh
from common.protocol import agent_avatar_id

LIN_SPEED       = 1.0   # m/s at full command
YAW_RATE        = 1.2   # rad/s at full command
COMMAND_TIMEOUT = 0.5   # s without a command → stop (teleop heartbeat is 100 ms)
AVATAR_Z        = 0.01  # hall floor top

CAM_RES           = (1280, 720)
CAMERA_POS_REL    = np.array((-3.0, 0.0, 2.0), dtype=np.float32)
CAMERA_LOOKAT_REL = np.array((0.0, 0.0, 1.0), dtype=np.float32)
CAMERA_UP_REL     = np.array((0.0, 0.0, 1.0), dtype=np.float32)
CAMERA_SMOOTHING  = 0.85
CAM_FOV_DEG       = 45      # vertical
JPEG_QUALITY      = 85

NAME_TAG_FONT = "/usr/share/fonts/truetype/nanum/NanumSquareRoundB.ttf"  # Hangul-capable
NAME_TAG_SIZE = 22
NAME_TAG_GAP  = 0.25        # m above the head
NAME_TAG_BG = (20, 20, 24, 170)
NAME_TAG_BG_HIGHLIGHT = (40, 150, 60, 210)


def yaw_quat(yaw):
    return (float(np.cos(yaw / 2.0)), 0.0, 0.0, float(np.sin(yaw / 2.0)))


def yaw_rotation(yaw):
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    return np.array(((cos_yaw, -sin_yaw, 0.0), (sin_yaw, cos_yaw, 0.0), (0.0, 0.0, 1.0)), dtype=np.float32)


class Avatar:
    def __init__(self, scene, layout, avatar_id, xy, yaw, rgb, name):
        self.avatar_id = avatar_id
        self.name = name
        self.tag_z = AVATAR_Z + layout["avatar"]["height"] + NAME_TAG_GAP
        self.xy = np.asarray(xy, dtype=np.float64)
        self.yaw = float(yaw)
        self.radius = layout["avatar"]["radius"]
        self.command = np.zeros(3)
        self.last_command_at = 0.0
        self.entity = scene.add_entity(
            gs.morphs.MeshSet(
                files=(avatar_mesh(layout, rgb),),
                pos=(*self.xy, AVATAR_Z),
                quat=yaw_quat(self.yaw),
                collision=False,
                decimate=False,
                convexify=False,
            )
        )

    @property
    def footprint(self):
        return self.xy, self.radius

    @property
    def tag_anchor(self):
        """World point the name tag is drawn at."""
        return np.array((*self.xy, self.tag_z))

    def set_command(self, command, now):
        self.command = np.asarray(command, dtype=np.float64)
        self.last_command_at = now

    def step(self, hall, others, dt, now):
        """Integrate the command for `dt`; slide along obstacles one axis at a time."""
        # A sender that dies without its final zero command must not leave the
        # avatar moving forever.
        if now - self.last_command_at > COMMAND_TIMEOUT:
            self.command = np.zeros(3)

        vx, vy, wz = self.command
        if not self.command.any():
            return  # idle (most agents): skip the entity update
        self.yaw += wz * YAW_RATE * dt
        cos_yaw, sin_yaw = np.cos(self.yaw), np.sin(self.yaw)
        delta = np.array((cos_yaw * vx - sin_yaw * vy, sin_yaw * vx + cos_yaw * vy)) * LIN_SPEED * dt
        if delta.any():
            for move in (delta, (delta[0], 0.0), (0.0, delta[1])):
                candidate = self.xy + np.asarray(move)
                if not hall.blocked(candidate, self.radius, others):
                    self.xy = candidate
                    break

        self.entity.set_pos((*self.xy, AVATAR_Z))
        self.entity.set_quat(yaw_quat(self.yaw))


class User(Avatar):
    """Browser-controlled avatar with a third-person camera following it."""

    def __init__(self, scene, layout, avatar_id, xy, yaw):
        rgb = layout["avatar"]["colors"]["users"][avatar_id - 1]
        name = layout["avatar"]["user_names"][avatar_id - 1]
        super().__init__(scene, layout, avatar_id, xy, yaw, rgb, name)
        # Genesis' default far plane (20 m) clips the hall's far end (21 m + 3 m TPS offset).
        self.camera = scene.add_camera(res=CAM_RES, pos=(0.0, 0.0, 2.0), lookat=(1.0, 0.0, 1.0),
                                       fov=CAM_FOV_DEG, far=40.0, GUI=False)
        self._camera_state = {}
        self._camera_pose = None

    def update_camera(self):
        """Follow from behind, smoothing position and heading."""
        anchor = np.array((*self.xy, AVATAR_Z))
        state = self._camera_state
        if not state:
            state["anchor"] = anchor
            state["yaw"] = self.yaw
        else:
            alpha = CAMERA_SMOOTHING
            state["anchor"] = alpha * state["anchor"] + (1.0 - alpha) * anchor
            yaw_delta = np.arctan2(np.sin(self.yaw - state["yaw"]), np.cos(self.yaw - state["yaw"]))
            state["yaw"] += (1.0 - alpha) * yaw_delta

        rotation = yaw_rotation(state["yaw"])
        pos = state["anchor"] + rotation @ CAMERA_POS_REL
        lookat = state["anchor"] + rotation @ CAMERA_LOOKAT_REL
        self.camera.set_pose(pos=pos, lookat=lookat, up=CAMERA_UP_REL)
        self._camera_pose = (pos, lookat)

    def render_jpeg(self, name_tags=()):
        """Render, draw `name_tags` ((world_xyz, text, highlighted) tuples), encode."""
        rgb, _, _, _ = self.camera.render()
        image = Image.fromarray(rgb)
        if name_tags and self._camera_pose is not None:
            draw_name_tags(image, self._camera_pose, name_tags)
        buf = BytesIO()
        image.save(buf, format="JPEG", quality=JPEG_QUALITY)
        return buf.getvalue()


class Agent(Avatar):
    """Presenter standing beside its poster board.

    The body mesh is white and its role color lives in the render material's
    base color factor (which multiplies the vertex colors), so the color can be
    swapped right before each user's camera renders: Genesis only refreshes its
    own visual state once per scene step, which cannot express per-viewer colors.

    The rasterizer's JIT renderer copies each material's factor into
    `jit.pbr_mat` when it (re)builds its primitive list and draws from that
    copy, so the tint is written to both: the baked row for the next draw, and
    the material so a later rebuild keeps it.
    """

    def __init__(self, scene, layout, board):
        xy, yaw = agent_pose(layout, board)
        default_name = "Author" if board["role"] == "author" else f"Agent {board['id']}"
        super().__init__(scene, layout, agent_avatar_id(board["id"]), xy, yaw, (255, 255, 255),
                         board.get("name", default_name))
        self.board = board
        self.role = board["role"]
        self._base_factor = _color_factor(layout["avatar"]["colors"][self.role])
        self._highlight_factor = _color_factor(layout["gaze"]["highlight"])
        self._context = None
        self._primitives = ()
        self._jit_rows = []
        self._jit_rows_for = None
        self._highlighted = None

    def bind_materials(self, scene):
        """Find this agent's render primitives; call once after scene.build()."""
        self._context = scene.visualizer.context
        nodes = self._context.rigid_nodes
        self._primitives = tuple(
            primitive for geom in self.entity.vgeoms for primitive in nodes[geom.uid].mesh.primitives
        )
        self.set_highlight(False)

    @property
    def highlighted(self):
        return bool(self._highlighted)

    def set_highlight(self, highlighted):
        if highlighted == self._highlighted:
            return
        factor = self._highlight_factor if highlighted else self._base_factor
        for primitive in self._primitives:
            primitive.material.baseColorFactor = factor
        jit = self._context.jit
        if jit.primitive_list is not self._jit_rows_for:  # rebuilt (or first built) since last lookup
            mine = {id(primitive) for primitive in self._primitives}
            self._jit_rows = [i for i, primitive in enumerate(jit.primitive_list) if id(primitive) in mine]
            self._jit_rows_for = jit.primitive_list
        jit.pbr_mat[self._jit_rows, :4] = factor
        self._highlighted = highlighted


def _load_tag_font():
    if os.path.exists(NAME_TAG_FONT):
        return ImageFont.truetype(NAME_TAG_FONT, NAME_TAG_SIZE)
    return ImageFont.load_default(size=NAME_TAG_SIZE)


_TAG_FONT = None


def draw_name_tags(image, camera_pose, name_tags):
    """Project each tag's world point through the TPS camera and draw a label.

    Tags behind the camera or off-screen are skipped; occlusion is not checked.
    """
    global _TAG_FONT
    if _TAG_FONT is None:
        _TAG_FONT = _load_tag_font()
    pos, lookat = (np.asarray(v, dtype=np.float64) for v in camera_pose)
    # Camera frame as Genesis/OpenGL builds it: looks along -z, y up.
    z = pos - lookat
    z /= np.linalg.norm(z)
    x = np.cross(CAMERA_UP_REL, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    width, height = image.size
    focal = (height / 2) / np.tan(np.radians(CAM_FOV_DEG) / 2)

    projected = []
    for point, text, highlighted in name_tags:
        rel = np.asarray(point) - pos
        depth = -(rel @ z)
        if depth >= 0.2:
            projected.append((depth, rel, text, highlighted))
    # Far tags first so nearer ones are drawn on top where they overlap.
    projected.sort(key=lambda tag: -tag[0])

    draw = ImageDraw.Draw(image, "RGBA")
    for depth, rel, text, highlighted in projected:
        u = width / 2 + focal * (rel @ x) / depth
        v = height / 2 - focal * (rel @ y) / depth
        left, top, right, bottom = draw.textbbox((u, v), text, font=_TAG_FONT, anchor="mb")
        if right < 0 or left > width or bottom < 0 or top > height:
            continue
        pad = 5
        draw.rounded_rectangle((left - pad, top - pad, right + pad, bottom + pad), radius=6,
                               fill=NAME_TAG_BG_HIGHLIGHT if highlighted else NAME_TAG_BG)
        draw.text((u, v), text, font=_TAG_FONT, anchor="mb", fill=(255, 255, 255))


def _color_factor(rgb):
    return np.array((*(c / 255.0 for c in rgb), 1.0), dtype=np.float32)
