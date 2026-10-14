"""Conference hall layout (assets/conference_hall/layout.yaml) loader.

Kept free of Genesis/torch imports so the teleop server and agent backends can
reuse it (e.g. for proximity checks) without pulling in the simulator.
"""

import os

import numpy as np
import yaml

DEFAULT_LAYOUT = os.path.join(
    os.path.dirname(__file__), "..", "assets", "conference_hall", "layout.yaml"
)


def load_layout(path=DEFAULT_LAYOUT):
    path = os.path.abspath(path)
    with open(path) as f:
        layout = yaml.safe_load(f)
    root = os.path.dirname(path)
    layout["root"] = root
    layout["build_dir"] = os.path.join(root, "build")
    for board in layout["boards"]:
        if board.get("poster"):
            board["poster"] = os.path.join(root, board["poster"])
    return layout


def build_paths(layout):
    """Generated files consumed by the simulator (see scripts/make_conference_hall.py)."""
    build_dir = layout["build_dir"]
    return {
        "hall_mesh": os.path.join(build_dir, "hall.ply"),
        "poster_mesh": lambda board_id: os.path.join(build_dir, f"poster_{board_id}.obj"),
        "poster_texture": lambda board_id: os.path.join(build_dir, f"poster_{board_id}.png"),
    }


def board_axes(board):
    """Unit XY vectors: `normal` (poster facing) and `lateral` (normal rotated +90 deg)."""
    yaw = np.radians(board["yaw_deg"])
    normal = np.array((np.cos(yaw), np.sin(yaw)))
    lateral = np.array((-np.sin(yaw), np.cos(yaw)))
    return normal, lateral


def agent_pose(layout, board):
    """(xy, yaw_rad) of the agent presenting `board`."""
    normal, lateral = board_axes(board)
    offset = layout["agent"]
    xy = np.asarray(board["pos"], dtype=np.float64)
    xy = xy + offset["forward"] * normal + offset["lateral"] * lateral
    return xy, float(np.radians(board["yaw_deg"]))


def avatar_mesh(layout, rgb):
    """Vertex-colored cylinder (origin at its base) with a heading bar along +X."""
    import trimesh  # local import keeps the server-side loader light

    spec = layout["avatar"]
    radius, height, nose = spec["radius"], spec["height"], spec["nose"]
    body = trimesh.creation.cylinder(radius=radius, height=height, sections=32)
    body.apply_translation((0.0, 0.0, height / 2))
    body.visual.vertex_colors = np.tile((*rgb, 255), (len(body.vertices), 1)).astype(np.uint8)
    # Start slightly inside the surface so the bar reads as attached.
    bar_start = radius - 0.05
    bar = trimesh.creation.box(extents=(nose["length"] + 0.05, nose["thickness"], nose["thickness"]))
    bar.apply_translation((bar_start + (nose["length"] + 0.05) / 2, 0.0, nose["z"]))
    bar.visual.vertex_colors = np.tile((40, 40, 46, 255), (len(bar.vertices), 1)).astype(np.uint8)
    return trimesh.util.concatenate((body, bar))


def obstacles(layout):
    """Static 2D footprints that block avatars.

    `bounds` is the walkable interior (xmin, xmax, ymin, ymax); each box is
    (center_xy, half_extents along (lateral, normal), board). Avatars are not
    included: they move, so callers pass them to `blocked` each step.
    """
    size_x, size_y = layout["hall"]["size"]
    spec = layout["board"]
    panel_w = spec["poster_size"][0] + 2 * spec["frame_margin"]
    # The stand's feet (0.5 m deep) are wider than the panel itself.
    half = np.array((panel_w / 2, 0.25))
    return {
        "bounds": (-size_x / 2, size_x / 2, -size_y / 2, size_y / 2),
        "boxes": [(np.asarray(b["pos"], dtype=np.float64), half, b) for b in layout["boards"]],
    }


def blocked(obs, xy, radius, others=()):
    """True if a circle of `radius` at `xy` overlaps a wall, a board or any of
    the `others` circles, given as (center_xy, radius)."""
    xmin, xmax, ymin, ymax = obs["bounds"]
    if not (xmin + radius <= xy[0] <= xmax - radius and ymin + radius <= xy[1] <= ymax - radius):
        return True
    for center, half, board in obs["boxes"]:
        normal, lateral = board_axes(board)
        rel = xy - center
        local = np.array((rel @ lateral, rel @ normal))
        nearest = np.clip(local, -half, half)
        if np.sum((local - nearest) ** 2) < radius ** 2:
            return True
    for center, other_radius in others:
        if np.sum((xy - center) ** 2) < (radius + other_radius) ** 2:
            return True
    return False
