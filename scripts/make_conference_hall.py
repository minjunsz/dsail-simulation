"""Build the conference hall scene from assets/conference_hall/layout.yaml.

Writes into assets/conference_hall/build/:
  hall.ply          vertex-colored floor, walls and poster board stands (visual + collision)
  poster_<id>.obj   one UV-mapped quad per board, already in world coordinates
  poster_<id>.png   its texture (RGB, downscaled; numbered placeholder if the board has no poster)

Run, then launch the simulator with --layout:

    uv run python scripts/make_conference_hall.py
    python -m simulator.main
"""
import argparse
import os
import sys

import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from common.hall_layout import DEFAULT_LAYOUT, board_axes, build_paths, load_layout  # noqa: E402

TEXTURE_MAX_WIDTH = 2048
POSTER_FRONT_OFFSET = 0.003  # in front of the panel to avoid z-fighting

FLOOR_RGB = (120, 52, 52)    # carpet
WALL_RGB = (214, 208, 196)
PANEL_RGB = (60, 60, 64)
LEG_RGB = (150, 150, 155)


def colored_box(extents, center, rgb, yaw=0.0):
    box = trimesh.creation.box(extents=extents)
    box.apply_transform(trimesh.transformations.rotation_matrix(yaw, (0, 0, 1)))
    box.apply_translation(center)
    box.visual.vertex_colors = np.tile((*rgb, 255), (len(box.vertices), 1)).astype(np.uint8)
    return box


def build_hall(layout):
    hall = layout["hall"]
    size_x, size_y = hall["size"]
    height, thick = hall["wall_height"], hall["wall_thickness"]
    parts = [
        # Floor top sits 1 cm above the ground plane to hide it without z-fighting.
        colored_box((size_x + 2 * thick, size_y + 2 * thick, 0.02), (0, 0, 0), FLOOR_RGB),
        colored_box((size_x + 2 * thick, thick, height), (0, (size_y + thick) / 2, height / 2), WALL_RGB),
        colored_box((size_x + 2 * thick, thick, height), (0, -(size_y + thick) / 2, height / 2), WALL_RGB),
        colored_box((thick, size_y, height), ((size_x + thick) / 2, 0, height / 2), WALL_RGB),
        colored_box((thick, size_y, height), (-(size_x + thick) / 2, 0, height / 2), WALL_RGB),
    ]

    spec = layout["board"]
    poster_w, poster_h = spec["poster_size"]
    panel_w = poster_w + 2 * spec["frame_margin"]
    panel_h = poster_h + 2 * spec["frame_margin"]
    panel_t = spec["panel_thickness"]
    center_z = spec["poster_center_z"]
    panel_bottom = center_z - panel_h / 2
    for board in layout["boards"]:
        _, lateral = board_axes(board)
        # Box local X spans the board width, local Y its thickness (see board_axes).
        yaw = np.radians(board["yaw_deg"]) - np.pi / 2
        xy = np.asarray(board["pos"], dtype=np.float64)
        parts.append(colored_box((panel_w, panel_t, panel_h), (*xy, center_z), PANEL_RGB, yaw))
        for side in (-1.0, 1.0):
            leg_xy = xy + side * (panel_w / 2 - 0.1) * lateral
            parts.append(colored_box((0.06, 0.06, panel_bottom), (*leg_xy, panel_bottom / 2), LEG_RGB, yaw))
            # Wide feet so the stand reads as free-standing.
            parts.append(colored_box((0.08, 0.5, 0.03), (*leg_xy, 0.025), LEG_RGB, yaw))
    return trimesh.util.concatenate(parts)


def write_poster_quad(path, layout, board):
    """Quad facing the board normal; u runs left-to-right for a viewer in front."""
    spec = layout["board"]
    w, h = spec["poster_size"]
    normal, lateral = board_axes(board)
    center = np.array((*board["pos"], spec["poster_center_z"]))
    center[:2] += (spec["panel_thickness"] / 2 + POSTER_FRONT_OFFSET) * normal
    # A viewer facing -normal has `lateral` on their right.
    right = np.array((*lateral, 0.0))
    up = np.array((0.0, 0.0, 1.0))
    corners = [center + (u - 0.5) * w * right + (v - 0.5) * h * up for u, v in ((0, 0), (1, 0), (1, 1), (0, 1))]
    with open(path, "w") as f:
        for p in corners:
            f.write(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
        for uv in ((0, 0), (1, 0), (1, 1), (0, 1)):
            f.write(f"vt {uv[0]} {uv[1]}\n")
        f.write(f"vn {normal[0]:.6f} {normal[1]:.6f} 0\n")
        # right x up == normal, so this winding is counter-clockwise from the front.
        f.write("f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")


def placeholder_poster(board, size):
    img = Image.new("RGB", size, (236, 238, 242))
    draw = ImageDraw.Draw(img)
    w, h = size
    draw.rectangle((0, 0, w, h // 7), fill=(28, 52, 110))
    draw.text((w // 2, h // 14), f"Poster #{board['id']}", fill="white",
              font=ImageFont.load_default(size=h // 12), anchor="mm")
    body = ImageFont.load_default(size=h // 20)
    for col in range(3):
        x0 = w * (0.03 + col * 0.325)
        for row in range(4):
            y0 = h * (0.2 + row * 0.19)
            draw.rectangle((x0, y0, x0 + w * 0.29, y0 + h * 0.16), outline=(160, 168, 184), width=3)
    draw.text((w // 2, h * 0.96), "Placeholder", fill=(120, 128, 144), font=body, anchor="mm")
    return img


def write_poster_texture(path, layout, board):
    w, h = layout["board"]["poster_size"]
    if board.get("poster"):
        img = Image.open(board["poster"]).convert("RGB")
    else:
        img = placeholder_poster(board, (TEXTURE_MAX_WIDTH, round(TEXTURE_MAX_WIDTH * h / w)))
    if img.width > TEXTURE_MAX_WIDTH:
        img = img.resize((TEXTURE_MAX_WIDTH, round(img.height * TEXTURE_MAX_WIDTH / img.width)), Image.LANCZOS)
    img_aspect, quad_aspect = img.width / img.height, w / h
    if abs(img_aspect - quad_aspect) > 0.02:
        print(f"  warning: board {board['id']} poster aspect {img_aspect:.2f} != quad {quad_aspect:.2f}")
    img.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", default=DEFAULT_LAYOUT)
    args = ap.parse_args()

    layout = load_layout(args.layout)
    paths = build_paths(layout)
    os.makedirs(layout["build_dir"], exist_ok=True)

    hall = build_hall(layout)
    hall.export(paths["hall_mesh"])
    print(f"wrote {paths['hall_mesh']}: {len(hall.faces)} faces, bounds {hall.bounds.tolist()}")
    for board in layout["boards"]:
        write_poster_quad(paths["poster_mesh"](board["id"]), layout, board)
        write_poster_texture(paths["poster_texture"](board["id"]), layout, board)
        print(f"wrote poster {board['id']} ({board['role']})")


if __name__ == "__main__":
    main()
