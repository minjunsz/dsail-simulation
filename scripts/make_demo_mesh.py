"""Build a small vertex-colored demo scene as a stand-in for assets/dsail_mesh.ply.

Three low walls (open on +X, where the robots spawn), a few boxes and pillars, and
a shallow ramp, all in meters and Z-up. Load it with:

    python -m simulator.main --scene-mesh assets/demo_mesh.ply --scene-scale 1.0
"""
import argparse
import os

import numpy as np
import trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def colored_box(extents, center, rgb):
    box = trimesh.creation.box(extents=extents)
    box.apply_translation(center)
    box.visual.vertex_colors = np.tile((*rgb, 255), (len(box.vertices), 1)).astype(np.uint8)
    return box


def colored_cylinder(radius, height, center, rgb):
    cyl = trimesh.creation.cylinder(radius=radius, height=height, sections=24)
    cyl.apply_translation(center)
    cyl.visual.vertex_colors = np.tile((*rgb, 255), (len(cyl.vertices), 1)).astype(np.uint8)
    return cyl


def build():
    wall = (200, 196, 186)
    parts = [
        # Walls: back (-X) and both sides; +X is left open for the spawn area.
        colored_box((0.2, 8.0, 1.2), (-4.0, 0.0, 0.6), wall),
        colored_box((8.0, 0.2, 1.2), (0.0, 4.0, 0.6), wall),
        colored_box((8.0, 0.2, 1.2), (0.0, -4.0, 0.6), wall),
        # Obstacles.
        colored_box((0.8, 0.8, 0.8), (-1.5, 1.5, 0.4), (196, 82, 64)),
        colored_box((1.2, 0.6, 0.5), (-2.2, -1.8, 0.25), (64, 118, 196)),
        colored_box((1.6, 0.8, 0.75), (0.5, -2.5, 0.375), (150, 110, 70)),
        colored_cylinder(0.25, 1.5, (0.8, 1.2, 0.75), (90, 160, 90)),
        colored_cylinder(0.25, 1.5, (-0.5, -0.4, 0.75), (90, 160, 90)),
    ]
    # Shallow ramp (~8.5 deg) the G1 can walk onto.
    ramp = trimesh.creation.box(extents=(2.0, 1.2, 0.05))
    ramp.apply_transform(trimesh.transformations.rotation_matrix(np.radians(-8.5), (0, 1, 0)))
    ramp.apply_translation((-2.0, 0.6, 0.15))
    ramp.visual.vertex_colors = np.tile((220, 170, 60, 255), (len(ramp.vertices), 1)).astype(np.uint8)
    parts.append(ramp)
    return trimesh.util.concatenate(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "assets", "demo_mesh.ply"))
    args = ap.parse_args()
    mesh = build()
    mesh.export(args.out)
    print(f"wrote {args.out}: {len(mesh.faces)} faces, bounds {mesh.bounds.tolist()}")


if __name__ == "__main__":
    main()
