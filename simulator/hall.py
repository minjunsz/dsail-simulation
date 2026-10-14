"""Static conference hall: hall mesh, poster boards and their collision footprint."""

import os

import trimesh

import genesis as gs

from common.hall_layout import blocked, build_paths, obstacles


class Hall:
    def __init__(self, scene, layout):
        self.layout = layout
        paths = build_paths(layout)
        if not os.path.exists(paths["hall_mesh"]):
            raise FileNotFoundError(f"{paths['hall_mesh']} missing; run scripts/make_conference_hall.py")

        hall = trimesh.load(paths["hall_mesh"], force="mesh", process=False)
        scene.add_entity(gs.morphs.MeshSet(files=(hall,), fixed=True, collision=False,
                                           decimate=False, convexify=False))
        for board in layout["boards"]:
            scene.add_entity(
                gs.morphs.Mesh(
                    file=paths["poster_mesh"](board["id"]),
                    fixed=True,
                    collision=False,
                    decimate=False,
                    convexify=False,
                ),
                surface=gs.surfaces.Default(
                    diffuse_texture=gs.textures.ImageTexture(
                        image_path=paths["poster_texture"](board["id"]),
                    ),
                ),
            )
        self._obstacles = obstacles(layout)

    def blocked(self, xy, radius, others=()):
        """True if a circle at `xy` hits a wall, a board or one of `others` ((xy, radius) pairs)."""
        return blocked(self._obstacles, xy, radius, others)
