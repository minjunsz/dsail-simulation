"""Fit and correct the floor plane of a gravity-aligned room scan mesh.

Indoor LiDAR/photogrammetry scans are rarely perfectly level: the floor sits
some distance off the origin and tilts by a degree or two, and the scan often
contains large outliers along the up axis (reflective surfaces, glass,
mirrors). This assumes gravity alignment is already done -- the floor's
normal is already close to the scan's own `--up-axis` -- and only needs to:

  1. Locate the floor's height from the sharpest peak in the vertices'
     histogram along the up axis. A real floor is a large, unbroken flat
     surface, so it is far denser in a thin slice than clutter, furniture,
     or scan noise.
  2. Robustly fit a plane to that slab via SVD, iteratively trimming
     outliers (furniture resting on the floor, residual scan noise).
  3. Rotate the mesh so the fitted normal is exactly +z (Genesis's up axis,
     regardless of which axis was "up" in the raw scan), and translate it so
     the floor sits at z=0 -- pivoting the rotation about the floor patch's
     own centroid so the room's horizontal layout is left where it was,
     matching the project's existing "preserve scan XY, don't renormalize"
     convention (see project_design/03_dsail_scene_mesh.md).

Run:
    uv run python scripts/fit_ground_plane.py assets/301_1F.ply assets/301_1F_ground_aligned.ply --up-axis y
"""

from __future__ import annotations

import argparse

import numpy as np
import trimesh

AXES = {"x": 0, "y": 1, "z": 2}


def _svd_plane_fit(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Total-least-squares plane fit. Returns (unit normal, centroid)."""
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[-1]
    return normal, centroid


def fit_ground_plane(
    vertices: np.ndarray,
    *,
    up_axis: int = 2,
    coarse_bin_size: float = 0.01,
    search_max_quantile: float = 0.6,
    slab_half_thickness: float = 0.06,
    outlier_reject_std: float = 3.0,
    n_refine_iters: int = 3,
    max_tilt_deg: float = 15.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Locate and fit the floor plane of a gravity-aligned scan.

    `up_axis` is the scan's own vertical axis (0/1/2 for x/y/z) -- whatever
    it is, the returned normal is oriented to point that way, and
    `ground_alignment_transform` rotates it onto Genesis's +z regardless.

    Returns (unit normal pointing up, a point on the plane).
    """
    z = vertices[:, up_axis]

    # A real floor is a large, unbroken flat surface, so it produces a much
    # sharper peak in a fine histogram than clutter or scan noise. Search
    # only the lower `search_max_quantile` of points so a similarly flat
    # ceiling can't win instead.
    z_cutoff = np.quantile(z, search_max_quantile)
    low_z = z[z <= z_cutoff]
    bins = np.arange(low_z.min(), z_cutoff + coarse_bin_size, coarse_bin_size)
    hist, edges = np.histogram(low_z, bins=bins)
    peak = np.argmax(hist)
    floor_z_guess = 0.5 * (edges[peak] + edges[peak + 1])

    slab_mask = np.abs(z - floor_z_guess) <= slab_half_thickness
    floor_pts = vertices[slab_mask]
    if len(floor_pts) < 100:
        raise ValueError(
            f"Only {len(floor_pts)} candidate floor points found around "
            f"up_axis={floor_z_guess:.3f}; widen slab_half_thickness or check the mesh."
        )

    normal, point = _svd_plane_fit(floor_pts)
    residuals = (floor_pts - point) @ normal
    for _ in range(n_refine_iters):
        std = residuals.std()
        if std < 1e-6:
            break
        keep = np.abs(residuals) <= outlier_reject_std * std
        if keep.all() or keep.sum() < 100:
            break
        floor_pts = floor_pts[keep]
        normal, point = _svd_plane_fit(floor_pts)
        residuals = (floor_pts - point) @ normal

    if normal[up_axis] < 0:
        normal = -normal
    tilt_deg = float(np.degrees(np.arccos(np.clip(normal[up_axis], -1.0, 1.0))))
    if tilt_deg > max_tilt_deg:
        raise ValueError(
            f"Fitted floor normal is {tilt_deg:.1f} deg off the up axis (> "
            f"{max_tilt_deg} deg). Mesh may not be gravity-aligned, --up-axis "
            "may be wrong, or the wrong horizontal surface was picked up."
        )

    print(
        f"[fit_ground_plane] up_axis={up_axis}  floor_pos~={floor_z_guess:.4f}  "
        f"inliers={len(floor_pts)}  tilt={tilt_deg:.3f} deg  "
        f"residual_std={residuals.std() * 1000:.2f} mm"
    )
    return normal, point


def ground_alignment_transform(normal: np.ndarray, point: np.ndarray) -> np.ndarray:
    """4x4 transform: rotate `normal` onto +z, snap the plane to z=0.

    Pivots the rotation about `point` (the floor patch centroid) so the XY
    layout near the room's center is preserved, then restores that point's
    original X/Y after the rotation.
    """
    rotation = trimesh.geometry.align_vectors(normal, np.array([0.0, 0.0, 1.0]))
    transform = rotation.copy()
    transform[:3, 3] = -rotation[:3, :3] @ point
    transform[:2, 3] += point[:2]
    return transform


def clean_and_crop_mesh(
    mesh: trimesh.Trimesh,
    *,
    floor_margin: float,
    ceiling_height: float,
    min_edge: float,
) -> trimesh.Trimesh:
    """Drop sliver/degenerate faces and crop to a walkable vertical band.

    Assumes the mesh is already ground-aligned (floor at z=0). Raw scans like
    this one carry two kinds of z-noise that a naive scale/offset never
    handles: SfM floaters that trail many meters above and below the real
    structure (visible as a sparse "tube" far past the dense corridor cross-
    section when the scan is plotted in x-z/y-z), and sliver triangles from
    mesh decimation that bridge real geometry to those far floaters. A single
    such sliver, extruded into a collision proxy, has such an extreme aspect
    ratio that it defeats Genesis's automatic SDF-resolution clamp and blows
    up 'n_cells' past int32 range. Cropping to the robot's walkable band and
    dropping thin/degenerate faces removes both at once.
    """
    z = mesh.vertices[:, 2]
    vertex_in_band = (z >= -floor_margin) & (z <= ceiling_height)
    face_mask = vertex_in_band[mesh.faces].all(axis=1)
    face_mask &= mesh.nondegenerate_faces(height=min_edge)

    n_faces_before = len(mesh.faces)
    mesh.update_faces(face_mask)
    mesh.remove_unreferenced_vertices()
    print(
        f"[clean_and_crop_mesh] kept {len(mesh.faces)}/{n_faces_before} faces "
        f"(z in [{-floor_margin:.2f}, {ceiling_height:.2f}], min_edge={min_edge}); "
        f"{len(mesh.vertices)} vertices, bounds={mesh.bounds.tolist()}"
    )
    return mesh


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", help="raw scan .ply")
    parser.add_argument("output", help="ground-aligned .ply to write")
    parser.add_argument(
        "--up-axis",
        choices=sorted(AXES),
        default="z",
        help="which axis is vertical in the raw scan's own coordinates (default z)",
    )
    parser.add_argument("--search-max-quantile", type=float, default=0.6)
    parser.add_argument("--slab-half-thickness", type=float, default=0.06)
    parser.add_argument(
        "--floor-margin",
        type=float,
        default=0.3,
        help="meters below the fitted floor to keep (captures baseboards/floor noise)",
    )
    parser.add_argument(
        "--ceiling-height",
        type=float,
        default=4.0,
        help="meters above the fitted floor to keep; drops stairwells/mezzanines/scan floaters above this",
    )
    parser.add_argument(
        "--min-edge",
        type=float,
        default=1e-6,
        help=(
            "drop faces whose oriented 2D bbox has an edge shorter than this. Keep tiny: this mesh's own "
            "triangles are naturally sub-centimeter, so anything much above ~1e-6 discards valid geometry, "
            "not just true zero-area/duplicate-vertex degenerate faces."
        ),
    )
    parser.add_argument(
        "--crop",
        action="store_true",
        help=(
            "crop to [-floor-margin, ceiling-height] around the single fitted floor plane. Off by default: "
            "this assumes one flat floor across the whole mesh, which does not hold on scans with SfM "
            "z-drift along their length (a real, gradual bend, not just noise) -- on 301_1F.ply it would "
            "discard most of the corridor's length. Use scripts.fit_ground_plane's clean_and_crop_mesh "
            "directly if you need this on a mesh you've confirmed is flat."
        ),
    )
    parser.add_argument("--dry-run", action="store_true", help="fit and report, but don't write output")
    args = parser.parse_args()

    mesh = trimesh.load(args.input, force="mesh", process=False)
    print(f"loaded {args.input}: {len(mesh.vertices)} vertices, bounds={mesh.bounds.tolist()}")

    normal, point = fit_ground_plane(
        mesh.vertices,
        up_axis=AXES[args.up_axis],
        search_max_quantile=args.search_max_quantile,
        slab_half_thickness=args.slab_half_thickness,
    )
    transform = ground_alignment_transform(normal, point)

    if args.dry_run:
        preview = mesh.copy()
        preview.apply_transform(transform)
        print(f"[dry-run] aligned bounds would be={preview.bounds.tolist()}")
        if args.crop:
            clean_and_crop_mesh(
                preview,
                floor_margin=args.floor_margin,
                ceiling_height=args.ceiling_height,
                min_edge=args.min_edge,
            )
        return

    mesh.apply_transform(transform)
    print(f"aligned bounds={mesh.bounds.tolist()}")
    if args.crop:
        clean_and_crop_mesh(
            mesh,
            floor_margin=args.floor_margin,
            ceiling_height=args.ceiling_height,
            min_edge=args.min_edge,
        )
    mesh.export(args.output)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
