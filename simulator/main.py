"""Genesis simulator process with Unitree G1 humanoid locomotion.

Loads Unitree's official recurrent G1 locomotion policy and 12-DoF URDF.
Runs the policy at 50 Hz, follows the robot with a camera, and publishes
JPEG-compressed frames over ZeroMQ PUB socket.

Commands (ZMQ PULL) expect JSON from teleop_server:
{"robot_id": 1|2, "vx": ±1, "vy": ±1, "wz": ±1}
or, for the 'p' pose-print key: {"robot_id": 1|2, "type": "print_pose"}

Run:
    CUDA_VISIBLE_DEVICES=3 EGL_DEVICE_ID=3 python -m simulator.main
"""

import argparse
import os
import struct
import time
from io import BytesIO

import torch
import numpy as np
import trimesh
import zmq
from PIL import Image
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import genesis as gs
from genesis.utils.geom import transform_by_quat, inv_quat

# ── Sim params ────────────────────────────────────────────────────────────────
CONTROL_DT      = 0.02   # 50 Hz
PHYSICS_SUBSTEPS = 4     # 0.005 s; stable for G1 while keeping teleoperation responsive.
CAM_RES         = (1280, 720)
STABILIZE_STEPS = 100
GAIT_PERIOD_S   = 0.8

_ASSETS_DIR = os.path.join(os.path.dirname(__file__), "..", "assets")
PLANE_URDF  = os.path.join(_ASSETS_DIR, "urdf/plane/plane.urdf")
G1_URDF     = os.path.join(_ASSETS_DIR, "urdf/g1/g1_12dof.urdf")
G1_POLICY   = os.path.join(_ASSETS_DIR, "checkpoints/g1-locomotion/motion.pt")
DSAIL_MESH  = os.path.join(_ASSETS_DIR, "301_1F_ground_aligned.ply")
# G1_POLICY was trained on the full-size G1's own dynamics (leg length,
# torque-to-mass ratio, gait timing); shrinking the ROBOT to fit the scene
# breaks that (it fell and couldn't recover) even though scale keeps mass and
# inertia consistent. Growing the SCENE instead keeps the robot untouched --
# scale the DSAIL mesh up and the collision/SDF tuning below with it so the
# proxy count and total SDF cells stay the same as at scale 1.
DSAIL_SCALE = 2.0
DSAIL_Z_OFFSET = 0.0
# Safety-net ground plane, well below the DSAIL floor so it never z-fights or
# shows through the reconstructed floor mesh -- the DSAIL collision proxies
# are the real floor; this only catches a robot that somehow clips through.
GROUND_PLANE_Z_OFFSET = -1.0
ROBOT_SCALE = 1.0
CAMERA_POS_REL = np.array((-3.0, 0.0, 1.15), dtype=np.float32) * ROBOT_SCALE
CAMERA_LOOKAT_REL = np.array((0.0, 0.0, 0.0), dtype=np.float32)
CAMERA_UP_REL = np.array((0.0, 0.0, 1.0), dtype=np.float32)
CAMERA_SMOOTHING = 0.85
ROBOT_COUNT = 2
FRAME_HEADER = struct.Struct("<BQ")  # one-byte robot id + timestamp (ms)
# (pos_xyz, yaw_deg), found via the `p` teleop key -- one robot per poster below.
ROBOT_INIT_POSES = [
    ((6.388, -1.726, 0.746), 158.7),
    ((5.503, -2.498, 0.740), 164.4),
]

POSTER_TEXTURE = os.path.join(_ASSETS_DIR, "textures/example_poster.png")
G1_STANDING_HEIGHT = 1.3  # pelvis (~0.77m) + torso/head, see project_design/04_g1_humanoid_locomotion.md
G1_KNEE_HEIGHT = 0.33  # forward-kinematics'd from g1_12dof.urdf's leg joint origins at zero pose
POSTER_SIZE_SCALE = 2.0
POSTER_HEIGHT = G1_STANDING_HEIGHT * (2.0 / 3.0) * POSTER_SIZE_SCALE
POSTER_FLOAT_HEIGHT = G1_KNEE_HEIGHT  # bottom edge floats up to knee height, not resting on the floor
# (pos_xy, facing_yaw_deg). The raw `p`-key readings (roughly (3.950,-0.387)
# @ -27.0deg and (3.340,-1.545) @ -18.1deg) were close but not quite in line
# or parallel -- these are that pair snapped onto their common line (still
# through its midpoint), sharing one yaw (perpendicular to that line), and
# spaced by exactly poster width + a 0.2m gap, so they sit flush and level.
POSTER_INIT_POSES = [
    ((3.977, -0.336), -27.8),
    ((3.313, -1.596), -27.8),
]


def decimate_visual_mesh(mesh, face_count):
    """Simplify a colored mesh for rendering, re-attaching vertex colors.

    trimesh's quadric decimation drops vertex colors entirely (it has no
    color-aware error metric), so a naive decimate+render would turn the
    scan grey. Nearest-neighbor color transfer from the source mesh is cheap
    (a few seconds even for millions of points) and visually indistinguishable
    at this vertex density.
    """
    if len(mesh.faces) <= face_count:
        return mesh
    decimated = mesh.simplify_quadric_decimation(face_count=face_count)
    _, nearest = cKDTree(mesh.vertices).query(decimated.vertices, k=1)
    decimated.visual.vertex_colors = mesh.visual.vertex_colors[nearest]
    return decimated


def yaw_to_quat(yaw_rad):
    """Genesis (w,x,y,z) quat for a rotation of yaw_rad about world +z."""
    return (float(np.cos(yaw_rad / 2.0)), 0.0, 0.0, float(np.sin(yaw_rad / 2.0)))


def upright_plane_quat(normal_xy):
    """Genesis (w,x,y,z) quat for a vertical gs.morphs.Plane facing normal_xy.

    A default Plane lies flat (normal +z, local x/y axes = its width/height).
    Building the rotation from explicit basis vectors -- a horizontal width
    axis, world-up as the height axis, and the given horizontal normal --
    keeps the poster upright and right-side-up regardless of which way the
    scan's corridor happens to run in x/y.
    """
    normal = np.array([normal_xy[0], normal_xy[1], 0.0])
    normal /= np.linalg.norm(normal)
    up = np.array([0.0, 0.0, 1.0])
    width_dir = np.cross(up, normal)  # already unit length: up, normal orthonormal
    rotation = np.column_stack([width_dir, up, normal])
    qx, qy, qz, qw = Rotation.from_matrix(rotation).as_quat()
    return (float(qw), float(qx), float(qy), float(qz))


def build_spatial_convex_proxies(mesh, cell_size, half_thickness):
    """Approximate a scanned surface with local convex hulls.

    CoACD on the complete room shell creates hulls across its free interior.
    Grouping triangles into small spatial cells first keeps every hull local;
    a small two-sided extrusion gives thin walls and floors collision volume.
    """
    triangles = mesh.triangles
    normals = mesh.face_normals
    cell_ids = np.floor((mesh.triangles_center - mesh.bounds[0]) / cell_size).astype(np.int32)
    _, inverse = np.unique(cell_ids, axis=0, return_inverse=True)

    proxies = []
    for cell_idx in range(inverse.max() + 1):
        mask = inverse == cell_idx
        cell_vertices = triangles[mask].reshape(-1, 3)
        offsets = np.repeat(normals[mask], 3, axis=0) * half_thickness
        points = np.concatenate((cell_vertices + offsets, cell_vertices - offsets))
        proxies.append(trimesh.convex.convex_hull(points))

    return tuple(proxies)


def spawn_column_is_clear(pos_xy, proxies, z_range=(0.05, 1.8), n_samples=20):
    """Check that a vertical column at pos_xy doesn't intersect any collision proxy.

    A coarse --collision-cell-size merges every triangle in a cell into one
    convex hull; being convex, that hull can bulge across a hallway's open
    width between two grouped wall/floor sections even though the real scan
    has nothing there. A robot spawned inside it gets an instant, violent
    de-penetration and falls over before it ever takes a step -- with no
    error, so this is otherwise silent. Catch it at scene-build time instead.
    """
    test_pts = np.array([[pos_xy[0], pos_xy[1], z] for z in np.linspace(*z_range, n_samples)])
    for proxy in proxies:
        if np.any(test_pts.min(axis=0) > proxy.bounds[1]) or np.any(test_pts.max(axis=0) < proxy.bounds[0]):
            continue
        if proxy.contains(test_pts).any():
            return False
    return True


def get_robot_pose(robot):
    """Return (pos_xyz, yaw_rad) of a robot's base link, world frame."""
    pos = robot.base_link.get_pos().detach().cpu().numpy().reshape(3)
    quat = robot.base_link.get_quat().detach().cpu().numpy().reshape(4)
    rotation = gs.utils.geom.quat_to_R(quat)
    yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return pos, yaw


def update_tps_camera(cam, robot, state):
    """Follow the robot using only its smoothed position and yaw.

    Attaching directly to the base also transfers the humanoid's roll, pitch,
    and body bounce to the camera.  A TPS camera should follow its heading while
    remaining level with the world, so project the base orientation onto XY.
    """
    base_pos = robot.base_link.get_pos().detach().cpu().numpy().reshape(3)
    base_quat = robot.base_link.get_quat().detach().cpu().numpy().reshape(4)
    base_rotation = gs.utils.geom.quat_to_R(base_quat)
    target_yaw = float(np.arctan2(base_rotation[1, 0], base_rotation[0, 0]))

    if not state:
        state["anchor"] = base_pos.copy()
        state["yaw"] = target_yaw
    else:
        alpha = CAMERA_SMOOTHING
        state["anchor"] = alpha * state["anchor"] + (1.0 - alpha) * base_pos
        yaw_delta = np.arctan2(
            np.sin(target_yaw - state["yaw"]),
            np.cos(target_yaw - state["yaw"]),
        )
        state["yaw"] += (1.0 - alpha) * yaw_delta

    cos_yaw = np.cos(state["yaw"])
    sin_yaw = np.sin(state["yaw"])
    yaw_rotation = np.array(
        ((cos_yaw, -sin_yaw, 0.0), (sin_yaw, cos_yaw, 0.0), (0.0, 0.0, 1.0)),
        dtype=np.float32,
    )
    camera_pos = state["anchor"] + yaw_rotation @ CAMERA_POS_REL
    camera_lookat = state["anchor"] + yaw_rotation @ CAMERA_LOOKAT_REL
    cam.set_pose(pos=camera_pos, lookat=camera_lookat, up=CAMERA_UP_REL)


def build_obs(robot, motors_dof_idx, default_dof_pos, commands, actions,
              global_gravity, phase):
    """
    Unitree G1's 47-dim observation (all tensors 1-D):
      ang_vel*ang_vel_scale | proj_gravity | commands*[lin_vel,lin_vel,ang_vel] |
      (dof_pos-default) | dof_vel*0.05 | actions | sin/cos gait phase
    """
    base_quat    = robot.get_quat()                                 # (4,)
    inv_q        = inv_quat(base_quat)                              # (4,)
    base_ang_vel = transform_by_quat(robot.get_ang(), inv_q)        # (3,)
    proj_gravity = transform_by_quat(global_gravity, inv_q)         # (3,)
    dof_pos      = robot.get_dofs_position(motors_dof_idx)          # (12,)
    dof_vel      = robot.get_dofs_velocity(motors_dof_idx)          # (12,)

    cmd_scale = torch.tensor(
        [2.0, 2.0, 0.25],
        dtype=gs.tc_float, device=gs.device,
    )
    gait_phase = torch.tensor(
        [np.sin(2.0 * np.pi * phase), np.cos(2.0 * np.pi * phase)],
        dtype=gs.tc_float,
        device=gs.device,
    )

    return torch.cat([
        base_ang_vel * 0.25,                                     # 3
        proj_gravity,                                            # 3
        commands * cmd_scale,                                    # 3
        dof_pos - default_dof_pos,                               # 12
        dof_vel * 0.05,                                          # 12
        actions,                                                 # 12
        gait_phase,                                              # 2
    ])  # (47,)


def recv_latest_commands(sock, current_commands):
    """Drain pending input and keep the newest command for each robot.

    A single conflated socket cannot be used here because a command for robot 2
    could replace robot 1's latest command. Draining the small shared queue and
    updating by robot id preserves independent current-state channels.

    Also returns the set of (0-indexed) robots whose browser client pressed
    'p' this tick -- teleop_server forwards it as {"robot_id":.., "type":
    "print_pose"} over the same channel as move commands, since that's the
    only path from the browser's keyboard (WASD) to this process.
    """
    pose_print_robot_idxs = set()
    while True:
        try:
            latest = sock.recv_json(zmq.NOBLOCK)
        except zmq.Again:
            break

        try:
            robot_idx = int(latest["robot_id"]) - 1
        except (KeyError, TypeError, ValueError):
            continue
        if not 0 <= robot_idx < len(current_commands):
            continue

        if latest.get("type") == "print_pose":
            pose_print_robot_idxs.add(robot_idx)
            continue

        current_commands[robot_idx] = torch.tensor(
            [
                float(latest.get("vx", 0.0)) * 0.5,
                float(latest.get("vy", 0.0)) * 0.5,
                float(latest.get("wz", 0.0)) * 1.0,
            ],
            dtype=gs.tc_float,
            device=gs.device,
        )

    return current_commands, pose_print_robot_idxs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pub-port",   type=int, default=5555)
    parser.add_argument("--cmd-port",   type=int, default=5556)
    parser.add_argument("--checkpoint", type=str, default=G1_POLICY)
    parser.add_argument(
        "--collision-cell-size",
        type=float,
        default=1.0 * DSAIL_SCALE,
        help=(
            "Spatial convex decomposition cell size in meters (smaller is more accurate, "
            "but more geoms -- 0.5 at DSAIL_SCALE=1 gives ~1800 collision proxies on the "
            "301_1F scan, which dominates both scene-build time and per-step broad-phase "
            "cost). Scales with DSAIL_SCALE so proxy count stays roughly constant. Keep this "
            "no larger than ~1x DSAIL_SCALE: since each cell's triangles get merged into one "
            "convex hull, a coarser cell can bulge across a hallway's open width and clip a "
            "robot spawned inside it (confirmed at 1.5x -- one robot spawned fine, the other "
            "landed inside a hull that bridged across the corridor and fell over instantly)."
        ),
    )
    parser.add_argument(
        "--visual-face-count",
        type=int,
        default=500_000,
        help="Decimate the rendered (colored) DSAIL mesh to at most this many faces (0 disables).",
    )
    parser.add_argument(
        "--collision-half-thickness",
        type=float,
        default=0.01 * DSAIL_SCALE,
        help="One-sided collision proxy extrusion in meters. Scales with DSAIL_SCALE.",
    )
    parser.add_argument(
        "--sdf-cell-size",
        type=float,
        default=0.05 * DSAIL_SCALE,
        help=(
            "Rigid material SDF cell size for the DSAIL collision proxies. Genesis's "
            "auto-clamp gives each proxy geom up to sdf_max_res^3 cells, summed over every "
            "proxy; at the default --collision-cell-size that stays well under int32 range "
            "and keeps the total SDF memory small, which matters for both scene-build time "
            "and per-step lookup cost. Raise --collision-cell-size or this if you still hit "
            "'SDF Gradient shape ... too large'. Scales with DSAIL_SCALE."
        ),
    )
    args = parser.parse_args()

    if args.collision_cell_size <= 0.0:
        parser.error("--collision-cell-size must be positive")
    if args.collision_half_thickness <= 0.0:
        parser.error("--collision-half-thickness must be positive")

    publish_addr = f"tcp://*:{args.pub_port}"
    cmd_addr     = f"tcp://*:{args.cmd_port}"

    resume_path = args.checkpoint
    if not os.path.exists(resume_path):
        raise FileNotFoundError(f"Checkpoint not found: {resume_path}")

    # --- Genesis setup (n_envs=0 → single-env rasterizer, no batch renderer) ---
    gs.init(backend=gs.gpu, logging_level="warning", performance_mode=True)

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=CONTROL_DT, substeps=PHYSICS_SUBSTEPS),
        rigid_options=gs.options.RigidOptions(
            enable_self_collision=False,
            max_collision_pairs=100,
        ),
        vis_options=gs.options.VisOptions(
            lights=[],  # drop the default directional "sun" light
            ambient_light=(0.5, 0.5, 0.5),  # uniform, shadow-free illumination
            shadow=False,
        ),
        show_viewer=False,
    )

    scene.add_entity(gs.morphs.URDF(file=PLANE_URDF, fixed=True, pos=(0.0, 0.0, GROUND_PLANE_Z_OFFSET)))
    # Genesis 0.4.x does not accept PLY paths directly, but MeshSet accepts an
    # in-memory trimesh while preserving the PLY's vertex colors.
    dsail_trimesh = trimesh.load(DSAIL_MESH, force="mesh", process=False)
    dsail_trimesh.apply_scale(DSAIL_SCALE)
    dsail_trimesh.apply_translation((0.0, 0.0, DSAIL_Z_OFFSET))
    dsail_collision_trimesh = dsail_trimesh.simplify_quadric_decimation(
        face_count=5_000,
        aggression=5,
    )
    dsail_collision_parts = build_spatial_convex_proxies(
        dsail_collision_trimesh,
        cell_size=args.collision_cell_size,
        half_thickness=args.collision_half_thickness,
    )
    print(f"[simulator] DSAIL collision proxies: {len(dsail_collision_parts)} convex hulls")
    print(f"[simulator] DSAIL bounds: {dsail_trimesh.bounds.tolist()}")
    # Collision uses its own decimated copy above; the render copy is kept
    # separate so the GPU never builds collision data from it. It still needs
    # its own decimation -- at native res (8.5M faces on 301_1F) the
    # rasterizer redraws every triangle every frame regardless of `fixed`,
    # which tanks FPS far more than a 500k-face mesh looks worse.
    dsail_visual_trimesh = (
        decimate_visual_mesh(dsail_trimesh, args.visual_face_count)
        if args.visual_face_count > 0
        else dsail_trimesh
    )
    print(f"[simulator] DSAIL visual mesh: {len(dsail_visual_trimesh.faces)} faces (from {len(dsail_trimesh.faces)})")
    scene.add_entity(
        gs.morphs.MeshSet(
            files=(dsail_visual_trimesh,),
            fixed=True,
            collision=False,
            decimate=False,
            convexify=False,
        )
    )
    scene.add_entity(
        gs.morphs.MeshSet(
            files=dsail_collision_parts,
            fixed=True,
            visualization=False,
            collision=True,
            # Every input part is already a local convex hull.
            convexify=True,
            decimate=False,
        ),
        # Default sdf_cell_size=0.005 gives each of the ~1800 proxies up to
        # sdf_max_res^3 SDF cells; summed across all of them that overflows
        # int32 ("SDF Gradient shape ... too large"). A coarser cell size is
        # plenty for these thin collision slabs.
        material=gs.materials.Rigid(sdf_cell_size=args.sdf_cell_size),
    )
    robot_init_positions = [pos for pos, _ in ROBOT_INIT_POSES]
    robot_init_quats = [yaw_to_quat(np.radians(yaw_deg)) for _, yaw_deg in ROBOT_INIT_POSES]
    robots = []
    for robot_id, (robot_init_pos, robot_init_quat) in enumerate(zip(robot_init_positions, robot_init_quats), start=1):
        print(f"[simulator] G1 #{robot_id} initial position: {robot_init_pos}")
        if not spawn_column_is_clear(robot_init_pos[:2], dsail_collision_parts):
            print(
                f"[simulator] WARNING: G1 #{robot_id} spawn point intersects a collision "
                f"proxy -- it will likely spawn falling over. Try a smaller "
                f"--collision-cell-size (currently {args.collision_cell_size})."
            )
        robots.append(
            scene.add_entity(
                gs.morphs.URDF(
                    file=G1_URDF,
                    pos=robot_init_pos,
                    quat=robot_init_quat,
                    scale=ROBOT_SCALE,
                )
            )
        )

    # Poster: a flat, un-tiled textured Plane (a thin Box has no per-face UVs
    # in Genesis, so it can't show a single right-side-up image). Position and
    # facing were found by walking a robot to the spot and reading its pose
    # off with the `p` teleop key.
    poster_image_w, poster_image_h = Image.open(POSTER_TEXTURE).size
    poster_size = (POSTER_HEIGHT * poster_image_w / poster_image_h, POSTER_HEIGHT)
    for poster_id, (poster_xy, poster_yaw_deg) in enumerate(POSTER_INIT_POSES, start=1):
        poster_facing = np.array((np.cos(np.radians(poster_yaw_deg)), np.sin(np.radians(poster_yaw_deg))))
        poster_pos = (poster_xy[0], poster_xy[1], POSTER_FLOAT_HEIGHT + POSTER_HEIGHT / 2.0)
        poster_quat = upright_plane_quat(poster_facing)
        print(f"[simulator] poster #{poster_id} position: {poster_pos}  size: {poster_size}")
        scene.add_entity(
            gs.morphs.Plane(
                pos=poster_pos,
                quat=poster_quat,
                plane_size=poster_size,
                tile_size=poster_size,
                fixed=True,
            ),
            surface=gs.surfaces.Default(
                diffuse_texture=gs.textures.ImageTexture(image_path=POSTER_TEXTURE),
            ),
        )

    cameras = []
    for robot_init_pos, robot_yaw_deg in zip(robot_init_positions, (yaw for _, yaw in ROBOT_INIT_POSES)):
        cos_yaw = np.cos(np.radians(robot_yaw_deg))
        sin_yaw = np.sin(np.radians(robot_yaw_deg))
        yaw_rotation = np.array(
            ((cos_yaw, -sin_yaw, 0.0), (sin_yaw, cos_yaw, 0.0), (0.0, 0.0, 1.0)),
            dtype=np.float32,
        )
        cameras.append(
            scene.add_camera(
                res=CAM_RES,
                pos=tuple(np.asarray(robot_init_pos) + yaw_rotation @ CAMERA_POS_REL),
                lookat=tuple(np.asarray(robot_init_pos) + yaw_rotation @ CAMERA_LOOKAT_REL),
                fov=45,
                GUI=False,
            )
        )

    scene.build()   # no n_envs → rasterizer path, cam.render() returns (H, W, 3)
    camera_states = [{} for _ in range(ROBOT_COUNT)]

    # --- DOF setup ---
    joint_names = [
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    ]
    motors_dof_indices = []
    for robot in robots:
        motors_dof_idx = torch.tensor(
            [
                robot.get_joint(name).dof_start - robot.dof_start
                for name in joint_names
            ],
            dtype=gs.tc_int,
            device=gs.device,
        )
        robot.set_dofs_kp(
            [100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40],
            motors_dof_idx,
        )
        robot.set_dofs_kv(
            [2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2],
            motors_dof_idx,
        )
        motors_dof_indices.append(motors_dof_idx)

    default_dof_pos = torch.tensor(
        [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0],
        dtype=gs.tc_float, device=gs.device,
    )  # (12,)

    for robot, robot_init_pos, robot_init_quat in zip(robots, robot_init_positions, robot_init_quats):
        robot.set_qpos(torch.cat([
            torch.tensor(robot_init_pos, dtype=gs.tc_float, device=gs.device),
            torch.tensor(robot_init_quat, dtype=gs.tc_float, device=gs.device),
            default_dof_pos,
        ]))

    global_gravity = torch.tensor([0.0, 0.0, -1.0], dtype=gs.tc_float, device=gs.device)
    action_scale = 0.25

    # Unitree exports the actor and its recurrent state as one TorchScript module.
    print(f"Loading policy from {resume_path}")
    # The policy is recurrent, so each robot needs its own module and memory.
    actors = []
    for _ in range(ROBOT_COUNT):
        actor = torch.jit.load(resume_path, map_location=gs.device).eval()
        actor.reset_memory()
        actors.append(actor)

    # --- ZeroMQ setup ---
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, ROBOT_COUNT * 2)
    sock.bind(publish_addr)
    print(f"[simulator] publishing frames on {publish_addr}")

    cmd_sock = ctx.socket(zmq.PULL)
    cmd_sock.setsockopt(zmq.RCVHWM, 100)
    cmd_sock.bind(cmd_addr)
    print(f"[simulator] receiving commands on {cmd_addr}")

    # All 1-D tensors (n_envs=0 API)
    curr_actions = [
        torch.zeros(12, dtype=gs.tc_float, device=gs.device)
        for _ in range(ROBOT_COUNT)
    ]
    commands = [
        torch.zeros(3, dtype=gs.tc_float, device=gs.device)
        for _ in range(ROBOT_COUNT)
    ]

    # --- Stabilisation ---
    print(f"Stabilising ({STABILIZE_STEPS} steps, {STABILIZE_STEPS * CONTROL_DT:.1f} s)...")
    zero_cmd = torch.zeros(3, dtype=gs.tc_float, device=gs.device)
    with torch.no_grad():
        for step in range(STABILIZE_STEPS):
            phase = (step * CONTROL_DT % GAIT_PERIOD_S) / GAIT_PERIOD_S
            for robot_idx in range(ROBOT_COUNT):
                obs = build_obs(
                    robots[robot_idx],
                    motors_dof_indices[robot_idx],
                    default_dof_pos,
                    zero_cmd,
                    curr_actions[robot_idx],
                    global_gravity,
                    phase,
                )
                curr_actions[robot_idx] = actors[robot_idx](obs.unsqueeze(0)).squeeze(0)

                target = curr_actions[robot_idx] * action_scale + default_dof_pos
                robots[robot_idx].control_dofs_position(
                    target,
                    motors_dof_indices[robot_idx],
                )
            scene.step()

    print("[simulator] ready")

    policy_step = STABILIZE_STEPS
    while True:
        t0 = time.perf_counter()

        # 1. Receive velocity command
        # teleop_server forwards normalized velocity commands from the browser keyboard.
        commands, pose_print_robot_idxs = recv_latest_commands(cmd_sock, commands)
        for robot_idx in pose_print_robot_idxs:
            pos, yaw = get_robot_pose(robots[robot_idx])
            print(
                f"[simulator] G1 #{robot_idx + 1} pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) "
                f"yaw={np.degrees(yaw):.1f}deg forward=({np.cos(yaw):.3f}, {np.sin(yaw):.3f})"
            )

        # 2. Policy inference. The gait phase is part of Unitree's training observation.
        phase = (policy_step * CONTROL_DT % GAIT_PERIOD_S) / GAIT_PERIOD_S
        with torch.no_grad():
            for robot_idx in range(ROBOT_COUNT):
                obs = build_obs(
                    robots[robot_idx],
                    motors_dof_indices[robot_idx],
                    default_dof_pos,
                    commands[robot_idx],
                    curr_actions[robot_idx],
                    global_gravity,
                    phase,
                )
                curr_actions[robot_idx] = actors[robot_idx](obs.unsqueeze(0)).squeeze(0)

                target = curr_actions[robot_idx] * action_scale + default_dof_pos
                robots[robot_idx].control_dofs_position(
                    target,
                    motors_dof_indices[robot_idx],
                )

        # 3. Physics step + render
        scene.step()
        for robot_idx in range(ROBOT_COUNT):
            update_tps_camera(
                cameras[robot_idx],
                robots[robot_idx],
                camera_states[robot_idx],
            )

        policy_step += 1

        # 4. Render and publish one addressed frame for each robot.
        ts_ms = int(time.time() * 1000)
        for robot_id, cam in enumerate(cameras, start=1):
            rgb_arr, _, _, _ = cam.render()
            buf = BytesIO()
            Image.fromarray(rgb_arr).save(buf, format="JPEG", quality=85)
            jpeg_bytes = buf.getvalue()

            try:
                sock.send(FRAME_HEADER.pack(robot_id, ts_ms) + jpeg_bytes, zmq.NOBLOCK)
            except zmq.Again:
                pass

        elapsed = time.perf_counter() - t0
        sleep_time = CONTROL_DT - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
