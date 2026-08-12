"""Genesis simulator process with Unitree G1 humanoid locomotion.

Loads Unitree's official recurrent G1 locomotion policy and 12-DoF URDF.
Runs the policy at 50 Hz, follows the robot with a camera, and publishes
JPEG-compressed frames over ZeroMQ PUB socket.

Commands (ZMQ PULL) expect JSON from teleop_server:
{"robot_id": 1|2, "vx": ±1, "vy": ±1, "wz": ±1}

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
DSAIL_MESH  = os.path.join(_ASSETS_DIR, "dsail_mesh.ply")
DSAIL_SCALE = 0.5
DSAIL_Z_OFFSET = -0.05
ROBOT_SCENE_CLEARANCE = 0.75
ROBOT_INIT_YAW = np.pi  # Spawn on +X while facing inward toward the DSAIL scene.
CAMERA_POS_REL = np.array((-3.0, 0.0, 1.15), dtype=np.float32)
CAMERA_LOOKAT_REL = np.array((0.0, 0.0, 0.0), dtype=np.float32)
CAMERA_UP_REL = np.array((0.0, 0.0, 1.0), dtype=np.float32)
CAMERA_SMOOTHING = 0.85
ROBOT_COUNT = 2
ROBOT_SPAWN_SEPARATION = 1.5
FRAME_HEADER = struct.Struct("<BQ")  # one-byte robot id + timestamp (ms)


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
    """
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

        current_commands[robot_idx] = torch.tensor(
            [
                float(latest.get("vx", 0.0)) * 0.5,
                float(latest.get("vy", 0.0)) * 0.5,
                float(latest.get("wz", 0.0)) * 1.0,
            ],
            dtype=gs.tc_float,
            device=gs.device,
        )

    return current_commands


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pub-port",   type=int, default=5555)
    parser.add_argument("--cmd-port",   type=int, default=5556)
    parser.add_argument("--checkpoint", type=str, default=G1_POLICY)
    parser.add_argument(
        "--collision-cell-size",
        type=float,
        default=0.5,
        help="Spatial convex decomposition cell size in meters (smaller is more accurate).",
    )
    parser.add_argument(
        "--collision-half-thickness",
        type=float,
        default=0.01,
        help="One-sided collision proxy extrusion in meters.",
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
        show_viewer=False,
    )

    scene.add_entity(gs.morphs.URDF(file=PLANE_URDF, fixed=True))
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
    # Keep the full-resolution scan for rendering, separate from physics so the
    # GPU never builds collision data from the 1.5M-face source mesh.
    scene.add_entity(
        gs.morphs.MeshSet(
            files=(dsail_trimesh,),
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
        )
    )
    robot_init_x = float(dsail_trimesh.bounds[1, 0] + ROBOT_SCENE_CLEARANCE)
    robot_init_positions = [
        (
            robot_init_x,
            (robot_idx - (ROBOT_COUNT - 1) / 2.0) * ROBOT_SPAWN_SEPARATION,
            0.8,
        )
        for robot_idx in range(ROBOT_COUNT)
    ]
    robot_init_quat = (
        float(np.cos(ROBOT_INIT_YAW / 2.0)),
        0.0,
        0.0,
        float(np.sin(ROBOT_INIT_YAW / 2.0)),
    )
    robots = []
    for robot_id, robot_init_pos in enumerate(robot_init_positions, start=1):
        print(f"[simulator] G1 #{robot_id} initial position: {robot_init_pos}")
        robots.append(
            scene.add_entity(
                gs.morphs.URDF(
                    file=G1_URDF,
                    pos=robot_init_pos,
                    quat=robot_init_quat,
                )
            )
        )

    init_cos_yaw = np.cos(ROBOT_INIT_YAW)
    init_sin_yaw = np.sin(ROBOT_INIT_YAW)
    init_yaw_rotation = np.array(
        ((init_cos_yaw, -init_sin_yaw, 0.0), (init_sin_yaw, init_cos_yaw, 0.0), (0.0, 0.0, 1.0)),
        dtype=np.float32,
    )
    cameras = [
        scene.add_camera(
            res=CAM_RES,
            pos=tuple(np.asarray(robot_init_pos) + init_yaw_rotation @ CAMERA_POS_REL),
            lookat=tuple(np.asarray(robot_init_pos) + init_yaw_rotation @ CAMERA_LOOKAT_REL),
            fov=45,
            GUI=False,
        )
        for robot_init_pos in robot_init_positions
    ]

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

    for robot, robot_init_pos in zip(robots, robot_init_positions):
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
        commands = recv_latest_commands(cmd_sock, commands)

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
