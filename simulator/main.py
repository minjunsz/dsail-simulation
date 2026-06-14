"""
Genesis simulator process — omnidirectional Go2 policy.

Loads the trained actor MLP directly from a rsl_rl checkpoint
(logs/<exp>/model_<ckpt>.pt) and config (logs/<exp>/cfgs.pkl).
Runs the policy at 50 Hz, follows the robot with a camera, and publishes
JPEG-compressed frames over ZeroMQ PUB socket.

Commands (ZMQ PULL) expect JSON from teleop_server: {"vx": ±1, "vy": ±1, "wz": ±1}

Run:
    python -m simulator.main --checkpoint /path/to/logs/go2-omni/model_1500.pt
"""

import argparse
import os
import pickle
import struct
import time
from io import BytesIO

import torch
import torch.nn as nn
import zmq
from PIL import Image

import genesis as gs
from genesis.utils.geom import transform_by_quat, inv_quat

# ── Sim params ────────────────────────────────────────────────────────────────
CONTROL_DT      = 0.02   # 50 Hz
KP              = 20.0
KD              = 0.5
CAM_RES         = (1280, 720)
STABILIZE_STEPS = 100

_ASSETS_DIR = os.path.join(os.path.dirname(__file__), "..", "assets")
PLANE_URDF  = os.path.join(_ASSETS_DIR, "urdf/plane/plane.urdf")
GO2_URDF    = os.path.join(_ASSETS_DIR, "urdf/go2/urdf/go2.urdf")


def load_actor(checkpoint_path: str, num_obs: int, num_actions: int, device) -> nn.Module:
    """Build actor MLP and load weights from rsl_rl checkpoint."""
    actor = nn.Sequential(
        nn.Linear(num_obs, 512), nn.ELU(),
        nn.Linear(512, 256),     nn.ELU(),
        nn.Linear(256, 128),     nn.ELU(),
        nn.Linear(128, num_actions),
    ).to(device)

    ckpt     = torch.load(checkpoint_path, map_location=device)
    actor_sd = {k[len("actor."):]: v for k, v in ckpt["model_state_dict"].items()
                if k.startswith("actor.")}
    actor.load_state_dict(actor_sd)
    actor.eval()
    return actor


def build_obs(robot, motors_dof_idx, default_dof_pos, commands, actions,
              global_gravity, obs_scales):
    """
    45-dim observation (all tensors 1-D, matching n_envs=0 API):
      ang_vel*ang_vel_scale | proj_gravity | commands*[lin_vel,lin_vel,ang_vel] |
      (dof_pos-default)*dof_pos_scale | dof_vel*dof_vel_scale | actions
    Returns (45,) tensor.
    """
    base_quat    = robot.get_quat()                                 # (4,)
    inv_q        = inv_quat(base_quat)                              # (4,)
    base_ang_vel = transform_by_quat(robot.get_ang(), inv_q)        # (3,)
    proj_gravity = transform_by_quat(global_gravity, inv_q)         # (3,)
    dof_pos      = robot.get_dofs_position(motors_dof_idx)          # (12,)
    dof_vel      = robot.get_dofs_velocity(motors_dof_idx)          # (12,)

    cmd_scale = torch.tensor(
        [obs_scales["lin_vel"], obs_scales["lin_vel"], obs_scales["ang_vel"]],
        dtype=gs.tc_float, device=gs.device,
    )

    return torch.cat([
        base_ang_vel * obs_scales["ang_vel"],                   # 3
        proj_gravity,                                            # 3
        commands * cmd_scale,                                    # 3
        (dof_pos - default_dof_pos) * obs_scales["dof_pos"],   # 12
        dof_vel * obs_scales["dof_vel"],                        # 12
        actions,                                                 # 12
    ])  # (45,)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pub-port",   type=int, default=5555)
    parser.add_argument("--cmd-port",   type=int, default=5556)
    parser.add_argument("--checkpoint", type=str, default="logs/go2-omni/model_1500.pt")
    args = parser.parse_args()

    publish_addr = f"tcp://*:{args.pub_port}"
    cmd_addr     = f"tcp://*:{args.cmd_port}"

    resume_path = args.checkpoint
    if not os.path.exists(resume_path):
        raise FileNotFoundError(f"Checkpoint not found: {resume_path}")

    # --- Load config ---
    cfgs_path = os.path.join(os.path.dirname(resume_path), "cfgs.pkl")
    if not os.path.exists(cfgs_path):
        raise FileNotFoundError(f"Config not found: {cfgs_path}")
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(open(cfgs_path, "rb"))

    # --- Genesis setup (n_envs=0 → single-env rasterizer, no batch renderer) ---
    gs.init(backend=gs.cpu)

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=CONTROL_DT, substeps=2),
        rigid_options=gs.options.RigidOptions(
            enable_self_collision=False,
            max_collision_pairs=20,
        ),
        show_viewer=False,
    )

    scene.add_entity(gs.morphs.URDF(file=PLANE_URDF, fixed=True))
    robot = scene.add_entity(
        gs.morphs.URDF(
            file=GO2_URDF,
            pos=env_cfg["base_init_pos"],
            quat=env_cfg["base_init_quat"],
        )
    )

    cam = scene.add_camera(
        res=CAM_RES,
        pos=(2.5, 0.0, 1.5),
        lookat=(0.0, 0.0, 0.4),
        fov=45,
        GUI=False,
    )
    cam.follow_entity(robot, fix_orientation=True)

    scene.build()   # no n_envs → rasterizer path, cam.render() returns (H, W, 3)

    # --- DOF setup ---
    joint_names     = env_cfg["joint_names"]
    motors_dof_idx  = torch.tensor(
        [robot.get_joint(n).dof_start for n in joint_names],
        dtype=gs.tc_int, device=gs.device,
    )
    actions_dof_idx = torch.argsort(motors_dof_idx)

    robot.set_dofs_kp([KP] * 12, motors_dof_idx)
    robot.set_dofs_kv([KD] * 12, motors_dof_idx)

    default_dof_pos = torch.tensor(
        [env_cfg["default_joint_angles"][n] for n in joint_names],
        dtype=gs.tc_float, device=gs.device,
    )  # (12,)

    init_dof_flat = torch.tensor(
        [env_cfg["default_joint_angles"][j.name] for j in robot.joints[1:]],
        dtype=gs.tc_float, device=gs.device,
    )
    robot.set_qpos(torch.cat([
        torch.tensor(env_cfg["base_init_pos"],  dtype=gs.tc_float, device=gs.device),
        torch.tensor(env_cfg["base_init_quat"], dtype=gs.tc_float, device=gs.device),
        init_dof_flat,
    ]))

    global_gravity = torch.tensor([0.0, 0.0, -1.0], dtype=gs.tc_float, device=gs.device)
    obs_scales     = obs_cfg["obs_scales"]
    action_scale   = env_cfg["action_scale"]

    # --- Load actor weights directly ---
    print(f"Loading policy from {resume_path}")
    actor = load_actor(resume_path, obs_cfg["num_obs"], env_cfg["num_actions"], gs.device)

    # --- ZeroMQ setup ---
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 2)
    sock.bind(publish_addr)
    print(f"[simulator] publishing frames on {publish_addr}")

    cmd_sock = ctx.socket(zmq.PULL)
    cmd_sock.setsockopt(zmq.RCVHWM, 1)
    cmd_sock.bind(cmd_addr)
    print(f"[simulator] receiving commands on {cmd_addr}")

    # All 1-D tensors (n_envs=0 API)
    exec_actions = torch.zeros(12, dtype=gs.tc_float, device=gs.device)
    curr_actions = torch.zeros(12, dtype=gs.tc_float, device=gs.device)
    commands     = torch.zeros(3,  dtype=gs.tc_float, device=gs.device)

    # --- Stabilisation ---
    print(f"Stabilising ({STABILIZE_STEPS} steps, {STABILIZE_STEPS * CONTROL_DT:.1f} s)...")
    zero_cmd = torch.zeros(3, dtype=gs.tc_float, device=gs.device)
    with torch.no_grad():
        for _ in range(STABILIZE_STEPS):
            obs         = build_obs(robot, motors_dof_idx, default_dof_pos,
                                    zero_cmd, curr_actions, global_gravity, obs_scales)
            new_actions = actor(obs.unsqueeze(0)).squeeze(0)   # (45,) → (1,45) → (1,12) → (12,)

            target = exec_actions * action_scale + default_dof_pos
            robot.control_dofs_position(target[actions_dof_idx], slice(6, 18))
            scene.step()

            exec_actions = curr_actions
            curr_actions = new_actions

    print("[simulator] ready")

    while True:
        t0 = time.perf_counter()

        # 1. Receive velocity command
        # teleop_server forwards {"dx": ±1, "dy": ±1, "dz": ±1} from browser keyboard
        try:
            msg = cmd_sock.recv_json(zmq.NOBLOCK)
            commands = torch.tensor([
                float(msg.get("vx", 0.0)) * 0.5,   # x_vel (m/s)
                float(msg.get("vy", 0.0)) * 0.5,   # y_vel (m/s)
                float(msg.get("wz", 0.0)) * 1.0,   # yaw_vel (rad/s)
            ], dtype=gs.tc_float, device=gs.device)
        except zmq.Again:
            pass

        # 2. Policy inference + 1-step latency
        with torch.no_grad():
            obs         = build_obs(robot, motors_dof_idx, default_dof_pos,
                                    commands, curr_actions, global_gravity, obs_scales)
            new_actions = actor(obs.unsqueeze(0)).squeeze(0)

        target = exec_actions * action_scale + default_dof_pos
        robot.control_dofs_position(target[actions_dof_idx], slice(6, 18))

        # 3. Physics step + render
        scene.step()
        rgb_arr, _, _, _ = cam.render()   # (H, W, 3) numpy array — rasterizer, no batch dim

        # Update 1-step latency buffers
        exec_actions = curr_actions
        curr_actions = new_actions

        # 4. JPEG compress; message = 8-byte little-endian timestamp (ms) + JPEG bytes
        buf = BytesIO()
        Image.fromarray(rgb_arr).save(buf, format="JPEG", quality=85)
        jpeg_bytes = buf.getvalue()

        ts_ms = int(time.time() * 1000)
        try:
            sock.send(struct.pack("<Q", ts_ms) + jpeg_bytes, zmq.NOBLOCK)
        except zmq.Again:
            pass

        elapsed = time.perf_counter() - t0
        sleep_time = CONTROL_DT - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
