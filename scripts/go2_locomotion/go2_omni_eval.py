"""
Evaluate a trained omnidirectional Go2 policy and record a demo video.
Sequence: Stabilise → Forward → Backward → Left → Right → TurnLeft → TurnRight

Run with:
    CUDA_VISIBLE_DEVICES=2 EGL_DEVICE_ID=2 python go2_omni_eval.py --ckpt 1500
    CUDA_VISIBLE_DEVICES=2 EGL_DEVICE_ID=2 python go2_omni_eval.py --ckpt 1500 -o my_demo.mp4
"""

import argparse
import os
import pickle
from importlib import metadata

try:
    try:
        if metadata.version("rsl-rl"):
            raise ImportError
    except metadata.PackageNotFoundError:
        if metadata.version("rsl-rl-lib") != "2.2.4":
            raise ImportError
except (metadata.PackageNotFoundError, ImportError) as e:
    raise ImportError("Please uninstall 'rsl_rl' and install 'rsl-rl-lib==2.2.4'.") from e

from rsl_rl.runners import OnPolicyRunner

import torch
import genesis as gs
from genesis.utils.geom import transform_by_quat, inv_quat

# ── Demo sequence ─────────────────────────────────────────────────────────────
STABILIZE_STEPS = 100  # 2 s at 50 Hz

PHASES = [
    ( 0.5,  0.0,  0.0,  50, "Forward"),
    (-0.5,  0.0,  0.0,  50, "Backward"),
    ( 0.0,  0.5,  0.0,  50, "Left"),
    ( 0.0, -0.5,  0.0,  50, "Right"),
    ( 0.0,  0.0,  1.0,  50, "TurnLeft"),
    ( 0.0,  0.0, -1.0,  50, "TurnRight"),
]

CONTROL_DT = 0.02
KP, KD     = 20.0, 0.5


def build_obs(robot, motors_dof_idx, default_dof_pos, commands, actions, global_gravity, obs_scales):
    """
    45-dim obs matching go2_env._update_observation() exactly:
      ang_vel*0.25 | projected_gravity | commands*[2,2,0.25] |
      (dof_pos-default)*1.0 | dof_vel*0.05 | actions
    All tensors remain on gs.device.
    """
    base_quat     = robot.get_quat()                              # (1, 4)
    inv_q         = inv_quat(base_quat)                           # (1, 4)
    base_ang_vel  = transform_by_quat(robot.get_ang(), inv_q)     # (1, 3)
    proj_gravity  = transform_by_quat(global_gravity, inv_q)      # (1, 3)
    dof_pos       = robot.get_dofs_position(motors_dof_idx)       # (1, 12) genesis order
    dof_vel       = robot.get_dofs_velocity(motors_dof_idx)       # (1, 12)

    commands_scale = torch.tensor(
        [obs_scales["lin_vel"], obs_scales["lin_vel"], obs_scales["ang_vel"]],
        dtype=gs.tc_float, device=gs.device,
    )

    return torch.cat([
        base_ang_vel * obs_scales["ang_vel"],                        # 3
        proj_gravity,                                                 # 3
        commands * commands_scale,                                    # 3
        (dof_pos - default_dof_pos) * obs_scales["dof_pos"],        # 12
        dof_vel * obs_scales["dof_vel"],                             # 12
        actions,                                                      # 12
    ], dim=-1)  # (1, 45)


def run_loop(robot, scene, cam, motors_dof_idx, actions_dof_idx, default_dof_pos,
             global_gravity, obs_scales, action_scale, policy, commands, n_steps):
    """
    Run n_steps with given commands, recording video frames.
    Simulates 1-step action latency matching go2_env (simulate_action_latency=True).
    """
    for _ in range(n_steps):
        obs        = build_obs(robot, motors_dof_idx, default_dof_pos,
                               commands, run_loop.curr_actions, global_gravity, obs_scales)
        new_actions = policy(obs)

        # Apply exec_actions (one step behind) to motor
        target = run_loop.exec_actions * action_scale + default_dof_pos  # (1, 12)
        robot.control_dofs_position(target[:, actions_dof_idx], slice(6, 18))
        scene.step()
        cam.render()

        # Update: exec ← curr ← new
        run_loop.exec_actions = run_loop.curr_actions
        run_loop.curr_actions = new_actions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="go2-omni")
    parser.add_argument("--ckpt",           type=int, default=1500)
    parser.add_argument("-o", "--output",   type=str, default="go2_omni_demo.mp4")
    args = parser.parse_args()

    log_dir   = f"logs/{args.exp_name}"
    cfgs_path = os.path.join(log_dir, "cfgs.pkl")
    if not os.path.exists(cfgs_path):
        raise FileNotFoundError(f"Config not found: {cfgs_path}\nRun go2_omni_train.py first.")

    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(open(cfgs_path, "rb"))
    reward_cfg["reward_scales"] = {}

    gs.init(backend=gs.cpu)

    # ── Build scene with camera before scene.build() ──────────────────────────
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=CONTROL_DT, substeps=2),
        rigid_options=gs.options.RigidOptions(
            enable_self_collision=False,
            max_collision_pairs=20,
        ),
        show_viewer=False,
    )

    _assets = os.path.join(os.path.dirname(__file__), "../../../Genesis/genesis/assets")
    scene.add_entity(gs.morphs.URDF(
        file=os.path.join(_assets, "urdf/plane/plane.urdf"), fixed=True,
    ))
    robot = scene.add_entity(gs.morphs.URDF(
        file=os.path.join(_assets, "urdf/go2/urdf/go2.urdf"),
        pos=env_cfg["base_init_pos"],
        quat=env_cfg["base_init_quat"],
    ))

    cam = scene.add_camera(res=(1280, 720), pos=(2.5, 0.0, 1.5),
                           lookat=(0.0, 0.0, 0.4), fov=45, GUI=False)
    cam.follow_entity(robot, fix_orientation=True)

    scene.build(n_envs=1)

    # ── DOF indices, PD gains, default pose ───────────────────────────────────
    joint_names    = env_cfg["joint_names"]
    motors_dof_idx = torch.tensor(
        [robot.get_joint(n).dof_start for n in joint_names],
        dtype=gs.tc_int, device=gs.device,
    )
    actions_dof_idx = torch.argsort(motors_dof_idx)

    robot.set_dofs_kp([KP] * 12, motors_dof_idx)
    robot.set_dofs_kv([KD] * 12, motors_dof_idx)

    default_dof_pos = torch.tensor(
        [env_cfg["default_joint_angles"][n] for n in joint_names],
        dtype=gs.tc_float, device=gs.device,
    ).unsqueeze(0)  # (1, 12)

    init_dof_flat = torch.tensor(
        [env_cfg["default_joint_angles"][j.name] for j in robot.joints[1:]],
        dtype=gs.tc_float, device=gs.device,
    )
    robot.set_qpos(torch.cat([
        torch.tensor(env_cfg["base_init_pos"],  dtype=gs.tc_float, device=gs.device),
        torch.tensor(env_cfg["base_init_quat"], dtype=gs.tc_float, device=gs.device),
        init_dof_flat,
    ]))

    global_gravity = torch.tensor([[0.0, 0.0, -1.0]], dtype=gs.tc_float, device=gs.device)
    obs_scales     = obs_cfg["obs_scales"]
    action_scale   = env_cfg["action_scale"]

    # ── Load policy via OnPolicyRunner ────────────────────────────────────────
    class _FakeEnv:
        num_obs            = obs_cfg["num_obs"]
        num_privileged_obs = None
        num_actions        = env_cfg["num_actions"]
        num_envs           = 1

        def get_observations(self):
            obs    = torch.zeros(1, self.num_obs, dtype=gs.tc_float, device=gs.device)
            extras = {"observations": {}}
            return obs, extras

    resume_path = os.path.join(log_dir, f"model_{args.ckpt}.pt")
    if not os.path.exists(resume_path):
        raise FileNotFoundError(f"Checkpoint not found: {resume_path}")

    runner = OnPolicyRunner(_FakeEnv(), train_cfg, log_dir, device=gs.device)
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=gs.device)

    # ── Init loop state ───────────────────────────────────────────────────────
    zeros = torch.zeros((1, 12), dtype=gs.tc_float, device=gs.device)
    run_loop.exec_actions = zeros.clone()  # applied to motor (one step delayed)
    run_loop.curr_actions = zeros.clone()  # in obs (what policy just decided)

    output_path = os.path.join(os.path.dirname(__file__), args.output)
    cam.start_recording()

    print(f"Stabilising ({STABILIZE_STEPS} steps, {STABILIZE_STEPS * CONTROL_DT:.1f} s)...")
    zero_cmd = torch.zeros((1, 3), dtype=gs.tc_float, device=gs.device)
    with torch.no_grad():
        run_loop(robot, scene, cam, motors_dof_idx, actions_dof_idx, default_dof_pos,
                 global_gravity, obs_scales, action_scale, policy,
                 commands=zero_cmd, n_steps=STABILIZE_STEPS)

    for x_vel, y_vel, yaw_vel, steps, label in PHASES:
        cmd = torch.tensor([[x_vel, y_vel, yaw_vel]], dtype=gs.tc_float, device=gs.device)
        print(f"  {label:10s}  x={x_vel:+.1f}  y={y_vel:+.1f}  yaw={yaw_vel:+.1f}  ({steps} steps)")
        with torch.no_grad():
            run_loop(robot, scene, cam, motors_dof_idx, actions_dof_idx, default_dof_pos,
                     global_gravity, obs_scales, action_scale, policy,
                     commands=cmd, n_steps=steps)

    cam.stop_recording(save_to_filename=output_path, fps=int(1.0 / CONTROL_DT))
    print(f"\nSaved: {output_path}")


if __name__ == "__main__":
    main()
