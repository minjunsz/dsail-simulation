"""Genesis simulator process for the conference hall demo.

Assembles the hall (simulator/hall.py), one presenter Agent per poster board
and the browser-driven Users (simulator/avatar.py), then runs a 50 Hz loop:
receive move commands → step avatars → render and publish one JPEG per user.
Message formats and avatar ids are defined in common/protocol.py.

Build the hall once, then run:
    uv run python scripts/make_conference_hall.py
    CUDA_VISIBLE_DEVICES=0 EGL_DEVICE_ID=0 uv run python -m simulator.main

The previous G1 humanoid simulator lives in simulator/legacy_g1_main.py.
"""

import argparse
import time

import numpy as np
import zmq

import genesis as gs

from common.gaze import in_view
from common.hall_layout import DEFAULT_LAYOUT, load_layout
from common.protocol import USER_COUNT, pack_frame, parse_move
from simulator.avatar import Agent, User
from simulator.hall import Hall

CONTROL_DT = 0.02   # 50 Hz target
MAX_STEP_DT = 0.1   # cap the integration step after a stall (e.g. a slow frame)
USER_SPAWN_SEPARATION = 1.5


class Simulator:
    def __init__(self, layout, pub_port, cmd_port):
        # n_envs=0 → single-env rasterizer. Everything is kinematic: no gravity,
        # and blocking is resolved in 2D by Avatar.step instead of the solver.
        gs.init(backend=gs.gpu, logging_level="warning", performance_mode=True)
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=CONTROL_DT, gravity=(0.0, 0.0, 0.0)),
            show_viewer=False,
        )

        self.hall = Hall(self.scene, layout)
        self.agents = [Agent(self.scene, layout, board) for board in layout["boards"]]
        print(f"[simulator] conference hall: {len(self.agents)} boards and agents")

        spawn_xy = np.asarray(layout["spawn"]["xy"], dtype=np.float64)
        spawn_yaw = float(np.radians(layout["spawn"]["yaw_deg"]))
        lateral = np.array((-np.sin(spawn_yaw), np.cos(spawn_yaw)))
        self.users = []
        for user_id in range(1, USER_COUNT + 1):
            offset = (user_id - 1 - (USER_COUNT - 1) / 2.0) * USER_SPAWN_SEPARATION
            xy = spawn_xy + offset * lateral
            print(f"[simulator] user {user_id} spawn: {xy.tolist()}")
            self.users.append(User(self.scene, layout, user_id, xy, spawn_yaw))

        self.avatars = {avatar.avatar_id: avatar for avatar in (*self.users, *self.agents)}
        self.scene.build()
        for agent in self.agents:
            agent.bind_materials(self.scene)
        gaze = layout["gaze"]
        self.gaze_distance = gaze["max_distance"]
        self.gaze_half_angle = np.radians(gaze["half_angle_deg"])
        self.name_tag_distance = layout["avatar"]["name_tag_max_distance"]

        ctx = zmq.Context()
        self.pub_sock = ctx.socket(zmq.PUB)
        self.pub_sock.setsockopt(zmq.SNDHWM, USER_COUNT * 2)
        self.pub_sock.bind(f"tcp://*:{pub_port}")
        print(f"[simulator] publishing frames on tcp://*:{pub_port}")
        self.cmd_sock = ctx.socket(zmq.PULL)
        self.cmd_sock.setsockopt(zmq.RCVHWM, 100)
        self.cmd_sock.bind(f"tcp://*:{cmd_port}")
        print(f"[simulator] receiving commands on tcp://*:{cmd_port}")

    def recv_commands(self, now):
        """Drain pending input and keep the newest command for each avatar.

        A single conflated socket cannot be used here because a command for one
        avatar could replace another's latest command.
        """
        while True:
            try:
                msg = self.cmd_sock.recv_json(zmq.NOBLOCK)
            except zmq.Again:
                break
            parsed = parse_move(msg)
            if parsed is None or parsed[0] not in self.avatars:
                continue
            avatar_id, command = parsed
            self.avatars[avatar_id].set_command(command, now)

    def step_avatars(self, now, dt):
        # Each avatar treats every other one as an obstacle at its current pose.
        for avatar in self.avatars.values():
            others = [other.footprint for other in self.avatars.values() if other is not avatar]
            avatar.step(self.hall, others, dt, now)
        self.scene.step()
        for user in self.users:
            user.update_camera()

    def publish_frames(self):
        ts_ms = int(time.time() * 1000)
        for user in self.users:
            # Each user sees the agents they are looking at highlighted.
            for agent in self.agents:
                agent.set_highlight(
                    in_view(user.xy, user.yaw, agent.xy, self.gaze_distance, self.gaze_half_angle)
                )
            # Name tags for everyone nearby except the user themself.
            name_tags = [
                (avatar.tag_anchor, avatar.name, getattr(avatar, "highlighted", False))
                for avatar in self.avatars.values()
                if avatar is not user and np.hypot(*(avatar.xy - user.xy)) <= self.name_tag_distance
            ]
            try:
                jpeg = user.render_jpeg(name_tags)
                self.pub_sock.send(pack_frame(user.avatar_id, ts_ms, jpeg), zmq.NOBLOCK)
            except zmq.Again:
                pass

    def run(self):
        print("[simulator] ready")
        stat_sim = stat_render = stat_total = 0.0
        stat_n = 0
        last_t = time.perf_counter()
        while True:
            t0 = time.perf_counter()
            # Integrate wall-clock time so speeds stay in m/s when the loop
            # runs below 50 Hz (e.g. on a shared GPU).
            dt = min(t0 - last_t, MAX_STEP_DT)
            last_t = t0
            self.recv_commands(t0)
            self.step_avatars(t0, dt)
            t_sim = time.perf_counter()
            self.publish_frames()

            elapsed = time.perf_counter() - t0
            stat_sim += t_sim - t0
            stat_render += elapsed - (t_sim - t0)
            stat_total += elapsed
            stat_n += 1
            if stat_n == 100:
                print(
                    f"[simulator] avg over {stat_n} steps: sim {stat_sim / stat_n * 1e3:.1f} ms, "
                    f"render+publish ({len(self.users)} cams) {stat_render / stat_n * 1e3:.1f} ms, "
                    f"loop {stat_total / stat_n * 1e3:.1f} ms -> {stat_n / stat_total:.1f} FPS "
                    f"(target {1 / CONTROL_DT:.0f})",
                    flush=True,
                )
                stat_sim = stat_render = stat_total = 0.0
                stat_n = 0
            sleep_time = CONTROL_DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pub-port", type=int, default=5555)
    parser.add_argument("--cmd-port", type=int, default=5556)
    parser.add_argument("--layout", type=str, default=DEFAULT_LAYOUT,
                        help="Conference hall layout YAML (build it with scripts/make_conference_hall.py).")
    args = parser.parse_args()
    Simulator(load_layout(args.layout), args.pub_port, args.cmd_port).run()


if __name__ == "__main__":
    main()
