# G1 humanoid locomotion

## Decision

Replace the simulator's Go2 runtime with Unitree G1 using the official
`unitree_rl_gym` 12-DoF URDF and pretrained recurrent policy. The engine file
`Genesis/genesis/engine/simulator.py` is a generic solver manager and does not
load Go2; the application-level robot loading point is
`genesis_sim/simulator/main.py`.

Source revision: `276801e46c5d433564f24658bac64f254b7d2d4b`.

## Policy contract

- Target control rate: 50 Hz; Genesis physics substep: 0.005 s (4 substeps).
- Actions: 12 lower-body position offsets, scaled by 0.25 rad.
- Observation: 47 values comprising body angular velocity, projected gravity,
  3-D velocity command, joint position error, joint velocity, previous action,
  and sine/cosine gait phase.
- Gait period: 0.8 s.
- Policy: TorchScript LSTM with persistent recurrent state.
- Command ranges used upstream: `vx`, `vy`, and `wz` each in `[-1, 1]`.

## Validation

The policy was run in Genesis for 500 policy steps (10 seconds) on GPU 3 with
a 0.5 m/s forward command. G1 stayed upright and moved 4.20 m. Its sampled
body-frame forward velocity was 0.48 m/s at 2 s, 0.43 m/s at 5 s, and 0.50 m/s
at 10 s; final pelvis height was 0.768 m.

The complete application path also reached `ready` with all 362 DSAIL
collision proxies, accepted a ZMQ forward command, and published a valid
1280x720 JPEG in which the upright G1 and scene were correctly framed.

Because the official policy transfers successfully into Genesis, additional
training is not currently justified. The upstream G1 configuration applies
friction/base-mass randomization and periodic pushes, but its checked-in
terrain setting is a plane. If the policy later proves insufficient on the
DSAIL geometry, the next step is Genesis fine-tuning with a progressive mix of
low steps, ramps, height noise, randomized friction/mass, and lateral pushes.

## Teleoperation latency

Browser keyboard state is produced at roughly 30 Hz. Both the command channel
and JPEG subscriber use ZeroMQ conflation, and the simulator drains all pending
commands before every policy step. This prevents old key-down/key-up states or
old video frames from accumulating when rendering is slower than the producer.
The browser transmits immediately when key state changes and repeats the active
state as a 10 Hz heartbeat while a key remains held. A stress test that queued
1,000 superseded commands delivered only the final command, and the measured
command-to-render timestamp in the complete DSAIL application was 25 ms.
