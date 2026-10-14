## Installation

```bash
uv venv --python 3.12
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126
uv pip install genesis-world pyzmq aiohttp aiortc av
```

## Usage

### Conference hall demo

Users and presenter agents are cylinders with a bar at face height showing
their heading; the scene is defined by `assets/conference_hall/layout.yaml`
(see `project_design/06_conference_hall_demo.md`). Build the hall once, then
start the simulator and the teleop server:

```bash
uv run python scripts/make_conference_hall.py
# Default ports: pub=5555, cmd=5556, http=8090
CUDA_VISIBLE_DEVICES=0 EGL_DEVICE_ID=0 uv run python -m simulator.main
uv run python -m teleop_server.main
```

Open `http://<host>:8090`: W/S forward/back, A/D strafe, Q/E turn. The first two
browsers get User 1 and User 2.

```bash
# Custom ports example
CUDA_VISIBLE_DEVICES=0 EGL_DEVICE_ID=0 uv run python -m simulator.main --pub-port 6000 --cmd-port 6001
uv run python -m teleop_server.main --sub-port 6000 --cmd-port 6001 --http-port 9000 --host 0.0.0.0
```

### Legacy G1 humanoid simulator

The previous simulator (Unitree G1 with its recurrent locomotion policy, DSAIL
scan / demo mesh / Nyx 3DGS scenes) is kept as `simulator.legacy_g1_main`:

```bash
CUDA_VISIBLE_DEVICES=0 EGL_DEVICE_ID=0 uv run python -m simulator.legacy_g1_main
```

The G1 asset and pretrained policy come from Unitree Robotics'
`unitree_rl_gym` repository (BSD-3-Clause), revision
`276801e46c5d433564f24658bac64f254b7d2d4b`. The upstream license is retained
at `assets/urdf/g1/LICENSE.unitree_rl_gym`.
