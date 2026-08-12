## Installation

```bash
uv venv --python 3.12
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126
uv pip install genesis-world pyzmq aiohttp aiortc av
```

## Usage

### Humanoid controller

The simulator uses Unitree's official 12-DoF G1 URDF and recurrent locomotion
policy. They are vendored in `assets/urdf/g1/` and
`assets/checkpoints/g1-locomotion/`. The policy accepts forward, lateral, and
yaw velocity commands at 50 Hz.

```bash
CUDA_VISIBLE_DEVICES=3 EGL_DEVICE_ID=3 uv run python -m simulator.main
```

The existing Go2 training/evaluation scripts remain available as references,
but are no longer used by the simulator entry point.

### Launch simulator

```bash
# Default ports: pub=5565, cmd=5566, http=8090
CUDA_VISIBLE_DEVICES=3 EGL_DEVICE_ID=3 python -m simulator.main
uv run python -m teleop_server.main
```

```bash
# Custom ports example
CUDA_VISIBLE_DEVICES=3 EGL_DEVICE_ID=3 uv run python -m simulator.main --pub-port 6000 --cmd-port 6001
uv run python -m teleop_server.main --sub-port 6000 --cmd-port 6001 --http-port 9000 --host 0.0.0.0
```

The G1 asset and pretrained policy come from Unitree Robotics'
`unitree_rl_gym` repository (BSD-3-Clause), revision
`276801e46c5d433564f24658bac64f254b7d2d4b`. The upstream license is retained
at `assets/urdf/g1/LICENSE.unitree_rl_gym`.
