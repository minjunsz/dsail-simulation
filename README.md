## Installation

```bash
uv venv --python 3.12
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126
uv pip install genesis-world pyzmq aiohttp aiortc av
```

## Launch

```bash
CUDA_VISIBLE_DEVICES=2 uv run python -m simulator.main
uv run python -m teleop_server.main
```