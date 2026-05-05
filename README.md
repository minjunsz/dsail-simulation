## Installation

```bash
uv venv --python 3.12
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126
uv pip install genesis-world pyzmq aiohttp aiortc av
```

## Launch

```bash
# Default ports: pub=5565, cmd=5566, http=8096
CUDA_VISIBLE_DEVICES=2 uv run python -m simulator.main
uv run python -m teleop_server.main
```

```bash
# Custom ports example
CUDA_VISIBLE_DEVICES=2 uv run python -m simulator.main --pub-port 6000 --cmd-port 6001
uv run python -m teleop_server.main --sub-port 6000 --cmd-port 6001 --http-port 9000 --host 0.0.0.0
```