## Installation

```bash
sudo apt-get update && sudo apt-get install libglu1-mesa

uv venv --python 3.11
uv pip install isaacsim[all]==5.1.0 --extra-index-url https://pypi.nvidia.com
uv pip install pyzmq aiohttp aiortc av usd-core

# activate venv
isaacsim # accept EULA
python -m isaacsim --generate-vscode-settings # generate vscode settings
```

## Launch

```bash
CUDA_VISIBLE_DEVICES=2 uv run python -m simulator.main
uv run python -m teleop_server.main
```