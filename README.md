## Installation

```bash
uv venv --python 3.12
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126
uv pip install genesis-world pyzmq aiohttp aiortc av
```

## Usage

### (Optional) Train robot control policy

Pre-trained checkpoints are available in `assets/checkpoints/go2-omni/`.

```bash
CUDA_VISIBLE_DEVICES=2 EGL_DEVICE_ID=2 uv run scripts/go2_locomotion/go2_omni_train.py --max_iterations 1501
CUDA_VISIBLE_DEVICES=2 EGL_DEVICE_ID=2 uv run scripts/go2_locomotion/go2_omni_eval.py --ckpt 1500 -o my_demo.mp4
```

### Launch simulator

```bash
# Default ports: pub=5565, cmd=5566, http=8090
CUDA_VISIBLE_DEVICES=2 EGL_DEVICE_ID=2 python -m simulator.main --checkpoint assets/checkpoints/go2-omni/model_1500.pt 
uv run python -m teleop_server.main
```

```bash
# Custom ports example
CUDA_VISIBLE_DEVICES=2 uv run python -m simulator.main --pub-port 6000 --cmd-port 6001 --checkpoint assets/checkpoints/go2-omni/model_1500.pt
uv run python -m teleop_server.main --sub-port 6000 --cmd-port 6001 --http-port 9000 --host 0.0.0.0
```