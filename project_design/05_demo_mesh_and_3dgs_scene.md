# Step 5: Demo Mesh / 3DGS Scene

## 목표

`assets/dsail_mesh.ply`가 없는 환경에서도 시뮬을 돌릴 수 있도록 씬 메시를 교체 가능하게 하고,
PoC_1의 garden 3DGS(2DGS splat)를 Nyx path tracer로 렌더링해 속도를 확인한다.

## 구현

- `--scene-mesh`, `--scene-scale`, `--scene-z-offset`로 씬 메시와 변환을 지정한다(기본값은 기존 DSAIL 설정).
- face 수가 5,000 이하인 메시는 collision용 decimation을 건너뛴다.
- `scripts/make_demo_mesh.py`: 벽 3면(+X 개방), 박스/기둥/경사로로 된 vertex-color 데모 씬(`assets/demo_mesh.ply`).
- `--splat <ply>`: 씬 메시는 collision 전용, 시각은 Nyx `LightFieldAsset`(GaussianField)으로 렌더링.
  - Nyx는 fixed link가 merge된 URDF를 거부하므로 ground는 숨긴 `gs.morphs.Plane`, G1은 `merge_fixed_links=False`.
  - 카메라는 로봇별 `NyxCameraOptions` sensor, TPS 추적은 `update_camera_pose`.
  - 조명이 하나도 없으면 Nyx가 segfault하므로 directional light 1개를 둔다.
- 루프에서 100 step마다 sim / render+publish(JPEG 포함) / FPS 평균을 출력한다.

## 실행

```bash
# 데모 메시 (rasterizer)
CUDA_VISIBLE_DEVICES=0 EGL_DEVICE_ID=0 uv run python -m simulator.main \
    --scene-mesh assets/demo_mesh.ply --scene-scale 1.0

# garden 3DGS (Nyx, GPU 0). garden 지면이 z≈-0.93이므로 +0.93 올린다.
# Vulkan은 CUDA_VISIBLE_DEVICES를 따르지 않으므로 DRI_PRIME(PCI 주소)으로 같은 GPU를 고정한다.
# 안 하면 Nyx가 "Failed to import external memory into cuda"로 죽는다.
CUDA_VISIBLE_DEVICES=0 EGL_DEVICE_ID=0 VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    DRI_PRIME=pci-0000_3a_00_0 MESA_VK_DEVICE_SELECT_FORCE_DEFAULT_DEVICE=1 \
    uv run python -m simulator.main --scene-mesh assets/garden/garden_collision.obj \
    --scene-scale 1.0 --scene-z-offset 0.93 --splat assets/garden/garden_visual.ply --spp 16
```

`assets/garden/`은 PoC_1에서 복사한 파일로 git에 포함하지 않는다(.gitignore).

## 실행 검증 (2026-10-13, A40 GPU 0, 1280x720 카메라 2대)

| 씬 / 렌더러 | sim | render+publish | FPS (목표 50) |
|---|---|---|---|
| 데모 메시 / rasterizer | 6.1 ms | 30.3 ms | 27.5 |
| garden 3DGS / Nyx spp 4 | 5.4 ms | 18.6 ms | 41.8 |
| garden 3DGS / Nyx spp 16 | 5.4 ms | 19.3 ms | 40.4 |
| garden 3DGS / Nyx spp 64 | 5.3 ms | 22.0 ms | 36.6 |

render+publish에는 1280x720 JPEG 인코딩 2회가 포함된다.
