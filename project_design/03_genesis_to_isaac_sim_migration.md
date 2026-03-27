# Genesis → Isaac Sim 마이그레이션

## 개요

현재 `genesis-world` 기반으로 구현된 시뮬레이터(`simulator/`)를 **NVIDIA Isaac Sim**으로 교체한다.
`teleop_server/`와 `static/`은 ZeroMQ 인터페이스가 유지되는 한 변경 없음.

---

## 주요 차이점 요약

| 항목 | Genesis | Isaac Sim |
|------|---------|-----------|
| 설치 방식 | `pip install genesis-world` | 독립 설치 (Omniverse / pip extras) |
| Python 실행 환경 | 일반 venv | Isaac Sim 전용 Python 또는 pip 설치 |
| Scene 진입점 | `gs.Scene()` | `SimulationApp` + `World` |
| 물리 엔진 | Genesis 자체 | PhysX (NVIDIA) |
| 렌더 엔진 | Genesis GPU backend | RTX / Hydra (Omniverse) |
| 오브젝트 모델 | `gs.morphs.*` | `omni.isaac.core.objects.*` (USD Prim 기반) |
| 카메라 / 프레임 캡처 | `cam.render()` → numpy | `omni.replicator` annotator API |
| Headless 모드 | `show_viewer=False` | `SimulationApp({"headless": True})` |
| 위치 제어 | `entity.set_pos()` | `prim.set_world_pose()` |

---

## 변경 대상 파일

```
변경 필요
├── simulator/main.py          ← 전면 재작성
├── requirements.txt           ← genesis-world 제거, isaac sim 패키지 추가
└── README.md                  ← 설치 및 실행 방법 업데이트

변경 불필요 (ZeroMQ 인터페이스 동일)
├── teleop_server/main.py
├── static/index.html
└── static/client.js
```

---

## Step-by-Step 작업 목록

### Step A. 환경 설정

#### A-1. Isaac Sim 설치

Isaac Sim은 두 가지 방식으로 설치 가능:

**방법 1 — pip (권장, Isaac Sim 4.x 이상)**
```bash
pip install isaacsim-rl isaacsim-replicator isaacsim-robot-setup \
    --extra-index-url https://pypi.nvidia.com
```
> 내부적으로 Omniverse Kit 런타임을 포함하므로 별도 Launcher 불필요.

**방법 2 — Omniverse Launcher 경유**
- `~/.local/share/ov/pkg/isaac-sim-4.x.x/` 에 설치
- 실행 시 해당 디렉토리의 `python.sh` 사용

#### A-2. requirements.txt 수정

```diff
-genesis-world
+# Isaac Sim은 별도 설치 (위 방법 참조)
+# 아래는 teleop_server 및 공통 의존성
 pyzmq
 aiohttp
 aiortc
 av
 Pillow
 numpy
```

---

### Step B. `simulator/main.py` 재작성

#### B-1. 앱 초기화 (Genesis `gs.init()` 대응)

```python
# Genesis
import genesis as gs
gs.init(backend=gs.gpu)

# Isaac Sim
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})
```

- `SimulationApp`은 **파일 최상단에서 가장 먼저** 생성해야 함 (omni 모듈들은 이후에 import)
- `renderer`: `"RayTracedLighting"` (RTX) 또는 `"PathTracing"`

#### B-2. Scene / World 구성 (Genesis `gs.Scene()` 대응)

```python
# Genesis
scene = gs.Scene(show_viewer=False)
scene.add_entity(gs.morphs.Plane())
scene.build()

# Isaac Sim
from omni.isaac.core import World
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()
```

#### B-3. 오브젝트 추가 (Genesis `gs.morphs.Box()` 대응)

```python
# Genesis
box = scene.add_entity(gs.morphs.Box(size=(0.3, 0.3, 0.3), pos=(0.0, 0.0, 0.5)))

# Isaac Sim
from omni.isaac.core.objects import DynamicCuboid
import numpy as np
box = world.scene.add(
    DynamicCuboid(
        prim_path="/World/box",
        name="box",
        position=np.array([0.0, 0.0, 0.5]),
        size=0.3,         # 단일 float (정육면체) 또는 np.array([x, y, z])
        mass=1.0,
    )
)
```

> **주의**: Isaac Sim의 `DynamicCuboid`는 PhysX 물리 적용 오브젝트.
> 키네마틱(위치 직접 지정) 제어를 원하면 `kinematic_enabled=True` 옵션 또는
> `StaticCuboid` + `set_world_pose()` 조합 사용.

#### B-4. 카메라 설정 (Genesis `scene.add_camera()` 대응)

```python
# Genesis
cam = scene.add_camera(res=(1280, 720), pos=(3.5, 0, 2.5), lookat=(0, 0, 0.5), fov=30)

# Isaac Sim — omni.replicator 사용
import omni.replicator.core as rep

camera = rep.create.camera(
    position=(3.5, 0, 2.5),
    look_at=(0, 0, 0.5),
    focal_length=18.0,   # fov 대신 focal_length로 조정
)
render_product = rep.create.render_product(camera, resolution=(1280, 720))
rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annotator.attach([render_product])
```

#### B-5. 렌더 루프 (Genesis `scene.step()` + `cam.render()` 대응)

```python
# Genesis
scene.step()
rgb, depth, seg, normal = cam.render()

# Isaac Sim
world.step(render=True)               # physics + rendering 동시 진행
rep.orchestrator.step(rt_subframes=1) # replicator 렌더 동기화
rgb_data = rgb_annotator.get_data()   # dict, rgb_data["data"]: np.ndarray (H, W, 4) RGBA
frame = rgb_data["data"][:, :, :3]   # RGBA → RGB
```

> `world.step(render=True)`만으로 화면 갱신은 되지만, annotator로 numpy를 받으려면
> `rep.orchestrator.step()` 호출이 추가로 필요할 수 있음 (버전에 따라 상이).

#### B-6. 위치 제어 (Genesis `entity.set_pos()` 대응)

```python
# Genesis
pos = box.get_pos()
box.set_pos(pos + delta)

# Isaac Sim
position, orientation = box.get_world_pose()
box.set_world_pose(position=position + delta)
```

#### B-7. 앱 종료 처리

```python
# Genesis — 별도 종료 처리 없음

# Isaac Sim
simulation_app.close()
```

루프 탈출 시 반드시 호출해야 Omniverse Kit 프로세스가 정상 종료됨.

---

### Step C. 렌더링 검증 항목

| 항목 | 확인 방법 |
|------|----------|
| Headless 모드 실행 | `nvidia-smi`로 GPU 메모리 점유 확인, 화면 창 미출력 |
| RGB numpy 형상 | `assert frame.shape == (720, 1280, 3)` |
| dtype | `assert frame.dtype == np.uint8` |
| 30fps 유지 여부 | ZeroMQ timestamp 기반 fps 측정 (기존 방식 동일) |
| 브라우저 스트림 | teleop_server 실행 후 동일한 WebRTC 파이프라인으로 검증 |

---

### Step D. 실행 방법 변경

#### 기존 (Genesis)
```bash
CUDA_VISIBLE_DEVICES=2 uv run python -m simulator.main
```

#### 변경 후 (Isaac Sim pip 방식)
```bash
CUDA_VISIBLE_DEVICES=2 python -m simulator.main
# (Isaac Sim pip 패키지가 설치된 venv 활성화 상태에서)
```

#### 변경 후 (Omniverse Launcher 방식)
```bash
CUDA_VISIBLE_DEVICES=2 ~/.local/share/ov/pkg/isaac-sim-4.x.x/python.sh simulator/main.py
```

> Isaac Sim pip 방식을 택하면 기존 uv venv 워크플로우를 유지할 수 있으나,
> `isaacsim` 패키지 크기가 수 GB에 달하므로 전용 venv 분리 권장.

---

## API 대응 빠른 참조

| Genesis | Isaac Sim |
|---------|-----------|
| `gs.init(backend=gs.gpu)` | `SimulationApp({"headless": True})` |
| `gs.Scene(show_viewer=False)` | `World()` |
| `scene.add_entity(gs.morphs.Plane())` | `world.scene.add_default_ground_plane()` |
| `scene.add_entity(gs.morphs.Box(...))` | `world.scene.add(DynamicCuboid(...))` |
| `scene.add_camera(res, pos, lookat, fov)` | `rep.create.camera(...)` + `rep.create.render_product(...)` |
| `scene.build()` | `world.reset()` |
| `scene.step()` | `world.step(render=True)` |
| `cam.render()` → `(rgb, ...)` | `rep.orchestrator.step()` + `annotator.get_data()` |
| `entity.get_pos()` | `prim.get_world_pose()[0]` |
| `entity.set_pos(pos)` | `prim.set_world_pose(position=pos)` |
| (암묵적 종료) | `simulation_app.close()` |

---

## 영향 없는 구성 요소

다음 구성 요소는 ZeroMQ 메시지 포맷(`[8B timestamp | JPEG bytes]`)이 동일하게 유지되므로 **수정 불필요**:

- `teleop_server/main.py`: ZeroMQ SUB 수신 → WebRTC 전송
- `static/index.html`, `static/client.js`: WebRTC 재생 + WASD 입력
- ZeroMQ 소켓 구성 (PUB port 5555, PULL port 5556)

---

## 주의사항

1. **import 순서**: `SimulationApp` 생성 전에 `omni.*` 모듈을 import하면 충돌. `SimulationApp` 먼저 생성 후 나머지 import.
2. **RGBA 출력**: Isaac Sim annotator는 기본적으로 RGBA(4채널)를 반환. ZeroMQ 전송 전 `[:, :, :3]`으로 슬라이싱.
3. **PhysX 키네마틱 제어**: `DynamicCuboid`에 중력이 적용되므로, WASD 이동 시 키네마틱 모드 활성화 필요 (`kinematic_enabled=True` 또는 velocity 0 유지).
4. **첫 프레임 지연**: `world.reset()` 후 첫 몇 스텝은 렌더러 워밍업으로 인해 프레임이 검게 나올 수 있음. 기존 `LatestFrame` fallback 로직으로 커버 가능.
5. **GPU 메모리**: Omniverse RTX 렌더러는 Genesis 대비 VRAM 사용량이 클 수 있음. RTX 4090(24GB) 환경에서는 문제 없을 것으로 예상.
