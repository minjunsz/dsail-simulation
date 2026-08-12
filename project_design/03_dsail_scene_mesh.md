# Step 3: DSAIL Scene Mesh

## 목표

기존 ground plane을 유지하면서 `assets/dsail_mesh.ply`를 고정된 실내 환경으로 로드하고,
Go2가 메시 표면 및 장애물과 충돌하도록 구성한다.

## 구현

- ground plane은 기존과 동일하게 먼저 추가한다.
- Genesis 0.4.x는 PLY 경로를 직접 지원하지 않으므로 `trimesh`로 PLY를 읽은 뒤
  `gs.morphs.MeshSet`에 전달하며, `fixed=True`로 추가한다.
- 원본 스캔의 방향과 XY 중심은 유지한다.
- DSAIL visual/collision mesh 모두 원점을 기준으로 0.85배 축소하며 z offset은 적용하지 않는다.
- Go2는 변환된 DSAIL +X 경계에서 0.75m 바깥에 생성하여 초기 상태에서 사물이나 벽에 끼지 않는다.
- 카메라는 Go2 기준 위치 `(-2.5, 0.0, 1.08)`에서 로봇을 바라본다. base의 위치와 yaw만
  smoothing하여 추적하고 world Z축을 up으로 유지하므로, 로봇을 따라 움직이는 TPS 구도를
  유지하면서 보행 중 roll/pitch 진동은 카메라에 전달되지 않는다.
- 원본 visual entity는 `convexify=False`, collision proxy entity는 이미 convex인 hull들만
  포함하므로 `convexify=True`를 사용한다.
- 원본 시각 메시(약 157만 face)는 visual-only entity로 유지한다. 별도의 약 5,000-face
  collision-only mesh를 미리 생성하여 GPU가 원본 형상으로 collision SDF를 만들지 않게 한다.
- 실내 shell 전체를 한 번에 CoACD 처리하면 convex hull이 빈 실내까지 채운다. 이를 피하기 위해
  기본 0.5m 공간 cell별로 삼각형을 묶고 각 cell에서 local convex hull을 만든다. 얇은 바닥/벽에도
  collision volume이 생기도록 양방향 1cm extrusion을 적용한다.
- `--collision-cell-size`를 줄일수록 hull 수와 정확도가 증가하고 성능 및 GPU 메모리 비용도 증가한다.
  `--collision-half-thickness`로 스캔 표면의 collision 두께를 조절할 수 있다.
- PLY의 원래 좌표 범위는 그대로 사용한다. z 최솟값이 약 -0.11m이므로 기존 ground와
  일부 겹치지만, ground 높이와 스캔 좌표계를 임의로 변경하지 않는다.

## 검증 기준

- [x] 기존 ground entity를 유지한 채 scene build에 성공한다.
- [x] DSAIL 메시가 원본 좌표계로 로드되고 카메라 렌더가 성공한다.
- [x] DSAIL의 local convex proxy들이 collision detection에 참여한다.
- [x] scene build 시 collision mesh를 약 5,000 face로 단순화한다.
- [x] 단순 collision mesh를 local convex hull들로 분해한다.

## 실행 검증 (2026-08-12)

```bash
EGL_DEVICE_ID=3 CUDA_VISIBLE_DEVICES=3 uv run python -m simulator.main \
  --checkpoint assets/checkpoints/go2-omni/model_1500.pt
```

- Genesis 0.4.3에서 scene build, 100-step stabilization, `[simulator] ready` 진입을 확인했다.
- ZeroMQ PUB에서 `1280x720 RGB JPEG` 프레임을 실제 수신했다.
- `nvidia-smi`에서 simulator PID가 GPU 3의 graphics process로 표시되고 약 628 MiB를 사용했다.
- 최초 검증 당시 physics backend가 `gs.cpu`여서 약 3 FPS였다. 이후 physics backend도
  `gs.gpu`로 전환했으며 아래의 후속 검증 결과를 참고한다.

## GPU physics 후속 검증 (2026-08-12)

- `gs.init(backend=gs.gpu)`로 변경하고 같은 장치 설정으로 재실행했다.
- Genesis가 GPU 3의 NVIDIA A40을 `gs.cuda` backend로 인식했다.
- 원본 visual mesh와 단순 collision mesh를 분리하여, collision 전처리 결과는 5,839 face였다.
- scene build, stabilization, `[simulator] ready`, `1280x720 RGB JPEG` 수신을 모두 확인했다.
- 중간 단계인 단일 non-convex collision mesh는 약 2.2 FPS였다. 최종적으로 0.75m spatial
  decomposition을 적용하여 260개의 local convex hull을 생성했다.
- 최종 구성은 초기 위치의 자유 공간을 막지 않고 100-step stabilization을 통과했다.
- 물리-only stabilization은 약 150 FPS, 1280x720 렌더와 JPEG 전송을 포함한 전체 루프는
  약 33~34 FPS로 목표 30 FPS를 달성했다.
- 최종 simulator는 GPU 3에서 약 5.7 GiB를 사용했다.

## High-accuracy proxy 검증 (2026-08-12)

- 기본 cell 크기를 0.75m에서 0.5m로 줄이고 extrusion을 양방향 1cm로 줄였다.
- local convex hull 수는 260개에서 554개로 증가했다.
- GPU 3에서 scene build와 stabilization, `[simulator] ready`, JPEG 수신을 확인했다.
- 물리-only 구간은 약 60 FPS, 렌더 포함 구간은 약 25~28 FPS였으며 GPU 메모리는 약 9.5 GiB였다.
- 정확도와 속도는 `--collision-cell-size`로 조정할 수 있다. 0.5가 고정밀 기본값이고,
  30 FPS가 더 중요하면 0.75를 사용한다.

## CoACD 참고

Genesis에서는 별도 CoACD CLI 대신 `gs.morphs.Mesh(..., convexify=True,
decompose_object_error_threshold=0.0, coacd_options=...)`로 scene entity를 추가할 때 자동 실행한다.
그러나 DSAIL처럼 방 전체를 감싸는 하나의 shell에 global CoACD를 적용하면 hull이 빈 실내를
가로질러 로봇 초기 위치를 막았다. 현재 구현은 공간 cell별 local convex decomposition을 사용하므로
CoACD 전처리를 따로 실행할 필요가 없다.

## Scene scale / spawn 검증 (2026-08-12)

- 초기에는 DSAIL을 0.5배 축소하고 0.08m 올렸으나, 이후 0.85배와 z offset 0m로 조정했다.
- Go2는 항상 변환된 DSAIL의 +X 경계에서 0.75m 바깥에 자동 배치된다.
- 134개의 local convex proxy로 scene build와 stabilization을 통과하고 JPEG 프레임을 수신했다.
- GPU 3에서 약 3.7 GiB를 사용했으며 렌더 포함 실행은 대체로 30 FPS 이상이었다.
- Go2 기준 카메라 pose 적용 후 실제 프레임에서 로봇이 화면 중앙에 표시되고 DSAIL scene이
  전방에 함께 렌더링되는 것을 확인했다.

## 0.85 scale 검증 (2026-08-12)

- DSAIL scale을 0.85, z offset을 0m로 변경했다.
- 변환 bounds는 `x=[-3.852, 3.852]`, `y=[-2.992, 2.992]`,
  `z=[-0.094, 3.152]m`였으며 local convex proxy 408개가 생성됐다.
- Go2는 +X 경계 바깥 `(4.602, 0.0, 0.42)`에 배치됐다.
- GPU 3에서 stabilization, 카메라 렌더 및 약 33~34 FPS 실행을 확인했다.

## Robot-relative camera 검증 (2026-08-12)

- 카메라를 world-coordinate follow 방식에서 Go2 기준 TPS transform으로 변경했다.
- local camera position은 `(-2.5, 0.0, 1.08)`, local look-at은 `(0.0, 0.0, -0.02)`로,
  카메라가 로봇의 뒤쪽에서 정면 방향을 바라본다.
- `base_link`에 직접 attach하면 보행 중 roll/pitch와 body bounce가 모두 카메라에 전달되므로,
  매 프레임 base position과 yaw만 추출하고 EMA smoothing 0.85를 적용한다. 카메라 up은 world
  Z축에 고정해 수평을 유지한다.
- Go2는 +X spawn 위치에서 DSAIL 쪽을 보도록 초기 yaw를 180도로 설정했다. 따라서 후방
  카메라는 mesh 내부가 아니라 scene 바깥 빈 공간에 위치한다.
- 기존 `cam.render()` 기반 JPEG 스트리밍은 그대로 유지된다.
- `EGL_DEVICE_ID=3`, `CUDA_VISIBLE_DEVICES=3`에서 yaw 명령 전후 프레임을 비교했다.
  Go2는 화면 내 같은 위치와 크기를 유지하고 배경 scene이 회전해, 카메라가 로봇의 위치와
  방향을 함께 따라가는 것을 확인했다.
- 후방 전환 후 정지 상태에서 약 1초 간격 프레임을 비교해 카메라 수평과 구도가 안정적으로
  유지되는 것을 확인했다.
