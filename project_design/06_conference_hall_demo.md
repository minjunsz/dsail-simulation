# Step 6: Metaverse Agora Conference Hall 데모

## 데모 시나리오

- **Sub Scenario 1. 학회장 poster tour**: User 1, 2가 학회장 안을 돌아다니며 포스터를 본다. 보드마다 Agent가 서 있다(Dummy 7, Author 1).
- **Sub Scenario 2. 3자 대화**: User 1과 User 2가 포스터에 대해 이야기하다가, 막히는 부분을 Author agent에게 묻는다. Author agent는 가능하면 실시간으로 답한다.

## 결정 사항 (2026-10-14)

| 항목 | 결정 |
|---|---|
| 대화 방식 | 음성 (User ↔ User, User → Author) |
| 아바타 | G1 사용 안 함. 사용자와 Agent 모두 원기둥으로 표현하고, 얼굴 높이의 막대로 바라보는 방향 표시 |
| Author 답변 | 우선 mock 응답. 이후 LLM으로 교체 |
| 포스터 | 실제 포스터 `HeSS.png` 1장(Author 보드). 나머지 7개는 번호가 적힌 임시 이미지 |

## 진행 단계

1. **학회장 씬 + 포스터 + Agent 배치** ← 이번 단계 (완료)
2. 시뮬레이터 → 서버 위치 전송(PUB topic) + 근접 판정 + 브라우저 포스터 오버레이
3. User ↔ User 근접 음성 (서버 중계, 사용자별 mixer track, HTTPS 필요)
4. Author 음성 응답 (VAD → STT → mock 답변 → TTS)

## 구현 (1단계)

- `assets/conference_hall/layout.yaml`: 씬의 단일 기준 파일. 홀 크기, 보드 8개(위치·방향·포스터·역할), Agent 오프셋, 아바타 크기·색, 스폰 위치를 담는다. 이후 서버의 근접 판정도 이 파일을 쓴다.
- `scripts/make_conference_hall.py`: `build/`에 다음 파일을 생성한다. git에서 제외된다.
  - `hall.ply`: 바닥, 벽, 보드 받침 (vertex color)
  - `poster_<id>.obj`: 월드 좌표 기준 UV 사각형
  - `poster_<id>.png`: 포스터 텍스처 (최대 폭 2048)
- 시뮬레이터에서 G1과 보행 정책을 제거했다. 아바타는 kinematic으로 움직인다(1.0 m/s, 1.2 rad/s). 중력은 0이고 Genesis 솔버 충돌은 쓰지 않는다.
- 기존 G1 / DSAIL / Nyx 시뮬레이터는 `simulator/legacy_g1_main.py`로 보관했다. Step 5(데모 메시, Nyx) 작업이 커밋되지 않아서 이 파일이 유일한 사본이다.

## 코드 구조 (리팩터링, 2026-10-14)

전체 목표 구조는 `conference_demo_arch.svg`에 있다. 이번 리팩터링은 그중 Genesis 쪽 구조와 `avatar_id` 프로토콜까지다.

```
common/                    # 시뮬레이터·서버·agent backend 공용, Genesis/torch 의존 없음
  hall_layout.py           # layout.yaml 로더, board_axes/agent_pose, avatar_mesh, obstacles/blocked
  protocol.py              # avatar id 체계, 프레임 pack/unpack, move 명령 생성/파싱
  gaze.py                  # in_view: 거리 + 시야각 판정 (하이라이트, 이후 attention)
simulator/
  hall.py                  # Hall: 홀 메시·포스터 entity + 정적 충돌(벽, 보드 받침)
  avatar.py                # Avatar: 몸(pose, 명령, watchdog, 2D 이동·충돌)
                           #   User(Avatar):  TPS 카메라, JPEG 렌더
                           #   Agent(Avatar): 담당 보드, 역할(dummy/author)
  main.py                  # Simulator: 조립, ZMQ, 50 Hz 루프
  legacy_g1_main.py
teleop_server/main.py      # common.protocol 사용, avatar_id로 명령 전달
```

- **avatar id**: User는 `1..USER_COUNT`(2), Agent는 `100 + 보드 id`(101~108)다. 프레임 헤더와 move 명령 모두 이 id를 쓴다.
  - move 명령: `{"avatar_id", "vx", "vy", "wz"}`
  - WS `assigned` 메시지: `avatar_id`
  - 빈 슬롯이 없을 때: `no_slot`
- **Agent도 명령으로 움직인다.** agent backend가 User와 같은 move 명령으로 Agent의 몸을 움직일 수 있다(예: 말하는 사람 쪽으로 돌기).
- **아바타끼리의 충돌**: 모든 아바타가 매 step 서로를 원으로 보고 충돌 판정한다. Agent가 고정이 아니므로 `obstacles()`에는 벽과 보드만 남겼다.
- **이동량은 실제 경과 시간(dt, 최대 0.1 s)으로 적분한다.** 루프가 50 Hz보다 느려도 속도가 m/s 단위로 유지된다. 이전에는 고정 0.02 s를 써서, 13 Hz일 때 실제 속도가 0.26 m/s였다.
- **멈춰 있는 아바타는 entity를 갱신하지 않는다.** 대부분 서 있기만 하는 Agent 8개의 `set_pos` 호출 비용을 줄이기 위함이다.
- `legacy_g1_main.py`는 `avatar_id`와 `robot_id`를 둘 다 받는다. 그래서 새 teleop server로도 동작한다.

### 검증

- **E2E**: 시뮬레이터 + teleop server + aiortc 헤드리스 클라이언트 3개로 확인했다.
  - 슬롯 배정은 user 1, 2이고, 3번째 클라이언트는 `no_slot`을 받았다.
  - 두 클라이언트 모두 WebRTC 영상(1280x720)을 수신했다.
  - WS로 보낸 move 명령으로 두 아바타가 이동·회전한 것을 시뮬레이터 PUB에 직접 SUB로 붙어 받은 화면으로 확인했다.
- **위치 추적**: user 1은 직진했고(0.02 s × 40 step마다 0.8 m), user 2는 회전 후 보드 받침 앞(y=-3.09)에서 멈췄다.

### 성능 메모

리팩터링 후 측정한 수치(sim 24~30 ms, 12~15 FPS)는 PoC_5의 3DGS 학습과 GPU 0을 함께 쓴 상태에서 나온 값이다. 학습이 GPU 사용률 100%였다. 프로파일링한 결과, 시간은 주로 아래 두 곳에서 든다.

| 항목 | 루프당 시간 (2 cams) | 원인 |
|---|---|---|
| `camera.set_pose` | 약 30 ms | Genesis 내부의 작은 torch GPU 연산과 동기화. GPU를 공유하면 특히 느려진다 |
| JPEG 변환 중 `PIL.fromarray`의 `tobytes` | 약 27 ms | |
| GPU 렌더링 | 약 16 ms | |

GPU 0이 비었을 때 다시 측정해야 한다. 최적화할 후보는 다음과 같다.

- 카메라 pose를 바뀐 경우에만 갱신
- JPEG 인코더 교체 (예: simplejpeg/turbojpeg)
- 해상도 낮추기

## 시선 하이라이트와 이름표 (2026-10-14)

### 시선 하이라이트
- Agent가 사용자 기준으로 `gaze.max_distance`(3 m) 이내에 있고, 사용자가 바라보는 방향(방향 막대)에서 ±`half_angle_deg`(30°) 안에 들어오면, **그 사용자의 화면에서만** 초록색(`gaze.highlight`)으로 바뀐다.
  - 가려짐(occlusion)은 확인하지 않는다.
  - 판정 함수는 `common/gaze.py`의 `in_view`다. 이후 서버와 agent backend에서 "누가 누구를 보고 있는지" 판단할 때도 이 함수를 쓴다.
- **사용자마다 다른 색을 보여 주는 방법**
  - Genesis는 씬 step마다 한 번만 시각 상태를 갱신한다. 그래서 Genesis API로 색을 바꾸면 카메라마다 다르게 보여 줄 수 없다.
  - 대신 Agent 몸통을 흰색 버텍스 색으로 만들고, 역할 색은 렌더링 재질의 `baseColorFactor`(버텍스 색에 곱해지는 값)로 준다. 그리고 카메라마다 렌더링 직전에 이 값을 바꾼다.
  - 주의: rasterizer의 JIT 렌더러는 재질 값을 `context.jit.pbr_mat`에 복사해 두고 그 배열로 그린다. 그래서 재질 객체만 바꾸면 반영되지 않는다(실측: 픽셀이 바뀌지 않음). `Agent.set_highlight`는 이 배열의 해당 행과 재질 객체를 함께 바꾼다. 재질도 바꾸는 이유는 JIT가 primitive 목록을 다시 만들 때 값이 유지되도록 하기 위해서다.

### 이름표
- 각 사용자 화면에서 자신을 제외한 아바타의 머리 위(높이 + 0.25 m)에 이름표를 그린다. 그리는 대상은 `avatar.name_tag_max_distance`(12 m) 이내의 아바타다.
- 렌더링한 이미지 위에 2D로 그린다. 머리 위 3D 점을 TPS 카메라로 투영해 위치를 정한다.
  - 3D 텍스트 메시로 만들면 뒤에서 볼 때 글자가 뒤집히므로 이 방식을 택했다.
  - 가려짐은 확인하지 않는다.
  - 먼 이름표부터 그려서 겹치면 가까운 이름표가 위에 온다.
  - 하이라이트된 Agent는 이름표 배경도 초록색이다.
- 이름 설정
  - User: `avatar.user_names`
  - Agent: 보드의 `name`. 없으면 `Author` / `Agent <보드 id>`
  - 글꼴: 한글을 지원하는 NanumSquareRound Bold. 없으면 PIL 기본 글꼴을 쓴다.

### 측정 (GPU 0 단독, PoC_5 학습 종료 후)
- sim 5~6 ms, render+publish (2 cams) 40~46 ms, 약 19~22 FPS
- 하이라이트·이름표 추가 전 측정(25.7 FPS, render 29 ms)보다 느리다. 아직 원인을 분리하지 않았다.

## 경위: G1에서 원기둥으로 바꾼 이유

처음에는 정적 G1 8대로 Agent를 배치했다. 그러나 G1 한 대가 약 50만 폴리곤이라, 10대를 그리면 render+publish가 34 ms에서 55~80 ms로 늘었다(12~14 FPS). 메시를 5%로 줄인 저폴리 G1로도 개선이 거의 없었고(13~15 FPS, sim 26~29 ms), 이후 G1 자체를 쓰지 않기로 결정했다.

## 실행

```bash
uv run python scripts/make_conference_hall.py
CUDA_VISIBLE_DEVICES=0 EGL_DEVICE_ID=0 uv run python -m simulator.main
uv run python -m teleop_server.main
```

## 실행 검증 (2026-10-14, A40 GPU 0, 1280x720 카메라 2대)

| 상황 | sim | render+publish | FPS (목표 50) |
|---|---|---|---|
| 시작 직후 | 9.6 ms | 29 ms | 25.7 |
| 두 사용자 이동 중 | 10~12 ms | 44~52 ms | 15.7~18.3 |

- 두 사용자 모두 이동, 회전, 보드·Agent 앞에서 멈추는 동작을 확인했다(ZMQ로 명령을 보내 받은 프레임으로 확인). 리팩터링 전 측정이다.
- 이동 중에 render 시간이 늘어나는 원인은 아직 분석하지 않았다. 다음 단계로 렌더링과 JPEG 인코딩 시간을 나눠서 측정해야 한다. 사용자당 30 fps 목표(00_overview)에 못 미친다.
