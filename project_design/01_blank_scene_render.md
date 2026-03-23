# Step 1: Blank Scene Headless Rendering & WebRTC Streaming

## 목표

Genesis에서 ground plane만 있는 blank scene을 headless 렌더링하고, teleop 서버가 이를 받아 WebRTC로 브라우저에 스트리밍하는 **전체 파이프라인 검증**.

## 컴포넌트 구성

```
genesis_sim/
├── simulator/
│   ├── main.py              # Genesis scene 초기화 및 렌더 루프
│   └── frame_publisher.py   # ZeroMQ PUB으로 프레임 전송
├── teleop_server/
│   ├── main.py              # 서버 진입점
│   ├── frame_receiver.py    # ZeroMQ SUB으로 프레임 수신
│   ├── webrtc_handler.py    # WebRTC 연결 관리 및 비디오 트랙
│   └── signaling.py         # WebSocket signaling 처리
└── static/
    ├── index.html
    └── client.js
```

## 데이터 흐름

```
Genesis render loop
    → numpy array (H×W×3, uint8)
    → JPEG 압축
    → ZeroMQ PUB (tcp://localhost:5555)
    → ZeroMQ SUB (teleop server)
    → aiortc VideoStreamTrack
    → WebRTC
    → 브라우저 <video>
```

## 세부 구현 계획

### 1. Genesis 시뮬레이터 (`simulator/`)

#### 1-1. Scene 초기화

- `gs.init(backend=gs.gpu)`로 GPU 렌더링 초기화
- `gs.Scene(show_viewer=False)`로 headless 모드
- `scene.add_entity(gs.morphs.Plane())`으로 ground plane 추가
- 카메라 고정 위치 추가: `scene.add_camera(res=(1280, 720), pos=(...), lookat=(0,0,0))`

#### 1-2. 렌더 루프

- `scene.build()` 후 매 스텝마다 `cam.render()` 호출
- 렌더 결과를 numpy array로 획득 (RGB, uint8, shape `(H, W, 3)`)
- 목표 30fps → 루프에서 ~33ms 간격 유지

#### 1-3. 프레임 전송 (ZeroMQ PUB)

- `zmq.PUB` 소켓, `tcp://*:5555`에 바인드
- 프레임을 JPEG 압축 (`Pillow` 또는 `cv2.imencode`) 후 bytes 직렬화
- 메시지 구조: `[timestamp_ms(8B little-endian) | jpeg_bytes]`

### 2. Teleop 서버 (`teleop_server/`)

#### 2-1. 프레임 수신 (ZeroMQ SUB)

- `zmq.SUB` 소켓으로 `tcp://localhost:5555` 구독
- 별도 asyncio task에서 비동기 수신 (`asyncio.get_event_loop().run_in_executor`)
- 수신된 JPEG → numpy array 디코딩 후 최신 프레임으로 캐싱 (이전 프레임 덮어쓰기)

#### 2-2. WebRTC Video Track (aiortc)

- `aiortc.VideoStreamTrack` 상속 → `recv()` 메서드 오버라이드
- 캐시된 최신 프레임을 `av.VideoFrame`으로 변환 후 반환
- 프레임 미수신 시 검정 프레임 반환 (연결 유지)

#### 2-3. WebSocket Signaling

- `aiohttp` WebSocket 엔드포인트: `GET /ws`
- 클라이언트 접속 시 `RTCPeerConnection` 생성, VideoTrack 추가 (`addTrack`)
- SDP offer/answer 및 ICE candidate를 JSON 메시지로 교환

#### 2-4. HTTP 서버

- `GET /` → `static/index.html` 서빙
- `GET /static/{filename}` → 정적 파일 서빙

### 3. 브라우저 클라이언트 (`static/`)

#### 3-1. Signaling (WebSocket)

- `ws://<host>/ws` 연결
- JSON 메시지 포맷:
  - `{ "type": "offer", "sdp": "..." }`
  - `{ "type": "answer", "sdp": "..." }`
  - `{ "type": "ice", "candidate": {...} }`

#### 3-2. WebRTC (JavaScript RTCPeerConnection)

- `RTCPeerConnection` 생성 (localhost 테스트 시 STUN 불필요)
- `ontrack` 이벤트로 video stream 수신
- `<video autoplay>` 엘리먼트에 stream 연결하여 재생

## 기술 스택

| 컴포넌트 | 라이브러리 |
|---|---|
| 시뮬레이터 | `genesis-world` |
| 프레임 전송 | `pyzmq` |
| 이미지 압축 | `Pillow` (JPEG encode) |
| 웹서버 | `aiohttp` |
| WebRTC | `aiortc` |
| 비디오 프레임 | `av` (PyAV) |

## 검증 기준

- [ ] Genesis가 headless 모드로 실행되고 ground plane 렌더링 확인
- [ ] ZeroMQ 채널로 30fps 수준의 프레임 전송 확인 (타임스탬프 기반 측정)
- [ ] 브라우저에서 WebRTC 스트림 재생 성공
- [ ] 단일 클라이언트 기준 30fps 달성
- [ ] End-to-end latency < 200ms (목표)

## 주의사항

- Genesis의 `cam.render()`는 GPU 동기화가 필요하므로, simulator는 독립 프로세스로 실행
- `aiortc`는 asyncio 기반 → teleop 서버 전체를 async로 구성
- localhost 테스트 시 ICE는 host candidate만으로 동작 가능 (STUN 서버 불필요)
- ZeroMQ PUB/SUB는 slow subscriber 문제 방지를 위해 `HWM(High Water Mark)`를 1~2로 설정하여 항상 최신 프레임만 유지
