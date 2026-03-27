# Step 2: Box Object WASD 조종

## 목표

Genesis scene에 box object를 추가하고, 브라우저에서 WASD 키로 이동 명령을 보내면 simulator가 object를 실시간으로 이동시키는 **양방향 통신 파이프라인** 구축.

---

## 추가되는 데이터 흐름

Step 1은 단방향 (simulator → browser):
```
simulator → ZeroMQ PUB → teleop_server → WebRTC → browser
```

Step 2는 **역방향 제어 채널** 추가:
```
browser → WebSocket → teleop_server → ZeroMQ PUSH → simulator
```

---

## 소켓 포트 정리

| 소켓 | 방향 | 포트 | 용도 |
|---|---|---|---|
| ZeroMQ PUB/SUB | simulator → teleop | 5555 | 프레임 전송 (기존) |
| ZeroMQ PUSH/PULL | teleop → simulator | 5556 | 이동 명령 (신규) |
| WebSocket `/ws` | browser ↔ teleop | 8090 | signaling + 명령 (기존 확장) |

---

## 컴포넌트별 변경 사항

### 1. `simulator/main.py`

**Box entity 추가**
- `scene.build()` 전에 `scene.add_entity(gs.morphs.Box(...))` 추가
- 초기 위치: `(0, 0, 0.5)` (바닥 위)
- Dynamic body가 아닌 **kinematic** 방식으로 위치를 직접 제어 (`box.set_pos`)

**ZeroMQ PULL 소켓 추가** (teleop_server로부터 명령 수신)
- `zmq.PULL` 소켓, `tcp://*:5556` 바인드
- `RCVHWM=1`, non-blocking recv로 매 스텝마다 polling

**Box 이동 로직**
- 수신된 명령 `{"dx": float, "dy": float}` 파싱
- `box.set_pos(current_pos + delta)` 로 위치 업데이트
- 속도: `MOVE_SPEED = 0.05` m/step (튜닝 가능)

**렌더 루프 수정 (명령 처리 → step → render 순서 유지)**
```python
# 1. 명령 polling
try:
    msg = cmd_sock.recv_json(zmq.NOBLOCK)
    pos = box.get_pos()
    pos[0] += msg["dx"] * MOVE_SPEED
    pos[1] += msg["dy"] * MOVE_SPEED
    box.set_pos(pos)
except zmq.Again:
    pass  # 명령 없으면 현 위치 유지

# 2. physics step
scene.step()

# 3. render & publish (기존과 동일)
rgb, *_ = cam.render()
...
```

---

### 2. `teleop_server/main.py`

**ZeroMQ PUSH 소켓 추가** (simulator로 명령 전송)
- `zmq.PUSH` 소켓, `tcp://localhost:5556` connect
- `SNDHWM=1` 으로 최신 명령만 유지
- `app["cmd_sock"]` 으로 앱 상태에 저장

**`on_startup` 수정**
```python
cmd_ctx = zmq.Context()
cmd_sock = cmd_ctx.socket(zmq.PUSH)
cmd_sock.setsockopt(zmq.SNDHWM, 1)
cmd_sock.connect(CMD_ADDR)
app["cmd_sock"] = cmd_sock
```

**WebSocket 핸들러 수정**
- 기존 `offer`/`answer` 처리 외에 `move` 타입 메시지 추가 처리
```python
if data["type"] == "move":
    cmd_sock = request.app["cmd_sock"]
    try:
        cmd_sock.send_json(
            {"dx": data["dx"], "dy": data["dy"]},
            zmq.NOBLOCK,
        )
    except zmq.Again:
        pass
```

---

### 3. `static/client.js`

**WASD 키 상태 추적**
- `keydown` / `keyup` 이벤트로 현재 눌린 키 상태를 `Set`으로 관리
- `setInterval` (~33ms) 마다 상태를 delta로 변환하여 WebSocket 전송
- 아무 키도 안 눌렸을 때는 메시지 전송하지 않음

```javascript
const keys = new Set();
document.addEventListener('keydown', e => keys.add(e.key));
document.addEventListener('keyup',   e => keys.delete(e.key));

setInterval(() => {
    let dx = 0, dy = 0;
    if (keys.has('w')) dx += 1;
    if (keys.has('s')) dx -= 1;
    if (keys.has('a')) dy += 1;
    if (keys.has('d')) dy -= 1;
    if (dx !== 0 || dy !== 0) {
        ws.send(JSON.stringify({ type: 'move', dx, dy }));
    }
}, 33);
```

---

## 파일 변경 목록

| 파일 | 변경 유형 | 주요 내용 |
|---|---|---|
| `simulator/main.py` | 수정 | Box entity 추가, PULL 소켓, 명령 polling, box 위치 업데이트 |
| `teleop_server/main.py` | 수정 | PUSH 소켓 추가, `move` 메시지 처리 |
| `static/client.js` | 수정 | WASD 키 감지 및 WebSocket 전송 |

---

## 검증 기준

- [ ] Genesis scene에 box가 렌더링되어 브라우저에 보임
- [ ] WASD 키 입력 시 box가 해당 방향으로 이동함
- [ ] 키를 떼면 box가 정지 (drift 없음)
- [ ] 기존 30fps 스트리밍 성능 유지
- [ ] 명령 지연 체감 < 100ms

---

## 주의사항

- `scene.step()` 후에만 물리 상태가 업데이트되므로, **명령 처리 → `scene.step()` → `cam.render()` 순서** 반드시 유지
- Box는 kinematic 방식으로 위치 직접 제어; dynamic body로 하면 물리 시뮬레이션과 충돌하여 의도치 않은 움직임 발생
- PUSH/PULL은 1:1이므로 Step 3 (다중 클라이언트) 대비 시 명령에 `client_id` 포함하는 구조로 확장 예정
