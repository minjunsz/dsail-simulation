'use strict';

const video = document.getElementById('video');
const statusEl = document.getElementById('status');
let activeWs = null;

function setStatus(msg) {
  statusEl.textContent = msg;
  console.log('[client]', msg);
}

function connect() {
  const wsUrl = `ws://${location.host}/ws`;
  setStatus(`Connecting to ${wsUrl}…`);

  const ws = new WebSocket(wsUrl);
  activeWs = ws;
  let pc = null;

  ws.onopen = () => setStatus('WebSocket connected, waiting for offer…');

  ws.onmessage = async (event) => {
    const data = JSON.parse(event.data);

    if (data.type === 'offer') {
      setStatus('Offer received, creating answer…');

      pc = new RTCPeerConnection();

      pc.ontrack = (event) => {
        if (event.streams && event.streams[0]) {
          video.srcObject = event.streams[0];
          setStatus('Streaming');
        }
      };

      pc.oniceconnectionstatechange = () => {
        setStatus(`ICE: ${pc.iceConnectionState}`);
        if (pc.iceConnectionState === 'connected') {
          setStatus('Streaming');
        }
      };

      await pc.setRemoteDescription({ type: 'offer', sdp: data.sdp });
      const answer = await pc.createAnswer();
      await pc.setLocalDescription(answer);

      ws.send(JSON.stringify({ type: 'answer', sdp: answer.sdp }));
      setStatus('Answer sent, establishing connection…');
    }
  };

  ws.onclose = () => {
    setStatus('Disconnected. Reconnecting in 2s…');
    if (pc) { pc.close(); pc = null; }
    activeWs = null;
    setTimeout(connect, 2000);
  };

  ws.onerror = (err) => {
    console.error('[client] WebSocket error', err);
  };
}

connect();

// WASD teleop
const keys = new Set();
document.addEventListener('keydown', e => keys.add(e.key));
document.addEventListener('keyup',   e => keys.delete(e.key));

setInterval(() => {
  let vx = 0, vy = 0, wz = 0;
  if (keys.has('w')) vx += 1;
  if (keys.has('s')) vx -= 1;
  if (keys.has('a')) vy += 1;
  if (keys.has('d')) vy -= 1;
  if (keys.has('q')) wz += 1;
  if (keys.has('e')) wz -= 1;
  if (activeWs && activeWs.readyState === WebSocket.OPEN) {
    activeWs.send(JSON.stringify({ type: 'move', vx, vy, wz }));
  }
}, 33);
