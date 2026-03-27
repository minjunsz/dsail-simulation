'use strict';

const statusEl = document.getElementById('status');
let activeWs = null;

function setStatus(msg) {
  statusEl.textContent = msg;
  console.log('[client]', msg);
}

function connectWs() {
  const wsUrl = `ws://${location.host}/ws`;
  const ws = new WebSocket(wsUrl);
  activeWs = ws;

  ws.onopen = () => setStatus('Connected — use WASD to move');
  ws.onclose = () => {
    setStatus('Disconnected. Reconnecting in 2s…');
    activeWs = null;
    setTimeout(connectWs, 2000);
  };
  ws.onerror = (err) => console.error('[client] WebSocket error', err);
}

connectWs();

// WASD teleop
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
    if (activeWs && activeWs.readyState === WebSocket.OPEN) {
      activeWs.send(JSON.stringify({ type: 'move', dx, dy }));
    }
  }
}, 33);
