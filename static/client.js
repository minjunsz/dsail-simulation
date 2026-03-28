'use strict';

const canvas = document.getElementById('canvas');
const ctx2d = canvas.getContext('2d');
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
  ws.binaryType = 'arraybuffer';
  activeWs = ws;

  let decoder = null;
  let configured = false;

  function resetDecoder() {
    if (decoder && decoder.state !== 'closed') decoder.close();
    decoder = new VideoDecoder({
      output: (frame) => {
        ctx2d.drawImage(frame, 0, 0, canvas.width, canvas.height);
        frame.close();
      },
      error: (e) => {
        console.error('[decoder]', e);
        configured = false;
      },
    });
  }

  resetDecoder();

  ws.onopen = () => setStatus('Connected — waiting for video…');

  ws.onmessage = (event) => {
    if (!(event.data instanceof ArrayBuffer)) return;

    const view = new DataView(event.data);
    const isKey = view.getUint8(0) === 1;
    const pts   = view.getUint32(1, false);           // big-endian
    const videoData = event.data.slice(5);
    const timestamp = Math.round(pts * 1_000_000 / 30); // microseconds

    if (!configured) {
      if (!isKey) return;   // wait for the first keyframe
      decoder.configure({
        codec: 'vp8',
        codedWidth: 1280,
        codedHeight: 720,
        optimizeForLatency: true,
      });
      configured = true;
      setStatus('Streaming');
    }

    try {
      decoder.decode(new EncodedVideoChunk({
        type: isKey ? 'key' : 'delta',
        timestamp,
        data: videoData,
      }));
    } catch (e) {
      console.error('[decode]', e);
      configured = false;   // will re-configure on next keyframe
    }
  };

  ws.onclose = () => {
    setStatus('Disconnected. Reconnecting in 2s…');
    activeWs = null;
    configured = false;
    setTimeout(connect, 2000);
  };

  ws.onerror = (err) => console.error('[client] WebSocket error', err);
}

connect();

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
