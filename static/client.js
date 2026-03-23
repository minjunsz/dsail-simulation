'use strict';

const video = document.getElementById('video');
const statusEl = document.getElementById('status');

function setStatus(msg) {
  statusEl.textContent = msg;
  console.log('[client]', msg);
}

function connect() {
  const wsUrl = `ws://${location.host}/ws`;
  setStatus(`Connecting to ${wsUrl}…`);

  const ws = new WebSocket(wsUrl);
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
    setTimeout(connect, 2000);
  };

  ws.onerror = (err) => {
    console.error('[client] WebSocket error', err);
  };
}

connect();
