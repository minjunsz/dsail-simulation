/*
 * voicechat.js -- the browser half of the voice chat. PART OF THE LIBRARY.
 *
 * It owns the websocket, keeps the participant list, and plays every utterance
 * it receives at a volume that fades with distance. It draws nothing and knows
 * nothing about the simulator.
 *
 * Your page only has to:
 *     chat.connect(name)                 join
 *     chat.say("hello")                  speak
 *     chat.send({...})                   anything your own server needs
 *     onBinary(tag, payload)             frames that are not speech are yours
 *
 * Positions are NOT sent from here -- the simulator owns them and the server
 * feeds them to room.move(). See app.py.
 */

const TAG_SPEECH = 1;      // must match voicechat/protocol.py

class VoiceChat {
  /*
   * options:
   *   url       websocket url, default: /ws on the current host
   *   onWelcome (you)                    -> your own id / name / voice
   *   onPeople  (people[])               -> the participant list changed
   *   onSaid    (id, name, text)         -> someone submitted a line
   *   onBinary  (tag, ArrayBuffer)       -> a binary frame this library does
   *                                         not own (your app's own data)
   */
  constructor(options = {}) {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    this.url = options.url || `${proto}://${location.host}/ws`;
    this.onWelcome = options.onWelcome || (() => {});
    this.onPeople = options.onPeople || (() => {});
    this.onSaid = options.onSaid || (() => {});
    this.onBinary = options.onBinary || (() => {});

    this.you = null;      // {id, name, voice}
    this.people = [];     // [{id, name, voice, x, y, z}, ...]
    this.ws = null;
    this.audio = null;
  }

  connect(name) {
    this.ws = new WebSocket(this.url);
    this.ws.binaryType = 'arraybuffer';
    this.ws.onopen = () => this.send({ t: 'join', name });
    this.ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') this._onJson(JSON.parse(ev.data));
      else this._onFrame(ev.data);
    };
    return this;
  }

  /* Say something. The server synthesizes it and everyone nearby hears it. */
  say(text) {
    text = (text || '').trim();
    if (text) this.send({ t: 'say', text });
  }

  /* Send any JSON message to your server (key presses, actions, ...). */
  send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(obj));
    }
  }

  /* Browsers block audio until the user interacts -- call this from a click. */
  unlockAudio() {
    if (!this.audio) this.audio = new (window.AudioContext || window.webkitAudioContext)();
    if (this.audio.state === 'suspended') this.audio.resume();
  }

  // --- internals -----------------------------------------------------------

  _onJson(msg) {
    if (msg.t === 'welcome') {
      this.you = msg.you;
      this.onWelcome(msg.you);
    } else if (msg.t === 'people') {
      this.people = msg.people;
      this.onPeople(msg.people);
    } else if (msg.t === 'said') {
      this.onSaid(msg.speaker_id, msg.speaker_name, msg.text);
    }
  }

  _onFrame(buffer) {
    const tag = new DataView(buffer).getUint8(0);
    if (tag === TAG_SPEECH) this._onSpeech(buffer);
    else this.onBinary(tag, buffer.slice(1));
  }

  /* Speech frame (voicechat/protocol.py):
   * [1 byte tag][4 byte header length BE][UTF-8 JSON header][mono int16 PCM] */
  _onSpeech(buffer) {
    const view = new DataView(buffer);
    const headerLength = view.getUint32(1);
    const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 5, headerLength)));
    // slice() copies, which also gives the even byte offset Int16Array requires
    // (the header is JSON, so its length is odd about half the time).
    const pcm = new Int16Array(buffer.slice(5 + headerLength));
    this._play(pcm, header.sample_rate, header.gain);
  }

  _play(pcm, sampleRate, gain) {
    this.unlockAudio();
    const buffer = this.audio.createBuffer(1, pcm.length, sampleRate);
    const channel = buffer.getChannelData(0);
    for (let i = 0; i < pcm.length; i++) channel[i] = pcm[i] / 32768;

    const source = this.audio.createBufferSource();
    source.buffer = buffer;
    const volume = this.audio.createGain();
    volume.gain.value = gain;
    source.connect(volume).connect(this.audio.destination);
    source.start();
  }
}
