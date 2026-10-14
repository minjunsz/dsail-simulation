/*
 * client.js -- the demo page. Sends key presses, shows the video stream the
 * server renders, and lets you type a line.
 *
 * Everything about hearing other people lives in voicechat.js, not here.
 */

const view = document.getElementById('view');
const keys = { up: false, down: false, left: false, right: false };
const KEYMAP = { ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right' };
let lastUrl = null;

const chat = new VoiceChat({
  onWelcome: (you) => {
    document.getElementById('boot').remove();
    document.getElementById('who').textContent = `${you.name} · voice #${you.voice}`;
    log(`you joined as <b>${you.name}</b>`);
  },
  onPeople: (people) => {
    document.getElementById('near').textContent =
      people.length > 1 ? `${people.length} penguins here` : 'alone here';
  },
  onSaid: (id, name, text) => log(`<b>${name}</b>: ${text}`),
  onBinary: (tag, payload) => {              // tag 0 = server-rendered picture
    if (tag !== 0) return;
    const url = URL.createObjectURL(new Blob([payload], { type: 'image/jpeg' }));
    view.src = url;
    if (lastUrl) URL.revokeObjectURL(lastUrl);
    lastUrl = url;
  },
});

chat.connect(prompt('Your name?') || `penguin${Math.floor(Math.random() * 100)}`);

// --- input ------------------------------------------------------------------
function sendKeys() { chat.send({ t: 'keys', keys }); }

addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' || !KEYMAP[e.key]) return;
  e.preventDefault();
  if (!keys[KEYMAP[e.key]]) { keys[KEYMAP[e.key]] = true; sendKeys(); }
});
addEventListener('keyup', (e) => {
  if (!KEYMAP[e.key]) return;
  if (keys[KEYMAP[e.key]]) { keys[KEYMAP[e.key]] = false; sendKeys(); }
});
addEventListener('click', () => chat.unlockAudio());

const input = document.getElementById('text');
const speak = () => { chat.unlockAudio(); chat.say(input.value); input.value = ''; };
document.getElementById('send').onclick = speak;
input.onkeydown = (e) => { if (e.key === 'Enter') speak(); };

function log(html) {
  const el = document.getElementById('log');
  el.insertAdjacentHTML('beforeend', `<div>${html}</div>`);
  el.scrollTop = el.scrollHeight;
}
