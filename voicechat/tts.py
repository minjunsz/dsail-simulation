"""Text -> speech audio.

This is the ONLY file that knows about a TTS model. To swap Piper for another
engine, rewrite this file and keep the same three members:

    engine.sample_rate          int    Hz of the audio it returns
    engine.voice_count          int    how many distinct voices it can produce
    engine.synth(text, voice)   bytes  mono 16-bit PCM

Nothing else in the project imports piper.
"""

import json
import os
import threading

import onnxruntime as ort
from piper import PiperVoice, SynthesisConfig
from piper.config import PiperConfig


# Measured on this model: 1 thread 0.33 s/sentence, 4 -> 0.16, 8 -> 0.12,
# 16 -> 0.21, 48 -> 1.42. It is a small model, so piling on cores makes it
# dramatically slower, not faster.
BEST_THREADS = 8


def _default_threads():
    """CPUs this process may use -- under SLURM that is --cpus-per-task -- capped."""
    try:
        available = len(os.sched_getaffinity(0))
    except AttributeError:
        available = os.cpu_count() or 1
    return max(1, min(available, BEST_THREADS))


class TTSEngine:
    """Turns text into mono 16-bit PCM audio, in a chosen voice.

    in : text (str), voice (int, 0 .. voice_count-1)
    out: bytes -- mono int16 PCM at self.sample_rate
    """

    def __init__(self, model_path, threads=None, config_path=None):
        model_path = str(model_path)
        with open(config_path or model_path + ".json", encoding="utf-8") as f:
            config = PiperConfig.from_dict(json.load(f))

        # Build the session ourselves rather than calling PiperVoice.load():
        # left alone, onnxruntime sizes its pool to every core on the machine,
        # which is both slower (see BEST_THREADS) and, inside a SLURM
        # allocation, noisy with failed thread-affinity warnings.
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads or _default_threads()
        options.inter_op_num_threads = 1
        session = ort.InferenceSession(
            model_path, sess_options=options, providers=["CPUExecutionProvider"]
        )
        self._voice = PiperVoice(session=session, config=config)
        self.sample_rate = self._voice.config.sample_rate
        self.voice_count = max(1, self._voice.config.num_speakers)
        # One onnxruntime session, so only one synthesis at a time.
        self._lock = threading.Lock()

        # The first call costs ~1.5 s (onnx warm-up, espeak init). Pay it here
        # instead of making the first person who talks wait for it.
        self.synth("warm up", 0)

    def synth(self, text, voice=0):
        text = (text or "").strip()
        if not text:
            return b""
        config = SynthesisConfig(
            speaker_id=voice % self.voice_count if self.voice_count > 1 else None
        )
        with self._lock:
            chunks = self._voice.synthesize(text, syn_config=config)
            return b"".join(c.audio_int16_bytes for c in chunks)
