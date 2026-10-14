"""Who is in the shared space, where they stand, and what they say.

There is no 3D engine here on purpose. A participant is just an id, a name,
a voice and an (x, y, z). Your simulator owns the real avatars -- it only has
to call room.move() when one of them moves.
"""

import itertools
import math
from dataclasses import asdict, dataclass

# Consecutive joiners get voice ids far apart so they sound clearly different.
VOICE_STRIDE = 37

# Distance model, in whatever unit your world uses.
NEAR = 2.0    # closer than this -> full volume
FAR = 12.0    # further than this -> cannot be heard at all

MAX_TEXT = 300


@dataclass
class Participant:
    id: int
    name: str
    voice: int
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Speech:
    """One utterance: who said it, what it was, and the audio for it."""
    speaker_id: int
    speaker_name: str
    text: str
    pcm: bytes        # mono int16
    sample_rate: int
    x: float          # where the speaker stood when they said it
    y: float
    z: float


def hearing_gain(listener, speaker, near=NEAR, far=FAR):
    """How loud `speaker` is for `listener`.

    in : two Participants
    out: float 0.0 (inaudible) .. 1.0 (full volume)
    """
    distance = math.dist(
        (listener.x, listener.y, listener.z),
        (speaker.x, speaker.y, speaker.z),
    )
    if distance <= near:
        return 1.0
    if distance >= far:
        return 0.0
    return (far - distance) / (far - near)


class Room:
    """A shared space that many participants can join and talk in."""

    def __init__(self, tts):
        self.tts = tts
        self._people = {}
        self._ids = itertools.count()

    # --- membership ------------------------------------------------------
    def join(self, name):
        """in : display name -> out: the new Participant (with its own voice)"""
        pid = next(self._ids)
        person = Participant(
            id=pid,
            name=(name or f"user{pid}").strip()[:24] or f"user{pid}",
            voice=(pid * VOICE_STRIDE) % self.tts.voice_count,
        )
        self._people[pid] = person
        return person

    def leave(self, participant_id):
        self._people.pop(participant_id, None)

    def get(self, participant_id):
        return self._people.get(participant_id)

    def participants(self):
        return list(self._people.values())

    def snapshot(self):
        """Everyone's current state, as plain dicts -- safe to send to clients."""
        return [asdict(p) for p in self._people.values()]

    # --- talking ---------------------------------------------------------
    def move(self, participant_id, x, y, z=0.0):
        person = self._people.get(participant_id)
        if person:
            person.x, person.y, person.z = float(x), float(y), float(z)

    def say(self, participant_id, text):
        """in : who is talking + what they type
        out: a Speech (audio + speaker + position), or None if nothing to say

        This call runs the TTS model, so it takes ~0.5 s per sentence.
        Run it off the event loop (see server.py).
        """
        person = self._people.get(participant_id)
        if person is None:
            return None
        text = (text or "").strip()[:MAX_TEXT]
        if not text:
            return None
        return Speech(
            speaker_id=person.id,
            speaker_name=person.name,
            text=text,
            pcm=self.tts.synth(text, person.voice),
            sample_rate=self.tts.sample_rate,
            x=person.x, y=person.y, z=person.z,
        )

    def audience(self, speech):
        """in : a Speech -> out: [(Participant, gain), ...] for everyone who can hear it.

        The speaker is included, so you hear your own voice too.
        """
        source = Participant(
            id=speech.speaker_id, name=speech.speaker_name, voice=0,
            x=speech.x, y=speech.y, z=speech.z,
        )
        out = []
        for person in self._people.values():
            gain = hearing_gain(person, source)
            if gain > 0.0:
                out.append((person, gain))
        return out
