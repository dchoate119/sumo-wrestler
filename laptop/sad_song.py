"""Sad trombone ("wah wah wah wahhh") for when the robot loses.

The notes are synthesized with numpy and written to a WAV once; play() hands it
to aplay in the background, so the tracker keeps running.

    python laptop/sad_song.py   # hear it
"""

import subprocess
import tempfile
import wave
from pathlib import Path

import numpy as np

RATE = 22050
# (Hz, seconds): Bb3, A3, Ab3, then a long G3 with a wobble
NOTES = [(233.1, 0.45), (220.0, 0.45), (207.7, 0.45), (196.0, 1.4)]
WOBBLE_HZ = 6       # vibrato on the last note
WOBBLE_DEPTH = 0.02  # +- this fraction of the pitch
GAP_S = 0.08        # silence after each note

_wav = None


def _note(freq, secs, wobble):
    t = np.arange(int(RATE * secs)) / RATE
    f = np.full_like(t, freq)
    if wobble:
        f *= 1 + WOBBLE_DEPTH * np.sin(2 * np.pi * WOBBLE_HZ * t)
    phase = 2 * np.pi * np.cumsum(f) / RATE
    tone = np.sign(np.sin(phase)) * 0.3 + np.sin(phase) * 0.7  # a bit brassy
    env = np.minimum(1, np.minimum(t / 0.03, (secs - t) / 0.08))  # soft attack and release
    return np.concatenate([tone * env, np.zeros(int(RATE * GAP_S))])


def _make_wav():
    audio = np.concatenate([_note(f, s, i == len(NOTES) - 1) for i, (f, s) in enumerate(NOTES)])
    pcm = (audio * 0.5 * 32767).astype(np.int16)
    path = Path(tempfile.gettempdir()) / "sumo_sad_song.wav"
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(pcm.tobytes())
    return path


def play():
    """Start the song in the background. Prints a note instead if aplay is missing."""
    global _wav
    _wav = _wav or _make_wav()
    try:
        subprocess.Popen(["aplay", "-q", str(_wav)])
    except FileNotFoundError:
        print("sad song: aplay not found")


if __name__ == "__main__":
    play()
    import time
    time.sleep(sum(s for _, s in NOTES) + 0.5)
