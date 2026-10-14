#!/bin/sh
# The Piper voice is a 75 MB binary, so it is downloaded instead of committed.
# One file holds 904 speakers, which is what gives every participant a
# different voice.
set -e
cd "$(dirname "$0")"
BASE=https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/libritts_r/medium
for f in en_US-libritts_r-medium.onnx en_US-libritts_r-medium.onnx.json; do
    if [ -f "$f" ]; then
        echo "have $f"
    else
        echo "downloading $f"
        curl -sSL -O "$BASE/$f"
    fi
done
echo "done"
