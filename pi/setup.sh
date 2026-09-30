#!/usr/bin/env bash
# Build libmedia_codec and install the RoboMaster SDK on Raspberry Pi OS 64-bit.
#
#   bash pi/setup.sh [venv_dir]
#
# The repo root is derived from this script's location, so it works straight out
# of a fresh clone.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$SCRIPT_DIR")"
VENV="${1:-$HOME/rm-venv}"

echo "== repo: $REPO"
echo "== venv: $VENV"
test -f "$REPO/setup.py" || { echo "no setup.py in $REPO"; exit 1; }

sudo apt update
# No libopus-dev: Opus is decoded through libavcodec.
sudo apt install -y build-essential cmake pkg-config python3-dev python3-venv \
    libavcodec-dev libavutil-dev libswscale-dev libswresample-dev

# Raspberry Pi OS marks the system Python externally managed (PEP 668), so pip
# has to install into a virtualenv.
[ -d "$VENV" ] || python3 -m venv "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python -V
pip install --upgrade pip wheel

echo "== ffmpeg dev libraries visible to pkg-config:"
pkg-config --modversion libavcodec libavutil libswscale libswresample

# A CMake cache committed from another machine would break the configure step.
rm -rf "$REPO/lib/libmedia_codec/build" "$REPO/lib/libmedia_codec/output"

pip install "$REPO/lib/libmedia_codec"
pip install "$REPO"

python - <<'PY'
import libmedia_codec
libmedia_codec.H264Decoder()
libmedia_codec.OpusDecoder()
print("libmedia_codec OK:", libmedia_codec.__version__)
import cv2, numpy, robomaster
from robomaster import robot  # noqa: F401  pulls in camera -> media -> libmedia_codec
print("robomaster OK; numpy", numpy.__version__, "cv2", cv2.__version__)
PY

echo
echo "Done. Activate with: source $VENV/bin/activate"
echo "Then: python $REPO/pi/smoke_test.py --conn rndis"
