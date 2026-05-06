#!/usr/bin/env bash
# One-time setup after fresh clone of calib_ws on a new machine.
# Pulls livox_camera_calib source and creates data subdirs.
# Idempotent — safe to re-run.
set -euo pipefail

WS_DIR="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$WS_DIR/data/pcd" "$WS_DIR/data/images" "$WS_DIR/data/results"

CALIB_DIR="$WS_DIR/src/livox_camera_calib"
if [ ! -d "$CALIB_DIR/.git" ]; then
  echo "[setup] cloning livox_camera_calib into src/"
  git clone https://github.com/hku-mars/livox_camera_calib.git "$CALIB_DIR"
else
  echo "[setup] livox_camera_calib already present — skipping clone"
fi

echo "[setup] done. next:"
echo "  bash docker/build.sh        # build image (5-15 min, once)"
echo "  bash docker/run.sh          # enter container"
echo "  # inside container:"
echo "  catkin_make && source devel/setup.bash"
