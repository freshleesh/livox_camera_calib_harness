#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-livox_calib:noetic}"

xhost +local:docker >/dev/null

docker run --rm -it \
  --name livox_calib \
  --net=host \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$WS_DIR":/catkin_ws \
  -w /catkin_ws \
  "$IMAGE" \
  "$@"
