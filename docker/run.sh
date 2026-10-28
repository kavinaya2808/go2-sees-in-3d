#!/usr/bin/env bash
set -euo pipefail
WS="${WS:-$HOME/robotics/go2_sees_3d/ws}"

xhost +local:docker > /dev/null

docker run -it --name go2_sees_3d \
  --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  --net=host --ipc=host \
  -e ROS_DOMAIN_ID=43 -e GZ_PARTITION=go2_sees_3d \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$WS":/root/go2_ws \
  go2_sees_3d:jazzy bash
