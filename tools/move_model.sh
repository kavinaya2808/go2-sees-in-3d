#!/usr/bin/env bash
# Teleport a model in the running world, e.g. to set up a new evaluation viewpoint.
# Usage: tools/move_model.sh <name> <x> <y> [yaw_deg]
set -euo pipefail
NAME="$1"; X="$2"; Y="$3"; YAW_DEG="${4:-0}"
read -r QZ QW < <(python3 -c "import math; h=math.radians(${YAW_DEG})/2; print(math.sin(h), math.cos(h))")

gz service -s /world/default/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: \"${NAME}\", position: {x: ${X}, y: ${Y}, z: 0}, orientation: {z: ${QZ}, w: ${QW}}"
