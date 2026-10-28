#!/usr/bin/env bash
# Spawn a person (Gazebo Fuel model) into the running world.
# Usage: tools/spawn_person.sh [x] [y] [name]      default: 3.0 0.0 person1
set -euo pipefail
X="${1:-3.0}"
Y="${2:-0.0}"
NAME="${3:-person1}"
MODEL="https://fuel.gazebosim.org/1.0/OpenRobotics/models/Casual female"

gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 60000 \
  --req "sdf_filename: \"${MODEL}\", name: \"${NAME}\", pose: {position: {x: ${X}, y: ${Y}, z: 0}}"
