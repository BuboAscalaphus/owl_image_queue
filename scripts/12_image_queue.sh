#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
TOOLS_DIR="$HERE/.."
source "$TOOLS_DIR/config.sh"

if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "❌ Container '$NAME' is not running. Start it first (e.g. ./01_start_container.sh)." >&2
  exit 1
fi

BASE_DIR=${BASE_DIR:-/home/dev/bags}
STATUS_TOPIC=${STATUS_TOPIC:-/enabled}
PROCESS_WHEN=${PROCESS_WHEN:-false}
VELOCITY_TOPIC=${VELOCITY_TOPIC:-/velocity}
MIN_SPEED=${MIN_SPEED:--1.0}
PROCESSOR=${PROCESSOR:-stats}
RESULTS_FORMAT=${RESULTS_FORMAT:-jsonl}

docker exec -it "$NAME" bash -lc "
  source /opt/ros/humble/setup.bash &&
  [ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash || true &&
  ros2 launch owl_image_queue orchard_processing.launch.py \    base_dir:=$BASE_DIR \    status_topic:=$STATUS_TOPIC \    process_when:=$PROCESS_WHEN \    velocity_topic:=$VELOCITY_TOPIC \    min_speed_mps:=$MIN_SPEED \    processor_name:=$PROCESSOR \    results_format:=$RESULTS_FORMAT
"
