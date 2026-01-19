#!/usr/bin/env bash
set -euo pipefail

cd docker

docker compose exec ugv_ros bash -lc "
  source /work/ros2_ws/install/setup.bash &&
  ros2 action send_goal /scan_and_offload ugv_rover_pt_scan_mission/action/ScanAndOffload \
  '{duration_sec: 30.0}'
"
