#!/usr/bin/env bash
set -eo pipefail
if [[ -z "${DISPLAY:-}" ]]; then
  exec xvfb-run -a -s '-screen 0 1280x800x24' bash "$0"
fi
source /work/ros2_ws/install/setup.bash
# Reuse a surveyed, visible area; do not fabricate terrain under the initial footprint.
setsid ros2 launch mooncar_navigation terrain_navigation.launch.py \
  use_rviz:=false require_workbench:=true initial_x:=3.5 initial_y:=3.0 initial_yaw:=0.0 \
  gz_args:='-r -s -v 2 --render-engine ogre' > /evidence/workbench-launch-01.log 2>&1 &
launch_pid=$!
cleanup() {
  trap - EXIT
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
wait "$launch_pid"
