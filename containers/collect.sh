#!/usr/bin/env bash
set -eo pipefail
if [[ -z "${DISPLAY:-}" ]]; then
  exec xvfb-run -a -s '-screen 0 1280x800x24' bash "$0"
fi
source /opt/ros/jazzy/setup.bash
source /work/ros2_ws/install/setup.bash
session_dir="$(mktemp -d /evidence/gazebo-map-XXXXXX)"
pids=()
stop_children() {
  for pid in "${pids[@]}"; do kill -INT -- "-${pid}" 2>/dev/null || true; done
  for _ in {1..20}; do
    local running=false
    for pid in "${pids[@]}"; do kill -0 -- "-${pid}" 2>/dev/null && running=true; done
    [[ "$running" == false ]] && break
    sleep .5
  done
  for pid in "${pids[@]}"; do kill -TERM -- "-${pid}" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do
    kill -KILL -- "-${pid}" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  pids=()
}
cleanup() {
  local code=$?
  trap - EXIT
  stop_children
  exit "$code"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
setsid ros2 launch mooncar_slam mapping.launch.py use_rviz:=false \
  gz_args:='-r -s -v 2 --render-engine ogre' >"$session_dir/launch.log" 2>&1 &
pids+=("$!")
setsid ros2 run mooncar_terrain terrain_node.py --ros-args -p use_sim_time:=true -p depth_pixel_offset:=0.5 \
  >"$session_dir/terrain.log" 2>&1 &
pids+=("$!")
setsid ros2 launch mooncar_terrain record_map.launch.py output_directory:="$session_dir" \
  >"$session_dir/recorder.log" 2>&1 &
pids+=("$!")
timeout -k 2s 120s ros2 topic echo --once /mooncar/map/points --field width >"$session_dir/first-cloud.txt"
ros2 run mooncar_control drive_smoke_test.py >"$session_dir/drive.log" 2>&1
timeout -k 2s 30s ros2 service call /mooncar/map/save std_srvs/srv/Trigger '{}' >"$session_dir/save.log"
stop_children
PYTHONPATH=/work/ros2_ws/src/mooncar_terrain/scripts python3 /source/containers/verify_recording.py "$session_dir"
echo "Observed Gazebo archive saved in $session_dir"
