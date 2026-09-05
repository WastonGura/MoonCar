#!/usr/bin/env bash
set -eo pipefail
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [[ -z "${DISPLAY:-}" ]]; then
  command -v xvfb-run >/dev/null || { echo 'Install xvfb and xauth for SSH/server-only acceptance.' >&2; exit 1; }
  exec xvfb-run -a -s '-screen 0 1280x800x24' "$0" "$@"
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${repo_root}/ros2_ws"
launch_log="/tmp/mooncar_m5_acceptance.log"
launch_pid=""
use_rviz=true
use_gz_gui=true
launch_file=navigation.launch.py
test_file=navigation_smoke_test.py
milestone=M5

cleanup() {
  local status=$?
  trap - EXIT

  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 -- "-${launch_pid}" 2>/dev/null || break
      sleep 0.5
    done
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  fi
  [[ -z "${launch_pid}" ]] || wait "${launch_pid}" 2>/dev/null || true

  exit "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

while [[ "${1:-}" == --* ]]; do
  case "$1" in
    --server-only) use_rviz=false; use_gz_gui=false ;;
    --rviz-only) use_rviz=true; use_gz_gui=false ;;
    --terrain)
      launch_file=terrain_navigation.launch.py
      test_file=mission_smoke_test.py
      milestone=M6
      launch_log=/tmp/mooncar_m6_acceptance.log
      ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
if (( $# > 1 )); then
  echo "Usage: $0 [--server-only|--rviz-only] [--terrain] [map.yaml]" >&2
  exit 2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy was not found at /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

cd "${workspace}"
source /opt/ros/jazzy/setup.bash

echo "[1/4] Building MoonCar workspace"
colcon build --symlink-install
source install/setup.bash
set -u

map_yaml="${1:-$(ros2 pkg prefix mooncar_navigation)/share/mooncar_navigation/maps/lunar_yard.yaml}"
if [[ ! -f "${map_yaml}" ]]; then
  echo "Map YAML was not found: ${map_yaml}" >&2
  exit 1
fi

echo "[2/4] Starting simulation (Gazebo GUI: ${use_gz_gui}, RViz: ${use_rviz})"
rm -f "${launch_log}"
launch_args=(map:="${map_yaml}" use_rviz:="${use_rviz}")
if [[ "${use_gz_gui}" == false ]]; then
  launch_args+=(gz_args:="-r -s -v 2 --render-engine ogre")
fi
setsid ros2 launch mooncar_navigation "${launch_file}" "${launch_args[@]}" \
  >"${launch_log}" 2>&1 &
launch_pid=$!

echo "[3/4] Waiting for Nav2 lifecycle nodes"
if ! ros2 run mooncar_navigation navigation_smoke_test.py --ros-args \
  -p ready_only:=true -p server_timeout_s:=90.0; then
  echo "Nav2 did not become active. Launch log:" >&2
  tail -n 80 "${launch_log}" >&2
  exit 1
fi

echo "[4/4] Running obstacle-navigation goal"
if ! ros2 run mooncar_navigation "${test_file}" --ros-args \
  -p use_sim_time:=true \
  -p goal_x:=3.5 -p goal_y:=3.0 \
  -p goal_timeout_s:=180.0 -p min_displacement_m:=2.0; then
  echo "Navigation acceptance failed. Launch log: ${launch_log}" >&2
  tail -n 40 "${launch_log}" >&2
  exit 1
fi

echo
echo "${milestone} acceptance passed."
echo "Map: ${map_yaml}"
echo "Launch log: ${launch_log}"

if [[ "${use_rviz}" == true && -t 0 ]]; then
  read -r -p "Press Enter to stop simulation and close its windows..."
fi
