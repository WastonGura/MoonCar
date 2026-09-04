#!/usr/bin/env bash
set -eo pipefail
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [[ -z "${DISPLAY:-}" ]]; then
  command -v xvfb-run >/dev/null || { echo 'Install xvfb and xauth for SSH/server-only acceptance.' >&2; exit 1; }
  exec xvfb-run -a -s '-screen 0 1280x800x24' "$0" "$@"
fi

use_rviz=true
if [[ "${1:-}" == --server-only ]]; then
  use_rviz=false
  shift
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${repo_root}/ros2_ws"
map_prefix="${1:-${HOME}/mooncar_m4_acceptance}"
launch_log="/tmp/mooncar_m4_acceptance.log"
launch_pid=""

cleanup() {
  local status=$?
  trap - EXIT

  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 -- "-${launch_pid}" 2>/dev/null || break
      sleep 0.5
    done
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  fi
  [[ -z "${launch_pid}" ]] || wait "${launch_pid}" 2>/dev/null || true

  exit "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

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

echo "[2/4] Starting Gazebo, SLAM, EKF, and RViz"
rm -f "${launch_log}"
launch_args=(use_rviz:="${use_rviz}")
if [[ "${use_rviz}" == false ]]; then
  launch_args+=(gz_args:="-r -s -v 2 --render-engine ogre")
fi
setsid ros2 launch mooncar_slam mapping.launch.py "${launch_args[@]}" >"${launch_log}" 2>&1 &
launch_pid=$!

echo "[3/4] Waiting for laser data"
deadline=$((SECONDS + 60))
until ros2 topic type /mooncar/scan >/dev/null 2>&1; do
  if ! kill -0 "${launch_pid}" 2>/dev/null || ((SECONDS >= deadline)); then
    echo "MoonCar simulation did not create the laser topic. Launch log:" >&2
    tail -n 80 "${launch_log}" >&2
    exit 1
  fi
  sleep 1
done
if ! timeout -k 2s 60s ros2 topic echo --once /mooncar/scan >/dev/null; then
  echo "MoonCar simulation did not publish laser data. Launch log:" >&2
  tail -n 80 "${launch_log}" >&2
  exit 1
fi

echo "[4/4] Running drive and mapping checks"
ros2 run mooncar_control drive_smoke_test.py
mkdir -p "$(dirname "${map_prefix}")"
ros2 run mooncar_slam mapping_smoke_test.sh "${map_prefix}"

echo
echo "M4 acceptance passed."
echo "Map: ${map_prefix}.yaml"
echo "Launch log: ${launch_log}"

if [[ "${use_rviz}" == true && -t 0 ]]; then
  read -r -p "Press Enter to close Gazebo and RViz..."
fi
