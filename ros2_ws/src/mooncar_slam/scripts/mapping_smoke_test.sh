#!/usr/bin/env bash
set -euo pipefail

timeout_seconds="${MOONCAR_SMOKE_TIMEOUT:-60}"
tf_timeout_seconds="${MOONCAR_TF_TIMEOUT:-5}"
map_prefix="${1:-/tmp/mooncar_map}"

check_topic() {
  local topic="$1"
  echo "Waiting for ${topic}"
  timeout -k 2s "${timeout_seconds}s" ros2 topic echo --once "${topic}" >/dev/null
}

check_transform() {
  local source_frame="$1"
  local target_frame="$2"
  local output
  output="$(mktemp)"

  timeout -k 2s "${tf_timeout_seconds}s" ros2 run tf2_ros tf2_echo \
    "${source_frame}" "${target_frame}" >"${output}" 2>&1 || true
  if ! grep -q "Translation:" "${output}"; then
    cat "${output}"
    rm -f "${output}"
    return 1
  fi
  rm -f "${output}"
}

check_topic /mooncar/scan
check_topic /odometry/filtered
check_topic /map
check_transform odom base_footprint
check_transform base_footprint lidar_link

rm -f "${map_prefix}.yaml" "${map_prefix}.pgm"
timeout -k 2s "${timeout_seconds}s" ros2 run nav2_map_server map_saver_cli -f "${map_prefix}"
test -s "${map_prefix}.yaml"
test -s "${map_prefix}.pgm"

echo "Mapping smoke test passed: ${map_prefix}.yaml"
