#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
for package in mooncar_control mooncar_navigation mooncar_terrain; do
  python3 -m unittest discover -s "$root/ros2_ws/src/$package/tests" -v
done
temp="$(mktemp -d)"
trap 'rm -f "$temp/test_core"; rmdir "$temp"' EXIT
gcc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  -I "$root/firmware/stm32/core" "$root/firmware/stm32/core/mooncar_core.c" \
  "$root/firmware/stm32/tests/test_core.c" -lm -o "$temp/test_core"
"$temp/test_core"
echo 'Software regression checks passed.'
