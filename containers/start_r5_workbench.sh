#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p /work /evidence/sessions
cd /work
colcon --log-base /work/log build --base-paths /source/ros2_ws/src \
  --build-base /work/build --install-base /work/install --merge-install
source /work/install/setup.bash
# Refuse an unqualified runtime before presenting the candidate in the UI.
python3 -B -c 'import sys; sys.path.insert(0,"/work/install/share/mooncar_gazebo/launch"); import support_profile; support_profile.verify("/opt/mooncar/turn/r5-v2")'
python3 -B /source/scripts/runtime_baseline.py verify \
  --manifest /evidence/runtime-source.json --install /work/install \
  --output /evidence/runtime-baseline.json
export MOONCAR_RUNTIME_MANIFEST=/evidence/runtime-baseline.json
exec python3 -B /source/scripts/map_workbench.py --sessions /evidence/sessions \
  --simulation --host 0.0.0.0 --port 8776
