#!/usr/bin/env bash
set -eo pipefail
bash /source/scripts/prepare_r5_turn_control.sh
source /opt/ros/jazzy/setup.bash
source /work/install/setup.bash
export PYTHONPATH=/source/scripts${PYTHONPATH:+:$PYTHONPATH}
cmake -S /source/containers/r5_slip_observer -B /work/slip-observer-build > /evidence/slip-observer-configure.log 2>&1
cmake --build /work/slip-observer-build -j4 > /evidence/slip-observer-build.log 2>&1
cp /work/slip-observer-build/libmooncar-origin-observer.so /evidence/observer/
python3 - <<'PY'
from r5_slip_probe import generate
import json
from pathlib import Path
manifest=generate('/evidence/r5-d0.sdf','/evidence/slip-models','/evidence/authorization.json')
prediction=dict(configuration=manifest,mechanism_probe_maximum_excursion_m=.03,yaw_loss_limit_rad=.05,
                previous_one_cm_screen_unchanged=True,production_change=False,not_hardware_calibration=True,
                scope='Flat test-only mechanism diagnostic, no whole vehicle acceptance')
Path('/evidence/prediction.json').write_text(json.dumps(prediction,indent=2)+'\n')
PY
python3 -m unittest discover -s /source/ros2_ws/src/mooncar_gazebo/tests -p test_slip_probe.py -v > /evidence/slip-probe-tests.log 2>&1
ldd /evidence/observer/libmooncar-origin-observer.so > /evidence/slip-observer-dependencies.txt
gz sim --versions > /evidence/gazebo-versions.txt 2>&1
