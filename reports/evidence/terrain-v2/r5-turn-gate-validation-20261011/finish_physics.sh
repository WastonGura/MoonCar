#!/usr/bin/env bash
# Post-run checks only; this entry never starts a Gazebo trial.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /work/install/setup.bash
set -u
export PYTHONPATH=/source/scripts
python3 /evidence/verify_completed.py > /evidence/completed-verification.log
python3 /evidence/analyze_turn_slip.py --plots-only > /evidence/metrics-analysis.log
python3 /evidence/archive_installed.py > /evidence/installed-verification.log
dpkg-query -W > /evidence/installed-packages.txt
uname -a > /evidence/kernel.txt
ps -eo pid,ppid,stat,args > /evidence/processes-after-physics.txt
bash /source/scripts/check_software.sh > /evidence/software-regression.log 2>&1
