#!/usr/bin/env bash
set -eo pipefail
if [ "${1:-}" != --budget-inner ]; then
  stamp=/evidence/build-start.txt
  if [ ! -f "$stamp" ]; then date -u +%FT%TZ > "$stamp"; fi
  remaining=$((2700 - $(date +%s) + $(date -d "$(cat "$stamp")" +%s)))
  if [ "$remaining" -le 0 ]; then echo 'DDS build/configuration budget exhausted' >&2; exit 1; fi
  exec timeout --signal=TERM --kill-after=5 "$remaining" bash "$0" --budget-inner
fi
source /opt/ros/jazzy/setup.bash
prefix=/work/dds-observed
for p in fastdds rmw_fastrtps_shared_cpp rmw_fastrtps_cpp; do
  git -C /work/dds/$p restore .
done
python3 /source/containers/r5_dds_observer/patch_runtime.py
cmake -S /source/containers/r5_dds_observer -B /work/dds/trace-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix
cmake --build /work/dds/trace-build -j4
cmake --install /work/dds/trace-build
flags="-L$prefix/lib -Wl,--no-as-needed -lmooncar-dds-trace -Wl,-rpath,$prefix/lib"
cmake -S /work/dds/fastdds -B /work/dds/fastdds-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix -DCMAKE_PREFIX_PATH=/opt/ros/jazzy -DINSTALL_EXAMPLES=OFF -DSECURITY=ON -DAPPEND_PROJECT_NAME_TO_INCLUDEDIR=ON -DBUILD_TESTING=OFF -DCMAKE_SHARED_LINKER_FLAGS="$flags"
cmake --build /work/dds/fastdds-build -j4
cmake --install /work/dds/fastdds-build
for p in rmw_fastrtps_shared_cpp rmw_fastrtps_cpp; do
  cmake -S /work/dds/$p -B /work/dds/$p-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix -DCMAKE_PREFIX_PATH="$prefix;/opt/ros/jazzy" -DBUILD_TESTING=OFF -DCMAKE_SHARED_LINKER_FLAGS="$flags"
  cmake --build /work/dds/$p-build -j4
  cmake --install /work/dds/$p-build
done
date -u +%FT%TZ > /evidence/build-finished.txt
