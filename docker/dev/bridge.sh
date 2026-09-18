#!/usr/bin/env bash
# Persistent bridge for a tuning session: started once, Connect pressed once,
# experiments come and go with bridge:=false against it.
#   HZ_CAP=stock  -> the stock devkit socket path (~18 Hz here)
#   HZ_CAP=<n>    -> tools/libnodelay.so preloaded, loop paced to n Hz (0 = uncapped)
set -eo pipefail
REBUILD=1 source /root/Documents/roboracer/docker/dev/common.sh
fix_owner devkit_ws/build devkit_ws/install devkit_ws/log
if [ "${HZ_CAP:-stock}" = stock ]; then
    exec ros2 launch racer_bringup bridge.launch.py
fi
[ -f tools/libnodelay.so ] || gcc -shared -fPIC -O2 -o tools/libnodelay.so tools/nodelay.c -ldl
fix_owner tools/libnodelay.so
exec ros2 launch racer_bringup bridge.launch.py tcp_nodelay:=true loop_hz_cap:=${HZ_CAP}
