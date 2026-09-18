#!/usr/bin/env bash
# Runs inside roboracer-dev: builds devkit_ws from the mounted repo, launches
# race.launch.py, and stops the stack once the lap counter reaches LAPS.
#
#   docker/dev/race.sh <track> <raceline csv, relative to raceline/<track>/> <laps> <log name>
set -eo pipefail
TRACK=${1:-iros2026}; LINE=${2:-raceline_tum_iqp.csv}; LAPS=${3:-25}; NAME=${4:-run}
cd /root/Documents/roboracer
source /opt/ros/humble/setup.bash
( cd devkit_ws && PYTHONNOUSERSITE=1 colcon build 2>&1 | tail -5 )
source devkit_ws/install/setup.bash
export CYCLONEDDS_URI=file://$(ros2 pkg prefix racer_common)/share/racer_common/config/cyclonedds.xml
mkdir -p logs
ros2 launch racer_bringup race.launch.py track:=$TRACK rviz:=false \
    path_csv:=$PWD/raceline/$TRACK/$LINE log_csv:=$PWD/logs/$NAME.csv &
LAUNCH=$!
trap 'kill -INT $LAUNCH 2>/dev/null; wait $LAUNCH' EXIT
# lap_count is restricted; read here only to end a development run.
python3 - "$LAPS" <<'PY'
import sys, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Float32
laps = int(sys.argv[1]); rclpy.init(); n = rclpy.create_node('lap_watch')
qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=1)
st = {'lap': -1, 'last': 0.0, 'col': 0}
n.create_subscription(Float32, '/autodrive/roboracer_1/last_lap_time', lambda m: st.update(last=m.data), qos)
n.create_subscription(Int32, '/autodrive/roboracer_1/collision_count', lambda m: st.update(col=m.data), qos)
def on_lap(m):
    if m.data != st['lap']:
        st['lap'] = m.data
        print(f"LAPWATCH lap={m.data} last_lap={st['last']:.3f} collisions={st['col']}", flush=True)
n.create_subscription(Int32, '/autodrive/roboracer_1/lap_count', on_lap, qos)
while rclpy.ok() and st['lap'] < laps:
    rclpy.spin_once(n, timeout_sec=0.5)
print(f"LAPWATCH done: {laps} laps", flush=True)
PY
