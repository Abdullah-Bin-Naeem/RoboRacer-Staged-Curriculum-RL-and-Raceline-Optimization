#!/usr/bin/env bash
# Stop the follower WITHOUT leaving the car driving.
#
# The simulator latches the last throttle/steering command it received, so
# killing the follower outright leaves the car rolling forever with
# "Publisher count: 0" on both command topics -- it looks stopped and is not.
# A rolling car keeps banking collisions and lap events against the cumulative
# counters and contaminates the next run's baseline.
#
# Order matters: KILL FIRST, then zeros, then verify. Zeros sent while the
# follower still lives are overridden by its next tick, and the command it
# was publishing when it died is the one the simulator keeps holding.
# NOTE: no `set -u` -- /opt/ros/humble/setup.bash reads unset variables
# (AMENT_TRACE_SETUP_FILES) and dies under it.
set -e
set +u
NS=/autodrive/roboracer_1
source /opt/ros/humble/setup.bash

echo "killing follower and logger by PID..."
# grep -v grep is not enough on its own: a `pkill -f pure_pursuit` also matches
# the pkill process itself and can exit non-zero having killed nothing.
for p in $(ps -eo pid,cmd | grep -E 'pure_pursuit|log_run\.py|ros2 launch racer_control' \
           | grep -v grep | awk '{print $1}'); do
    kill -9 "$p" 2>/dev/null && echo "  killed $p"
done
sleep 3

echo "sending zero throttle/steering..."
timeout 20 ros2 topic pub --times 15 $NS/throttle_command std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1
timeout 20 ros2 topic pub --times 15 $NS/steering_command std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1

echo "verifying..."
timeout 10 ros2 topic info $NS/throttle_command 2>&1 | grep Publisher
timeout 8 ros2 topic echo $NS/odom --once 2>&1 | grep -A1 'linear:' | tail -1 \
  | sed 's/^/  odom linear.x:/'
echo "(x must read 0.0 -- Publisher count 0 alone does NOT mean the car stopped)"
