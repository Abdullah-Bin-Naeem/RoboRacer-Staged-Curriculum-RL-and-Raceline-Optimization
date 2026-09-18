#!/bin/bash
# Runs on the multi-track line family (raceline/opt_mintime.py), everything else
# identical to S03 (accel campaign best: 9.436 s mean over 43 laps).
#   run_mt.sh NAME LINE_CSV DISTANCE_SOURCE [extra launch args...]
#     DISTANCE_SOURCE: encoder (as S03) | slip (dead_reckoning.py wheelspin correction)
# Run inside rr_bridge, against the persistent 45 Hz bridge. The simulator must
# be reset + connected BY HAND first; never publish /autodrive/reset_command.
set -e
NAME=$1; LINE=$2; DIST=${3:-encoder}; shift 3 || true
[ -n "$NAME" ] && [ -f "/root/Documents/roboracer/raceline/iros2026/$LINE" ] || { echo "usage: $0 NAME LINE_CSV [encoder|slip]"; exit 2; }
cd /root/Documents/roboracer
source /opt/ros/humble/setup.bash
source devkit_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(ros2 pkg prefix racer_common)/share/racer_common/config/cyclonedds.xml
exec ros2 launch racer_bringup race.launch.py \
  bridge:=false rviz:=false track:=iros2026 \
  path_csv:=/root/Documents/roboracer/raceline/iros2026/$LINE \
  log_csv:=/root/Documents/roboracer/logs/$NAME.csv \
  distance_source:=$DIST \
  control_hz:=45 warmup_v_max:=2.0 warmup_dist_m:=21.0 enc_rate_window_s:=0.10 \
  amcl_params_file:=/root/Documents/roboracer/experiments/iros2026/params/amcl_beams360.yaml \
  exit_guard_from:=0.0 exit_guard_full:=0.0 \
  controller_mode:=hybrid_lqr lqr_k_lat:=0.03 lqr_k_head:=0.05 lqr_k_yaw:=0.0 \
  lqr_max_correction_rad:=0.02 \
  recover_settle_s:=2.0 recover_creep_s:=3.0 \
  "$@"
