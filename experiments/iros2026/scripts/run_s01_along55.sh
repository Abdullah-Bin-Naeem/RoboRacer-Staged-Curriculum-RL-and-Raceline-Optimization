#!/bin/bash
# S01: raceline a_long 5.0 -> 5.5, everything else identical to A03 (the control).
# A03 = hybrid_lqr steering, 38 clean laps, mean 9.594 s.
set -e
cd /root/Documents/roboracer
source /opt/ros/humble/setup.bash
source devkit_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(ros2 pkg prefix racer_common)/share/racer_common/config/cyclonedds.xml
ros2 launch racer_bringup race.launch.py \
  bridge:=false rviz:=false track:=iros2026 \
  path_csv:=/root/Documents/roboracer/raceline/iros2026/raceline_tum_iqp_h7.0_L55_a7.0b.csv \
  log_csv:=/root/Documents/roboracer/logs/accel_s01.csv \
  control_hz:=45 warmup_v_max:=2.0 warmup_dist_m:=21.0 enc_rate_window_s:=0.10 \
  amcl_params_file:=/root/Documents/roboracer/experiments/iros2026/params/amcl_beams360.yaml \
  exit_guard_from:=0.0 exit_guard_full:=0.0 \
  controller_mode:=hybrid_lqr lqr_k_lat:=0.03 lqr_k_head:=0.05 lqr_k_yaw:=0.0 \
  lqr_max_correction_rad:=0.02 \
  recover_settle_s:=2.0 recover_creep_s:=3.0
