# Sourced by the in-container scripts: ROS, the workspace, DDS settings.
cd /root/Documents/roboracer
source /opt/ros/humble/setup.bash
if [ ! -f devkit_ws/install/setup.bash ] || [ "${REBUILD:-0}" = 1 ]; then
    ( cd devkit_ws && PYTHONNOUSERSITE=1 colcon build 2>&1 | tail -3 )
fi
source devkit_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(ros2 pkg prefix racer_common)/share/racer_common/config/cyclonedds.xml
fix_owner() { [ -n "$HOST_UID" ] && chown -R "$HOST_UID:$HOST_GID" "$@" 2>/dev/null || true; }
