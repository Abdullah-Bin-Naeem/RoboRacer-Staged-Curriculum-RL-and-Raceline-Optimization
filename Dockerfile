# Multi-track development image: the multi-track classical stack, runnable
# without ROS on the host.
#
#     ./scripts/build.sh                       -> autodrive_racer:multi-track
#     ./scripts/run.sh racer track:=iros2026 tcp_nodelay:=true loop_hz_cap:=45 log_csv:=run_iros_21.csv
#
# Starts FROM the official devkit image, which already ships autodrive_roboracer
# built in /home/autodrive_devkit (byte-identical to devkit_ws/src/autodrive_devkit
# on this branch, checked 2026-09-17). Our racer_* packages are built in their
# own workspace on top of it, so the devkit is never rebuilt or edited.
#
# WHERE THE REPO LIVES IN THE IMAGE
# ---------------------------------
# racer_common/frames.py resolves maps, racelines and tools/libnodelay.so from
# REPO = ~/Documents/roboracer, at run time, not from the install tree. The
# container runs as root, so the tree goes to /root/Documents/roboracer and no
# code has to change. scripts/run.sh bind-mounts the host's raceline/ over the
# baked copy, so a new line can be raced without rebuilding.

ARG BASE_TAG=2026-iros-practice
FROM autodriveecosystem/autodrive_roboracer_api:${BASE_TAG}

# The base image has Python 3.10, numpy, rviz2, tf2_ros and gcc; no nav2, no
# slam_toolbox, no scipy.
#
#   nav2-amcl / map-server / lifecycle-manager   localizer:=amcl (the default)
#   slam-toolbox                                 localizer:=slam
#   rmw-cyclonedds-cpp                           every lap time was measured on it
#   python3-scipy                                log_localization (log_csv:=)
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-nav2-amcl \
        ros-humble-nav2-map-server \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-slam-toolbox \
        ros-humble-rmw-cyclonedds-cpp \
        python3-scipy \
    && rm -rf /var/lib/apt/lists/*

ENV RACER_REPO=/root/Documents/roboracer
WORKDIR ${RACER_REPO}

# Only what the stack reads at run time: the workspace sources, the maps
# (inside devkit_ws), the racelines, the socket shim and the analysis script.
COPY devkit_ws/src ${RACER_REPO}/devkit_ws/src
COPY raceline ${RACER_REPO}/raceline
COPY tools ${RACER_REPO}/tools
COPY ros_env.sh ${RACER_REPO}/ros_env.sh

# The multi-track race config is not all in the registry: run_mt.sh passes
# amcl_params_file:=experiments/iros2026/params/amcl_beams360.yaml, so the params
# and the launch/teardown scripts have to be in the image or the winning config
# cannot be reproduced in here. Only those two directories -- the rest of
# experiments/ is 86 MB of run artefacts that nothing reads at run time.
COPY experiments/iros2026/params ${RACER_REPO}/experiments/iros2026/params
COPY experiments/iros2026/scripts ${RACER_REPO}/experiments/iros2026/scripts

# run_mt.sh writes log_csv to logs/; scripts/run.sh bind-mounts it to the host.
RUN mkdir -p ${RACER_REPO}/logs

# tcp_nodelay:=true preloads this into the bridge (bridge.launch.py).
RUN gcc -shared -fPIC -O2 -o tools/libnodelay.so tools/nodelay.c -ldl

# autodrive_roboracer comes from the image's workspace (the underlay), so the
# vendored copy is skipped rather than built twice.
RUN bash -c 'source /opt/ros/humble/setup.bash \
    && source /home/autodrive_devkit/install/setup.bash \
    && cd devkit_ws \
    && colcon build --packages-ignore autodrive_roboracer'

# DDS settings as image environment, so `docker exec` shells see the same graph.
#   ROS_LOCALHOST_ONLY   the simulator talks to the bridge over TCP 4567, not DDS
#   CYCLONEDDS_URI       MaxAutoParticipantIndex above 9; race.launch.py is ~10 nodes
ENV ROS_LOCALHOST_ONLY=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///root/Documents/roboracer/devkit_ws/install/racer_common/share/racer_common/config/cyclonedds.xml

# Extra shells (docker exec) get the workspace sourced. This only sets the
# environment; nothing is launched from it.
RUN echo 'source /opt/ros/humble/setup.bash && source /root/Documents/roboracer/devkit_ws/install/setup.bash' >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 4567
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
