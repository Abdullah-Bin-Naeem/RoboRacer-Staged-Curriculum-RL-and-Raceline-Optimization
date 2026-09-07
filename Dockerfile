# Qualification 1 submission image: AMCL + pure pursuit on the Porto track.
#
#     docker build -t <you>/roboracer:qualification-1 .
#     (or just: ./scripts/build.sh)
#
# The rules require the submission to derive from the official devkit image and
# forbid modifying the autodrive_roboracer package, so this starts FROM that
# image and adds roboracer_stack as a SEPARATE package beside it in the devkit's
# own workspace -- which is exactly what this repository's root directory is a
# copy of. autodrive_roboracer itself is never rebuilt and never edited; where
# its behaviour needs changing we do it with a launch-level remap
# (roboracer_stack/launch/bridge.launch.py).
#
# autodrive_devkit/ is committed on this branch for reference -- the bridge
# source is what documents the topic names and QoS every node here matches --
# but it is deliberately NOT copied into the image. The base image already ships
# it, built; overwriting it would be exactly the modification the rules
# prohibit. See .dockerignore.

ARG BASE_TAG=2026-iros-practice
FROM autodriveecosystem/autodrive_roboracer_api:${BASE_TAG}

# The base image has Python 3.10, numpy, opencv, laser_geometry and rviz2 -- and
# no nav2 and no slam_toolbox. Everything below is a localizer dependency:
#
#   nav2-amcl              the localizer this branch is qualified on
#   nav2-map-server        serves maps/track_clean.pgm to it
#   nav2-lifecycle-manager amcl and map_server are lifecycle nodes; without the
#                          manager they come up UNCONFIGURED and silently do
#                          nothing
#   slam-toolbox           the `localizer:=slam` fallback, kept because it is a
#                          working second option (6.7 s) if AMCL misbehaves on
#                          the evaluation machine
#   rmw-cyclonedds-cpp     every lap time on this branch was measured on cyclone
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-nav2-amcl \
        ros-humble-nav2-map-server \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-slam-toolbox \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Our package, into the devkit workspace's src/ beside autodrive_roboracer.
# --packages-select means autodrive_roboracer is not rebuilt and its sources are
# never touched; the rule is about the package, not the workspace.
#
# The track is baked in: setup.py installs maps/ and raceline/ into the package
# share and roboracer_stack.common.frames resolves both through the ament index,
# so nothing depends on where the repo was cloned.
COPY roboracer_stack /home/autodrive_devkit/src/roboracer_stack
RUN bash -c 'source /opt/ros/humble/setup.bash \
    && cd /home/autodrive_devkit \
    && colcon build --packages-select roboracer_stack'

# DDS settings as image environment rather than shell startup files. Every
# process in the container inherits them, `docker exec` shells included, which
# is what lets the organizers run `ros2 topic echo` and `ros2 bag record`
# against our nodes at all -- a shell on a different RMW simply sees no nodes.
# ENV is not ~/.bashrc and starts nothing; the entrypoint exports the same
# values itself so that `--entrypoint /bin/bash` behaves identically.
#
#   ROS_LOCALHOST_ONLY   the simulator is reached over a TCP socket on 4567, not
#                        over DDS, so nothing needs to leave the container.
#   RMW_IMPLEMENTATION   cyclonedds; every lap time here was measured on it.
#   CYCLONEDDS_URI       raises MaxAutoParticipantIndex above its default of 9.
#                        race.launch.py is about ten nodes, so without this a
#                        different node dies each run.
ENV ROS_LOCALHOST_ONLY=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///home/autodrive_devkit/install/roboracer_stack/share/roboracer_stack/config/cyclonedds.xml

# The entrypoint the rules name, replacing the stub the base image ships. It is
# the ONLY automation in this image: nothing is appended to ~/.bashrc, so the
# organizers' extra bash sessions start no nodes.
COPY autodrive_devkit.sh /home/autodrive_devkit.sh
RUN chmod +x /home/autodrive_devkit.sh

WORKDIR /home/autodrive_devkit
EXPOSE 4567
ENTRYPOINT ["/home/autodrive_devkit.sh"]
CMD ["bash"]
