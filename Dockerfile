# IROS 2026 final submission image: localizer v2 + pure pursuit.
#
#     docker build -t <you>/roboracer:iros-2026-final .
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
# The provided autodrive_roboracer package is not in this repository at all:
# the base image ships it built, and the rules forbid modifying it.

ARG BASE_TAG=2026-iros-practice
FROM autodriveecosystem/autodrive_roboracer_api:${BASE_TAG}

# The base image has Python 3.10, numpy and opencv. Two additions, and no nav2:
#
#   python3-scipy       the localizer's likelihood field is a distance
#                       transform of the occupancy grid. scan_matcher.py has a
#                       pure-numpy fallback for development hosts, but scipy is
#                       what every measured lap time ran on.
#   rmw-cyclonedds-cpp  likewise: every lap time on this branch was measured on
#                       cyclone, and the base image ships only fastrtps.
#
# NO nav2. The previous submission ran AMCL, which needs a map server and a
# lifecycle manager to go with it. This localizer reads the PGM itself and is a
# plain node, so three packages and their dependency trees leave the image.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-scipy \
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

# The LD_PRELOAD shim bridge.launch.py preloads into the devkit bridge, built
# into the same share directory frames.NODELAY_SHIM points at. It sets
# TCP_NODELAY on the bridge's websocket and re-arms TCP_QUICKACK after every
# recv AND every send: without it Nagle's algorithm and the receiver's delayed
# ACK hold the request/reply loop at 10-20 Hz, whatever the machine. Measured
# here on the lidar topic: 18.6 Hz off, 77.3 Hz on.
RUN gcc -shared -fPIC -O2 \
        -o /home/autodrive_devkit/install/roboracer_stack/share/roboracer_stack/tools/libnodelay.so \
        /home/autodrive_devkit/install/roboracer_stack/share/roboracer_stack/tools/nodelay.c -ldl

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
