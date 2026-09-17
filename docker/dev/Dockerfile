# Local development image for the classical stack (devkit_ws), not a submission.
#
#     docker build -t roboracer-dev:iros-compete docker/dev
#
# The repo is bind-mounted at /root/Documents/roboracer so racer_common.frames
# (REPO = ~/Documents/roboracer) resolves maps and racelines unchanged, and
# devkit_ws is built in place from the mount by docker/dev/race.sh.
ARG BASE_TAG=2026-iros-compete
FROM autodriveecosystem/autodrive_roboracer_api:${BASE_TAG}

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-nav2-amcl \
        ros-humble-nav2-map-server \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-slam-toolbox \
        ros-humble-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions \
        python3-scipy python3-yaml python3-matplotlib python3-pil python3-skimage \
    && rm -rf /var/lib/apt/lists/*

# Raceline generation (raceline/centreline_editor/make_raceline.py). quadprog's
# sdist build is broken against this numpy; the manylinux wheel works, and tph's
# own dependency pins would pull that sdist, hence --no-deps.
RUN pip install --no-cache-dir --only-binary=:all: 'quadprog==0.1.13' \
    && pip install --no-cache-dir --no-deps trajectory_planning_helpers==0.79

ENV ROS_LOCALHOST_ONLY=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /root/Documents/roboracer
ENTRYPOINT []
CMD ["bash"]
