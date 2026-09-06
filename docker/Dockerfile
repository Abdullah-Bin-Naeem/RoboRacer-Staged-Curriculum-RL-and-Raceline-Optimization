# Qualification 1 submission image: AMCL + pure pursuit on the Porto track.
#
#     docker build -f docker/Dockerfile -t <you>/roboracer:qualification-1 .
#     (or just: ./docker/build.sh)
#
# The rules require the submission to derive from the official devkit image and
# forbid modifying the autodrive_roboracer package, so this starts FROM that
# image and adds our packages in a SEPARATE workspace at /home/racer_ws. The
# devkit at /home/autodrive_devkit is left exactly as the base image built it --
# where its behaviour needs changing we do it with a launch-level remap
# (racer_bringup/bridge.launch.py), never by editing it.
#
# devkit_ws/src/autodrive_devkit/ is committed on this branch for reference --
# the bridge source is what documents the topic names and QoS every node here
# matches -- but it is deliberately NOT copied into the image. The base image
# already ships it, built; overwriting it would be exactly the modification the
# rules prohibit. See .dockerignore.

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

# Our five packages. Copied one by one rather than as devkit_ws/src, so that
# adding autodrive_devkit back to that directory can never leak into the image.
COPY devkit_ws/src/racer_common       /home/racer_ws/src/racer_common
COPY devkit_ws/src/racer_localization /home/racer_ws/src/racer_localization
COPY devkit_ws/src/racer_control      /home/racer_ws/src/racer_control
COPY devkit_ws/src/racer_mapping      /home/racer_ws/src/racer_mapping
COPY devkit_ws/src/racer_bringup      /home/racer_ws/src/racer_bringup

# The track is baked in: racer_mapping installs maps/ and racer_control installs
# raceline/ into their package shares, and racer_common.frames resolves both
# through the ament index. Nothing here depends on where the repo was cloned.
RUN bash -c 'source /opt/ros/humble/setup.bash \
    && source /home/autodrive_devkit/install/setup.bash \
    && cd /home/racer_ws \
    && colcon build --packages-select racer_common racer_localization \
                    racer_control racer_mapping racer_bringup'

COPY docker/racer_env.sh    /home/racer_env.sh
COPY docker/start_racer.sh  /home/start_racer.sh
COPY docker/autodrive_devkit.sh /home/autodrive_devkit.sh
RUN chmod +x /home/start_racer.sh /home/autodrive_devkit.sh

# So that `docker exec -it <container> bash` can run ros2 commands. This sources
# the environment and NOTHING ELSE -- the rules forbid automating startup from
# ~/.bashrc, and re-launching the stack in every inspection shell is precisely
# what that rule exists to prevent.
RUN echo 'source /home/racer_env.sh' >> /root/.bashrc

WORKDIR /home/racer_ws
EXPOSE 4567
ENTRYPOINT ["/home/autodrive_devkit.sh"]
CMD ["bash"]
