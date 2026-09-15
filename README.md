# RoboRacer RL — AutoDRIVE Sim Racing League 2026

SAC policy driving the AutoDRIVE RoboRacer (F1TENTH) simulator straight off
LiDAR: no map, no localization, race-legal sensors only. Branch
`rl-fresh-fov110` holds the RL stack and the devkit bridge it needs; the
classical stack lives on `multi-track` / `main`.

## Setup on a new machine

Needs Ubuntu 22.04 with Python 3.10, an NVIDIA GPU with driver 575+ for the
CUDA 12.9 torch build (older driver: the cu126 index noted in
`rl_racer/requirements.txt`), and ~3 GB of disk per run.

```bash
# 1. ROS 2 Humble + the bits the bridge and the build need
sudo apt install ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp ros-humble-cv-bridge \
     ros-humble-tf-transformations ros-humble-imu-tools python3-colcon-common-extensions \
     python3-pip python3-venv build-essential
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# 2. clone this branch, build the bridge shim and the workspace (system python, NO venv)
git clone -b rl-fresh-fov110 <repo-url> roboracer && cd roboracer
pip3 install -r devkit_ws/src/autodrive_devkit/requirements_python_3.10.txt   # bridge deps: install EXACTLY as pinned
gcc -shared -fPIC -O2 -o tools/libnodelay.so tools/nodelay.c -ldl
(cd devkit_ws && PYTHONNOUSERSITE=1 colcon build)

# 3. the RL venv (torch CUDA 12.9 build; older driver -> see rl_racer/requirements.txt)
python3 -m venv .venv-rl && . .venv-rl/bin/activate && pip install -r rl_racer/requirements.txt && deactivate

# 4. the simulator: AutoDRIVE RoboRacer Sim Racing release (Linux build), unpacked anywhere;
#    use the build that carries the competition track
```

Every ROS terminal starts with `source ros_env.sh` (from the repo root); RL
terminals then also activate `.venv-rl`. Paths are relative to the clone.

Two things that bite on a fresh machine:

- **Source order.** ROS first, then the venv, in every RL terminal. The other
  way round torch is missing or rclpy comes from the wrong python;
  `verify_env.py` says which.
- **Sleep and screen lock.** Disable both for a multi-day run, and run the
  training inside `tmux` so a dropped SSH session does not kill it. Ctrl-C at
  any point saves `final.zip` and the replay buffer.

## Run

```bash
./AutoDRIVE\ Simulator.x86_64                     # select the track, low quality is fine; leave it running
source ros_env.sh && ros2 launch racer_bringup bridge.launch.py tcp_nodelay:=true loop_hz_cap:=45
source ros_env.sh && source .venv-rl/bin/activate && cd rl_racer
python verify_env.py && python probe.py            # imports from the venv; ~45 Hz tick
./run_train.sh --stage 5 --timesteps 3600000       # ~2 days at 20 fps (4300000 for 2.5)
python enjoy.py runs/stage5_fresh/final.zip --episodes 3          # validate after it stops
```

Resume a stopped run with `./run_train.sh --stage 5 --resume runs/stage5_fresh/final.zip --timesteps <more>`;
it loads the saved buffer. Do not run `enjoy.py` while training: the sim is
single-instance and a second client fights the trainer for the car.

`CLAUDE.md` is the full guide (environment, build, running, design, legality).
`rl_racer/README.md` describes the observation, action, reward and stages;
`rl_racer/EXPERIMENTS.md` the measured constants and the bugs found.

## Layout

| path | what |
|---|---|
| `rl_racer/` | env, config, sensors, stages 5–6, trainer, `enjoy.py`, mock-bridge test |
| `devkit_ws/src/autodrive_devkit` | third-party bridge, unmodified |
| `devkit_ws/src/racer_bringup` | `bridge.launch.py`: TCP shim, loop-rate cap, TF remap |
| `devkit_ws/src/racer_common` | workspace paths, `cyclonedds.xml` |
| `tools/` | `nodelay.c` (the shim), loop-rate probes |
