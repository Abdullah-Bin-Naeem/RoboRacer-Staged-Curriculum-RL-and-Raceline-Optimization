# RoboRacer RL — AutoDRIVE Sim Racing League 2026

SAC policy driving the AutoDRIVE RoboRacer (F1TENTH) simulator straight off
LiDAR: no map, no localization, race-legal sensors only. Branch
`rl-fresh-fov110` holds the RL stack and the devkit bridge it needs; the
classical stack lives on `multi-track` / `main`.

## Quick start

```bash
cd devkit_ws && PYTHONNOUSERSITE=1 colcon build && cd ..
gcc -shared -fPIC -O2 -o tools/libnodelay.so tools/nodelay.c -ldl
python3 -m venv .venv-rl && . .venv-rl/bin/activate && pip install stable-baselines3 torch gymnasium numpy tensorboard

# simulator running (select the competition track), then:
source ros_env.sh && ros2 launch racer_bringup bridge.launch.py tcp_nodelay:=true loop_hz_cap:=45
source ros_env.sh && source .venv-rl/bin/activate && cd rl_racer
python probe.py && ./run_train.sh --stage 5
```

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
