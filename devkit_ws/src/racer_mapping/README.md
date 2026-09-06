# racer_mapping

Map an AutoDRIVE track with slam_toolbox. Copy-paste, one terminal each.

## Once

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server
mkdir -p ~/Documents/roboracer/devkit_ws/src/racer_mapping/maps
```

## T1 — sim

```bash
cd ~/Documents/roboracer/simulator_practice/autodrive_simulator
./"AutoDRIVE Simulator.x86_64"
```

## T2 — bridge

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/Documents/roboracer/devkit_ws/install/setup.bash

ros2 launch autodrive_roboracer bringup_headless.launch.py
```

## T3 — drive (pick one)

**RL policy** — swap the checkpoint path for whichever run you want:

```bash
cd ~/Documents/roboracer/rl_racer
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/Documents/roboracer/.venv-rl/bin/activate

python enjoy.py runs/stage2_sensors/checkpoints/sac_520000_steps.zip --episodes 1 --hz 7.7
# python enjoy.py runs/stage3_v3/checkpoints/sac_990000_steps.zip --episodes 1 --hz 7.7
```

**Manual in the sim** — switch the sim to Manual drive mode and steer there.
No T3 terminal at all.


## T4 — slam + rviz

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/Documents/roboracer/devkit_ws/install/setup.bash

ros2 launch racer_mapping mapping.launch.py
```

## Save

Stop Drive. Leave T4 running. In the RViz **SlamToolboxPlugin** panel, paste
this into **both** text boxes (they are separate) and click both buttons:

```
/home/theflash/Documents/roboracer/devkit_ws/src/racer_mapping/maps/track
```

Save Map -> `track.pgm` + `.yaml`   Serialize Map -> `track.posegraph` + `.data`

Check:

```bash
ls -la ~/Documents/roboracer/devkit_ws/src/racer_mapping/maps/
```

Then Ctrl-C T4, T2, sim.

## Notes

- Map lives in memory only — save before killing T4.
- Policy crashed early? Rerun T3, leave T4 up. The map keeps accumulating.
- Done when a full extra lap stops changing the map. Grey = unseen, black =
  wall, white = free. Check the loop closes.
- `ROS_LOCALHOST_ONLY=1` in every terminal or nodes won't see each other.
- venv only in the RL terminal.
- Two warnings at T4 startup (min laser range, clipped threshold) are harmless.

## Rebuild

```bash
cd ~/Documents/roboracer/devkit_ws
PYTHONNOUSERSITE=1 colcon build --packages-select racer_mapping
```

`PYTHONNOUSERSITE=1` required, and not from a venv prompt.

Tuning lives in [`config/slam_mapping.yaml`](config/slam_mapping.yaml) — scan
matching and loop closure are off on purpose, since sim odometry is ground truth.
