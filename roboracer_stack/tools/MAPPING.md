# How the shipped map was made

`maps/track_clean.*` and `maps/track_sm.*` are finished artefacts — the race
stack never runs SLAM. This is the provenance, and the procedure to redo it if
the track ever changes.

`track_sm` is what came out of the mapping run: the occupancy grid *and* the
serialized pose graph, from the same session, which is why it is the map to use
when debugging the slam localizer. `track_clean` is `track_sm` put through
[`clean_map.py`](clean_map.py) — speckle and stray-blob removal — and is what
AMCL localizes against.

## Once

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server
```

## T1 — simulator

```bash
./scripts/run.sh sim
```

## T2 — bridge

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch autodrive_roboracer bringup_headless.launch.py
```

## T3 — drive

Switch the simulator to Manual drive mode and steer there. Slow and smooth; the
map only needs geometry, not pace.

## T4 — slam + rviz

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch roboracer_stack mapping.launch.py
```

## Save

Stop driving. Leave T4 running. In the RViz **SlamToolboxPlugin** panel, paste
the destination into **both** text boxes (they are separate) and click both
buttons:

```
<repo>/roboracer_stack/maps/track_sm
```

Save Map → `track_sm.pgm` + `.yaml`   Serialize Map → `track_sm.posegraph` + `.data`

Then clean it:

```bash
python3 roboracer_stack/tools/clean_map.py     # needs scipy; offline only
```

Then Ctrl-C T4, T2, the sim.

## Notes

- The map lives in memory only — save before killing T4.
- Done when a full extra lap stops changing the map. Grey = unseen, black =
  wall, white = free. Check that the loop closes.
- `ROS_LOCALHOST_ONLY=1` in every terminal or the nodes will not see each other.
- Two warnings at T4 startup (min laser range, clipped threshold) are harmless.
- Tuning lives in [`config/slam_mapping.yaml`](../config/slam_mapping.yaml) —
  scan matching and loop closure are off on purpose, since sim odometry is
  ground truth.
