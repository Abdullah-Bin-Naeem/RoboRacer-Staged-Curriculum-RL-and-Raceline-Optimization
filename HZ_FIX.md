# The loop rate: what was wrong, the fix, and how to control it

**The problem.** Every bridge topic arrived at ~18 Hz when the simulator and the
devkit ran on one machine. The simulator sends its next frame only after the
bridge replies, and on one host each reply was stuck for ~40 ms in a TCP quirk:
Nagle's algorithm held the simulator's message waiting for an acknowledgment
that Linux delays on purpose. Not rendering, not CPU, not the network.
Measured 2026-09-12 (HUD 144 fps while the loop ran at 19 Hz).

**The fix.** `tools/libnodelay.so`, preloaded into the bridge process, makes the
kernel acknowledge immediately after every send. The devkit package is
untouched. Same laptop, every topic: 18.6 Hz -> 77-85 Hz.

**Build it once** (Linux; built and tested on Ubuntu 22.04, and it loads inside
the official image):

    gcc -shared -fPIC -O2 -o tools/libnodelay.so tools/nodelay.c -ldl

## Can you control the Hz on your run? Yes, on Linux

With the shim preloaded, `NODELAY_CAP_HZ=<hz>` pins the loop at that rate (it
paces the bridge's replies; the simulator follows). Without the variable the
loop runs as fast as your machine allows; without the shim you get the old
~18 Hz. Windows or macOS hosts: no, `LD_PRELOAD` and `TCP_QUICKACK` are Linux.
What a given machine does uncapped is not predictable: measure it.

## The command we used, one run per cap

Dev mode is required for the per-tick CSV. Reset the car to the spawn and
reconnect the simulator between runs, or the next run seeds localization in
the wrong place.

    HZ=45
    mkdir -p logs
    docker run -d --rm --name autodrive_roboracer_api --network=host --ipc=host \
      -v "$PWD/tools/libnodelay.so:/home/libnodelay.so:ro" \
      -v "$PWD/logs:/home/autodrive_devkit/log" \
      -e LD_PRELOAD=/home/libnodelay.so -e NODELAY_CAP_HZ=$HZ \
      -e RACER_MODE=dev \
      -e RACER_PATH_CSV=/home/autodrive_devkit/install/roboracer_stack/share/roboracer_stack/raceline/raceline_a7.0_rec_6.36.csv \
      -e RACER_EXTRA_ARGS="log_csv:=/home/autodrive_devkit/log/run_cap${HZ}.csv" \
      autodrive_racer:qualification-1

    tail -f "$(ls -t logs/racer_*.log | head -1)" | grep "lap "   # watch laps
    docker rm -f autodrive_roboracer_api                          # stop

Output on the host: `logs/run_cap<HZ>.csv` (per tick, 20 Hz, truth vs
estimate plus the follower's status) and `logs/racer_<stamp>.log` (lap times).
The CSV logger needs scipy in the image; the Dockerfile has the layer, so
`./scripts/build.sh` once if your image predates it.

## Seeing the rate

    # on the host (needs ROS Humble installed)
    source /opt/ros/humble/setup.bash
    export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file://$PWD/roboracer_stack/config/cyclonedds.xml   # else: "Failed to find a free participant index"
    python3 tools/topic_rates.py 15 /autodrive/roboracer_1/

    # or inside the running container
    ./scripts/hz.sh

## Notes

- `-e LD_PRELOAD` reaches every process in the container, not only the bridge.
  Fine for measuring; a submission image would enable it from the bridge
  launch file instead. It is not in the submission image today.
- The organizers say their machine runs 40-50 Hz. `NODELAY_CAP_HZ=45` is how
  to tune at that rate on a faster laptop.
- The results of running this at 20/40/45/50/60/70/80 Hz are in
  `HZ_ANALYSIS.md`; the full measurement story, experiment by experiment, is
  `LOOP_RATE.md`.
