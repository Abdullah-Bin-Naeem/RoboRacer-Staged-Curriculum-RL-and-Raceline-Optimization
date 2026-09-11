# The 18 Hz loop: what it was, how we found it, how it is fixed

Every sensor topic from the bridge arrived at 18.5 Hz on the development
laptop, identical to three decimals across lidar, IMU, encoders and camera, while
the technical guide lists the lidar at 40 Hz and the organizers said to tune for
40-50 Hz. Their diagnosis was "networking between processes on one machine" and
their advice was to run the simulator and the devkit on two machines. This
document records what we measured on 2026-09-12, what the cause actually is,
and the fix, which needs no second machine: the same laptop now runs the loop
at 77-85 Hz, or at any rate we choose.

## What the loop is

The simulator does not stream. It emits one telemetry message, all sensors at
once, only in reply to the bridge's message, and the bridge publishes the ROS
topics and replies inside the same socket.io handler. So the "sensor rate" is
one round trip: simulator -> bridge -> simulator. That is why every topic shows
the same rate, and why the rate is a property of the socket path, not of any
sensor.

## The experiments, in order

All on the ASUS TUF (i7-12700H, RTX 4070, Ubuntu 22.04), simulator HUD at
144 fps throughout. `tools/sim_rate_probe.py` is a stand-in bridge with no ROS
that answers every message at once and prints the round-trip statistics; it
gives the ceiling the real bridge can never exceed.

| # | arrangement | loop rate | what it told us |
|---|---|---|---|
| 1 | probe on loopback, no options | 19.4 Hz, median 50.8 ms | the cycle is ~7 rendered frames: NOT the simulator's frame |
| 2 | probe on a second laptop (Windows, i5) over a direct LAN cable | 63.6 Hz, 16 ms | the same-host socket path was the limit; 16 ms is the Windows 15.6 ms timer tick |
| 3 | simulator on that Windows laptop, probe here | 7.5 Hz | that laptop has no GPU and cannot draw the simulator; the two-machine route is moot for us |
| 4 | probe on loopback, `sudo ip link set lo mtu 1500` | 107 Hz | the mechanism is Nagle: with a 1500-byte MTU the 13 KB telemetry is full segments that Nagle sends at once |
| 5 | probe, Python-level TCP_QUICKACK re-armed after `emit` | 19.4 Hz | no effect: gevent-websocket writes the frame later from another greenlet, so the re-arm lands before the write |
| 6 | probe with the rebuilt `libnodelay.so` preloaded | 101.6 Hz, 8.8 ms | the fix, at the syscall level |
| 7 | real bridge, `tcp_nodelay:=false` vs `:=true` (native) | 18.6 -> 77.3 Hz | the fix on the actual stack |
| 8 | real bridge in the racer container, `--network=host`, shim preloaded | 84.8 Hz | Docker changes nothing; host networking is the host's loopback |
| 9 | container with `NODELAY_CAP_HZ=45` | 45.0 Hz | the loop can be pinned to the organizers' evaluation rate |

Experiments 1-3 were a detour: the organizers' advice does work as a workaround
(63.6 Hz over a cable), but only because moving the replier off the host
sidesteps the deadlock described below. Experiment 4 identified the mechanism
and experiments 5-6 found where the fix has to be applied.

## The cause, in two lines

The simulator only sends its next frame after it hears back from the bridge,
and on one machine each exchange was stuck for ~40 ms in a TCP quirk: Nagle's
algorithm holds the simulator's message waiting for an acknowledgment that
Linux deliberately delays. The bottleneck was never rendering or the CPU.

In more detail: on loopback the MTU is 65536, so the simulator's ~13 KB
telemetry is a single segment smaller than the MSS, and Nagle holds it until
the previous data is acknowledged. Our side delays that acknowledgment (Linux
delayed ACK, up to 40 ms). Every cycle pays that wait, plus a frame: ~50 ms,
i.e. 18-20 Hz on any machine whose frame is shorter than that. On a 1500-byte
path (experiment 4, or a real NIC, experiment 2) the message is full-size
segments, which Nagle sends immediately, so the wait disappears.

## The fix

`tools/libnodelay.so` (source `tools/nodelay.c`, `LD_PRELOAD`ed into the bridge
process so the devkit package itself stays unmodified) sets TCP_NODELAY on the
bridge's sockets and re-arms TCP_QUICKACK, which tells the kernel to
acknowledge immediately. The first version of the shim (2026-09-11) re-armed
only after reads and measured "no change", which led to the wrong conclusion
that the simulator's frame was the limit. The reason: Linux re-enters
delayed-ACK mode whenever a process sends right after it received
(`tcp_event_data_sent`, the "pingpong" heuristic), and a bridge reply is
exactly that, so the arm on the read was undone by the reply that followed it.
Re-arming after every send/write syscall as well is the fix. It has to be the
syscall: doing it from Python after `sio.emit()` does nothing (experiment 5).

Build it (any Ubuntu 22.04, host or container; it loads in the official image):

    gcc -shared -fPIC -O2 -o tools/libnodelay.so tools/nodelay.c -ldl

## Using it on this branch (Docker)

The image copies only `roboracer_stack`, so the shim is applied by bind mount
and environment. `NODELAY_CAP_HZ` is optional and pins the loop at that rate
(see below); leave it out for the full 77-85 Hz.

    docker run --name autodrive_roboracer_api --rm -it --network=host --ipc=host \
      -v "$PWD/tools/libnodelay.so:/home/libnodelay.so:ro" \
      -e LD_PRELOAD=/home/libnodelay.so -e NODELAY_CAP_HZ=45 \
      autodrive_racer:qualification-1

`-e LD_PRELOAD` reaches every process in the container, not only the bridge;
the shim only touches TCP sockets and DDS is UDP, so nothing else changes, but
it is a wider footprint than the per-node environment used on `multi-track`.
For a submission image the shim would have to be copied in by the Dockerfile
and enabled from `roboracer_stack/launch/bridge.launch.py`.

On `multi-track` the same thing is launch arguments: `race.launch.py
tcp_nodelay:=true loop_hz_cap:=45` (commits `a79b367`, `2911525`).

## Capping the loop

The organizers say the evaluation machine runs 40-50 Hz. `NODELAY_CAP_HZ=<hz>`
makes the shim sleep before each reply until the previous reply is 1/hz old,
scheduled against the previous target so jitter does not drift the average.
Since the simulator only speaks when replied to, the whole loop follows.
Measured: cap 45 -> 45.0 Hz, median 21.6 ms, p90 28 ms. The average is exact;
the interval spread is wider than a natural loop's, so treat it as "the right
average rate", not a replica of the evaluation machine's timing.

## Measuring

    source /opt/ros/humble/setup.bash
    export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 tools/topic_rates.py 15 /autodrive/roboracer_1/   # every topic, one table

`tools/topic_rates.py` subscribes to every topic raw with its publisher's QoS
and prints Hz, median and max interval per topic. It shows the one-tick
structure directly: 16 bridge topics at identical rates, the follower's
commands at `control_hz`. `ros2 topic hz` works too, one topic at a time, but
buffers badly through pipes.

`tools/sim_rate_probe.py` measures the simulator alone (stop the bridge, run
it, press Connect). On a second machine it needs `python-socketio==4.2.0` and
`python-engineio==3.13.0`, the devkit's versions: the simulator speaks
Socket.IO 2 and a 5.x server completes the TCP handshake and then never
raises `connect`. It binds `0.0.0.0` on purpose (`''` is IPv6-only on Windows).

## Cap sweep on this branch, 2026-09-12

Line `raceline_a7.0_rec_6.36.csv`, dev mode, `control_hz` 40, one run per cap,
logs in `logs/` (`racer_<stamp>.log` lap times, `run_cap<hz>.csv` per-tick).

| cap | laps | best | outcome | measured command delay |
|---|---|---|---|---|
| 20 (scripted) | 28 | 6.35 | clean | 0.150 s |
| 20 (manual) | 22 | 6.31 | clean for 21 laps, then a wall contact | |
| 40, before the image rebuild | 98 | 6.27 | clean | 0.079 s |
| 40, after | 9 | 6.42 | contact at lap 7 | |
| 45 | 32 | 6.33 | clean | 0.075 s |
| 60 | 0 | - | started 2.8 m mislocalized: car not on the spawn at seed time | 0.097 s |
| 70 | 37 | 6.47 | clean | 0.050 s |
| 80 | 6 | 6.47 | contact at lap 5 | 0.052 s |

What the logs say about the contacts: the two runs that failed from the start
(60 Hz, and a 126-lap run at 7.2 m off) were seeded with the car away from the
spawn, i.e. the reset-and-reconnect step between runs, not the cap. The three
mid-run contacts (20, 40, 80) came after clean starts with localization stable
at 0.2-0.4 m and normal lap times up to the contact: the car knew where it
was and clipped a wall. Lap times after the image rebuild (which only added
scipy, so the `log_csv:=` logger runs instead of crashing at startup) were
slower at the same cap, 6.47-6.71 s against 6.27-6.36 s before; the logger's
per-scan work at 80 Hz is the suspect, untested. The 50 Hz run was not made.
The follower itself is not rate-tuned: it runs on its own `control_hz` timer,
integrates with real `dt`, and measures the throttle-to-wheel delay
(`cmd_delay_auto`), which the last column shows shrinking with the cap; the
one per-sample constant is `pose_corr_max`.

## What this means for the submission

Nothing in the submission image is changed by this yet. `tcp_nodelay` is off
by default on `multi-track` and the shim is not in this branch's image. The
evaluation machine's rate is theirs to set; ours is now measurable and
settable, which is what tuning at 40-50 Hz needs. Validate the follower at the
chosen rate before baking the shim into an image the organizers run.
