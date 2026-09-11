
Yes, your workflow is right, with one gotcha

# 1. on the racer machine
docker load -i autodrive_racer_qualification-1.tar
docker run --name autodrive_roboracer_api --rm -it -p 4567:4567 autodrive_racer:qualification-1

# 2. second PowerShell window, same machine
docker cp rate_monitor.py autodrive_roboracer_api:/tmp/
docker exec -it autodrive_roboracer_api bash -lc "source /opt/ros/humble/setup.bash && python3 /tmp/rate_monitor.py --laps --csv /tmp/rate.csv"

# 3. after the session
docker cp autodrive_roboracer_api:/tmp/rate.csv .

The gotcha: docker cp copies from the machine running Docker, so rate_monitor.py has to be on the Windows box — copy it across next to the tar. It's one 400-line file with no dependencies; it deliberately imports nothing from roboracer_stack, so it runs from /tmp with only the ROS base sourced. Nothing to rebuild, and the container keeps running while you start and stop the monitor as often as you like.

Then on this machine: ./scripts/run.sh sim --headless <windows-ip>, or the GUI with that IP typed into Connect.

What --laps gives you

LAP   7    6.382 s   best   6.351   tick  44.9 Hz   worst gap   31 ms   stalls 0
LAP   8    7.104 s   best   6.351   tick  41.2 Hz   worst gap  287 ms   stalls 3

Lap time and the tick that produced it, on one line — so "lap 8 was 0.7 s slower" comes with the reason attached. Closing table repeats every lap, and calls out the best lap that had no stall in it, which is the number representing your setup rather than a quiet moment on the machine. CSV gains lap_count, last_lap_s, best_lap_s.

It's off by default, because the lap topics are on this repo's restricted list (common/restricted.py:50-56 — race telemetry), and it prints a warning when on, mirroring pure_pursuit's dev_lap_telemetry. That keeps the default tool's race-legal claim honest. The sim window shows lap times anyway, so an official run loses nothing without it.

Two bugs the testing caught

Off-by-one on lap numbering. The bridge publishes lap_count before last_lap_time in the same burst, so reading both on the count callback labels every lap with its predecessor's time. First fix (defer one report tick) then broke differently when messages arrived slowly — so the lap now waits for a last_lap_time that provably arrived after the lap closed, with a 2 s fallback so a missing topic degrades to -- rather than laps that never print.

Per-lap worst gap. The rolling window is 3 s and your laps are ~6.4 s, so a stall in the first half of a lap ages out before the lap closes — it's accumulated as the lap runs instead. Both are pinned by tests, since either would silently return wrong numbers rather than fail.

One note: your working tree now has these staged (git status shows A/M) — I didn't run git add, and I haven't committed anything.