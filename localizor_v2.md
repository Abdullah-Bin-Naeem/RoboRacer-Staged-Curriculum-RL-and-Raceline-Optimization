No rebuild needed — run.sh bind-mounts your host raceline/ over the image's copy, so every line in raceline/iros2026/ is already raceable. Only TAG=lv matters (that's the image with v2 in it).

And here's the number that says it's worth trying. FINDINGS §11 records that tb10 is solved for the 0.09 m worst-case error a true-pose car has, and that AMCL's 0.13 m put it into the right wall:

┌────────────────────┬───────────────────────┬───────┬───────────────────────────────────┐
│                    │ cross-track error p90 │  p99  │                max                │
├────────────────────┼───────────────────────┼───────┼───────────────────────────────────┤
│ AMCL (lv_shadow_1) │ 0.043                 │ 0.074 │ 0.122 m ← above the design margin │
├────────────────────┼───────────────────────┼───────┼───────────────────────────────────┤
│ v2 (lv_race_2)     │ 0.030                 │ 0.048 │ 0.077 m ← below it                │
└────────────────────┴───────────────────────┴───────┴───────────────────────────────────┘

v2's worst cross-track error over 9 laps is inside the margin that line was designed for. That's the first time that's been true.

Run the fast line on v2

TAG=lv LOCALIZER=v2 SCAN_DUMP=1 ./scripts/run.sh race lv_fast_1 \
  path_csv:=/root/Documents/roboracer/raceline/iros2026/rl_mt_tb10_lat875_b55_L70.csv \
  steer_a_lat_max:=8.0

steer_a_lat_max:=8.0 is not optional — that line needs it (§11 step 5: the follower's own lateral cap was clipping the hairpin apexes and starving the throttle). The path must be the container path, as in your very first message.

If it contacts, the intermediate rung is rl_mt_amcl_lat80_hp725_b55_L70.csv (hairpins back at 7.25, predicts 8.55) with warmup_dist_m:=28.

Full command reference

Common knobs: TAG=lv (image), LOCALIZER=amcl|v2|slam, SCAN_DUMP=1 (writes runs_docker/NAME.npz for offline replay), V_MAX=, HZ_CAP=, RVIZ=true, AMCL_PARAMS=.

# ── simulator (leave running) ────────────────────────────────────────────
./scripts/run.sh sim                     # GUI: press Connect
./scripts/run.sh sim --headless          # auto-connects to 127.0.0.1:4567

# ── GROUND TRUTH pose — diagnostic, NOT race-legal ───────────────────────
TAG=lv ./scripts/run.sh truth gt_1                       # tb10 line + steer_a_lat_max 8.0 built in
TAG=lv ./scripts/run.sh truth gt_2 \                     # ... on any other line
  path_csv:=/root/Documents/roboracer/raceline/iros2026/<LINE>.csv

# ── AMCL ─────────────────────────────────────────────────────────────────
TAG=lv LOCALIZER=amcl ./scripts/run.sh race amcl_1
TAG=lv LOCALIZER=amcl ./scripts/run.sh race amcl_2 \
  path_csv:=/root/Documents/roboracer/raceline/iros2026/<LINE>.csv steer_a_lat_max:=8.0

# ── v2 ───────────────────────────────────────────────────────────────────
TAG=lv LOCALIZER=v2 ./scripts/run.sh race v2_1
TAG=lv LOCALIZER=v2 SCAN_DUMP=1 ./scripts/run.sh race v2_2 \
  path_csv:=/root/Documents/roboracer/raceline/iros2026/<LINE>.csv steer_a_lat_max:=8.0

# ── A/B: AMCL drives, v2 logged beside it on identical scans ─────────────
TAG=lv SCAN_DUMP=1 ./scripts/run.sh shadow shadow_1

# ── anything else: raw launch args ───────────────────────────────────────
TAG=lv ./scripts/run.sh racer track:=iros2026 localizer:=v2 tcp_nodelay:=true \
  loop_hz_cap:=45 path_csv:=... log_csv:=/root/Documents/roboracer/runs/x.csv

Analysing a run — two gotchas that bit me

.venv-plot/bin/python raceline/analyze_run.py runs_docker/NAME.csv \
  --path raceline/iros2026/<THE LINE YOU RACED>.csv

1. Always pass --path. Auto-detection picked raceline_a4.0.csv for lv_race_2 and reported "laps: none completed" — your 9 laps at 8.90–8.95 s only appeared once I named the real line.
2. Use .venv-plot/bin/python, not python3 — the system python has no numpy.

For offline tuning (needs SCAN_DUMP=1), no simulator required:

.venv-plot/bin/python devkit_ws/src/racer_localization/tools/replay_localization_v2.py \
  runs_docker/NAME.npz --set rate_m_s=1.5,1.5,1.5,1.5

Send me the analyze_run output and I'll read it. What I'll be watching on the fast line: contacts, and whether cross-track max stays near 0.077 m — if it climbs toward 0.09 the line is at its limit rather than v2 failing.