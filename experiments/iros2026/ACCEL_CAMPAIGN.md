# Longitudinal acceleration campaign — 9.59 → 9.44 s

**Result: mean 9.436 s, median 9.440 s, best 9.250 s over 43 timed laps** ([S03](S03_along60_40laps/report.md)),
against a 9.59 s baseline. One hairpin-1 contact at lap 45, recovered in 5.1 s with no cascade.

The whole gain came from **one change to the raceline's velocity profile**. No controller
code was touched, the geometry is byte-for-byte the raced line, and corner speeds and
braking points are unchanged.

| run | a_long | line | timed laps | contacts | best | median | **mean** | \|e\| p90 |
|---|---|---|---|---|---|---|---|---|
| [A03](A03_hybrid_lqr_control/report.md) | 5.0 | `..._h7.0_a7.0b.csv` | 38 | 0 | 9.450 | 9.600 | **9.594** | 0.148 |
| [S02](S02_along55_40laps/report.md) | 5.5 | `..._h7.0_L55_a7.0b.csv` | 33 | 1 | 9.400 | 9.530 | **9.538** | 0.159 |
| [S03](S03_along60_40laps/report.md) | 6.0 | `..._h7.0_L60_a7.0b.csv` | 43 | 1 | **9.250** | **9.440** | **9.436** | 0.157 |

S03's **worst lap (9.560) beats A03's best (9.450)** — the distributions do not overlap.
24 of 43 timed laps ran under 9.45; A03 managed none.

---

## 1. The diagnosis

The starting question was why the car takes ~70 % of the straight to reach 8.5 m/s.
Three candidates, measured off A03's telemetry (`logs/hybrid_lqr_a03.csv`):

**Throttle authority — not the limit.** `throttle_max` is 1.0 and the straight peaks at
**0.353**. Two-thirds of the range was never used.

**The slip band — not the limit, and a dead end.** `slip_circle 0.12` saturates exactly
(`pp_slip` max 0.1200, binding on 80 % of straight ticks), so it looked like the culprit.
But the sim's tire curve (`TIRE_S_PEAK 0.15`, `TIRE_MU_PEAK 0.72`) is almost flat near its
peak: slip 0.12 already delivers **99.2 %** of peak grip, and going to 0.15 buys **+0.8 %**
force. Raising `slip_circle` would have been a wasted run.

| slip | mu | a = mu·g | vs 0.12 |
|---|---|---|---|
| 0.120 | 0.7142 | 7.007 | — |
| 0.135 | 0.7193 | 7.056 | +0.7 % |
| 0.150 | 0.7200 | 7.063 | +0.8 % |
| 0.180 | 0.6647 | 6.521 | −6.9 % |

**The raceline profile — this was the limit.** The profile plans a constant **4.95 m/s²
gross of drag** (the apparent decay 4.05 → 2.70 net along the straight is purely
`DRAG_LIN·v`). It reaches 8.5 m/s at s = 9.58 m of a 12.97 m straight — **74 % of it**,
which is exactly the reported symptom.

Meanwhile the car *demonstrably* accelerates harder than it is asked to:

| v bin (m/s) | achieved a, p90 | + drag = gross |
|---|---|---|
| 3–4 | 5.03 | 5.99 |
| 5–6 | 4.20 | 5.70 |
| 7–8 | 3.46 | 5.50 |

p90 gross 5.5–6.0 m/s², max 6.5, against a tire peak of 7.06 and a plan of 4.95.
`optimize_raceline.py` says as much in `a_long_profile`: the car "was measured braking and
accelerating at ~5.5 m/s² while tracking", and 5.0 was chosen as deliberately conservative.

## 2. What changed in the pipeline

Only the velocity profile, re-solved on the **same geometry** with `make_speed_variants.py`:

```bash
# baseline, reproduces raceline_tum_iqp_h7.0_a7.0b.csv byte-for-byte
python3 make_speed_variants.py raceline_tum_iqp_h7.0.csv \
    --ladder 7.0 --a-long 5.0 --a-brake 5.5 --v-max 8.5 --track iros2026

# the S03 line: a_long 5.0 -> 6.0, everything else identical
python3 make_speed_variants.py raceline_tum_iqp_h7.0_L60.csv \
    --ladder 7.0 --a-long 6.0 --a-brake 5.5 --v-max 8.5 --track iros2026
```

The baseline command was verified to reproduce the raced line **exactly** before anything
was changed — that check is what makes the ladder trustworthy.

> **Trap.** The output filename encodes only the *lateral* rung (`_a7.0b`), not `a_long`.
> Running the second command on the original stem silently **overwrites the raced line**.
> Copy the source to a stem that carries the a_long tag first (`_L60`), as above.

Verified invariant across every rung — geometry identical, and:

| | corner v min | peak a_lat | max brake | max accel gross |
|---|---|---|---|---|
| a_long 5.0 | 2.393 | 7.000 | −7.76 | 5.02 |
| a_long 6.0 | 2.393 | 7.000 | −7.77 | 6.02 |

Every speed change is **positive** — no part of the lap got slower.

## 3. What it bought, measured

The straight now reaches 8.0 m/s at **s = 7.1 m (54 %)** instead of 8.8 m (74 %):

| | straight v mean | straight target mean | reaches 8.0 m/s at |
|---|---|---|---|
| A03 (5.0) | 6.45 | 6.91 | s = 8.78 m |
| S02 (5.5) | 6.67 | 7.19 | s = 7.88 m |
| S03 (6.0) | 6.76 | 7.30 | s = 7.08 m |

By the brake point (s = 12) all three runs are at the *same* speed — the extra pace is
spent before braking starts, which is why corner entry is unaffected. The point-mass model
predicted −0.10 s for the 5.5 → 6.0 step; measured −0.102.

## 4. Negative results — do not repeat these

- **Raising `slip_circle` toward 0.15.** +0.8 % tire force. The band is already at the peak.
- **Feeding dead reckoning the tire observer (`pp_v_est`) instead of raw encoder.** It
  over-corrects and flips the sign: straight over-read goes +5.3 % → **−3.0 %**, whole-lap
  +0.71 → −0.64 phantom m/lap. Not a fix.
- **`distance_scale` for the drift.** It is a lap-wide constant, but the over-read is
  +5.3 % on the straight and +0.3 % in corners. Trimming it relocates error rather than
  removing it.
- **Hybrid LQR steering (A03) for lap time.** Mean 9.594 vs pure pursuit T03's 9.589 —
  neutral. Kept for clean-lap count, not speed. Steering is at a plateau; the remaining
  time was longitudinal.
- **`exit_guard`.** Historically −0.13 to −0.20 s/lap, and the contacts here are
  localization jumps, not corner-exit overspeed. It treats the wrong thing.

## 5. The open issue: along-track drift

Every hairpin-1 contact in this campaign (S01 lap 10, S02 lap 35, S03 lap 45) has the same
signature: the position estimate drifts **ahead** of truth along the straight, then
collapses during corner entry. On S01's crash lap it peaked at **+1.16 m** and fell to
+0.30 m through the entry — a 0.8 m backward jump mid-corner, which walks `e_lat` out from
+0.07 to +0.33 m until the steering saturates.

Root cause, measured against ground truth:

| | straight (s 1–12) | corners (s 14–43) | lap-wide |
|---|---|---|---|
| encoder over-read, A03 | **+5.34 %** | +0.34 % | +1.64 % |
| encoder over-read, S01 | **+5.46 %** | +0.65 % | +1.96 % |

`dead_reckoning.py` integrates **raw encoder arc length**, and the encoder reports the
*throttle command* — i.e. wheel speed — so it over-reads exactly when the car is
accelerating hard. That is why the error builds on the straight and washes out in corners,
and why harder acceleration feeds it (S03 loc p90 0.435 vs A03's 0.392).

**A correction that does work**, fitted on A03 alone and validated on S01 held out:

```
distance -= k * max(u_cmd - v_est, 0) * dt        k = 0.3707
```

| | straight over-read | phantom m/lap on straight |
|---|---|---|
| raw encoder | +5.34 % / +5.46 % | +2.32 / +2.37 |
| slip-corrected | **+0.19 % / +0.20 %** | **+0.08 / +0.09** |

It over-corrects the corners (−2.3 % / −1.8 %), so whole-lap error magnitude is about
unchanged — it *relocates* error out of the straight and into the corners. That is still
worth having, because AMCL has no longitudinal features to correct against on the straight
and fixes corner error within a metre. **Not yet tested on the car.** This is the next
thing to do, and it should come before the a_long 6.5 rung
(`raceline_tum_iqp_h7.0_L65_a7.0b.csv`, already generated and verified).

## 6. Reproducing a run

```bash
experiments/iros2026/scripts/run_s03_along60.sh      # inside the rr_bridge container
experiments/iros2026/scripts/teardown.sh             # stop + PROVE the stack is dead
python3 experiments/iros2026/scripts/compare_runs.py NAME=launch.log:telemetry.csv ...
```

Method notes that cost us time:

- **Reset and connect the simulator by hand.** Do not publish to
  `/autodrive/reset_command`; it looks like it works from ROS and is not reliable here.
- **Verify teardown by listing processes.** `pkill -f race.launch.py` silently failed
  twice and left *two* `pure_pursuit` nodes commanding the car at once, with a follow-up
  grep that printed nothing and looked like success. `teardown.sh` uses `docker restart`
  and counts survivors.
- **Do not judge a rung on a partial mean.** S02 read 9.589 at lap 14 and 9.572 at lap 16,
  and settled at 9.538 by lap 33. Wait for ~25 timed laps.
- **Watch the run to the end.** S03 was briefly recorded as "0 contacts in 42 laps"; it
  went on to contact at lap 45.
