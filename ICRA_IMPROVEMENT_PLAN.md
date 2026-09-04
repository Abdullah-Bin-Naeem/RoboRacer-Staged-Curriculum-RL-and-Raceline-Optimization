# ICRA 2026 track — review, improvement plan, and how to split it three ways

Written 2026-09-05 after reading the repo and re-analysing `icra_run8_a70.csv`.
Current state: **12.05 s best / 12.09 s mean**, 8 clean laps, a7.0 line.
Reported first place: **11–12 s**.

---

## 1. Review — what this stack is, and what is already solved

This is a mature, unusually well-evidenced codebase. Two independent stacks
share the workspace:

- **Classical** (`devkit_ws/` + `raceline/`): SLAM map → min-curvature raceline
  → AMCL localization → pure pursuit. **6.50 s on Porto** against a 6.46 s
  record, race-legal.
- **RL** (`rl_racer/`): SAC on LiDAR + encoders, 7.44 s on Porto, legal by
  construction.

The parts worth knowing before touching anything:

**The one fact that shapes everything.** The simulator spins each wheel to
`25.25 × throttle` m/s within a millisecond. So the *encoders report the
throttle command, not the car*. The follower therefore runs the sim's own tire
curve and drag on the measured wheel speed to recover true speed
(`speed_source: tire`, p90 error 0.15–0.22 m/s), and commands throttle as a
**wheel speed inside a slip band** placed one measured round trip (175 ms)
ahead. Anyone who closes a speed loop on raw encoders is closing it around the
wheel and leaving the car open-loop.

**The loop rate is a fixed ~18–20 Hz cadence, not compute.** Pinning the sim to
one core did not change it. So the evaluation machine will be near this too —
and everything latency-tuned is tuned at a 175 ms round trip (three sim frames).

**Track-scoping is already correct.** `racer_common.frames.TRACKS` is the single
registry resolving a track to map, raceline, spawn and margin zones. The car
model and the whole controller transfer between tracks unchanged; what does
*not* transfer is the spawn, the safe grip rung, and the margin zones — which
are placed **from measured wall clearance**, never guessed.

**Legality is enforced structurally, not by comment.** `restricted.seed()` vs
`restricted.warn()`, everything continuous-and-restricted quarantined in
`instruments.launch.py`, which `mode:=race` omits wholesale. This is stronger
than most teams will have.

### Where the ICRA ladder got to

| run | line | best / mean | note |
|---|---|---|---|
| icra 2 | a4.0 | 14.70 / 14.75 | controller transferred unchanged |
| icra 4 | a5.0 | 13.45 / 13.47 | |
| icra 6 | a6.0 + 2 zones | 12.55 / 12.58 | |
| icra 7 | a6.5 | 12.20 / 12.33 | submission rung |
| **icra 8** | **a7.0** | **12.05 / 12.09** | tire limit; zones deepened |

⚠️ `frames.TRACKS['icra2026']['raceline']` still says **`raceline_a4.0.csv`**
while the ladder has been climbed to 7.0. Anyone launching without an explicit
`path_csv:=` gets the 14.7 s line. Fix that first — it is a one-line change and
a very easy way to lose a race.

---

## 2. Where the 12.05 s actually goes — measured, not assumed

Re-analysis of `icra_run8_a70.csv` (2230 valid ticks, 8 laps, track 54.1 m):

```
best lap                     12.05 s
what the profile asked for   11.53 s
lost to the profile           0.52 s/lap      <- the whole budget
tracking |e_lat|              mean 0.065  p95 0.158  max 0.386 m   (good)
measured round trip           0.176 s
ticks >0.3 m/s below target   49.3 %
```

**Split of the 0.52 s:**

| where | lost | character |
|---|---|---|
| corners (κ > 0.25) | 0.24 s | tire at its lateral limit — the known Porto ceiling |
| **straights (κ < 0.25)** | **0.28 s** | **acceleration-limited, NOT grip** |

This is the important difference from Porto. Porto is 27 m and corner-dominated,
so the tire limit was the whole story. **ICRA is 54 m with real straights, and
over half the loss is now longitudinal.**

### The specific cause

On straights where the car is below target and accelerating:

```
achieved a_long   p50 3.10   p90 4.81   p95 5.15 m/s²
profile plans     5.0 m/s²
slip while there  mean 0.061, p90 0.080
ticks at/over the slip_accel = 0.08 cap        52.5 %
```

**The slip band is saturating on half the ticks.** `slip_accel: 0.08` is
~5.5 m/s² on the fitted curve; the tire's forward peak is at slip **0.15**
(~7.06 m/s²). The band was set on Porto, where straights are short and the cap
almost never bound. On ICRA it binds constantly and is directly buying the
0.28 s.

Worst single sectors (all low curvature = straights):

| s (m) | v actual | v target | deficit | κ |
|---|---|---|---|---|
| 6.8 | 5.08 | 6.71 | **1.63** | 0.098 |
| 9.0 | 6.05 | 7.00 | 0.95 | 0.057 |
| 47.3 | 4.84 | 5.72 | 0.87 | 0.079 |

---

## 3. Plan forward

Ordered by expected value per unit of risk. Everything is measured on the car
before it is kept — the repo's existing discipline, unchanged.

### P0 — free, do today

1. **Set the ICRA default raceline to the rung that is actually validated**
   (`raceline_a6.5.csv` for submission, 7.0 as the fast line), matching Porto's
   pattern. One line in `frames.py`.
2. **Re-run the a7.0 ladder rung with the run logged and `analyze_run.py`'d**
   so P1 has a clean before/after baseline on the same machine and tick rate.

### P1 — the 0.28 s of longitudinal loss (highest value, lowest risk)

The lever is `slip_accel`, and it is a **one-parameter sweep** with an existing
measurement harness. It is bounded above by the tire peak at 0.15 and below by
what is there now.

- Sweep `slip_accel` 0.08 → 0.10 → 0.12 → 0.15, everything else fixed, on the
  a6.5 line first (so a mistake is not also at the grip limit).
- Read each with `analyze_run.py`: lap time, `pp_slip` saturation fraction, and
  the encoder-vs-tire estimator error (the band existing at ±0.08 is *also* what
  bounds the observer error — raising it widens that error, which is the real
  cost and must be measured, not assumed).
- Expect to recover a meaningful share of 0.28 s. **Do not raise `slip_brake`
  with it** — braking into a corner at higher slip is how you lose the tire.
  Separate parameters, separate sweeps.

⚠️ The honest risk: `VEHICLE_MODEL.md` §3.2 notes demands *between* the
asymptote and the peak are what makes the car drift. Above the asymptote the
band is trading stability for force, so this needs the wall-clearance check
every rung got, not just a lap time.

### P2 — the corner 0.24 s

This is the Porto ceiling and it did not move there. Two untried levers:

- **Raise the profile's longitudinal limit on straights only.** The profile
  plans one longitudinal limit everywhere; the measured p95 achieved is 5.15
  with the cap on. If P1 lands, re-profile with the measured value.
- **Combined-slip exponent.** `VEHICLE_MODEL.md` §6 lists this as unmeasured:
  the friction ellipse currently assumes exponent 2. Unity documents separate
  per-axis curves and says nothing about coupling — if the axes are effectively
  independent, the ellipse is conservative and corner-exit acceleration is
  being left on the table. **Measure it before assuming it** (steady circles at
  fixed steering while adding throttle).

### P3 — the structural lever nobody has tried

`FINDINGS.md` §7 names it: **a predictive controller that steers on the vehicle
model run forward by the measured 175 ms delay**, instead of pure pursuit plus
anticipation. On Porto it was judged not worth the weave risk once the record
was beaten. On ICRA the delay costs more (longer lap, more time at speed), and
the tracking headroom is there (p95 0.158 m). This is the only thing that
attacks the 0.176 s round trip itself rather than working around it.

Real rewrite, real risk. Only worth starting if P1/P2 stall above 11.5 s.

### P4 — worth knowing, probably not worth doing now

- **RL as a fallback, not a competitor.** Stage 3 is 7.44 s on Porto vs the
  classical 6.50 s. It has never been run on ICRA. Its value is that it needs
  no map and no localization, so it is the insurance policy if the ICRA map or
  AMCL misbehaves on the evaluation machine — not a path to a faster lap.
- **Minimum-time optimization** gained nothing on Porto; on a 54 m track with
  long straights it might. Cheap to test offline — it is a flag on
  `optimize_raceline.py`, no car time.

---

## 4. Dividing this between three people

The instinct is right: **this system does not decompose by parameter.** Nearly
every constant is coupled to the 175 ms round trip and to the tire model, and
tuning two of them simultaneously on separate branches produces two runs that
cannot be compared and do not compose.

So do not divide by parameter. Divide by **the three things that genuinely are
independent**, and serialise the one thing that is not.

### The real constraint

There is **one car**. Every lap time is measured on one machine at one tick
rate, and the tick rate itself drifts within a session (17.3 → 12.8 Hz was
observed). Two people tuning on two laptops produce numbers that cannot be
compared to each other at all.

**Therefore: car time is a single serialized resource with one owner at a time.**
Everything else is offline work that produces artifacts *queued* for the car.

### The split

| | Owner A — **Car & control loop** | Owner B — **Track assets & offline optimization** | Owner C — **Measurement, model & fallback** |
|---|---|---|---|
| **Owns** | `racer_control/`, `pure_pursuit.yaml`, the tuning ladder | `raceline/`, `racer_mapping/`, `frames.TRACKS` rows | `analyze_run.py`, `VEHICLE_MODEL.md`, `rl_racer/` |
| **Does** | P0.2, P1 (`slip_accel` sweep), P2 rung climbs | Regenerates ladders + margin zones from A's logs; min-time line (P4); any new track | Measures the unmeasured constants (§6): combined slip, lateral curve, rising slope. Keeps RL warm as fallback |
| **Needs the car?** | **Yes — exclusively** | No | Yes, but only in short scripted blocks (steady circles, constant-throttle steps) |
| **Output** | A measured run + config diff | A raceline CSV + margin zones | A number in `VEHICLE_MODEL.md`, with its provenance |

### The contracts that make this compose

This is the part that actually matters. Three rules:

1. **One variable per run.** The repo already works this way (the ladder, the
   zone regenerations). Keep it. A run that changed two things is a run that
   taught you nothing and has to be repeated.

2. **Artifacts cross boundaries, not tuning.** B never tunes the follower; B
   ships a **CSV**. C never tunes anything; C ships a **measured constant with
   its provenance**. Only A changes controller behaviour, and only against a
   logged run. This is why the existing layout works — `optimize_raceline.py`
   emits CSVs and `pure_pursuit` consumes them, with `s,x,y,psi,kappa,w_r,w_l,v`
   as the contract.

3. **Every claim carries its tick rate.** Lap times are only comparable at the
   same measured round trip. `analyze_run.py` already reports it; make it
   mandatory in any result anyone reports. This alone prevents most of the
   "it was faster on my machine" failures.

### Concrete first week

- **A**: fix the default line (P0.1), re-baseline a7.0 (P0.2), then run the
  `slip_accel` ladder (P1). This is the critical path — it is where the 0.28 s
  is, and only A can do it.
- **B**: regenerate the ICRA ladder against whatever margin zones A's new logs
  justify; in parallel, produce a min-time line for the 54 m layout and hand it
  to A's queue. Zero car time needed.
- **C**: measure combined slip and the lateral curve (short scripted car
  sessions, bookable around A). These unblock P2 and are currently the largest
  *unmeasured* assumptions in the whole model. Separately, run stage-3 RL on
  ICRA once to know whether the fallback exists.

### What NOT to do

- Do not let two people hold controller parameters. It is the one genuinely
  indivisible thing here.
- Do not tune on Porto and port to ICRA. The controller transfers; the grip
  rung and the margin zones **do not**, and reusing Porto's zones pushes the
  line toward a wall rather than away from one.
- Do not compare a graphics run to a headless run. 17.6 vs 20.3 Hz is worth
  more than most of the changes being tested.

---

## 5. Honest assessment of the target

12.05 s now; the profile the car is already following would give 11.53 s if it
were tracked perfectly. So **~11.5 s is reachable without any new physics** —
it is the acceleration saturation plus the corner tire limit, and P1 attacks the
larger half with a one-parameter sweep.

Below ~11.4 s needs either the combined-slip measurement to come back favourable
(P2) or the predictive controller (P3). Neither is guaranteed.

The reported 11–12 s first place is therefore beatable, but the margin is thin
and the biggest risk is not lap time at all — it is the evaluation machine's
tick rate, which the derate logic already handles and which should be left
alone.
