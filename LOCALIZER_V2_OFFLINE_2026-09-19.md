# localizer v2 — offline investigation, 2026-09-19

Ten recorded runs replayed through the same `V2Filter` the node runs
(`tools/replay_localization_v2.py`). Nothing was run in the simulator and
nothing is committed. `git diff` is four files; the working tree is the change.

## The headline

One change ships: **the cross axis gets its own, slower rate limit.**

|  | shipped | new |
|---|---|---|
| `err_cross` p90 | 0.032 m | **0.025 m** (better on 8 of 9 clean runs, −18 %) |
| cross per-sample step p90 | 16.9 mm | **6.6 mm** |
| cross per-sample step max | 35–51 mm | **8–11 mm** (steady state) |
| `err_along` | — | **unchanged to 3 decimals on every run** |

Cross is the axis the 0.09–0.13 m wall margin and the follower's steering
actually feel, and it was already at its geometric floor — so it can afford to
be slow. Along keeps 1.0 m/s because it still has to dump the straight's drift
before the braking point.

Two smaller items: a real bug in `est_scale`, and one measured-but-rejected idea.

## What was actually wrong

**1. The rate limiter forced one rate on both axes.** It scaled the whole
correction *vector* by `step_max/|gap|`, and the along gap (0.1–0.3 m off the
blind straight) always decided the step. Splitting them per axis is what makes a
separate cross rate possible. Note the honest detail: decoupling *alone*, at
equal rates, is slightly **worse** on cross (0.030 → 0.032) — the shared limit
was incidentally smoothing it. The win is the slower rate, not the decoupling.

**2. `est_scale:=false` disabled slip compensation entirely**, rather than
freezing it at the measured value — `scale_init` had no effect at all in that
mode (2.77 % and 3.50 % scored identically, both equal to no compensation:
along p90 0.333 vs 0.173 with it). This matters because that is the fallback I
told you restores validated behaviour. It does not; it was strictly worse.
Now false freezes-and-applies, which offline matches the estimator (0.173 vs
0.174) without its saturation dynamics — a genuinely useful safe mode.

**3. Recovery needed an escape hatch.** A flat 0.20 m/s cross rate made the two
runs containing wall resets *worse* (lv_L750 cross p90 0.035 → 0.047,
lv_fast_1 cross max 0.185 → 0.240): tens of centimetres at 0.20 m/s takes a
second. Above a 0.06 m cross gap the along rate applies instead, and both runs
then come out better than the shipped limiter (0.030 and 0.184).

## Measured but NOT shipped: slip is acceleration-dependent

Throttle commands a *wheel* speed, so the encoder reads the wheel; the wheel
leads the ground by the slip ratio, and slip ratio is what makes tyre force, so
it tracks longitudinal acceleration. Over 13 059 noise-immune chords on three
runs:

    k(%) = 0.961*a + 2.766      R2 0.486
    braking a<-3: -1.1 %    steady |a|<1: +2.1 %    accel a>+3: +6.6 %

consistent to ±0.5 points across all three runs. This is the reason the single
scale state always runs to whatever bound it is given: along is observable at
the *corners*, where the car brakes and k is ~−1 %, and the error is spent on
the *straight*.

It ships at `k_accel: 0.0` anyway, because it is **not a net win**: over a
closed lap the correction integrates to k1·Δ(v²)/2, which cancels, so it
redistributes error rather than removing drift — and on this track the blind
straight is the part the car is *not* accelerating on (a ≈ −0.2 at s 34–42; the
acceleration is at s 26–32 where the scan still sees along). Offline it helps
gt_dist / lv_slow_dist / lv_L750_warm and hurts lv_race_2 / lv_shadow_1. The
code and the number are kept so a track whose blind section *is* the
accelerating one can switch it on.

## Hypotheses I tested and had to discard

Each of these was a plausible story that the data killed:

- **A systematic yaw error absorbed as position bias.** Refuted: `yaw_hint`
  is −0.006° ± 0.017° over 4 190 scans. The zero-yaw assumption is excellent.
- **The rate limiter lags and costs accuracy.** `pending_m` exceeds 10 mm on
  35 % of samples, so it looked guilty — but raising the limit to 100 m/s does
  not improve along p90 (0.174 → 0.171) and wrecks smoothness (48 → 291 mm).
  It is a good low-pass, not a lag.
- **The scale bound is set too low.** I measured the over-read at +4.6–6.6 %
  and nearly changed the bound. That was wrong: raw path length is inflated by
  per-sample noise. Smoothed and chord-based estimates converge on **1.022–1.030**,
  which confirms the documented 2.0–3.4 % and the existing 3.0 % cap.
- **The weakly-observable band (info 5–20) injects bad along corrections.**
  Refuted: raising `blind_along_info` from 5 to 80 makes along p90 worse
  monotonically (0.173 → 0.280). Those corrections are informative; the guard
  at 5 is right.
- **Cross error spikes at the corner because the limiter starves it.**
  Refuted, as above — the spike (0.001–0.020 m through the straight, 0.098 m at
  s 42–44) is the straight's accumulated *along* drift rotating into the cross
  axis as the car turns in. Only less drift fixes that.

## The thing worth knowing about the remaining error

Matching every scan **from the true pose** gives a mean along correction of
−0.018 m, p90 0.055, residual 0.008 m at inlier fraction 1.000. The map, the
extrinsic and the timestamps are mutually consistent — the scans contain the
pose to 2–5 cm where the geometry allows it at all. Binned by observability:

| live along_info | n | \|err_along\| mean | p90 |
|---|---|---|---|
| 0–5 | 801 | 0.154 | 0.278 |
| 5–10 | 168 | 0.098 | 0.249 |
| 10–20 | 587 | 0.063 | 0.137 |
| 20–40 | 1072 | 0.050 | 0.103 |
| 40–80 | 871 | 0.042 | 0.088 |
| 80+ | 691 | 0.020 | 0.043 |

The along error is *entirely* an observability story, and s 28–44 (along_info
17.8 → 0.2) is where it lives. There is no algorithmic fix there; that was the
original finding and it survives.

## Status

Not run on the car. The offline gate passes on every clean run. The change is
one parameter's worth of behaviour (`rate_cross_m_s`), it costs nothing on
along, and `rate_cross_m_s: [1,1,1,1]` reverts it exactly.
