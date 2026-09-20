"""Stage 9 - LIMIT. Grip instead of wheelspin, and a racing line instead of the centre.

Resumes stage 8's final.zip WITH its buffer. Stage 8 did what it was for --
|d_throttle| 0.165 -> 0.126, full brake 24.7% -> 21.6%, brake bursts 83 -> 60
per 700 steps, lap 16.4 -> 15.6 s -- but the car still averages 3.1 m/s against
a classical setup measured at 8 s/lap and 9.5 m/s peak.

Measured on stage 8's DETERMINISTIC policy over 1400 steps, the two things
holding it there:

  1. IT NEVER GRIPS. Mean |S| = 0.924, with 75.6% of steps past the tire's
     asymptote (|S| >= 0.25) and only 6.3% in the peak band [0.10, 0.20]. It
     spins up under throttle and locks under brake, so it spends three quarters
     of the lap at mu = 0.464 against the 0.72 available -- 64% of the grip it
     has. That is the speed ceiling, and it caps cornering and braking as much
     as acceleration.

     w_grip could not fix this and never could: mu(|S|) is FLAT at the
     asymptote for every |S| >= 0.25, so mu/mu_peak is a constant 0.644 across
     the whole region the policy occupies and the term has ZERO gradient there.
     Tripling it (0.05 -> 0.15) duly left slip/frac_peak unmoved at 0.064 while
     paying out 477/episode for nothing. `w_slip` is the fix: a linear penalty
     on max(0, |S| - 0.15), which has a constant non-zero gradient everywhere
     above the peak and so actually pulls slip down. It is symmetric because
     peak mu is at |S| = 0.15 for braking too, which makes it threshold braking
     and threshold acceleration -- the limit of the car, asked for directly.

  2. IT WILL NOT LEAVE THE CENTRELINE. Mean lateral offset 0.18 m of a 0.77 m
     half-width: it uses 23% of the room it has, in a corridor averaging 1.55 m
     wide. A racing line is out-in-out; `w_center` pays it to be exactly the
     opposite, and `prox` with safe_dist 0.5 m starts charging before the car
     can reach an apex at all (measured min_range mean 0.55 m -- it is riding
     that threshold constantly).

  w_slip     0 -> 0.25    at the measured |S| 0.924 that is 0.194/step, ~28% of
                          progress; at the peak 0.15 it is exactly 0.
  w_center   0.15 -> 0.05 room to take a line. NOT to 0: EXPERIMENTS.md records
                          the crash rate rising 20x at 0 in the earlier
                          lineage. It is safer now than it was then -- that
                          lineage had no ttc term and a smaller crash penalty --
                          but 0 is still not a step to take blind.
  safe_dist  0.5 -> 0.35  so clipping an apex is not automatically taxed. The
                          car is ~0.3 m wide, so 0.35 m to its centre is still
                          real clearance.

Unchanged: the 117-dim observation and every normaliser (raising obs.slip_max
would un-blind slot 116 above |S| = 0.5, but it would also invalidate every
observation already in the replay buffer -- and the policy can derive S from
slots 111 and 115 anyway), throttle scale 1.0, 30 s horizon, w_progress 5.0,
w_speed 0.6, w_lap 200, crash 25, both smoothness terms.
"""
NAME = "stage9_limit"
EXPECTED_OBS_DIM = 117
RESUME_FROM = "runs/stage8_v1/final.zip"
COMPETITION_LEGAL = True
DEFAULTS = dict(timesteps=900_000, learning_rate=1.5e-4, warmup=10_000, gradient_steps=1)


def apply(cfg):
    from stages import stage8_flow
    stage8_flow.apply(cfg)                  # identical observation, scale, timing
    cfg.rew.w_slip = 0.25
    cfg.rew.w_center = 0.05
    cfg.rew.safe_dist = 0.35
    return cfg
