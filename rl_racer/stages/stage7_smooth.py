"""Stage 7 - SMOOTH AND FAST. Same policy, same observation, same action scale.

Resumes stage 6's final.zip WITH its buffer. Stage 6 bought speed (22.2 ->
19.5 s/lap, 9.6 -> 12.4 laps/episode) and then stopped gaining for ~400k
steps at 2.26 m/s average. The reference for this track is the classical
raceline: 44.0 m, 9.94 s/lap, 4.42 m/s average, 8.0 m/s peak -- it never
exceeds 8 -- so there is about 2x left, not 4x.

Four changes, each against a measured cause of the plateau:

  horizon 13 -> 30 s   gamma is derived from it, and at 13 s the planning
                       horizon (267 steps) was SHORTER THAN A LAP (19.5 s,
                       401 steps): the critic could not see one lap ahead, so
                       w_lap's 200/lap_time arrived discounted to ~22% of face
                       value and lap TIME barely entered the value function.
                       Lap time is the only term that pays for being fast
                       rather than far. This roughly doubles the critic's
                       target scale, so expect train/critic_loss to spike
                       before it settles.
  w_ttc   1.0 -> 0.3   the brake term was the largest single cost in stage 6
                       (-362/episode against +217 of speed reward). It was
                       needed when the policy would not brake at all; now that
                       it brakes, the same term is what caps the speed, because
                       on a 44 m track the car is always within ~2 m of a wall
                       through the corners and the only way to pay less is to
                       go slower. ttc_forward also no longer charges for a
                       SATURATED beam: a reading of range_max means "nothing
                       within 10 m", but dividing it by speed made empty road
                       register as a 1.0 s time-to-collision above 8.3 m/s and
                       taxed exactly the straight where 8 m/s belongs.
  crash   50 -> 25     50 taught survival and survival is now in the policy and
                       in the buffer. The implicit cost of a crash is the
                       forfeited rest of the episode, which the longer horizon
                       RAISES from ~128 to ~339 in discounted value, so the
                       explicit penalty can come down without reopening the
                       sprint-crash equilibrium that -15 allowed.
  w_smooth2 0 -> 3.0   steering jitter. MEASURED at stage 6: |d_steer| averaged
                       0.164/step -- 4.9 deg every 48.6 ms, ~101 deg/s --
                       against a mean lock of only 8.2 deg, so the policy
                       reversed most of its steering every step, and
                       slip/yaw_res_rms rose 0.485 -> 0.779 with it. The
                       existing linear w_smooth charged that only 9% of
                       progress. The square charges a big reversal hard and a
                       small correction almost nothing, which is the shape that
                       removes jitter without making the hands slow: at the
                       current 0.164 the pair costs 0.16/step (29% of
                       progress), at a smooth 0.05 only 0.025 (4.6%).

Unchanged and deliberately so: the 117-dim observation, throttle scale 1.0,
w_progress 5.0 with v_ref off, w_speed 0.6, w_lap 200, w_grip 0.05, prox, and
every observation source. A true jerk term (second difference) would need
prev_prev_steer in the observation, which would unload every checkpoint.
"""
NAME = "stage7_smooth"
EXPECTED_OBS_DIM = 117
RESUME_FROM = "runs/stage6_v1/final.zip"
COMPETITION_LEGAL = True
# lr below stage 6's 2e-4: this is a fine-tune of a working policy under a
# changed discount, which is the least stable moment in the lineage.
# gradient_steps 1 -- MEASURED on the GTX 1080 Ti, 2 put diag/overhead_ms at
# 29-49 ms of a ~51 ms window and 3 blew it.
DEFAULTS = dict(timesteps=600_000, learning_rate=1.5e-4, warmup=10_000, gradient_steps=1)


def apply(cfg):
    from stages import stage6_push
    stage6_push.apply(cfg)                  # identical observation, scale, timing
    cfg.env.horizon_seconds = 30.0          # gamma is derived from this
    cfg.rew.w_ttc = 0.3
    cfg.rew.crash_penalty = 25.0
    cfg.rew.w_smooth2 = 3.0
    return cfg
