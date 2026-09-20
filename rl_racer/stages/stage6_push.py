"""Stage 6 - PUSH. Same policy, same observation, more speed pressure.

Resumes stage 5's final.zip WITH its replay buffer (final_replay_buffer.pkl is
saved next to it). Only the reward changes:

  w_speed 0.2 -> 0.6     the direct counterweight to crash-aversion; 0.6 is the
                         value that produced the best earlier policy.
  w_lap   0   -> 200     paid per completed lap as 200 / lap_time (sim timer),
                         so it grows as laps get quicker -- the gradient that
                         progress lacks near the limit. ~10% of reward.
  w_grip  0   -> 0.05    mu(|S|)/mu_peak per step: pays for being AT the tire's
                         limit, accelerating or braking; ~3% of reward.
  start point -> line    stage 5's corner drill stops: lap time is paid per
                         COMPLETED lap, which needs whole laps from the line.
  v_ref   3.0 -> 0       stage 5's reward-side speed cap comes off: progress
                         pays at any speed. No action is re-labelled by this,
                         only the incentive moves. The ttc term and the crash
                         penalty stay: they are what makes the speed survivable.

The buffer's stored rewards are under stage 5's weights (~10-20% low). That is
a bias in the critic's targets that ages out over ~150k steps, not a dynamics
error; clearing the buffer instead is the empty-buffer collapse (v4, stage 3
runs 1-2). Load it.
"""
NAME = "stage6_push"
EXPECTED_OBS_DIM = 117
RESUME_FROM = "runs/stage5_fresh/final.zip"
COMPETITION_LEGAL = True
DEFAULTS = dict(timesteps=300_000, learning_rate=2e-4, warmup=10_000, gradient_steps=3)


def apply(cfg):
    from stages import stage5_fresh
    stage5_fresh.apply(cfg)                 # identical observation, scale, timing
    cfg.rew.w_speed = 0.6
    cfg.rew.w_lap = 200.0
    cfg.rew.w_grip = 0.05
    cfg.rew.v_ref = 0.0                     # cap off; ttc + crash penalty stay
    # Stage 5's corner-first curriculum comes off: w_lap pays per COMPLETED lap
    # (w_lap / lap_time), so this stage needs whole laps from the start line,
    # not repeated attempts at one corner.
    cfg.env.crash_restart = False
    cfg.env.spawn_drive = False              # full laps from the line, not corner drills
    return cfg
