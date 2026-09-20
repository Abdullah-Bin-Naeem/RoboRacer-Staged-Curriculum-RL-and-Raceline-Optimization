"""Stage 8 - FLOW. Stop pumping the brake. Same policy, observation, action scale.

Resumes stage 7's final.zip WITH its buffer. Stage 7 delivered what it was for
(laps/episode 11.4 -> 14.9, lap 21.0 -> 16.4 s, yaw_res_rms 0.779 -> 0.607) and
its steering fix worked: replaying the DETERMINISTIC policy, |d_steer| fell
0.094 -> 0.055 per step, -41%. But the car still averages 2.9 m/s against a
classical setup measured at 8 s/lap and 9.5 m/s peak on this track.

The cause is not the reward weights and not exploration, both of which were
tested and ruled out:

  - Replaying the policy with its throttle multiplied by 1.5, 2.0 and 2.5
    barely moved the speed (v_mean 2.98 -> 3.18). Scaling a zero is still a
    zero, so throttle MAGNITUDE was never the limit.
  - It already commands throttle >0.3 on 27% of steps and >0.9 on 2.9%, which
    sustained is 7+ m/s (throttle 0.40 -> 9.6 m/s terminal). So the policy is
    not shy of the throttle either.

What it actually does is CHATTER: |d_throttle| 0.165 per step, 25% of steps at
full brake, in 83 bursts of ~2.1 steps (104 ms) per 700 steps -- roughly 6x
more often than a lap's corners require. Throttle 0 is brake LOCK in this sim,
so every burst throws away ~0.47 m/s at 4.55 m/s^2. The throttle histogram is
bimodal: median 0.130 (coasting) against a p90 of 0.613. It surges and slams.

Nothing in the reward charged for that -- `w_smooth` penalises steering only.
Stage 8 closes that gap and removes the incentives that were paying for the
braking:

  w_thr_smooth 0 -> 0.6   the missing term, |throttle_t - throttle_{t-1}| in
                          throttle units. At the measured 0.165/step it costs
                          0.099/step, ~16% of progress; at a smooth 0.05 only
                          0.030. LINEAR, deliberately, where the steering term
                          is quadratic: here the fault is the FREQUENCY of
                          transitions and linear total variation measures
                          exactly that, while a square would charge one
                          decisive 0.6 brake application more than six 0.1
                          chatters -- backwards, since braking hard once per
                          corner is what a fast lap looks like.
  w_grip   0.05 -> 0.15   pays for sitting AT the tire's peak slip, which is
                          what sustained acceleration looks like and the direct
                          opposite of pump-and-slam. slip/frac_peak is 0.064:
                          the policy uses the tire properly 6% of the time, so
                          0.05 was plainly too weak to matter.
  w_ttc    0.3  -> 0.15   both of these spike locally near a wall (ttc can
  w_prox   0.5  -> 0.3    reach 0.24/step in a corner, ~39% of progress) and
                          the one thing we now know for certain is that this
                          policy over-brakes. Episode-average cost is small
                          (0.026 and 0.018/step), so these are secondary --
                          w_thr_smooth is the change being tested.

Unchanged: the 117-dim observation, throttle scale 1.0, 30 s horizon, w_progress
5.0 with v_ref off, w_speed 0.6, w_lap 200, crash 25, w_smooth 0.3 + 3.0 square.
"""
NAME = "stage8_flow"
EXPECTED_OBS_DIM = 117
RESUME_FROM = "runs/stage7_v1/final.zip"
COMPETITION_LEGAL = True
DEFAULTS = dict(timesteps=900_000, learning_rate=1.5e-4, warmup=10_000, gradient_steps=1)


def apply(cfg):
    from stages import stage7_smooth
    stage7_smooth.apply(cfg)                # identical observation, scale, timing
    cfg.rew.w_thr_smooth = 0.6
    cfg.rew.w_grip = 0.15
    cfg.rew.w_ttc = 0.15
    cfg.rew.w_prox = 0.3
    return cfg
