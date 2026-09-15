"""Stage 5 - FRESH, from scratch. Learn to drive the competition track.

  observation  110 beams at +/-110 deg (2.0 deg/beam), wheel speed u, IMU yaw
               rate, previous action, and three race-legal slip slots: observer
               car speed v_est, longitudinal slip S, yaw-rate residual. Encoder
               speed is a 3-tick window with a counter-discontinuity guard (the
               sim's ResetManager restores the encoders on every reset).
  throttle     scale 1.0, FIXED: the full simulator range, no curriculum, so a
               learned action is never re-labelled. The peak-grip throttle is
               ~1.15 x v/25.25; beyond it the extra is wheelspin, which the
               slip slot shows and stage 6's grip term charges for. RL finds
               its own speed; the long straight is where the top of the range
               pays.
  timing       decimation 2 on the 45 Hz-capped shimmed bridge -> 22.5 Hz
               control, phase- and latency-locked (env.py). Episode and
               horizon are in seconds, derived at startup. Refuses a sim tick
               outside 15-32 ms.
  reward       progress-dominated with speed pressure LOW (0.2): this stage's
               job is a policy that survives. Stage 6 applies pressure.

Bring the bridge up with the fast loop pinned to the evaluation rate:
    ros2 launch racer_bringup bridge.launch.py tcp_nodelay:=true loop_hz_cap:=45
"""
NAME = "stage5_fresh"
EXPECTED_OBS_DIM = 117
RESUME_FROM = None
COMPETITION_LEGAL = True
DEFAULTS = dict(timesteps=250_000, learning_rate=3e-4, warmup=0, gradient_steps=3)


def apply(cfg):
    # --- observation ---
    cfg.obs.fov_half_deg = 110.0
    cfg.obs.n_beams = 110
    cfg.obs.range_max = 10.0
    cfg.obs.v_max = 26.0
    cfg.obs.yaw_rate_max = 5.0
    # --- action ---
    cfg.act.max_steer = 1.0
    cfg.act.throttle_min = 0.0
    cfg.act.throttle_max = 1.0              # the SCALE; never raised or lowered
    # --- reward (low speed pressure) ---
    cfg.rew.w_progress = 5.0
    cfg.rew.w_speed = 0.2
    cfg.rew.w_grip = 0.0
    cfg.rew.w_center = 0.15
    cfg.rew.w_smooth = 0.3
    cfg.rew.w_prox = 0.5
    cfg.rew.step_penalty = 0.02
    cfg.rew.w_lap = 0.0
    cfg.rew.crash_penalty = 15.0
    cfg.rew.stall_penalty = 5.0
    # --- timing, all in seconds where it matters ---
    cfg.env.decimation = 2
    cfg.env.episode_seconds = 245.0         # ~30 laps
    cfg.env.horizon_seconds = 13.0
    cfg.env.tick_ms_min = 15.0             # refuse an uncapped 13 ms loop ...
    cfg.env.tick_ms_max = 32.0             # ... and a stock 55 ms one
    return cfg
