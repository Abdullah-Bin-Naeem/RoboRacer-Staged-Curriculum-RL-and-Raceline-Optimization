"""Stage 5 - FRESH LINEAGE, from scratch. Learn to drive the competition track.

Not a continuation of stages 1-4: the observation is 118-dim, so no earlier
checkpoint loads. Everything the earlier lineage learned the hard way is built
in from step one:

  observation  110 beams at +/-110 deg (2.0 deg/beam; the exit corridor of a
               >=180 deg hairpin sat ON the old +/-90 boundary), the 5 existing
               slots, and 3 race-legal slip slots: observer car speed v_est,
               longitudinal slip S, yaw-rate residual. Encoder speed is a
               3-tick window with a counter-discontinuity guard (the sim's
               ResetManager restores the encoders on every reset).
  throttle     scale 0.5, FIXED. No curriculum, so no re-labelling of learned
               actions. Peak-grip acceleration needs throttle ~1.15 x v/25.25
               (0.36 at 8 m/s); above ~0.40 the tire is past its asymptote and
               more throttle is wheelspin, so 0.5 covers the track with margin.
               RL finds its own speed.
  timing       decimation 2 on the 45 Hz-capped shimmed bridge -> 22.5 Hz
               control; 3 GPU gradient steps (~30 ms) fit the 44 ms budget.
               Episode and horizon are in seconds, derived at startup.
  reward       stage 3's proven weights with speed pressure LOW (0.2): this
               stage's job is a policy that survives. Stage 6 applies pressure.

Bring the bridge up with the fast loop pinned to the evaluation rate:
    ros2 launch racer_bringup bridge.launch.py tcp_nodelay:=true loop_hz_cap:=45
"""
NAME = "stage5_fresh"
EXPECTED_OBS_DIM = 118
RESUME_FROM = None
COMPETITION_LEGAL = True
DEFAULTS = dict(timesteps=250_000, learning_rate=3e-4, curriculum=False, warmup=0,
                gradient_steps=3, throttle_ceiling=0.5)


def apply(cfg):
    # --- observation ---
    cfg.obs.fov_half_deg = 110.0
    cfg.obs.n_beams = 110
    cfg.obs.range_max = 10.0
    cfg.obs.v_max = 20.0
    cfg.obs.yaw_rate_max = 5.0
    cfg.obs.use_encoder_speed = True
    cfg.obs.slip_slots = True
    # --- action ---
    cfg.act.max_steer = 1.0
    cfg.act.throttle_min = 0.0
    cfg.act.throttle_max = 0.5              # the SCALE; never raised
    # --- reward (stage 3's weights, low speed pressure) ---
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
    cfg.env.episode_seconds = 245.0         # v3's episode length (~30 laps)
    cfg.env.horizon_seconds = 13.0
    cfg.env.legacy_speed_sign = False
    cfg.env.tick_ms_min = 15.0             # refuse an uncapped 13 ms loop ...
    cfg.env.tick_ms_max = 32.0             # ... and a stock 55 ms one
    return cfg
