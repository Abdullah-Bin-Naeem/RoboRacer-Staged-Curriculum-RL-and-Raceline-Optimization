"""Stage 1 - BASE. Reproduce the v3 policy from scratch.

Uses /odom for speed, exactly as v3 did, INCLUDING the forward/reverse sign bug
it trained with. That is deliberate: this stage exists to reproduce a known-good
policy, not to be correct. /odom is restricted at race time, so a stage-1 policy
is NOT competition-legal -- stage 2 fixes that.

Reference result: v3 @370k -> 1800 steps, 30 laps, zero crashes, 7.79 s/lap.
"""
NAME = "stage1_base"
EXPECTED_OBS_DIM = 95
RESUME_FROM = None                      # trains from scratch
COMPETITION_LEGAL = False
# 370k: the step count that actually produced the good v3 policy.
DEFAULTS = dict(timesteps=370_000, learning_rate=3e-4, curriculum=False, warmup=0)


def apply(cfg):
    # --- observation geometry (v3) ---
    cfg.obs.fov_half_deg = 90.0
    cfg.obs.n_beams = 90
    cfg.obs.v_max = 20.0
    cfg.obs.yaw_rate_max = 5.0
    # --- the two slots that define the stage ---
    cfg.obs.use_encoder_speed = False   # slot 91 <- /odom  (RESTRICTED at race)
    cfg.env.legacy_speed_sign = True    # as v3 trained
    # --- reward (v3) ---
    cfg.rew.w_progress = 5.0
    cfg.rew.w_speed = 0.2
    cfg.rew.w_lap = 0.0
    # --- actions ---
    cfg.act.max_steer = 1.0
    cfg.act.throttle_max = 0.20
    return cfg
