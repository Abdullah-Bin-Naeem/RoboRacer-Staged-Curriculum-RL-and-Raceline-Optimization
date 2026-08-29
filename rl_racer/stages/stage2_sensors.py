"""Stage 2 - SENSOR ADAPTATION. Same policy, race-legal inputs.

Identical to stage 1 except the two observation slots move to permitted topics.
Only slot 91 actually changes value; slot 92 is the same number from a different
topic (the bridge feeds /imu and /odom from one variable).

Encoder speed matches /odom to 0.3% at steady state but overreads up to ~3x
under hard acceleration (real wheel slip), so the policy has one noisier input
to adapt to. Measured cost of the swap on the un-adapted v3 policy: ~9% lap time
(8.50 s vs 7.79 s). Recovering that is this stage's whole job.

Keep it SHORT and stop when lap time plateaus -- this is fine-tuning one input,
not relearning to drive. Low LR to avoid destroying a good policy.
"""
NAME = "stage2_sensors"
EXPECTED_OBS_DIM = 95
RESUME_FROM = "runs/stage1_base/checkpoints/  (or runs/v3/checkpoints/sac_370000_steps.zip)"
COMPETITION_LEGAL = True
DEFAULTS = dict(timesteps=150_000, learning_rate=1e-4, curriculum=False, warmup=10_000)


def apply(cfg):
    from stages import stage1_base
    stage1_base.apply(cfg)              # everything else identical to stage 1
    # --- the only difference ---
    cfg.obs.use_encoder_speed = True    # slot 91 <- wheel encoders (legal)
    cfg.env.legacy_speed_sign = False   # moot on the encoder path; sign is natural
    return cfg
