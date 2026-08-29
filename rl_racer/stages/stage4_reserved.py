"""Stage 4 - RESERVED. Placeholder, not yet defined.

Candidate use: robustness. Randomise control rate and inject sensor noise so the
policy does not depend on this machine's timing -- the competition hardware may
run at a different rate. Measured here: the v3 policy was rate-robust at
inference (7.79 s/lap at 7.7 Hz vs 7.99 s/lap at 18.8 Hz), so this is insurance
rather than a known problem.
"""
NAME = "stage4_reserved"
EXPECTED_OBS_DIM = 95
RESUME_FROM = "runs/stage3_speed/checkpoints/"
COMPETITION_LEGAL = True
DEFAULTS = dict(timesteps=200_000, learning_rate=1e-4, curriculum=False, warmup=0)


def apply(cfg):
    raise SystemExit("stage 4 is not defined yet -- see stages/stage4_reserved.py")
