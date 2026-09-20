"""Staged training pipeline. Every stage emits the same 117-dim observation
(110 beams at +/-110 deg + u, yaw_rate, prev_steer, prev_throttle, v_est, S,
yaw_residual), which is what lets checkpoints resume across stages. Stage N's
apply() calls stage N-1's, so stages compose rather than duplicate.
"""
import importlib

STAGES = {
    "5": "stages.stage5_fresh",
    "6": "stages.stage6_push",
    "7": "stages.stage7_smooth",
    "8": "stages.stage8_flow",
    "9": "stages.stage9_limit",
}


def load(name):
    if name not in STAGES:
        raise SystemExit(f"unknown stage {name!r}; choose from {sorted(STAGES)}")
    return importlib.import_module(STAGES[name])
