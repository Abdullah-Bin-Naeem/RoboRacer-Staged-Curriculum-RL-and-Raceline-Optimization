"""Staged training pipeline.

Every stage MUST produce the same 95-dim observation, otherwise checkpoints
cannot be loaded across stages. The only thing that differs between stage 1 and
stages 2-3 is where two observation slots are sourced from:

    slot 91 (speed)     stage 1: /odom twist   ->  stage 2+: wheel encoders
    slot 92 (yaw rate)  stage 1: /odom angular ->  stage 2+: /imu angular

Slot 92 is a relabel only -- the bridge feeds both topics from the same
variable, so the number is identical. Slot 91 is a real signal change.
"""
import importlib

STAGES = {
    "1": "stages.stage1_base",
    "2": "stages.stage2_sensors",
    "3": "stages.stage3_speed",
    "4": "stages.stage4_reserved",
}


def load(name):
    if name not in STAGES:
        raise SystemExit(f"unknown stage {name!r}; choose from {sorted(STAGES)}")
    return importlib.import_module(STAGES[name])
