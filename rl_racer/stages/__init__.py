"""Staged training pipeline.

Two lineages. Within a lineage every stage emits the same observation, which is
what lets checkpoints resume across stages; across lineages nothing loads.

  v3 lineage, 95-dim (stages 1-4): 90 beams at +/-90 deg. Stage 1 -> 2 differs
  only in where two observation slots are sourced from:

    slot 91 (speed)     stage 1: /odom twist   ->  stage 2+: wheel encoders
    slot 92 (yaw rate)  stage 1: /odom angular ->  stage 2+: /imu angular

Slot 92 is a relabel only -- the bridge feeds both topics from the same
variable, so the number is identical. Slot 91 is a real signal change.

  fresh lineage, 118-dim (stages 5-6): 110 beams at +/-110 deg plus three
  race-legal slip slots; throttle scale fixed at 0.5, no curriculum.
"""
import importlib

STAGES = {
    "1": "stages.stage1_base",
    "2": "stages.stage2_sensors",
    "3": "stages.stage3_speed",
    "4": "stages.stage4_reserved",
    "5": "stages.stage5_fresh",
    "6": "stages.stage6_push",
}


def load(name):
    if name not in STAGES:
        raise SystemExit(f"unknown stage {name!r}; choose from {sorted(STAGES)}")
    return importlib.import_module(STAGES[name])
