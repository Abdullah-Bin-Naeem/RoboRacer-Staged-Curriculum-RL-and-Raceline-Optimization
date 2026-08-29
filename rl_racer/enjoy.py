#!/usr/bin/env python3
"""Run a trained checkpoint in AutoDRIVE (deterministic, no exploration)."""
import argparse, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from stable_baselines3 import SAC
from rl_racer.config import Cfg
from rl_racer.env import AutoDriveRacerEnv

p = argparse.ArgumentParser()
p.add_argument("model")
p.add_argument("--episodes", type=int, default=5)
p.add_argument("--legacy-obs", action="store_true",
               help="restore v3's exact 95-dim observation (90 beams, +/-90 deg, "
                    "odom speed with the old sign bug). Needed to load a v1-v3 "
                    "checkpoint. Runtime override only -- config.py is untouched.")
p.add_argument("--speed", choices=["odom", "encoder", "config"], default="config",
               help="which SPEED source feeds observation slot 91. "
                    "'odom' = restricted topic the v1-v3 policies trained on; "
                    "'encoder' = race-legal wheel encoders. Comparing the two on "
                    "the SAME checkpoint measures the deployment gap.")
p.add_argument("--steps", type=int, default=0,
               help="cap the episode at this many control steps (0 = use "
                    "config.py's max_episode_steps, 4400). Handy when driving "
                    "for a mapping run rather than a timed evaluation. "
                    "Runtime override only -- config.py is untouched.")
p.add_argument("--hz", type=float, default=0.0,
               help="pace the control loop to this rate (v3 trained at ~7.7 Hz; "
                    "0 = run as fast as the sim allows, ~18 Hz)")
a = p.parse_args()

cfg = Cfg()
if a.steps > 0:
    cfg.env.max_episode_steps = a.steps
    print(f"[steps] episode capped at {a.steps} steps")

if a.legacy_obs:
    cfg.obs.fov_half_deg = 90.0     # v3 field of view
    cfg.obs.n_beams = 90            # -> 95-dim observation
    cfg.obs.v_max = 20.0
    cfg.obs.yaw_rate_max = 5.0
    cfg.obs.use_encoder_speed = False   # speed from /odom, as v3 had it
    cfg.env.legacy_speed_sign = True    # including the sign bug it trained with
    print(f"[legacy-obs] obs_dim={cfg.obs.dim} fov=+/-90 beams=90")

if a.speed == "odom":
    cfg.obs.use_encoder_speed = False
    cfg.env.legacy_speed_sign = True     # as the v1-v3 policies were trained
elif a.speed == "encoder":
    cfg.obs.use_encoder_speed = True     # race-legal source
print(f"[speed] slot 91 <- {'/odom (RESTRICTED at race time)' if not cfg.obs.use_encoder_speed else 'wheel encoders (race-legal)'}")

env = AutoDriveRacerEnv(cfg)
model = SAC.load(a.model, device="cpu")
try:
    for ep in range(a.episodes):
        obs, _ = env.reset()
        import time as _t; ep_t0 = _t.perf_counter()
        total, steps, done = 0.0, 0, False
        import time
        period = 1.0 / a.hz if a.hz > 0 else 0.0
        while not done:
            t0 = time.perf_counter()
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            total += r; steps += 1; done = term or trunc
            if period:
                slack = period - (time.perf_counter() - t0)
                if slack > 0: time.sleep(slack)
        # Lap time straight from the SIMULATOR's own timer -- do not compute it
        # as steps x dt: the startup-measured control period excludes per-step
        # policy inference, which made every derived lap time ~5% optimistic.
        sim_last = float(getattr(env.node, "last_lap_time", 0.0) or 0.0)
        wall = time.perf_counter() - ep_t0
        laps = info.get("laps", 0) or 0
        derived = wall / laps if laps else float("nan")
        print(f"ep {ep}: reward={total:8.1f} steps={steps:5d} "
              f"laps={laps} end={info.get('reason','?')}")
        print(f"        lap time: sim={sim_last:.2f}s  "
              f"wall-clock avg={derived:.2f}s  (episode {wall:.1f}s)")
finally:
    env.close()
