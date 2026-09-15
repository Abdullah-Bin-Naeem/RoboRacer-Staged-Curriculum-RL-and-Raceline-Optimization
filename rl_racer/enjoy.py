#!/usr/bin/env python3
"""Run a trained checkpoint in AutoDRIVE (deterministic, no exploration).

    python enjoy.py runs/stage5_fresh/final.zip --episodes 3     # validation gate
    python enjoy.py runs/stage6_push/final.zip --race            # deployment
"""
import argparse, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stable_baselines3 import SAC
import stages
from rl_racer.config import Cfg
from rl_racer.env import AutoDriveRacerEnv

p = argparse.ArgumentParser()
p.add_argument("model")
p.add_argument("--stage", choices=["5", "6"], default="5",
               help="stage config to apply (both share the observation; only the "
                    "reward differs, which does not run here)")
p.add_argument("--episodes", type=int, default=3)
p.add_argument("--steps", type=int, default=0,
               help="cap the episode at this many control steps instead of the "
                    "stage's episode_seconds (0 = stage default)")
p.add_argument("--race", action="store_true",
               help="DEPLOYMENT: never send a reset pulse, never stop on a collision "
                    "or stall, no step cap -- drive until Ctrl-C. The sim-tick window "
                    "check becomes a warning instead of a refusal.")
a = p.parse_args()

cfg = Cfg()
_st = stages.load(a.stage)
_st.apply(cfg)
if cfg.obs.dim != _st.EXPECTED_OBS_DIM:
    raise SystemExit(f"obs_dim {cfg.obs.dim} != stage expectation {_st.EXPECTED_OBS_DIM}")
print(f"[stage {a.stage}] {_st.NAME}: obs_dim={cfg.obs.dim} fov=+/-{cfg.obs.fov_half_deg:g} "
      f"beams={cfg.obs.n_beams} throttle_scale={cfg.act.throttle_max} decimation={cfg.env.decimation}")
if a.steps > 0:
    cfg.env.max_episode_steps = a.steps
    cfg.env.episode_seconds = 0.0       # else the env re-derives the cap at startup
    print(f"[steps] episode capped at {a.steps} steps")
_tick_window = (cfg.env.tick_ms_min, cfg.env.tick_ms_max)
if a.race:
    cfg.env.race_mode = True
    cfg.env.episode_seconds = 0.0
    cfg.env.max_episode_steps = 1 << 62
    cfg.env.tick_ms_min = cfg.env.tick_ms_max = 0.0     # warn below, never refuse
    a.episodes = 1
    print("[race] no reset pulse, no termination on collision/stall, no step cap; Ctrl-C to stop")

env = AutoDriveRacerEnv(cfg)
if a.race and any(_tick_window):
    _tick_ms = 1000.0 * env.control_period / cfg.env.decimation
    lo, hi = _tick_window
    if (lo > 0 and _tick_ms < lo) or (hi > 0 and _tick_ms > hi):
        print(f"[race] WARNING: sim tick {_tick_ms:.1f} ms is outside the {lo:g}-{hi:g} ms "
              f"window this policy trained in; driving anyway.")
model = SAC.load(a.model, device="cpu")
try:
    for ep in range(a.episodes):
        obs, _ = env.reset()
        ep_t0 = time.perf_counter()
        total, steps, done = 0.0, 0, False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            total += r; steps += 1; done = term or trunc
        # Lap time straight from the SIMULATOR's own timer -- not steps x dt:
        # the startup-measured period excludes per-step inference and made
        # every derived lap time ~5% optimistic.
        sim_last = float(getattr(env.node, "last_lap_time", 0.0) or 0.0)
        wall = time.perf_counter() - ep_t0
        laps = info.get("laps", 0) or 0
        derived = wall / laps if laps else float("nan")
        print(f"ep {ep}: reward={total:8.1f} steps={steps:5d} "
              f"laps={laps} end={info.get('reason','?')}")
        st = info.get("episode_stats", {})
        print(f"        lap time: sim={sim_last:.2f}s  "
              f"wall-clock avg={derived:.2f}s  (episode {wall:.1f}s)")
        if "slip_v_est_max" in st:
            print(f"        v_est max={st['slip_v_est_max']:.2f} m/s  mean={st['slip_v_est_mean']:.2f}  "
                  f"|S| mean={st['slip_abs_mean']:.3f}  at-peak-grip={st['slip_frac_peak']:.0%} of steps")
except KeyboardInterrupt:
    print("\n[enjoy] stopped by Ctrl-C -- throttle released")
finally:
    env.close()
