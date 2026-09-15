#!/usr/bin/env python3
"""SAC training against the live AutoDRIVE RoboRacer simulator (single instance).

Usage:
    ./run_train.sh --stage 5                                   # from scratch
    ./run_train.sh --stage 6 --resume runs/stage5_fresh/final.zip
"""
import argparse
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor

import stages as stage_registry
from rl_racer.config import Cfg
from rl_racer.env import AutoDriveRacerEnv


class RewardBreakdown(BaseCallback):
    """Logs each reward component separately.

    This is the single most useful diagnostic for 'is SAC converging or is my
    reward broken' -- total episode reward alone cannot tell you which term is
    dominating.
    """

    def __init__(self):
        super().__init__()
        self._acc, self._reasons = [], {}
        self._warned_yaw = False
        self._warned_period = False
        self.design_period_ms = None     # set by main() from the env

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            if "episode_stats" in info:
                self._acc.append(info["episode_stats"])
                r = info.get("reason", "?")
                self._reasons[r] = self._reasons.get(r, 0) + 1
            if "laps" in info and info.get("episode_stats"):
                self.logger.record("race/laps_completed", info["laps"])

        if len(self._acc) >= 5:
            keys = self._acc[0].keys()
            for k in keys:
                mean = float(np.mean([d[k] for d in self._acc]))
                if k.startswith("act_"):
                    self.logger.record(f"action/{k[4:]}", mean)
                elif k.startswith("slip_"):
                    # slip/frac_peak: share of steps with |S| in [0.10, 0.20] --
                    # "is it using the tire". slip/v_est_max: real top speed.
                    self.logger.record(f"slip/{k[5:]}", mean)
                elif k == "enc_discontinuities":
                    self.logger.record("diag/enc_discontinuities", mean)
                elif k.startswith("diag_"):
                    self.logger.record(f"diag/{k[5:]}", mean)
                elif k == "max_speed":
                    self.logger.record("race/max_speed", mean)
                elif k == "dist":
                    self.logger.record("race/dist_per_episode_m", mean)
                else:
                    self.logger.record(f"reward/{k}", mean)
            # Yaw-residual sign sanity: if the residual is LARGER than the yaw
            # rate itself, the kinematic term is being added instead of
            # subtracted (steering sign convention wrong) and slot 118 is junk.
            st = self._acc[0]
            if ("slip_yaw_res_rms" in st and not self._warned_yaw
                    and float(np.mean([d["slip_yaw_rate_rms"] for d in self._acc])) > 0.5
                    and float(np.mean([d["slip_yaw_res_rms"] for d in self._acc]))
                        > 1.3 * float(np.mean([d["slip_yaw_rate_rms"] for d in self._acc]))):
                print("[rl_racer] WARNING: yaw residual rms exceeds yaw-rate rms -- "
                      "steering sign convention in sensors.yaw_residual is likely "
                      "inverted for this build; the residual slot is not informative.",
                      flush=True)
                self._warned_yaw = True
            # Control-period sanity: gamma and the episode length were derived
            # from the startup-measured period. If the loop as DRIVEN is slower,
            # the caller's overhead is outlasting the decimation window and
            # every step is a tick longer than designed.
            if ("diag_step_ms" in st and self.design_period_ms and not self._warned_period):
                _p = float(np.mean([d["diag_step_ms"] for d in self._acc]))
                _o = float(np.mean([d["diag_overhead_ms"] for d in self._acc]))
                if _p > 1.15 * self.design_period_ms:
                    print(f"[rl_racer] WARNING: control period as driven is {_p:.1f} ms vs "
                          f"{self.design_period_ms:.1f} ms designed (overhead {_o:.1f} ms/step). "
                          f"Gradient steps are outlasting the decimation window: lower "
                          f"--gradient-steps or the tick budget is wrong for gamma.", flush=True)
                    self._warned_period = True
            total = sum(self._reasons.values())
            for r, c in self._reasons.items():
                self.logger.record(f"ends/{r}", c / total)
            self._acc.clear()
            self._reasons.clear()
        return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--stage", choices=["5", "6"], default="5",
                   help="5 = learn to drive (from scratch), 6 = speed pressure "
                        "(resumes stage 5 with its buffer). Sets config, run name "
                        "and defaults for the stage.")
    p.add_argument("--timesteps", type=int, default=None)
    p.add_argument("--run-name", default=None)
    p.add_argument("--logdir", default="runs")
    p.add_argument("--resume", default=None, help="path to a saved .zip to continue from")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--warmup", type=int, default=10_000,
                   help="on resume without a saved buffer, steps to collect "
                        "with the loaded policy before training starts")
    p.add_argument("--no-sde", action="store_true",
                   help="disable gSDE (state-dependent exploration)")
    # MEASURED: ~10 ms per gradient step on the RTX 4070. The env absorbs up to
    # one 44 ms control window of caller time (phase lock); 3 steps (~30 ms)
    # fit, and diag/overhead_ms in TensorBoard shows the real figure.
    p.add_argument("--gradient-steps", type=int, default=None,
                   help="updates per env step. Default: the stage's value, or the "
                        "checkpoint's on --resume. Never changed silently.")
    p.add_argument("--ent-coef", type=float, default=None,
                   help="FREEZE the entropy coefficient at this value instead of "
                        "letting SAC re-tune it. On a fine-tune resume auto-tuning "
                        "inflates alpha (a converged policy sits below the entropy "
                        "target), which re-randomises a good policy.")
    p.add_argument("--gamma", type=float, default=None,
                   help="discount. Default: derived from the MEASURED control rate "
                        "so the planning horizon stays fixed in seconds.")
    p.add_argument("--device", default="auto")
    args = p.parse_args()

    stage = stage_registry.load(args.stage) if args.stage else None
    cfg = Cfg()
    if stage is not None:
        stage.apply(cfg)
        d = stage.DEFAULTS
        if args.timesteps is None:     args.timesteps = d["timesteps"]
        if args.run_name is None:      args.run_name = stage.NAME
        if args.warmup == 10_000:      args.warmup = d["warmup"]
        if args.gradient_steps is None and "gradient_steps" in d:
            args.gradient_steps = d["gradient_steps"]
        _lr = d["learning_rate"]
        print(f"\n=== STAGE {args.stage}: {stage.NAME} ===")
        print(f"    {stage.__doc__.strip().splitlines()[0]}")
        print(f"    obs_dim={cfg.obs.dim} (expected {stage.EXPECTED_OBS_DIM})  "
              f"competition_legal={stage.COMPETITION_LEGAL}")
        print(f"    fov=+/-{cfg.obs.fov_half_deg:g} beams={cfg.obs.n_beams}  "
              f"throttle scale={cfg.act.throttle_max}  decimation={cfg.env.decimation}")
        print(f"    reward: progress={cfg.rew.w_progress} speed={cfg.rew.w_speed} "
              f"center={cfg.rew.w_center} lap={cfg.rew.w_lap} grip={cfg.rew.w_grip}")
        print(f"    lr={_lr}  timesteps={args.timesteps}  gradient_steps={args.gradient_steps}")
        if stage.RESUME_FROM and not args.resume:
            print(f"    NOTE: this stage expects --resume from {stage.RESUME_FROM}")
        if cfg.obs.dim != stage.EXPECTED_OBS_DIM:
            raise SystemExit(f"obs_dim {cfg.obs.dim} != stage expectation "
                             f"{stage.EXPECTED_OBS_DIM}; checkpoints will not load")
    else:
        _lr = 3e-4
    if args.timesteps is None: args.timesteps = 400_000
    if args.run_name is None:  args.run_name = "v1"

    run_dir = os.path.join(args.logdir, args.run_name)
    os.makedirs(run_dir, exist_ok=True)
    raw_env = AutoDriveRacerEnv(cfg)
    # gamma from the MEASURED control rate, so the planning horizon is a fixed
    # number of SECONDS regardless of how fast this machine happens to run.
    _dt = raw_env.control_period
    _gamma = float(min(0.9999, max(0.90, 1.0 - _dt / cfg.env.horizon_seconds)))
    print(f"[rl_racer] gamma={_gamma:.5f} from dt={_dt*1000:.1f}ms "
          f"and horizon={cfg.env.horizon_seconds:g}s "
          f"({cfg.env.horizon_seconds/_dt:.0f} steps)")
    env = Monitor(raw_env, filename=os.path.join(run_dir, "monitor.csv"))

    if args.resume:
        print(f"[rl_racer] resuming from {args.resume}")
        model = SAC.load(args.resume, env=env, device=args.device)
        # SAC.load restores these from the checkpoint, so CLI flags are ignored
        # unless we reapply them. Without the first line the run logs into the
        # OLD run's tensorboard directory.
        model.tensorboard_log = run_dir

        # Everything below is restored from the checkpoint by SAC.load(), so any
        # value we want to differ must be reapplied EXPLICITLY -- and printed, so
        # nothing changes silently between stages.
        if args.gradient_steps is not None and args.gradient_steps != model.gradient_steps:
            print(f"[rl_racer] gradient_steps {model.gradient_steps} -> {args.gradient_steps}")
            model.gradient_steps = args.gradient_steps
        else:
            print(f"[rl_racer] gradient_steps {model.gradient_steps} (from checkpoint)")

        _g = args.gamma if args.gamma is not None else _gamma
        if abs(model.gamma - _g) > 1e-9:
            print(f"[rl_racer] gamma {model.gamma:.5f} -> {_g:.5f}  "
                  f"(checkpoint horizon {_dt/(1-model.gamma):.1f}s -> {_dt/(1-_g):.1f}s "
                  f"at this machine's {1/_dt:.1f} Hz)")
            model.gamma = _g
        else:
            print(f"[rl_racer] gamma {model.gamma:.5f} (unchanged)")

        if abs(model.learning_rate - _lr) > 1e-12:
            print(f"[rl_racer] learning_rate {model.learning_rate} -> {_lr}")
        if args.ent_coef is not None:
            import torch as _th
            _old = float(model.log_ent_coef.exp()) if getattr(model, "log_ent_coef", None) is not None \
                   else float(getattr(model, "ent_coef_tensor", float("nan")))
            model.ent_coef_optimizer = None
            model.log_ent_coef = None
            model.ent_coef_tensor = _th.tensor(float(args.ent_coef), device=model.device)
            print(f"[rl_racer] ent_coef FROZEN at {args.ent_coef} (was auto, {_old:.4f})")

        model.learning_rate = _lr        # stages fine-tune at a lower LR
        model._setup_lr_schedule()

        # SB3's CheckpointCallback saves "<prefix>_replay_buffer_<N>_steps.pkl",
        # NOT "<checkpoint>_replay_buffer.pkl". Getting this wrong means the
        # buffer silently never loads and every resume starts empty -- which is
        # what collapsed run v4 and stage3_v2.
        import re as _re, glob as _glob
        _d = os.path.dirname(args.resume) or "."
        _m = _re.search(r"_(\d+)_steps\.zip$", os.path.basename(args.resume))
        _cands = []
        if _m:
            _cands += _glob.glob(os.path.join(_d, f"*replay_buffer_{_m.group(1)}_steps.pkl"))
        _cands += _glob.glob(args.resume.replace(".zip", "_replay_buffer.pkl"))
        _cands += _glob.glob(args.resume.replace(".zip", "_replay_buffer.pkl").replace("final", "final"))
        buf = _cands[0] if _cands else ""
        if buf and os.path.exists(buf):
            model.load_replay_buffer(buf)
            print(f"[rl_racer] loaded replay buffer: {model.replay_buffer.size():,} transitions "
                  f"from {os.path.basename(buf)}")
            model.learning_starts = 0        # buffer is full; train immediately
        elif args.warmup > 0:
            # Checkpoints saved before this fix carry NO replay buffer. Starting
            # gradient updates against an empty buffer is exactly what collapsed
            # run v4 (episodes fell 850 -> 24 steps): ~4 updates/step on a few
            # hundred samples makes the critic memorise a sliver of experience.
            # learning_starts gates training, so push it ahead of the current
            # step count to collect first; use_sde_at_warmup makes that
            # collection use the LOADED POLICY instead of random actions.
            model.learning_starts = model.num_timesteps + args.warmup
            model.use_sde_at_warmup = True
            print(f"[rl_racer] NO buffer found next to {os.path.basename(args.resume)} -- "
                  f"collecting {args.warmup} steps with the loaded policy first")
        else:
            print("[rl_racer] WARNING: no replay buffer AND warmup=0 -- gradient "
                  "updates will start against a near-empty buffer. This is how "
                  "v4 and stage3_v2 collapsed. Use --warmup 10000.")
    else:
        model = SAC(
            "MlpPolicy",
            env,
            learning_rate=_lr,
            buffer_size=300_000,
            learning_starts=5_000,     # ~5 min of random driving to seed the buffer
            batch_size=256,
            tau=0.005,
            gamma=(args.gamma if args.gamma is not None else _gamma),
            train_freq=1,
            gradient_steps=(args.gradient_steps if args.gradient_steps is not None else 3),
            ent_coef="auto",
            # gSDE gives temporally correlated exploration instead of per-step
            # white noise -- much smoother steering, which is what we want here.
            use_sde=not args.no_sde,
            sde_sample_freq=8,
            policy_kwargs=dict(net_arch=[256, 256]),
            tensorboard_log=run_dir,
            seed=args.seed,
            verbose=1,
            device=args.device,
        )

    ckpt = CheckpointCallback(
        save_freq=10_000,
        save_path=os.path.join(run_dir, "checkpoints"),
        name_prefix="sac",
        save_replay_buffer=True,    # ~250 MB each, but without it a resume
                                    # starts from an empty buffer and collapses
    )

    def _bail(signum, frame):
        print("\n[rl_racer] interrupted -- saving and stopping the car")
        model.save(os.path.join(run_dir, "interrupted"))
        env.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, _bail)

    import torch as _t
    _dev = str(model.device)
    _gpu = _t.cuda.get_device_name(0) if _t.cuda.is_available() else "none"
    print(f"[rl_racer] obs_dim={cfg.obs.dim} "
          f"fov=+/-{cfg.obs.fov_half_deg:g}deg beams={cfg.obs.n_beams} "
          f"decimation={cfg.env.decimation}")
    print(f"[rl_racer] DEVICE = {_dev.upper()}   gpu={_gpu}   "
          f"gradient_steps={model.gradient_steps}")
    if _dev == "cpu":
        print("[rl_racer] WARNING: running on CPU -- expect ~7 fps instead of ~18")
    _rb = RewardBreakdown()
    _rb.design_period_ms = 1000.0 * raw_env.control_period
    cbs = [ckpt, _rb]
    try:
        model.learn(total_timesteps=args.timesteps, callback=cbs,
                    reset_num_timesteps=args.resume is None,
                    tb_log_name="sac", progress_bar=False)
    finally:
        model.save(os.path.join(run_dir, "final"))
        try:      # so the NEXT stage can resume with a populated buffer
            model.save_replay_buffer(os.path.join(run_dir, "final_replay_buffer"))
            print(f"[rl_racer] saved replay buffer -> {run_dir}/final_replay_buffer.pkl")
        except Exception as e:
            print(f"[rl_racer] could not save replay buffer: {e}")
        env.close()
        print(f"[rl_racer] saved -> {run_dir}/final.zip")


if __name__ == "__main__":
    main()
