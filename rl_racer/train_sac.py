#!/usr/bin/env python3
"""SAC training against the live AutoDRIVE RoboRacer simulator (single instance).

Usage:
    python3 train_sac.py --timesteps 400000 --run-name v1
    python3 train_sac.py --resume runs/v1/checkpoints/rl_model_50000_steps.zip
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
                    # action/throttle_sat is the key diagnostic: if it pins near
                    # 1.0 the throttle cap is binding and should be raised.
                    self.logger.record(f"action/{k[4:]}", mean)
                elif k == "max_speed":
                    self.logger.record("race/max_speed", mean)
                elif k == "dist":
                    self.logger.record("race/dist_per_episode_m", mean)
                else:
                    self.logger.record(f"reward/{k}", mean)
            total = sum(self._reasons.values())
            for r, c in self._reasons.items():
                self.logger.record(f"ends/{r}", c / total)
            self._acc.clear()
            self._reasons.clear()
        return True


class ThrottleCurriculum(BaseCallback):
    """Raises the throttle cap once the agent proves it can drive at the current one.

    The cap starts low because uncapped random exploration crashes within a few
    steps and fills the replay buffer with nothing but crashes. It is raised
    only after the agent sustains long, crash-free episodes, so the ceiling
    never becomes what limits lap time.

    The current cap is part of the observation, so transitions recorded under an
    old cap stay valid: action[1]=+1 means different things at different caps,
    and the critic can see which regime a transition came from.
    """

    def __init__(self, env, ceiling=1.0, step=0.02, window=20,
                 crash_rate_max=0.25, min_len=400, cooldown=25_000,
                 demand_min=0.55):
        super().__init__()
        self.env = env
        # step=0.02 is ~+10% top speed per raise. The original 0.10 was ~+50%,
        # which invalidated every corner speed the policy had learned at once
        # (run v3: 28 laps -> 0, episodes 1060 -> 24 steps).
        self.ceiling, self.step = ceiling, step
        self.window, self.crash_rate_max = window, crash_rate_max
        self.min_len, self.cooldown = min_len, cooldown
        # DEMAND gate: mean throttle as a fraction of the cap. Competence alone
        # ("it drives well") is not evidence the agent WANTS more speed, and
        # raising a cap it is not pressing against only destabilises it.
        self.demand_min = demand_min
        self._ends, self._lens, self._thr = [], [], []
        self._last_raise = 0

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            if "episode" in info:                     # added by Monitor
                self._lens.append(info["episode"]["l"])
                self._ends.append(info.get("reason", "?"))
                st = info.get("episode_stats", {})
                self._thr.append(st.get("act_throttle_mean", 0.0))
                self._ends = self._ends[-self.window:]
                self._lens = self._lens[-self.window:]
                self._thr = self._thr[-self.window:]

        cap = self.env.cfg.act.throttle_max
        self.logger.record("action/throttle_cap", cap)

        if (len(self._lens) >= self.window
                and cap < self.ceiling - 1e-6
                and self.num_timesteps - self._last_raise >= self.cooldown):
            crash_rate = sum(e in ("crash", "scrape") for e in self._ends) / len(self._ends)
            mean_len = float(np.mean(self._lens))
            demand = float(np.mean(self._thr)) / max(cap, 1e-6)
            self.logger.record("action/throttle_demand", demand)
            if (crash_rate <= self.crash_rate_max
                    and mean_len >= self.min_len
                    and demand >= self.demand_min):
                new_cap = min(self.ceiling, cap + self.step)
                self.env.cfg.act.throttle_max = new_cap
                self._last_raise = self.num_timesteps
                self._ends.clear(); self._lens.clear(); self._thr.clear()
                print(f"[curriculum] step {self.num_timesteps}: "
                      f"crash={crash_rate:.2f} len={mean_len:.0f} demand={demand:.0%} "
                      f"-> cap {cap:.2f} -> {new_cap:.2f} "
                      f"(~{24*new_cap:.1f} m/s ceiling)", flush=True)
        return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--stage", choices=["1", "2", "3", "4"], default=None,
                   help="staged pipeline: 1=base(v3 repro) 2=sensor adaptation "
                        "3=speed curriculum 4=reserved. Sets config, run name, "
                        "and sensible defaults for that stage.")
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
    # MEASURED: on CPU, 4 gradient steps cost ~75 ms and drop the control
    # loop from 18 Hz to 7.7 Hz. 2 fits inside the 55 ms sim tick, so it
    # keeps full control rate AND does more updates/second than 4 did.
    p.add_argument("--gradient-steps", type=int, default=None,
                   help="updates per env step. Default: keep the checkpoint's value "
                        "on --resume, else 2. Never changed silently.")
    p.add_argument("--ent-coef", type=float, default=None,
                   help="FREEZE the entropy coefficient at this value instead of "
                        "letting SAC re-tune it. On a fine-tune resume auto-tuning "
                        "inflates alpha (a converged policy sits below the entropy "
                        "target), which re-randomises a good policy.")
    p.add_argument("--gamma", type=float, default=None,
                   help="discount. Default: derived from the MEASURED control rate "
                        "so the planning horizon stays fixed in seconds.")
    p.add_argument("--device", default="auto")
    p.add_argument("--no-curriculum", action="store_true",
                   help="fix the throttle cap instead of raising it as the agent improves")
    p.add_argument("--throttle-ceiling", type=float, default=None,
                   help="maximum throttle the curriculum may reach")
    args = p.parse_args()

    stage = stage_registry.load(args.stage) if args.stage else None
    cfg = Cfg()
    if stage is not None:
        stage.apply(cfg)
        d = stage.DEFAULTS
        if args.timesteps is None:     args.timesteps = d["timesteps"]
        if args.run_name is None:      args.run_name = stage.NAME
        if not args.no_curriculum:     args.no_curriculum = not d["curriculum"]
        if args.warmup == 10_000:      args.warmup = d["warmup"]
        if args.throttle_ceiling is None:
            args.throttle_ceiling = d.get("throttle_ceiling", 1.0)
        _lr = d["learning_rate"]
        print(f"\n=== STAGE {args.stage}: {stage.NAME} ===")
        print(f"    {stage.__doc__.strip().splitlines()[0]}")
        print(f"    obs_dim={cfg.obs.dim} (expected {stage.EXPECTED_OBS_DIM})  "
              f"competition_legal={stage.COMPETITION_LEGAL}")
        print(f"    speed slot 91 <- "
              f"{'wheel encoders (legal)' if cfg.obs.use_encoder_speed else '/odom (RESTRICTED)'}")
        print(f"    lr={_lr}  curriculum={d['curriculum']}  timesteps={args.timesteps}")
        if stage.RESUME_FROM and not args.resume:
            print(f"    NOTE: this stage expects --resume from {stage.RESUME_FROM}")
        if cfg.obs.dim != stage.EXPECTED_OBS_DIM:
            raise SystemExit(f"obs_dim {cfg.obs.dim} != stage expectation "
                             f"{stage.EXPECTED_OBS_DIM}; checkpoints will not load")
    else:
        _lr = 3e-4
    if args.throttle_ceiling is None: args.throttle_ceiling = 1.0
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
            # The sim is real-time locked at 20 Hz control, so the GPU is idle
            # ~95% of the time. Spending it on extra gradient steps buys sample
            # efficiency for zero wall-clock cost.
            gradient_steps=(args.gradient_steps if args.gradient_steps is not None else 2),
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
    cbs = [ckpt, RewardBreakdown()]
    if not args.no_curriculum:
        _crm = (stage.DEFAULTS.get("crash_rate_max", 0.25) if stage else 0.25)
        cbs.append(ThrottleCurriculum(raw_env, ceiling=args.throttle_ceiling,
                                      crash_rate_max=_crm))
        print(f"[rl_racer] curriculum: ceiling={args.throttle_ceiling} "
              f"crash_rate_max={_crm} step=+0.02 cooldown=25k")
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
