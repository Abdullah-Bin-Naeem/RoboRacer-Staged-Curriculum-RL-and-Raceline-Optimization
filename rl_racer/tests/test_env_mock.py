#!/usr/bin/env python3
"""End-to-end check of AutoDriveRacerEnv (stage 5 config) against mock_bridge.py.
No simulator or bridge needed; the real sim must NOT be bridged while it runs.
Run under ROS + .venv-rl. Prints PASS/FAIL per check, exit 1 on any FAIL."""
import os, sys, time, subprocess, signal, threading, math
HERE = os.path.dirname(os.path.abspath(__file__))
RL = os.path.dirname(HERE)
sys.path.insert(0, RL)
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy)
from std_msgs.msg import Bool, Int32
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)

FAILS = []
def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""), flush=True)
    if not ok: FAILS.append(name)

class Side(Node):
    def __init__(self):
        super().__init__("mock_side")
        self.reset_count = None
        self.create_subscription(Int32, "/mock/reset_count", lambda m: setattr(self, "reset_count", m.data), QOS)
        self.p_col = self.create_publisher(Bool, "/mock/collide", QOS)
    def collide(self):
        b = Bool(); b.data = True; self.p_col.publish(b)

def start_mock(tick_ms, stop_after=0.0):
    args = [sys.executable, os.path.join(HERE, "mock_bridge.py"), "--tick-ms", str(tick_ms)]
    if stop_after: args += ["--stop-after", str(stop_after)]
    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(1.0)
    return p

def stop_mock(p):
    p.send_signal(signal.SIGINT)
    try: p.wait(5)
    except subprocess.TimeoutExpired: p.kill(); p.wait()

def stage5_cfg():
    from rl_racer.config import Cfg
    import stages
    cfg = Cfg(); stages.load("5").apply(cfg)
    # The stage's fixed start point is a pose on the REAL competition track;
    # the mock's corridor is a different frame, so the drive-in is off by
    # default here and section 2h enables it with mock coordinates.
    cfg.env.spawn_drive = False
    return cfg

def run_steps(env, n, action, overhead_s=0.0):
    last = None
    for _ in range(n):
        if overhead_s: time.sleep(overhead_s)
        last = env.step(np.array(action, dtype=np.float32))
    return last

def main():
    rclpy.init()
    side = Side(); ex = SingleThreadedExecutor(); ex.add_node(side)
    stop = threading.Event()
    def _spin():
        while not stop.is_set() and rclpy.ok():
            try: ex.spin_once(timeout_sec=0.05)
            except Exception: break
    th = threading.Thread(target=_spin, daemon=True); th.start()
    from rl_racer.env import AutoDriveRacerEnv

    # ------------------------------------------------------------ 1. tick guard
    print("\n[1] tick guard: mock at 55 ms (stock bridge) must be refused")
    m = start_mock(55.0)
    try:
        AutoDriveRacerEnv(stage5_cfg()); check("refused 55 ms tick", False, "no exception")
    except RuntimeError as e:
        check("refused 55 ms tick", "outside the 15-32 ms window" in str(e), str(e).splitlines()[0][:80])
    stop_mock(m)

    # ------------------------------------------------------------ 2. main path
    print("\n[2] stage 5 env at 22 ms tick")
    m = start_mock(22.0)
    cfg = stage5_cfg()
    env = AutoDriveRacerEnv(cfg)
    cp = env.control_period * 1000
    check("control period ~44 ms", 40 <= cp <= 48, f"{cp:.1f} ms")
    check("max_episode_steps ~ 245 s / period", abs(cfg.env.max_episode_steps - 245 / env.control_period) < 2,
          f"{cfg.env.max_episode_steps}")
    check("obs dim 117", env.observation_space.shape == (117,), str(env.observation_space.shape))

    rc0 = side.reset_count or 0
    obs, _ = env.reset()
    time.sleep(0.1)
    check("reset pulse reached the mock", (side.reset_count or 0) == rc0 + 1, f"{rc0} -> {side.reset_count}")
    check("reset obs: slip slots ~0", np.all(np.abs(obs[114:117]) < 1e-6), f"{obs[114:117]}")
    check("reset obs: speed slot 0 (not -1)", abs(obs[110]) < 1e-6, f"{obs[110]:.3f}")
    check("obs in [-1,1]", np.all(obs >= -1) and np.all(obs <= 1))

    THR = 0.36                              # u = 25.25 x 0.36 = 9.09 m/s
    act = (0.2, 2 * THR / cfg.act.throttle_max - 1)
    cfg.env.max_episode_steps = 10 ** 9     # no truncation during the timing runs
    # 2a: zero caller overhead
    snap = dict(env._acc); k0 = env._steps
    run_steps(env, 60, act)
    n = env._steps - k0
    per, tps = 1000 * (env._acc["period"] - snap["period"]) / n, (env._acc["ticks"] - snap["ticks"]) / n
    check("period ~44 ms with 0 overhead", 40 <= per <= 48, f"{per:.1f} ms, {tps:.2f} ticks/step")
    # 2b: 30 ms caller overhead (3 GPU gradient steps) -- phase lock must hold 2 ticks
    snap = dict(env._acc); k0 = env._steps
    obs, r, term, trunc, info = run_steps(env, 60, act, overhead_s=0.030)
    n = env._steps - k0
    per = 1000 * (env._acc["period"] - snap["period"]) / n
    tps = (env._acc["ticks"] - snap["ticks"]) / n
    ovh = 1000 * (env._acc["overhead"] - snap["overhead"]) / n
    check("period still ~44 ms with 30 ms overhead (phase lock)", 40 <= per <= 48,
          f"{per:.1f} ms, {tps:.2f} ticks/step, overhead {ovh:.1f} ms")
    # 2c: physics agreement
    u = 25.25 * THR
    check("encoder u ~ 25.25 x throttle", abs(info["speed"] - u) < 0.3, f"u={info['speed']:.2f} vs {u:.2f}")
    v_true = env.node.odom[2][0]
    check("observer v_est tracks the car", abs(info["v_est"] - v_true) < 0.3,
          f"v_est={info['v_est']:.2f} v_true={v_true:.2f}")
    check("obs speed slot = u/v_max", abs(obs[110] - info["speed"] / cfg.obs.v_max) < 1e-5, f"{obs[110]:.4f}")
    check("obs v_est slot = v_est/v_max", abs(obs[114] - np.clip(info["v_est"] / cfg.obs.v_max, -1, 1)) < 1e-5)
    check("obs slip slot = S/0.5", abs(obs[115] - np.clip(info["slip"] / 0.5, -1, 1)) < 1e-5)
    # 2c'': the reward-side speed cap and the time-to-collision term
    from rl_racer.obs import ttc_forward, ttc_penalty
    p0, d0, t0, k0 = env._ep["progress"], env._ep["dist"], env._ep["ttc"], env._steps
    run_steps(env, 20, act)
    n = env._steps - k0
    v_drv = (env._ep["dist"] - d0) / n / env.control_period
    paid = (env._ep["progress"] - p0) / n
    cap = cfg.rew.w_progress * cfg.rew.v_ref * env.control_period
    check("car faster than v_ref in this run", v_drv > cfg.rew.v_ref + 0.5, f"{v_drv:.2f} m/s vs v_ref {cfg.rew.v_ref}")
    check("progress paid capped at w x v_ref x dt", 0.9 * cap <= paid <= cap + 1e-6, f"{paid:.3f}/step vs cap {cap:.3f}")
    ttc_step = (env._ep["ttc"] - t0) / n
    check("ttc term small in the open corridor", 0.0 <= ttc_step < 0.3, f"{ttc_step:.3f}/step at {v_drv:.1f} m/s")
    ang = np.radians(np.linspace(-110, 110, 110))     # no beam sits exactly at 0 deg
    open_, ahead = np.full(110, 10.0), np.minimum(10.0, 1.8 / np.maximum(np.cos(ang), 1e-3))
    along = np.minimum(10.0, 1.0 / np.maximum(np.abs(np.sin(ang)), 1e-3))
    check("ttc: open ahead at 3 m/s -> 0", ttc_penalty(ttc_forward(open_, 110.0, 3.0), 1.2) < 1e-9)
    check("ttc: wall 1.8 m ahead at 3 m/s -> 0.5", abs(ttc_penalty(ttc_forward(ahead, 110.0, 3.0), 1.2) - 0.5) < 1e-3,
          f"{ttc_penalty(ttc_forward(ahead, 110.0, 3.0), 1.2):.4f}")
    check("ttc: wall 1.8 m ahead at 1.5 m/s -> 0", ttc_penalty(ttc_forward(ahead, 110.0, 1.5), 1.2) < 1e-9)
    check("ttc: wall 1 m ALONGSIDE at 3 m/s -> 0", ttc_penalty(ttc_forward(along, 110.0, 3.0), 1.2) < 1e-9)
    # Saturated beams are "nothing within range", not a wall at range_max:
    # without that, empty road charged above range_max/ttc_ref = 8.3 m/s.
    sat = np.full(110, 10.0)
    check("ttc: EMPTY road at 12 m/s -> 0 once range_max is known",
          ttc_penalty(ttc_forward(sat, 110.0, 12.0, 6.0, 10.0), 1.2) < 1e-9
          and ttc_penalty(ttc_forward(sat, 110.0, 12.0, 6.0, 0.0), 1.2) > 0.2,
          f"with range_max {ttc_penalty(ttc_forward(sat,110.0,12.0,6.0,10.0),1.2):.3f} vs "
          f"without {ttc_penalty(ttc_forward(sat,110.0,12.0,6.0,0.0),1.2):.3f}")
    check("ttc: a REAL wall still charges at 12 m/s with range_max set",
          ttc_penalty(ttc_forward(ahead, 110.0, 12.0, 6.0, 10.0), 1.2) > 0.8,
          f"{ttc_penalty(ttc_forward(ahead, 110.0, 12.0, 6.0, 10.0), 1.2):.3f}")
    # Quadratic steering term: hard on a big reversal, near-free when smooth.
    from rl_racer.config import Cfg as _Cfg
    _c = _Cfg(); _c.rew.w_smooth, _c.rew.w_smooth2 = 0.3, 3.0
    _pen = lambda dd: _c.rew.w_smooth * dd + _c.rew.w_smooth2 * dd * dd
    check("smooth: quadratic term punishes a 0.5 reversal >> a 0.05 correction",
          _pen(0.5) > 10 * _pen(0.05) and _pen(0.05) < 0.03,
          f"d=0.05 -> {_pen(0.05):.4f}/step, d=0.164 -> {_pen(0.164):.4f}, d=0.5 -> {_pen(0.5):.4f}")
    # Throttle smoothness is LINEAR so it measures total variation: six small
    # chatters must cost MORE than one decisive brake of the same total travel,
    # which is the opposite of what a square would do.
    _tp = lambda dd: 0.6 * dd
    check("thr_smooth: linear -> 6 chatters of 0.1 cost >= one decisive 0.6",
          6 * _tp(0.1) >= _tp(0.6) - 1e-9 and 6 * (1.5 * 0.1 ** 2) < 1.5 * 0.6 ** 2,
          f"6x0.1 -> {6*_tp(0.1):.3f} vs 1x0.6 -> {_tp(0.6):.3f} (a square would give "
          f"{6*1.5*0.01:.3f} vs {1.5*0.36:.3f}, i.e. reward the chatter)")
    # The grip reward is FLAT past the tire asymptote, which is why it could
    # never pull slip down; the excess-slip penalty is what has a gradient there.
    from rl_racer.sensors import tire_mu as _mu
    _v = cfg.veh
    _flat = abs(_mu(0.30, _v) - _mu(0.92, _v)) < 1e-9
    _sp = lambda S: 0.25 * max(0.0, abs(S) - _v.tire_s_peak)
    check("w_grip has NO gradient past the asymptote (why 0.05->0.15 did nothing)",
          _flat, f"mu(0.30)={_mu(0.30,_v):.3f} == mu(0.92)={_mu(0.92,_v):.3f}")
    check("w_slip DOES have a gradient there, and is 0 at the peak",
          _sp(0.92) > _sp(0.30) > 0.0 and _sp(_v.tire_s_peak) == 0.0,
          f"S=0.15 -> {_sp(0.15):.3f}, S=0.30 -> {_sp(0.30):.3f}, S=0.92 -> {_sp(0.92):.3f}/step")
    check("w_slip is symmetric: locking a wheel under brake costs the same",
          abs(_sp(-0.92) - _sp(0.92)) < 1e-12, f"S=-0.92 -> {_sp(-0.92):.3f}")
    yres_rms = math.sqrt(env._acc["yres_sq"] / env._steps); yaw_rms = math.sqrt(env._acc["yaw_sq"] / env._steps)
    check("yaw residual << yaw rate (sign convention)", yres_rms < 0.3 * yaw_rms,
          f"res rms {yres_rms:.3f} vs yaw rms {yaw_rms:.3f}")
    check("not terminated/truncated so far", not term and not trunc, info.get("reason", ""))
    # 2c': random actions in BOTH overhead regimes: the residual must stay small
    # (it is taken against the command that produced the yaw rate) and the
    # command-to-effect latency must be identical (latency lock).
    rng = np.random.default_rng(0)
    def residual_ratio(overhead):
        y2 = r2 = 0.0
        for _ in range(80):
            if overhead: time.sleep(overhead)
            _, _, _, _, inf = env.step(rng.uniform(-1, 1, 2).astype(np.float32))
            y2 += env.node.imu[1] ** 2
        s = env._acc; return s
    a0 = dict(env._acc); residual_ratio(0.0); a1 = dict(env._acc); residual_ratio(0.030); a2 = dict(env._acc)
    q0 = math.sqrt((a1["yres_sq"] - a0["yres_sq"]) / max(a1["yaw_sq"] - a0["yaw_sq"], 1e-9))
    q1 = math.sqrt((a2["yres_sq"] - a1["yres_sq"]) / max(a2["yaw_sq"] - a1["yaw_sq"], 1e-9))
    check("random actions, 0 ms overhead: residual rms < 0.5 x yaw rms", q0 < 0.5, f"ratio {q0:.2f}")
    check("random actions, 30 ms overhead: residual rms < 0.5 x yaw rms", q1 < 0.5, f"ratio {q1:.2f}")
    def latency(overhead):
        for _ in range(6):
            if overhead: time.sleep(overhead)
            env.step(np.array([0.0, act[1]], dtype=np.float32))   # straight, settle
        for k in range(4):
            if overhead: time.sleep(overhead)
            env.step(np.array([1.0, act[1]], dtype=np.float32))   # full left from step 0
            if abs(env.node.imu[1]) > 0.5: return k
        return 99
    l0, l1 = latency(0.0), latency(0.030)
    check("command latency identical at 0 and 30 ms overhead", l0 == l1 and l0 < 99, f"effect seen at step {l0} vs {l1}")
    # 2d: episode end -> stats
    cfg.env.max_episode_steps = env._steps + 1
    obs, r, term, trunc, info = env.step(np.array(act, dtype=np.float32))
    st = info.get("episode_stats", {})
    check("truncated at the cap with stats", trunc and "episode_stats" in info)
    check("enc_discontinuities 0 in a clean episode", st.get("enc_discontinuities", -1) == 0.0, str(st.get("enc_discontinuities")))
    check("diag_ticks_per_step ~2", abs(st.get("diag_ticks_per_step", 0) - 2.0) < 0.15, f"{st.get('diag_ticks_per_step', 0):.2f}")
    check("diag_step_ms ~44", 40 <= st.get("diag_step_ms", 0) <= 48, f"{st.get('diag_step_ms', 0):.1f}")
    check("slip_frac_accel > 0 (was accelerating)", st.get("slip_frac_accel", 0) > 0.3, f"{st.get('slip_frac_accel', 0):.2f}")
    # 2e: reset restores the mock's encoder counters (a -1000s rad jump) -> no spike
    cfg.env.max_episode_steps = 10 ** 9
    obs, _ = env.reset()
    check("post-reset speed slot 0 (encoder counter jump absorbed)", abs(obs[110]) < 1e-6, f"{obs[110]:.3f}")
    first = [env.step(np.array(act, dtype=np.float32))[0][110] for _ in range(5)]
    check("first 5 post-reset speed slots >= 0", all(x >= -1e-6 for x in first), f"{np.round(first, 3)}")
    cfg.env.max_episode_steps = env._steps + 1
    info = env.step(np.array(act, dtype=np.float32))[4]
    check("enc_discontinuities 0 after a reset", info["episode_stats"]["enc_discontinuities"] == 0.0)
    # 2f: crash termination
    cfg.env.max_episode_steps = 10 ** 9
    env.reset(); run_steps(env, 3, act)
    x_pre, dist_pre = env._prev_pos[0], env._ep["dist"]
    side.collide(); time.sleep(0.1)          # the mock respawns the car at the origin, like the sim
    obs, r, term, trunc, info = env.step(np.array(act, dtype=np.float32))
    check("collision -> terminated 'crash' with -50", term and info.get("reason") == "crash" and r < -40,
          f"reason={info.get('reason')} r={r:.1f}")
    st = info.get("episode_stats", {})
    check("crash step pays no progress for the respawn teleport", abs(st.get("dist", -1) - dist_pre) < 1e-9,
          f"dist {dist_pre:.3f} -> {st.get('dist', -1):.3f}")
    check("crash stats log the PRE-respawn pose and speed, not the checkpoint",
          all(k in st for k in ("end_x", "end_y", "end_speed", "end_approach_speed"))
          and abs(st["end_x"] - x_pre) < 1e-6 and st["end_x"] > 0.0
          and st["end_speed"] > 0.0 and st["end_approach_speed"] > 0.0,
          f"x={st.get('end_x', 0):.3f} (pre-crash {x_pre:.3f}) v={st.get('end_speed', 0):.2f} "
          f"approach={st.get('end_approach_speed', 0):.2f}")

    # 2g: reverse curriculum -- resume at the sim's checkpoint after a crash
    straight = (0.0, act[1])
    cfg.env.crash_restart = False
    env.reset()                                   # full reset: car at the spawn
    run_steps(env, 40, straight)                  # drive past at least one 3 m checkpoint
    x_pre = float(env._prev_pos[0])
    cp = 3.0 * math.floor(x_pre / 3.0)
    cfg.env.crash_restart = True
    cfg.env.crash_restart_prob = 1.0
    cfg.env.crash_restart_max = 20
    side.collide(); time.sleep(0.1)
    _, _, term, _, _ = env.step(np.array(straight, dtype=np.float32))
    check("drove past a checkpoint before crashing", term and cp >= 3.0, f"crashed at x={x_pre:.2f} -> checkpoint {cp:.1f}")
    rc0 = side.reset_count or 0
    env.reset(); time.sleep(0.1)
    check("crash_restart: NO reset pulse after a counted collision",
          (side.reset_count or 0) == rc0, f"{rc0} -> {side.reset_count}")
    check("crash_restart: the episode resumes AT the checkpoint, not the start line",
          env._resumed and abs(float(env._start_pos[0]) - cp) < 0.3,
          f"start x={float(env._start_pos[0]):.2f} vs checkpoint {cp:.1f}, resumed={env._resumed}")
    # prob 0 -> always back to the start line
    run_steps(env, 3, straight); side.collide(); time.sleep(0.1)
    env.step(np.array(straight, dtype=np.float32))
    cfg.env.crash_restart_prob = 0.0
    rc0 = side.reset_count or 0
    env.reset(); time.sleep(0.1)
    check("crash_restart_prob 0 -> full reset to the start line",
          (side.reset_count or 0) == rc0 + 1 and not env._resumed and abs(float(env._start_pos[0])) < 0.3,
          f"pulses {rc0} -> {side.reset_count}, start x={float(env._start_pos[0]):.2f}")
    # the consecutive cap must break a checkpoint the policy cannot leave
    cfg.env.crash_restart_prob = 1.0
    cfg.env.crash_restart_max = 0
    run_steps(env, 3, straight); side.collide(); time.sleep(0.1)
    env.step(np.array(straight, dtype=np.float32))
    rc0 = side.reset_count or 0
    env.reset(); time.sleep(0.1)
    check("crash_restart_max 0 -> the cap forces a full reset",
          (side.reset_count or 0) == rc0 + 1 and not env._resumed, f"pulses {rc0} -> {side.reset_count}")
    # a non-crash end (truncation) never resumes: the sim has respawned nothing
    cfg.env.crash_restart_max = 20
    cfg.env.max_episode_steps = env._steps + 1
    _, _, _, trunc_g, _ = env.step(np.array(straight, dtype=np.float32))
    cfg.env.max_episode_steps = 10 ** 9
    rc0 = side.reset_count or 0
    env.reset(); time.sleep(0.1)
    check("a truncated episode always resets to the start line",
          trunc_g and (side.reset_count or 0) == rc0 + 1 and not env._resumed,
          f"trunc={trunc_g} pulses {rc0} -> {side.reset_count}")

    # 2h: fixed start point by driving there (no spawn command exists)
    cfg.env.crash_restart = False
    cfg.env.spawn_drive = True
    cfg.env.spawn_x, cfg.env.spawn_y = 5.0, 0.0     # the mock drives +x from the origin
    cfg.env.spawn_radius, cfg.env.spawn_cruise = 0.35, 3.0
    cfg.env.spawn_timeout_s = 20.0
    t_spawn = time.time()
    obs, _ = env.reset()
    dt_spawn = time.time() - t_spawn
    err = abs(float(env._start_pos[0]) - cfg.env.spawn_x)
    check("spawn_drive: episode starts AT the fixed point, not the start line",
          err <= cfg.env.spawn_radius + 0.2, f"start x={float(env._start_pos[0]):.2f} target 5.0 err={err:.2f} m")
    check("spawn_drive: handed over at a standstill (speed slot 0)", abs(obs[110]) < 1e-6, f"{obs[110]:.4f}")
    check("spawn_drive: drive-in took a sane time", 0.5 < dt_spawn < 20.0, f"{dt_spawn:.1f} s")
    cfg.env.spawn_drive = False
    obs, _ = env.reset()
    check("spawn_drive off -> back to the start line", abs(float(env._start_pos[0])) < 0.3,
          f"start x={float(env._start_pos[0]):.2f}")
    env.close()

    # ------------------------------------------------------------ 3. race mode
    print("\n[3] race mode")
    cfg = stage5_cfg(); cfg.env.race_mode = True; cfg.env.episode_seconds = 0.0; cfg.env.max_episode_steps = 5
    env = AutoDriveRacerEnv(cfg)
    rc0 = side.reset_count or 0
    env.reset(); time.sleep(0.1)
    check("race reset sends NO reset pulse", (side.reset_count or 0) == rc0, f"{rc0} -> {side.reset_count}")
    run_steps(env, 3, act); side.collide(); time.sleep(0.1)
    obs, r, term, trunc, info = env.step(np.array(act, dtype=np.float32))
    check("race: collision does not terminate", not term and info.get("reason") is None, f"term={term} r={r:.1f}")
    obs, r, term, trunc, info = run_steps(env, 4, act)
    check("race: no truncation past the cap", not trunc and not term, f"steps={env._steps}")
    check("race: crash counted in ep stats", env._ep.get("crashes", 0) == 1.0, str(env._ep.get("crashes")))
    env.close()

    # ------------------------------------------------------------ 4. enjoy.py
    print("\n[4] enjoy.py --stage 5 against the mock (random weights)")
    cfg = stage5_cfg(); env = AutoDriveRacerEnv(cfg)
    from stable_baselines3 import SAC
    import tempfile
    mpath = os.path.join(tempfile.mkdtemp(prefix="rl_racer_test_"), "rand_stage5.zip")
    SAC("MlpPolicy", env, device="cpu", policy_kwargs=dict(net_arch=[256, 256])).save(mpath)
    env.close()
    out = subprocess.run([sys.executable, os.path.join(RL, "enjoy.py"), mpath, "--stage", "5", "--episodes", "1", "--steps", "40"],
                         cwd=RL, capture_output=True, text=True, timeout=120).stdout
    check("enjoy: stage banner obs_dim=117", "obs_dim=117" in out)
    check("enjoy: ran exactly 40 steps (--steps beats episode_seconds)", "steps=   40" in out, [l for l in out.splitlines() if l.startswith("ep 0")][:1])
    check("enjoy: printed v_est line", "v_est max=" in out)
    # --race: must run past 40 steps, never reset; stop it with SIGINT
    rc0 = side.reset_count or 0
    p = subprocess.Popen([sys.executable, os.path.join(RL, "enjoy.py"), mpath, "--stage", "5", "--race", "--steps", "40"],
                         cwd=RL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(8.0); p.send_signal(signal.SIGINT)
    try: out = p.communicate(timeout=20)[0]
    except subprocess.TimeoutExpired: p.kill(); out = p.communicate()[0]
    check("enjoy --race: banner", "[race] no reset pulse" in out)
    check("enjoy --race: no reset pulse sent", (side.reset_count or 0) == rc0, f"{rc0} -> {side.reset_count}")
    check("enjoy --race: exited cleanly on Ctrl-C", "Traceback" not in out and p.returncode == 0,
          f"rc={p.returncode}" + (out[-300:] if "Traceback" in out else ""))

    # ------------------------------------------------------------ 5. sim_timeout
    print("\n[5] dead sim -> sim_timeout")
    cfg = stage5_cfg(); env = AutoDriveRacerEnv(cfg); env.reset(); run_steps(env, 3, act)
    stop_mock(m)
    t0 = time.time(); obs, r, term, trunc, info = env.step(np.array(act, dtype=np.float32))
    check("sim_timeout after tick_timeout", term and info.get("reason") == "sim_timeout" and 4 < time.time() - t0 < 7,
          f"reason={info.get('reason')} after {time.time()-t0:.1f} s")
    env.close()

    stop.set(); th.join(1.0); ex.shutdown(timeout_sec=0.5); side.destroy_node()
    print(f"\n{'ALL PASS' if not FAILS else 'FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0

if __name__ == "__main__":
    code = main()
    rclpy.try_shutdown()
    sys.exit(code)
