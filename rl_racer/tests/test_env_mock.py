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
    side.collide(); time.sleep(0.1)
    obs, r, term, trunc, info = env.step(np.array(act, dtype=np.float32))
    check("collision -> terminated 'crash' with -15", term and info.get("reason") == "crash" and r < -10,
          f"reason={info.get('reason')} r={r:.1f}")
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
