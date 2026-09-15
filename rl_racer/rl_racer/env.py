"""Gymnasium environment wrapping the AutoDRIVE RoboRacer simulator over ROS 2.

The simulator is a free-running real-time process, so step() is made
synchronous by blocking on the LiDAR topic: the scan is the master clock. Every
sim tick the bridge publishes all topics together inside its 'Bridge' socket.io
handler, so counting scans counts sim ticks.

COMPETITION LEGALITY -- what feeds the OBSERVATION vs the REWARD:
  observation: /lidar, /left_encoder + /right_encoder, /imu, our own commands,
               and the simulator's published vehicle model  <- all permitted
  reward / episode management only: /odom (progress, speed), /collision_count,
               /lap_count, /last_lap_time, /reset_command  <- restricted at
               race time, but training-only; none of it exists at inference.
"""
import math
import threading
import time

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)
from std_msgs.msg import Float32, Bool, Int32
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry

from .config import Cfg
from .obs import LidarFOV, beam_features, build_obs, yaw_from_quat
from .sensors import WheelSpeed, TireObserver, slip, tire_mu, yaw_residual

NS = "/autodrive/roboracer_1"

# Must match the bridge exactly (autodrive_bridge.py: RELIABLE/VOLATILE/KEEP_LAST/1).
# A mismatch here connects but silently delivers nothing.
QOS = QoSProfile(
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class _RacerNode(Node):
    def __init__(self, fov: LidarFOV, wheels: WheelSpeed):
        super().__init__("rl_racer_env")
        self._fov = fov
        self.cv = threading.Condition()
        self.tick = 0
        self.beams = None
        self.odom = None          # (pos xyz, quat xyzw, lin xyz, ang xyz) -- REWARD ONLY
        self.imu = None           # (quat xyzw, yaw_rate)   -- permitted topic
        self.wheels = wheels      # encoder -> wheel speed  -- permitted topic
        self.collisions = 0
        self.laps = 0
        self.last_lap_time = 0.0

        self.create_subscription(LaserScan, f"{NS}/lidar", self._cb_lidar, QOS)
        self.create_subscription(Odometry, f"{NS}/odom", self._cb_odom, QOS)
        self.create_subscription(Imu, f"{NS}/imu", self._cb_imu, QOS)
        self.create_subscription(JointState, f"{NS}/left_encoder",
                                 lambda m: self._cb_enc("l", m), QOS)
        self.create_subscription(JointState, f"{NS}/right_encoder",
                                 lambda m: self._cb_enc("r", m), QOS)
        self.create_subscription(Int32, f"{NS}/collision_count", self._cb_col, QOS)
        self.create_subscription(Int32, f"{NS}/lap_count", self._cb_lap, QOS)
        self.create_subscription(Float32, f"{NS}/last_lap_time", self._cb_lap_time, QOS)

        self.pub_throttle = self.create_publisher(Float32, f"{NS}/throttle_command", QOS)
        self.pub_steering = self.create_publisher(Float32, f"{NS}/steering_command", QOS)
        self.pub_reset = self.create_publisher(Bool, "/autodrive/reset_command", QOS)

    def _cb_lidar(self, msg):
        if not self._fov.ready:
            self._fov.configure(msg.angle_min, msg.angle_increment, len(msg.ranges))
        beams = self._fov.process(msg.ranges)
        with self.cv:
            self.beams = beams
            self.tick += 1
            self.cv.notify_all()

    def _cb_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
        # Single atomic rebind; readers never see a torn tuple.
        self.odom = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                     (lin.x, lin.y, lin.z), (ang.x, ang.y, ang.z))

    def _cb_imu(self, msg):
        q, w = msg.orientation, msg.angular_velocity
        self.imu = ((q.x, q.y, q.z, q.w), w.z)

    def _cb_enc(self, side, msg):
        if not msg.position:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.cv:
            self.wheels.update(side, float(msg.position[0]), t)

    def _cb_col(self, msg):
        self.collisions = int(msg.data)

    def _cb_lap(self, msg):
        self.laps = int(msg.data)

    def _cb_lap_time(self, msg):
        self.last_lap_time = float(msg.data)

    @property
    def enc_speed(self) -> float:
        with self.cv:
            return self.wheels.speed

    def reset_sensors(self):
        with self.cv:
            self.wheels.reset()

    def send(self, throttle: float, steering: float):
        t, s = Float32(), Float32()
        t.data, s.data = float(throttle), float(steering)
        self.pub_throttle.publish(t)
        self.pub_steering.publish(s)

    def send_reset(self, flag: bool):
        b = Bool()
        b.data = bool(flag)
        self.pub_reset.publish(b)

    def wait_ticks(self, n: int, timeout: float) -> bool:
        with self.cv:
            target = self.tick + n
            return self.cv.wait_for(lambda: self.tick >= target, timeout=timeout)

    def wait_until(self, target: int, timeout: float) -> bool:
        with self.cv:
            return self.cv.wait_for(lambda: self.tick >= target, timeout=timeout)


class AutoDriveRacerEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, cfg: Cfg | None = None):
        super().__init__()
        self.cfg = cfg or Cfg()
        o = self.cfg.obs

        self.observation_space = spaces.Box(-1.0, 1.0, (o.dim,), dtype=np.float32)
        # [steering, throttle], both in [-1, 1]; scaled to sim units on publish.
        self.action_space = spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)

        self._fov = LidarFOV(o.fov_half_deg, o.n_beams, o.range_max)
        self._wheels = WheelSpeed(o.wheel_radius, window=o.enc_window, step_max=o.enc_step_max)
        self._observer = TireObserver(self.cfg.veh)
        self._obs_t = None
        if not rclpy.ok():
            rclpy.init()
        self.node = _RacerNode(self._fov, self._wheels)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        # Own spin loop with a stop flag: Executor.spin() cannot be stopped
        # from close() (after shutdown() it busy-loops until the context dies),
        # and a spin thread still alive at interpreter exit races rclpy's
        # teardown -- a traceback on Ctrl-C and, at times, a segfault.
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

        if not self.node.wait_ticks(1, self.cfg.env.startup_timeout):
            raise RuntimeError(
                "No LiDAR received on %s/lidar within %.0fs.\n"
                "Start the simulator and the bridge first:\n"
                "  ./AutoDRIVE\\ Simulator.x86_64 -ip 127.0.0.1 -port 4567\n"
                "  ros2 launch racer_bringup bridge.launch.py tcp_nodelay:=true loop_hz_cap:=45"
                % (NS, self.cfg.env.startup_timeout)
            )

        # Measure the real control period. gamma, the episode length and the
        # curriculum cooldown are all derived from it so they stay fixed in
        # SECONDS whatever rate this machine (or the evaluation box) delivers.
        _n0, _t0 = self.node.tick, time.time()
        self.node.wait_ticks(20, self.cfg.env.tick_timeout * 4)
        _dt = (time.time() - _t0) / max(self.node.tick - _n0, 1)
        self.control_period = float(_dt * self.cfg.env.decimation)
        print(f"[rl_racer] measured control period {self.control_period*1000:.1f} ms "
              f"({1.0/max(self.control_period,1e-6):.1f} Hz) = sim tick "
              f"{_dt*1000:.1f} ms x decimation {self.cfg.env.decimation}")
        _tick_ms = _dt * 1000.0
        lo, hi = self.cfg.env.tick_ms_min, self.cfg.env.tick_ms_max
        if (lo > 0 and _tick_ms < lo) or (hi > 0 and _tick_ms > hi):
            self.close()
            raise RuntimeError(
                f"sim tick {_tick_ms:.1f} ms is outside the {lo:g}-{hi:g} ms window this "
                f"stage was designed for. Bring the bridge up pinned to the evaluation rate:\n"
                f"  ros2 launch racer_bringup bridge.launch.py tcp_nodelay:=true loop_hz_cap:=45\n"
                f"(too slow = stock bridge without the shim; too fast = shim without the cap)")
        if self.cfg.env.episode_seconds > 0:
            self.cfg.env.max_episode_steps = int(round(
                self.cfg.env.episode_seconds / self.control_period))
            print(f"[rl_racer] max_episode_steps = {self.cfg.env.max_episode_steps} "
                  f"({self.cfg.env.episode_seconds:g} s at this rate)")

        self._prev_action = np.zeros(2, dtype=np.float32)
        self._prev_pos = None
        self._col_base = 0
        self._lap_base = 0
        self._steps = 0
        self._stalled = 0
        self._too_close = 0
        self._prev_laps = 0
        # Full key sets from the start, so a step() before reset() cannot KeyError.
        self._ep = {k: 0.0 for k in self._EP_KEYS}
        self._acc = {k: 0.0 for k in self._ACC_KEYS}
        self._ep_max_speed = 0.0
        self._ep_max_vest = 0.0
        # Phase lock: the tick the last observation was taken at. step() waits
        # for `decimation` ticks past THAT tick, not past the send, so the
        # caller's overhead (policy + gradient steps) is absorbed into the
        # window instead of extending it. Counting from the send made the
        # period tick x (decimation + floor(overhead / tick)): 3 ticks, not 2,
        # as soon as 3 gradient steps (~30 ms) outlasted one 22 ms tick.
        #
        # Latency lock (decimation >= 2): the command is sent no earlier than
        # the tick AFTER the observation. The bridge forwards a command in its
        # reply to the next telemetry, so a command sent 2 ms after the
        # observation (deployment) rides tick 1's reply and shows in this
        # step's observation, while one sent 30 ms after it (training, three
        # gradient steps) rides tick 2's and shows in the NEXT step's. Holding
        # the send past tick 1 makes it "next step" in both. Costs one tick
        # of latency at deployment; buys train == deploy dynamics.
        self._obs_tick = 0
        self._obs_wall = None       # perf_counter at the last observation

    def _spin(self):
        while not self._stop.is_set() and rclpy.ok():
            try:
                self._exec.spin_once(timeout_sec=0.05)
            except Exception:       # ExternalShutdownException at exit
                break

    _EP_KEYS = ("progress", "speed", "center", "smooth", "prox", "dist", "lap", "grip")
    _ACC_KEYS = ("thr", "steer", "sat", "vest", "slip_abs", "peak", "accel", "yres_sq", "yaw_sq",
                 "period", "overhead", "ticks")

    # ---------------------------------------------------------------- helpers
    def _scale(self, action):
        a = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        steering = float(a[0]) * self.cfg.act.max_steer
        lo, hi = self.cfg.act.throttle_min, self.cfg.act.throttle_max
        throttle = lo + (float(a[1]) + 1.0) * 0.5 * (hi - lo)
        return a, throttle, steering

    def _state(self):
        """Returns (beams, position, forward_vec, speed, yaw_rate).

        position is /odom and is used by the REWARD only. speed is the wheel
        surface speed u from the encoders (the throttle echo, see sensors.py).
        """
        with self.node.cv:
            beams = self.node.beams

        imu = self.node.imu
        if imu is not None:
            quat, yaw_rate = imu
        else:
            quat, yaw_rate = (0.0, 0.0, 0.0, 1.0), 0.0
        yaw = yaw_from_quat(*quat)
        fwd = (math.cos(yaw), math.sin(yaw))

        odom = self.node.odom
        pos = np.array(odom[0][:2]) if odom is not None else np.zeros(2)

        if self.cfg.obs.use_encoder_speed:
            speed = self.node.enc_speed
        else:
            # Legacy path: speed from /odom twist (RESTRICTED at race time).
            lin = odom[2] if odom is not None else (0.0, 0.0, 0.0)
            speed = math.hypot(lin[0], lin[1])
            if self.cfg.env.legacy_speed_sign:
                if lin[0] * fwd[0] + lin[1] * fwd[1] < 0.0:   # original (buggy)
                    speed = -speed
            elif lin[0] < 0.0:                                # corrected
                speed = -speed

        return beams, pos, fwd, speed, yaw_rate

    def _slip_features(self, u: float, yaw_rate: float, steer_cmd: float):
        """Advance the race-legal speed observer and derive the slip triple."""
        now = time.perf_counter()
        if self._obs_t is not None:
            self._observer.update(u, now - self._obs_t)
        self._obs_t = now
        v_est = self._observer.v
        s = slip(u, v_est, self.cfg.veh)
        yres = yaw_residual(yaw_rate, v_est, steer_cmd, self.cfg.veh)
        return v_est, s, yres

    def _build(self, beams, speed, yaw_rate, a0, a1, slip_triple):
        o = self.cfg.obs
        return build_obs(beams, o.range_max, speed, o.v_max, yaw_rate, o.yaw_rate_max,
                         a0, a1, self.cfg.act.throttle_max,
                         slip=slip_triple if o.slip_slots else None,
                         slip_max=o.slip_max, yaw_res_max=o.yaw_res_max)

    # ------------------------------------------------------------------ gym
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        env = self.cfg.env

        self.node.send(0.0, 0.0)
        if not env.race_mode:
            # reset_command is level-triggered: the bridge re-emits it as
            # 'V1 Reset' every tick, so it must be pulsed or the sim resets
            # forever.
            self.node.send_reset(True)
            self.node.wait_ticks(env.reset_pulse_ticks, env.tick_timeout)
            self.node.send_reset(False)
            time.sleep(env.reset_settle_s)
        self.node.wait_ticks(1, env.tick_timeout)
        with self.node.cv:
            self._obs_tick = self.node.tick
        self._obs_wall = time.perf_counter()

        # The simulator's ResetManager restores the encoder counters to their
        # spawn values, so the pre-reset samples describe a different counter.
        # Clear the sensor state AFTER the settle so the first episode step
        # starts from post-reset samples only; the car is stationary, so a speed
        # of 0 for the first tick or two is correct. The observer restarts at 0.
        # The counter jump itself is expected, so it is not a "discontinuity"
        # worth reporting: the stat counts only mid-episode ones.
        self.node.reset_sensors()
        self._wheels.discontinuities = 0
        self._observer.reset(0.0)
        self._obs_t = None

        beams, pos, _, speed, yaw_rate = self._state()
        v_est, s, yres = self._slip_features(speed, yaw_rate, 0.0)
        # Re-baseline counters: works whether or not the sim zeroes them itself.
        self._col_base = self.node.collisions
        self._lap_base = self.node.laps
        self._prev_pos = pos
        self._prev_action[:] = 0.0
        self._steps = 0
        self._stalled = 0
        self._too_close = 0
        self._ep = {k: 0.0 for k in self._EP_KEYS}
        self._acc = {k: 0.0 for k in self._ACC_KEYS}
        self._prev_laps = self.node.laps
        self._ep_max_speed = 0.0
        self._ep_max_vest = 0.0

        return self._build(beams, speed, yaw_rate, 0.0, 0.0, (v_est, s, yres)), {}

    def step(self, action):
        cfg, rw, veh = self.cfg, self.cfg.rew, self.cfg.veh
        a, throttle, steering = self._scale(action)

        _t_send = time.perf_counter()          # caller overhead ends here
        fresh = True
        if cfg.env.decimation >= 2:
            fresh = self.node.wait_until(self._obs_tick + 1, cfg.env.tick_timeout)
        with self.node.cv:
            _sent_tick = self.node.tick
        self.node.send(throttle, steering)
        # Phase-locked: `decimation` ticks past the LAST observation, and at
        # least one past the send so the observation postdates the command.
        target = max(self._obs_tick + cfg.env.decimation, _sent_tick + 1)
        fresh = fresh and self.node.wait_until(target, cfg.env.tick_timeout)
        with self.node.cv:
            _now_tick = self.node.tick
        _t_obs = time.perf_counter()
        if self._obs_wall is not None:
            self._acc["period"] += _t_obs - self._obs_wall
            self._acc["overhead"] += _t_send - self._obs_wall
            self._acc["ticks"] += _now_tick - self._obs_tick
        self._obs_tick, self._obs_wall = _now_tick, _t_obs

        beams, pos, fwd, speed, yaw_rate = self._state()
        feats = beam_features(beams, cfg.obs.range_max, rw.safe_dist)
        # The yaw rate in this observation was produced by the PREVIOUS
        # command (see the latency lock), so the residual is taken against
        # that one. Using the command just sent made the residual ~ the yaw
        # rate itself under random exploration.
        v_est, s, yres = self._slip_features(
            speed, yaw_rate, float(self._prev_action[0]) * cfg.act.max_steer)

        # Forward progress = displacement projected on heading. Clamped because
        # a reset teleport would otherwise inject a huge spurious reward.
        d = pos - self._prev_pos
        ds = float(np.clip(d[0] * fwd[0] + d[1] * fwd[1], -1.0, 1.0))
        self._prev_pos = pos

        r_progress = rw.w_progress * ds
        # Speed reward from /odom (training-only, accurate) -- NOT the encoder,
        # which overreads under wheelspin and would pay the agent to spin.
        _od = self.node.odom
        _rew_speed = math.hypot(_od[2][0], _od[2][1]) if _od is not None else abs(speed)
        r_speed = rw.w_speed * float(np.clip(_rew_speed / cfg.obs.v_max, 0.0, 1.0))
        # Grip utilisation: mu(|S|)/mu_peak, 1.0 at the tire's peak, 0.64 at the
        # asymptote (wheelspin / lock), ~0 when coasting.
        r_grip = rw.w_grip * (tire_mu(s, veh) / veh.tire_mu_peak) if rw.w_grip > 0.0 else 0.0
        p_center = rw.w_center * feats["center_err"]
        p_smooth = rw.w_smooth * abs(float(a[0]) - float(self._prev_action[0]))
        p_prox = rw.w_prox * feats["prox"]

        # Lap bonus from the simulator's own timer, once per completed lap.
        r_lap = 0.0
        laps_now = self.node.laps
        if rw.w_lap > 0.0 and laps_now > self._prev_laps:
            n_new = laps_now - self._prev_laps
            lt = self.node.last_lap_time
            if lt and lt > 0.0:
                r_lap = n_new * rw.w_lap / max(lt, rw.min_lap_time)
            else:
                r_lap = n_new * rw.lap_bonus_flat
        if laps_now > self._prev_laps:
            self._prev_laps = laps_now

        reward = (r_progress + r_speed + r_lap + r_grip
                  - p_center - p_smooth - p_prox - rw.step_penalty)

        ep, acc = self._ep, self._acc
        ep["progress"] += r_progress; ep["speed"] += r_speed; ep["center"] += p_center
        ep["smooth"] += p_smooth;     ep["prox"] += p_prox;    ep["dist"] += ds
        ep["lap"] += r_lap;           ep["grip"] += r_grip
        acc["thr"] += throttle
        acc["steer"] += abs(float(a[0]))
        acc["sat"] += 1.0 if float(a[1]) > 0.9 else 0.0
        acc["vest"] += v_est
        acc["slip_abs"] += abs(s)
        # "peak grip" = |S| in the [0.10, 0.20] band around the extremum
        acc["peak"] += 1.0 if 0.10 <= abs(s) <= 0.20 else 0.0
        acc["accel"] += 1.0 if s > 0.02 else 0.0
        acc["yres_sq"] += yres * yres
        acc["yaw_sq"] += yaw_rate * yaw_rate
        self._ep_max_speed = max(self._ep_max_speed, speed)
        self._ep_max_vest = max(self._ep_max_vest, v_est)

        terminated = False
        reason = ""
        if self.node.collisions > self._col_base:
            reward -= rw.crash_penalty
            terminated, reason = True, "crash"
            self._col_base = self.node.collisions   # in race mode: charge once, keep driving

        # Backup detector for sims that do not count wall scrapes.
        self._too_close = (self._too_close + 1
                           if feats["min_range"] < rw.crash_dist else 0)
        if not terminated and self._too_close >= rw.crash_dist_steps:
            reward -= rw.crash_penalty
            terminated, reason = True, "scrape"

        self._stalled = self._stalled + 1 if abs(speed) < rw.stall_speed else 0
        if self._stalled >= rw.stall_steps:
            reward -= rw.stall_penalty
            terminated, reason = True, "stall"

        if cfg.env.race_mode and terminated:
            # A collision costs 10 s at the race, it does not end it. Only a
            # dead sim (below) may end a race-mode episode.
            self._ep["crashes"] = self._ep.get("crashes", 0.0) + 1.0
            terminated, reason = False, ""

        if not fresh:
            # Sim stopped ticking (paused, crashed, or disconnected). End the
            # episode rather than training on stale observations.
            terminated, reason = True, "sim_timeout"

        self._steps += 1
        truncated = (self._steps >= cfg.env.max_episode_steps) and not cfg.env.race_mode
        self._prev_action[:] = a

        obs = self._build(beams, speed, yaw_rate, a[0], a[1], (v_est, s, yres))

        info = {"speed": speed, "v_est": v_est, "slip": s,
                "min_range": feats["min_range"],
                "laps": self.node.laps - self._lap_base,
                "throttle_cap": cfg.act.throttle_max}
        if terminated or truncated:
            info["reason"] = reason or "timeout"
            stats = dict(ep)
            n = max(1, self._steps)
            stats["act_throttle_mean"] = acc["thr"] / n
            stats["act_steer_abs_mean"] = acc["steer"] / n
            stats["act_throttle_sat"] = acc["sat"] / n
            stats["max_speed"] = self._ep_max_speed          # encoder u
            stats["slip_v_est_max"] = self._ep_max_vest      # observer car speed
            stats["slip_v_est_mean"] = acc["vest"] / n
            stats["slip_abs_mean"] = acc["slip_abs"] / n
            stats["slip_frac_peak"] = acc["peak"] / n
            stats["slip_frac_accel"] = acc["accel"] / n
            stats["slip_yaw_res_rms"] = math.sqrt(acc["yres_sq"] / n)
            stats["slip_yaw_rate_rms"] = math.sqrt(acc["yaw_sq"] / n)
            stats["enc_discontinuities"] = float(self._wheels.discontinuities)
            self._wheels.discontinuities = 0
            # Effective control period as driven, vs the one gamma and the
            # episode length were derived from at startup. `overhead` is what
            # the caller spent between observation and next command (policy
            # + gradient steps); it must stay under the window or the period
            # grows in whole ticks.
            stats["diag_step_ms"] = 1000.0 * acc["period"] / n
            stats["diag_overhead_ms"] = 1000.0 * acc["overhead"] / n
            stats["diag_ticks_per_step"] = acc["ticks"] / n
            info["episode_stats"] = stats
            self.node.send(0.0, 0.0)   # never leave the throttle open
        return obs, float(reward), terminated, truncated, info

    def close(self):
        try:
            self.node.send(0.0, 0.0)    # never leave the throttle open
        except Exception:
            pass
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._exec.shutdown(timeout_sec=0.5)
            self.node.destroy_node()
        except Exception:
            pass
