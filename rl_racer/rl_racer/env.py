"""Gymnasium environment wrapping the AutoDRIVE RoboRacer simulator over ROS 2.

The simulator is a free-running real-time process, so step() is made
synchronous by blocking on the LiDAR topic: the scan is the master clock. Every
sim tick the bridge publishes all topics together inside its 'Bridge' socket.io
handler, so counting scans counts sim ticks.
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
    def __init__(self, fov: LidarFOV, wheel_radius: float = 0.0590):
        super().__init__("rl_racer_env")
        self._fov = fov
        self.cv = threading.Condition()
        self.tick = 0
        self.beams = None
        self.odom = None          # (pos xyz, quat xyzw, lin xyz, ang xyz)
        self.imu = None           # (quat xyzw, yaw_rate)   -- permitted topic
        self._enc = {}            # wheel -> (angle_rad, stamp_s)
        self.enc_speed = 0.0      # m/s from wheel encoders -- permitted topic
        self._enc_rate = {}
        self._wheel_r = wheel_radius
        self.collisions = 0
        self.laps = 0
        self.last_lap_time = 0.0

        self.create_subscription(LaserScan, f"{NS}/lidar", self._cb_lidar, QOS)
        # /odom: TRAINING ONLY (progress reward). Never feeds the observation.
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
        """Wheel speed from encoder deltas. position is wheel angle in RADIANS
        (measured; the Technical Guide's 'ticks' figure is wrong). The delta is
        naturally signed, so this also gives forward/reverse for free."""
        if not msg.position:
            return
        ang = float(msg.position[0])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        prev = self._enc.get(side)
        self._enc[side] = (ang, t)
        if prev is None:
            return
        dt = t - prev[1]
        if dt <= 1e-4 or dt > 0.5:
            return
        rate = (ang - prev[0]) / dt                 # rad/s
        other = self._enc_rate.get("r" if side == "l" else "l")
        self._enc_rate[side] = rate
        rates = [v for v in self._enc_rate.values() if v is not None]
        self.enc_speed = float(np.mean(rates)) * self._wheel_r

    def _cb_col(self, msg):
        self.collisions = int(msg.data)

    def _cb_lap(self, msg):
        self.laps = int(msg.data)

    def _cb_lap_time(self, msg):
        self.last_lap_time = float(msg.data)

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


class AutoDriveRacerEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, cfg: Cfg | None = None):
        super().__init__()
        self.cfg = cfg or Cfg()
        o, a = self.cfg.obs, self.cfg.act

        self.observation_space = spaces.Box(-1.0, 1.0, (o.dim,), dtype=np.float32)
        # [steering, throttle], both in [-1, 1]; scaled to sim units on publish.
        self.action_space = spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)

        self._fov = LidarFOV(o.fov_half_deg, o.n_beams, o.range_max)
        if not rclpy.ok():
            rclpy.init()
        self.node = _RacerNode(self._fov, self.cfg.obs.wheel_radius)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

        if not self.node.wait_ticks(1, self.cfg.env.startup_timeout):
            raise RuntimeError(
                "No LiDAR received on %s/lidar within %.0fs.\n"
                "Start the simulator and the bridge first:\n"
                "  ./AutoDRIVE\\ Simulator.x86_64 -batchmode -nographics -ip 127.0.0.1 -port 4567\n"
                "  ros2 launch autodrive_roboracer bringup_headless.launch.py"
                % (NS, self.cfg.env.startup_timeout)
            )

        # Measure the real control period so gamma can be derived from it.
        import time as _t
        _n0, _t0 = self.node.tick, _t.time()
        self.node.wait_ticks(20, self.cfg.env.tick_timeout * 4)
        _dt = (_t.time() - _t0) / max(self.node.tick - _n0, 1)
        self.control_period = float(_dt * self.cfg.env.decimation)
        print(f"[rl_racer] measured control period {self.control_period*1000:.1f} ms "
              f"({1.0/max(self.control_period,1e-6):.1f} Hz)")

        self._prev_action = np.zeros(2, dtype=np.float32)
        self._prev_pos = None
        self._col_base = 0
        self._lap_base = 0
        self._steps = 0
        self._stalled = 0
        self._too_close = 0
        self._prev_laps = 0
        self._ep = {}
        self._acc_thr = self._acc_steer = self._acc_sat = 0.0
        self._ep_max_speed = 0.0

    # ---------------------------------------------------------------- helpers
    def _scale(self, action):
        a = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        steering = float(a[0]) * self.cfg.act.max_steer
        lo, hi = self.cfg.act.throttle_min, self.cfg.act.throttle_max
        throttle = lo + (float(a[1]) + 1.0) * 0.5 * (hi - lo)
        return a, throttle, steering

    def _state(self):
        """Returns (beams, position, forward_vec, speed, yaw_rate).

        COMPETITION LEGALITY -- what feeds the OBSERVATION vs the REWARD:
          observation: beams (/lidar), speed (/left+right_encoder),
                       yaw_rate (/imu)          <- all permitted topics
          reward only: position (/odom)         <- restricted at race time,
                       but training-only and never a policy input.
        """
        with self.node.cv:
            beams = self.node.beams

        # Orientation + yaw rate from the IMU (permitted).
        imu = self.node.imu
        if imu is not None:
            quat, yaw_rate = imu
        else:
            quat, yaw_rate = (0.0, 0.0, 0.0, 1.0), 0.0
        yaw = yaw_from_quat(*quat)
        fwd = (math.cos(yaw), math.sin(yaw))

        # Position: REWARD ONLY.
        odom = self.node.odom
        pos = np.array(odom[0][:2]) if odom is not None else np.zeros(2)

        if self.cfg.obs.use_encoder_speed:
            speed = self.node.enc_speed          # signed, from wheel encoders
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

    # ------------------------------------------------------------------ gym
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        env = self.cfg.env

        self.node.send(0.0, 0.0)
        # reset_command is level-triggered: the bridge re-emits it as 'V1 Reset'
        # every tick, so it must be pulsed or the sim resets forever.
        self.node.send_reset(True)
        self.node.wait_ticks(env.reset_pulse_ticks, env.tick_timeout)
        self.node.send_reset(False)

        time.sleep(env.reset_settle_s)
        self.node.wait_ticks(1, env.tick_timeout)

        beams, pos, _, speed, yaw_rate = self._state()
        # Re-baseline counters: works whether or not the sim zeroes them itself.
        self._col_base = self.node.collisions
        self._lap_base = self.node.laps
        self._prev_pos = pos
        self._prev_action[:] = 0.0
        self._steps = 0
        self._stalled = 0
        self._too_close = 0
        self._ep = {k: 0.0 for k in
                    ("progress", "speed", "center", "smooth", "prox", "dist", "lap")}
        self._prev_laps = self.node.laps
        self._acc_thr = 0.0
        self._acc_steer = 0.0
        self._acc_sat = 0.0
        self._ep_max_speed = 0.0

        obs = build_obs(beams, self.cfg.obs.range_max, speed, self.cfg.obs.v_max,
                        yaw_rate, self.cfg.obs.yaw_rate_max, 0.0, 0.0,
                        self.cfg.act.throttle_max)
        return obs, {}

    def step(self, action):
        cfg, rw = self.cfg, self.cfg.rew
        a, throttle, steering = self._scale(action)

        self.node.send(throttle, steering)
        fresh = self.node.wait_ticks(cfg.env.decimation, cfg.env.tick_timeout)

        beams, pos, fwd, speed, yaw_rate = self._state()
        feats = beam_features(beams, cfg.obs.range_max, rw.safe_dist)

        # Forward progress = displacement projected on heading. Clamped because
        # a reset teleport would otherwise inject a huge spurious reward.
        d = pos - self._prev_pos
        ds = float(np.clip(d[0] * fwd[0] + d[1] * fwd[1], -1.0, 1.0))
        self._prev_pos = pos

        r_progress = rw.w_progress * ds
        # Speed REWARD uses /odom, not the encoder value in `speed`.
        # /odom is restricted only at RACE TIME; the reward is training-only and
        # never an input at inference, so it may use the accurate source. The
        # encoders overread up to ~3x under hard acceleration (wheel slip), so
        # paying reward on them would literally pay the agent to spin its wheels.
        _od = self.node.odom
        _rew_speed = math.hypot(_od[2][0], _od[2][1]) if _od is not None else abs(speed)
        r_speed = rw.w_speed * float(np.clip(_rew_speed / cfg.obs.v_max, 0.0, 1.0))
        p_center = rw.w_center * feats["center_err"]
        p_smooth = rw.w_smooth * abs(float(a[0]) - float(self._prev_action[0]))
        p_prox = rw.w_prox * feats["prox"]

        # Lap bonus: the only term that targets lap TIME directly rather than
        # distance. Paid once per completed lap, larger for a faster lap.
        r_lap = 0.0
        laps_now = self.node.laps
        if rw.w_lap > 0.0 and laps_now > self._prev_laps:
            n_new = laps_now - self._prev_laps
            lt = self.node.last_lap_time
            if lt and lt > 0.0:
                r_lap = n_new * rw.w_lap / max(lt, rw.min_lap_time)
            else:
                r_lap = n_new * rw.lap_bonus_flat
            self._prev_laps = laps_now

        reward = (r_progress + r_speed + r_lap
                  - p_center - p_smooth - p_prox - rw.step_penalty)

        self._ep["progress"] += r_progress
        self._ep["speed"] += r_speed
        self._ep["center"] += p_center
        self._ep["smooth"] += p_smooth
        self._ep["prox"] += p_prox
        self._ep["dist"] += ds
        self._ep["lap"] += r_lap
        # Action statistics: these tell us empirically whether the throttle cap
        # is binding, i.e. whether the agent WANTS to go faster than we allow.
        self._acc_thr += throttle
        self._acc_steer += abs(float(a[0]))
        self._acc_sat += 1.0 if float(a[1]) > 0.9 else 0.0
        self._ep_max_speed = max(self._ep_max_speed, speed)

        terminated = False
        reason = ""
        if self.node.collisions > self._col_base:
            reward -= rw.crash_penalty
            terminated, reason = True, "crash"

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

        if not fresh:
            # Sim stopped ticking (paused, crashed, or disconnected). End the
            # episode rather than training on stale observations.
            terminated, reason = True, "sim_timeout"

        self._steps += 1
        truncated = self._steps >= cfg.env.max_episode_steps
        self._prev_action[:] = a

        obs = build_obs(beams, cfg.obs.range_max, speed, cfg.obs.v_max,
                        yaw_rate, cfg.obs.yaw_rate_max, a[0], a[1],
                        cfg.act.throttle_max)

        info = {"speed": speed, "min_range": feats["min_range"],
                "laps": self.node.laps - self._lap_base,
                "throttle_cap": cfg.act.throttle_max}
        if terminated or truncated:
            info["reason"] = reason or "timeout"
            stats = dict(self._ep)
            n = max(1, self._steps)
            stats["act_throttle_mean"] = self._acc_thr / n
            stats["act_steer_abs_mean"] = self._acc_steer / n
            stats["act_throttle_sat"] = self._acc_sat / n
            stats["max_speed"] = self._ep_max_speed
            info["episode_stats"] = stats
            self.node.send(0.0, 0.0)   # never leave the throttle open
        return obs, float(reward), terminated, truncated, info

    def close(self):
        try:
            self.node.send(0.0, 0.0)
            self._exec.shutdown()
            self.node.destroy_node()
        except Exception:
            pass
