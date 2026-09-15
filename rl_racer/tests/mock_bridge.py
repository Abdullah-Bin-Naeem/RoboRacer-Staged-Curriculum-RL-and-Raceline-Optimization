#!/usr/bin/env python3
"""Stand-in for the AutoDRIVE bridge: publishes every topic the env reads, in
lockstep, at a chosen tick, driven by a car that uses the SAME tire model the
observer assumes. Consumes the env's commands. Test-only; never the real sim.

  --tick-ms 22      publish period
  --stop-after S    stop publishing after S seconds (sim_timeout test)

Extra test topics: /mock/collide (Bool) bumps collision_count once;
/mock/reset_count (Int32) reports how many reset_command=True were seen.
"""
import argparse, math, time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)
from std_msgs.msg import Float32, Bool, Int32
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry

NS = "/autodrive/roboracer_1"
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)

# vehicle model, identical numbers to rl_racer.config.VehicleCfg
U_PER = 25.25; S_PK = 0.15; MU_PK = 0.72; S_AS = 0.25; MU_AS = 0.464; RISE = 3.0
DRAG = 0.273; SLIP_DEN = 4.0; L = 0.324; MAX_STEER = 0.5236; G = 9.81; R_WHEEL = 0.0581


def mu(s):
    a = abs(s)
    if a <= S_PK:
        t = a / S_PK
        return MU_PK * ((3 * t * t - 2 * t ** 3) + RISE * (t - 2 * t * t + t ** 3))
    if a <= S_AS:
        t = (a - S_PK) / (S_AS - S_PK)
        return MU_PK - (MU_PK - MU_AS) * (3 * t * t - 2 * t ** 3)
    return MU_AS


class Mock(Node):
    def __init__(self, tick_ms, stop_after):
        super().__init__("mock_bridge")
        self.dt = tick_ms / 1000.0
        self.stop_after = stop_after
        self.t0 = time.time()
        self.lock = threading.Lock()
        self.throttle = 0.0; self.steer = 0.0
        self.thr_applied = 0.0; self.steer_applied = 0.0     # one-tick command delay
        self.reset_level = False; self.reset_count = 0
        self.collisions = 0; self.laps = 0
        self.spawn()
        self.p_lidar = self.create_publisher(LaserScan, f"{NS}/lidar", QOS)
        self.p_odom = self.create_publisher(Odometry, f"{NS}/odom", QOS)
        self.p_imu = self.create_publisher(Imu, f"{NS}/imu", QOS)
        self.p_le = self.create_publisher(JointState, f"{NS}/left_encoder", QOS)
        self.p_re = self.create_publisher(JointState, f"{NS}/right_encoder", QOS)
        self.p_col = self.create_publisher(Int32, f"{NS}/collision_count", QOS)
        self.p_lap = self.create_publisher(Int32, f"{NS}/lap_count", QOS)
        self.p_lt = self.create_publisher(Float32, f"{NS}/last_lap_time", QOS)
        self.p_rc = self.create_publisher(Int32, "/mock/reset_count", QOS)
        self.create_subscription(Float32, f"{NS}/throttle_command", self._thr, QOS)
        self.create_subscription(Float32, f"{NS}/steering_command", self._st, QOS)
        self.create_subscription(Bool, "/autodrive/reset_command", self._rst, QOS)
        self.create_subscription(Bool, "/mock/collide", self._collide, QOS)
        self.timer = self.create_timer(self.dt, self.tick)
        # scan geometry: 1081 beams, +/-135 deg, 0.25 deg/beam, corridor
        self.n = 1081; self.a0 = -math.radians(135.0); self.inc = math.radians(0.25)
        self.scan_ranges = []
        for i in range(self.n):
            a = self.a0 + i * self.inc
            # walls at 1.5 m (right) and 2.0 m (left) along y, open ahead to 10 m
            sa = math.sin(a)
            if sa < -1e-3: r = 1.5 / -sa
            elif sa > 1e-3: r = 2.0 / sa
            else: r = 10.0
            self.scan_ranges.append(float(min(r, 10.0)))

    def spawn(self):
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0; self.v = 0.0; self.yaw_rate = 0.0
        self.ang_l = 0.0; self.ang_r = 0.0          # ResetManager restores counters

    def _thr(self, m):
        with self.lock: self.throttle = float(m.data)
    def _st(self, m):
        with self.lock: self.steer = float(m.data)
    def _rst(self, m):
        with self.lock:
            if m.data and not self.reset_level:
                self.reset_count += 1
            self.reset_level = bool(m.data)
            if m.data: self.spawn()
    def _collide(self, m):
        with self.lock:
            if m.data: self.collisions += 1

    def tick(self):
        if self.stop_after and time.time() - self.t0 > self.stop_after:
            return
        with self.lock:
            if self.reset_level:
                self.spawn()
            # car physics with one tick of command delay (bridge round trip)
            u = U_PER * self.thr_applied
            delta = self.steer_applied * MAX_STEER
            s = (u - self.v) / max(self.v, SLIP_DEN)
            a = math.copysign(mu(s) * G, s) - DRAG * self.v
            self.v = max(0.0, self.v + a * self.dt)
            self.yaw_rate = self.v * math.tan(delta) / L
            self.yaw += self.yaw_rate * self.dt
            self.x += self.v * math.cos(self.yaw) * self.dt
            self.y += self.v * math.sin(self.yaw) * self.dt
            self.ang_l += u / R_WHEEL * self.dt
            self.ang_r += u / R_WHEEL * self.dt
            self.thr_applied, self.steer_applied = self.throttle, self.steer
            now = self.get_clock().now().to_msg()
            # --- publish everything together, like the bridge does
            sc = LaserScan(); sc.header.stamp = now; sc.header.frame_id = "lidar"
            sc.angle_min = self.a0; sc.angle_max = -self.a0; sc.angle_increment = self.inc
            sc.range_min = 0.0; sc.range_max = 10.0; sc.ranges = self.scan_ranges
            od = Odometry(); od.header.stamp = now
            od.pose.pose.position.x = self.x; od.pose.pose.position.y = self.y
            od.pose.pose.orientation.z = math.sin(self.yaw / 2); od.pose.pose.orientation.w = math.cos(self.yaw / 2)
            od.twist.twist.linear.x = self.v; od.twist.twist.angular.z = self.yaw_rate
            im = Imu(); im.header.stamp = now
            im.orientation.z = math.sin(self.yaw / 2); im.orientation.w = math.cos(self.yaw / 2)
            im.angular_velocity.z = self.yaw_rate
            le = JointState(); le.header.stamp = now; le.name = ["left"]; le.position = [self.ang_l]
            re = JointState(); re.header.stamp = now; re.name = ["right"]; re.position = [self.ang_r]
            c = Int32(); c.data = self.collisions
            lp = Int32(); lp.data = self.laps
            lt = Float32(); lt.data = 0.0
            rc = Int32(); rc.data = self.reset_count
        self.p_odom.publish(od); self.p_imu.publish(im)
        self.p_le.publish(le); self.p_re.publish(re)
        self.p_col.publish(c); self.p_lap.publish(lp); self.p_lt.publish(lt); self.p_rc.publish(rc)
        self.p_lidar.publish(sc)           # last: the env's master clock


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tick-ms", type=float, default=22.0)
    ap.add_argument("--stop-after", type=float, default=0.0)
    a = ap.parse_args()
    rclpy.init()
    n = Mock(a.tick_ms, a.stop_after)
    print(f"[mock] publishing at {a.tick_ms:g} ms" + (f", stopping after {a.stop_after:g} s" if a.stop_after else ""), flush=True)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
