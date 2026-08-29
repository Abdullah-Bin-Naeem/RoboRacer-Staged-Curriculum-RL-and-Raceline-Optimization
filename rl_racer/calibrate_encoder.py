#!/usr/bin/env python3
"""Resolve the encoder units and measure metres-per-encoder-unit empirically.

The Technical Guide says JointState.position is cumulative TICKS (1920/rev);
autodrive_bridge.py line 217 treats it as RADIANS (`angle % 6.283`). These
differ by ~305x, so we measure instead of guessing.

Uses /odom for ground-truth distance -- legal during TRAINING (restricted only
at race time), which is exactly what calibration is for. The constant it prints
is then used to derive speed from encoders alone at inference.

    source /opt/ros/humble/setup.bash
    source ~/Documents/roboracer/.venv-rl/bin/activate
    python calibrate_encoder.py
"""
import math, threading, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy)
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import Odometry

NS = "/autodrive/roboracer_1"
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
WHEEL_R = 0.0590          # m,  from the Technical Guide
TICKS_PER_REV = 1920.0    # PPR 16 x CR 120


class Cal(Node):
    def __init__(s):
        super().__init__("encoder_cal")
        s.enc = None; s.pos = None; s.front = 99.0; s.speed = 0.0; s.n = 0
        s.create_subscription(JointState, f"{NS}/left_encoder",  s._el, QOS)
        s.create_subscription(JointState, f"{NS}/right_encoder", s._er, QOS)
        s.create_subscription(Odometry,   f"{NS}/odom",          s._od, QOS)
        s.create_subscription(LaserScan,  f"{NS}/lidar",         s._ld, QOS)
        s.pt = s.create_publisher(Float32, f"{NS}/throttle_command", QOS)
        s.ps = s.create_publisher(Float32, f"{NS}/steering_command", QOS)
        s.pr = s.create_publisher(Bool, "/autodrive/reset_command", QOS)
        s.l = s.r = 0.0
    def _el(s, m): s.l = m.position[0] if m.position else 0.0
    def _er(s, m): s.r = m.position[0] if m.position else 0.0
    def _od(s, m):
        p = m.pose.pose.position; v = m.twist.twist.linear
        s.pos = np.array([p.x, p.y]); s.speed = math.hypot(v.x, v.y); s.n += 1
    def _ld(s, m):
        a = np.nan_to_num(np.asarray(m.ranges, dtype=np.float32), nan=10.0, posinf=10.0)
        c = len(a)//2; s.front = float(a[c-60:c+60].min())
    def drive(s, t):
        a, b = Float32(), Float32(); a.data, b.data = float(t), 0.0
        s.pt.publish(a); s.ps.publish(b)
    def reset(s):
        s.drive(0.0); time.sleep(0.3)
        m = Bool(); m.data = True; s.pr.publish(m); time.sleep(0.3)
        m.data = False; s.pr.publish(m); time.sleep(1.5)


def main():
    rclpy.init(); n = Cal()
    ex = SingleThreadedExecutor(); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.time()
    while n.n == 0 and time.time() - t0 < 30: time.sleep(0.2)
    if n.n == 0:
        print("No /odom. Start the simulator and bridge first."); return

    samples = []
    for trial, th in enumerate([0.08, 0.12, 0.16]):
        n.reset()
        e0 = 0.5*(n.l + n.r); p0 = n.pos.copy(); t0 = time.time()
        while time.time() - t0 < 4.0 and n.front > 2.5:
            n.drive(th); time.sleep(0.05)
        n.drive(0.0)
        de = 0.5*(n.l + n.r) - e0
        dp = float(np.linalg.norm(n.pos - p0))
        if abs(de) > 1e-9 and dp > 0.3:
            samples.append((dp, de))
            print(f"  trial {trial}: throttle {th:.2f}  distance {dp:6.2f} m  "
                  f"encoder delta {de:12.3f}  -> {dp/de:.6e} m/unit")
        time.sleep(0.8)
    n.reset(); n.drive(0.0)

    if not samples:
        print("\nNo usable samples (car did not move, or no clear road).")
    else:
        mpu = float(np.mean([d/e for d, e in samples]))
        rad_pred = WHEEL_R                       # if position is radians
        tick_pred = WHEEL_R * 2*math.pi / TICKS_PER_REV   # if position is ticks
        print(f"\n  MEASURED   : {mpu:.6e} m per encoder unit")
        print(f"  if RADIANS : {rad_pred:.6e}   (ratio {mpu/rad_pred:.3f})")
        print(f"  if TICKS   : {tick_pred:.6e}   (ratio {mpu/tick_pred:.3f})")
        best = "RADIANS" if abs(math.log10(mpu/rad_pred)) < abs(math.log10(mpu/tick_pred)) else "TICKS"
        print(f"\n  => encoder position is in {best}")
        print(f"  => set ObsCfg.metres_per_encoder_unit = {mpu:.6e}")
    ex.shutdown(); n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
