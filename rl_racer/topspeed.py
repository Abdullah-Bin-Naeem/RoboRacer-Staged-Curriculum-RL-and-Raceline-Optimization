#!/usr/bin/env python3
"""Measure the real throttle -> speed curve of the AutoDRIVE car.

For each throttle level: reset, drive straight, record the speed reached.
Aborts before hitting anything (watches the forward LiDAR), so it is safe to
run on a walled track. Reports whether the speed PLATEAUED (a true terminal
speed) or was CUT SHORT by a wall (a lower bound only).

    source /opt/ros/humble/setup.bash
    source ~/Documents/roboracer/.venv-rl/bin/activate
    python topspeed.py
"""
import math, threading, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy)
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

NS = "/autodrive/roboracer_1"
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
THROTTLES = [0.10, 0.20, 0.30, 0.40, 0.60, 0.80, 1.00]
ABORT_DIST = 2.5      # m of clear road ahead required to keep accelerating
MAX_T      = 8.0      # s per trial


class T(Node):
    def __init__(s):
        super().__init__("topspeed")
        s.speed = 0.0; s.front = 99.0; s.n = 0
        s.create_subscription(Odometry, f"{NS}/odom", s._o, QOS)
        s.create_subscription(LaserScan, f"{NS}/lidar", s._l, QOS)
        s.pt = s.create_publisher(Float32, f"{NS}/throttle_command", QOS)
        s.ps = s.create_publisher(Float32, f"{NS}/steering_command", QOS)
        s.pr = s.create_publisher(Bool, "/autodrive/reset_command", QOS)
    def _o(s, m):
        v = m.twist.twist.linear
        s.speed = math.hypot(v.x, v.y)      # magnitude: frame-independent
    def _l(s, m):
        r = np.nan_to_num(np.asarray(m.ranges, dtype=np.float32), nan=10.0, posinf=10.0)
        c = len(r) // 2
        s.front = float(r[c - 60:c + 60].min())   # ~+/-15 deg ahead
        s.n += 1
    def drive(s, t, st=0.0):
        a, b = Float32(), Float32(); a.data, b.data = float(t), float(st)
        s.pt.publish(a); s.ps.publish(b)
    def reset(s):
        s.drive(0.0); time.sleep(0.3)
        m = Bool(); m.data = True; s.pr.publish(m); time.sleep(0.3)
        m.data = False; s.pr.publish(m); time.sleep(1.5)


def main():
    rclpy.init(); n = T()
    ex = SingleThreadedExecutor(); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.time()
    while n.n == 0 and time.time() - t0 < 30: time.sleep(0.2)
    if n.n == 0:
        print("No LiDAR. Start the simulator and bridge first."); return

    print(f"{'throttle':>9} {'max speed':>10} {'ratio':>7}  {'ended':>10}")
    print("-" * 42)
    rows = []
    for th in THROTTLES:
        n.reset()
        peak, hist, t0, why = 0.0, [], time.time(), "timeout"
        while time.time() - t0 < MAX_T:
            n.drive(th)
            time.sleep(0.05)
            peak = max(peak, n.speed); hist.append(n.speed)
            if n.front < ABORT_DIST: why = "wall"; break
            if len(hist) > 30 and (max(hist[-20:]) - min(hist[-20:])) < 0.05:
                why = "PLATEAU"; break
        n.drive(0.0)
        rows.append((th, peak, why))
        print(f"{th:>9.2f} {peak:>9.2f}m/s {peak/th:>7.1f}  {why:>10}")
        time.sleep(1.0)
    n.reset(); n.drive(0.0)

    plateaued = [r for r in rows if r[2] == "PLATEAU"]
    print()
    if plateaued:
        th, pk, _ = plateaued[-1]
        print(f"Highest TRUE terminal speed measured: {pk:.2f} m/s at throttle {th:.2f}")
    print(f"Highest speed reached at all: {max(r[1] for r in rows):.2f} m/s")
    print("\n'wall' rows are LOWER BOUNDS - the car ran out of road before topping out.")
    ex.shutdown(); n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
