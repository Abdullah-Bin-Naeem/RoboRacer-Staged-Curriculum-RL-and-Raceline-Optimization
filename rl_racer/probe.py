#!/usr/bin/env python3
"""Pre-flight check of the AutoDRIVE <-> ROS plumbing. Needs only rclpy + numpy.

Run this BEFORE training. It answers the questions that otherwise waste hours:
does data actually flow, what is the real tick rate, does reset work, and does
the collision counter behave the way the reward function assumes?
"""
import math, sys, time, threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)
from std_msgs.msg import Float32, Bool, Int32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

NS = "/autodrive/roboracer_1"
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class Probe(Node):
    def __init__(self):
        super().__init__("rl_racer_probe")
        self.cv = threading.Condition()
        self.tick = 0
        self.scan = None
        self.odom = None
        self.col = None
        self.lap = None
        self.create_subscription(LaserScan, f"{NS}/lidar", self._lidar, QOS)
        self.create_subscription(Odometry, f"{NS}/odom", self._odom, QOS)
        self.create_subscription(Int32, f"{NS}/collision_count",
                                 lambda m: setattr(self, "col", m.data), QOS)
        self.create_subscription(Int32, f"{NS}/lap_count",
                                 lambda m: setattr(self, "lap", m.data), QOS)
        self.pt = self.create_publisher(Float32, f"{NS}/throttle_command", QOS)
        self.ps = self.create_publisher(Float32, f"{NS}/steering_command", QOS)
        self.pr = self.create_publisher(Bool, "/autodrive/reset_command", QOS)

    def _lidar(self, m):
        with self.cv:
            self.scan = m
            self.tick += 1
            self.cv.notify_all()

    def _odom(self, m):
        self.odom = m

    def send(self, t, s):
        a, b = Float32(), Float32()
        a.data, b.data = float(t), float(s)
        self.pt.publish(a); self.ps.publish(b)

    def reset(self, flag):
        b = Bool(); b.data = bool(flag); self.pr.publish(b)

    def wait(self, n, timeout):
        with self.cv:
            tgt = self.tick + n
            return self.cv.wait_for(lambda: self.tick >= tgt, timeout=timeout)


def pos(n):
    if n.odom is None:
        return np.zeros(2)
    p = n.odom.pose.pose.position
    return np.array([p.x, p.y])


def main():
    rclpy.init()
    n = Probe()
    ex = SingleThreadedExecutor(); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    print("[1/5] waiting for first LiDAR scan (30 s) ...")
    if not n.wait(1, 30.0):
        print("  FAIL: no scan. Is the simulator running AND the bridge launched?")
        print("        ros2 launch autodrive_roboracer bringup_headless.launch.py")
        return 1
    s = n.scan
    print(f"  OK  beams={len(s.ranges)} angle=[{math.degrees(s.angle_min):.1f},"
          f"{math.degrees(s.angle_max):.1f}]deg inc={s.angle_increment:.6f} "
          f"range_max={s.range_max}")
    half = math.radians(90.0)
    i0 = int(round((-half - s.angle_min) / s.angle_increment))
    i1 = int(round((half - s.angle_min) / s.angle_increment))
    print(f"  +/-90deg FOV -> indices [{i0}:{i1}] = {i1-i0} raw beams")

    print("[2/5] measuring sim tick rate over 5 s ...")
    t0, k0 = time.time(), n.tick
    time.sleep(5.0)
    hz = (n.tick - k0) / (time.time() - t0)
    print(f"  OK  {hz:.1f} Hz  -> decimation=2 gives {hz/2:.1f} Hz control")
    if hz < 5:
        print("  WARN: very low tick rate; training will be painfully slow.")

    print("[3/5] checking topic liveness ...")
    for name, v in (("odom", n.odom), ("collision_count", n.col), ("lap_count", n.lap)):
        print(f"  {'OK ' if v is not None else 'FAIL'} {name}"
              + ("" if v is not None else "  <-- no data (QoS mismatch?)"))

    print("[4/5] actuation test: throttle 0.10 for 2 s ...")
    p0 = pos(n)
    n.send(0.10, 0.0)
    time.sleep(2.0)
    n.send(0.0, 0.0)
    moved = float(np.linalg.norm(pos(n) - p0))
    v = n.odom.twist.twist.linear
    print(f"  moved {moved:.2f} m, |v|={math.hypot(v.x, v.y):.2f} m/s")
    if moved < 0.05:
        print("  WARN: car did not move. Check the sim is unpaused and not stuck.")
    time.sleep(1.0)

    print("[5/5] reset test (pulse True -> False) ...")
    before, col_before = pos(n), n.col
    n.reset(True); n.wait(3, 5.0); n.reset(False)
    time.sleep(1.5)
    after = pos(n)
    print(f"  position {before} -> {after}  (moved {np.linalg.norm(after-before):.2f} m)")
    print(f"  collision_count {col_before} -> {n.col}"
          f"  ({'sim zeroes it' if (n.col or 0) < (col_before or 0) else 'cumulative / unchanged'})")
    n.send(0.0, 0.0)

    print("\nAll checks done. If steps 1-4 are OK you are clear to train.")
    ex.shutdown(); n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
