#!/usr/bin/env python3

"""Measure the map from steering command to path curvature.

The devkit's steering_command is normalised to [-1, 1] with no documented
mechanical angle, and the two wheelbase figures in the stack disagree (the
bridge's Ackermann math implies 0.283 m, the TF wheel positions say 0.33 m).

Pure pursuit does not need either number. It asks for a CURVATURE, and a bicycle
model gives kappa = tan(delta)/L, so command -> kappa is one relationship that
absorbs both unknowns at once. Measure it instead of deriving it.

Method: hold a constant steering command, let the car settle into a steady-state
circle, and fit that circle to the ground-truth positions. kappa = 1/radius.

Uses /odom for ground truth -- legal during DEVELOPMENT, restricted at race
time, which is exactly what calibration is for.

    ros2 run roboracer_stack calibrate_steering
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def fit_circle(x, y):
    """Algebraic (Kasa) circle fit. Returns (cx, cy, radius)."""
    a = np.column_stack([x, y, np.ones(len(x))])
    b = x ** 2 + y ** 2
    cx, cy, c = np.linalg.lstsq(a, b, rcond=None)[0]
    cx, cy = cx / 2, cy / 2
    return cx, cy, math.sqrt(c + cx ** 2 + cy ** 2)


class SteeringCal(Node):

    def __init__(self):
        super().__init__('calibrate_steering')
        self.pos = None
        self.create_subscription(Odometry, f'{NS}/odom', self._odom, QOS)
        self.pub_t = self.create_publisher(Float32, f'{NS}/throttle_command', QOS)
        self.pub_s = self.create_publisher(Float32, f'{NS}/steering_command', QOS)
        self.pub_r = self.create_publisher(Bool, '/autodrive/reset_command', QOS)

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y)

    def send(self, throttle, steering):
        t, s = Float32(), Float32()
        t.data, s.data = float(throttle), float(steering)
        self.pub_t.publish(t)
        self.pub_s.publish(s)

    def reset(self):
        for flag in (True, True, True, False):
            b = Bool()
            b.data = flag
            self.pub_r.publish(b)
            self.spin(0.1)
        self.send(0.0, 0.0)
        self.spin(1.0)

    def spin(self, seconds):
        end = self.get_clock().now().nanoseconds * 1e-9 + seconds
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_one(self, cmd, throttle=0.08, settle=2.5, record=6.0):
        """Drive a steady circle at `cmd` and return signed curvature."""
        self.reset()
        self.send(throttle, cmd)
        self.spin(settle)            # discard the transient

        pts, end = [], self.get_clock().now().nanoseconds * 1e-9 + record
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.pos:
                pts.append(self.pos)
        self.send(0.0, 0.0)

        pts = np.array(pts)
        if len(pts) < 30:
            return None
        # Deduplicate: a stalled car yields a degenerate fit.
        span = np.ptp(pts, axis=0).max()
        if span < 0.3:
            self.get_logger().warn(f'cmd {cmd:+.2f}: car barely moved ({span:.2f} m)')
            return None

        cx, cy, r = fit_circle(pts[:, 0], pts[:, 1])
        # Sign: cross product of heading and centre direction says which way it turned.
        d0 = pts[min(10, len(pts) - 1)] - pts[0]
        to_c = np.array([cx, cy]) - pts[0]
        sign = math.copysign(1.0, d0[0] * to_c[1] - d0[1] * to_c[0])
        return sign / r, r, len(pts)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringCal()
    node.get_logger().info('waiting for /odom ...')
    while node.pos is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    commands = [0.15, 0.3, 0.45, 0.6, 0.8, 1.0, -0.3, -0.6, -1.0]
    rows = []
    try:
        for cmd in commands:
            out = node.run_one(cmd)
            if out is None:
                node.get_logger().warn(f'cmd {cmd:+.2f}: skipped')
                continue
            kappa, radius, n = out
            rows.append((cmd, kappa))
            node.get_logger().info(
                f'cmd {cmd:+.2f}  radius {radius:6.2f} m  kappa {kappa:+.3f} 1/m  ({n} pts)')

        if len(rows) >= 3:
            c = np.array([r[0] for r in rows])
            k = np.array([r[1] for r in rows])
            # Through the origin: zero command must mean zero curvature.
            gain = float((c @ k) / (c @ c))
            resid = k - gain * c
            print('\n' + '=' * 58)
            print(f'  kappa_per_command = {gain:.4f}   [1/m per unit command]')
            print(f'  max curvature at |cmd|=1: {abs(gain):.3f} 1/m '
                  f'-> tightest radius {1/abs(gain):.2f} m')
            print(f'  fit residual (rms): {np.sqrt((resid**2).mean()):.4f} 1/m')
            print('=' * 58)
            print('\nPut this in config/pure_pursuit.yaml as kappa_per_command.')
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
