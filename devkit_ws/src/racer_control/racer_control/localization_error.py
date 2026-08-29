#!/usr/bin/env python3

"""Measure how far AMCL's pose is from the truth.

The whole point of running the race-legal stack in the simulator is that ground
truth is still available to check it against. This node subscribes to both and
reports the error, so localization quality is a number rather than an impression.

    estimated: /amcl_pose                       (lidar + map + dead reckoning)
    truth:     /autodrive/roboracer_1/odom      (RESTRICTED at race time)

Development only -- it consumes a restricted topic by design.

Publishes the running error on ~/error (Float32, metres) so it can be plotted,
and prints a summary every few seconds.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class LocalizationError(Node):

    def __init__(self):
        super().__init__('localization_error')

        self.declare_parameter('estimate_topic', '/amcl_pose')
        self.declare_parameter('truth_topic', f'{NS}/odom')
        self.declare_parameter('report_period', 5.0)

        est_topic = self.get_parameter('estimate_topic').value
        truth_topic = self.get_parameter('truth_topic').value

        self.truth = None            # (x, y, yaw)
        self.estimate = None
        self.pos_err = []
        self.yaw_err = []

        self.create_subscription(Odometry, truth_topic, self._cb_truth, QOS)
        self.create_subscription(PoseWithCovarianceStamped, est_topic, self._cb_est, QOS)
        self.pub = self.create_publisher(Float32, '~/error', QOS)
        self.create_timer(self.get_parameter('report_period').value, self._report)

        self.get_logger().warn(
            f'DEVELOPMENT ONLY: comparing {est_topic} against {truth_topic}, '
            'which is RESTRICTED during racing')

    def _cb_truth(self, msg):
        p = msg.pose.pose.position
        self.truth = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

    def _cb_est(self, msg):
        p = msg.pose.pose.position
        self.estimate = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        if self.truth is None:
            return
        d = math.hypot(self.estimate[0] - self.truth[0],
                       self.estimate[1] - self.truth[1])
        self.pos_err.append(d)
        self.yaw_err.append(abs(wrap(self.estimate[2] - self.truth[2])))
        m = Float32()
        m.data = float(d)
        self.pub.publish(m)

    def _report(self):
        if not self.pos_err:
            if self.estimate is None:
                self.get_logger().info('waiting for /amcl_pose ...')
            elif self.truth is None:
                self.get_logger().info('waiting for ground truth ...')
            return
        p = np.array(self.pos_err)
        a = np.degrees(np.array(self.yaw_err))
        self.get_logger().info(
            f'n={len(p):5d}  position mean {p.mean():.3f} m  p95 {np.percentile(p,95):.3f}  '
            f'max {p.max():.3f}  |  heading mean {a.mean():.2f} deg  max {a.max():.2f}')

    def summary(self):
        if not self.pos_err:
            print('\nno samples collected')
            return
        p = np.array(self.pos_err)
        a = np.degrees(np.array(self.yaw_err))
        print('\n' + '=' * 56)
        print(f'  localization error over {len(p)} updates')
        print(f'    position  mean {p.mean():.3f}  p95 {np.percentile(p,95):.3f}  max {p.max():.3f}  m')
        print(f'    heading   mean {a.mean():.2f}  p95 {np.percentile(a,95):.2f}  max {a.max():.2f}  deg')
        print('=' * 56)
        # The raceline's tightest point sits ~0.13 m from a wall, so error much
        # above that will put the car into one.
        if p.max() > 0.13:
            print('  WARNING: peak error exceeds the raceline\'s wall margin (0.134 m).')
            print('           Expect contact at the tightest point of the lap.')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
