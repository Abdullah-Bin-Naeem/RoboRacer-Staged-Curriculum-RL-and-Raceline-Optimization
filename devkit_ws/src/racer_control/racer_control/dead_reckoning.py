#!/usr/bin/env python3

"""Race-legal odometry from wheel encoders + IMU.

AMCL cannot run without an odometry input -- its particle propagation step IS a
motion model driven by odometry deltas. The devkit's own /odom is ground truth
and RESTRICTED at race time, so this node rebuilds an equivalent from the two
permitted sources:

    distance  <- wheel encoders  (cumulative wheel angle, radians)
    heading   <- IMU orientation (absolute quaternion, so yaw does NOT drift)

Publishes `odom -> base_frame` on /tf plus a nav_msgs/Odometry. Position drifts,
because wheel slip makes the encoders overread (measured at up to ~3x under hard
acceleration in the RL work). Correcting that drift is exactly AMCL's job -- but
it is also why the motion-model noise in the AMCL config has to be generous.

Because IMU yaw is absolute and world-referenced, the `odom` frame here comes out
aligned with the simulator's world orientation; only its origin differs.
"""

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DeadReckoning(Node):

    def __init__(self):
        super().__init__('dead_reckoning')

        p = self.declare_parameter
        p('wheel_radius', 0.0590)       # published spec
        p('odom_frame', 'odom')
        p('base_frame', 'roboracer_1')  # devkit's name, so `lidar` still parents to it
        p('publish_rate', 50.0)
        p('publish_tf', True)
        # Encoders overread under slip. <1.0 trims the systematic part; AMCL
        # handles what is left.
        p('distance_scale', 1.0)

        g = lambda n: self.get_parameter(n).value
        self.wheel_r = g('wheel_radius')
        self.odom_frame, self.base_frame = g('odom_frame'), g('base_frame')
        self.publish_tf = g('publish_tf')
        self.scale = g('distance_scale')

        self.x = self.y = 0.0
        self.yaw = None                 # from IMU; None until the first message
        self.speed = 0.0
        self.yaw_rate = 0.0
        self._enc = {}                  # side -> (angle, stamp)
        self._rate = {}                 # side -> m/s
        self._last = None

        self.create_subscription(JointState, f'{NS}/left_encoder',
                                 lambda m: self._cb_enc('l', m), QOS)
        self.create_subscription(JointState, f'{NS}/right_encoder',
                                 lambda m: self._cb_enc('r', m), QOS)
        self.create_subscription(Imu, f'{NS}/imu', self._cb_imu, QOS)

        self.pub = self.create_publisher(Odometry, 'odom', QOS)
        self.tfb = tf2_ros.TransformBroadcaster(self)
        self.create_timer(1.0 / g('publish_rate'), self._tick)

        self.get_logger().info(
            f'dead reckoning: {self.odom_frame} -> {self.base_frame}, '
            f'wheel r={self.wheel_r} m, scale={self.scale}')

    def _cb_enc(self, side, msg):
        """position is cumulative wheel angle in RADIANS (measured, not ticks)."""
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
            return          # stale or duplicate frame; a bad dt gives garbage
        self._rate[side] = (ang - prev[0]) / dt * self.wheel_r
        rates = [v for v in self._rate.values() if v is not None]
        if rates:
            self.speed = float(np.mean(rates)) * self.scale

    def _cb_imu(self, msg):
        self.yaw = yaw_from_quat(msg.orientation)
        self.yaw_rate = msg.angular_velocity.z

    def _tick(self):
        if self.yaw is None:
            return
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self._last is None:
            self._last = t
            return
        dt = t - self._last
        self._last = t
        if dt <= 0.0 or dt > 0.5:
            return

        # Integrate encoder distance along the IMU heading. No bicycle model
        # needed: the IMU already gives the true heading each step.
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt

        stamp = now.to_msg()
        half = self.yaw / 2.0
        qz, qw = math.sin(half), math.cos(half)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tfb.sendTransform(tf)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = self.x, self.y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x = self.speed
        od.twist.twist.angular.z = self.yaw_rate
        # Position is dead-reckoned and drifts; heading comes from an absolute
        # sensor. Say so, so AMCL weights them sensibly.
        od.pose.covariance[0] = od.pose.covariance[7] = 0.05
        od.pose.covariance[35] = 0.01
        od.twist.covariance[0] = 0.02
        od.twist.covariance[35] = 0.01
        self.pub.publish(od)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
