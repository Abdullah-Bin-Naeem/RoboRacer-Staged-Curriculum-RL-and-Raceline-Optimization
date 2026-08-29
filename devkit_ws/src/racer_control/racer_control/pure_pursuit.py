#!/usr/bin/env python3

"""Pure pursuit path follower for the AutoDRIVE RoboRacer.

Loads a path CSV (s, x, y, psi, kappa, w_right, w_left -- as produced by the
raceline notebook), picks a point one lookahead ahead of the car, and steers
along the arc that reaches it.

Geometry: with the vehicle frame at the rear axle, an arc through a target at
(local_x, local_y) a distance Ld away has curvature

    kappa = 2 * local_y / Ld^2

The steering command follows from the competition's published vehicle spec
(wheelbase 0.3240 m, steering limit +/-0.5236 rad, command normalised to [-1,1]):

    delta = atan(kappa * wheelbase)
    command = delta / max_steer_rad

Note this is NOT linear in kappa -- a single gain would be wrong near full lock.
`steering_gain` is a trim multiplier for whatever the model does not capture;
verify it with racer_control/calibrate_steering.py.

RACE LEGALITY: pose comes from /odom, which is RESTRICTED during racing, and the
rules also forbid pre-recorded maps at evaluation time. This node is therefore a
DEVELOPMENT tool -- for validating a path and a controller offline. Point
`pose_topic` at an online-SLAM or particle-filter pose to make it raceable.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Int32
from visualization_msgs.msg import Marker

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        p = self.declare_parameter
        p('path_csv', '')
        p('pose_topic', f'{NS}/odom')          # or an AMCL PoseWithCovarianceStamped
        p('control_hz', 20.0)
        # Pose arrives in /odom's frame, which the devkit calls 'world'.
        p('viz_frame', 'world')

        # Lookahead grows with speed: too short oscillates, too long cuts corners.
        p('lookahead_min', 0.45)
        p('lookahead_max', 1.30)
        p('lookahead_k', 0.35)                 # Ld = k * speed, then clamped

        # From the competition vehicle spec, not guessed.
        p('wheelbase', 0.3240)                 # [m]
        p('max_steer_rad', 0.5236)             # [rad] = 30 deg at command 1.0
        p('steering_gain', 1.0)                # empirical trim; 1.0 = trust the model

        # Speed from path curvature: v = sqrt(a_lat_max / |kappa|), clamped.
        p('a_lat_max', 3.0)
        p('v_min', 1.0)
        p('v_max', 4.0)
        p('curvature_preview_m', 1.0)          # look this far ahead for the worst corner
        # If the CSV carries a v_mps column (8th), follow THAT instead of
        # re-deriving speed from curvature. The optimizer's profile accounts for
        # the traction ellipse and for braking before a corner; the fallback
        # below does neither, so comparing optimized paths without this compares
        # only their geometry.
        p('use_path_speed', True)

        p('throttle_kp', 0.030)
        p('throttle_ff', 0.040)                # measured: throttle 0.10 -> ~2.5 m/s
        p('throttle_max', 0.20)
        p('use_encoder_speed', True)           # race-legal source
        # Lap topics are RESTRICTED during racing. Development telemetry only.
        p('dev_lap_telemetry', False)
        # Stay silent until localization_bootstrap says the pose has converged.
        # Driving a raceline off an unconverged pose just chases a moving guess.
        p('wait_for_ready', False)
        # Drive on ground truth for this long, then switch to `pose_topic`.
        # AMCL converges far better while the car is MOVING than standing still,
        # so this gives it real motion to work with before it has to be trusted.
        # 0 disables. Ground truth is RESTRICTED at race time -- development only.
        p('bootstrap_seconds', 0.0)
        p('bootstrap_pose_topic', f'{NS}/odom')

        g = lambda n: self.get_parameter(n).value
        self.pose_topic = g('pose_topic')
        self.viz_frame = g('viz_frame')
        self.ld_min, self.ld_max, self.ld_k = g('lookahead_min'), g('lookahead_max'), g('lookahead_k')
        self.wheelbase = g('wheelbase')
        self.max_steer = g('max_steer_rad')
        self.steer_gain = g('steering_gain')
        self.a_lat, self.v_min, self.v_max = g('a_lat_max'), g('v_min'), g('v_max')
        self.preview = g('curvature_preview_m')
        self.use_path_speed = g('use_path_speed')
        self.kp, self.ff, self.thr_max = g('throttle_kp'), g('throttle_ff'), g('throttle_max')
        self.use_enc = g('use_encoder_speed')
        self.dev_lap = g('dev_lap_telemetry')
        self.wait_for_ready = g('wait_for_ready')
        self.ready = not self.wait_for_ready
        self.bootstrap_s = float(g('bootstrap_seconds'))
        self.bootstrap_topic = g('bootstrap_pose_topic')
        self.t_first = None          # when the first pose of any kind arrived
        self.using_truth = self.bootstrap_s > 0.0
        self.truth_pose = None

        csv = g('path_csv')
        if not csv:
            raise RuntimeError('path_csv parameter is required')
        data = np.loadtxt(csv, delimiter=',')
        self.s, self.px, self.py = data[:, 0], data[:, 1], data[:, 2]
        self.kappa = data[:, 4]
        self.path_v = data[:, 7] if data.shape[1] > 7 else None
        if self.use_path_speed and self.path_v is None:
            self.get_logger().warn(
                'use_path_speed is set but the CSV has no v_mps column; '
                'falling back to curvature-derived speed')
        self.lap_len = float(self.s[-1] + (self.s[1] - self.s[0]))
        self.get_logger().info(
            f'path: {len(self.px)} points, {self.lap_len:.2f} m lap, from {csv}')

        self.pose = None          # (x, y, yaw)
        self.speed = 0.0
        self._enc = {}
        self._enc_rate = {}
        self.laps = 0

        if 'odom' in self.pose_topic:
            self.create_subscription(Odometry, self.pose_topic, self._cb_odom, QOS)
        else:
            self.create_subscription(PoseWithCovarianceStamped, self.pose_topic,
                                     self._cb_amcl, QOS)

        if self.using_truth:
            self.create_subscription(Odometry, self.bootstrap_topic,
                                     self._cb_truth, QOS)
            self.get_logger().warn(
                f'driving on {self.bootstrap_topic} (GROUND TRUTH, restricted at '
                f'race time) for the first {self.bootstrap_s:.0f} s, then '
                f'switching to {self.pose_topic}')
        self.create_subscription(JointState, f'{NS}/left_encoder',
                                 lambda m: self._cb_enc('l', m), QOS)
        self.create_subscription(JointState, f'{NS}/right_encoder',
                                 lambda m: self._cb_enc('r', m), QOS)
        if self.dev_lap:
            # RESTRICTED topics -- never enable this for an evaluation run.
            self.get_logger().warn('dev_lap_telemetry ON: subscribing to RESTRICTED lap topics')
            self.create_subscription(Int32, f'{NS}/lap_count', self._cb_lap, QOS)
            self.create_subscription(Float32, f'{NS}/last_lap_time', self._cb_lap_time, QOS)

        if self.wait_for_ready:
            latched = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=QoSReliabilityPolicy.RELIABLE,
                                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.create_subscription(Bool, '/localization_ready', self._cb_ready, latched)
            self.get_logger().info('holding until /localization_ready')

        self.pub_t = self.create_publisher(Float32, f'{NS}/throttle_command', QOS)
        self.pub_s = self.create_publisher(Float32, f'{NS}/steering_command', QOS)
        self.pub_path = self.create_publisher(Path, '~/path', 1)
        self.pub_target = self.create_publisher(Marker, '~/lookahead', 1)

        self.create_timer(1.0 / g('control_hz'), self._control)
        self.create_timer(2.0, self._publish_path)

    # ---- callbacks -------------------------------------------------------

    def _cb_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        if not self.use_enc:
            t = msg.twist.twist.linear
            self.speed = math.hypot(t.x, t.y)

    def _cb_amcl(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

    def _cb_truth(self, msg):
        p = msg.pose.pose.position
        self.truth_pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        t = msg.twist.twist.linear
        if not self.use_enc:
            self.speed = math.hypot(t.x, t.y)

    def _cb_enc(self, side, msg):
        """Wheel angle is in RADIANS (the bridge treats it that way)."""
        if not msg.position:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ang = float(msg.position[0])
        prev = self._enc.get(side)
        self._enc[side] = (ang, t)
        if prev is None:
            return
        dt = t - prev[1]
        if dt <= 1e-4 or dt > 0.5:
            return   # stale or duplicate frame; a bad dt yields a garbage speed
        self._enc_rate[side] = (ang - prev[0]) / dt * 0.0590   # wheel radius [m]
        rates = [v for v in self._enc_rate.values() if v is not None]
        if rates and self.use_enc:
            self.speed = float(np.mean(rates))

    def _cb_lap(self, msg):
        if msg.data > self.laps:
            self.laps = msg.data

    def _cb_lap_time(self, msg):
        if msg.data > 0.0:
            self.get_logger().info(f'lap {self.laps}: {msg.data:.2f} s')

    # ---- control ---------------------------------------------------------

    def _cb_ready(self, msg):
        if msg.data and not self.ready:
            self.ready = True
            self.get_logger().info('localization converged, taking over')

    def _control(self):
        if not self.ready:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        pose = self.pose
        if self.using_truth:
            pose = self.truth_pose
            if pose is not None:
                if self.t_first is None:
                    self.t_first = now
                elif now - self.t_first >= self.bootstrap_s:
                    self.using_truth = False
                    gap = (math.hypot(self.pose[0] - pose[0], self.pose[1] - pose[1])
                           if self.pose is not None else float('nan'))
                    self.get_logger().info(
                        f'handing over to {self.pose_topic} after '
                        f'{self.bootstrap_s:.0f} s -- estimate was {gap:.3f} m from '
                        'truth at the switch')
                    pose = self.pose

        if pose is None:
            return
        x, y, yaw = pose

        d = np.hypot(self.px - x, self.py - y)
        near = int(np.argmin(d))

        ld = float(np.clip(self.ld_k * abs(self.speed), self.ld_min, self.ld_max))

        # Walk forward along the path until one lookahead away, wrapping the loop.
        n = len(self.px)
        idx = near
        for _ in range(n):
            idx = (idx + 1) % n
            if math.hypot(self.px[idx] - x, self.py[idx] - y) >= ld:
                break

        dx, dy = self.px[idx] - x, self.py[idx] - y
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        local_y = dx * sin_y + dy * cos_y
        actual_ld = max(math.hypot(dx, dy), 1e-3)

        kappa_cmd = 2.0 * local_y / (actual_ld ** 2)
        delta = math.atan(kappa_cmd * self.wheelbase)          # bicycle model
        steering = float(np.clip(self.steer_gain * delta / self.max_steer, -1.0, 1.0))

        step = self.lap_len / n
        span = max(1, int(self.preview / step))
        window = [(near + i) % n for i in range(span)]

        if self.use_path_speed and self.path_v is not None:
            # Take the slowest speed the profile demands within the preview, so
            # the car is already slowing when the corner arrives.
            v_target = float(np.clip(min(self.path_v[i] for i in window),
                                     self.v_min, self.v_max))
        else:
            k_worst = max(abs(self.kappa[i]) for i in window)
            v_target = float(np.clip(math.sqrt(self.a_lat / max(k_worst, 1e-3)),
                                     self.v_min, self.v_max))

        err = v_target - self.speed
        throttle = float(np.clip(self.ff * v_target + self.kp * err, 0.0, self.thr_max))

        t, s = Float32(), Float32()
        t.data, s.data = throttle, steering
        self.pub_t.publish(t)
        self.pub_s.publish(s)
        self._publish_target(self.px[idx], self.py[idx])

    # ---- viz -------------------------------------------------------------

    def _publish_path(self):
        msg = Path()
        msg.header.frame_id = self.viz_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        from geometry_msgs.msg import PoseStamped
        for xi, yi in zip(self.px, self.py):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(xi), float(yi)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _publish_target(self, tx, ty):
        m = Marker()
        m.header.frame_id = self.viz_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y = float(tx), float(ty)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r, m.color.g, m.color.a = 1.0, 0.2, 1.0
        self.pub_target.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        t, s = Float32(), Float32()
        t.data = s.data = 0.0
        node.pub_t.publish(t)
        node.pub_s.publish(s)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
