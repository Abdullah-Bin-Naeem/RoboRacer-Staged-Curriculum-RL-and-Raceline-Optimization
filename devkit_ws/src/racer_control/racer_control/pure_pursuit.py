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

SPEED AND THROTTLE (raceline/VEHICLE_MODEL.md, read out of the simulator's
source): the sim spins each wheel to u = 25.25 * throttle m/s within a
millisecond whatever the car does, so the ENCODERS REPORT THE THROTTLE COMMAND,
not the car -- 5x high at launch, ~5 % high at cruise, near zero while braking.
Closing the speed loop on them closes it around the wheel and leaves the car
open loop. Hence two modes, both selectable from the yaml or the launch line:

    speed_source=tire     v_est is a simulation of the car driven by the MEASURED
                          wheel speed through the sim's own tire curve and drag
                          (VEHICLE_MODEL.md §3): v' = sign(S) mu(|S|) g - 0.273 v
                          with S = (u - v)/max(v, 4). Two copies of this system
                          driven by the same u converge, so the error is bounded
                          by the curve's slip error times the denominator and
                          never drifts. Measured offline on 4 runs against ground
                          truth: p90 0.17-0.28 m/s, max under 1 m/s, launches,
                          stalls and braking included. Feeds the lookahead, the
                          throttle and ~/status. Race-legal.
    speed_source=fused    IMU integral corrected toward the pose speed. Kept for
                          reference; it FAILED in the sim twice. The sim's IMU is
                          a 1 ms sample of a tire force that a 20 Hz throttle law
                          makes jump every tick (measured +6.3 at launch against a
                          true 4.0, -7.6 in a corner against -3.6), and the pose
                          speed between localizer updates is dead reckoning, i.e.
                          the encoders, so the correction chased wheelspin.
    throttle_mode=slip    the wheel speed is commanded to the target speed but
                          clamped to [v_est*(1-slip_brake), v_est*(1+slip_accel)],
                          the slips where the tire's force peaks. Encoder error is
                          bounded to that band in every regime, launch included,
                          and a restart after a wall respawn is covered by
                          construction because the band keys on the car's speed.

`speed_source=encoder` and `throttle_mode=legacy` keep the old behaviour for
A/B runs. ~/status carries v_est, the encoder speed, target, wheel command and
path error every tick; log_localization records it and raceline/analyze_run.py
reads it back.

RACE LEGALITY: pose comes from /odom, which is RESTRICTED during racing, and the
rules also forbid pre-recorded maps at evaluation time. This node is therefore a
DEVELOPMENT tool -- for validating a path and a controller offline. Point
`pose_topic` at an online-SLAM or particle-filter pose to make it raceable.
"""

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from racer_common import restricted
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32
from visualization_msgs.msg import Marker

NS = '/autodrive/roboracer_1'

# ~/status: one Float32MultiArray per control tick, fields in this order.
# log_localization records them as pp_<name> columns; raceline/analyze_run.py
# reads them back. Keep the three in step.
STATUS_FIELDS = ('v_est', 'v_enc', 'v_pose', 'v_target', 'u_cmd', 'throttle', 'steering',
                 'ld', 'e_lat', 'e_head', 'kappa', 's', 'slip', 'a_imu', 'delay')

# The simulator's tire and drag, from its source (raceline/VEHICLE_MODEL.md §2-3).
# Forward friction curve: effective extremum (0.15, 0.72), asymptote (0.25, 0.464),
# zero slope at both, flat beyond; Rigidbody linear drag 0.273 /s.
TIRE_S_PEAK, TIRE_MU_PEAK = 0.15, 0.72
TIRE_S_ASYM, TIRE_MU_ASYM = 0.25, 0.464
DRAG_LIN = 0.273
G = 9.81
TRACK_WIDTH = 0.236                     # [m] between left and right contact patches (VEHICLE_MODEL 2)

# What the launch ramp releases towards. It only shapes the tail of the ramp
# between standstill and a_long_launch_v -- above that speed the limit is off
# entirely -- so it just has to sit past anything the tyres can deliver.
# Measured peak true acceleration across the logged runs is 9.24 m/s^2.
A_LONG_RELEASE = 6.0

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

        # Take the pose from TF instead of from `pose_topic`.
        #
        # /amcl_pose is published only when the filter RESAMPLES, so it is
        # motion-triggered, steps discontinuously, and stops entirely when the
        # car is stationary. AMCL's real output is the map->odom correction,
        # which it broadcasts continuously with transform_tolerance padding;
        # composing that with dead_reckoning's 50 Hz odom->base gives a pose
        # that is smooth AND drift-free. Neither half is both.
        #
        # Leave False to keep reading `pose_topic` (the /odom development path).
        p('use_tf_pose', False)
        p('map_frame', 'map')
        p('base_frame', 'roboracer_1')
        p('odom_frame', 'odom')                # intermediate frame of the localizer's correction

        # Lookahead grows with speed: too short oscillates, too long cuts corners.
        # Ld = k * v_est, clamped. If max < min the clamp would silently return
        # max for every speed (numpy applies min first), which is how a run with
        # lookahead_max:=0.70 became a FIXED 0.70 m lookahead; now it warns and
        # lowers min instead, so the intent -- "a short one" -- is at least explicit.
        p('lookahead_min', 0.45)
        p('lookahead_max', 1.30)
        p('lookahead_k', 0.35)                 # Ld = k * speed, then clamped
        p('lookahead_curv_gain', 0.67)         # Ld /= (1 + gain * max |kappa| within Ld); 0 = off
        p('steer_a_lat_max', 7.0)              # cap |kappa_cmd| at this / v^2 [m/s^2]; 0 = off
        p('lookahead_sag_frac', 0.5)           # chord sagitta <= frac * inside margin; 0 = off
        p('lookahead_delay_ref', 0.175)        # k and max scale by cmd_delay / this; 0 = off
        # ---- derating on a slow loop ------------------------------------
        # The profile's lateral limit was proven on a 175 ms round trip. A slower
        # machine (the evaluation box is unknown; this one decayed from 17.3 to
        # 12.8 Hz within a session) loses phase margin and tracking, and the
        # same profile then meets walls it cleared before. Corner speed goes as
        # sqrt(a_lat), so the target is scaled by sqrt(a_lat_eff / a_lat_profile)
        # with a_lat_eff sliding from the profile's own limit at derate_delay_from
        # to derate_a_lat at derate_delay_to. The profile's limit is read from
        # the CSV itself as max(v^2 |kappa|). A slow machine gets a slower clean
        # lap instead of a hit; a fast one is untouched.
        p('derate_delay_from', 0.175)
        p('derate_delay_to', 0.21)
        p('derate_a_lat', 6.0)

        # From the competition vehicle spec, not guessed.
        p('wheelbase', 0.3240)                 # [m]
        p('max_steer_rad', 0.5236)             # [rad] = 30 deg at command 1.0
        p('steering_gain', 1.0)                # empirical trim; 1.0 = trust the model

        # Speed from path curvature: v = sqrt(a_lat_max / |kappa|), clamped.
        p('a_lat_max', 3.0)
        p('v_min', 1.0)
        p('v_max', 4.0)
        # ---- speed source ---------------------------------------------
        # 'tire' (default), 'fused' or 'encoder'. See the module docstring: the
        # encoder is the throttle echo in this simulator, so 'tire' runs the
        # sim's own tire model on the measured wheel speed to recover the car's
        # speed, which is bounded-error by construction. 'fused' (IMU integral +
        # pose blend) is kept for reference only; it failed twice in the sim.
        p('speed_source', 'tire')
        # 1: the observer simulates one wheel at the car's speed. 4: four contact
        # speeds -- inner/outer offset by yaw_rate * track/2, fronts faster by
        # 1/cos(steer) along their steered direction -- all spun at the same u.
        # The curve is concave, so four wheels at spread slips deliver LESS force
        # than one at the mean slip, and the one-wheel model reads high exactly
        # where that spread is large: ICRA run 13, +0.15-0.19 m/s at the hairpin
        # apexes and 0 on the straights, the car 10 % slower than the follower
        # believed. Replayed offline on that log, 4 takes the apex bias to ~0,
        # and did so on the car (run 14: +0.02, p90 0.14). It STAYS 1: no lap
        # gain, and with the true speed in hand the follower asked for more out
        # of the left-leg apex at full lock (exit slip +60 %), the front tires
        # lost lateral grip to it, and the car understeered into the exit wall.
        # The one-wheel optimism was doubling as the throttle limiter there; the
        # principled replacement is a friction-circle scaling of slip_accel.
        # Float so it passes through the launch files like the other tunables.
        p('observer_wheels', 1.0)
        # Initial slope of the rising branch of the friction curve, as a multiple
        # of the smoothstep's (which is zero). Unity does not document it; the
        # top-speed datum implies ~2 and the offline fit against ground truth
        # improves monotonically up to 3 (p90 0.28 -> 0.17 m/s on the legacy laps).
        p('tire_rise_slope', 3.0)
        p('imu_topic', f'{NS}/imu')
        # Between localizer updates the pose is dead reckoning, i.e. the encoders,
        # so over 0.25 s the pose speed WAS the wheel speed (measured 1.15x under
        # acceleration, 0.85x braking) and correcting toward it fed the slip band
        # back on itself until the car stopped. Over 1 s the localizer's
        # corrections dominate the displacement. Small symmetric gain; the IMU
        # integral is exact in the sim, so this only has to hold drift.
        p('pose_speed_window', 1.0)            # s
        p('pose_speed_gain', 0.15)             # per new pose sample
        p('pose_corr_max', 0.15)               # m/s per sample; replaces outlier rejection
        # Propagate the pose forward by the loop delay before steering on it.
        # 0 = off; the lookahead was tuned with the delay in, so change together.
        p('latency_comp_s', 0.05)

        # ---- throttle -------------------------------------------------
        # 'slip' (default): wheel speed = target, clamped to the tire's peak
        # slip band around v_est. 'legacy': ff*v_t + kp*(v_t - speed) plus the
        # launch ramp below -- kept for A/B against the old runs.
        p('throttle_mode', 'slip')
        p('u_per_throttle', 25.25)             # m/s of wheel speed per unit throttle (sim source)
        # The curve peaks at 0.15 both ways, but the band edge is a SATURATION:
        # it binds only at launch, restarts and when the target is far from the
        # car, and at 0.15 it braked at 7 m/s^2 into every corner where the
        # profile planned 4.55, so the car arrived a metre per second slow and
        # re-accelerated inside the corner. 0.12 / 0.10 sit on the rising branch
        # near the profile's own limits; in normal running u ~ v_target anyway.
        # With the command-delay prediction below the commanded slip is close to
        # the slip the wheel actually sees, so these are real tire slips again:
        # 0.08 is ~5.5 m/s^2 on the fitted curve, above the profile's 4.55 and
        # under the 7.06 peak, with the encoders about 0.3 m/s high while it binds.
        p('slip_accel', 0.08)
        p('slip_brake', 0.08)
        # Loop delay from publishing a throttle to seeing it on the wheel. MEASURED
        # 0.15 s (see _throttle_slip). Everything in the band is predicted this far
        # ahead with the observer's acceleration. 0 = the old behaviour.
        p('cmd_delay_s', 0.175)
        # Measure the delay online instead of assuming it. The round trip is the
        # simulator's frame rate, which is machine- and settings-dependent:
        # three frames: 170-175 ms at 17.5 Hz on the development machine, 190-210
        # ms at 13-14.6 Hz as a long session decayed, and the laps moved from 6.65
        # to 6.85 with it because the band was placed early. The evaluation
        # machine will be faster.
        # Every cmd_delay_update_s the last cmd_delay_window_s of (throttle,
        # wheel speed) pairs are cross-correlated at lags 0..0.35 s and the best
        # lag is low-pass blended into cmd_delay_s. Needs throttle to vary, which
        # any lap provides. Off = the fixed value above.
        p('cmd_delay_auto', True)
        p('cmd_delay_window_s', 6.0)
        p('cmd_delay_update_s', 2.0)
        # Proportional gain on (v_target - v_land) added to the wheel command inside
        # the band; 0.76 = 25.25 * throttle_kp, the legacy law's effective gain.
        p('slip_kp', 0.76)
        p('u_launch', 0.4)                     # wheel-speed floor from rest [m/s]
        # PhysX computes longitudinal slip as (u - v) / max(|v|, minLongSlipDenominator)
        # with a default of 4 m/s (VEHICLE_MODEL.md §3.1). Below 4 m/s the band is
        # therefore +-slip*4 m/s of WHEEL SPEED, not a percentage: peak grip from
        # rest needs u = v + 0.6. Measured with a percentage band: the car sat at
        # 0.5-0.9 m/s for five seconds (227 logged samples at u ~ 1.15 v, a ~ 0).
        p('v_slip_den', 4.0)
        # The sim's IMU differentiates the WORLD velocity and rotates it into the
        # body, so a_x = dv/dt - omega_z * v_y. At the centre of mass v_y is about
        # lever * omega_z, so the integrator adds lever * omega_z^2 back: up to
        # 1.1 m/s^2 in this track's corners, which is where v_est used to collapse.
        p('imu_lever_arm', 0.155)              # m, CoM ahead of the rear axle; 0 disables

        # ---- legacy launch traction limit ------------------------------
        # Only used with throttle_mode=legacy. It capped how fast the TARGET
        # could rise while the car was barely moving, because a standing start
        # without it commanded 120 m/s^2 and the encoders read 12.5x the
        # distance (dead reckoning 3.2 m out by t=3 s, wall at t=4 s, with the
        # map fitting perfectly). Measured slip by starting speed, 1620 windows:
        #     from 0.0-0.5 m/s  median 5.32x     from 2.5-4.0 m/s  median 1.13x
        # It read the ENCODER speed, so it degenerated into an open-loop ramp
        # that happened to sit near what the tire delivers, and it could not
        # cover a restart after a wall respawn (the wheels read > 2 m/s while
        # the car sat still). The slip band above supersedes it.
        p('a_long_launch', 0.6)                # m/s^2 allowed at standstill
        p('a_long_launch_v', 2.0)              # above this speed, no limiting
        p('curvature_preview_m', 1.0)          # curvature FALLBACK only: worst corner within this distance
        p('lead_min_m', 0.10)                  # floor on the profile lead (v * cmd_delay_s) when slow
        # Extra preview for the SPEED target only, on top of the measured
        # throttle-to-wheel delay. The wheel lands on the command after cmd_delay,
        # the car's deceleration follows the wheel as slip builds, and the tire is
        # at its limit in R1 and the S entry at the 7.0 rung: arriving there at the
        # profile speed runs wide and the longer path costs more than braking a
        # little early. Measured: 0 = 6.70/6.78 (run 26), 0.05 = 6.65/6.70 (runs 25,
        # 27), 0.08 = 6.60/6.69 with the corner bias halved (run 28), same steering.
        # Independent of latency_comp_s (steering).
        p('target_lead_s', 0.08)
        # If the CSV carries a v_mps column (8th), follow THAT instead of
        # re-deriving speed from curvature. The optimizer's profile accounts for
        # the traction ellipse and for braking before a corner; the fallback
        # below does neither, so comparing optimized paths without this compares
        # only their geometry.
        p('use_path_speed', True)

        p('throttle_kp', 0.030)                # legacy mode
        p('throttle_ff', 0.040)                # legacy mode; = 1/25.25, i.e. it WAS the physics
        p('throttle_max', 0.20)
        # False = speed from the pose topic's odometry twist: ground truth, DEV ONLY.
        p('use_encoder_speed', True)
        # MEASURED 0.0581, not the published 0.0590 (see dead_reckoning.py).
        # Was hardcoded in _cb_enc; a 1.55% high speed reading biases the
        # throttle feedforward low as well as corrupting dead reckoning.
        p('wheel_radius', 0.0581)
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
        self.use_tf_pose = bool(g('use_tf_pose'))
        self.map_frame, self.base_frame = g('map_frame'), g('base_frame')
        self.odom_frame = g('odom_frame')
        # The path and lookahead marker have to be drawn in the frame the poses
        # actually live in, or RViz drops them. Under race.launch.py the devkit's
        # `world` frame is remapped off /tf entirely and no longer exists.
        self.viz_frame = self.map_frame if self.use_tf_pose else g('viz_frame')
        self.tf_buffer = None
        self.tf_listener = None
        self.tf_warned = False
        self.ld_min, self.ld_max, self.ld_k = g('lookahead_min'), g('lookahead_max'), g('lookahead_k')
        self.ld_curv_gain = float(g('lookahead_curv_gain'))
        self.steer_a_lat_max = float(g('steer_a_lat_max'))
        self.ld_sag_frac = float(g('lookahead_sag_frac'))
        self.ld_delay_ref = float(g('lookahead_delay_ref'))
        self.derate_from = float(g('derate_delay_from'))
        self.derate_to = float(g('derate_delay_to'))
        self.derate_a_lat = float(g('derate_a_lat'))
        self.wheelbase = g('wheelbase')
        self.max_steer = g('max_steer_rad')
        self.steer_gain = g('steering_gain')
        self.a_lat, self.v_min, self.v_max = g('a_lat_max'), g('v_min'), g('v_max')
        self.hz = float(g('control_hz'))
        self.a_long_launch = float(g('a_long_launch'))
        self.a_long_launch_v = float(g('a_long_launch_v'))
        self._v_cmd = 0.0               # rate-limited speed target, m/s
        self._v_cmd_t = None            # when it was last updated
        self.preview = g('curvature_preview_m')
        self.lead_min = float(g('lead_min_m'))
        self.target_lead = float(g('target_lead_s'))
        self.use_path_speed = g('use_path_speed')
        self.kp, self.ff, self.thr_max = g('throttle_kp'), g('throttle_ff'), g('throttle_max')
        self.use_enc = g('use_encoder_speed')
        self.wheel_r = g('wheel_radius')
        self.speed_source = str(g('speed_source')).lower()
        self.throttle_mode = str(g('throttle_mode')).lower()
        self.obs_wheels = int(round(float(g('observer_wheels'))))
        if self.obs_wheels not in (1, 4):
            raise RuntimeError(f'observer_wheels must be 1 or 4, not {self.obs_wheels}')
        if self.speed_source not in ('tire', 'fused', 'encoder'):
            raise RuntimeError(f"speed_source must be 'tire', 'fused' or 'encoder', not {self.speed_source!r}")
        self.tire_m = float(g('tire_rise_slope'))
        self.cmd_delay = float(g('cmd_delay_s'))
        self.slip_kp = float(g('slip_kp'))
        self.delay_auto = bool(g('cmd_delay_auto'))
        self.delay_win = float(g('cmd_delay_window_s'))
        self.delay_every = float(g('cmd_delay_update_s'))
        self._thr_hist = []             # (t, throttle published)
        self._enc_hist = []             # (t, wheel speed measured)
        self._delay_next = None
        self.delay_meas = float('nan')  # last raw estimate, for the status topic
        self._obs_t = None              # last observer step
        self.a_est = 0.0                # observer's acceleration, m/s^2
        self.v_land = 0.0               # speed the slip band was placed around
        if self.throttle_mode not in ('slip', 'legacy'):
            raise RuntimeError(f"throttle_mode must be 'slip' or 'legacy', not {self.throttle_mode!r}")
        self.u_per_thr = float(g('u_per_throttle'))
        self.slip_accel, self.slip_brake = float(g('slip_accel')), float(g('slip_brake'))
        self.u_launch = float(g('u_launch'))
        self.v_slip_den = float(g('v_slip_den'))
        self.imu_lever = float(g('imu_lever_arm'))
        self._imu_yaw = None            # last IMU heading, for teleport detection
        self.latency = float(g('latency_comp_s'))
        self.pose_win = float(g('pose_speed_window'))
        self.pose_gain = float(g('pose_speed_gain'))
        self.pose_corr_max = float(g('pose_corr_max'))
        self.a_imu = float('nan')       # last raw IMU a_x, published for diagnosis
        # fused speed estimate
        self.v_est = 0.0                # IMU-integrated, pose-corrected car speed
        self.v_enc = 0.0                # wheel surface speed from the encoders
        self.v_pose = float('nan')      # speed implied by the pose over pose_win
        self.yaw_rate = 0.0             # IMU gyro z, for latency compensation
        self.steer_angle = 0.0          # last commanded steer [rad], for the 4-wheel observer
        self._imu_t = None
        self._imu_seen = False
        self._pose_hist = []            # (stamp, x, y, v_est) of the poses steered on
        self._pose_new = False
        self.u_cmd = 0.0                # wheel speed commanded this tick
        if self.ld_max < self.ld_min:
            self.get_logger().warn(
                f'lookahead_max {self.ld_max} < lookahead_min {self.ld_min}: lowering min to '
                f'match. (Before this check the clamp returned max for EVERY speed.)')
            self.ld_min = self.ld_max
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
        self.psi, self.kappa = data[:, 3], data[:, 4]
        # Room to the wall on the INSIDE of each point's curve, minus half the car:
        # what a corner-cutting chord may consume. The CSV's width columns are
        # ray-cast from the map at the line (optimize_raceline.export_csv).
        half_w = 0.135
        self.margin_in = np.where(self.kappa > 0.0, data[:, 6], data[:, 5]) - half_w
        self.path_v = data[:, 7] if data.shape[1] > 7 else None
        # The profile's own longitudinal acceleration, v dv/ds, closed loop. Used to
        # place the slip band where the car will be when a command lands.
        if self.path_v is not None:
            ds = np.hypot(np.roll(self.px, -1) - np.roll(self.px, 1), np.roll(self.py, -1) - np.roll(self.py, 1))
            self.path_a = self.path_v * (np.roll(self.path_v, -1) - np.roll(self.path_v, 1)) / np.maximum(ds, 1e-3)
            # the profile's own lateral limit, for derating on a slow loop
            self.profile_a_lat = float(np.max(self.path_v ** 2 * np.abs(self.kappa)))
        else:
            self.path_a = np.zeros_like(self.px)
            self.profile_a_lat = 0.0
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
        self._logged_lap = -1

        if self.use_tf_pose:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.get_logger().info(
                f'pose from TF {self.map_frame} -> {self.base_frame} '
                f'(continuous); {self.pose_topic} kept as a fallback')

        if 'odom' in self.pose_topic:
            self.create_subscription(Odometry, self.pose_topic, self._cb_odom, QOS)
        else:
            self.create_subscription(PoseWithCovarianceStamped, self.pose_topic,
                                     self._cb_amcl, QOS)

        # Say out loud what the car is actually steering on. `pose_topic`
        # defaults to the devkit's odometry, which is simulator GROUND TRUTH --
        # and for a long time nothing anywhere printed that, so development runs
        # produced lap times that read as race-legal and were not.
        if restricted.is_restricted(self.pose_topic):
            if self.use_tf_pose:
                self.get_logger().warn(
                    f'{self.pose_topic} is RESTRICTED and is subscribed as a '
                    f'fallback only -- steering on TF {self.map_frame} -> '
                    f'{self.base_frame}. Legal as long as the TF lookup keeps '
                    'succeeding; a fallback WOULD silently make this run illegal.')
            else:
                restricted.warn(self, self.pose_topic,
                                'THIS IS THE STEERING SOURCE')
        else:
            self.get_logger().info(f'steering on {self.pose_topic}')

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
        # IMU: legal, and the only onboard signal that is not the throttle echo.
        self.create_subscription(Imu, g('imu_topic'), self._cb_imu, QOS)
        self.get_logger().info(
            f'speed_source={self.speed_source}  throttle_mode={self.throttle_mode}  '
            f'slip band [-{self.slip_brake:g}, +{self.slip_accel:g}]  u_launch {self.u_launch:g} m/s  '
            f'lookahead {self.ld_min:g}-{self.ld_max:g} m (k {self.ld_k:g})  '
            f'latency_comp {self.latency:g} s')
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
        self.pub_status = self.create_publisher(Float32MultiArray, '~/status', 1)

        self.create_timer(1.0 / g('control_hz'), self._control)
        self.create_timer(2.0, self._publish_path)

    # ---- callbacks -------------------------------------------------------

    @staticmethod
    def _stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _cb_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        if not self.use_tf_pose:
            self._note_pose(self._stamp(msg.header), p.x, p.y)
        if not self.use_enc:
            t = msg.twist.twist.linear
            self.speed = math.hypot(t.x, t.y)

    def _cb_amcl(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        if not self.use_tf_pose:
            self._note_pose(self._stamp(msg.header), p.x, p.y)

    def _cb_truth(self, msg):
        p = msg.pose.pose.position
        self.truth_pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        if self.using_truth:
            self._note_pose(self._stamp(msg.header), p.x, p.y)
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
        self._enc_rate[side] = (ang - prev[0]) / dt * self.wheel_r
        rates = [v for v in self._enc_rate.values() if v is not None]
        if rates:
            self.v_enc = float(np.mean(rates))
            if self.delay_auto and side == 'l':
                self._enc_hist.append((t, self.v_enc))
            # The encoder drives the control speed only when asked to, or when
            # no IMU has ever arrived and there is nothing better.
            if self.use_enc and (self.speed_source == 'encoder'
                                 or (self.speed_source == 'fused' and not self._imu_seen)):
                self.speed = self.v_enc

    def _cb_imu(self, msg):
        """Integrate longitudinal acceleration into v_est.

        IMU.cs computes (v_world(t) - v_world(t-dt))/dt and rotates it into the
        body frame, with no gravity term (GravityCompensation defaults on). The
        x component is therefore dv/dt - omega_z * v_y, not dv/dt: in a corner
        the lateral velocity of the centre of mass couples in. With the
        kinematic v_y ~ lever * omega_z the missing term is lever * omega_z^2,
        added back here. The pose correction in _fuse_speed holds what is left.

        A heading jump of more than 20 deg between two messages 55 ms apart is
        physically impossible (3.2 rad/s of steering is 10 deg) and is the one
        onboard signature of a wall respawn: the sim zeroes the velocity in a
        single 1 ms step this integral never sees, and the dead-reckoned pose
        keeps moving on the spinning wheels, so nothing else would ever bring
        v_est back down.
        """
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:                              # unstamped: fall back to arrival time
            t = self.get_clock().now().nanoseconds * 1e-9
        yaw = yaw_from_quat(msg.orientation)
        if self._imu_yaw is not None and self._imu_t is not None:
            # A jump the measured yaw rate cannot account for: a late tick at
            # 2.7 rad/s is 20 deg too, so compare against rate * dt, not zero.
            dyaw = (yaw - self._imu_yaw + math.pi) % (2.0 * math.pi) - math.pi
            dyaw -= self.yaw_rate * max(0.0, min(t - self._imu_t, 0.3))
            if abs(dyaw) > 0.35:
                self.get_logger().warn(
                    f'heading jumped {math.degrees(dyaw):+.0f} deg in one tick: respawn. '
                    f'Speed estimate {self.v_est:.2f} -> 0, pose history cleared.')
                self.v_est = 0.0
                self._pose_hist.clear()
        self._imu_yaw = yaw
        self.yaw_rate = float(msg.angular_velocity.z)
        self.a_imu = float(msg.linear_acceleration.x)
        if self.speed_source == 'fused' and self._imu_t is not None:
            dt = t - self._imu_t
            if 0.0 < dt < 0.5:
                a = self.a_imu + self.imu_lever * self.yaw_rate ** 2
                self.v_est = max(0.0, self.v_est + a * dt)
        self._imu_t = t
        self._imu_seen = True

    def _mu(self, S):
        """Sim friction coefficient at longitudinal slip S (either sign)."""
        a = abs(S)
        if a <= TIRE_S_PEAK:
            t = a / TIRE_S_PEAK
            return TIRE_MU_PEAK * ((3.0 * t * t - 2.0 * t ** 3) + self.tire_m * (t - 2.0 * t * t + t ** 3))
        if a <= TIRE_S_ASYM:
            t = (a - TIRE_S_PEAK) / (TIRE_S_ASYM - TIRE_S_PEAK)
            return TIRE_MU_PEAK - (TIRE_MU_PEAK - TIRE_MU_ASYM) * (3.0 * t * t - 2.0 * t ** 3)
        return TIRE_MU_ASYM

    def _observe(self, now):
        """Advance the tire-model speed observer to `now` on the measured wheel speed.

        v' = sign(S) mu(|S|) g - 0.273 v,  S = (u - v)/max(v, 4 m/s)

        All four wheels are driven at the same u, so the total force is mu m g
        and the mass cancels. Sub-stepped: near zero slip the curve's slope
        makes the linearised rate up to ~25 /s, and a 50 ms Euler step would
        sit at the edge of stability. If the estimate is high the slip it sees
        is too small, the force too small, and it falls back toward the car;
        if low, the reverse -- that is the contraction that bounds the error.
        """
        if self._obs_t is None:
            self._obs_t = now
            return
        dt = now - self._obs_t
        self._obs_t = now
        if not 0.0 < dt < 0.5:
            return
        u, h = self.v_enc, dt / 5.0
        for _ in range(5):
            if self.obs_wheels == 4:
                # Four contact speeds along each wheel's rolling direction, one
                # wheel speed u, equal loads: a = (g/4) sum mu(S_i). See the
                # observer_wheels parameter for why this differs from one wheel.
                w = self.yaw_rate * TRACK_WIDTH / 2.0
                vf = self.v_est / max(math.cos(self.steer_angle), 0.5)
                a = 0.25 * sum(self._wheel_accel(u, vc)
                               for vc in (self.v_est - w, self.v_est + w, vf - w, vf + w))
                a -= DRAG_LIN * self.v_est
            else:
                a = self._wheel_accel(u, self.v_est) - DRAG_LIN * self.v_est
            self.v_est = max(0.0, self.v_est + a * h)
        self.a_est = a                            # for the command-delay prediction

    def _wheel_accel(self, u, vc):
        """g * mu at one wheel: rim speed u, contact-patch speed vc (PhysX slip)."""
        S = (u - vc) / max(vc, self.v_slip_den)
        return math.copysign(self._mu(S) * G, S)

    def _note_pose(self, t, x, y):
        """Record a pose sample, stamped AT ITS SOURCE, for the speed correction.

        Sampling the pose at the control tick instead quantised the window's
        displacement by up to a whole 18 Hz update and made the pose speed swing
        +-20 %; with source stamps the intervals are exact. Same stamp = same
        sample, skipped.
        """
        if self._pose_hist and t <= self._pose_hist[-1][0]:
            return
        self._pose_hist.append((t, x, y, self.v_est))
        while self._pose_hist and t - self._pose_hist[0][0] > self.pose_win:
            self._pose_hist.pop(0)
        self._pose_new = True

    def _fuse_speed(self):
        """Correct v_est toward the speed the pose implies over pose_win.

        The pose the controller steers on is map->base: the localizer's
        correction composed with dead reckoning. Over a window of several
        localizer updates its displacement is the car's, so it anchors the IMU
        integral. Two details matter:

        * displacement/time is the MEAN speed over the window, so it is compared
          with the mean of v_est over the same window, not with the current
          value. Comparing with the current value biased the estimate low by
          a*T/2 under acceleration (measured 10-15 %), and a 15 % low estimate
          turns the "accelerate" edge of the slip band into a coast.
        * the window has to be long. Between localizer updates the pose is dead
          reckoning, i.e. the encoders: over 0.25 s the pose speed measured
          1.15x the car under acceleration and 0.85x under braking, exactly the
          slip band, and correcting toward it (fast, downward) fed the band back
          on itself until the car stopped inside the S-curve. Over 1 s the
          localizer's corrections dominate the displacement.
        * nothing is ever rejected; the correction is CLAMPED per sample instead.
          A rejection threshold deadlocked the car: with v_est stuck at 0 the
          band commanded u = 0.6 m/s, the car settled at 0.6 m/s where the slip
          is zero, the pose speed said 0.6-0.8, and 0.6 > 0.5 was "an outlier"
          -- for up to 30 s at a time, 70-80 % of three runs. A clamp of
          pose_corr_max per sample (~20 samples/s) still limits what a
          wheelspin spike in the pose speed can do to the estimate while
          guaranteeing any persistent disagreement closes within a second.
          Respawns are additionally caught in _cb_imu from the heading jump.

        Applied once per new pose sample, never twice for the same one.
        """
        if not self._pose_new or len(self._pose_hist) < 3:
            return
        self._pose_new = False
        t0, x0, y0, _ = self._pose_hist[0]
        t1, x1, y1, _ = self._pose_hist[-1]
        span = t1 - t0
        if span < 0.5 * self.pose_win:
            return
        self.v_pose = math.hypot(x1 - x0, y1 - y0) / span
        v_mean = sum(h[3] for h in self._pose_hist) / len(self._pose_hist)
        dv = self.pose_gain * (self.v_pose - v_mean)
        dv = max(-self.pose_corr_max, min(self.pose_corr_max, dv))
        self.v_est = max(0.0, self.v_est + dv)

    def _cb_lap(self, msg):
        if msg.data > self.laps:
            self.laps = msg.data

    def _cb_lap_time(self, msg):
        # The bridge re-publishes last_lap_time EVERY tick, not on change, so
        # logging per message spams ~20 lines/s of the same lap and buries
        # everything else. Log once per completed lap instead.
        if msg.data > 0.0 and self.laps != self._logged_lap:
            self._logged_lap = self.laps
            self.get_logger().info(f'lap {self.laps}: {msg.data:.2f} s')

    # ---- control ---------------------------------------------------------

    def _pose_from_tf(self):
        """Latest map->base, composed from the NEWEST sample of each leg.

        Not a chain lookup at the latest time: the localizer post-dates its
        map->odom correction (nav2 amcl by transform_tolerance, 0.5 s here) so
        that "now" falls inside its window, and tf2 then serves a chain query
        at "now" by interpolating between the corrections computed 0.5 s ago.
        For navigation that is harmless; at 6.5 m/s with 2-4 % odometry
        overread it hands the controller a correction that is half a second
        and ~0.1 m stale, and every heading swing 0.5 s late. Reading each leg
        at its own latest stamp gives the newest correction on top of the
        newest dead reckoning (which extrapolates to its stamp, see
        dead_reckoning.py). Falls back to the chain lookup when the odom leg
        is not there (localizer:=none, or a localizer publishing map->base).
        """
        try:
            pose, stamp = compose_latest(self.tf_buffer, self.map_frame,
                                         self.odom_frame, self.base_frame)
        except Exception as exc:                        # noqa: BLE001
            if not self.tf_warned:
                self.tf_warned = True
                self.get_logger().warn(
                    f'{self.map_frame} -> {self.base_frame} not available yet '
                    f'({exc}); falling back to {self.pose_topic}')
            return None
        if self.tf_warned:
            self.tf_warned = False
            self.get_logger().info(f'{self.map_frame} -> {self.base_frame} is up')
        # The stamp is dead reckoning's, which advances at its publish rate;
        # that is the interval the speed correction differences over.
        if not self.using_truth:
            self._note_pose(stamp, pose[0], pose[1])
        return pose

    def _cb_ready(self, msg):
        if msg.data and not self.ready:
            self.ready = True
            self.get_logger().info('localization converged, taking over')

    def _limit_accel(self, v_target):
        """Hold the speed TARGET down while the car is barely moving.

        Above a_long_launch_v this does nothing at all -- the velocity profile
        is followed exactly as written. Below it, the allowed rise ramps
        linearly from a_long_launch at rest up to A_LONG_RELEASE, which is past
        anything the tyres deliver, so the limit has faded out by the time it
        switches off.

        Upward only: braking is never limited, so the profile can still stop as
        hard as it likes for a corner.
        """
        if self.a_long_launch_v <= 0.0:
            return v_target

        v = max(self.speed, 0.0)
        if v >= self.a_long_launch_v:
            # Keep tracking the target even while unlimited, so dropping back
            # below the threshold (a spin, a wall, a hairpin) resumes from the
            # real speed rather than from a stale command.
            self._v_cmd = v_target
            return v_target

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = (now - self._v_cmd_t) if self._v_cmd_t is not None else (1.0 / self.hz)
        self._v_cmd_t = now
        if not 0.0 < dt <= 0.5:
            dt = 1.0 / self.hz

        frac = v / self.a_long_launch_v
        a_allowed = self.a_long_launch + (A_LONG_RELEASE - self.a_long_launch) * frac

        # Re-anchor to reality before ramping, so the command cannot drift away
        # from the car. min() means wheelspin (which reads HIGH) never raises it.
        self._v_cmd = min(self._v_cmd, v + a_allowed * dt)
        self._v_cmd = min(v_target, self._v_cmd + a_allowed * dt)
        return self._v_cmd

    def _throttle_slip(self, v_target, a_plan=0.0):
        """Command a WHEEL speed, bounded to the tire's peak-force slip band.

        The sim spins the wheel to u = u_per_throttle * throttle regardless of
        the car, so throttle IS a wheel-speed command and PhysX's slip is
        (u - v) / max(v, 4 m/s). Asking for the target speed directly makes
        the slip proportional to the car's lag behind the target, which is the
        right proportional behaviour; clamping u to v +- slip * max(v, 4) caps
        the force demand at the peak of the friction curve both ways and bounds
        the encoder error to that band in every regime (VEHICLE_MODEL.md §3.1,
        §3.2, §5.3). Below 4 m/s the band is +-0.6 m/s of wheel speed, so from
        rest u = 0.6 and the car launches at peak grip; unlike the old launch
        ramp this keys on the CAR's speed, so a restart after a wall respawn is
        the same launch.
        """
        # The wheel reaches 25.25 * throttle with a 0.98 gain but cmd_delay late
        # (regression on run4: RMS 0.18 m/s at 150 ms, 0.59 at zero; at 5 ms
        # resolution the trough is 165-180 ms, three sim frames). A band
        # placed around the CURRENT speed is therefore mostly gone by the time
        # it lands: commanded lead 0.40-0.53 m/s measured as 0.04-0.08 m/s of
        # wheel slip and 1.7 m/s^2 of acceleration against the profile's 3.5-4.
        # So the band is placed around the speed the car will have when the
        # command arrives. The prediction uses the PROFILE's planned acceleration
        # at this point of the path, a static function of position. Predicting
        # with the observer's own acceleration instead closes a loop of delay
        # 0.15 s and gain ~5 (the curve's initial slope times the delay) and
        # oscillated at 1 Hz in the wire test, the car stuck at 0.5-1.4 m/s.
        # If the car accelerates less than planned the slip that lands is larger
        # and the tire pushes harder: negative feedback, the right way round.
        # Only a planned acceleration that points toward the target is used: a
        # car put back into a braking zone by a respawn must accelerate although
        # the plan there says brake, and with the plan's sign the band collapsed
        # onto the launch floor and held the car at 0.4 m/s for good.
        v = max(self.speed, 0.0)
        a_pred = max(0.0, min(6.0, a_plan)) if v_target >= v else min(0.0, max(-6.0, a_plan))
        v_land = max(v + a_pred * self.cmd_delay, 0.0)
        den = max(v_land, self.v_slip_den)
        # Proportional action inside the band. The legacy law was
        # u = 25.25 (ff v_t + kp (v_t - v_enc)) = v_t + 0.76 (v_t - v): a lead of
        # 1.76x the speed error. Commanding u = v_t alone tracked the target 0.15
        # m/s low against legacy's 0.07 (run 5 vs run 4); the same gain, applied
        # to the error at landing time, closes that. The band still bounds the slip.
        u = v_target + self.slip_kp * (v_target - v_land)
        u = min(max(u, v_land - self.slip_brake * den), v_land + self.slip_accel * den)
        if v_target > v and u < self.u_launch:
            u = self.u_launch
        self.v_land = v_land
        self.u_cmd = u
        return float(np.clip(u / self.u_per_thr, 0.0, self.thr_max))

    def _update_delay(self, now):
        """Cross-correlate published throttle against measured wheel speed.

        The wheel follows 25.25 * throttle with gain ~0.98 after one round trip
        (VEHICLE_MODEL.md §3.6). Over the last window, for each candidate lag,
        the RMS of wheel_speed(t) - 25.25 * throttle(t - lag) is evaluated with
        linear interpolation of the throttle history; the lag with the smallest
        RMS is the delay. Blended at 0.3 per update so a single noisy window
        cannot move the band by more than a few tens of milliseconds.
        """
        if self._delay_next is not None and now < self._delay_next:
            return
        self._delay_next = now + self.delay_every
        t_min = now - self.delay_win
        self._thr_hist = [p for p in self._thr_hist if p[0] >= t_min - 0.5]
        self._enc_hist = [p for p in self._enc_hist if p[0] >= t_min]
        if len(self._thr_hist) < 20 or len(self._enc_hist) < 20:
            return
        tt = np.array([p[0] for p in self._thr_hist]); ut = np.array([p[1] for p in self._thr_hist]) * self.u_per_thr
        te = np.array([p[0] for p in self._enc_hist]); ve = np.array([p[1] for p in self._enc_hist])
        if ut.std() < 0.3:                          # throttle barely moved: nothing to correlate
            return
        lags = np.arange(0.0, 0.36, 0.025)
        rms = [math.sqrt(np.mean((ve - np.interp(te - lag, tt, ut)) ** 2)) for lag in lags]
        best = float(lags[int(np.argmin(rms))])
        self.delay_meas = best
        self.cmd_delay += 0.3 * (best - self.cmd_delay)

    def _publish_status(self, values):
        m = Float32MultiArray()
        m.data = [float(v) for v in values]
        self.pub_status.publish(m)

    def _control(self):
        if not self.ready:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        pose = self.pose
        if self.use_tf_pose:
            tf_pose = self._pose_from_tf()
            if tf_pose is not None:
                pose = self.pose = tf_pose
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

        # Car speed: the tire observer by default; the IMU/pose fusion or the raw
        # encoder when asked (encoder is what the follower used until 2026-09-04).
        if self.use_enc and self.speed_source == 'tire':
            self._observe(now)
            self.speed = self.v_est
        elif self.use_enc and self.speed_source == 'fused' and self._imu_seen:
            self._fuse_speed()
            self.speed = self.v_est

        # Steer on where the car will be when the command lands, not where it was.
        # The position moves along the MID-interval heading (the chord of the
        # arc the car is on); the heading itself advances by the gyro rate.
        # Until 2026-09-04 the localizer's estimate carried a hidden lead of
        # 25-50 ms x speed (dead_reckoning extrapolate_pos), which this tuning
        # was done against; with an honest estimate this is where the lead
        # belongs, explicitly.
        x0, y0 = x, y
        if self.latency > 0.0:
            mid = yaw + 0.5 * self.yaw_rate * self.latency
            x += self.speed * math.cos(mid) * self.latency
            y += self.speed * math.sin(mid) * self.latency
            yaw += self.yaw_rate * self.latency

        d = np.hypot(self.px - x, self.py - y)
        near = int(np.argmin(d))
        # The speed target is placed from the UNPROPAGATED pose: the throttle
        # has its own preview (cmd_delay, measured throttle-to-wheel), and
        # adding the steering propagation on top braked 50 ms early at every
        # corner: run 25, braking zones 9 % under the profile against 5 % in
        # run 24, 0.05 s a lap.
        near_v = near if self.latency <= 0.0 else int(np.argmin(np.hypot(self.px - x0, self.py - y0)))

        # Phase margin of the pursuit loop is set by v * delay / Ld, so when the
        # measured round trip changes the lookahead must change with it. The
        # yaml values were tuned at 175 ms (17.3 Hz sim tick); at 14.1 Hz the
        # delay is 210 ms and the same 2.2 m lookahead weaved into a wall 0.33 m
        # from the line at 5 m/s (yaw -6.7 -> +4.6 deg in one second). Corners
        # are unaffected: the curvature and sagitta rules below still bound it.
        # Capped at 1.2: with the old 0.15 reference the uncapped 1.33 at 200 ms
        # gave a 2.9 m lookahead whose
        # lateral gain (2/Ld^2) was too soft to catch a 0.1 m drift with 5 deg of
        # heading before a wall 0.33 m away, twice, on the straight after R1.
        d_scale = 1.0
        if self.ld_delay_ref > 0.0:
            d_scale = max(0.7, min(1.2, self.cmd_delay / self.ld_delay_ref))
        ld = float(np.clip(self.ld_k * d_scale * abs(self.speed), self.ld_min, self.ld_max * d_scale))
        # Shorten the lookahead where the path AHEAD curves. Pure pursuit aims at
        # the chord to the lookahead point, so on corner entry it cuts inside by
        # about the chord's sagitta, kappa * Ld^2 / 8: 0.26 m at kappa 0.8 and
        # Ld 1.6, against 0.145 m of margin to the inside wall. Measured: 0.18 m
        # inside at s = 26.7 m and a wall hit, at the profile's own speed. The
        # long lookahead is only needed for phase margin at high speed, and the
        # car is slow where the path curves, so scale by the worst curvature
        # within the nominal lookahead: at kappa 0.9 with gain 0.67, 1.6 -> 1.0 m.
        if self.ld_curv_gain > 0.0:
            step = self.lap_len / len(self.px)
            span = max(1, int(ld / step))
            k_ahead = max(abs(self.kappa[(near + i) % len(self.px)]) for i in range(span + 1))
            ld = max(self.ld_min, ld / (1.0 + self.ld_curv_gain * k_ahead))
        # And bound the cut directly: the chord to the lookahead point sags inside
        # the curve by |kappa| Ld^2 / 8, and that must stay under a fraction of the
        # room the line has on the inside. Curvature scaling alone missed the
        # S-exit left-hander: kappa only 0.36, so 2.2 m scaled to 1.74 m, but its
        # inside wall is 0.28 m from the line and the car went 0.20 m inside.
        # Ld <= sqrt(8 f margin / |kappa|): 0.8 m at kappa 0.9, 1.3 m at 0.36
        # with 0.145 m of margin; untouched on straights.
        if self.ld_sag_frac > 0.0:
            n_pts = len(self.px)
            step = self.lap_len / n_pts
            span = max(1, int(ld / step))
            for i in range(span + 1):
                p_i = (near + i) % n_pts
                k_i = abs(self.kappa[p_i])
                if k_i > 1e-3:
                    ld = min(ld, math.sqrt(8.0 * self.ld_sag_frac * max(self.margin_in[p_i], 0.02) / k_i))
            ld = max(self.ld_min, ld)

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
        # Never ask the front tires for more lateral acceleration than they can
        # give. The sideways curve peaks at 1.0 g at 0.57 deg of slip and falls
        # to 0.5 g by 5.7 deg (VEHICLE_MODEL.md §3.2), so once the car runs wide
        # a geometric controller's answer, MORE steering, is the wrong sign: the
        # front slip angle grows, the force falls, the car runs wider. Measured
        # at the 6.5 rung: 0.2 -> 0.9 m wide over one second at full lock with
        # 3.4-4.4 m/s^2 of lateral against 6.0 demanded. Capping the commanded
        # curvature at a_cap / v^2 holds the steering where the force is still
        # near its maximum and lets the car run wide by the physical amount
        # instead of spiralling. Measured usable maxima: 6.4-7.8 m/s^2.
        if self.steer_a_lat_max > 0.0 and self.speed > 0.5:
            k_cap = self.steer_a_lat_max / (self.speed ** 2)
            kappa_cmd = max(-k_cap, min(k_cap, kappa_cmd))
        delta = math.atan(kappa_cmd * self.wheelbase)          # bicycle model
        steering = float(np.clip(self.steer_gain * delta / self.max_steer, -1.0, 1.0))
        self.steer_angle = steering * self.max_steer

        step = self.lap_len / n

        if self.use_path_speed and self.path_v is not None:
            # Target = the profile's speed at the point the car reaches when THIS
            # command acts, i.e. cmd_delay_s ahead, plus a small floor.
            #
            # It used to be the MINIMUM of the profile over a fixed 1 m preview.
            # That reaches corner speed a metre early at every entry and holds
            # it there: about 0.11 s per corner, 0.55 s a lap. Measured: the
            # profile followed with that preview predicts 7.89 s and the car
            # drove 7.90; followed at the delay-matched point it predicts 7.35.
            # The profile already contains its own braking distances, so the
            # follower's only job is to land the wheel on the profile's speed at
            # the profile's position.
            lead = max(self.speed, 0.0) * (self.cmd_delay + self.target_lead) + self.lead_min
            j = (near_v + int(lead / step)) % n
            v_target = float(self.path_v[j])
            # Derate on a slow loop: see derate_delay_from.
            if self.derate_to > self.derate_from and self.profile_a_lat > self.derate_a_lat:
                f = (self.cmd_delay - self.derate_from) / (self.derate_to - self.derate_from)
                a_eff = self.profile_a_lat + (self.derate_a_lat - self.profile_a_lat) * max(0.0, min(1.0, f))
                v_target *= math.sqrt(a_eff / self.profile_a_lat)
            v_target = float(np.clip(v_target, self.v_min, self.v_max))
        else:
            # Curvature fallback has no braking distances of its own, so it
            # still needs to look for the worst corner within the preview.
            span = max(1, int(self.preview / step))
            window = [(near_v + i) % n for i in range(span)]
            k_worst = max(abs(self.kappa[i]) for i in window)
            v_target = float(np.clip(math.sqrt(self.a_lat / max(k_worst, 1e-3)),
                                     self.v_min, self.v_max))

        if self.throttle_mode == 'slip':
            throttle = self._throttle_slip(v_target, float(self.path_a[near]))
        else:
            v_target = self._limit_accel(v_target)
            err = v_target - self.speed
            throttle = float(np.clip(self.ff * v_target + self.kp * err, 0.0, self.thr_max))
            self.u_cmd = throttle * self.u_per_thr
            self.v_land = max(self.speed, 0.0)

        t, s = Float32(), Float32()
        t.data, s.data = throttle, steering
        self.pub_t.publish(t)
        self.pub_s.publish(s)
        self._publish_target(self.px[idx], self.py[idx])
        if self.delay_auto:
            self._thr_hist.append((now, throttle))
            self._update_delay(now)

        # Path error at the nearest point, in the pose the controller used.
        e_lat = ((x - self.px[near]) * -math.sin(self.psi[near])
                 + (y - self.py[near]) * math.cos(self.psi[near]))
        e_head = (yaw - self.psi[near] + math.pi) % (2.0 * math.pi) - math.pi
        self._publish_status([                                   # order: STATUS_FIELDS
            self.speed, self.v_enc, self.v_pose, v_target, self.u_cmd, throttle, steering,
            ld, e_lat, e_head, self.kappa[near], self.s[near],
            # the slip the band asked for, relative to the predicted landing speed;
            # the slip the wheel then really saw is (v_enc - v)/max(v, 4) in the log
            (self.u_cmd - self.v_land) / max(self.v_land, self.v_slip_den),
            self.a_imu, self.cmd_delay])

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


def compose_latest(buf, map_frame, odom_frame, base_frame):
    """(x, y, yaw), stamp of map->base from the newest sample of each leg.

    A single-leg lookup at Time() returns that leg's newest sample untouched;
    a chain lookup would evaluate both legs at their latest COMMON time, which
    for a post-dated correction means an interpolation of old samples.
    """
    try:
        m = buf.lookup_transform(map_frame, odom_frame, rclpy.time.Time())
        o = buf.lookup_transform(odom_frame, base_frame, rclpy.time.Time())
    except Exception:                                   # noqa: BLE001
        tr = buf.lookup_transform(map_frame, base_frame, rclpy.time.Time())
        t = tr.transform.translation
        return (t.x, t.y, yaw_from_quat(tr.transform.rotation)), \
            tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
    my, oy = yaw_from_quat(m.transform.rotation), yaw_from_quat(o.transform.rotation)
    mt, ot = m.transform.translation, o.transform.translation
    c, s = math.cos(my), math.sin(my)
    return (mt.x + c * ot.x - s * ot.y, mt.y + s * ot.x + c * ot.y, my + oy), \
        o.header.stamp.sec + o.header.stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Zero the actuators on the way out -- but never let cleanup raise.
        # Under `ros2 launch`, SIGINT reaches rclpy's own handler first and the
        # context is already invalid by the time this runs, so the publish threw
        # RCLError, which skipped destroy_node() AND the shutdown, and exited 1.
        # (The stop command cannot be delivered in that case either way: the
        # bridge is being torn down in the same breath. Say so rather than
        # crashing.)
        try:
            if rclpy.ok():
                t, s = Float32(), Float32()
                t.data = s.data = 0.0
                node.pub_t.publish(t)
                node.pub_s.publish(s)
            else:
                node.get_logger().warn(
                    'context already shut down; stop command not sent')
        except Exception as exc:                     # noqa: BLE001 - cleanup
            node.get_logger().warn(f'stop command failed: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
