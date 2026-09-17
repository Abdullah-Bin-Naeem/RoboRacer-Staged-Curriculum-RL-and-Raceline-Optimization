#!/usr/bin/env python3
"""Measure what the simulated car can actually do: acceleration, drag, top speed,
braking, and how much curvature it really produces for a steering command.

Runs against the bridge alone (no pure_pursuit, no localization):

    terminal 1:  source ros_env.sh && ros2 launch autodrive_devkit bringup_headless.launch.py
    terminal 2:  source ros_env.sh && python3 tools/measure_car.py <test> [options]

Every test writes a CSV and prints its numbers at the end. Re-analyse a CSV
without driving:  python3 tools/measure_car.py analyze <file.csv>

Tests
-----
record  YOU drive (keyboard / manual mode in the simulator); the script only
        records and never publishes a command.  Ctrl-C to stop; it then prints
        every full-throttle run, every braking, and the cornering it saw
        (achieved curvature per steering command and speed, max lateral accel).
        Options: --out drive1.csv

accel   Full throttle from standstill down the straight, then brake to a stop.
        Gives a(v) = a0 - c*v (tyre/motor accel and linear drag), the top speed
        that fit predicts, 0-4 / 0-6 m/s times, wheel slip, and the braking
        deceleration.  Options: --throttle 1.0  --brake 0 (coast/lock) or -1
        (reverse torque)  --y-brake -2.5 (world y where braking starts).

swerve  Hold a target speed on the straight, then steer +d for `pulse` s,
        -d for 2*pulse, +d for pulse (a lane change out and back).  Gives the
        curvature the car really produces for command d at that speed
        (kappa = yaw_rate / v), compared with the kinematic value
        tan(d*0.5236)/0.324, and the lateral acceleration it reached.
        Options: --v 3.0  --steer 0.5  --pulse 0.3

Safety: the front LiDAR is watched; if anything is closer than --guard m
(default 0.45) the steering is zeroed and the brake applied.  Respawn the car
(restart the scene) between tests so every run starts from the spawn.

Topics used (all in /autodrive/roboracer_1): odom (ground truth, dev only),
imu, left_encoder, right_encoder, lidar, throttle_command, steering_command.
"""
import argparse, csv, math, sys, time
import numpy as np
trapz = getattr(np, 'trapezoid', None) or np.trapz

NS = '/autodrive/roboracer_1'
WHEEL_R = 0.0581       # m, from pure_pursuit params
WHEELBASE = 0.3240     # m
STEER_MAX = 0.5236     # rad at command 1.0


# ----------------------------------------------------------------------------- analysis
def load(path):
    rows = list(csv.DictReader(open(path)))
    d = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    return d


def true_speed(d):
    """Speed from ground-truth positions, one value per sim tick (repeated poses skipped)."""
    x, y, t = d['x'], d['y'], d['t']
    v = np.full(len(t), np.nan)
    last = 0
    for i in range(1, len(t)):
        if x[i] == x[last] and y[i] == y[last]:
            continue
        dt = t[i] - t[last]
        if dt > 0:
            v[i] = math.hypot(x[i] - x[last], y[i] - y[last]) / dt
        last = i
    ok = ~np.isnan(v)
    return np.interp(t, t[ok], v[ok]) if ok.sum() > 2 else v


def smooth(a, n):
    n = max(1, int(n) | 1)
    return np.convolve(np.pad(a, n // 2, mode='edge'), np.ones(n) / n, mode='valid')


def analyze_accel(d):
    t, thr = d['t'], d['throttle']
    v = smooth(true_speed(d), 5); a = np.gradient(v, t)
    drive = (thr > 0.05) & (v > 0.3)
    brake = (thr <= 0.0) & (np.arange(len(t)) > np.argmax(drive)) & (v > 0.3)
    print('\n=== accel test ===')
    if drive.sum() > 10:
        # a = a0 - c*v  (least squares on the full-throttle phase, first 0.3 s dropped: command lag)
        i0 = np.argmax(drive); m = drive & (t > t[i0] + 0.3) & (a > -2)
        # fit the speed curve itself, v(t) = vmax (1 - exp(-c (t - t0))), which is what a = a0 - c v integrates to;
        # fitting v instead of its noisy derivative is far more robust on a 16 Hz sim
        from scipy.optimize import curve_fit
        f = lambda tt, vmax, c, t0: vmax * (1 - np.exp(-c * (tt - t0)))
        (vmax_fit, c, t0), _ = curve_fit(f, t[m], v[m], p0=[10, 0.3, t[i0]], maxfev=20000)
        a0 = vmax_fit * c
        print(f'full-throttle phase: {t[m].max()-t[m].min():.2f} s, v {v[m].min():.2f} -> {v[m].max():.2f} m/s, '
              f'distance {trapz(v[m], t[m]):.1f} m')
        print(f'fit a = {a0:.2f} - {c:.3f} v   ->  a0 (accel at rest) {a0:.2f} m/s^2, drag {c:.3f} 1/s, '
              f'predicted top speed a0/c = {a0/c if c>0 else float("inf"):.1f} m/s')
        print(f'peak accel {a[m].max():.2f} m/s^2 (IMU ax peak {np.nanmax(d["imu_ax"][m]):.2f})')
        for vt in (2, 4, 6, 8):
            hit = np.where(drive & (v >= vt))[0]
            if len(hit): print(f'  0 -> {vt} m/s in {t[hit[0]]-t[i0]:.2f} s')
        ratio = d['v_enc'][m] / np.maximum(v[m], 0.1)
        print(f'wheel/true speed while accelerating: median {np.median(ratio):.3f}  (1.00 = no slip)')
        print(f'max true speed reached {v[drive].max():.2f} m/s')
    if brake.sum() > 5:
        vb = v[brake]; tb = t[brake]
        m = (vb > 0.5) & (vb < vb.max() - 0.3)
        if m.sum() > 3:
            dec = -np.polyfit(tb[m], vb[m], 1)[0]
            print(f'braking (throttle {d["throttle"][brake].min():.0f}): {vb.max():.2f} -> 0 m/s, mean decel {dec:.2f} m/s^2, '
                  f'IMU ax min {np.nanmin(d["imu_ax"][brake]):.2f}, stopping distance {trapz(vb, tb):.2f} m')
            r = d['v_enc'][brake] / np.maximum(vb, 0.1)
            print(f'  wheel/true speed while braking: median {np.median(r):.3f}  (0 = wheels locked)')


def analyze_swerve(d):
    t, st = d['t'], d['steer']; v = smooth(true_speed(d), 5); gz = smooth(d['imu_gz'], 3); ay = smooth(d['imu_ay'], 3)
    print('\n=== swerve test ===')
    print(f'speed during pulses: {v[np.abs(st)>0.05].mean():.2f} m/s')
    # each constant-steering segment
    edges = np.where(np.diff(np.sign(st)) != 0)[0] + 1
    segs = np.split(np.arange(len(t)), edges)
    print(f'{"cmd":>6} {"v":>5} {"yaw_rate":>9} {"kappa":>7} {"r":>6} {"kin kappa":>10} {"ratio":>6} {"a_y":>6} {"IMU a_y":>8}')
    amax = 0
    for s in segs:
        c = st[s].mean()
        if abs(c) < 0.05 or len(s) < 4: continue
        s = s[len(s)//3:]                                  # let the steering settle (slew 3.2 rad/s)
        w = np.abs(gz[s]).max(); vv = v[s].mean(); k = w / max(vv, 0.1); kk = math.tan(abs(c) * STEER_MAX) / WHEELBASE
        amax = max(amax, vv * w)
        print(f'{c:+6.2f} {vv:5.2f} {w:9.2f} {k:7.2f} {1/k if k>0 else 0:6.2f} {kk:10.2f} {k/kk:6.2f} {vv*w:6.2f} {np.abs(ay[s]).max():8.2f}')
    print(f'largest lateral acceleration in this run: {amax:.2f} m/s^2  (ratio < 1 = understeer: the car turns less than the steering asks)')


def analyze_record(d):
    """Manual drive. Speed from ground-truth position (central difference over sim ticks), acceleration
    from the IMU (smooth, the position derivative is too noisy at the sim's odom rate)."""
    x, y, t, thr, st, ax, gz = d['x'], d['y'], d['t'], d['throttle'], d['steer'], d['imu_ax'], d['imu_gz']
    keep = [0]
    for i in range(1, len(t)):
        if x[i] != x[keep[-1]] or y[i] != y[keep[-1]]: keep.append(i)
    k = np.array(keep); tt = t[k]
    v = np.zeros(len(k))
    for i in range(1, len(k) - 1):
        v[i] = math.hypot(x[k[i+1]] - x[k[i-1]], y[k[i+1]] - y[k[i-1]]) / (tt[i+1] - tt[i-1])
    th, a, w, sc, ve = thr[k], ax[k], gz[k], st[k], d['v_enc'][k]
    print('\n=== manual drive ===')
    print(f'{t[-1]:.0f} s, {len(k)} sim ticks ({1/np.mean(np.diff(tt)):.1f} Hz odom), top true speed {v.max():.2f} m/s')
    def fit(name, m, sign):
        m = m & (v > 0.3) & (np.abs(sc) < 0.15)
        if m.sum() < 5: print(f'{name}: not enough data'); return
        X = np.c_[np.ones(m.sum()), v[m]]; (a0, c), *_ = np.linalg.lstsq(X, sign * a[m], rcond=None)
        print(f'{name}: n={m.sum()}  a = {a0:.2f} {c:+.3f}*v  m/s^2   (v {v[m].min():.1f}..{v[m].max():.1f})')
        for v0 in range(0, 12, 2):
            mm = m & (v >= v0) & (v < v0 + 2)
            if mm.sum(): print(f'    v {v0:2d}-{v0+2:2d}: {sign*np.median(a[mm]):5.2f} m/s^2  wheel/true {np.median(ve[mm]/np.maximum(v[mm],0.1)):5.2f}  (n={mm.sum()})')
    fit('FULL THROTTLE (thr > 0.95)', (th > 0.95) & (a > 0.5), +1)
    fit('BRAKE, throttle 0 (wheels lock)', (th <= 0.0) & (th > -0.05) & (a < -2), -1)
    fit('BRAKE, reverse throttle (thr < -0.5)', (th < -0.5) & (a < -2), -1)
    print('  (a0 = force at rest, the v term is the sim\'s linear drag: it fights acceleration and helps braking)')
    print('\ncornering (achieved kappa = yaw_rate / v; ratio 1.0 = no understeer):')
    print(f'{"steer":>10} {"v":>8} {"n":>4} {"kappa":>7} {"r":>6} {"kin":>6} {"ratio":>6} {"a_y":>6}')
    amax = 0
    for s0, s1 in [(0.2, 0.4), (0.4, 0.6), (0.6, 0.8), (0.8, 0.95), (0.95, 1.01)]:
        for v0, v1 in [(0.8, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.5), (3.5, 5.0), (5.0, 9.0)]:
            m = (np.abs(sc) >= s0) & (np.abs(sc) < s1) & (v >= v0) & (v < v1)
            if m.sum() < 4: continue
            kap = np.median(np.abs(w[m]) / v[m]); kk = math.tan(np.median(np.abs(sc[m])) * STEER_MAX) / WHEELBASE
            ay = np.median(v[m] * np.abs(w[m])); amax = max(amax, np.percentile(v[m] * np.abs(w[m]), 90))
            print(f'{s0:4.2f}-{s1:4.2f} {v0:3.1f}-{v1:3.1f} {m.sum():4d} {kap:7.2f} {1/kap if kap>0 else 0:6.2f} {kk:6.2f} {kap/kk:6.2f} {ay:6.2f}')
    if amax: print(f'lateral acceleration, p90 of the best bin: {amax:.2f} m/s^2')
    else: print('  (no cornering recorded -- steering stayed at 0)')


# ----------------------------------------------------------------------------- driving
def drive(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
    from std_msgs.msg import Float32
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState, LaserScan

    QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                     durability=QoSDurabilityPolicy.VOLATILE)
    BE = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)

    class M(Node):
        def __init__(self):
            super().__init__('measure_car')
            self.pose = None; self.imu = (0., 0., 0.); self.enc = {}; self.v_side = {'l': 0., 'r': 0.}; self.v_enc = 0.; self.front = 9.
            self.create_subscription(Odometry, f'{NS}/odom', self.cb_odom, QOS)
            self.create_subscription(Imu, f'{NS}/imu', self.cb_imu, QOS)
            self.create_subscription(JointState, f'{NS}/left_encoder', lambda m: self.cb_enc('l', m), QOS)
            self.create_subscription(JointState, f'{NS}/right_encoder', lambda m: self.cb_enc('r', m), QOS)
            self.create_subscription(LaserScan, f'{NS}/lidar', self.cb_scan, BE)
            self.pt = self.create_publisher(Float32, f'{NS}/throttle_command', QOS)
            self.ps = self.create_publisher(Float32, f'{NS}/steering_command', QOS)
            if args.test == 'record':   # listen to what the sim reports (feedback) and to any command on the bus
                for tp in ('throttle', 'throttle_command'):
                    self.create_subscription(Float32, f'{NS}/{tp}', lambda m: setattr(self, 'thr', float(m.data)), QOS)
                for tp in ('steering', 'steering_command'):
                    self.create_subscription(Float32, f'{NS}/{tp}', lambda m: setattr(self, 'steer', float(m.data)), QOS)
            self.rows = []; self.thr = 0.; self.steer = 0.; self.t0 = None; self.aborted = False; self.yaw_prev = None

        def cb_odom(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            if self.yaw_prev is not None and abs((yaw - self.yaw_prev + math.pi) % (2 * math.pi) - math.pi) > math.radians(40):
                self.get_logger().error('heading jumped: the car was respawned (wall hit). Aborting.'); self.aborted = True
            self.yaw_prev = yaw
            self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

        def cb_imu(self, m):
            self.imu = (m.linear_acceleration.x, m.linear_acceleration.y, m.angular_velocity.z)

        def cb_enc(self, side, m):
            if not m.position: return
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; ang = float(m.position[0])
            prev = self.enc.get(side); self.enc[side] = (ang, t)
            if prev and t > prev[1]:
                self.v_side[side] = WHEEL_R * (ang - prev[0]) / (t - prev[1])
                self.v_enc = 0.5 * (self.v_side['l'] + self.v_side['r'])

        def cb_scan(self, m):
            r = np.array(m.ranges); n = len(r); ang = m.angle_min + np.arange(n) * m.angle_increment
            f = r[(np.abs(ang) < math.radians(40)) & np.isfinite(r) & (r > 0.05)]
            self.front = float(f.min()) if len(f) else 9.

        def cmd(self, thr, steer):
            self.thr, self.steer = float(thr), float(steer)
            self.pt.publish(Float32(data=self.thr)); self.ps.publish(Float32(data=self.steer))

        def log(self):
            if self.pose is None: return
            t = time.time()
            if self.t0 is None: self.t0 = t
            self.rows.append(dict(t=t - self.t0, x=self.pose[0], y=self.pose[1], yaw=self.pose[2], v_enc=self.v_enc,
                                  imu_ax=self.imu[0], imu_ay=self.imu[1], imu_gz=self.imu[2],
                                  throttle=self.thr, steer=self.steer, front=self.front))

        def guard(self):
            if self.front < args.guard:
                self.get_logger().warn(f'wall {self.front:.2f} m ahead: braking'); return True
            return self.aborted

    rclpy.init(); n = M()
    print('waiting for odom + lidar ...')
    while rclpy.ok() and (n.pose is None or n.front == 9.):
        rclpy.spin_once(n, timeout_sec=0.1)
    print(f'car at x={n.pose[0]:.2f} y={n.pose[1]:.2f} yaw={math.degrees(n.pose[2]):.0f} deg, wall ahead {n.front:.2f} m')
    dt = 0.02

    def run(steps):
        """steps: list of (duration_s or condition-fn, throttle-fn, steer). Stops early on guard."""
        for dur, thr_fn, steer in steps:
            t_start = time.time()
            while rclpy.ok():
                rclpy.spin_once(n, timeout_sec=0.0)
                if n.guard(): n.cmd(args.brake, 0.0); n.log(); return False
                if callable(dur):
                    if dur(): break
                elif time.time() - t_start > dur: break
                n.cmd(thr_fn() if callable(thr_fn) else thr_fn, steer); n.log(); time.sleep(dt)
        return True

    def vnow():  # true speed from the last two logged poses (fallback: encoder)
        r = n.rows
        for i in range(len(r) - 1, 0, -1):
            if r[i]['x'] != r[i - 1]['x'] or r[i]['y'] != r[i - 1]['y']:
                d = math.hypot(r[i]['x'] - r[i - 1]['x'], r[i]['y'] - r[i - 1]['y']); dtt = r[i]['t'] - r[i - 1]['t']
                return d / dtt if dtt > 0 else n.v_enc
        return n.v_enc

    if args.test == 'record':
        print('recording -- drive the car yourself. Ctrl-C to stop and analyse.')
        try:
            while rclpy.ok():
                rclpy.spin_once(n, timeout_sec=0.0); n.log(); time.sleep(dt)
        except KeyboardInterrupt:
            pass
        n.aborted = False
    elif args.test == 'accel':
        y_stop = args.y_brake
        heading_down = math.cos(n.pose[2] + math.pi / 2) > 0     # spawn heading is -y
        print(f'full throttle {args.throttle:.2f} until y {"<" if heading_down else ">"} {y_stop}, then brake {args.brake:+.0f}')
        ok = run([(lambda: (n.pose[1] < y_stop) if heading_down else (n.pose[1] > y_stop), args.throttle, 0.0)])
        run([(lambda: vnow() < 0.15 and n.v_enc < 0.15, args.brake, 0.0), (0.5, 0.0, 0.0)])
    elif args.test == 'swerve':
        kp = 0.35; ff = lambda: min(1.0, max(0.0, kp * (args.v - n.v_enc) + 0.035 * args.v))
        print(f'reaching {args.v:.1f} m/s, then steer +{args.steer} / -{args.steer} / +{args.steer} for {args.pulse} s / {2*args.pulse} s / {args.pulse} s')
        run([(lambda: n.v_enc > args.v - 0.15, ff, 0.0), (0.4, ff, 0.0),
             (args.pulse, ff, +args.steer), (2 * args.pulse, ff, -args.steer), (args.pulse, ff, +args.steer), (0.4, ff, 0.0)])
        run([(lambda: n.v_enc < 0.15, args.brake, 0.0), (0.3, 0.0, 0.0)])
    if args.test != 'record': n.cmd(0.0, 0.0)
    out = args.out or f'{args.test}_{time.strftime("%H%M%S")}.csv'
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(n.rows[0])); w.writeheader(); w.writerows(n.rows)
    print(f'wrote {out} ({len(n.rows)} rows)')
    rclpy.shutdown()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('test', choices=['record', 'accel', 'swerve', 'analyze'])
    ap.add_argument('file', nargs='?', help='CSV for analyze')
    ap.add_argument('--throttle', type=float, default=1.0)
    ap.add_argument('--brake', type=float, default=0.0, help='throttle used to stop: 0 (release/lock) or -1 (reverse torque)')
    ap.add_argument('--y-brake', type=float, default=-2.5, help='world y at which the accel test starts braking')
    ap.add_argument('--v', type=float, default=3.0); ap.add_argument('--steer', type=float, default=0.5)
    ap.add_argument('--pulse', type=float, default=0.3); ap.add_argument('--guard', type=float, default=0.45)
    ap.add_argument('--out')
    a = ap.parse_args()
    if a.test == 'analyze':
        d = load(a.file); analyze_record(d); return
    out = drive(a); d = load(out)
    {'accel': analyze_accel, 'swerve': analyze_swerve, 'record': analyze_record}[a.test](d)


if __name__ == '__main__':
    main()
