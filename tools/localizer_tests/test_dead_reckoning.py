"""Offline harness (no simulator):

    python3 test_dead_reckoning.py RACELINE.csv OLD_dead_reckoning.py NEW_dead_reckoning.py
    (JIT=0.15 sets receipt-stamp jitter as a fraction of a frame)

Drive dead_reckoning (old from git HEAD, new from the tree)
with a simulated bridge along the raceline and score the TF AMCL would read.

Bridge model: per sim frame, left encoder, right encoder, IMU, scan, each
stamped at receipt (frame time + latency + jitter). 200 Hz timer at a random
phase. Metric: odom->base interpolated at each SCAN stamp (what AMCL looks up)
against the truth of that frame, relative to the start.
"""
import importlib.util, math, os, sys
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from roboracer_stack.common.tire_model import TireSpeedObserver

LINE = np.loadtxt(sys.argv[1], delimiter=',')
OLD_PATH = sys.argv[2]
S, X, Y, V = LINE[:, 0], LINE[:, 1], LINE[:, 2], LINE[:, 7]
STEP = S[1] - S[0]
LAP = S[-1] + STEP
XC = np.append(X, X[0]); YC = np.append(Y, Y[0]); SC = np.append(S, LAP)
R = 0.0581


def at(s):
    s = s % LAP
    return np.interp(s, SC, XC), np.interp(s, SC, YC)


def yaw_at(s):
    a, b = at(s - 0.02), at(s + 0.02)
    return math.atan2(b[1] - a[1], b[0] - a[0])


def vprof(s):
    return float(np.interp(s % LAP, SC, np.append(V, V[0])))


def make_run(hz, jitter, laps, slip_mode, seed=1, rise=2.5):
    """Return events and truth. slip_mode: 'none' | 'band' (tire-model car)."""
    rng = np.random.default_rng(seed)
    dt_phys = 0.001
    true_s = 0.0; v = 0.0; wheel = 0.0
    car = TireSpeedObserver(rise_slope=rise)
    frames = []
    t = 0.0; next_frame = 0.0; parked = 1.0
    while true_s < laps * LAP:
        if t >= parked:
            target = vprof(true_s + 0.6)
            if slip_mode == 'none':
                v = target if v == 0.0 else v + np.clip(target - v, -5 * dt_phys, 5 * dt_phys)
                u = v
            else:
                lo, hi = v - 0.08 * max(v, 4.0), v + 0.11 * max(v, 4.0)
                u = float(np.clip(target, lo, hi))
                car.v = v
                v = car.step(u, dt_phys)
            true_s += v * dt_phys
            wheel += u * dt_phys
        if t >= next_frame - 1e-9:
            x, y = at(true_s)
            frames.append((t, true_s, x, y, yaw_at(true_s), wheel / R))
            next_frame += 1.0 / hz
        t += dt_phys
    events = []
    rate = 0.0
    for k, (tf, s, x, y, yaw, ang) in enumerate(frames):
        rx = tf + 0.004 + rng.uniform(-jitter, jitter) / hz
        if k:
            dyaw = (yaw - frames[k - 1][4] + math.pi) % (2 * math.pi) - math.pi
            rate = dyaw / (tf - frames[k - 1][0])
        events.append((rx, 'l', ang))
        events.append((rx + 0.0003, 'r', ang))
        events.append((rx + 0.0006, 'imu', (yaw, rate)))
        events.append((rx + 0.0020, 'scan', k))
    phase = rng.uniform(0, 0.005)
    tick = phase
    end = events[-1][0] + 0.05
    while tick < end:
        events.append((tick, 'tick', None))
        tick += 0.005
    events.sort(key=lambda e: e[0])
    return events, frames


def stamp(msg_header, t):
    ns = int(t * 1e9)
    msg_header.stamp.sec, msg_header.stamp.nanosec = ns // 10 ** 9, ns % 10 ** 9


class FakeClock:
    def __init__(self): self.t = 0.0
    def now(self): return Time(nanoseconds=int(self.t * 1e9))


def run(path, events, frames, params=None, dead_after=None):
    from rclpy.parameter import Parameter
    spec = importlib.util.spec_from_file_location('dr_' + str(abs(hash(path))), path)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    ovr = [Parameter(k, value=v) for k, v in (params or {}).items()]
    orig = rclpy.node.Node.__init__
    def init(self, name, **kw):
        orig(self, name, parameter_overrides=ovr, **kw)
    mod.Node.__init__ = init
    node = mod.DeadReckoning()
    mod.Node.__init__ = orig
    clock = FakeClock()
    node.get_clock = lambda: clock
    tfs = []
    node.tfb.sendTransform = lambda tf: tfs.append(
        (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9,
         tf.transform.translation.x, tf.transform.translation.y))
    node.pub.publish = lambda m: None
    node.get_logger().set_level(40)
    scans = []
    for t, kind, p in events:
        clock.t = t
        if kind in ('l', 'r'):
            if dead_after is not None and kind == 'r' and t > dead_after:
                continue
            m = JointState(); m.position = [p]; stamp(m.header, t)
            node._cb_enc(kind, m)
        elif kind == 'imu':
            m = Imu(); stamp(m.header, t)
            m.orientation.z, m.orientation.w = math.sin(p[0] / 2), math.cos(p[0] / 2)
            m.angular_velocity.z = p[1]
            node._cb_imu(m)
        elif kind == 'scan':
            scans.append((t, p))
        else:
            node._tick()
    node.destroy_node()
    # TF at each scan stamp, linear interpolation as tf2 does
    ts = np.array([a[0] for a in tfs]); xs = np.array([a[1] for a in tfs]); ys = np.array([a[2] for a in tfs])
    order = np.argsort(ts, kind='stable'); ts, xs, ys = ts[order], xs[order], ys[order]
    x0, y0 = frames[0][2], frames[0][3]
    errs = []
    for t, k in scans:
        if t < ts[0] or t > ts[-1] or k < 20:
            continue
        ex = np.interp(t, ts, xs) - (frames[k][2] - x0)
        ey = np.interp(t, ts, ys) - (frames[k][3] - y0)
        errs.append(math.hypot(ex, ey))
    errs = np.array(errs)
    return errs


rclpy.init()
NEW = sys.argv[3]
print(f'raceline lap {LAP:.1f} m')
for hz, jit in ((18, 0.15), (40, 0.15)):
    ev, fr = make_run(hz, jit, 1, 'none')
    for label, path in (('old', OLD_PATH), ('new', NEW)):
        e = run(path, ev, fr)
        print(f'no slip  {hz} Hz  {label}: error at scan stamps  mean {e.mean():.3f}  p95 {np.percentile(e, 95):.3f}  max {e.max():.3f}  end {e[-1]:.3f} m')

ev, fr = make_run(18, 0.15, 1, 'none')
for label, path in (('old', OLD_PATH), ('new', NEW)):
    e = run(path, ev, fr, dead_after=8.0)
    print(f'right encoder dies at 8 s, {label}: end error {e[-1]:.3f} m  (car travelled {fr[-1][1]:.1f} m)')

for rise in (3.0, 2.5):
  ev, fr = make_run(18, float(os.environ.get("JIT", "0.15")), 1, "band", seed=3, rise=rise)
  print('--- truth tire rise_slope', rise, '(observer uses 3.0)')
  travel = fr[-1][1]; wheel = fr[-1][5] * R
  print(f'slip band run: car {travel:.2f} m, wheels {wheel:.2f} m ({(wheel / travel - 1) * 100:+.1f}%)')
  for label, path, prm in (('old encoder', OLD_PATH, None), ('new encoder', NEW, None),
                         ('new tire', NEW, {'distance_source': 'tire'})):
    e = run(path, ev, fr, prm)
    print(f'  slip band 18 Hz {label}: mean {e.mean():.3f}  p95 {np.percentile(e, 95):.3f}  max {e.max():.3f}  end {e[-1]:.3f} m')
rclpy.shutdown()
