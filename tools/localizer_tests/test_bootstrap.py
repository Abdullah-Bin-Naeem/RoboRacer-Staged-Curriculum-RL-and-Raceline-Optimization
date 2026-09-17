"""Offline tests for localization_bootstrap (no simulator):

    python3 test_bootstrap.py maps/track_clean.pgm raceline/raceline_a7.0.csv

Covers scan-vs-map score calibration,
respawn seed search, and the node's confirm / refuse / respawn flow."""
import math, sys
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu, LaserScan
from roboracer_stack.localization import bootstrap as B

MAP_PGM, RACELINE = sys.argv[1], sys.argv[2]
RES, ORIGIN = 0.025, (0.00427, -16.8)


def read_pgm(p):
    b = open(p, 'rb').read()
    toks, i = [], 0
    while len(toks) < 4:
        while b[i:i + 1].isspace():
            i += 1
        if b[i:i + 1] == b'#':
            while b[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not b[j:j + 1].isspace():
            j += 1
        toks.append(b[i:j]); i = j
    w, h = int(toks[1]), int(toks[2])
    return np.frombuffer(b[i + 1:], np.uint8)[:w * h].reshape(h, w)


def to_grid(img):
    """nav2 map_io, trinary, negate 0, thresholds from track_clean.yaml."""
    occ = 1.0 - img / 255.0
    g = np.full(img.shape, -1, np.int8)
    g[occ > 0.65] = 100
    g[occ < 0.25] = 0
    return np.flipud(g)                  # image row 0 is the TOP; grid row 0 the bottom


GRID = to_grid(read_pgm(MAP_PGM))
H, W = GRID.shape
print(f'grid {W}x{H}: occupied {np.sum(GRID == 100)}, free {np.sum(GRID == 0)}, unknown {np.sum(GRID == -1)}')


def raycast(grid, origin, pose, lidar_x=0.2733, n=1080):
    x, y, yaw = pose
    lx, ly = x + lidar_x * math.cos(yaw), y + lidar_x * math.sin(yaw)
    ang = -2.35619 + np.arange(n) * 0.004363323
    rs = np.arange(0.06, 10.0, 0.01)
    out = np.full(n, np.inf)
    for k, a in enumerate(ang):
        ex = lx + rs * math.cos(yaw + a); ey = ly + rs * math.sin(yaw + a)
        c = np.floor((ex - origin[0]) / RES).astype(int); r = np.floor((ey - origin[1]) / RES).astype(int)
        ok = (c >= 0) & (c < grid.shape[1]) & (r >= 0) & (r < grid.shape[0])
        hit = np.zeros(len(rs), bool)
        hit[ok] = grid[r[ok], c[ok]] == 100
        if hit.any():
            out[k] = rs[np.argmax(hit)]
    return out


def scan_msg(ranges):
    m = LaserScan()
    m.angle_min, m.angle_max, m.angle_increment = -2.35619, 2.35619, 0.004363323
    m.range_min, m.range_max = 0.06, 10.0
    m.ranges = [float(r) for r in ranges]
    return m


def grid_msg(grid, origin):
    m = OccupancyGrid()
    m.info.resolution = RES
    m.info.width, m.info.height = grid.shape[1], grid.shape[0]
    m.info.origin.position.x, m.info.origin.position.y = origin
    m.data = grid.flatten().astype(int).tolist()
    return m


LINE = np.loadtxt(RACELINE, delimiter=',')
mask = B.near_wall_mask(GRID.flatten(), W, H, math.ceil(0.10 / RES))

# ---------------- 1. score calibration ----------------
spawn = (0.818, 3.158, -1.5707)
poses = [spawn] + [(LINE[i, 1], LINE[i, 2], LINE[i, 3]) for i in range(0, len(LINE), 25)]
perturb = [('true pose', 0, 0, 0), ('lateral 0.10 m', 0, 0.10, 0), ('lateral 0.20 m', 0, 0.20, 0),
           ('lateral 0.30 m', 0, 0.30, 0), ('along 0.30 m', 0.30, 0, 0), ('along 0.50 m', 0.50, 0, 0),
           ('yaw 3 deg', 0, 0, 3), ('yaw 5 deg', 0, 0, 5), ('yaw 10 deg', 0, 0, 10)]
table = {p[0]: [] for p in perturb}
for (x, y, yaw) in poses:
    ranges = raycast(GRID, ORIGIN, (x, y, yaw))
    sc = scan_msg(ranges)
    for name, da, dl, dy in perturb:
        for sgn in (1, -1):
            px = x + sgn * (da * math.cos(yaw) - dl * math.sin(yaw))
            py = y + sgn * (da * math.sin(yaw) + dl * math.cos(yaw))
            s = B.scan_match_score(mask, RES, ORIGIN, sc, (px, py, yaw + sgn * math.radians(dy)), 0.2733, 6.0)
            if s is not None:
                table[name].append(s)
            if name == 'true pose':
                break
print('\nscan-vs-map score over', len(poses), 'poses (spawn + raceline), min / median / max:')
for name, *_ in perturb:
    v = np.array(table[name])
    print(f'  {name:15s} {v.min():.2f} / {np.median(v):.2f} / {v.max():.2f}   below 0.5: {np.mean(v < 0.5) * 100:3.0f}%')
sp = scan_msg(raycast(GRID, ORIGIN, spawn))
print('  at the spawn only: true', round(B.scan_match_score(mask, RES, ORIGIN, sp, spawn, 0.2733, 6.0), 2),
      ' lateral 0.3:', round(B.scan_match_score(mask, RES, ORIGIN, sp, (spawn[0] + 0.3, spawn[1], spawn[2]), 0.2733, 6.0), 2),
      ' along 0.5:', round(B.scan_match_score(mask, RES, ORIGIN, sp, (spawn[0], spawn[1] - 0.5, spawn[2]), 0.2733, 6.0), 2),
      ' yaw 5:', round(B.scan_match_score(mask, RES, ORIGIN, sp, (spawn[0], spawn[1], spawn[2] + math.radians(5)), 0.2733, 6.0), 2))

# ---------------- 2. respawn seed ----------------
xs, ys, psis, step = LINE[:, 1], LINE[:, 2], LINE[:, 3], LINE[1, 0] - LINE[0, 0]
errs, misses = [], 0
for crash in range(0, len(LINE), 7):
    for back in (1.0, 2.0, 3.0):
        chk = int(crash - round(back / step)) % len(LINE)
        est = (xs[crash], ys[crash])
        seed = B.respawn_seed(xs, ys, psis, step, est, psis[chk], 6.0, math.radians(15))
        if seed is None:
            misses += 1; continue
        sx, sy, spsi, run = seed
        along_std = max(0.5, run / math.sqrt(12))
        d = math.hypot(sx - xs[chk], sy - ys[chk])
        errs.append(d / along_std)
errs = np.array(errs)
print(f'\nrespawn seed: {len(errs)} cases, {misses} with no match; distance to the true checkpoint '
      f'in units of the seeded along-track std: median {np.median(errs):.2f}, p95 {np.percentile(errs, 95):.2f}, '
      f'within 2 std {np.mean(errs <= 2) * 100:.0f}%')

# ---------------- 3. node flow ----------------
rclpy.init()


class Clock:
    t = 1000.0
    def now(self): return Time(nanoseconds=int(self.t * 1e9))


def make_node(extra=None):
    params = [Parameter('mode', value='spawn'), Parameter('path_csv', value=RACELINE),
              Parameter('max_seed_attempts', value=2)] + [Parameter(k, value=v) for k, v in (extra or {}).items()]
    orig = rclpy.node.Node.__init__
    B.Node.__init__ = lambda self, name, **kw: orig(self, name, parameter_overrides=params, **kw)
    n = B.LocalizationBootstrap()
    B.Node.__init__ = orig
    clk = Clock(); n.get_clock = lambda: clk
    n.sent = []
    n.pub_init.publish = lambda m: n.sent.append(m)
    n.pub_ready.publish = lambda m: None
    n.pub_t.publish = n.pub_s.publish = lambda m: None
    n.get_logger().set_level(50)
    n.localizer_up = True
    return n, clk


def imu(yaw, t, rate=0.0):
    m = Imu(); ns = int(t * 1e9)
    m.header.stamp.sec, m.header.stamp.nanosec = ns // 10 ** 9, ns % 10 ** 9
    m.orientation.z, m.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.angular_velocity.z = rate
    return m


def amcl(x, y, yaw):
    m = PoseWithCovarianceStamped()
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.pose.covariance[0] = m.pose.covariance[7] = 0.01; m.pose.covariance[35] = 0.001
    return m


def flow(map_origin, label):
    n, clk = make_node()
    n._cb_map(grid_msg(GRID, map_origin))
    n._cb_scan(scan_msg(raycast(GRID, ORIGIN, spawn)))     # the world is the TRUE map
    n._cb_imu(imu(spawn[2], clk.t))
    for _ in range(40):
        n._tick()
        clk.t += 0.25
        if n.seeded_at is not None and clk.t - n.seeded_at > 0.5:
            s = n.seed_pose
            n._cb_pose(amcl(*s))                          # AMCL adopts the seed exactly
        if n.finished:
            break
    print(f'  {label}: finished={n.finished} ready={n.ready} seeds sent={len(n.sent)}')
    return n, clk


print('\nnode flow (spawn mode, AMCL adopts the seed exactly):')
n, clk = flow(ORIGIN, 'map aligned        ')
flow((ORIGIN[0] + 0.30, ORIGIN[1]), 'map shifted 0.30 m ')

# respawn: car was at raceline index 150, respawns 2 m behind with that heading
back_n = int(round(2.0 / step))
i_crash = next(i for i in range(len(LINE)) if abs(B.wrap(psis[i] - psis[i - back_n])) > 0.8)
i_chk = (i_crash - back_n) % len(LINE)
detected_on_line = sum(abs(B.wrap(psis[i] - psis[i - back_n])) > 0.35 + 0.1 for i in range(len(LINE))) / len(LINE)
print(f'  fraction of the lap where a respawn 2 m back changes heading enough to detect: {detected_on_line * 100:.0f}%')
n.sent.clear()
t = clk.t
n._cb_pose(amcl(xs[i_crash], ys[i_crash], psis[i_crash]))
n._cb_imu(imu(psis[i_crash], t, 0.0))
n._cb_imu(imu(psis[i_crash] + 0.05, t + 0.055, 0.0))           # normal: no respawn
n._cb_imu(imu(psis[i_chk], t + 0.110, 0.0))                     # jump
if n.sent:
    m = n.sent[-1]; c = m.pose.covariance
    sx, sy = m.pose.pose.position.x, m.pose.pose.position.y
    print(f'  respawn: heading {math.degrees(psis[i_crash]):+.0f} -> {math.degrees(psis[i_chk]):+.0f} deg, '
          f'seeded {math.hypot(sx - xs[i_chk], sy - ys[i_chk]):.2f} m from the true checkpoint, '
          f'cov xx {c[0]:.2f} xy {c[1]:.2f} yy {c[7]:.2f}, sends={len(n.sent)}')
else:
    print(f'  respawn: NOT detected (heading {math.degrees(psis[i_crash]):+.0f} -> {math.degrees(psis[i_chk]):+.0f} deg)')
rclpy.shutdown()
