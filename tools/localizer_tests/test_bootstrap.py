"""Offline tests for localization_bootstrap (no simulator):

    python3 test_bootstrap.py maps/track_clean.pgm raceline/raceline_a7.0.csv

Covers scan-vs-map score calibration, the recovery prior against every logged
IROS 2026 reset (the node's own pick_back / load_centreline), and the node's
confirm / refuse / reset-recovery flow."""
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

# ---------------- 2. recovery prior ----------------
# Every contact seen in a logged multi-track run, and the checkpoint the
# simulator actually reset the car to (runs 14, 19, 20). Same cases as
# check_recovery_prior.py, run through the node's own helpers.
from roboracer_stack.common import frames as F
CASES = [((1.11, -14.66), (0.718, -15.575)), ((2.38, -15.64), (1.713, -15.770)),
         ((1.05, -14.52), (0.716, -15.574)), ((3.70, -12.26), (3.018, -13.624)),
         ((5.52, -11.49), (5.047, -11.480)), ((0.52, 3.89), (0.800, 3.653))]
cl = B.load_centreline(F.CENTRELINE_CSV, (F.SPAWN_X, F.SPAWN_Y, F.SPAWN_YAW))
assert cl is not None, F.CENTRELINE_CSV
cs, cx, cy, rev = cl
lap = float(np.max(cs))
s_of = lambda px, py: float(cs[int(np.argmin((cx - px) ** 2 + (cy - py) ** 2))])
cps = [tuple(float(v) for v in c) for c in F.CHECKPOINTS]
bad = 0
for contact, expect in CASES:
    sl = s_of(*contact)
    backs = [(B.pick_back(sl, s_of(x, y), lap), x, y) for x, y, _ in cps]
    back, px, py = min((b for b in backs if b[0] is not None and b[0] <= 25.0), key=lambda b: b[0])
    ok = math.hypot(px - expect[0], py - expect[1]) < 0.05
    bad += not ok
    print(f'  contact {contact}: picked ({px:.3f}, {py:.3f}) {back:.2f} m back -- {"ok" if ok else "WRONG"}')
print(f'\nrecovery prior: lap {lap:.2f} m (centreline reversed={rev}), {len(CASES) - bad}/{len(CASES)} resets pick the right checkpoint')

# ---------------- 3. node flow ----------------
rclpy.init()


class Clock:
    t = 1000.0
    def now(self): return Time(nanoseconds=int(self.t * 1e9))


def make_node(extra=None):
    params = [Parameter('mode', value='spawn'),
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

# reset: car at the hairpin-1 apex contact of run 14, reset to the exit checkpoint
contact, chk = (2.38, -15.64, 0.30), (1.713, -15.770, 0.268)
print('\nreset recovery (spawn mode node from above, checkpoint adopted exactly):')
t = clk.t
n.sent.clear()
n._cb_pose(amcl(*contact))                                    # the trusted pose before the hit
n._cb_imu(imu(contact[2], t, 0.0)); n._cb_imu(imu(contact[2] + 0.02, t + 0.022, 0.0))
n._cb_imu(imu(1.2, t + 0.044, 0.0))                           # heading step: the reset
n._cb_scan(scan_msg(raycast(GRID, ORIGIN, (chk[0], chk[1], 1.2))))
flagged = n.rec_state
ready_dropped = False
n.pub_ready.publish = lambda m, n=n: globals().__setitem__('ready_dropped', ready_dropped or not m.data)
for _ in range(60):
    n._tick(); clk.t += 0.1
    if n.rec_state == 'seed' and n.seeded_at is not None and clk.t - n.seeded_at > 0.3:
        n._cb_pose(amcl(*n.seed_pose))
    if n.rec_state == 'watch' and n.recoveries:
        break
if n.sent:
    m = n.sent[0]
    sx, sy = m.pose.pose.position.x, m.pose.pose.position.y
    print(f'  flagged={flagged} ready dropped={ready_dropped} seeded at ({sx:.3f}, {sy:.3f}), '
          f'{math.hypot(sx - chk[0], sy - chk[1]):.3f} m from the true checkpoint; '
          f'state now {n.rec_state}, recoveries {n.recoveries}')
else:
    print(f'  reset NOT recovered: state {n.rec_state}')
rclpy.shutdown()
