#!/usr/bin/env python3

"""Record a race run from outside the container: lap times, collisions, cmd_delay.

WHY NOT bench.sh
----------------
`scripts/bench.sh record` is the right tool when you are benchmarking the
LOCALIZER: it needs a host mount, RACER_MODE=dev and scipy, so it runs its own
container. But `./scripts/run.sh racer` is the submission, run exactly as the
organizers run it -- no mounts, no extra environment, `--rm`. Nothing may be
added to it, and a file written inside it dies with the container.

This needs none of that. `run.sh racer` uses `--network=host --ipc=host`, and
the Dockerfile puts RMW_IMPLEMENTATION and CYCLONEDDS_URI in ENV rather than in
a shell startup file precisely so `docker exec` shells inherit them and can see
the nodes. So: pipe this file into a `docker exec` on stdin and let the CSV come
back on stdout, where the HOST shell redirects it to a host file. The image, the
container and the run are untouched, and nothing is copied in -- `python3 -`
reads the program from stdin.

    docker exec -i autodrive_roboracer_api bash -lc '
        source /opt/ros/humble/setup.bash
        source /home/autodrive_devkit/install/setup.bash
        exec python3 - --seconds 180
    ' < roboracer_stack/tools/record_run.py > runs/run42.csv

LAP TIMES
---------
The devkit bridge publishes the race telemetry the organizers time with, and
bridge.launch.py remaps only /tf, so these topics are on the graph in EVERY run
including a race one:

    <ns>/lap_count        <ns>/last_lap_time      <ns>/collision_count

They are RESTRICTED (common/restricted.py) and this tool reads them, so it is a
DEVELOPMENT instrument and prints a banner saying so. The restriction is about
the control path -- a node steering on ground truth -- and this is a separate
observer process that is no part of the submission. Do not run it during a
timed submission run; do use it for every tuning run, because it is the same
number the organizers will read.

`last_lap_time` is a Float32, so the topic is exact even though the simulator's
on-screen readout is rounded. Two ordering details, both measured on run43:

* the bridge republishes last_lap_time EVERY tick rather than on change, so a
  lap must not be banked on the value alone; and
* lap_count increments up to ~25 ms BEFORE last_lap_time is updated. Banking on
  the count increment therefore records the PREVIOUS lap's time -- run43 had
  2 of 11 laps wrong that way. So the count only ARMS a lap; it is banked when
  the time then changes, or after `LAP_TIME_GRACE` if it does not (two
  consecutive laps can genuinely share a time to the millisecond).

The first lap is a standing start and is reported as WARMUP, excluded from the
statistics.

`collision_count` is why this matters more than the lap time alone: a lap that
clipped a wall is not a lap you can compare. Laps are reported clean or dirty.

As a cross-check (and as the fallback when --no-lap-topics is given, which is
the only configuration legal in a timed run) laps are ALSO derived from `s`, the
arc length of the nearest point on the racing line, which the follower publishes
itself and which is not restricted. `s` wrapping from lap_len back to 0 is a lap
boundary; the crossing is interpolated between the two samples either side, so
the resolution is not the 20 Hz tick. Those times measure a circuit of the
racing line from s = 0, NOT the simulator's start/finish line, so they are a
repeatable relative measure, not the official one.

CMD_DELAY
---------
The field that gates every other number here: the follower's measured
throttle-to-wheel round trip. It sets the speed target's lead point, places the
slip band, scales the lookahead, and drives the speed derate in
config/pure_pursuit.yaml -- at 0.21 s every speed target is cut by 7.4 %. A lap
time recorded without it cannot be compared with another lap time.

OFFLINE
-------
Re-read a CSV on the host, where there is no ROS:

    python3 roboracer_stack/tools/record_run.py --analyze runs/run42.csv
"""

import argparse
import math
import signal
import sys
import time

# Imported lazily so --analyze works on the host, which has no ROS.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from std_msgs.msg import Float32, Float32MultiArray, Int32
except ImportError:                                   # --analyze only
    rclpy = None
    Node = object

NS = '/autodrive/roboracer_1'

# Mirrors STATUS_FIELDS in control/pure_pursuit.py. If that tuple grows, the
# extra values land in 'extra*' rather than being silently misnamed.
FIELDS = ('v_est', 'v_enc', 'v_pose', 'v_target', 'u_cmd', 'throttle', 'steering',
          'ld', 'e_lat', 'e_head', 'kappa', 's', 'slip', 'a_imu', 'delay')

# The derate, from config/pure_pursuit.yaml. Only used for the summary.
DERATE_FROM, DERATE_TO, DERATE_A_LAT = 0.175, 0.21, 6.0
PROFILE_A_LAT = 7.0

# How long to wait for last_lap_time to catch up with lap_count before banking
# the lap with the value already published. Measured lead: ~25 ms.
LAP_TIME_GRACE = 0.40

# Bound on a plausible change in s between two samples: the car cannot exceed
# this speed, plus a little slack for the path's own point spacing. Used to tell
# real motion from the nearest-point projection jumping after a crash.
MAX_SPEED_MPS = 12.0
STEP_SLACK_M = 0.10


def derate_factor(delay, profile_a_lat=PROFILE_A_LAT,
                  d_from=DERATE_FROM, d_to=DERATE_TO, a_derate=DERATE_A_LAT):
    """The factor pure_pursuit multiplies every speed target by at this delay.

    Same arithmetic as _control(): a_eff slides linearly from the profile's own
    lateral limit at d_from to a_derate at d_to, and corner speed goes as
    sqrt(a_lat), so targets scale by sqrt(a_eff / profile_a_lat). Disarmed when
    the profile was planned at or below a_derate.
    """
    if not (d_to > d_from and profile_a_lat > a_derate):
        return 1.0
    f = max(0.0, min(1.0, (delay - d_from) / (d_to - d_from)))
    a_eff = profile_a_lat + (a_derate - profile_a_lat) * f
    return math.sqrt(a_eff / profile_a_lat)


def stats(xs):
    """(n, best, mean, median, worst, sd) for a list of lap times."""
    q = sorted(xs)
    n = len(q)
    mean = sum(q) / n
    sd = math.sqrt(sum((x - mean) ** 2 for x in q) / n)
    return n, q[0], mean, q[n // 2], q[-1], sd


class Run:
    """Everything derived from a run, live or replayed from a CSV.

    Kept free of rclpy so the offline path runs the SAME code as the live one
    and the two can never drift apart.
    """

    def __init__(self, lap_len_override=0.0, quiet=False):
        self.rows = 0
        self.delays = []
        self.official = []          # (lap number, time, collisions during it)
        self.derived = []           # lap times from the s wrap
        self.derived_dropped = 0    # s-derived laps discarded as unphysical
        self._progress = 0.0        # NET distance covered since the last crossing
        self.lap_len_override = lap_len_override
        self.quiet = quiet
        self._lap_seen = None
        self._pending = None        # (lap number, time when the count moved, armed at t)
        self._banked_time = None
        self._coll = 0
        self._coll_at_lap = 0
        self._prev = None
        self._s_seen = 0.0
        self._steps = []
        self._s_max = 0.0
        self._last_cross = None

    # ---- official telemetry ------------------------------------------------

    def note_lap(self, t, lap_count, last_lap_time):
        """Arm on the lap_count increment, bank when last_lap_time catches up.

        lap_count leads last_lap_time by up to a sample, so banking on the count
        alone records the previous lap. Arm instead, and bank on the first time
        value that differs from the one already banked -- or, if none arrives
        within LAP_TIME_GRACE, on whatever is current, so two laps that really
        do share a time are not lost.
        """
        if lap_count > 0 and lap_count != self._lap_seen:
            self._lap_seen = lap_count
            self._pending = (lap_count, t)
        if self._pending is None or last_lap_time <= 0.0:
            return
        lap, armed = self._pending
        fresh = self._banked_time is None or abs(last_lap_time - self._banked_time) > 1e-9
        if not (fresh or t - armed >= LAP_TIME_GRACE):
            return
        self._pending = None
        self._banked_time = last_lap_time
        dirty = self._coll - self._coll_at_lap
        self._coll_at_lap = self._coll
        self.official.append((lap, last_lap_time, dirty))
        if not self.quiet:
            tag = 'CLEAN' if dirty == 0 else f'{dirty} COLLISION(S)'
            d = f'   delay {self.delays[-1]:.3f}' if self.delays else ''
            warm = '   WARMUP (standing start, excluded)' if lap == 1 else ''
            print(f'  lap {lap:2d}: {last_lap_time:.3f} s   {tag}{d}{warm}',
                  file=sys.stderr)

    def note_collision(self, count):
        self._coll = count

    # ---- derived from s ----------------------------------------------------

    def _median_step(self):
        if not self._steps:
            return 0.0
        q = sorted(self._steps)
        return q[len(q) // 2]

    def lap_len(self):
        return self.lap_len_override or (self._s_seen + 0.5 * self._median_step())

    def note_s(self, t, s_now):
        """Detect the s wrap and interpolate the crossing time.

        lap_len is learned, so this works on any racing line without being told
        which one is loaded. The largest s ever seen is the last sample before a
        wrap, which falls short of the true lap length by however far the car
        travelled in that final part-step -- uniform in [0, ds], averaging ds/2.
        Adding half the median step removes that bias; --lap-len overrides it
        with the exact value (s of the CSV's last row plus one row spacing).

        A wrap counts only once the car has been most of the way round. Without
        that guard the s of the NEAREST path point jitters across the seam while
        the car crawls, and every jitter would be a lap.

        After a crash the nearest-point projection stops being a position at
        all: run61's s walked 11.6 -> 24.7 -> 3.9 within a second while the car
        sat against a wall, which reads as several laps and produced a 4.10 s
        "best lap". So any lap in which s jumps BACKWARDS by more than a metre
        (other than at the wrap itself) is discarded rather than reported.
        """
        self._s_seen = max(self._s_seen, s_now)
        self._s_max = max(self._s_max, s_now)
        prev = self._prev
        self._prev = (t, s_now)
        if prev is None or self._s_seen <= 0.0:
            return
        t_p, s_p = prev
        if 0.0 < s_now - s_p < 0.5 * self._s_seen:
            self._steps.append(s_now - s_p)

        # Net, not absolute: a car oscillating against a wall accumulates
        # plenty of forward motion but no net progress round the lap. And only
        # PHYSICALLY POSSIBLE steps count -- once the car is against a wall the
        # nearest-point projection teleports (run61 stepped 11.6 -> 24.7 m in
        # one 25 ms sample), and a single such jump is otherwise worth half a
        # lap of fake progress.
        if abs(s_now - s_p) <= MAX_SPEED_MPS * (t - t_p) + STEP_SLACK_M:
            self._progress += s_now - s_p
        lap_len = self.lap_len()
        if not (s_p - s_now > 0.5 * lap_len and self._s_max > 0.8 * lap_len):
            return
        travelled = (lap_len - s_p) + s_now
        frac = ((lap_len - s_p) / travelled) if travelled > 1e-9 else 0.0
        t_cross = t_p + frac * (t - t_p)
        self._s_max = s_now
        if self._last_cross is not None:
            # A real lap means the car went round. After a crash the
            # nearest-point projection walks backwards through s = 0 in small
            # steps and forward again: that passes any per-step jump test and
            # still manufactured a 4.10 s "best lap" in run61. Require the
            # FORWARD distance covered to be most of a lap.
            if self._progress < 0.75 * lap_len:
                self.derived_dropped += 1
            else:
                self.derived.append(t_cross - self._last_cross)
        self._progress = 0.0
        self._last_cross = t_cross

    # ---- report ------------------------------------------------------------

    def summary(self, elapsed=None):
        out = sys.stderr
        print('', file=out)
        if elapsed:
            print(f'{self.rows} rows in {elapsed:.1f} s '
                  f'({self.rows / elapsed:.1f} Hz)', file=out)

        racing = [(l, t, d) for l, t, d in self.official if l > 1]
        if racing:
            clean = [t for _, t, d in racing if d == 0]
            dirty = [t for _, t, d in racing if d != 0]
            n, best, mean, med, worst, sd = stats([t for _, t, _ in racing])
            warm = [t for l, t, _ in self.official if l == 1]
            print(f'OFFICIAL laps (last_lap_time)  {n} racing, '
                  f'{len(clean)} clean, {len(dirty)} with contact'
                  + (f'   [lap 1 warmup {warm[0]:.3f} s excluded]' if warm else ''),
                  file=out)
            print(f'  all      best {best:.3f}  mean {mean:.3f}  median {med:.3f}'
                  f'  worst {worst:.3f}  sd {sd:.3f}', file=out)
            if clean:
                n, best, mean, med, worst, sd = stats(clean)
                print(f'  CLEAN    best {best:.3f}  mean {mean:.3f}  median {med:.3f}'
                      f'  worst {worst:.3f}  sd {sd:.3f}   <- compare on these',
                      file=out)
            if dirty:
                print(f'  contact  {len(dirty)} lap(s) hit something; their times '
                      f'mean nothing', file=out)
        else:
            print('OFFICIAL laps: none seen (--no-lap-topics, only a warmup lap, '
                  'or no lap completed yet)', file=out)

        if self.derived:
            n, best, mean, med, worst, sd = stats(self.derived)
            label = 'DERIVED from s' if self.official else 'LAPS derived from s'
            drop = (f', {self.derived_dropped} discarded as unphysical'
                    if self.derived_dropped else '')
            print(f'{label}  {n}{drop}: best {best:.3f}  mean {mean:.3f}  '
                  f'median {med:.3f}  sd {sd:.3f}   '
                  f'(lap {self.lap_len():.2f} m, s=0 crossing, not the '
                  f'start/finish line)', file=out)

        if not self.delays:
            print('cmd_delay: NO DATA. Is the follower up, and has the car moved? '
                  'It only updates once the throttle varies (std > 0.3 over a 6 s '
                  'window).', file=out)
            return
        d = sorted(self.delays)
        n = len(d)
        mean = sum(d) / n
        print(f'cmd_delay  min {d[0]:.3f}  p50 {d[n // 2]:.3f}  mean {mean:.3f}  '
              f'p90 {d[min(n - 1, int(0.9 * n))]:.3f}  max {d[-1]:.3f}  s', file=out)
        f = derate_factor(mean)
        print(f'derate at the mean: targets x{f:.4f}', file=out)
        if f >= 0.9995:
            print('  -> NO derate. Lap times are comparable across runs.', file=out)
        else:
            base = 6.40
            print(f'  -> a {base:.2f} s lap becomes {base / f:.2f} s: '
                  f'{base / f - base:+.2f} s of pure derate. Fix the loop rate '
                  f'before judging any raceline.', file=out)


class Recorder(Node):
    def __init__(self, topic, every, run, lap_topics):
        super().__init__('record_run')
        self.run = run
        self.every = every
        self.t0 = time.time()
        self._next = self.t0 + every if every > 0 else None
        self.width = None
        self.lap_count = 0
        self.last_lap = 0.0
        self.collisions = 0
        # Same profile the follower uses for these publishers, so the
        # subscriptions match rather than silently failing to connect.
        qos = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Float32MultiArray, topic, self._cb, qos)
        print(f'# status: {topic}', file=sys.stderr)
        if lap_topics:
            print(f'# RESTRICTED race telemetry: {NS}/lap_count, /last_lap_time, '
                  f'/collision_count\n'
                  f'# development only -- never during a timed submission run.',
                  file=sys.stderr)
            self.create_subscription(Int32, f'{NS}/lap_count',
                                     self._cb_lap, qos)
            self.create_subscription(Float32, f'{NS}/last_lap_time',
                                     self._cb_time, qos)
            self.create_subscription(Int32, f'{NS}/collision_count',
                                     self._cb_coll, qos)

    def _cb_lap(self, msg):
        self.lap_count = int(msg.data)
        self.run.note_lap(time.time() - self.t0, self.lap_count, self.last_lap)

    def _cb_time(self, msg):
        self.last_lap = float(msg.data)
        if not math.isfinite(self.last_lap):
            return                                  # the bridge publishes inf before lap 1
        # The count leads the time; re-offer so the armed lap can be banked.
        self.run.note_lap(time.time() - self.t0, self.lap_count, self.last_lap)

    def _cb_coll(self, msg):
        self.collisions = int(msg.data)
        self.run.note_collision(self.collisions)

    def _cb(self, msg):
        d = list(msg.data)
        if self.width is None:
            self.width = len(d)
            names = list(FIELDS[:self.width])
            names += [f'extra{i}' for i in range(len(names), self.width)]
            print('t_s,' + ','.join(names) + ',lap_count,last_lap_time,collisions',
                  flush=True)
            if self.width != len(FIELDS):
                print(f'# WARNING: {self.width} fields published, {len(FIELDS)} '
                      f'named here; check STATUS_FIELDS', file=sys.stderr)
        now = time.time()
        t = now - self.t0
        print(f'{t:.4f},' + ','.join(f'{x:.5f}' for x in d) +
              f',{self.lap_count},{self.last_lap:.5f},{self.collisions}', flush=True)
        self.run.rows += 1
        if self.width > 14:
            self.run.delays.append(d[14])
        if self.width > 11:
            self.run.note_s(t, d[11])
        if self.run._pending is not None:
            self.run.note_lap(t, self.lap_count, self.last_lap)   # grace timeout
        if self._next is not None and now >= self._next:
            self._next = now + self.every
            if self.run.delays:
                tail = self.run.delays[-200:]
                m = sum(tail) / len(tail)
                print(f'  [{t:6.1f}s] {self.run.rows:6d} rows   '
                      f'delay {self.run.delays[-1]:.3f} (mean {m:.3f}) '
                      f'-> x{derate_factor(m):.3f}   laps {self.lap_count}   '
                      f'collisions {self.collisions}', file=sys.stderr)


def analyze(path, lap_len=0.0):
    """Replay a CSV this tool wrote through the same Run object."""
    import csv as _csv

    with open(path) as fh:
        rows = [r for r in _csv.DictReader(fh) if r.get('t_s')]
    if not rows:
        raise SystemExit(f'{path}: no data rows')
    for need in ('s', 'delay'):
        if need not in rows[0]:
            raise SystemExit(f'{path}: no {need!r} column; is this a '
                             f'record_run.py CSV?')
    run = Run(lap_len_override=lap_len)
    run.rows = len(rows)
    has_laps = 'lap_count' in rows[0] and 'last_lap_time' in rows[0]
    for r in rows:
        run.delays.append(float(r['delay']))
        run.note_s(float(r['t_s']), float(r['s']))
        if 'collisions' in r and r['collisions']:
            run.note_collision(int(float(r['collisions'])))
        if has_laps and r['lap_count'] and r['last_lap_time']:
            lt = float(r['last_lap_time'])
            if lt == lt and lt not in (float('inf'), float('-inf')):   # bridge sends inf
                run.note_lap(float(r['t_s']), int(float(r['lap_count'])), lt)
    run.summary(elapsed=float(rows[-1]['t_s']))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--analyze', metavar='CSV',
                    help='re-read a CSV this tool wrote and print the summary; '
                         'no ROS needed, runs on the host')
    ap.add_argument('--topic', default='/pure_pursuit/status')
    ap.add_argument('--no-lap-topics', action='store_true',
                    help='do not touch the restricted race telemetry; lap times '
                         'then come only from the s wrap. The only configuration '
                         'legal in a timed run')
    ap.add_argument('--seconds', type=float, default=0.0,
                    help='stop after this long; 0 = until SIGINT/SIGTERM. Use a '
                         'duration under `docker exec -i`, where Ctrl-C does not '
                         'reach the container')
    ap.add_argument('--lap-len', type=float, default=0.0,
                    help='exact lap length [m] for the derived timing (s of the '
                         'raceline CSV last row plus one row spacing)')
    ap.add_argument('--every', type=float, default=10.0,
                    help='seconds between progress lines on stderr; 0 = silent')
    args = ap.parse_args()

    if args.analyze:
        return analyze(args.analyze, args.lap_len)
    if rclpy is None:
        raise SystemExit('rclpy not importable: run this inside the container, '
                         'or use --analyze on a saved CSV.')

    rclpy.init()
    run = Run(lap_len_override=args.lap_len)
    node = Recorder(args.topic, args.every, run, not args.no_lap_topics)
    stop = {'now': False}
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.update(now=True))
    deadline = time.time() + args.seconds if args.seconds > 0 else None
    try:
        while rclpy.ok() and not stop['now']:
            rclpy.spin_once(node, timeout_sec=0.2)
            if deadline and time.time() >= deadline:
                break
    finally:
        run.summary(elapsed=time.time() - node.t0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
