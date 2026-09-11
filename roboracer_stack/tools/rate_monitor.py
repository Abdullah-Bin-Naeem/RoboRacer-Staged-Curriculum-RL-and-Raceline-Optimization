#!/usr/bin/env python3

"""Live sensor-tick and control-loop rate, while the car is driving.

    python3 rate_monitor.py [--csv PATH] [--period 1.0] [--window 3.0]
                            [--warn 35] [--lidar]

The organizers tune and verify their machines at 40-50 Hz. This branch was
developed at 17.5 Hz, and the difference is not subtle: pure_pursuit measures
its own round trip and derates the speed targets when it grows, so a slow
machine quietly drives a slower, blunter car -- and when the rate collapses
mid-lap, into a wall. Nothing in the running stack reported that rate, which is
why it took a race admin to point at it. This does.

WHY /imu IS THE TICK
--------------------
The devkit bridge publishes all fourteen topics from one @sio.on('Bridge')
handler, one burst per WebSocket frame, and only then emits the command back
(autodrive_bridge.py). So /imu, /left_encoder and /lidar all arrive at exactly
the simulator's frame rate, and a 40-byte Imu measures it as well as a
1080-float LaserScan does -- without the subscription cost landing on the very
machine whose spare CPU is the thing in question. --lidar adds the scan anyway
when you want that confirmed rather than assumed.

WHY THE MEAN IS THE WRONG NUMBER
--------------------------------
A lap averaging 45 Hz with one 200 ms stall in it hits a wall; a flat 40 Hz does
not. `ros2 topic hz` reports the average and hides exactly the event that costs
the run. So each topic carries three numbers, and they catch different failures:

    hz      mean over the window. Sustained capacity.
    p5_hz   the 5th-percentile rate, 1 / p95(gap). How slow the loop gets when
            it is PERSISTENTLY degraded. At 40 Hz over a 3 s window there are
            ~120 gaps, so a single stall sits above the 99th percentile and does
            NOT move this -- it is a jitter measure, not a stall detector.
    gap     the largest single gap in the window, and `stalls`, a count of gaps
            over twice the median. THESE are what catch one 200 ms freeze, and
            one is enough to put the car in a wall.

On a healthy run hz and p5_hz sit close together and stalls stays at 0.

READING IT
----------
    imu high, cmd low     our algorithm is the limiter -- lower control_hz, or
                          the loop is starved
    imu low, cmd tracks   the bridge or the simulator is the limiter, i.e. the
                          machine running this container, or the LAN
    delay ~3 sim frames   normal: 175 ms at 17.5 Hz, ~65 ms at 45 Hz

It publishes nothing and, by default, reads no restricted topic (no /ips, /odom,
/tf, no lap or collision counters -- see common/restricted.py), so it is
race-legal and safe to leave running through a timed lap.

--laps IS THE EXCEPTION
-----------------------
The lap topics ARE restricted: they are race telemetry. --laps subscribes to
them and prints a line as each lap closes, together with how the tick behaved
DURING that lap -- which is the question the rest of this tool exists to answer:

    LAP  7   6.382 s   best 6.351   tick 44.9 Hz   worst gap  31 ms   stalls 0
    LAP  8   7.104 s   best 6.351   tick 41.2 Hz   worst gap 287 ms   stalls 3

Two numbers on one line, and the 0.7 s is explained. It is off by default and
announces itself when on, exactly like pure_pursuit's dev_lap_telemetry: use it
while testing, leave it off for anything you intend to quote as a result.

RUNNING IT AGAINST THE SUBMISSION IMAGE
---------------------------------------
It imports nothing from roboracer_stack, so it needs no overlay and no rebuild:

    docker cp rate_monitor.py autodrive_roboracer_api:/tmp/
    docker exec -it autodrive_roboracer_api bash -lc \
      'source /opt/ros/humble/setup.bash && python3 /tmp/rate_monitor.py --csv /tmp/rate.csv'

The DDS settings are image ENV (Dockerfile), so an exec shell already discovers
the running nodes. On Linux, scripts/hz.sh does both lines for you.
"""

import argparse
import csv
import math
import signal
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32, Float32MultiArray, Int32

NS = '/autodrive/roboracer_1'
# The profile every node in this stack and the bridge itself use. A mismatch
# here would show as a topic that simply never arrives.
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)

# pure_pursuit.STATUS_FIELDS, by index. Duplicated rather than imported so this
# file stays runnable from /tmp with no overlay sourced. KEEP IN STEP with
# control/pure_pursuit.py -- _cb_status checks the length and says so if a field
# is ever inserted, because the failure mode otherwise is a plausible-looking
# delay read out of the wrong slot.
PP_V_EST, PP_DELAY = 0, 14
PP_N_FIELDS = 15

# How long a closed lap waits for its last_lap_time before it is printed without
# one. The bridge sends both from a single callback, so in practice this is
# never reached -- it exists so that a renamed or missing topic degrades to one
# '--' rather than to laps that silently never appear.
_LAP_FLUSH_S = 2.0


def _pct(sorted_vals, q):
    """Linear-interpolated percentile of an already-sorted list."""
    if not sorted_vals:
        return float('nan')
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    i = q * (len(sorted_vals) - 1)
    lo = int(math.floor(i))
    hi = min(lo + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (i - lo)


def _hz(r, hz):
    """One topic's rate, in the three states that mean different things.

    '--'    never published at all: a wrong topic name, a QoS mismatch, or a
            node that never came up. Nothing to do with how fast the machine is.
    'DEAD'  published earlier, then nothing for a whole window. The sensor
            STOPPED -- the simulator disconnected, the LAN dropped, or the
            machine wedged. This is the state the tool exists to catch, and it
            must not read as a number.
    a rate  everything is arriving; the question is only how fast.
    """
    if r.n == 0:
        return '   --  '
    if hz != hz:                      # NaN: nothing left in the window
        return '  DEAD '
    return f'{hz:5.1f}Hz'


def _val(v, fmt='5.1f'):
    """A statistic that is undefined until the window holds two samples.

    'nan' in a column of rates reads like a very small number at a glance, which
    is the opposite of what it means.
    """
    return '   --'.rjust(len(format(0.0, fmt))) if v != v else format(v, fmt)


class Rate:
    """Arrival times of one topic, and the rate statistics over a window.

    Arrivals are stamped with time.monotonic(), NOT header.stamp. The bridge
    stamps every message with its own clock at socket receipt, so the header
    already carries the WebSocket and Unity jitter this is trying to measure --
    reading it back would report the rate the sim intended rather than the rate
    that arrived. Monotonic also survives an NTP step mid-run.
    """

    def __init__(self, window):
        self.window = window
        self.t = deque()
        self.n = 0              # total ever seen, for the "is it alive" check
        self.stalls = 0         # gaps over twice the median, cumulative
        self._prev = None
        # Median gap, set by stats(). Stall detection therefore starts at the
        # first report line rather than at the first message.
        self._med = None

    def add(self, now=None):
        now = time.monotonic() if now is None else now
        self.n += 1
        if self._prev is not None and self._med is not None:
            if now - self._prev > 2.0 * self._med:
                self.stalls += 1
        self._prev = now
        self.t.append(now)
        while self.t and now - self.t[0] > self.window:
            self.t.popleft()

    def stats(self, now):
        """(hz, p5_hz, max_gap_s). NaN until there are two samples in the window."""
        while self.t and now - self.t[0] > self.window:
            self.t.popleft()
        if len(self.t) < 2:
            return float('nan'), float('nan'), float('nan')
        gaps = sorted(b - a for a, b in zip(self.t, list(self.t)[1:]))
        self._med = _pct(gaps, 0.5)
        span = self.t[-1] - self.t[0]
        hz = (len(self.t) - 1) / span if span > 0 else float('nan')
        p95 = _pct(gaps, 0.95)
        return hz, (1.0 / p95 if p95 > 0 else float('nan')), gaps[-1]


class RateMonitor(Node):

    def __init__(self, args):
        super().__init__('rate_monitor')
        self.warn = args.warn
        self.period = args.period
        self.t0 = time.monotonic()

        w = args.window
        self.imu = Rate(w)
        self.enc = Rate(w)
        self.lidar = Rate(w) if args.lidar else None
        self.cmd = Rate(w)
        self.pp = Rate(w)
        self.delay = float('nan')       # pure_pursuit's live round-trip estimate
        self.v_est = float('nan')
        self._prev_stalls = {}          # per-topic, to report the delta
        self._warned_status = False

        # Lap telemetry. self.laps accumulates one closed lap per entry so the
        # closing summary can show the whole session without re-reading the CSV.
        self.lap_count = None
        self.last_lap = float('nan')
        self.best_lap = float('nan')
        self.laps = []
        self._pending_lap = None        # lap closed, waiting for its time
        self._last_lap_fresh = False    # has last_lap_time landed since it closed?
        self._lap_mark = None           # tick counters at the last lap boundary
        # The rolling window is 3 s and a lap is ~6.4 s, so a stall early in a
        # lap has already fallen out of the window by the time the lap closes.
        # The per-lap worst gap therefore has to be accumulated as it happens.
        self._lap_gap = 0.0

        self.create_subscription(Imu, f'{NS}/imu',
                                 lambda _m: self.imu.add(), QOS)
        self.create_subscription(JointState, f'{NS}/left_encoder',
                                 lambda _m: self.enc.add(), QOS)
        if self.lidar is not None:
            self.create_subscription(LaserScan, f'{NS}/lidar',
                                     lambda _m: self.lidar.add(), QOS)
        # Our own command, so this measures the loop that actually reaches the
        # car rather than the timer that was asked for.
        self.create_subscription(Float32, f'{NS}/steering_command',
                                 lambda _m: self.cmd.add(), QOS)
        self.create_subscription(Float32MultiArray, '/pure_pursuit/status',
                                 self._cb_status, QOS)
        if args.laps:
            # RESTRICTED -- race telemetry. Same treatment as pure_pursuit's
            # dev_lap_telemetry: allowed, but it says so every time.
            print('[rate_monitor] --laps ON: subscribing to RESTRICTED lap '
                  'topics (race telemetry). Development only -- a lap time '
                  'measured with this on is a development number.', flush=True)
            self.create_subscription(Int32, f'{NS}/lap_count',
                                     self._cb_lap_count, QOS)
            self.create_subscription(Float32, f'{NS}/last_lap_time',
                                     self._cb_last_lap, QOS)
            self.create_subscription(Float32, f'{NS}/best_lap_time',
                                     self._cb_best_lap, QOS)

        self.csv_f = self.csv_w = None
        if args.csv:
            self.csv_f = open(args.csv, 'w', newline='')
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow((
                't_s', 'imu_hz', 'imu_p5_hz', 'imu_max_gap_ms', 'imu_stalls',
                'enc_hz', 'lidar_hz', 'lidar_p5_hz', 'cmd_hz',
                'pp_hz', 'pp_p5_hz', 'pp_stalls', 'delay_s', 'v_est',
                'lap_count', 'last_lap_s', 'best_lap_s'))
            self.csv_f.flush()

        self.create_timer(self.period, self._report)
        print(f'[rate_monitor] window {w:.1f} s, warn under {self.warn:.0f} Hz'
              + (', lidar ON' if self.lidar is not None else '')
              + (', laps ON' if args.laps else '')
              + (f', csv {args.csv}' if args.csv else ''), flush=True)
        print('[rate_monitor] waiting for the simulator to connect', flush=True)

    def _cb_status(self, msg):
        self.pp.add()
        d = msg.data
        if len(d) != PP_N_FIELDS and not self._warned_status:
            self._warned_status = True
            print(f'[rate_monitor] WARNING /pure_pursuit/status has {len(d)} '
                  f'fields, expected {PP_N_FIELDS} -- STATUS_FIELDS has moved '
                  f'and the delay column may be reading the wrong one',
                  flush=True)
        if len(d) > PP_DELAY:
            self.delay = float(d[PP_DELAY])
            self.v_est = float(d[PP_V_EST])

    def _cb_lap_count(self, msg):
        """A lap closed. Park it -- its TIME has not arrived yet.

        The bridge publishes the whole burst from one callback in a fixed order
        (lap_count, lap_time, last_lap_time, best_lap_time), so at this instant
        self.last_lap still holds the PREVIOUS lap's time and self.best_lap is
        one lap stale. Printing here would label every lap with its
        predecessor's numbers.

        Rather than depend on that order -- it is the devkit's, not ours -- the
        lap is parked until a last_lap_time arrives AFTER the park, which is the
        evidence that the time belongs to this lap and not the one before it.
        _LAP_FLUSH_S then bounds the wait, so a missing or renamed topic costs a
        line that says '--' rather than a lap that never prints at all.
        """
        if self.lap_count is not None and msg.data != self.lap_count:
            self._pending_lap = (msg.data, self._close_lap(), time.monotonic())
            self._last_lap_fresh = False
        self.lap_count = msg.data
        if self._lap_mark is None:
            self._mark_lap()

    def _cb_last_lap(self, msg):
        self.last_lap = float(msg.data)
        self._last_lap_fresh = True

    def _cb_best_lap(self, msg):
        self.best_lap = float(msg.data)

    def _flush_pending_lap(self):
        """Print a parked lap once its time is in, or give up waiting."""
        if self._pending_lap is None:
            return
        lap, tick, parked_at = self._pending_lap
        if not self._last_lap_fresh:
            if time.monotonic() - parked_at < _LAP_FLUSH_S:
                return                  # the rest of the burst is still coming
            self.last_lap = float('nan')
        self._pending_lap = None
        self._print_lap(lap, self.last_lap, tick)

    def _mark_lap(self):
        """Snapshot the tick counters, so the next lap can be scored on its own."""
        self._lap_mark = (time.monotonic(), self.imu.n, self.imu.stalls)

    def _close_lap(self):
        """(mean tick Hz, worst gap, stalls) since the last boundary."""
        if self._lap_mark is None:
            self._mark_lap()
            return float('nan'), float('nan'), 0
        t0, n0, s0 = self._lap_mark
        dt = time.monotonic() - t0
        hz = (self.imu.n - n0) / dt if dt > 0 else float('nan')
        out = (hz, self._lap_gap, self.imu.stalls - s0)
        self._lap_gap = 0.0
        self._mark_lap()
        return out

    def _print_lap(self, lap, secs, tick):
        hz, gap, stalls = tick
        self.laps.append((lap, secs, hz, gap, stalls))
        mark = '  <-- best' if secs == self.best_lap else ''
        print(f'LAP {lap:3d}  {_val(secs, "7.3f")} s   '
              f'best {_val(self.best_lap, "7.3f")}   '
              f'tick {_val(hz)} Hz   worst gap {_val(gap * 1e3, "4.0f")} ms   '
              f'stalls {stalls}{mark}', flush=True)

    def _delta(self, name, r):
        """'+2' when stalls happened in THIS period. A cumulative count that
        moved four minutes ago reads identically to one moving right now, and
        the whole point is to catch the moment someone touches the machine."""
        d = r.stalls - self._prev_stalls.get(name, 0)
        self._prev_stalls[name] = r.stalls
        return f'(+{d})' if d else ''

    def _report(self):
        now = time.monotonic()
        t = now - self.t0
        imu_hz, imu_p5, imu_gap = self.imu.stats(now)
        enc_hz, _, _ = self.enc.stats(now)
        cmd_hz, _, _ = self.cmd.stats(now)
        pp_hz, pp_p5, _ = self.pp.stats(now)
        if self.lidar is not None:
            lid_hz, lid_p5, _ = self.lidar.stats(now)
        else:
            lid_hz = lid_p5 = float('nan')

        if imu_gap == imu_gap:
            self._lap_gap = max(self._lap_gap, imu_gap)
        self._flush_pending_lap()

        if self.imu.n == 0:
            print(f't={t:6.1f} | no sensor data yet -- is the simulator connected?',
                  flush=True)
        else:
            # '!!' rather than colour: this is read over docker exec, often
            # through PowerShell, where ANSI is not a given.
            bad = lambda hz: '!!' if hz == hz and hz < self.warn else '  '
            parts = [f't={t:6.1f}',
                     f'imu {_hz(self.imu, imu_hz)}{bad(imu_hz)} '
                     f'p5 {_val(imu_p5)} gap {_val(imu_gap * 1e3, "4.0f")}ms',
                     f'enc {_hz(self.enc, enc_hz)}']
            if self.lidar is not None:
                parts.append(f'lidar {_hz(self.lidar, lid_hz)}{bad(lid_hz)} '
                             f'p5 {_val(lid_p5)}')
            # cmd carries the marker too: sensors fast but commands slow is the
            # signature of our own loop being the limiter, and that is the whole
            # reason both numbers are on one line.
            parts.append(f'cmd {_hz(self.cmd, cmd_hz)}{bad(cmd_hz)}')
            if self.pp.n:
                parts.append(f'pp {_hz(self.pp, pp_hz)}{bad(pp_hz)} '
                             f'p5 {_val(pp_p5)} '
                             f'delay {_val(self.delay * 1e3, "4.0f")}ms')
            else:
                parts.append('pp    --   (no /pure_pursuit/status)')
            parts.append('stalls ' + ' '.join(
                f'{n}={r.stalls}{self._delta(n, r)}'
                for n, r in (('imu', self.imu), ('pp', self.pp))))
            print(' | '.join(parts), flush=True)

        if self.csv_w is not None:
            self.csv_w.writerow((
                f'{t:.3f}', f'{imu_hz:.3f}', f'{imu_p5:.3f}',
                f'{imu_gap * 1e3:.1f}', self.imu.stalls,
                f'{enc_hz:.3f}', f'{lid_hz:.3f}', f'{lid_p5:.3f}',
                f'{cmd_hz:.3f}', f'{pp_hz:.3f}', f'{pp_p5:.3f}', self.pp.stalls,
                f'{self.delay:.4f}', f'{self.v_est:.3f}',
                '' if self.lap_count is None else self.lap_count,
                f'{self.last_lap:.4f}', f'{self.best_lap:.4f}'))
            self.csv_f.flush()   # a run that ends in Ctrl-C still has its data

    def summarise(self):
        """One block at exit -- the number to quote, without re-reading the CSV."""
        now = time.monotonic()
        print('', flush=True)
        print(f'[rate_monitor] {now - self.t0:.1f} s, '
              f'{self.imu.n} imu / {self.pp.n} control samples', flush=True)
        for name, r in (('imu (sim tick)', self.imu), ('lidar', self.lidar),
                        ('steering_command', self.cmd),
                        ('pure_pursuit loop', self.pp)):
            if r is None or r.n == 0:
                continue
            hz, p5, gap = r.stats(now)
            # A run that ended with the sim already disconnected has an empty
            # window, so these are NaN. Say so in words rather than printing
            # 'nan Hz' as the headline number of the whole session.
            if hz != hz:
                print(f'  {name:20s}    silent for the last {r.window:.0f} s   '
                      f'stalls {r.stalls}', flush=True)
                continue
            print(f'  {name:20s} {hz:6.1f} Hz   worst {p5:6.1f} Hz   '
                  f'max gap {gap * 1e3:5.0f} ms   stalls {r.stalls}', flush=True)
        if self.delay == self.delay:
            print(f'  round-trip delay     {self.delay * 1e3:6.0f} ms  '
                  f'(pure_pursuit derates the speed targets above 175 ms)',
                  flush=True)

        if self.laps:
            # The table the session is actually judged on -- and beside each lap
            # time, the tick that produced it. A slow lap with stalls in it is a
            # different problem from a slow lap at a clean 45 Hz.
            print('', flush=True)
            print('  lap      time     tick    worst gap  stalls', flush=True)
            for lap, secs, hz, gap, stalls in self.laps:
                mark = ' <-- best' if secs == self.best_lap else ''
                print(f'  {lap:3d}  {secs:8.3f} s  {_val(hz)} Hz  '
                      f'{_val(gap * 1e3, "6.0f")} ms  {stalls:5d}{mark}',
                      flush=True)
            clean = [t for _l, t, _h, _g, st in self.laps if st == 0]
            if clean and len(clean) != len(self.laps):
                print(f'  best over the {len(clean)} lap(s) with no stall: '
                      f'{min(clean):.3f} s -- the rest were measured on a '
                      f'machine that was interrupted', flush=True)
        if self.csv_f is not None:
            self.csv_f.close()


def _on_sigterm(_sig, _frame):
    """`docker stop`, a killed exec and `timeout` all send SIGTERM, which would
    otherwise take the process out before the closing summary is printed. The CSV
    is flushed every period either way, so this only buys the summary -- but that
    is the block worth reading, and it should not need a Ctrl-C to appear."""
    raise KeyboardInterrupt


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--csv', default='', help='append per-period rows here')
    ap.add_argument('--period', type=float, default=1.0,
                    help='seconds between report lines (default 1.0)')
    ap.add_argument('--window', type=float, default=3.0,
                    help='rolling window the rates are computed over (default 3.0)')
    ap.add_argument('--warn', type=float, default=35.0,
                    help='mark any rate under this many Hz (default 35)')
    ap.add_argument('--lidar', action='store_true',
                    help='also subscribe to the 1080-beam scan; costs real CPU '
                         'on the machine under test, so it is off by default')
    ap.add_argument('--laps', action='store_true',
                    help='report lap times, and how the tick behaved during '
                         'each lap. RESTRICTED topics: development only')
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    signal.signal(signal.SIGTERM, _on_sigterm)

    rclpy.init()
    node = RateMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summarise()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
