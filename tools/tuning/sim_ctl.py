#!/usr/bin/env python3
"""Simulator housekeeping for unattended experiments. DEVELOPMENT ONLY: every
subcommand touches restricted topics (reset_command, lap and collision telemetry).

    sim_ctl.py reset                     pulse /autodrive/reset_command True -> False, confirm the counters read 0
    sim_ctl.py stop                      hold throttle 0 / steering 0 for a second (the sim latches the last command)
    sim_ctl.py watch LAPS MAX_CONTACTS OUT.csv [--stall S] [--timeout S]
                                         write one row per lap crossing; exit 0 at LAPS timed laps,
                                         3 once MAX_CONTACTS contacts are counted, 4 on a stall, 5 on timeout

Lap numbering: the simulator's lap_count reads 0 after a reset; the first crossing
(lap_count 1) closes the out-lap, so LAPS timed laps end at lap_count LAPS + 1.
reset_command is level-triggered (the bridge re-sends it every tick), so it must
be pulsed back to False or the simulator resets forever.
"""
import csv
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=1)


class Tel:
    def __init__(self, node):
        self.lap = None
        self.col = None
        self.last = float('nan')
        self.t_msg = None
        node.create_subscription(Int32, f'{NS}/lap_count', self._lap, QOS)
        node.create_subscription(Int32, f'{NS}/collision_count', self._col, QOS)
        node.create_subscription(Float32, f'{NS}/last_lap_time', self._last, QOS)

    def _lap(self, m):
        self.lap = m.data
        self.t_msg = time.monotonic()

    def _col(self, m):
        self.col = m.data

    def _last(self, m):
        self.last = m.data


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def reset(node):
    tel = Tel(node)
    pub = node.create_publisher(Bool, '/autodrive/reset_command', QOS)
    stop(node, 0.5)
    for val, dur in ((True, 0.6), (False, 1.5)):
        end = time.monotonic() + dur
        while time.monotonic() < end:
            pub.publish(Bool(data=val))
            spin_for(node, 0.05)
    spin_for(node, 1.0)
    print(f'reset: lap_count={tel.lap} collision_count={tel.col}', flush=True)
    return 0 if tel.lap == 0 and tel.col == 0 else 1


def stop(node, seconds=1.0):
    pt = node.create_publisher(Float32, f'{NS}/throttle_command', QOS)
    ps = node.create_publisher(Float32, f'{NS}/steering_command', QOS)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        pt.publish(Float32(data=0.0))
        ps.publish(Float32(data=0.0))
        spin_for(node, 0.05)
    return 0


def watch(node, laps, max_contacts, out, stall_s=60.0, timeout_s=3600.0):
    tel = Tel(node)
    t0 = time.monotonic()
    while tel.lap is None or tel.col is None:
        spin_for(node, 0.1)
        if time.monotonic() - t0 > 60:
            print('watch: no telemetry from the bridge', flush=True)
            return 5
    base_col = tel.col
    f = open(out, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['lap', 't', 'lap_time', 'collisions'])
    prev_lap, prev_col = tel.lap, tel.col
    t_progress = time.monotonic()
    print(f'watch: start lap_count={tel.lap} collision_count={tel.col}', flush=True)
    code = 0
    while True:
        spin_for(node, 0.1)
        now = time.monotonic()
        if tel.lap != prev_lap:
            spin_for(node, 0.15)            # last_lap_time lands a tick after lap_count
            prev_lap = tel.lap
            t_progress = now
            w.writerow([tel.lap, f'{now - t0:.2f}', f'{tel.last:.3f}', tel.col - base_col])
            f.flush()
            print(f'LAP {tel.lap - 1} time {tel.last:.3f} contacts {tel.col - base_col}', flush=True)
            if tel.lap >= laps + 1:
                code = 0
                break
        if tel.col != prev_col:
            prev_col = tel.col
            print(f'CONTACT {tel.col - base_col} at +{now - t0:.1f} s', flush=True)
            if tel.col - base_col >= max_contacts:
                spin_for(node, 2.0)          # keep the log running through the respawn
                code = 3
                break
        if now - t_progress > stall_s:
            print(f'STALL: no lap crossing for {stall_s:.0f} s', flush=True)
            code = 4
            break
        if now - t0 > timeout_s:
            code = 5
            break
    w.writerow(['end', f'{time.monotonic() - t0:.2f}', '', tel.col - base_col])
    f.close()
    return code


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    rclpy.init()
    node = rclpy.create_node('sim_ctl')
    cmd = sys.argv[1]
    try:
        if cmd == 'reset':
            code = reset(node)
        elif cmd == 'stop':
            code = stop(node)
        elif cmd == 'watch':
            args = sys.argv[2:]
            stall = float(args[args.index('--stall') + 1]) if '--stall' in args else 60.0
            tout = float(args[args.index('--timeout') + 1]) if '--timeout' in args else 3600.0
            code = watch(node, int(args[0]), int(args[1]), args[2], stall, tout)
        else:
            sys.exit(__doc__)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
