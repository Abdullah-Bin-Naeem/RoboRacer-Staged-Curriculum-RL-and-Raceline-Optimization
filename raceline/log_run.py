#!/usr/bin/env python3
"""Record a run to CSV in the schema raceline/analyze_run.py and
plot_speed_tracking.py read.

    python3 log_run.py out.csv [--hz 20]

Ground truth pose from /ips and speed from /odom -- DEVELOPMENT ONLY, both are
restricted at race time. The follower's ~/status vector is recorded as the
pp_<name> columns, in STATUS_FIELDS order.
"""
import argparse
import csv
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray, Int32

NS = '/autodrive/roboracer_1'
PP = ('v_est', 'v_enc', 'v_pose', 'v_target', 'u_cmd', 'throttle', 'steering',
      'ld', 'e_lat', 'e_head', 'kappa', 's', 'slip', 'a_imu', 'delay')
COLS = ['t', 'true_x', 'true_y', 'speed', 'collisions'] + [f'pp_{n}' for n in PP]


class Logger(Node):
    def __init__(self, path, hz):
        super().__init__('run_logger')
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(COLS)
        self.x = self.y = self.v = float('nan')
        # None until the sim has actually told us. Writing rows with a made-up
        # 0 before the first message makes the run look like it opened with
        # however many collisions the PREVIOUS run left on the cumulative
        # counter -- that is not a collision, it is the counter arriving.
        self.coll = None
        self.lap = 0
        self.coll0 = 0
        self._printed_lap = -1
        self.best = float('nan')
        self.status = [float('nan')] * len(PP)
        self.t0 = None
        self.n = 0
        self.create_subscription(Point, f'{NS}/ips', self._ips, 10)
        self.create_subscription(Odometry, f'{NS}/odom', self._odom, 10)
        self.create_subscription(Int32, f'{NS}/collision_count', self._coll, 10)
        self.create_subscription(Int32, f'{NS}/lap_count', self._lapc, 10)
        self.create_subscription(Float32, f'{NS}/best_lap_time', self._best, 10)
        self.create_subscription(Float32, f'{NS}/last_lap_time', self._last, 10)
        # The follower's node name decides the topic; try both spellings.
        for t in ('/pure_pursuit/status', '/racer_control/pure_pursuit/status'):
            self.create_subscription(Float32MultiArray, t, self._status, 10)
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(f'logging -> {path} at {hz} Hz')

    def _ips(self, m):
        self.x, self.y = m.x, m.y

    def _odom(self, m):
        t = m.twist.twist.linear
        self.v = math.sqrt(t.x * t.x + t.y * t.y)

    def _coll(self, m):
        if self.coll is None:
            self.coll0 = m.data
            self.get_logger().info(f'collision counter starts at {m.data} (cumulative since sim start)')
        elif m.data != self.coll:
            self.get_logger().warn(f'COLLISION -> count {m.data}  (+{m.data - self.coll0} this run)')
        self.coll = m.data

    def _lapc(self, m):
        self.lap = m.data

    def _best(self, m):
        self.best = m.data

    def _last(self, m):
        # The sim republishes this every frame, not once per lap: print only
        # when the lap counter moves, or the console is one line per message.
        if m.data > 0 and self.lap != self._printed_lap:
            self._printed_lap = self.lap
            self.get_logger().info(f'lap {self.lap}: {m.data:.2f} s   '
                                   f'best {self.best:.2f}   collisions {self.coll}')

    def _status(self, m):
        if len(m.data) >= len(PP):
            self.status = list(m.data[:len(PP)])

    def _tick(self):
        if self.coll is None:
            return                      # counter not yet received; see _coll
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        self.w.writerow([f'{now - self.t0:.4f}', f'{self.x:.5f}', f'{self.y:.5f}',
                         f'{self.v:.4f}', self.coll]
                        + [f'{s:.5f}' for s in self.status])
        self.n += 1
        # Flush EVERY row. It used to be every 400 (20 s at 20 Hz), and
        # destroy_node() never runs when stop_follower.sh SIGKILLs the logger --
        # which is exactly what an abort-on-collision does. E8 lost its only
        # collision that way: the console log recorded it, the CSV ended 8.6 s
        # earlier, and ab_report.py duly reported "0 collisions" for a run that
        # had failed the gate. A 20 Hz flush of one short row costs nothing.
        self.f.flush()

    def destroy_node(self):
        self.f.flush(), self.f.close()
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--hz', type=float, default=20.0)
    a = ap.parse_args(sys.argv[1:])
    rclpy.init()
    n = Logger(a.out, a.hz)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
