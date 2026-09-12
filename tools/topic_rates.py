#!/usr/bin/env python3
"""Rate of every topic at once: count messages per topic for a window, print a table.

    python3 tools/topic_rates.py            # 15 s, every topic
    python3 tools/topic_rates.py 30         # 30 s
    python3 tools/topic_rates.py 15 /autodrive/roboracer_1/   # only that prefix

Each topic is subscribed with its first publisher's reliability/durability (as
`ros2 topic hz` does), raw (no deserialisation), so the count is cheap even on
the camera. Needs the same RMW and domain as the stack: with the racer
container on --network=host that is

    source /opt/ros/humble/setup.bash
    export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

Why every topic: the bridge publishes all its sensors from one socket handler,
so they arrive as one tick and should show one rate; anything slower is a node
of ours dropping or throttling, anything faster is a timer of ours.
"""
import collections
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rosidl_runtime_py.utilities import get_message

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
PREFIX = sys.argv[2] if len(sys.argv) > 2 else ''


def main():
    rclpy.init()
    node = Node('topic_rates')
    t_end = time.monotonic() + 1.5          # let discovery settle
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    stamps = collections.defaultdict(list)
    types = {}
    subs = []
    for name, tlist in sorted(node.get_topic_names_and_types()):
        if not name.startswith(PREFIX) or name.startswith('/rosout') or name.startswith('/parameter_events'):
            continue
        try:
            msg = get_message(tlist[0])
        except Exception as e:                    # noqa: BLE001
            print(f'{name}: cannot load {tlist[0]} ({e})', flush=True)
            continue
        pubs = node.get_publishers_info_by_topic(name)
        if pubs:
            q = pubs[0].qos_profile
            qos = QoSProfile(reliability=q.reliability, durability=q.durability,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        else:
            qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        types[name] = tlist[0].split('/')[-1]
        stamps[name]
        subs.append(node.create_subscription(
            msg, name, (lambda n: lambda m: stamps[n].append(time.monotonic()))(name), qos, raw=True))
    print(f'{len(subs)} topics, counting for {DUR:.0f} s ...', flush=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < DUR:
        rclpy.spin_once(node, timeout_sec=0.02)
    print(f'\n{"topic":44s} {"type":18s} {"msgs":>6s} {"Hz":>7s} {"median":>7s} {"p90":>7s} {"max ms":>7s}')
    for name in sorted(stamps):
        t = np.asarray(stamps[name])
        if len(t) < 2:
            print(f'{name:44s} {types[name]:18s} {len(t):6d} {"-":>7s}')
            continue
        d = np.diff(t) * 1000.0
        span = t[-1] - t[0]
        print(f'{name:44s} {types[name]:18s} {len(t):6d} {len(t) / span if span else 0:7.2f} '
              f'{np.median(d):7.1f} {np.percentile(d, 90):7.1f} {d.max():7.0f}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
