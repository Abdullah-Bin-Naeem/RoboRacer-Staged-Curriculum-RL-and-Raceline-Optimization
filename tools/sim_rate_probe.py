#!/usr/bin/env python3
"""Measure the simulator's data rate with an ideal bridge and no ROS.

The simulator emits telemetry only in reply to a 'Bridge' message (Socket.cs,
OnBridge -> EmitTelemetry), so nothing can measure it alone -- but the replier
need not be the devkit bridge. This stand-in listens on the bridge's port,
answers every message immediately with zero commands, and records the arrival
times. What it prints is the simulator's own cycle: the frame rate the machine
sustains against the 1 kHz physics step, and the upper bound the real bridge,
which also publishes a dozen ROS topics per frame, can never exceed.

Stop the bridge first (it owns port 4567), run this, then press Connect in the
simulator. The car stays parked (throttle 0); the rate does not depend on it.

    python3 tools/sim_rate_probe.py            # 30 s
    python3 tools/sim_rate_probe.py 60         # 60 s

System python3: it needs the bridge's own socketio / gevent, not the venv's.
"""
import sys
import time

import numpy as np
import socketio
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
PORT = 4567

sio = socketio.Server(async_mode='gevent')
app = socketio.WSGIApp(sio)
stamps = []
t_connect = None


@sio.on('connect')
def on_connect(sid, environ):
    global t_connect
    t_connect = time.monotonic()
    print(f'simulator connected ({sid}); measuring for {DURATION:.0f} s ...', flush=True)


@sio.on('Bridge')
def on_bridge(sid, data):
    now = time.monotonic()
    stamps.append(now)
    # Answer at once, exactly the fields the simulator reads (autodrive_bridge.py).
    sio.emit('Bridge', data={'V1 Throttle': '0.0', 'V1 Steering': '0.0', 'V1 Reset': 'False'})
    if t_connect is not None and now - t_connect >= DURATION:
        report()
        raise SystemExit(0)


def report():
    t = np.asarray(stamps)
    if len(t) < 10:
        print(f'only {len(t)} messages; is the simulator connected?')
        return
    d = np.diff(t) * 1000.0
    span = t[-1] - t[0]
    print(f'\n{len(t)} messages in {span:.1f} s  ->  {len(t) / span:.2f} Hz  (median interval {np.median(d):.1f} ms)')
    print(f'interval p10 {np.percentile(d, 10):.1f}  p50 {np.percentile(d, 50):.1f}  p90 {np.percentile(d, 90):.1f}  '
          f'p99 {np.percentile(d, 99):.1f}  max {d.max():.0f} ms')
    f = np.median(d)
    for lo, hi, name in ((0.0, 1.5 * f, 'one frame'), (1.5 * f, 2.5 * f, 'two frames'), (2.5 * f, 1e9, 'stall (>2.5 frames)')):
        m = (d >= lo) & (d < hi)
        print(f'  {name:22s} {m.sum():6d}  ({m.mean() * 100:5.1f} %)')
    print('one frame ~ the simulator\'s rendered frame; two frames = a reply missed the frame boundary; '
          'compare with the FPS on the simulator\'s HUD')


if __name__ == '__main__':
    print(f'listening on :{PORT} as a stand-in bridge -- stop the real bridge first, then press Connect in the simulator')
    try:
        pywsgi.WSGIServer(('', PORT), app, handler_class=WebSocketHandler, log=None).serve_forever()
    except SystemExit:
        pass
