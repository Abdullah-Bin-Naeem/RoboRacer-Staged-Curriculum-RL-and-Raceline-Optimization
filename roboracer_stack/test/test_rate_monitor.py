"""The rate statistics in tools/rate_monitor.py, without a ROS graph.

The property worth pinning is the one that motivated the tool: a single stall
BARELY MOVES THE MEAN. `ros2 topic hz` reports the mean, which is why a 300 ms
freeze -- enough to put the car in a wall -- never showed up in testing. If a
refactor ever makes `hz` alone sensitive to one stall, that is a nice-looking
change that quietly removes the reason `gap` and `stalls` exist.

    pytest roboracer_stack/test/test_rate_monitor.py

tools/ is not an installed package (setup.py excludes it), so the module is
loaded from its path.
"""

import importlib.util
import os
import sys
import types

import pytest


def _load():
    """Import rate_monitor with the ROS imports stubbed out.

    It only needs rclpy for the Node base class and the QoS enums; the rate
    maths under test is plain Python and should be testable on a machine with
    no ROS installed at all -- which is where this test will usually run.
    """
    for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs',
                 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg'):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules['rclpy.node'].Node = object
    qos = sys.modules['rclpy.qos']
    for name in ('QoSDurabilityPolicy', 'QoSHistoryPolicy', 'QoSProfile',
                 'QoSReliabilityPolicy'):
        setattr(qos, name, type(name, (), {
            'VOLATILE': 0, 'RELIABLE': 0, 'KEEP_LAST': 0,
            '__init__': lambda self, **kw: None}))
    for name in ('Imu', 'JointState', 'LaserScan'):
        setattr(sys.modules['sensor_msgs.msg'], name, object)
    for name in ('Float32', 'Float32MultiArray', 'Int32'):
        setattr(sys.modules['std_msgs.msg'], name, object)

    path = os.path.join(os.path.dirname(__file__), os.pardir, 'tools',
                        'rate_monitor.py')
    spec = importlib.util.spec_from_file_location('rate_monitor', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


rm = _load()


def _stream(gaps, window=3.0, seed_at=None):
    """A Rate fed one gap at a time. seed_at primes the median, as a report does."""
    r = rm.Rate(window)
    t = 0.0
    r.add(t)
    for i, g in enumerate(gaps):
        t += g
        r.add(t)
        if seed_at is not None and i == seed_at:
            r.stats(t)
    return r, t


def test_percentile():
    assert rm._pct([5.0], 0.95) == 5.0
    assert rm._pct([0, 1, 2, 3, 4], 0.5) == pytest.approx(2.0)
    assert rm._pct([0, 1, 2, 3, 4], 0.95) == pytest.approx(3.8)
    assert rm._pct([], 0.5) != rm._pct([], 0.5)          # NaN


def test_steady_rate():
    r, t = _stream([0.025] * 199)
    hz, p5, gap = r.stats(t)
    assert hz == pytest.approx(40.0, abs=0.5)
    assert p5 == pytest.approx(40.0, abs=0.5)
    assert gap == pytest.approx(0.025, abs=0.001)
    assert r.stalls == 0


def test_one_stall_hides_in_the_mean_but_not_in_gap_or_stalls():
    """The whole reason this tool reports more than an average."""
    gaps = [0.025] * 199
    gaps[150] = 0.200
    r, t = _stream(gaps, seed_at=100)
    hz, p5, gap = r.stats(t)

    assert hz > 36.0, 'the mean must stay deceptively healthy -- that is the point'
    assert gap > 0.19, 'the largest gap must expose it'
    assert r.stalls == 1, 'and it must be counted'
    # p95 over ~120 gaps cannot see a single outlier. Documented, not a defect:
    # p5_hz is a jitter measure, and gap/stalls are the stall detector.
    assert p5 == pytest.approx(40.0, abs=1.0)


def test_p5_exposes_sustained_jitter():
    r, t = _stream([0.025 if i % 2 else 0.060 for i in range(120)])
    hz, p5, _ = r.stats(t)
    assert p5 < hz, 'the slow half must drag the percentile below the mean'
    assert p5 == pytest.approx(1 / 0.060, abs=0.5)


def test_window_evicts_and_reports_nan_when_silent():
    """A topic that stops is NaN, which the display renders as DEAD."""
    r, t = _stream([0.025] * 199)
    hz, p5, _ = r.stats(t + 5.0)
    assert hz != hz and p5 != p5


def test_never_published_and_went_silent_read_differently():
    fresh = rm.Rate(3.0)
    assert rm._hz(fresh, float('nan')).strip() == '--'
    silent, t = _stream([0.025] * 10)
    assert rm._hz(silent, float('nan')).strip() == 'DEAD'
    assert rm._hz(silent, 44.8).strip() == '44.8Hz'


def test_undefined_statistics_do_not_print_as_numbers():
    assert rm._val(float('nan')).strip() == '--'
    assert rm._val(44.8).strip() == '44.8'


def test_stall_delta_reports_only_new_stalls():
    class Monitor:
        _delta = rm.RateMonitor._delta

    m = Monitor()
    m._prev_stalls = {}
    r = rm.Rate(3.0)
    r.stalls = 3
    assert m._delta('imu', r) == '(+3)'
    assert m._delta('imu', r) == '', 'a count that has not moved is not news'
    r.stalls = 5
    assert m._delta('imu', r) == '(+2)'


class _Laps:
    """A RateMonitor reduced to its lap bookkeeping, with no ROS graph.

    Binding the real methods onto a bare object is enough: none of them touch
    anything but the lap attributes and self.imu.
    """

    _cb_lap_count = rm.RateMonitor._cb_lap_count
    _cb_last_lap = rm.RateMonitor._cb_last_lap
    _cb_best_lap = rm.RateMonitor._cb_best_lap
    _flush_pending_lap = rm.RateMonitor._flush_pending_lap
    _mark_lap = rm.RateMonitor._mark_lap
    _close_lap = rm.RateMonitor._close_lap
    _print_lap = rm.RateMonitor._print_lap

    def __init__(self):
        self.imu = rm.Rate(3.0)
        self.lap_count = None
        self.last_lap = float('nan')
        self.best_lap = float('nan')
        self.laps = []
        self._pending_lap = None
        self._last_lap_fresh = False
        self._lap_mark = None
        self._lap_gap = 0.0


class _Msg:
    def __init__(self, data):
        self.data = data


def test_lap_is_labelled_with_its_own_time_not_the_previous_one():
    """The bridge publishes lap_count BEFORE last_lap_time in the same burst.

    Printing on the lap_count callback would tag lap N with lap N-1's time. The
    lap is parked until the time arrives; this is that guarantee.
    """
    m = _Laps()
    m._cb_lap_count(_Msg(1))          # first lap seen: nothing closed yet
    assert m.laps == []

    m._cb_lap_count(_Msg(2))          # lap 2 opened -> lap 2 parked
    assert m._pending_lap is not None
    assert m.laps == [], 'must not print before the rest of the burst lands'
    m._cb_last_lap(_Msg(6.382))
    m._cb_best_lap(_Msg(6.382))
    m._flush_pending_lap()            # what the next report tick does
    assert m._pending_lap is None
    assert m.laps[-1][0] == 2 and m.laps[-1][1] == pytest.approx(6.382)

    m._cb_lap_count(_Msg(3))
    m._cb_last_lap(_Msg(6.351))
    m._flush_pending_lap()
    assert [(lap, round(t, 3)) for lap, t, _h, _g, _s in m.laps] == \
        [(2, 6.382), (3, 6.351)]


def test_flushing_with_nothing_parked_is_a_no_op():
    m = _Laps()
    m._flush_pending_lap()
    assert m.laps == []


def test_a_lap_waits_for_a_time_that_is_actually_its_own():
    """A flush that lands between lap_count and last_lap_time must not print.

    Printing there would use the PREVIOUS lap's time -- the off-by-one this
    whole park-and-flush dance exists to prevent.
    """
    m = _Laps()
    m._cb_lap_count(_Msg(1))
    m._cb_last_lap(_Msg(6.382))       # lap 1's time
    m._cb_lap_count(_Msg(2))          # lap 2 closes; its time has NOT arrived
    m._flush_pending_lap()
    assert m.laps == [], 'must not attribute 6.382 to lap 2'
    m._cb_last_lap(_Msg(6.351))       # now it has
    m._flush_pending_lap()
    assert m.laps[-1][:2] == (2, pytest.approx(6.351))


def test_a_missing_lap_time_topic_eventually_prints_a_blank_rather_than_nothing():
    m = _Laps()
    m._cb_lap_count(_Msg(1))
    m._cb_lap_count(_Msg(2))
    m._flush_pending_lap()
    assert m.laps == []
    # Backdate the park past the grace period.
    lap, tick, parked_at = m._pending_lap
    m._pending_lap = (lap, tick, parked_at - rm._LAP_FLUSH_S - 0.1)
    m._flush_pending_lap()
    assert len(m.laps) == 1 and m.laps[0][1] != m.laps[0][1]   # NaN


def test_per_lap_gap_survives_the_rolling_window():
    """A stall early in a 6 s lap is out of the 3 s window when the lap closes.

    So the per-lap worst gap is accumulated, not read from the window. If this
    ever regresses, every lap reports a clean gap and the tool stops answering
    the question it was built for.
    """
    m = _Laps()
    m._cb_lap_count(_Msg(1))
    m._lap_gap = 0.287                # a 287 ms freeze, seen mid-lap by _report
    m._cb_lap_count(_Msg(2))
    m._cb_last_lap(_Msg(7.104))
    m._flush_pending_lap()
    assert m.laps[-1][3] == pytest.approx(0.287)
    assert m._lap_gap == 0.0, 'and it must reset for the next lap'


def test_best_lap_comes_from_the_simulator_not_from_our_own_minimum():
    m = _Laps()
    m._cb_best_lap(_Msg(6.351))
    assert m.best_lap == pytest.approx(6.351)
