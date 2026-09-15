"""Race-legal sensor processing, kept free of ROS so it can be unit-tested.

Everything here consumes only permitted topics (wheel encoders, IMU, our own
steering command) plus the simulator's published vehicle model
(raceline/VEHICLE_MODEL.md). Nothing reads /odom or /ips.
"""
import math
from collections import deque

from .config import VehicleCfg


class WheelSpeed:
    """Wheel surface speed u [m/s] from the two rear encoders.

    * Encoder `position` is the cumulative wheel angle in RADIANS (measured;
      the Technical Guide's "ticks" figure is wrong by ~300x).
    * The rate is taken over a short window of samples rather than one tick:
      the bridge stamps messages at socket receipt, so single-tick dt carries
      the whole loop's jitter (median 12.8 ms, max 25 at the shimmed loop) and
      a one-tick derivative is ~15-100% noisy. Spanning `window` ticks averages
      it out at the cost of ~1.5 control steps of lag.
    * A per-sample jump larger than `step_max` rad is a counter discontinuity
      (the simulator's ResetManager restores the encoder to its spawn value on
      every reset; a lap is ~480 rad), never real motion (one tick at 24 m/s is
      ~9 rad). It clears that wheel's window instead of producing a spike.
    * NOTE this is the THROTTLE ECHO: the sim spins the wheel to 25.25*throttle
      within a millisecond regardless of the car, so u == car speed only at
      steady state. Car speed is `TireObserver`.
    """

    def __init__(self, wheel_radius: float, window: int = 3,
                 step_max: float = 300.0, dt_min: float = 1e-4, dt_max: float = 0.5):
        self.r = float(wheel_radius)
        self.window = int(window)
        self.step_max, self.dt_min, self.dt_max = float(step_max), float(dt_min), float(dt_max)
        self._hist = {"l": deque(maxlen=self.window + 1), "r": deque(maxlen=self.window + 1)}
        self._rate = {"l": None, "r": None}          # wheel rad/s
        self.discontinuities = 0

    def reset(self):
        for h in self._hist.values():
            h.clear()
        self._rate = {"l": None, "r": None}

    def update(self, side: str, angle: float, t: float):
        h = self._hist[side]
        if h:
            a_prev, t_prev = h[-1]
            dt = t - t_prev
            if abs(angle - a_prev) > self.step_max:
                self.discontinuities += 1
                h.clear(); self._rate[side] = None
                h.append((angle, t))
                return
            if dt <= self.dt_min:
                return                                  # duplicate / stale stamp
            if dt > self.dt_max:                        # stall or dropped stream
                h.clear(); self._rate[side] = None
        h.append((angle, t))
        if len(h) >= 2:
            a0, t0 = h[0]
            a1, t1 = h[-1]
            span = t1 - t0
            if span > self.dt_min:
                self._rate[side] = (a1 - a0) / span

    @property
    def speed(self) -> float:
        rates = [v for v in self._rate.values() if v is not None]
        return float(sum(rates) / len(rates)) * self.r if rates else 0.0


def tire_mu(s_abs: float, veh: VehicleCfg) -> float:
    """Simulator forward friction coefficient at |slip| (Unity WheelFrictionCurve,
    Hermite with zero tangent at the extremum and asymptote, flat beyond)."""
    a = abs(s_abs)
    if a <= veh.tire_s_peak:
        t = a / veh.tire_s_peak
        return veh.tire_mu_peak * ((3.0 * t * t - 2.0 * t ** 3)
                                   + veh.tire_rise_slope * (t - 2.0 * t * t + t ** 3))
    if a <= veh.tire_s_asym:
        t = (a - veh.tire_s_peak) / (veh.tire_s_asym - veh.tire_s_peak)
        return veh.tire_mu_peak - (veh.tire_mu_peak - veh.tire_mu_asym) * (3.0 * t * t - 2.0 * t ** 3)
    return veh.tire_mu_asym


def slip(u: float, v: float, veh: VehicleCfg) -> float:
    """Longitudinal slip as PhysX defines it: (u - v) / max(v, slip_den)."""
    return (u - v) / max(v, veh.slip_den)


class TireObserver:
    """Race-legal car speed from wheel speed alone, via the sim's own tire model.

        v' = sign(S) mu(|S|) g - drag v,   S = (u - v)/max(v, 4)

    If the estimate is high, the slip it computes is too small, the force too
    small, and it falls back toward the car; if low, the reverse. That
    contraction bounds the error (classical stack: p90 0.15-0.22 m/s vs truth).
    One-wheel model on purpose: the 4-wheel variant is more accurate but, on the
    classical car, removed an accidental throttle limiter and caused a hit
    (VEHICLE_MODEL.md s7, icra run 14).

    Sub-stepped so the Euler step stays <= 10 ms: near zero slip the curve's
    linearised rate is ~25 /s and a 50 ms step is at the edge of stability.
    """

    def __init__(self, veh: VehicleCfg, h_max: float = 0.010):
        self.veh = veh
        self.h_max = float(h_max)
        self.v = 0.0
        self.a = 0.0
        self.s = 0.0

    def reset(self, v0: float = 0.0):
        self.v, self.a, self.s = float(v0), 0.0, 0.0

    def update(self, u: float, dt: float) -> float:
        veh = self.veh
        if not (0.0 < dt <= 0.5):
            return self.v
        n = max(1, int(math.ceil(dt / self.h_max)))
        h = dt / n
        for _ in range(n):
            s = slip(u, self.v, veh)
            a = math.copysign(tire_mu(s, veh) * veh.g, s) - veh.drag * self.v
            self.v = max(0.0, self.v + a * h)
            self.a, self.s = a, s
        return self.v


def yaw_residual(yaw_rate: float, v_est: float, steer_cmd: float, veh: VehicleCfg) -> float:
    """Measured yaw rate minus the kinematic-bicycle yaw rate for the commanded
    steering. ~0 while the front tires grip; large when the car understeers
    (|measured| < |expected|) or oversteers. Sign convention: ROS, CCW positive;
    positive steering command = left = positive yaw (matches the devkit).
    """
    delta = float(steer_cmd) * veh.max_steer_rad
    expected = v_est * math.tan(delta) / veh.wheelbase
    return float(yaw_rate - expected)
