"""The simulator's tire, as a speed observer: wheel speed in, car speed out.

The sim spins each wheel to its commanded speed within a millisecond whatever
the car does, so the encoders report the THROTTLE, not the car: 5-12x high
from rest, ~1.1x from 2.5 m/s, low while braking (raceline/VEHICLE_MODEL.md on
main). Integrating them as distance puts every slip straight into the
position.

This runs the sim's own longitudinal model on the measured wheel speed u:

    v' = sign(S) mu(|S|) g - 0.273 v,    S = (u - v) / max(v, 4 m/s)

Two copies of that system driven by the same u converge, so the error is
bounded and does not drift. Measured offline against ground truth on four
runs: p90 0.17-0.28 m/s, launches, stalls and braking included.

Same model as pure_pursuit's `speed_source: tire` (_mu, _observe). KEEP IN STEP.
"""

import math

# Forward friction curve: effective extremum (0.15, 0.72), asymptote
# (0.25, 0.464), zero slope at both, flat beyond; Rigidbody linear drag 0.273 /s.
TIRE_S_PEAK, TIRE_MU_PEAK = 0.15, 0.72
TIRE_S_ASYM, TIRE_MU_ASYM = 0.25, 0.464
DRAG_LIN = 0.273
G = 9.81


class TireSpeedObserver:

    def __init__(self, rise_slope=3.0, v_slip_den=4.0, substeps=5):
        self.rise = float(rise_slope)
        self.den = float(v_slip_den)
        self.substeps = int(substeps)
        self.v = 0.0

    def mu(self, S):
        """Sim friction coefficient at longitudinal slip S (either sign)."""
        a = abs(S)
        if a <= TIRE_S_PEAK:
            t = a / TIRE_S_PEAK
            return TIRE_MU_PEAK * ((3.0 * t * t - 2.0 * t ** 3)
                                   + self.rise * (t - 2.0 * t * t + t ** 3))
        if a <= TIRE_S_ASYM:
            t = (a - TIRE_S_PEAK) / (TIRE_S_ASYM - TIRE_S_PEAK)
            return TIRE_MU_PEAK - (TIRE_MU_PEAK - TIRE_MU_ASYM) * (3.0 * t * t - 2.0 * t ** 3)
        return TIRE_MU_ASYM

    def step(self, u, dt):
        """Advance by dt on wheel speed u. Sub-stepped: near zero slip the
        linearised rate reaches ~25 /s, and one 50 ms Euler step would sit at
        the edge of stability. Forward only, like the follower's copy."""
        h = dt / self.substeps
        for _ in range(self.substeps):
            S = (u - self.v) / max(self.v, self.den)
            a = math.copysign(self.mu(S) * G, S) - DRAG_LIN * self.v
            self.v = max(0.0, self.v + a * h)
        return self.v

    def reset(self):
        self.v = 0.0
