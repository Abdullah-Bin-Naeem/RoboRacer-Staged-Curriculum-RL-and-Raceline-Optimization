"""The racing line: read the CSV, derive everything the follower needs from it.

The line itself is planned offline -- `raceline/optimize_raceline.py` on the
main branch solves for minimum curvature against the map and writes the CSV
this module reads. Nothing here plans; this is the runtime half, which turns
those columns into the arrays the controller indexes every cycle. The finished
lines live in this package's `raceline/` and are resolved through
`common.frames.RACELINE_DIR`.

CSV columns, in order, as written by export_csv():

    0 s        arc length along the line (m)
    1 x  2 y   the point, in map frame
    3 psi      heading of the line there (rad)
    4 kappa    signed curvature (1/m), positive = left turn
    5 w_left   free space to the left wall at that point (m)
    6 w_right  ...and to the right
    7 v_mps    the velocity profile, optional
"""

import numpy as np

# Half the car's width. What a corner-cutting chord may NOT consume of the
# free space on the inside of the curve.
HALF_WIDTH = 0.135


class Raceline:
    """One racing line, loaded. Plain attributes; the follower reads them directly.

    s, x, y, psi, kappa   the line, sampled uniformly in arc length
    margin_in             room to the wall on the INSIDE of each point's curve,
                          minus half the car -- what a chord may consume
    v                     the velocity profile, or None if the CSV has no v_mps
    a                     the profile's own longitudinal acceleration, v dv/ds,
                          closed loop. Used to place the slip band where the car
                          will actually be when a command lands.
    profile_a_lat         the lateral acceleration the profile itself assumes,
                          which is what the delay derate scales down
    lap_len               s of the last point plus one sample step
    """

    def __init__(self, path):
        data = np.loadtxt(path, delimiter=',')
        self.path = path
        self.s, self.x, self.y = data[:, 0], data[:, 1], data[:, 2]
        self.psi, self.kappa = data[:, 3], data[:, 4]
        # The CSV's width columns are ray-cast from the map at the line
        # (optimize_raceline.export_csv).
        self.margin_in = np.where(self.kappa > 0.0, data[:, 6], data[:, 5]) - HALF_WIDTH
        self.v = data[:, 7] if data.shape[1] > 7 else None
        if self.v is not None:
            ds = np.hypot(np.roll(self.x, -1) - np.roll(self.x, 1),
                          np.roll(self.y, -1) - np.roll(self.y, 1))
            self.a = self.v * (np.roll(self.v, -1) - np.roll(self.v, 1)) / np.maximum(ds, 1e-3)
            self.profile_a_lat = float(np.max(self.v ** 2 * np.abs(self.kappa)))
        else:
            self.a = np.zeros_like(self.x)
            self.profile_a_lat = 0.0
        self.lap_len = float(self.s[-1] + (self.s[1] - self.s[0]))

    def __len__(self):
        return len(self.s)


def load_raceline(path):
    """Read a raceline CSV. Raises if `path` is empty -- there is no default here."""
    if not path:
        raise RuntimeError('path_csv parameter is required')
    return Raceline(path)
