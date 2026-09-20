"""Observation construction, shared between AutoDRIVE and any future gym sim.

Keeping this in one module is deliberate: if you later train in f1tenth_gym and
transfer the policy, both sides MUST produce byte-identical observations.
"""
import numpy as np


class LidarFOV:
    """Crops a LaserScan to a forward field of view and min-pools it.

    Configured lazily from the first real scan so it tracks the bridge's actual
    angle_min / angle_increment instead of hard-coding them.
    """

    def __init__(self, fov_half_deg: float, n_beams: int, range_max: float):
        self.fov_half_deg = float(fov_half_deg)
        self.n_beams = int(n_beams)
        self.range_max = float(range_max)
        self._edges = None
        self._i0 = None
        self._i1 = None

    def configure(self, angle_min: float, angle_increment: float, n_total: int):
        half = np.radians(self.fov_half_deg)
        i0 = int(round((-half - angle_min) / angle_increment))
        i1 = int(round((half - angle_min) / angle_increment))
        i0, i1 = max(0, i0), min(n_total, i1)
        width = i1 - i0
        if width < self.n_beams:
            raise ValueError(
                f"FOV window is {width} raw beams but n_beams={self.n_beams}; "
                "reduce n_beams or widen fov_half_deg."
            )
        self._i0, self._i1 = i0, i1
        # Segment starts for np.minimum.reduceat over the cropped window.
        self._edges = np.linspace(0, width, self.n_beams + 1).astype(np.intp)[:-1]

    @property
    def ready(self) -> bool:
        return self._edges is not None

    def process(self, ranges) -> np.ndarray:
        """Raw scan -> (n_beams,) array of metres, min-pooled and sanitised."""
        raw = np.asarray(ranges, dtype=np.float32)
        # inf means "nothing within range_max", nan means bad return. Both are
        # safely represented as max range. -inf would mean an invalid near read.
        raw = np.nan_to_num(raw, nan=self.range_max,
                            posinf=self.range_max, neginf=0.0)
        win = raw[self._i0:self._i1]
        # Min-pool, never stride: striding can skip the single beam that sees a
        # thin obstacle or the edge of a barrier.
        pooled = np.minimum.reduceat(win, self._edges)
        return np.clip(pooled, 0.0, self.range_max)


def beam_features(beams: np.ndarray, range_max: float, safe_dist: float):
    """Derive the scalar geometry terms the reward needs from pooled beams.

    beams[0] is the right-most ray (-fov), beams[-1] the left-most (+fov).
    """
    n = len(beams)
    k = max(1, n // 18)                    # ~10 deg of beams at each flank
    d_right = float(beams[:k].mean())
    d_left = float(beams[-k:].mean())

    # Map-free centring proxy: lateral asymmetry, 0 when equidistant from both
    # walls, ->1 when hugging one of them.
    denom = d_left + d_right
    center_err = abs(d_left - d_right) / denom if denom > 1e-6 else 0.0

    # Forward sector (+/- a third of the FOV) for the proximity term.
    lo, hi = n // 2 - n // 6, n // 2 + n // 6
    front_min = float(beams[lo:hi].min())
    min_range = float(beams.min())

    # 0 when clear, ->1 as the nearest wall closes inside safe_dist.
    prox = max(0.0, 1.0 - min_range / safe_dist)

    return {
        "d_left": d_left,
        "d_right": d_right,
        "center_err": float(np.clip(center_err, 0.0, 1.0)),
        "front_min": front_min,
        "min_range": min_range,
        "prox": float(prox),
    }


def ttc_forward(beams: np.ndarray, fov_half_deg: float, speed: float,
                sector_deg: float = 10.0, range_max: float = 0.0) -> float:
    """Seconds until the nearest thing in the forward sector is reached at the
    current speed, per beam r_i / (v cos th_i). A wall running alongside the
    car is not ahead: its beams are long and nearly sideways. inf when clear.

    `range_max` (0 = ignore) marks saturated beams as clear. Without it a beam
    reading exactly range_max -- which means "nothing within range", not "a
    wall at 10 m" -- divides down to a 1.0 s time-to-collision at 10 m/s, so
    the penalty taxed a completely empty straight above 8.3 m/s.
    """
    n = len(beams)
    th = np.linspace(-fov_half_deg, fov_half_deg, n) * (np.pi / 180.0)
    sel = np.abs(th) <= sector_deg * (np.pi / 180.0)
    r = beams[sel]
    if range_max > 0.0:
        r = np.where(r >= range_max - 1e-3, np.inf, r)
    v_along = max(abs(speed), 0.1) * np.cos(th[sel])
    return float(np.min(r / v_along))


def ttc_penalty(ttc: float, ttc_ref: float) -> float:
    """0 when the wall ahead is more than ttc_ref seconds away, ->1 at contact."""
    if ttc_ref <= 0.0 or not np.isfinite(ttc):
        return 0.0
    return max(0.0, 1.0 - ttc / ttc_ref)


def build_obs(beams, range_max, speed, v_max, yaw_rate, yaw_rate_max,
              prev_steer, prev_throttle, slip, slip_max=0.5, yaw_res_max=8.0) -> np.ndarray:
    """Assemble the policy input. Every component is clipped into [-1, 1].

    Layout: beams | u (wheel speed) | yaw_rate | prev_steer | prev_throttle
            | v_est | S | yaw_residual          -- `slip` = (v_est, S, yaw_residual)

    Everything is race-legal: LiDAR, encoders, IMU, our own last command and
    the sim's published vehicle model. The previous action is in the state so
    the one-step command delay and the steering-jerk penalty stay Markovian.
    """
    n = len(beams)
    v_est, s, yres = slip
    obs = np.empty(n + 7, dtype=np.float32)
    obs[:n] = beams / range_max
    obs[n + 0] = np.clip(speed / v_max, -1.0, 1.0)
    obs[n + 1] = np.clip(yaw_rate / yaw_rate_max, -1.0, 1.0)
    obs[n + 2] = np.clip(prev_steer, -1.0, 1.0)
    obs[n + 3] = np.clip(prev_throttle, -1.0, 1.0)
    obs[n + 4] = np.clip(v_est / v_max, -1.0, 1.0)
    obs[n + 5] = np.clip(s / slip_max, -1.0, 1.0)
    obs[n + 6] = np.clip(yres / yaw_res_max, -1.0, 1.0)
    return np.clip(obs, -1.0, 1.0, out=obs)


def yaw_from_quat(x, y, z, w) -> float:
    return float(np.arctan2(2.0 * (w * z + x * y),
                            1.0 - 2.0 * (y * y + z * z)))
