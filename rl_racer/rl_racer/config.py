"""All tunables for SAC training on AutoDRIVE RoboRacer, in one place.

Reward weights are the thing you will actually iterate on -- everything else
should be left alone until the reward is producing sane behaviour.
"""
from dataclasses import dataclass, asdict, field


@dataclass
class ObsCfg:
    # LiDAR field of view. The AutoDRIVE scan is 1080 beams over +/-135 deg;
    # we keep only the forward +/-fov_half_deg because rear beams carry no
    # information for a solo time trial.
    # +/-100, not 90: at the apex of a >=180 deg hairpin the exit corridor sits
    # at exactly +/-90 deg from the heading, i.e. on the old FOV boundary, where
    # min-pooling could mask it behind a nearer wall edge. 100 deg gives real
    # margin. Not wider: the 90-135 deg quadrants measured std 0.18-0.24 m
    # (near-constant adjacent wall), so they carry little information here.
    # Raw indices 140..940 = 800 beams -> exactly 8 per bin, 2.0 deg/beam.
    fov_half_deg: float = 90.0   # v3 value
    n_beams: int = 90          # v3 value -> 95-dim observation
    range_max: float = 10.0    # matches bridge range_max, used to normalise
    # MUST stay above the top speed throttle_max can reach, otherwise the
    # observation clips and the agent cannot tell 5 m/s from 12 m/s -- and
    # the speed reward saturates, removing any incentive to go faster.
    v_max: float = 20.0        # v3 value
    yaw_rate_max: float = 5.0  # rad/s, for yaw-rate normalisation only
    # COMPETITION LEGALITY: /odom and /ips are restricted to debugging and must
    # not feed the policy at race time. Speed therefore comes from the wheel
    # encoders and yaw rate/orientation from the IMU -- all permitted topics.
    # /odom is still used for the PROGRESS REWARD, which is training-only and
    # never an input at inference.
    use_encoder_speed: bool = True
    # MEASURED: encoder position is wheel angle in RADIANS (4 trials gave
    # 0.05815 m/unit vs 0.0590 for the radians hypothesis, 301x off for ticks).
    # The Technical Guide's "cumulative ticks, 1920/rev" is wrong.
    # 0.0581, not the guide's 0.0590: two independent measurements agree
    # (path-integration 0.05815, steady-state speed match 0.05813). The ~1.5%
    # gap is residual tyre slip under load; 0.0581 makes encoder speed match
    # true speed to <1% at steady state.
    wheel_radius: float = 0.0581

    @property
    def dim(self) -> int:
        # beams + [speed, yaw_rate, prev_steer, prev_throttle, throttle_cap]
        return self.n_beams + 5


@dataclass
class ActionCfg:
    # Full mechanical steering range. The car cannot steer past its own limit
    # anyway, and capping this below 1.0 just makes hairpins unreachable.
    max_steer: float = 1.0
    throttle_min: float = 0.0   # no braking/reverse for now
    # Deliberately generous so the ceiling is NOT what limits lap time. Probe
    # measured throttle 0.10 -> 2.5 m/s, so 0.5 is roughly a 12 m/s ceiling --
    # well above any sane racing-line speed, i.e. effectively non-binding.
    # Watch action/throttle_sat: if it pins near 1.0 the cap IS binding, raise
    # this to 1.0. If it sits interior, the cap is not what is limiting speed.
    throttle_max: float = 0.20  # STARTING cap; the curriculum raises it


@dataclass
class RewardCfg:
    # --- what we pay for ---
    w_progress: float = 5.0     # per metre of forward progress (dominant term)
    w_speed: float = 0.2        # v3 value
    # --- what we charge for ---
    w_center: float = 0.3       # lateral asymmetry, 0 = perfectly centred
    w_smooth: float = 0.3       # |steer_t - steer_{t-1}|, steering smoothness
    w_prox: float = 0.5         # proximity to a wall
    step_penalty: float = 0.02  # small time cost, discourages dawdling
    # --- lap bonus: DISABLED (w_lap = 0) ---
    # Removed deliberately: progress already rewards distance covered within a
    # fixed 1800-step budget, which IS average speed, which IS lap time. The lap
    # term restated the same objective rather than adding one. Set w_lap > 0 to
    # re-enable (200 was ~10% of reward at ~10.6 s laps).
    w_lap: float = 0.0
    min_lap_time: float = 1.0
    lap_bonus_flat: float = 0.0
    # --- terminal ---
    crash_penalty: float = 15.0
    stall_penalty: float = 5.0
    # --- shaping parameters ---
    safe_dist: float = 0.5      # m; closer than this starts costing proximity
    # Backup crash detector. collision_count is authoritative, but if the sim
    # does not count wall scrapes the car can grind along a wall for a whole
    # episode. Terminating on a sustained very-close reading prevents that.
    crash_dist: float = 0.12    # m from the LiDAR
    crash_dist_steps: int = 5   # consecutive steps below crash_dist
    stall_speed: float = 0.15   # m/s
    stall_steps: int = 36       # ~2 s of no motion at 18 Hz


@dataclass
class EnvCfg:
    # MEASURED: the bridge advertises lidar_scan_rate=40 but real Bridge
    # round-trips land at ~18 Hz, so decimation=1 gives ~18 Hz control.
    decimation: int = 1
    # 4400, not 1800. This is a STEP count, so it means different amounts of
    # driving time on different machines: v3 ran 1800 steps at 7.7 Hz = 234 s,
    # but this machine runs 18.8 Hz where 1800 steps is only 96 s. 4400 restores
    # v3's ~234 s episode. TODO: derive from control_period like gamma does.
    max_episode_steps: int = 4400
    tick_timeout: float = 5.0   # s to wait for a sim tick before declaring a stall
    reset_pulse_ticks: int = 3  # hold reset_command=True this many ticks
    reset_settle_s: float = 0.6 # let the car come to rest after a reset
    startup_timeout: float = 60.0   # s to wait for the very first scan

    # Planning horizon in SECONDS, not steps. gamma is derived from this and the
    # MEASURED control period: gamma = 1 - dt/horizon_seconds.
    # Why: gamma's usual "1/(1-gamma) steps" horizon is a step count, so the same
    # gamma means different amounts of real time on different machines. v3 ran at
    # 7.7 Hz where 0.99 = 13 s; on this GPU 17.9 Hz makes 0.99 mean only 5.6 s.
    # Deriving gamma keeps the horizon fixed in time whatever rate the hardware
    # (or the competition machine) happens to deliver.
    horizon_seconds: float = 13.0

    # twist.linear is actually BODY frame (measured: velocity sits 2.9 deg off
    # the body x-axis, 93.9 deg off world yaw). The original code assumed WORLD
    # frame and projected onto the heading, which mislabelled 61% of steps as
    # reverse -- corrupting obs slot 91 and zeroing the speed reward.
    # Runs v1-v4 were all trained with the bug, so keep it ON by default to stay
    # comparable with those policies. Set False for the corrected behaviour.
    # Now FALSE: fast cornering is precise speed control, and the agent
    # cannot learn that with a speedometer that reads backwards 61% of
    # the time. Set True only to reproduce the v1-v4 runs.
    legacy_speed_sign: bool = False


@dataclass
class Cfg:
    obs: ObsCfg = field(default_factory=ObsCfg)
    act: ActionCfg = field(default_factory=ActionCfg)
    rew: RewardCfg = field(default_factory=RewardCfg)
    env: EnvCfg = field(default_factory=EnvCfg)

    def to_dict(self):
        return asdict(self)
