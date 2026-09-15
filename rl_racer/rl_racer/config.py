"""All tunables for SAC training on AutoDRIVE RoboRacer, in one place.

Reward weights are the thing you will actually iterate on -- everything else
should be left alone until the reward is producing sane behaviour. Every
constant carries the measurement that justifies it; read the comment before
changing a number. Stage modules (stages/) set the values they own explicitly
on top of these defaults.
"""
from dataclasses import dataclass, asdict, field


@dataclass
class VehicleCfg:
    """Simulator constants from its public Unity source (tools/VEHICLE_MODEL
    notes in EXPERIMENTS.md). Used by the race-legal speed observer and the
    slip features. The evaluation machine runs the same simulator build.
    """
    u_per_throttle: float = 25.25   # wheel surface speed per unit throttle, m/s
    tire_s_peak: float = 0.15       # forward friction: extremum slip
    tire_mu_peak: float = 0.72
    tire_s_asym: float = 0.25       # asymptote slip
    tire_mu_asym: float = 0.464
    tire_rise_slope: float = 3.0    # Hermite tangent at zero slip
    drag: float = 0.273             # Rigidbody linear drag, 1/s (a = -0.273 v)
    slip_den: float = 4.0           # PhysX minLongSlipDenominator, m/s
    wheelbase: float = 0.324
    max_steer_rad: float = 0.5236   # +/-30 deg at steering command +/-1
    g: float = 9.81


@dataclass
class ObsCfg:
    # LiDAR crop. The scan is 1081 beams over +/-135 deg; indices are derived
    # from the live header. Every 10 deg of half-FOV is exactly 40 raw beams,
    # so with n_beams = fov_half_deg the pool stays 8 raw beams -> 1 at 2.0 deg.
    # +/-110 -> raw [100:980] = 880 beams -> 110. The exit corridor of a
    # >=180 deg hairpin sat ON the old +/-90 boundary; this keeps 20 deg spare.
    fov_half_deg: float = 110.0
    n_beams: int = 110
    range_max: float = 10.0    # matches bridge range_max, used to normalise
    # Normalisers only -- never limits. Too SMALL clips and blinds the agent
    # above the value; too large only shrinks the numbers. Wheel speed reaches
    # 25.25 m/s at full throttle, the car 22.9 (drag-limited).
    v_max: float = 26.0
    yaw_rate_max: float = 5.0  # rad/s
    # MEASURED, two independent methods agree (0.05815 path-integration,
    # 0.05813 steady-state); the guide's 0.0590 leaves a 1.5% slip bias.
    wheel_radius: float = 0.0581
    # Encoder rate window in ticks and the counter-discontinuity guard (rad);
    # see sensors.WheelSpeed.
    enc_window: int = 3
    enc_step_max: float = 300.0
    # Slip-slot normalisers: observer car speed v_est shares v_max; S peaks at
    # 0.15 and hits the asymptote at 0.25; full lock at 5 m/s predicts ~8.9.
    slip_max: float = 0.5
    yaw_res_max: float = 8.0

    @property
    def dim(self) -> int:
        # beams + [u, yaw_rate, prev_steer, prev_throttle, v_est, S, yaw_residual]
        return self.n_beams + 7


@dataclass
class ActionCfg:
    # Full mechanical steering range; steering command +/-1 = +/-30 deg.
    max_steer: float = 1.0
    throttle_min: float = 0.0   # no reverse; throttle 0 = brake lock in this sim
    # throttle_max is the action SCALE: throttle = (a+1)/2 * throttle_max. 1.0
    # gives the network the simulator's full range, FIXED -- no curriculum, so
    # a learned action never gets re-labelled. Throttle commands a wheel speed
    # (25.25 m/s per unit); the peak-grip throttle is ~1.15 * v / 25.25, so
    # above it the extra is wheelspin at the tire's flat asymptote. On a long
    # straight the car gets fast enough (~18-20 m/s -> 0.8-0.9) to use most of
    # the range; in corners the slip slot and stage 6's grip term are what let
    # the policy learn where the useful throttle ends.
    throttle_max: float = 1.0


@dataclass
class RewardCfg:
    # --- what we pay for ---
    w_progress: float = 5.0     # per metre of forward progress (dominant term)
    w_speed: float = 0.2        # per unit of v/v_max, from /odom (training only)
    # Grip utilisation: mu(|S|)/mu_peak per step. Peaks at slip 0.15 and falls
    # past it, so it pays for being AT the tire's limit -- accelerating or
    # braking -- and not for wheelspin. Shaping, so small. 0 = off.
    w_grip: float = 0.0
    # --- what we charge for ---
    # Centring is wall-avoidance in disguise: at 0 the crash rate rose 20x in
    # the earlier lineage. 0.15 loosens the line without deleting the signal.
    w_center: float = 0.15      # lateral asymmetry, 0 = perfectly centred
    w_smooth: float = 0.3       # |steer_t - steer_{t-1}|
    w_prox: float = 0.5         # proximity to a wall
    step_penalty: float = 0.02
    # --- lap bonus: w_lap / lap_time per completed lap (sim's own timer) ---
    # The one term aimed at lap TIME rather than distance. 200 ~= 10% of reward
    # at ~10 s laps. 0 = off.
    w_lap: float = 0.0
    min_lap_time: float = 1.0
    lap_bonus_flat: float = 0.0
    # --- terminal ---
    # The real cost of a crash is forfeiting the rest of the episode's
    # progress (~230 discounted at gamma ~0.996); -15 is the deterrent on top.
    crash_penalty: float = 15.0
    stall_penalty: float = 5.0
    # --- shaping parameters ---
    safe_dist: float = 0.5
    crash_dist: float = 0.12    # backup scrape detector, m from the LiDAR
    crash_dist_steps: int = 5
    stall_speed: float = 0.15   # m/s
    stall_steps: int = 36


@dataclass
class EnvCfg:
    # Sim ticks per control step. The loop is ~18 Hz with the stock bridge and
    # 40-50 Hz on the evaluation box (77-85 here with the nodelay shim;
    # loop_hz_cap:=45 pins it). decimation=2 at 45 Hz -> 22.5 Hz control.
    decimation: int = 2
    # Episode length in SECONDS: a step count means different driving time at
    # different control rates (1800 steps = 234 s at 7.7 Hz but 96 s at 18 Hz),
    # which silently changed the task twice in the earlier lineage. The env
    # derives max_episode_steps from the measured period at startup; 0 keeps
    # max_episode_steps as given.
    episode_seconds: float = 245.0        # ~30 laps
    max_episode_steps: int = 4400
    tick_timeout: float = 5.0
    # Deployment switch (enjoy.py --race). True = reset() sends NO reset pulse
    # (which would teleport the car to spawn), a collision or stall does NOT
    # end the episode (the sim keeps driving; the race is +10 s, not over),
    # and there is no step cap. Only a dead sim still ends it. Never for
    # training.
    race_mode: bool = False
    reset_pulse_ticks: int = 3
    reset_settle_s: float = 0.6
    startup_timeout: float = 60.0
    # Refuse to train if the measured SIM TICK (before decimation) is outside
    # this window, ms; 0 = no check. A stock bridge ticks at ~55 ms
    # (decimation 2 -> 9 Hz control) and an uncapped shimmed bridge at ~13 ms;
    # both would silently train at the wrong rate.
    tick_ms_min: float = 15.0
    tick_ms_max: float = 32.0
    # Planning horizon in SECONDS: gamma = 1 - dt/horizon_seconds from the
    # measured control period, so the horizon is the same on every machine.
    # Hard-coding gamma halved the horizon when the loop sped up.
    horizon_seconds: float = 13.0


@dataclass
class Cfg:
    obs: ObsCfg = field(default_factory=ObsCfg)
    act: ActionCfg = field(default_factory=ActionCfg)
    rew: RewardCfg = field(default_factory=RewardCfg)
    env: EnvCfg = field(default_factory=EnvCfg)
    veh: VehicleCfg = field(default_factory=VehicleCfg)

    def to_dict(self):
        return asdict(self)
