"""All tunables for SAC training on AutoDRIVE RoboRacer, in one place.

Reward weights are the thing you will actually iterate on -- everything else
should be left alone until the reward is producing sane behaviour. Every
constant carries the measurement that justifies it; read the comment before
changing a number.

Base defaults reproduce the v3 / stage 1-3 lineage (95-dim observation) so old
checkpoints still load. The fresh lineage (stages 5-6) overrides them.
"""
from dataclasses import dataclass, asdict, field


@dataclass
class VehicleCfg:
    """Simulator constants from its public Unity source (raceline/VEHICLE_MODEL.md).

    Used only by the race-legal speed observer and the slip features. The
    evaluation machine runs the same simulator build, so they transfer.
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
    #   +/-90  -> [180:900]  90 beams   (v3 lineage; exit of a >=180 deg
    #                                    hairpin sits ON this boundary)
    #   +/-110 -> [100:980] 110 beams   (fresh lineage; 20 deg of margin)
    fov_half_deg: float = 90.0
    n_beams: int = 90
    range_max: float = 10.0    # matches bridge range_max, used to normalise
    # Normalisers only -- never limits. Too SMALL clips and blinds the agent
    # above the value; too large only shrinks the numbers. Top speed is 22.88.
    v_max: float = 20.0
    yaw_rate_max: float = 5.0  # rad/s
    # COMPETITION LEGALITY: /odom and /ips are restricted at race time. Speed
    # comes from the wheel encoders and yaw rate from the IMU. /odom feeds only
    # the reward, which does not exist at inference.
    use_encoder_speed: bool = True
    # MEASURED, two independent methods agree (0.05815 path-integration,
    # 0.05813 steady-state); the guide's 0.0590 leaves a 1.5% slip bias.
    wheel_radius: float = 0.0581
    # Encoder rate window in ticks and the counter-discontinuity guard (rad);
    # see sensors.WheelSpeed.
    enc_window: int = 3
    enc_step_max: float = 300.0
    # Three extra race-legal slots: observer car speed v_est, longitudinal slip
    # S, and the yaw-rate residual (understeer/oversteer). Off in the v3
    # lineage (95-dim), on in the fresh lineage (118-dim).
    slip_slots: bool = False
    slip_max: float = 0.5      # S normaliser; peak grip is 0.15, asymptote 0.25
    yaw_res_max: float = 8.0   # rad/s; full lock at 5 m/s predicts ~8.9

    @property
    def dim(self) -> int:
        # beams + [speed, yaw_rate, prev_steer, prev_throttle, throttle_cap]
        #       + [v_est, S, yaw_residual] when slip_slots
        return self.n_beams + 5 + (3 if self.slip_slots else 0)


@dataclass
class ActionCfg:
    # Full mechanical steering range; steering command +/-1 = +/-30 deg.
    max_steer: float = 1.0
    throttle_min: float = 0.0   # no reverse; throttle 0 = brake lock in this sim
    # throttle_max is the action SCALE, not a speed limit: throttle = (a+1)/2 *
    # throttle_max, so the network's outputs are numbers relative to it.
    #   v3 lineage: 0.20, raised by a curriculum (each raise re-labels every
    #               learned action; see EXPERIMENTS.md).
    #   fresh lineage: 0.5, FIXED. Throttle commands wheel speed (25.25 m/s per
    #               unit); peak-grip acceleration at 8 m/s needs 0.36, and above
    #               ~0.40 extra throttle is wheelspin at the flat asymptote, so
    #               0.5 already covers everything the track can use while
    #               keeping the network's range on throttles that do something.
    throttle_max: float = 0.20


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
    w_center: float = 0.3       # lateral asymmetry, 0 = perfectly centred
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
    # Sim ticks per control step. The loop is ~18 Hz stock and 40-50 Hz on the
    # evaluation box (77-85 here with the nodelay shim; loop_hz_cap:=45 pins
    # it). decimation=2 at 45 Hz -> 22.5 Hz control, and 3 GPU gradient steps
    # (~30 ms) fit inside the 44 ms budget.
    decimation: int = 1
    # Episode length. Prefer episode_seconds: a STEP count means different
    # driving time at different control rates (1800 steps = 234 s at 7.7 Hz but
    # 96 s at 18 Hz), which silently changed the task twice in this project.
    # When episode_seconds > 0 the env derives max_episode_steps from the
    # measured control period at startup; 0 keeps max_episode_steps as given.
    episode_seconds: float = 0.0
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
    # this window, ms; 0 = no check. The fresh lineage sets 15-32: a stock
    # bridge ticks at ~55 ms (decimation 2 -> 9 Hz control, the v3 trap) and an
    # uncapped shimmed bridge at ~13 ms (3 gradient steps overrun a 26 ms
    # budget). Both fail silently otherwise.
    tick_ms_min: float = 0.0
    tick_ms_max: float = 0.0
    # Planning horizon in SECONDS: gamma = 1 - dt/horizon_seconds from the
    # measured control period, so 0.99 at 7.7 Hz and 0.9957 at 18 Hz are the
    # same horizon. Hard-coding gamma halved the horizon when the loop sped up.
    horizon_seconds: float = 13.0
    # twist.linear is BODY frame; the original world-frame projection labelled
    # 61% of steps as reverse. True reproduces v1-v4 only; encoder speed is
    # naturally signed and never uses this.
    legacy_speed_sign: bool = False


@dataclass
class Cfg:
    obs: ObsCfg = field(default_factory=ObsCfg)
    act: ActionCfg = field(default_factory=ActionCfg)
    rew: RewardCfg = field(default_factory=RewardCfg)
    env: EnvCfg = field(default_factory=EnvCfg)
    veh: VehicleCfg = field(default_factory=VehicleCfg)

    def to_dict(self):
        return asdict(self)
