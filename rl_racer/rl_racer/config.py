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
    # Quadratic companion to w_smooth. The linear term taxes a big reversal and
    # a small correction at the same rate per degree, so raising it to kill
    # jitter also makes the policy sluggish where speed needs quick hands; a
    # square charges a large jump hard and a small one almost nothing. Both use
    # only prev_steer, which is observation slot 113, so the reward stays
    # Markovian -- a true jerk term would need prev_prev_steer in the
    # observation and would unload every checkpoint. MEASURED at stage 6:
    # |d_steer| averaged 0.164/step (4.9 deg every 48.6 ms) against a mean lock
    # of 8.2 deg, i.e. it reversed most of its steering every step.
    w_smooth2: float = 0.0
    # Throttle smoothness, |throttle_t - throttle_{t-1}|. MEASURED on stage 7's
    # deterministic policy: |d_throttle| 0.165/step, 25% of steps at FULL brake
    # in bursts averaging 2.1 steps -- 83 bursts in 700 steps, ~6x more often
    # than a lap's corners need. Throttle 0 is brake LOCK in this sim, so each
    # burst costs ~0.47 m/s at 4.55 m/s^2, which is why the car averages 2.9
    # m/s while commanding throttle >0.3 on 27% of steps. Nothing charged for
    # it: w_smooth covers steering only.
    # LINEAR on purpose, unlike w_smooth2. The fault here is the FREQUENCY of
    # transitions, not their size, and the total variation a linear term sums
    # is exactly that. A square would charge one decisive 0.6 brake application
    # (0.36) more than six 0.1 chatters (0.06) -- backwards, because braking
    # hard once per corner is what a fast lap looks like.
    # prev_throttle is observation slot 114, so this stays Markovian.
    w_thr_smooth: float = 0.0
    # Excess longitudinal slip, max(0, |S| - veh.tire_s_peak). The ONLY term
    # with a gradient in the region the policy actually occupies: mu(|S|) is
    # FLAT at the asymptote 0.464 for every |S| >= 0.25, so mu/mu_peak is a
    # constant 0.644 there and w_grip has literally zero gradient -- which is
    # why tripling it (0.05 -> 0.15) left slip/frac_peak unmoved at 0.064.
    # MEASURED on stage 8's deterministic policy: mean |S| 0.924, with 75.6%
    # of steps past the asymptote. The car therefore spends three quarters of
    # its lap at 64% of the grip available to it -- spinning up under throttle
    # and locking under brake, never gripping. Penalising the EXCESS is
    # symmetric and physically right: peak mu is at |S| = 0.15 for braking as
    # well as for driving, so this pushes toward threshold braking and
    # threshold acceleration, i.e. the limit of the car.
    # S is observation slot 116, and even though that slot saturates at
    # obs.slip_max the policy can still derive S from slots 111 (u) and 115
    # (v_est), so the term stays observable without touching a normaliser --
    # changing one would invalidate every observation already in the buffer.
    w_slip: float = 0.0
    w_prox: float = 0.5         # proximity to a wall
    step_penalty: float = 0.02
    # --- speed cap, in the REWARD, never the action. Progress faster than
    # v_ref earns nothing, so nothing pays above it and raising it later
    # re-labels no action (an action-space cap collapsed the old lineage
    # twice). w_progress is speed pressure in disguise: progress/step =
    # w x v x dt, and stage 5 run 1 sprinted 19.2 m into the first hairpin at
    # 4-8 m/s for 13 h on it. 0 = uncapped.
    v_ref: float = 0.0
    # --- braking gradient: time-to-collision over the forward +/-ttc_sector_deg
    # of the scan, per beam r_i / (v cos th_i), min. prox fires only inside
    # safe_dist (0.5 m = 1-2 control steps at 4-8 m/s), after braking is
    # possible. Charges w_ttc x max(0, 1 - ttc/ttc_ref). At 3 m/s it wakes
    # 3.6 m before a wall ahead and is silent at the raceline's 1.5 m/s hairpin
    # speed with 1.8 m to go; a wall 1 m to the side is not "ahead" below 5 m/s.
    # Full charge = w_ttc per step, against ~0.8/step of progress at v_ref 3.
    # Speed from /odom (training only). 0 = off.
    # Sector: MEASURED, the opening straight is ~1.2 m wide (pre-crash car
    # centre x spanned 0.27-1.23 about the 0.80 centreline), so the side walls
    # sit ~0.55 m off the sensor. At +/-10 deg their beams are 3.2 m long and
    # the term charged from 3.2 m/s on a straight road; at +/-6 deg (5.3 m)
    # it starts at ~4.4 m/s, above stage 5's v_ref and below racing speed.
    w_ttc: float = 0.0
    ttc_ref: float = 1.2
    ttc_sector_deg: float = 6.0
    # --- lap bonus: w_lap / lap_time per completed lap (sim's own timer) ---
    # The one term aimed at lap TIME rather than distance. 200 ~= 10% of reward
    # at ~10 s laps. 0 = off.
    w_lap: float = 0.0
    min_lap_time: float = 1.0
    lap_bonus_flat: float = 0.0
    # --- terminal ---
    # Forfeiting the rest of the episode (~230 discounted) only deters a
    # critic that has SEEN long episodes; a fresh policy never has, so the
    # penalty itself must beat the reward-to-go at the braking point (~20 at
    # 4 m/s and 1.1/step). 15 made sprint-crash-repeat the optimum for 743k
    # steps; 50 leaves margin without making creeping (stall -5) the better
    # deal.
    crash_penalty: float = 50.0
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
    # Fixed start pose. The simulator has NO spawn command -- the bridge sends
    # only throttle, steering and reset, and reset means "the start line" with
    # no argument (EXPERIMENTS.md 25) -- so a chosen start pose has to be
    # DRIVEN to: after the reset pulse the env steers dead straight to within
    # spawn_radius of (spawn_x, spawn_y), brakes to a standstill, and only then
    # hands over. Those ticks are not policy steps and not recorded
    # transitions, and the car is stationary at handover, so the observation
    # the policy sees is exactly the one a normal reset produces. At deployment
    # (race_mode) reset does nothing, so none of this exists there.
    spawn_drive: bool = False
    spawn_x: float = 0.80
    spawn_y: float = -13.31           # ~2.3 m before the corner at y = -15.6
    spawn_radius: float = 0.35        # >= one step of travel at spawn_cruise
    spawn_cruise: float = 3.0         # m/s on the way in
    spawn_timeout_s: float = 15.0     # give up and start from the line instead
    # Reverse curriculum, using the simulator's own checkpoints. A collision
    # already leaves the car at the last checkpoint (EXPERIMENTS.md 24), so
    # simply NOT pulsing reset starts the next episode there -- at whatever
    # corner the policy is currently failing, rather than back at the start
    # line. There is no spawn API to do this directly: the bridge sends only
    # throttle, steering and reset. Checkpoints are ~3 m apart and carry the
    # track's heading, so the car resumes pointed along the racing line.
    # The crash still TERMINATES, so the -crash_penalty target keeps its
    # no-bootstrap form; only the next episode's start state moves.
    crash_restart: bool = False
    # Fraction of crash-ended episodes that resume at the checkpoint. The rest
    # go back to the start line, so the approach to the corner -- and the
    # braking that belongs to it -- stays in the buffer. 0.75 gives ~4
    # attempts at the frontier per full lap attempt.
    crash_restart_prob: float = 0.75
    # Force a full reset after this many consecutive checkpoint restarts, so a
    # checkpoint the policy cannot leave cannot trap the run.
    crash_restart_max: int = 20
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
