"""Stage 5 - FRESH, from scratch. Learn to drive the competition track.

  observation  110 beams at +/-110 deg (2.0 deg/beam), wheel speed u, IMU yaw
               rate, previous action, and three race-legal slip slots: observer
               car speed v_est, longitudinal slip S, yaw-rate residual. Encoder
               speed is a 3-tick window with a counter-discontinuity guard (the
               sim's ResetManager restores the encoders on every reset).
  throttle     scale 1.0, FIXED: the full simulator range, no curriculum, so a
               learned action is never re-labelled. The peak-grip throttle is
               ~1.15 x v/25.25; beyond it the extra is wheelspin, which the
               slip slot shows and stage 6's grip term charges for. RL finds
               its own speed; the long straight is where the top of the range
               pays.
  timing       decimation 2 on the 45 Hz-capped shimmed bridge -> 22.5 Hz
               control, phase- and latency-locked (env.py). Episode and
               horizon are in seconds, derived at startup. Refuses a sim tick
               outside 15-32 ms.
  reward       progress-dominated, speed capped IN THE REWARD at v_ref 3 m/s:
               progress faster than that earns nothing, so this stage's job is a
               policy that survives and corners. Crash-ended episodes mostly
               RESUME at the simulator's own checkpoint instead of the start
               line (cfg.env.crash_restart), so the policy practises the corner
               that beat it rather than re-driving the straight it already
               knows. w_progress alone is speed
               pressure (progress/step = w x v x dt): run 1 without the cap
               sprinted 19.2 m into the first hairpin at 4-8 m/s, 100% crash for
               743k steps. A time-to-collision term gives the braking gradient
               prox (0.5 m) cannot, and the crash penalty (50) beats the
               reward-to-go at the braking point. Stage 6 lifts the cap.

Bring the bridge up with the fast loop pinned to the evaluation rate:
    ros2 launch racer_bringup bridge.launch.py tcp_nodelay:=true loop_hz_cap:=45
"""
NAME = "stage5_fresh"
EXPECTED_OBS_DIM = 117
RESUME_FROM = None
COMPETITION_LEGAL = True
# gradient_steps 2: on the GTX 1080 Ti 3 steps pushed diag/overhead_ms to 50
# against a 55 ms window and ticks_per_step to 2.4 late in run 1.
DEFAULTS = dict(timesteps=250_000, learning_rate=3e-4, warmup=0, gradient_steps=2)


def apply(cfg):
    # --- observation ---
    cfg.obs.fov_half_deg = 110.0
    cfg.obs.n_beams = 110
    cfg.obs.range_max = 10.0
    cfg.obs.v_max = 26.0
    cfg.obs.yaw_rate_max = 5.0
    # --- action ---
    cfg.act.max_steer = 1.0
    cfg.act.throttle_min = 0.0
    cfg.act.throttle_max = 1.0              # the SCALE; never raised or lowered
    # --- reward (speed capped in the reward, braking gradient, costly crash) ---
    cfg.rew.w_progress = 5.0
    cfg.rew.v_ref = 3.0                     # progress above 3 m/s earns nothing
    cfg.rew.w_speed = 0.2
    cfg.rew.w_grip = 0.0
    cfg.rew.w_center = 0.15
    cfg.rew.w_smooth = 0.3
    cfg.rew.w_prox = 0.5
    cfg.rew.w_ttc = 1.0                     # ~= one step of capped progress at full charge
    cfg.rew.ttc_ref = 1.2
    cfg.rew.ttc_sector_deg = 6.0            # ~1.2 m wide straight: 10 deg charged from 3.2 m/s
    cfg.rew.step_penalty = 0.02
    cfg.rew.w_lap = 0.0
    cfg.rew.crash_penalty = 50.0
    cfg.rew.stall_penalty = 5.0
    # --- timing, all in seconds where it matters ---
    cfg.env.decimation = 2
    # Corner first, using the SIMULATOR'S OWN respawn: a counted collision
    # already leaves the car at the last checkpoint, pointed along the track,
    # so skipping the reset pulse starts the next episode at whatever corner
    # just beat the policy. Verified working in run stage5_v4 -- 115 of the
    # first 374 episodes got past the corner that stopped all 8606 episodes of
    # run 1. 25% still start at the line, keeping the approach in the buffer.
    cfg.env.crash_restart = True
    cfg.env.crash_restart_prob = 0.75
    cfg.env.crash_restart_max = 20
    # The driven-to fixed start point (cfg.env.spawn_drive) is the alternative
    # and stays off: the sim's checkpoints already carry the track heading and
    # cost no sim time, and they follow the policy's frontier by themselves.
    cfg.env.spawn_drive = False
    cfg.env.episode_seconds = 245.0         # ~30 laps
    cfg.env.horizon_seconds = 13.0
    cfg.env.tick_ms_min = 15.0             # refuse an uncapped 13 ms loop ...
    cfg.env.tick_ms_max = 32.0             # ... and a stock 55 ms one
    return cfg
