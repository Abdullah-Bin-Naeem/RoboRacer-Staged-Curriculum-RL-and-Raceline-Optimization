"""Stage 3 - SPEED. Raise the throttle ceiling via the curriculum.

Identical observation and reward to stage 2. The only change is that the
throttle cap is no longer fixed: it ratchets up as the agent proves it can
drive at the current one.

Ladder: +0.02 per raise (~+10% top speed), 25k-step cooldown, and a raise
requires crash rate <= 0.25, mean episode length >= 400, AND throttle demand
>= 55% of the cap. That last gate is what v3 lacked -- its curriculum fired on
competence alone and destroyed the policy at step 379,318 with a +50% jump.

Speed ~= 24 x throttle, so cap 0.20 ~= 4.8 m/s and cap 0.30 ~= 7.2 m/s.
"""
NAME = "stage3_speed"
EXPECTED_OBS_DIM = 95
RESUME_FROM = "runs/stage2_sensors/final.zip"
COMPETITION_LEGAL = True
DEFAULTS = dict(timesteps=600_000, learning_rate=2e-4, curriculum=True, warmup=10_000, crash_rate_max=0.35,
                throttle_ceiling=0.5)   # ~12 m/s at speed ~= 24 x throttle


def apply(cfg):
    from stages import stage2_sensors
    stage2_sensors.apply(cfg)           # identical observation + sensors
    # ---------------- stage-3 reward changes (stages 1-2 unaffected) --------
    # Speed was worth ~3% of reward at w_speed=0.2. Raised so it can actually
    # influence decisions, but kept well under progress so the agent still has
    # to stay alive to earn: at ~3.3 m/s over a 4400-step episode this is
    # ~440 vs progress ~4090, i.e. ~11%.
    cfg.rew.w_speed = 0.6
    # Centring REDUCED, not removed (0.3 -> 0.15). Setting it to 0 was a mistake:
    # a racing line genuinely is not the centre line, but this policy was using
    # the centring term as wall-avoidance. With it at 0, mean steps between
    # crashes fell from 14,913 (stage 2) to ~500-700 (stage 3 runs 1 and v2).
    # Halving it loosens the line without deleting the safety signal.
    cfg.rew.w_center = 0.15
    # Episode cap 2250 steps ~= 15 laps (~125 s at 8.2 s/lap, 55.6 ms/step).
    # Chosen on HEADROOM, not just pass rate. The crash gate depends on the cap
    # as well as the policy, and every cap raise degrades the policy temporarily
    # (the car must relearn corner speeds ~10% faster). Headroom = how far MTTC
    # may fall before the gate shuts:
    #     cap 1200 / gate 0.25 -> 3.6x   but only ~8 laps/episode
    #     cap 2250 / gate 0.35 -> 2.9x   15 laps            <-- chosen
    #     cap 4400 / gate 0.35 -> 1.5x   stalls after 1-3 raises
    cfg.env.max_episode_steps = 2250
    # crash_penalty stays 15.0 -- deliberately the dominant deterrent. Note the
    # real cost of a crash is forfeiting the rest of the episode's progress
    # (~230 discounted at gamma~0.996), not the -15 itself.
    return cfg
