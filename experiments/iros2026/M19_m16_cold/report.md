# M19 — first timed lap (user: 2nd lap) hp1 exit

Warmup finished clean (58.68 s). Crash **~4 s into lap 2**, first race-speed hairpin 1.

## Where

s **21.3** (hp1 **exit**, not the apex). Pose `(+3.02, −13.62)` is the recovery freeze, a bit after the wall.

## Sequence (from `logs/mt_m19.csv`)

1. Straight at 9.89 m/s. Along-track **+0.28 m mean** (ahead of truth). Cross 2 cm. Loc is not wild.
2. Brake zone: still **+0.34 m along**. M16’s first flying lap had already flipped to **−0.03 m** here — that is the difference.
3. Apex: `e_lat` **+0.22 m inside**, steer **0.80–0.81**, heading error **−0.45 rad**.
4. Exit: loc at the car is fine (along +0.13, cross 3 cm). Throttle **0.10 → 0.19**, IMU **+5.7 m/s²**, `e_lat` **−0.27 m wide**, steer falling 0.46 → 0.04. Then the wall.

Same shape as M16 lap 22, **without** that lap’s +0.75 m loc spike. M19 died on **+0.4 m ahead in the brake + exit throttle while 25 cm out**.

## Verdict

Not a dirty sim. Not MPC. First full-speed hp1 with the estimate still sitting ahead on the 16 m straight, early turn-in, inside, then wide under `accel_ff`. M16’s 8.85 stint survived this corner when along went **negative** in the brake; that did not happen here.
