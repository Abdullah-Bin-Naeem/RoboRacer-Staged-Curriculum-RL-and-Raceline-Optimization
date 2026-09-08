# docs/plans

Working documents for the follower (pure pursuit + speed-profile matching), ICRA 2026 track.

| File | What it is | Read it when |
|---|---|---|
| [`CONTROLLER_TUNING_PLAN.md`](CONTROLLER_TUNING_PLAN.md) | The plan. Current gap, experiment queue, result matrices to fill, rejected experiments, measurement traps. | You are about to run or judge an experiment. |
| [`../../raceline/ab_report.py`](../../raceline/ab_report.py) | A/B report generator: lap/lateral/delay/section tables, the §8.5 gate, and a five-panel figure. | You have two run logs and need the before/after. |
| [`follower-internals.html`](follower-internals.html) | Interactive explanation of the controller: pure pursuit geometry, the lookahead chain, how `v_target` is sampled, the tire slip band. Open in a browser. | You want to understand *why* a parameter does what it does. |

**Audited 2026-09-09** against `pure_pursuit.py`, the launch files and `icra_base20_a70.csv`.
Corrections are marked **[audit 09-09]** in the plan; the headline ones are that the effective
lookahead is 1.54 m rather than the documented 2.20 m, and that 78 % of the longitudinal loss is
on accelerating stretches rather than in the braking zones.

The plan is written to stand alone: file paths, run commands, decision gates and the
already-tried list are all in it. Start there.

Published copy of the interactive page:
https://claude.ai/code/artifact/9693cf5a-a661-4030-b52e-08c6f265843e
