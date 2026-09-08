# docs/plans

Working documents for the follower (pure pursuit + speed-profile matching), ICRA 2026 track.

| File | What it is | Read it when |
|---|---|---|
| [`CONTROLLER_TUNING_PLAN.md`](CONTROLLER_TUNING_PLAN.md) | The plan. Current gap, experiment queue, result matrices to fill, rejected experiments, measurement traps. | You are about to run or judge an experiment. |
| [`follower-internals.html`](follower-internals.html) | Interactive explanation of the controller: pure pursuit geometry, the lookahead chain, how `v_target` is sampled, the tire slip band. Open in a browser. | You want to understand *why* a parameter does what it does. |

The plan is written to stand alone: file paths, run commands, decision gates and the
already-tried list are all in it. Start there.

Published copy of the interactive page:
https://claude.ai/code/artifact/9693cf5a-a661-4030-b52e-08c6f265843e
