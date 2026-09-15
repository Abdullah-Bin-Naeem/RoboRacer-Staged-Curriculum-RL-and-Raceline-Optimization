# Stages

Both stages emit the same 117-dim observation, so stage 6 resumes stage 5's
weights and replay buffer. `apply(cfg)` composes: stage 6 calls stage 5's.

| stage | job | reward change | resume |
|---|---|---|---|
| 5 `stage5_fresh` | learn to drive, from scratch | — (`w_speed` 0.2) | — |
| 6 `stage6_push` | speed pressure | `w_speed` 0.6, lap bonus 200/lap_time, grip 0.05 | `runs/stage5_fresh/final.zip` (+ buffer) |

```bash
ros2 launch racer_bringup bridge.launch.py tcp_nodelay:=true loop_hz_cap:=45   # system python
./run_train.sh --stage 5
python enjoy.py runs/stage5_fresh/final.zip --episodes 3
./run_train.sh --stage 6 --resume runs/stage5_fresh/final.zip
```

Banner to confirm: `measured control period ~44 ms (22.5 Hz) = sim tick ~22 ms
x decimation 2`, `max_episode_steps ≈ 5500 (245 s)`, `obs_dim=117`,
`gradient_steps=3`. `time/fps` ≈ 20 once episodes lengthen.

Gate before stage 6: zero crashes over 3 deterministic episodes and a sane lap
time from the simulator's timer. Stage 6 loads the buffer (its rewards are
under stage 5's weights, a bias that ages out; an empty buffer is the
collapse mode). If alpha climbs on resume, `--ent-coef <value>` freezes it.
