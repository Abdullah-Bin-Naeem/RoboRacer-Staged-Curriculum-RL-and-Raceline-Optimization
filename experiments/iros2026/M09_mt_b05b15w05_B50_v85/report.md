# M09 — mt_b05b15w05_B50_v85

- **date** 2026-09-18  **line** `rl_mt_b05b15w05_ell_L65_B50_v8.5.csv` (profile 8.833 s)  **loop cap** 45  **laps asked** 30
- **overrides** `distance_source:=slip control_hz:=45 warmup_v_max:=2.0 warmup_dist_m:=21.0 amcl_params_file:=amcl_beams360.yaml exit_guard_from:=0.0 exit_guard_full:=0.0 controller_mode:=hybrid_lqr lqr_k_lat:=0.03 lqr_k_head:=0.05 lqr_k_yaw:=0.0 lqr_max_correction_rad:=0.02 recover_settle_s:=2.0 recover_creep_s:=3.0 v_max:=8.5 target_lead_s:=0.08 brake_cap_margin:=0.2 enc_rate_window_s:=0.05`
- **hypothesis** Same geometry as M08, v_max 8.5 + target_lead 0.08 + brake_cap 0.2 give hairpin-1 the delay/observer margin that M08 lacked.
- **prediction** ~9.00–9.05 if clean; sim was already ~70 min old at launch.

## Result

Session ran until the weekly-limit cut, then sat crashed in recovery (~22 min wall, last t ≈ 1334 s). Sim age at teardown ~90 min.

First contact: **t = 21.95 s, s = 19.67 m (hairpin-1 EXIT)**. v ≈ 2.88 vs target 3.36, e_lat −0.64, along ≈ 0, steer 0.54 — understeer at *target-ish* speed, not the M08 late-brake drive.

s-wrap lap deltas after warmup: **9.15, 9.20, 9.35, 9.20, 9.20, 9.15, 9.20, 9.10**, then a 40 s recovery, then **9.00, 9.25, 9.20, 9.15, 9.15, 9.15**, another recovery, **9.25, 9.30**. Clean cluster ≈ **9.10–9.35**, one wrap at 9.00. Global v_max 8.5 ate the M06/M08 sub-9.

Later stall at s 36.7 (C3) with v_est still 3.8 after a wall hit; stack never recovered, dist frozen 1109.3 m.

## Verdict

**Rejected as a 30-lap sub-9 candidate.** Hairpin-1 still has zero lateral margin at a_lat 7.0; dropping global v_max moved the mean above 9.1 without fixing the exit. `brake_cap_margin` / `target_lead_s` were not given a fair test (hot sim, first-lap contact). Next: keep the M08 v_max-9.0 profile (the only sub-9 line), keep the two follower margins, **fresh simulator**, 30 laps. Hairpin a_lat 6.5 line is generated as fallback (`rl_mt_b05b15w05_ell_L65_B50_hp65_v9.0.csv`, paper 8.893 s, apex 2.31 vs 2.40) if hp1 still contacts on a cold sim.
