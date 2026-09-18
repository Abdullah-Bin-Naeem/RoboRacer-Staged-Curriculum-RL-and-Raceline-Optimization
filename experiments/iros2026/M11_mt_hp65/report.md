# M11 — hairpin a_lat 6.5, no target_lead

- **date** 2026-09-18  **line** `rl_mt_b05b15w05_ell_L65_B50_hp65_v9.0.csv` (paper 8.893 s)
- **overrides** `v_max:=9.0 target_lead_s:=0.0 brake_cap_margin:=0.2`  **exit_guard off**
- **hypothesis** Slower hairpin apex (2.31 vs 2.40) holds hp1 that M10 cut inside.

## Result

Timed before first contact: **9.18, 9.14**. First stall **t = 36.05 s, s = 43.10 (C4 / hairpin-2 EXIT)**. Loc along ~0, e_lat −0.36 (outside), v 2.99 vs tgt 3.52 — **accelerating wide**, not late-brake, not loc. Then 9 recoveries in ~20 s; each “follower resumes” re-hit within ~1 s (cascade, not a new racing defect).

Clean hp1 along-track was fine (±0.16 m). hp65 did not address this exit. Pace already 9.14+, so this line cannot deliver 30 sub-9 even if clean.

## Verdict

**Rejected.** Root cause of the *first* hit: exit throttle while 10–36 cm outside at C4 (exit_guard was 0). Root cause of the *loop*: recovery hands the follower full `v_max` on the wall. Next: M08 v9.0 line + `exit_guard` 0.08/0.20 + post-recovery 2 m/s for 8 m.
