# M10 — M08 line + lead + brake_cap, cold sim

- **date** 2026-09-18  **line** `rl_mt_b05b15w05_ell_L65_B50_v9.0.csv` (profile 8.801 s)
- **overrides** M08 + `v_max:=9.0 target_lead_s:=0.08 brake_cap_margin:=0.2`  **fresh sim**
- **hypothesis** Cold sim + follower margins hold hp1 without dropping v_max.
- **prediction** clean 8.95–9.05

## Result

Timed laps before contact: **9.06, 9.08, 9.13, 9.26, 9.17**. First contact **t = 66.80 s, s = 18.27 (hp1 turn-in/apex)**. RESET ~3 s after lap 6.

At impact: v 2.35 vs tgt 2.39, e_lat +0.33 (opening inside), along-track loc **+0.54…+0.73 m**, delay 0.098 s. Entry overspeed was small (~0.2–0.6 m/s); this is early turn-in from along-track error, not the M08 brake-floor drive. `target_lead_s 0.08` cost ~0.1 s vs M08’s 8.95 and did not buy the corner.

## Verdict

**Rejected.** Margins on a cold sim still not sub-9 and still not 30-lap. Next: hairpin a_lat 6.5 line (`rl_mt_b05b15w05_ell_L65_B50_hp65_v9.0.csv`, paper 8.893, apex 2.31), `brake_cap_margin:=0.2`, **`target_lead_s:=0.0`**.
