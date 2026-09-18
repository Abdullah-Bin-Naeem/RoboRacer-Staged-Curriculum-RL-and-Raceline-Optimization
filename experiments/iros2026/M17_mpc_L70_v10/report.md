# M17 — M16 line + kinematic MPC  **REJECTED**

- **line** `rl_mt_b05b15w05_ell_L70_B55_v10.csv`  `controller_mode:=kinematic_mpc`
- **result** Out-lap cascade. First RESET ~10 s in, still under the **2 m/s warmup**, at hp1 (`+3.02, -13.62`). Then recovery walls. No clean timed lap. Stopped by teardown.
- **verdict** Delay-shoot MPC is not race-ready on this car/loop. Leave the code opt-in; do not use it for IROS. Next: keep M16 hybrid LQR, only soften hairpin lateral in the profile.
