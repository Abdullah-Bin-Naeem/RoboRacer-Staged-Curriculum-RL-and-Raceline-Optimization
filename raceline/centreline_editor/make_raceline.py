#!/usr/bin/env python3
"""map.yaml -> raceline CSV, one command.

  python3 make_raceline.py track_clean.yaml [-o raceline_new.csv] [--old raceline_a7.0.csv] [--plot]

Pipeline
  1. skeleton -> medial ridge (distance-transform projection, no pixel staircase)
  2. TUM spline_approximation smoothing + normals-crossing check   (tph)
  3. TUM iterative minimum-curvature QP with kappa_bound            (tph.iqp_handler)
  4. TUM forward/backward velocity profile with ggv + drag table    (tph.calc_vel_profile)
  5. widths ray-cast from the FINAL line, export in roboracer_stack format:
        s_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m,v_mps    (psi = atan2, +kappa = left)

Every number that describes the car lives in PARAMS below. Change them there, nowhere else.
"""
import os, sys, argparse, numpy as np
from scipy import ndimage
from scipy.signal import savgol_filter, find_peaks
import trajectory_planning_helpers as tph
from auto_centreline import load, skeleton_loop, resample, sg, frame

PARAMS = dict(
    ds            = 0.10,     # m   resample / optimisation step
    open_px       = 21,       # px  morphological opening (removes wall notches < 52 cm at 2.5 cm/px)
    ref_sigma     = 0.60,     # m   Gaussian smoothing of the medial axis (kills pixel staircase, keeps hairpins)
    outer_iters   = 3,        # re-reference the QP on its own output (fixes hairpins / normal crossing)
    kappa_bound   = 1.5,      # 1/m steering limit is 1.78 (0.56 m radius); 15 % margin
    veh_width     = 0.27,     # m   car
    margin        = 0.20,     # m   per side: localisation + tracking error (measure it!)
    iters_min     = 3,
    curv_err      = 0.05,     # 1/m linearisation tolerance for the iterative QP
    m_veh         = 3.906,    # kg
    v_max         = 8.0,      # m/s
    ay_max        = 7.0,      # m/s² lateral (7.0 has 500 laps behind it; 5.5 = safe line)
    ax_brake      = 4.5,      # m/s² braking limit (tyre cap 7.06; slip band protects encoders)
    ax_drive      = 4.5,      # m/s² accel limit at v=0 ...
    drag_lin      = 0.273,    # 1/s  ... minus Unity Rigidbody.drag * v  (from the sim source)
    dyn_model_exp = 2.0,      # friction ellipse; raise to ~8 if the circle test says "box"
    filt_window   = 9,
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_yaml'); ap.add_argument('-o', '--out'); ap.add_argument('--old', help='previous raceline CSV to overlay')
    ap.add_argument('--plot', action='store_true'); ap.add_argument('--ay', type=float); ap.add_argument('--margin', type=float)
    ap.add_argument('--spawn', type=float, nargs=3, metavar=('X', 'Y', 'YAW'), default=(0.802, 3.158, -1.5708),
                    help='spawn pose (m, m, rad) — the line is oriented so it runs in the spawn heading')
    a = ap.parse_args()
    P = dict(PARAMS)
    if a.ay: P['ay_max'] = a.ay
    if a.margin is not None: P['margin'] = a.margin
    ds = P['ds']

    # ---------- map ----------
    m, img = load(a.map_yaml); res = float(m['resolution']); ox, oy = map(float, m['origin'][:2]); H, W = img.shape
    free = img >= 250
    opened = ndimage.binary_opening(free, structure=np.ones((P['open_px'], P['open_px']), bool))
    D = ndimage.distance_transform_edt(opened) * res
    def Dat(x, y):
        c = (x - ox) / res - 0.5; r = (H - 1) - ((y - oy) / res - 0.5)
        return ndimage.map_coordinates(D, [[r], [c]], order=1, mode='nearest')[0]
    def isfree(x, y):
        i = int((x - ox) / res); j = int((y - oy) / res); r = H - 1 - j
        return 0 <= i < W and 0 <= r < H and opened[r, i]
    def ray(x, y, nx, ny, maxd=3.0, st=0.01):
        d = 0.0
        while d < maxd:
            d += st
            if not isfree(x + nx * d, y + ny * d): return d - st
        return maxd
    def widths(C, psi):
        wl = np.array([ray(x, y, -np.sin(p), np.cos(p)) for (x, y), p in zip(C, psi)])
        wr = np.array([ray(x, y, np.sin(p), -np.cos(p)) for (x, y), p in zip(C, psi)])
        return wl, wr

    # ---------- 1. medial ridge ----------
    C, L = resample(skeleton_loop(opened, res, ox, oy), ds)
    for _ in range(4):
        k, psi = frame(C, ds); out = C.copy()
        for i, ((x, y), p) in enumerate(zip(C, psi)):
            nx, ny = -np.sin(p), np.cos(p); ts = np.arange(-0.6, 0.6001, 0.01)
            vals = np.array([Dat(x + nx * t, y + ny * t) for t in ts]); t = ts[np.argmax(vals)]; out[i] = [x + nx * t, y + ny * t]
        C = sg(out, 13); C, L = resample(C, ds)
    k, psi = frame(C, ds); wl, wr = widths(C, psi)
    print(f'[1] medial axis: {L:.2f} m, max|k| {np.abs(k).max():.2f}')
    # direction: the skeleton walk picks an arbitrary sense; make the line run the way the car spawns
    sx, sy, syaw = a.spawn
    i0 = int(np.argmin(np.hypot(C[:, 0] - sx, C[:, 1] - sy)))
    dot = np.cos(psi[i0] - syaw)
    if dot < 0:
        C = C[::-1].copy(); k, psi = frame(C, ds); wl, wr = wr, wl
    print(f'    direction vs spawn yaw {np.degrees(syaw):+.0f} deg: cos = {dot:+.2f} -> {"REVERSED" if dot < 0 else "ok"}; '
          f'line passes {np.hypot(C[i0,0]-sx, C[i0,1]-sy):.2f} m from spawn')
    # seam: put s=0 in the middle of the longest straight (never at a hairpin)
    kw, _ = frame(sg(C, 41), ds); small = np.abs(kw) < 0.12; best = (0, 0); i = 0; nC = len(C)
    while i < nC:
        if small[i]:
            j = i
            while j < nC and small[j]: j += 1
            if j - i > best[1] - best[0]: best = (i, j)
            i = j
        else: i += 1
    C = np.roll(C, -((best[0] + best[1]) // 2), axis=0)

    # ---------- 2. reference: Gaussian-smoothed medial axis, widths capped so Frenet normals cannot cross ----------
    def gauss_closed(Pn, sigma_m):
        return np.c_[ndimage.gaussian_filter1d(Pn[:, 0], sigma_m / ds, mode='wrap'), ndimage.gaussian_filter1d(Pn[:, 1], sigma_m / ds, mode='wrap')]
    def push_inside(Pn, clear_min):
        # any point closer than clear_min to a wall is moved along the distance-transform gradient until it isn't
        Pn = Pn.copy()
        for i, (x, y) in enumerate(Pn):
            for _ in range(60):
                d = Dat(x, y)
                if d >= clear_min: break
                gx = (Dat(x + 0.01, y) - Dat(x - 0.01, y)) / 0.02; gy = (Dat(x, y + 0.01) - Dat(x, y - 0.01)) / 0.02
                g = np.hypot(gx, gy) + 1e-9; x += 0.01 * gx / g; y += 0.01 * gy / g
            Pn[i] = [x, y]
        return Pn
    def build_ref(Pn, cap=True):
        Pn, _ = resample(Pn, ds)
        cx, cy, Mm, nv = tph.calc_splines.calc_splines(path=np.vstack((Pn, Pn[0])))
        ps, kp, dk = tph.calc_head_curv_an.calc_head_curv_an(cx, cy, np.arange(len(Pn)), np.zeros(len(Pn)), calc_dcurv=True)
        sl = tph.calc_spline_lengths.calc_spline_lengths(cx, cy)
        p_std = np.arctan2(np.gradient(Pn[:, 1]), np.gradient(Pn[:, 0]))
        wl_, wr_ = widths(Pn, p_std)
        # guarantee: on the near-wall side use the nearest-wall distance (EDT), not the ray-cast along the normal.
        # A point moved by d along the normal loses at most d of nearest-wall clearance, so the QP's w_veh/2 then
        # really is a clearance, not just a normal distance (matters on the inside of corners).
        for i, ((x, y), p) in enumerate(zip(Pn, p_std)):
            c = Dat(x, y); nx, ny = -np.sin(p), np.cos(p)
            if Dat(x + 0.05 * nx, y + 0.05 * ny) < Dat(x - 0.05 * nx, y - 0.05 * ny): wl_[i] = min(wl_[i], c)
            else: wr_[i] = min(wr_[i], c)
        if cap:   # inside width <= 0.9/|kappa| (never below the car+margin) so the normals stay well-posed
            lim = np.maximum(0.9 / np.maximum(np.abs(kp), 1e-6), w_veh / 2 + 0.06)
            wl_ = np.where(kp > 0, np.minimum(wl_, lim), wl_); wr_ = np.where(kp < 0, np.minimum(wr_, lim), wr_)
        rt = np.c_[Pn, wr_, wl_]
        return rt, nv, Mm, sl, ps, kp, dk
    w_veh = P['veh_width'] + 2 * P['margin']
    refline = push_inside(gauss_closed(C, P['ref_sigma']), w_veh / 2 + 0.03)
    reftrack, normvec, M, spline_len, psi_r, kappa_r, dkappa_r = build_ref(refline)
    crossing = tph.check_normals_crossing.check_normals_crossing(track=reftrack, normvec_normalized=normvec, horizon=10)
    print(f'[2] reference: sigma {P["ref_sigma"]:.2f} m, max|k| {np.abs(kappa_r).max():.2f} (r {1/np.abs(kappa_r).max():.2f} m), TUM crossing flag: {crossing}')

    # ---------- 3. iterative minimum curvature, re-referenced on its own output ----------
    for outer in range(P['outer_iters']):
        alpha, reftrack_i, normvec_i, spline_len_i, psi_i, kappa_i, dkappa_i = tph.iqp_handler.iqp_handler(
            reftrack=reftrack, normvectors=normvec, A=M, spline_len=spline_len, psi=psi_r, kappa=kappa_r, dkappa=dkappa_r,
            kappa_bound=P['kappa_bound'], w_veh=w_veh, print_debug=False, plot_debug=False,
            stepsize_interp=ds, iters_min=P['iters_min'], curv_error_allowed=P['curv_err'])
        rl, a_rl, cx_rl, cy_rl, inds, tvals, s_rl, spl_len_rl, el_cl = tph.create_raceline.create_raceline(
            refline=reftrack_i[:, :2], normvectors=normvec_i, alpha=alpha, stepsize_interp=ds)
        psi_rl, kappa_rl = tph.calc_head_curv_an.calc_head_curv_an(cx_rl, cy_rl, inds, tvals)
        clear = np.array([Dat(x, y) for x, y in rl])
        print(f'[3] outer {outer+1}: {len(rl)} pts, {s_rl[-1]+el_cl[-1]:.2f} m, max|k| {np.abs(kappa_rl).max():.2f} (r {1/np.abs(kappa_rl).max():.2f} m), over 1.78: {(np.abs(kappa_rl)>1.78).sum()}, min clearance {clear.min():.2f} m')
        # next round: the raceline itself is the reference, widths ray-cast fresh, no cap needed once it is drivable
        reftrack, normvec, M, spline_len, psi_r, kappa_r, dkappa_r = build_ref(rl, cap=(np.abs(kappa_rl).max() > 1.3))
    n = len(rl)

    # ---------- 4. velocity profile ----------
    vs = np.arange(0, P['v_max'] + 2.01, 2.0)
    ax_machines = np.c_[vs, np.maximum(0.3, P['ax_drive'] - P['drag_lin'] * vs)]
    ggv = np.array([[0.0, P['ax_brake'], P['ay_max']], [P['v_max'] + 5, P['ax_brake'], P['ay_max']]])
    v = tph.calc_vel_profile.calc_vel_profile(ax_max_machines=ax_machines, kappa=kappa_rl, el_lengths=el_cl, closed=True,
                                              drag_coeff=0.0, m_veh=P['m_veh'], ggv=ggv, v_max=P['v_max'],
                                              dyn_model_exp=P['dyn_model_exp'], filt_window=P['filt_window'])
    t_lap = np.sum(el_cl / np.maximum(v, 0.1))
    ax_dem = v * np.gradient(v, s_rl); ay_dem = v ** 2 * np.abs(kappa_rl)
    print(f'[4] profile: lap {t_lap:.3f} s, v {v.min():.2f}..{v.max():.2f}, max a_y {ay_dem.max():.2f}, a_x {ax_dem.min():.2f}..{ax_dem.max():.2f}')

    # ---------- 5. export in roboracer_stack format ----------
    psi_std = np.arctan2(np.gradient(rl[:, 1]), np.gradient(rl[:, 0]))          # atan2 convention (raceline.py)
    # sanity: TUM psi vs atan2 differ by a constant offset; report it so nobody guesses
    off = np.angle(np.exp(1j * (psi_rl - psi_std))).mean()
    print(f'    heading convention: TUM psi - atan2 psi = {off:+.3f} rad  -> exporting atan2')
    wl_f, wr_f = widths(rl, psi_std)
    clear = np.array([Dat(x, y) for x, y in rl])
    i0 = int(np.argmin(np.hypot(rl[:, 0] - sx, rl[:, 1] - sy)))
    if np.cos(psi_std[i0] - syaw) < 0:
        sys.exit('BUG: exported line runs against the spawn heading')
    print(f'    at spawn: line point ({rl[i0,0]:.2f}, {rl[i0,1]:.2f}) psi {np.degrees(psi_std[i0]):+.0f} deg, spawn yaw {np.degrees(syaw):+.0f} deg, offset {np.hypot(rl[i0,0]-sx, rl[i0,1]-sy):.2f} m')
    print(f'    min clearance to wall {clear.min():.2f} m (car half-width 0.135 + margin {P["margin"]:.2f} = {0.135+P["margin"]:.3f})')
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.map_yaml)), 'raceline_new.csv')
    hdr = '# s_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m,v_mps'
    np.savetxt(out, np.c_[s_rl, rl, psi_std, kappa_rl, wr_f, wl_f, v], delimiter=',', fmt='%.5f', header=hdr, comments='')
    print('wrote', out)
    pk, _ = find_peaks(np.abs(kappa_rl), height=0.35, distance=15)
    for p in pk: print(f'    corner s={s_rl[p]:5.1f}  k={kappa_rl[p]:+.2f}  r={1/abs(kappa_rl[p]):.2f} m  v={v[p]:.2f}  clearance {clear[p]:.2f}')

    if a.plot:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig = plt.figure(figsize=(16, 11)); ax0 = fig.add_subplot(3, 1, 1)
        ax0.imshow(img[::-1, :].T, cmap='gray', extent=[oy, oy + H * res, ox, ox + W * res], origin='lower')
        ax0.plot(refline[:, 1], refline[:, 0], color='#B4C5C1', lw=1, label='reference (smoothed medial axis)')
        if a.old:
            old = np.loadtxt(a.old, delimiter=','); ax0.plot(old[:, 2], old[:, 1], color='#0C7480', lw=1.2, label='old raceline')
        sc = ax0.scatter(rl[:, 1], rl[:, 0], c=v, cmap='viridis', s=7, label='new raceline (colour = v)'); plt.colorbar(sc, ax=ax0, fraction=0.02, pad=0.01, label='m/s')
        ax0.plot(rl[0, 1], rl[0, 0], 'ko', ms=7, label='s=0'); ax0.set_aspect('equal'); ax0.legend(loc='lower right', fontsize=9); ax0.set_title('screen x = world y')
        ax1 = fig.add_subplot(3, 1, 2); ax1.plot(s_rl, kappa_rl, color='#B0175C', lw=1.4, label='raceline κ'); ax1.plot(np.arange(len(kappa_r)) * ds, kappa_r, color='#B4C5C1', lw=1, label='reference κ')
        for vv in (1.78, -1.78): ax1.axhline(vv, ls='--', c='gray', lw=.9)
        for vv in (P['kappa_bound'], -P['kappa_bound']): ax1.axhline(vv, ls=':', c='gray', lw=.9)
        ax1.set_ylim(-3, 3); ax1.set_ylabel('κ 1/m'); ax1.legend(loc='upper right', fontsize=9)
        ax2 = fig.add_subplot(3, 1, 3); ax2.plot(s_rl, v, color='#B0175C', lw=1.6, label=f'v(s)  lap {t_lap:.2f} s')
        if a.old and old.shape[1] > 7: ax2.plot(old[:, 0], old[:, 7], color='#0C7480', lw=1, label=f'old profile  lap {np.sum((old[1,0]-old[0,0])/old[:,7]):.2f} s')
        ax2.set_ylabel('m/s'); ax2.set_xlabel('s (m)'); ax2.legend(loc='upper right', fontsize=9)
        plt.tight_layout(); png = os.path.splitext(out)[0] + '.png'; plt.savefig(png, dpi=100); print('wrote', png)


if __name__ == '__main__':
    main()
