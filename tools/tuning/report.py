#!/usr/bin/env python3
"""One experiment directory -> numbers, figures, and a row in the index.

    python3 tools/tuning/report.py experiments/iros2026/E01_baseline_a60z

Reads from the experiment directory
    config.json   what ran (written by run_experiment.sh)
    run.csv       log_localization per-tick log (truth, estimate, pp_* follower status)
    laps.csv      the simulator's own lap/collision telemetry (dev-only watcher)
Writes
    summary.json  every number below, machine-readable, for later comparison
    report.md     the human-readable version, with the figures linked
    figures/      track overlay, lateral error, clearance, speed, laps, localization,
                  and plot_speed_tracking.py's time-loss split
and appends/updates the experiment's row in experiments/<track>/EXPERIMENTS.md.

Definitions (all from ground truth, all per timed lap unless stated):
    e_lat       signed lateral offset of the TRUE car from the raceline, + = left.
    outward     e_lat signed toward the outside of the local turn (+ = running wide).
    clearance   min distance from the car's footprint (0.50 x 0.27 m, anchored at the
                IPS point with the rear axle 0.08 m behind it) to a non-free cell of
                the geometry map. Comparative: the anchor is uncertain by ~0.08 m.
    a_lat       speed x yaw_rate from ground truth, the lateral demand actually used.
    out-lap     lap 0 (spawn to first crossing) is excluded from every lap statistic.
Timed laps end at the first contact: laps after a respawn are not comparable.
"""
import csv
import json
import math
import os
import subprocess
import sys
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))

CAR_REAR, CAR_FRONT, CAR_HALF_W = -0.08 - 0.08, 0.414 - 0.08, 0.135   # relative to the IPS point
CORNER_KAPPA = 0.35


# ---------------------------------------------------------------- loading
def load_csv(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return {}
    out = {}
    for k in rows[0]:
        if not k:
            continue
        vals = []
        for r in rows:
            try:
                vals.append(float(r[k]))
            except (TypeError, ValueError):
                vals.append(np.nan)
        out[k] = np.array(vals)
    return out


def load_line(path):
    a = np.loadtxt(path, delimiter=',', comments='#')
    s, x, y, psi, k, wr, wl, v = a.T
    L = float(s[-1] + (s[1] - s[0]))
    ds = np.diff(np.r_[s, L])
    return dict(s=s, x=x, y=y, psi=psi, k=k, wr=wr, wl=wl, v=v, L=L,
                profile_lap=float(np.sum(ds / np.maximum(v, 0.1))))


def load_map(track):
    base = os.path.join(REPO, 'devkit_ws/src/racer_mapping/maps', track)
    name = 'track_solid' if os.path.exists(os.path.join(base, 'track_solid.yaml')) else 'track_clean'
    import yaml
    meta = yaml.safe_load(open(os.path.join(base, name + '.yaml')))
    raw = open(os.path.join(base, meta['image']), 'rb').read()
    parts, i = [], 0
    while len(parts) < 4:                                   # P5 header: magic, w, h, maxval
        while raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while raw[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not raw[j:j + 1].isspace():
            j += 1
        parts.append(raw[i:j])
        i = j
    w, h = int(parts[1]), int(parts[2])
    img = np.frombuffer(raw[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    return dict(img=img, res=res, ox=ox, oy=oy, w=w, h=h, name=name)


def distance_field(m):
    from scipy import ndimage
    free = m['img'] >= 250
    return ndimage.distance_transform_edt(free) * m['res']


def sample_field(field, m, x, y):
    col = np.clip(((x - m['ox']) / m['res']).astype(int), 0, m['w'] - 1)
    row = np.clip((m['h'] - 1 - (y - m['oy']) / m['res']).astype(int), 0, m['h'] - 1)
    return field[row, col]


# ---------------------------------------------------------------- geometry
def project(line, x, y, chunk=4000):
    """Nearest raceline point per sample -> (idx, s_on_line, e_lat signed + = left)."""
    idx = np.empty(len(x), dtype=int)
    for a in range(0, len(x), chunk):
        dx = x[a:a + chunk, None] - line['x'][None, :]
        dy = y[a:a + chunk, None] - line['y'][None, :]
        idx[a:a + chunk] = np.argmin(dx * dx + dy * dy, axis=1)
    psi = line['psi'][idx]
    e = -(x - line['x'][idx]) * np.sin(psi) + (y - line['y'][idx]) * np.cos(psi)
    return idx, line['s'][idx], e


def corners(line):
    k, s = line['k'], line['s']
    m = np.abs(k) > CORNER_KAPPA
    out, i, n = [], 0, len(s)
    while i < n:
        if not m[i]:
            i += 1
            continue
        j = i
        while j < n and m[j]:
            j += 1
        ia = i + int(np.argmax(np.abs(k[i:j])))
        out.append(dict(name=f'C{len(out) + 1}', s0=float(s[i]), s1=float(s[j - 1]), apex=int(ia),
                        side='L' if k[ia] > 0 else 'R', kappa=float(abs(k[ia])), v_plan=float(line['v'][ia])))
        i = j
    return out


def footprint_clearance(field, m, x, y, yaw):
    pts = [(CAR_REAR, -CAR_HALF_W), (CAR_REAR, CAR_HALF_W), (CAR_FRONT, -CAR_HALF_W), (CAR_FRONT, CAR_HALF_W),
           (0.5 * (CAR_REAR + CAR_FRONT), -CAR_HALF_W), (0.5 * (CAR_REAR + CAR_FRONT), CAR_HALF_W),
           (CAR_FRONT, 0.0), (CAR_REAR, 0.0)]
    c, s = np.cos(yaw), np.sin(yaw)
    d = np.full(len(x), np.inf)
    for px, py in pts:
        d = np.minimum(d, sample_field(field, m, x + px * c - py * s, y + px * s + py * c))
    return d


# ---------------------------------------------------------------- laps
def lap_segments(t, s_line, L, ready):
    """Crossings of s = 0 on the line (going forward) -> list of (start_idx, end_idx)."""
    wraps = np.flatnonzero((np.diff(s_line) < -0.5 * L))
    edges = [int(w + 1) for w in wraps]
    return [(a, b) for a, b in zip(edges[:-1], edges[1:])]


def first_contact_index(log, laps_tel):
    """Index of the first respawn in the log: a truth jump the speed cannot explain."""
    x, y, t = log['true_x'], log['true_y'], log['t']
    step = np.hypot(np.diff(x), np.diff(y))
    dt = np.maximum(np.diff(t), 1e-3)
    v = np.nan_to_num(log['speed'][1:], nan=0.0)
    yaw = np.radians(log['true_yaw_deg'])
    dyaw = np.abs(np.angle(np.exp(1j * np.diff(yaw))))
    # a respawn moves the car further than its speed allows OR turns it faster than
    # it can yaw; a slow respawn near the hit point (E02: 0.33 m, 33 deg) is only the second
    jump = np.flatnonzero((step > np.maximum(0.6, 3.0 * (np.abs(v) + 1.0) * dt))
                          | ((step > 0.15) & (dyaw > np.radians(20.0)) & (dyaw > 3.0 * dt + np.radians(10.0))))
    return int(jump[0] + 1) if len(jump) else None


# ---------------------------------------------------------------- main
def main(exp_dir):
    exp_dir = os.path.abspath(exp_dir)
    cfg = json.load(open(os.path.join(exp_dir, 'config.json')))
    track = cfg.get('track', 'iros2026')
    line_path = os.path.join(REPO, 'raceline', track, cfg['line'])
    line = load_line(line_path)
    log = load_csv(os.path.join(exp_dir, 'run.csv'))
    tel_path = os.path.join(exp_dir, 'laps.csv')
    tel = list(csv.DictReader(open(tel_path))) if os.path.exists(tel_path) else []
    figdir = os.path.join(exp_dir, 'figures')
    os.makedirs(figdir, exist_ok=True)

    ok = np.isfinite(log.get('true_x', np.array([]))) if log else np.array([], bool)
    if not log or ok.sum() < 20:
        summary = dict(id=cfg['id'], status='NO DATA', config=cfg)
        json.dump(summary, open(os.path.join(exp_dir, 'summary.json'), 'w'), indent=2)
        update_index(exp_dir, cfg, summary)
        print('no usable rows in run.csv')
        return
    log = {k: v[ok] for k, v in log.items()}
    t = log['t'] - log['t'][0]
    x, y = log['true_x'], log['true_y']
    yaw = np.radians(log['true_yaw_deg'])
    idx, s_line, e_lat = project(line, x, y)
    kap = line['k'][idx]
    outward = -np.sign(kap) * e_lat                      # + = toward the outside of the turn
    m = load_map(track)
    field = distance_field(m)
    clear = footprint_clearance(field, m, x, y, yaw)
    speed = log['speed']
    a_lat = np.abs(speed * log['yaw_rate'])
    loc_err = log.get('err_dist', np.full(len(t), np.nan))
    ready = log.get('ready', np.ones(len(t)))

    # ---- contacts: the simulator's counter first, the log's respawn signature second
    contacts_tel = int(max((int(float(r['collisions'])) for r in tel), default=0))
    i_contact = first_contact_index(log, tel)
    t_contact = float(t[i_contact]) if i_contact is not None else None

    # ---- laps: s-wrap segments, timed laps = all but the out-lap, before the first contact
    segs = lap_segments(t, s_line, line['L'], ready)
    timed = [(a, b) for a, b in segs if i_contact is None or b <= i_contact]
    lap_times_log = [float(t[b] - t[a]) for a, b in timed]
    sim_laps = [float(r['lap_time']) for r in tel if r.get('lap_time') not in (None, '', 'nan')
                and 0 < float(r['lap_time']) < 60 and int(float(r['lap'])) >= 2
                and int(float(r['collisions'])) == 0]
    lap_times = sim_laps if tel else lap_times_log          # the simulator's own count wins whenever it exists
    lt = np.array(lap_times)
    lap_mask = np.zeros(len(t), bool)
    for a, b in timed:
        lap_mask[a:b] = True

    def stat(arr, mask=None):
        v = arr[mask] if mask is not None else arr
        v = v[np.isfinite(v)]
        if not len(v):
            return dict(mean=None, p90=None, max=None)
        return dict(mean=round(float(np.mean(v)), 4), p90=round(float(np.percentile(v, 90)), 4),
                    max=round(float(np.max(v)), 4))

    mk = lap_mask if lap_mask.any() else np.ones(len(t), bool)
    dly = log.get('pp_delay', np.full(len(t), np.nan))
    tick_dt = np.diff(t[mk]) if mk.sum() > 2 else np.array([np.nan])
    # The logger samples at 20 Hz, so pose updates cannot show a faster loop. The
    # round trip is three simulator frames (VEHICLE_MODEL 3.6): tick = 3 / delay.
    dly_med = float(np.nanmedian(dly[mk])) if np.isfinite(dly[mk]).any() else float('nan')
    loop_hz = 3.0 / dly_med if dly_med == dly_med and dly_med > 0 else None

    crn = corners(line)
    corner_rows = []
    for c in crn:
        in_c = mk & (s_line >= c['s0'] - 0.5) & (s_line <= c['s1'] + 0.5)
        if not in_c.any():
            continue
        apex_win = mk & (np.abs(s_line - line['s'][c['apex']]) < 0.3)
        corner_rows.append(dict(
            name=c['name'], s=f"{c['s0']:.1f}-{c['s1']:.1f}", side=c['side'], kappa=round(c['kappa'], 2),
            v_plan_apex=round(c['v_plan'], 2),
            v_true_apex=round(float(np.nanmean(speed[apex_win])), 2) if apex_win.any() else None,
            outward_max=round(float(np.nanmax(outward[in_c])), 3),
            abs_e_p90=round(float(np.nanpercentile(np.abs(e_lat[in_c]), 90)), 3),
            clearance_min=round(float(np.nanmin(clear[in_c])), 3),
            a_lat_max=round(float(np.nanmax(a_lat[in_c])), 2),
            steer_lock_pct=round(100.0 * float(np.mean(np.abs(log['pp_steering'][in_c]) >= 0.99)), 1)
            if 'pp_steering' in log else None))

    summary = dict(
        id=cfg['id'], name=cfg.get('name'), date=cfg.get('date'), line=cfg['line'],
        overrides=cfg.get('overrides', []), hz_cap=cfg.get('hz_cap'), laps_target=cfg.get('laps'),
        profile_lap_s=round(line['profile_lap'], 3),
        timed_laps=len(lap_times), lap_source='simulator' if tel else 'log s-wrap',
        lap_best=round(float(lt.min()), 3) if len(lt) else None,
        lap_median=round(float(np.median(lt)), 3) if len(lt) else None,
        lap_mean=round(float(lt.mean()), 3) if len(lt) else None,
        lap_std=round(float(lt.std()), 3) if len(lt) > 1 else None,
        lap_worst=round(float(lt.max()), 3) if len(lt) else None,
        laps_under_10=int(np.sum(lt < 10.0)) if len(lt) else 0,
        gap_to_profile_s=round(float(np.median(lt) - line['profile_lap']), 3) if len(lt) else None,
        contacts=max(contacts_tel, 1 if i_contact is not None else 0),
        first_contact_t=round(t_contact, 1) if t_contact is not None else None,
        first_contact_s=round(float(s_line[i_contact - 1]), 2) if i_contact else None,
        e_lat_abs=stat(np.abs(e_lat), mk), outward=stat(outward, mk),
        clearance_min=round(float(np.nanmin(clear[mk])), 3), clearance_p05=round(float(np.nanpercentile(clear[mk], 5)), 3),
        loc_err=stat(loc_err, mk), a_lat=stat(a_lat, mk),
        v_est_err=stat(np.abs(log['pp_v_est'] - speed), mk) if 'pp_v_est' in log else None,
        delay_median=round(float(np.nanmedian(dly[mk])), 3) if np.isfinite(dly[mk]).any() else None,
        loop_hz=round(loop_hz, 1) if loop_hz else None,
        steer_lock_pct=round(100.0 * float(np.mean(np.abs(log['pp_steering'][mk]) >= 0.99)), 2) if 'pp_steering' in log else None,
        corners=corner_rows,
    )
    summary['gate_50'] = bool(summary['contacts'] == 0 and summary['timed_laps'] >= 50)
    summary['gate_sub10'] = bool(summary['lap_mean'] is not None and summary['lap_mean'] < 10.0
                                 and summary['lap_worst'] is not None and summary['lap_worst'] < 10.2)
    json.dump(summary, open(os.path.join(exp_dir, 'summary.json'), 'w'), indent=2)

    figures(figdir, cfg, line, m, log, t, x, y, s_line, e_lat, outward, clear, speed, a_lat, loc_err,
            segs, timed, i_contact, lt, crn)
    try:
        subprocess.run([sys.executable, os.path.join(REPO, 'raceline', 'plot_speed_tracking.py'),
                        os.path.join(exp_dir, 'run.csv'), '--path', line_path, '--out', figdir],
                       stdout=open(os.path.join(exp_dir, 'speed_tracking.txt'), 'w'), stderr=subprocess.STDOUT,
                       timeout=300)
    except Exception as exc:                                   # the report must not die on one plot
        print('plot_speed_tracking failed:', exc)
    write_markdown(exp_dir, cfg, summary)
    update_index(exp_dir, cfg, summary)
    print(json.dumps({k: summary[k] for k in ('id', 'timed_laps', 'lap_best', 'lap_median', 'lap_mean',
                                              'contacts', 'clearance_min', 'gap_to_profile_s', 'loop_hz',
                                              'delay_median')}, indent=1))


def shade_corners(ax, crn):
    for c in crn:
        ax.axvspan(c['s0'], c['s1'], color='0.85', zorder=0)
        ax.text(0.5 * (c['s0'] + c['s1']), 1.0, c['name'], transform=ax.get_xaxis_transform(),
                ha='center', va='bottom', fontsize=8)


def figures(figdir, cfg, line, m, log, t, x, y, s_line, e_lat, outward, clear, speed, a_lat, loc_err,
            segs, timed, i_contact, lt, crn):
    title = f"{cfg['id']} {cfg.get('name', '')} | {cfg['line']}"
    stop = i_contact if i_contact is not None else len(t)

    # 1. track overlay: map, line, true path coloured by |e_lat|, contacts marked
    fig, ax = plt.subplots(figsize=(6, 12))
    ext = [m['ox'], m['ox'] + m['w'] * m['res'], m['oy'], m['oy'] + m['h'] * m['res']]
    ax.imshow(m['img'], cmap='gray', extent=ext, vmin=0, vmax=255)
    ax.plot(line['x'], line['y'], color='#0b6bcb', lw=1.0, label='raceline')
    sc = ax.scatter(x[:stop], y[:stop], c=np.abs(e_lat[:stop]), s=2, cmap='inferno_r', vmin=0, vmax=0.4)
    fig.colorbar(sc, ax=ax, fraction=0.04, label='|lateral error| m (truth)')
    for c in crn:
        ax.annotate(c['name'], (line['x'][c['apex']], line['y'][c['apex']]), color='#c0392b', fontsize=9)
    if i_contact is not None:
        ax.plot(x[i_contact - 1], y[i_contact - 1], 'x', color='red', ms=12, mew=3, label='first contact')
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=9)
    ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, 'track_overlay.png'), dpi=120)
    plt.close(fig)

    # 2. lateral error, clearance, speed, localization against s: one line per timed lap
    fig, axs = plt.subplots(4, 1, figsize=(13, 13), sharex=True)
    cmap = plt.get_cmap('viridis')
    for n, (a, b) in enumerate(timed):
        col = cmap(n / max(1, len(timed) - 1))
        o = np.argsort(s_line[a:b])
        ss = s_line[a:b][o]
        axs[0].plot(ss, e_lat[a:b][o], color=col, lw=0.7, alpha=0.7)
        axs[1].plot(ss, clear[a:b][o], color=col, lw=0.7, alpha=0.7)
        axs[2].plot(ss, speed[a:b][o], color=col, lw=0.7, alpha=0.6)
        axs[3].plot(ss, loc_err[a:b][o], color=col, lw=0.7, alpha=0.7)
    if i_contact is not None:
        a0 = max(0, i_contact - 60)
        axs[0].plot(s_line[a0:i_contact], e_lat[a0:i_contact], color='red', lw=1.5, label='into first contact')
        axs[0].legend(fontsize=8)
    axs[0].axhline(0, color='k', lw=0.5)
    axs[0].plot(line['s'], line['wl'] - 0.135, 'k--', lw=0.6)
    axs[0].plot(line['s'], -(line['wr'] - 0.135), 'k--', lw=0.6)
    axs[0].set_ylim(-1.0, 1.0)
    axs[0].set_ylabel('e_lat (m, + left)\n-- = wall room')
    axs[1].axhline(0.10, color='orange', lw=0.8)
    axs[1].axhline(0.0, color='red', lw=0.8)
    axs[1].set_ylim(-0.05, 0.8)
    axs[1].set_ylabel('body clearance (m)')
    axs[2].plot(line['s'], line['v'], 'k', lw=1.2, label='plan')
    axs[2].set_ylabel('true speed (m/s)')
    axs[2].legend(fontsize=8)
    axs[3].set_ylabel('localization err (m)')
    axs[3].set_ylim(0, 0.8)
    axs[3].set_xlabel('s along raceline (m)')
    for ax in axs:
        shade_corners(ax, crn)
        ax.grid(alpha=0.3)
    axs[0].set_title(title + f'  ({len(timed)} timed laps, colour = lap order)', fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, 'along_track.png'), dpi=110)
    plt.close(fig)

    # 3. error envelope: mean and worst over laps, binned on s
    if timed:
        bins = np.linspace(0, line['L'], 216)
        mid = 0.5 * (bins[1:] + bins[:-1])
        env = {k: np.full((len(timed), len(mid)), np.nan) for k in ('e', 'o', 'c')}
        for n, (a, b) in enumerate(timed):
            bi = np.digitize(s_line[a:b], bins) - 1
            for j in range(len(mid)):
                sel = bi == j
                if sel.any():
                    env['e'][n, j] = np.nanmean(e_lat[a:b][sel])
                    env['o'][n, j] = np.nanmax(outward[a:b][sel])
                    env['c'][n, j] = np.nanmin(clear[a:b][sel])
        fig, axs = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
        axs[0].fill_between(mid, np.nanmin(env['e'], 0), np.nanmax(env['e'], 0), color='#0b6bcb', alpha=0.25,
                            label='min..max over laps')
        axs[0].plot(mid, np.nanmean(env['e'], 0), color='#0b6bcb', label='mean')
        axs[0].plot(mid, np.nanmax(env['o'], 0), color='#c0392b', lw=0.8, label='worst outward (+ = wide)')
        axs[0].axhline(0, color='k', lw=0.5)
        axs[0].set_ylabel('e_lat (m)')
        axs[0].legend(fontsize=8)
        axs[1].plot(mid, np.nanmin(env['c'], 0), color='#c0392b', label='worst lap')
        axs[1].plot(mid, np.nanmedian(env['c'], 0), color='0.3', label='median lap')
        axs[1].axhline(0.10, color='orange', lw=0.8)
        axs[1].set_ylabel('clearance (m)')
        axs[1].set_ylim(-0.05, 0.8)
        axs[1].legend(fontsize=8)
        axs[1].set_xlabel('s (m)')
        for ax in axs:
            shade_corners(ax, crn)
            ax.grid(alpha=0.3)
        axs[0].set_title(title + ' | envelope over timed laps', fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(figdir, 'envelope.png'), dpi=110)
        plt.close(fig)

    # 4. lap times
    fig, ax = plt.subplots(figsize=(11, 3.5))
    if len(lt):
        ax.bar(np.arange(1, len(lt) + 1), lt, color=['#1a7f37' if v < 10 else '#b7791f' for v in lt])
        ax.set_ylim(max(0, lt.min() - 0.5), lt.max() + 0.3)
    ax.axhline(10.0, color='red', lw=1, label='10 s target')
    ax.axhline(line['profile_lap'], color='k', ls='--', lw=1, label=f"profile {line['profile_lap']:.2f} s")
    ax.set_xlabel('timed lap')
    ax.set_ylabel('lap time (s)')
    ax.legend(fontsize=8)
    ax.set_title(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, 'lap_times.png'), dpi=110)
    plt.close(fig)


def fmt(v, nd=3):
    return '—' if v is None else (f'{v:.{nd}f}' if isinstance(v, float) else str(v))


def write_markdown(exp_dir, cfg, s):
    contact = ('none' if s['first_contact_t'] is None
               else f"+{s['first_contact_t']} s at s = {s['first_contact_s']} m")
    L = [f"# {s['id']} — {s.get('name') or ''}", '',
         f"- **date** {s.get('date')}  **line** `{s['line']}` (profile {s['profile_lap_s']} s)  "
         f"**loop cap** {s.get('hz_cap')}  **laps asked** {s.get('laps_target')}",
         f"- **overrides** `{' '.join(s.get('overrides') or []) or '(none)'}`",
         f"- **hypothesis** {cfg.get('hypothesis', '')}",
         f"- **prediction** {cfg.get('prediction', '')}", '',
         '## Result', '',
         '| timed laps | contacts | best | median | mean | std | worst | <10 s | gap to profile | loop Hz | delay |',
         '|---|---|---|---|---|---|---|---|---|---|---|',
         f"| {s['timed_laps']} | {s['contacts']} | {fmt(s['lap_best'])} | {fmt(s['lap_median'])} | {fmt(s['lap_mean'])} | "
         f"{fmt(s['lap_std'])} | {fmt(s['lap_worst'])} | {s['laps_under_10']} | {fmt(s['gap_to_profile_s'])} | "
         f"{fmt(s['loop_hz'], 1)} | {fmt(s['delay_median'])} |", '',
         f"First contact: {contact}", '',
         '| tracking (timed laps) | mean | p90 | max |', '|---|---|---|---|']
    for key, label in (('e_lat_abs', '|e_lat| m'), ('outward', 'outward m'), ('loc_err', 'localization m'),
                       ('a_lat', 'a_lat m/s²'), ('v_est_err', '|v_est − v| m/s')):
        v = s.get(key) or {}
        L.append(f"| {label} | {fmt(v.get('mean'))} | {fmt(v.get('p90'))} | {fmt(v.get('max'))} |")
    L += ['', f"Body clearance: min {s['clearance_min']} m, p05 {s['clearance_p05']} m.  "
          f"Steering at lock: {s['steer_lock_pct']} % of ticks.", '',
          '| corner | s | side | κ | v plan apex | v true apex | worst outward | |e| p90 | min clearance | a_lat max | at lock % |',
          '|---|---|---|---|---|---|---|---|---|---|---|']
    for c in s['corners']:
        L.append(f"| {c['name']} | {c['s']} | {c['side']} | {c['kappa']} | {c['v_plan_apex']} | {fmt(c['v_true_apex'], 2)} | "
                 f"{c['outward_max']} | {c['abs_e_p90']} | {c['clearance_min']} | {c['a_lat_max']} | {fmt(c['steer_lock_pct'], 1)} |")
    L += ['', f"Gates: 50 clean laps **{'PASS' if s['gate_50'] else 'no'}**, "
          f"mean < 10 s (worst < 10.2) **{'PASS' if s['gate_sub10'] else 'no'}**", '',
          '## Figures', '',
          '![track](figures/track_overlay.png)', '', '![along](figures/along_track.png)', '',
          '![envelope](figures/envelope.png)', '', '![laps](figures/lap_times.png)', '',
          '![speed](figures/run_speed_profile.png)', '', '![loss](figures/run_time_loss.png)', '',
          'Time-loss split (plot_speed_tracking.py): see `speed_tracking.txt`.', '',
          '## Verdict', '', cfg.get('verdict', '_to be written after reading the figures_'), '']
    open(os.path.join(exp_dir, 'report.md'), 'w').write('\n'.join(L))


def update_index(exp_dir, cfg, s):
    idx_path = os.path.join(os.path.dirname(exp_dir), 'EXPERIMENTS.md')
    head = ['# Experiment index', '',
            'One row per run, newest last. Written by `tools/tuning/report.py`; the verdict column is edited by hand.', '',
            '| id | date | line | overrides | loop cap | timed laps | contacts | best | median | mean | worst | gap | min clear | |e| p90 | verdict |',
            '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    rows = []
    if os.path.exists(idx_path):
        for ln in open(idx_path):
            if ln.startswith('| ') and not ln.startswith('| id ') and not ln.startswith('|---'):
                rows.append(ln.rstrip('\n'))
    ident = s['id']
    old_verdict = ''
    for r in rows:
        if r.startswith(f'| [{ident}]'):
            cells = [c.strip() for c in r.strip().strip('|').split('|')]
            old_verdict = cells[14] if len(cells) > 14 else ''
    rows = [r for r in rows if not r.startswith(f'| [{ident}]')]
    rel = os.path.basename(exp_dir)
    e = s.get('e_lat_abs') or {}
    rows.append(f"| [{ident}]({rel}/report.md) | {s.get('date') or cfg.get('date')} | `{cfg['line']}` | "
                f"`{' '.join(cfg.get('overrides') or []) or '-'}` | {cfg.get('hz_cap')} | {s.get('timed_laps', 0)} | "
                f"{s.get('contacts', '—')} | {fmt(s.get('lap_best'))} | {fmt(s.get('lap_median'))} | {fmt(s.get('lap_mean'))} | "
                f"{fmt(s.get('lap_worst'))} | {fmt(s.get('gap_to_profile_s'))} | {fmt(s.get('clearance_min'))} | "
                f"{fmt(e.get('p90'))} | {cfg.get('verdict_short') or old_verdict} |")
    rows.sort(key=lambda r: r.split(']')[0])
    open(idx_path, 'w').write('\n'.join(head + rows) + '\n')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
