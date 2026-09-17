#!/usr/bin/env python3
"""Convert an x,y,v,kappa raceline into the 8-column CSV pure_pursuit reads.

    python3 scripts/convert_raceline.py SRC.csv MAP.yaml OUT.csv [--n 400]

Input: a closed line with a header naming at least x and y (v and kappa are
used when present), e.g.

    x,y,v,kappa,steer_deg

Output, one row per point, what planning/raceline.py indexes by position:

    s_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m,v_mps

    s        arc length, resampled UNIFORMLY -- Raceline.lap_len assumes it
    psi      heading of the line, from central differences
    kappa    the source column when it agrees in sign with the geometry,
             otherwise recomputed (see --kappa)
    w_right  free space to the right / left wall, ray-cast perpendicular to
    w_left   the line through the map, min over neighbours (--window). Column 5
             is RIGHT and 6 is LEFT: that is what Raceline.margin_in reads
             (col 6 on a left turn), whatever raceline.py's docstring says.
    v        the source column; omitted if the source has none

Runs anywhere numpy is installed; needs nothing from ROS.
"""

import argparse
import os
import sys

import numpy as np


def read_source(path):
    with open(path) as f:
        header = f.readline().strip().lstrip('#').strip()
    names = [h.strip().lower() for h in header.split(',')]
    if 'x' not in names or 'y' not in names:
        sys.exit(f'{path}: header must name x and y columns, got {names}')
    data = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    col = {n: data[:, i] for i, n in enumerate(names)}
    return col['x'], col['y'], col.get('v'), col.get('kappa')


def read_map(yaml_path):
    """Minimal map_server YAML + binary PGM reader. Returns free mask and geometry."""
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            if ':' in line and not line.lstrip().startswith('#'):
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    res = float(meta['resolution'])
    ox, oy, oyaw = (float(t) for t in meta['origin'].strip('[]').split(','))
    if abs(oyaw) > 1e-9:
        sys.exit('map origin yaw is not 0; this converter does not handle rotated maps')
    negate = int(meta.get('negate', 0))
    free_thresh = float(meta.get('free_thresh', 0.25))
    img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    if not os.path.isfile(img_path):
        sys.exit(f'{yaml_path}: image {meta["image"]} not found next to it')

    with open(img_path, 'rb') as f:
        raw = f.read()
    # P5 header: magic, width, height, maxval, separated by whitespace/comments.
    tokens, i = [], 0
    while len(tokens) < 4:
        while raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while raw[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not raw[j:j + 1].isspace():
            j += 1
        tokens.append(raw[i:j])
        i = j
    if tokens[0] != b'P5':
        sys.exit(f'{img_path}: only binary PGM (P5) is supported')
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        sys.exit(f'{img_path}: 16-bit PGM not supported')
    img = np.frombuffer(raw, dtype=np.uint8, count=w * h, offset=i + 1).reshape(h, w)

    # map_server semantics: occupancy = (255 - p)/255, or p/255 when negated.
    occ = (img.astype(float) / 255.0) if negate else ((255.0 - img) / 255.0)
    free = occ < free_thresh            # unknown counts as wall, as for AMCL
    return free, res, ox, oy


def resample_closed(x, y, cols, n):
    """Uniform arc-length resampling of a closed polyline (periodic linear interp)."""
    if np.hypot(x[-1] - x[0], y[-1] - y[0]) < 1e-6:     # drop a repeated start point
        x, y, cols = x[:-1], y[:-1], [c[:-1] if c is not None else None for c in cols]
    xc, yc = np.append(x, x[0]), np.append(y, y[0])
    seg = np.hypot(np.diff(xc), np.diff(yc))
    s_src = np.concatenate([[0.0], np.cumsum(seg)])
    length = s_src[-1]
    s = np.arange(n) * (length / n)
    out = [np.interp(s, s_src, xc), np.interp(s, s_src, yc)]
    for c in cols:
        out.append(None if c is None else np.interp(s, s_src, np.append(c, c[0])))
    return s, length, out, seg


def heading_and_curvature(x, y, ds):
    dx = (np.roll(x, -1) - np.roll(x, 1)) / (2 * ds)
    dy = (np.roll(y, -1) - np.roll(y, 1)) / (2 * ds)
    ddx = (np.roll(x, -1) - 2 * x + np.roll(x, 1)) / ds ** 2
    ddy = (np.roll(y, -1) - 2 * y + np.roll(y, 1)) / ds ** 2
    psi = np.arctan2(dy, dx)
    kappa = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)
    return psi, kappa


def cast(free, res, ox, oy, x, y, ang, max_range):
    """Distance from (x, y) along `ang` to the first non-free cell (or max_range)."""
    h, w = free.shape
    step = res / 2.0
    r = np.arange(step, max_range + step, step)
    px = x[:, None] + r[None, :] * np.cos(ang)[:, None]
    py = y[:, None] + r[None, :] * np.sin(ang)[:, None]
    col = np.floor((px - ox) / res).astype(int)
    row = h - 1 - np.floor((py - oy) / res).astype(int)
    inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    hit = ~inside
    hit[inside] = ~free[row[inside], col[inside]]
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(r) - 1)
    return r[first]


def window_min(w, k):
    return np.min([np.roll(w, i) for i in range(-k, k + 1)], axis=0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src')
    ap.add_argument('map_yaml')
    ap.add_argument('out')
    ap.add_argument('--n', type=int, default=400, help='points in the output (default 400)')
    ap.add_argument('--kappa', choices=('auto', 'source', 'geometric'), default='auto',
                    help='auto: source column unless its sign disagrees with the geometry')
    ap.add_argument('--max-range', type=float, default=5.0, help='ray-cast limit in m')
    ap.add_argument('--window', type=int, default=2,
                    help='wall distance = min over this many neighbours each side (default 2)')
    a = ap.parse_args()

    x0, y0, v0, k0 = read_source(a.src)
    free, res, ox, oy = read_map(a.map_yaml)

    s, length, (x, y, v, k_src), seg = resample_closed(x0, y0, [v0, k0], a.n)
    ds = length / a.n
    psi, k_geo = heading_and_curvature(x, y, ds)

    kappa = k_geo
    if k_src is not None and a.kappa != 'geometric':
        bends = np.abs(k_geo) > 0.1
        agree = float(np.mean(np.sign(k_src[bends]) == np.sign(k_geo[bends]))) if bends.any() else 1.0
        print(f'kappa sign agreement (source vs geometry, |k|>0.1): {agree:.0%}')
        if a.kappa == 'source' or agree >= 0.9:
            kappa = k_src
        elif agree <= 0.1:
            print('  source kappa has the OPPOSITE sign convention; using geometric kappa')
        else:
            print('  source kappa disagrees with the geometry; using geometric kappa')

    # Minimum over +-window samples: in a hairpin the perpendicular ray can run
    # down the corridor and report metres of room on the inside, which would
    # let pure_pursuit's sag limit stretch the lookahead across the apex.
    w_right = window_min(cast(free, res, ox, oy, x, y, psi - np.pi / 2, a.max_range), a.window)
    w_left = window_min(cast(free, res, ox, oy, x, y, psi + np.pi / 2, a.max_range), a.window)

    cols = [s, x, y, psi, kappa, w_right, w_left]
    header = 's_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m'
    if v is not None:
        cols.append(v)
        header += ',v_mps'
    np.savetxt(a.out, np.column_stack(cols), delimiter=',', fmt='%.5f', header=header, comments='# ')

    # ---- Sanity report: these are the mistakes that fail silently on track.
    h, w = free.shape
    c = np.floor((x - ox) / res).astype(int)
    r = h - 1 - np.floor((y - oy) / res).astype(int)
    inside = (c >= 0) & (c < w) & (r >= 0) & (r < h)
    on_free = np.zeros_like(inside)
    on_free[inside] = free[r[inside], c[inside]]
    print(f'source: {len(x0)} pts, spacing {seg.min():.3f}-{seg.max():.3f} m')
    print(f'output: {a.n} pts, lap {length:.2f} m, ds {ds:.4f} m -> {a.out}')
    print(f'w_right {w_right.min():.2f}-{w_right.max():.2f} m, w_left {w_left.min():.2f}-{w_left.max():.2f} m')
    if v is not None:
        print(f'v {v.min():.2f}-{v.max():.2f} m/s')
    print(f'max |kappa| {np.abs(kappa).max():.2f} 1/m; start ({x[0]:.3f}, {y[0]:.3f}) heading {np.degrees(psi[0]):.1f} deg')
    bad = int((~on_free).sum())
    if bad:
        print(f'WARNING: {bad}/{a.n} points are NOT on free map cells -- '
              'the line and map are probably in different frames', file=sys.stderr)
    tight = int((np.minimum(w_left, w_right) < 0.135).sum())
    if tight:
        print(f'WARNING: {tight} points are closer than half the car (0.135 m) to a wall', file=sys.stderr)
    if bad > a.n // 2:
        sys.exit(1)


if __name__ == '__main__':
    main()
