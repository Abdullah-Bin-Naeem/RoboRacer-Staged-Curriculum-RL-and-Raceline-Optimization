#!/usr/bin/env python3
"""Build the Centreline Editor page for a track map.

    python3 make_centreline_editor.py track_clean.yaml [-r raceline.csv] [-o editor.html]

Reads the nav2 map (yaml + pgm), extracts the skeleton centreline the ForzaETH way
(morphological opening -> Lee skeleton -> spur pruning -> loop walk), resamples it,
and bakes everything into a self-contained HTML editor:

  * smoothing window slider with live curvature plot and pass/fail readouts,
  * drag-to-edit with a brush (for pushing the hairpin reference into open space),
  * set-start / reverse-direction,
  * export of a TUM reference-track CSV (x, y, w_right, w_left [, s, kappa]) with
    widths ray-cast on the map from every smoothed point.

Open the HTML in a browser. Dependencies: numpy, scipy, scikit-image, Pillow, PyYAML.
"""
import argparse, base64, io, json, os, sys
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize

HERE = os.path.dirname(os.path.abspath(__file__))


def load_map(yaml_path):
    m = yaml.safe_load(open(yaml_path))
    img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), m['image'])
    img = np.array(Image.open(img_path).convert('L'))
    if m.get('negate', 0):
        img = 255 - img
    return m, img


def centreline_from_map(img, res, ox, oy, free_min=250, open_px=9, open_iters=2):
    H, W = img.shape
    free = img >= free_min
    opened = ndimage.binary_opening(free, structure=np.ones((open_px, open_px), bool), iterations=open_iters)
    sk = skeletonize(opened, method='lee').astype(np.uint8)
    K = np.ones((3, 3), int)
    for _ in range(600):                              # prune spurs: peel endpoints until only the loop is left
        nb = ndimage.convolve(sk.astype(int), K, mode='constant') - sk
        ends = (sk == 1) & (nb == 1)
        if not ends.any():
            break
        sk[ends] = 0
    lab, n = ndimage.label(sk, structure=K)
    if n == 0:
        sys.exit('no skeleton found — check free_min / the map')
    sizes = ndimage.sum(sk, lab, range(1, n + 1))
    sk = lab == (int(np.argmax(sizes)) + 1)
    pts = np.argwhere(sk)
    tree = cKDTree(pts)
    vis = np.zeros(len(pts), bool); order = [0]; vis[0] = True; cur = 0
    for _ in range(len(pts) - 1):                     # walk the loop
        d, idx = tree.query(pts[cur], k=9)
        nxt = None
        for dd, ii in zip(d, idx):
            if not vis[ii] and dd < 1.5:
                nxt = ii; break
        if nxt is None:
            d, idx = tree.query(pts[cur], k=40)
            c = [(dd, ii) for dd, ii in zip(d, idx) if not vis[ii]]
            if not c or c[0][0] > 3:
                break
            nxt = c[0][1]
        vis[nxt] = True; order.append(nxt); cur = nxt
    P = pts[order]
    x = ox + (P[:, 1] + 0.5) * res
    y = oy + (H - P[:, 0] - 0.5) * res
    return np.c_[x, y], opened


def resample_closed(xy, ds):
    d = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)
    s = np.concatenate([[0], np.cumsum(d)])
    L = s[-1]
    su = np.arange(0, L, ds)
    xyc = np.vstack([xy, xy[:1]])
    return np.c_[np.interp(su, s, xyc[:, 0]), np.interp(su, s, xyc[:, 1])], L


def png_data_uri(arr_u8):
    buf = io.BytesIO(); Image.fromarray(arr_u8).save(buf, format='PNG', optimize=True)
    return 'data:image/png;base64,' + base64.b64encode(buf.getvalue()).decode()


def rotate_for_display(a):
    # rows = x index, cols = y index  (what the page expects: screen-x ∝ world-y)
    return a[::-1, :].T.copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_yaml')
    ap.add_argument('-r', '--raceline', help='optional CSV with x,y in columns 1,2 (roboracer format) or 0,1')
    ap.add_argument('-o', '--out', default=None)
    ap.add_argument('--ds', type=float, default=0.10)
    ap.add_argument('--free-min', type=int, default=250)
    ap.add_argument('--template', default=os.path.join(HERE, 'centreline_editor_template.html'))
    a = ap.parse_args()

    m, img = load_map(a.map_yaml)
    res = float(m['resolution']); ox, oy = float(m['origin'][0]), float(m['origin'][1])
    H, W = img.shape
    xy, opened = centreline_from_map(img, res, ox, oy, free_min=a.free_min)
    center, L = resample_closed(xy, a.ds)
    print(f'map {W}x{H} px @ {res} m, centreline {L:.2f} m, {len(center)} points')

    raceline = None
    if a.raceline:
        rl = np.loadtxt(a.raceline, delimiter=',')
        cols = (1, 2) if rl.shape[1] >= 5 else (0, 1)
        raceline = rl[:, cols].round(4).tolist()

    data = {
        'name': os.path.splitext(os.path.basename(m['image']))[0],
        'res': res, 'xmin': ox, 'ymin': oy, 'nx': W, 'ny': H,
        'mapPng': png_data_uri(rotate_for_display(img)),
        'maskPng': png_data_uri(rotate_for_display((opened * 255).astype(np.uint8))),
        'center': center.round(4).tolist(),
        'raceline': raceline,
    }
    tpl = open(a.template).read()
    html = tpl.replace('/*__DATA__*/null', json.dumps(data))
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.map_yaml)), data['name'] + '_editor.html')
    open(out, 'w').write(html)
    print('wrote', out, f'({len(html)//1024} kB)')


if __name__ == '__main__':
    main()
