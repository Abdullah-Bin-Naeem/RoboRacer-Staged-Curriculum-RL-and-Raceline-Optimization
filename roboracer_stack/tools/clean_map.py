#!/usr/bin/env python3
"""Remove SLAM speckle from a saved occupancy grid.

Two deterministic passes, no hand-editing:
  1. Drop wall blobs smaller than --min-wall cells (spurious lone hits).
  2. Keep only the largest free blob (the driving surface); everything else
     becomes unknown, so a planner cannot treat stray pockets as driveable.

Writes <name>_clean.pgm + <name>_clean.yaml next to the input.

    python clean_map.py ../maps/track.pgm
"""
import argparse
import pathlib

import numpy as np
import yaml
from scipy import ndimage

UNKNOWN, WALL, FREE = 205, 0, 254


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5', 'not a binary PGM'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        return np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w).copy()


def write_pgm(path, img):
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(img.tobytes())


ap = argparse.ArgumentParser()
ap.add_argument('pgm')
ap.add_argument('--min-wall', type=int, default=5,
                help='wall blobs with fewer cells than this are deleted')
a = ap.parse_args()

src = pathlib.Path(a.pgm)
img = read_pgm(src)
before_wall = int((img == WALL).sum())

# Pass 1: speckle walls -> unknown. 8-connectivity so diagonal wall runs survive.
lab, n = ndimage.label(img == WALL, structure=np.ones((3, 3)))
sizes = ndimage.sum(img == WALL, lab, range(1, n + 1))
doomed = np.isin(lab, [i + 1 for i in range(n) if sizes[i] < a.min_wall])
img[doomed] = UNKNOWN
print(f'pass 1: removed {int(doomed.sum())} speckle cells '
      f'in {int((sizes < a.min_wall).sum())} blobs')

# Pass 2: keep only the biggest free region.
flab, fn = ndimage.label(img >= FREE)
fsizes = ndimage.sum(img >= FREE, flab, range(1, fn + 1))
keep = int(np.argmax(fsizes)) + 1
stray = (flab != keep) & (flab != 0)
img[stray] = UNKNOWN
print(f'pass 2: kept driving surface ({int(fsizes.max())} cells), '
      f'discarded {int(stray.sum())} cells in {fn - 1} stray pockets')

dst = src.with_name(src.stem + '_clean.pgm')
write_pgm(dst, img)

meta = yaml.safe_load(open(src.with_suffix('.yaml')))
meta['image'] = dst.name
yaml.safe_dump(meta, open(dst.with_suffix('.yaml'), 'w'), default_flow_style=None)

print(f'\nwall cells {before_wall} -> {int((img == WALL).sum())}')
print(f'wrote {dst}\n      {dst.with_suffix(".yaml")}')
