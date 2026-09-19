#!/usr/bin/env python3
"""Recovery-prior arithmetic, against the real centreline and the real resets.

No ROS and no simulator: it imports the node module and calls the same helpers
the node calls, which is the whole point. An earlier version of this check
re-implemented the arithmetic instead, and took the lap length from the
un-reversed file, so it passed while the node itself divided by zero on the
first reset of every run (IROS 2026 run 20: 9 resets, none recovered, the node
dead after the first).

    source ros_env.sh
    python3 devkit_ws/src/racer_localization/tools/check_recovery_prior.py
"""
import math
import os
import sys

import numpy as np

from racer_common import frames
from racer_localization.localization_bootstrap import pick_back

# Every contact seen in a logged run, and the checkpoint the simulator actually
# reset the car to. Read off runs 14, 19 and 20.
CASES = [
    ('iros2026', (1.11, -14.66), (0.718, -15.575)),   # run 14, hairpin 1 apex
    ('iros2026', (2.38, -15.64), (1.713, -15.770)),   # run 14, hairpin 1 exit
    ('iros2026', (1.05, -14.52), (0.716, -15.574)),   # run 20, first of the cascade
    ('iros2026', (3.70, -12.26), (3.018, -13.624)),   # run 20, after hairpin 1
    ('iros2026', (5.52, -11.49), (5.047, -11.480)),   # run 20, after the chicane
    ('iros2026', (0.52, 3.89), (0.800, 3.653)),       # run 19, hairpin 2 exit
    # docker v2 runs, 2026-09-19 (contact = last moving ground-truth pose)
    ('iros2026', (3.71, -9.34), (3.728, -9.942)),     # lv_fast_19_1, launch-corner exit
    ('iros2026', (3.92, 2.08), (3.323, 2.124)),       # lv_margin_2, before hairpin 2 (new checkpoint)
    # AMBIGUOUS: 9 cm from run 19's contact above, but the simulator reset it to
    # the checkpoint 1.1 m behind instead of the one 0.3 m ahead (its trigger
    # fires when the body enters). The picker cannot tell them apart; the
    # bootstrap's recover_offprior_m accepts the localizer where it settles.
    ('iros2026', (0.53, 3.98), [(0.800, 3.653), (0.884, 4.715)]),   # lv_L750_warm
    ('iros2026', (2.35, -15.20), (1.713, -15.770)),   # lv_fast_19_1, hairpin 1 exit after recovery
]


def oriented_centreline(track):
    """What the node's _load_centreline produces, by the same steps."""
    path = os.path.join(frames.raceline_dir(track), 'centerline_full.csv')
    c = np.genfromtxt(path, delimiter=',', comments='#')
    s, x, y = c[:, 0], c[:, 1], c[:, 2]
    sx, sy, syaw = (float(v) for v in frames.spawn(track))
    j = int(np.argmin((x - sx) ** 2 + (y - sy) ** 2))
    k = (j + 3) % len(s)
    if math.cos(math.atan2(y[k] - y[j], x[k] - x[j]) - syaw) < 0.0:
        s = (s[-1] - s)[::-1]
        x, y = x[::-1], y[::-1]
    return s, x, y


def main():
    fails = []
    for track, contact, expect in CASES:
        s, x, y = oriented_centreline(track)
        lap = float(np.max(s))
        cps = frames.checkpoints(track)
        if not cps:
            print(f'skip {track}: no checkpoints registered')
            continue

        def s_of(px, py):
            return float(s[int(np.argmin((x - px) ** 2 + (y - py) ** 2))])

        s_last, best = s_of(*contact), None
        for cx, cy, _ in cps:
            back = pick_back(s_last, s_of(cx, cy), lap)
            if back is None:
                fails.append((contact, 'lap length unusable -- the node would crash', expect))
                break
            if back <= 25.0 and (best is None or back < best[0]):
                best = (back, cx, cy)
        else:
            # Within 5 cm: the same physical checkpoint is logged to the
            # millimetre by different runs, and the registry keeps one reading.
            got = (round(best[1], 3), round(best[2], 3)) if best else None
            alts = expect if isinstance(expect, list) else [expect]
            ok = best is not None and any(math.hypot(best[1] - e[0], best[2] - e[1]) < 0.05 for e in alts)
            print(f'{"ok  " if ok else "FAIL"} contact {contact} -> {got}, '
                  f'{best[0]:+.1f} m behind   (expected {expect})')
            if not ok:
                fails.append((contact, got, expect))

    # A lap length of zero is what killed the node, so assert the invariant
    # that prevents it, on every track that has a centreline.
    for track in sorted(frames.TRACKS):
        try:
            s, _, _ = oriented_centreline(track)
        except OSError:
            print(f'skip {track}: no centerline_full.csv')
            continue
        lap, asc = float(np.max(s)), bool(np.all(np.diff(s) >= 0))
        ok = lap > 1.0 and asc
        print(f'{"ok  " if ok else "FAIL"} {track}: lap {lap:.2f} m, s ascending {asc}')
        if not ok:
            fails.append((track, lap, asc))

    ok = pick_back(5.0, 1.0, 0.0) is None
    print(f'{"ok  " if ok else "FAIL"} pick_back guards a zero lap length')
    if not ok:
        fails.append(('zero lap', 'no guard', None))

    print('\nFAILED' if fails else '\nall passed')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
