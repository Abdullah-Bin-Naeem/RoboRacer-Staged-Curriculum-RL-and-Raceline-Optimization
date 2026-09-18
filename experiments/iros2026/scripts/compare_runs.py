#!/usr/bin/env python3
"""Lap stats + straight-section speed trace for one or more runs.

    python3 compare_runs.py <name>=<launch.log>[:<telemetry.csv>] ...

Lap times come from the follower's own 'lap N: T s' lines. The straight block
reports what the car actually did over s=0..13 m, which is where a change to the
profile's longitudinal limit shows up first.
"""
import csv, re, sys, statistics as st
import numpy as np

STRAIGHT = (0.5, 12.5)


def laps(log):
    out = []
    for line in open(log, errors='ignore'):
        m = re.search(r'\[pure_pursuit\].*?lap (\d+): ([0-9.]+) s', line)
        if m and float(m.group(2)) < 30:
            out.append((int(m.group(1)), float(m.group(2))))
    return out


def events(log):
    txt = open(log, errors='ignore').read()
    return (len(re.findall(r'RESET detected', txt)),
            len(re.findall(r'recovery #', txt)))


def straight(csv_path):
    rows = list(csv.DictReader(open(csv_path)))
    c = lambda n: np.array([float(r[n]) for r in rows])
    ok = c('ready') > 0.5
    s, vt, ve = c('pp_s')[ok], c('pp_v_target')[ok], c('pp_v_est')[ok]
    vtrue, thr = c('speed')[ok], c('pp_throttle')[ok]
    # Gate out warmup (v_max capped to 2.0), creep and standing samples: the
    # profile asks >= 3.2 m/s everywhere in the straight window, so a target
    # below 3.0 there means the car is not at race pace.
    m = (s >= STRAIGHT[0]) & (s <= STRAIGHT[1]) & (vt >= 3.0) & (vtrue > 1.0)
    if m.sum() < 50:
        return None
    hit = s[m][vtrue[m] >= 8.0]
    return dict(v_peak=vtrue[m].max(), lag=(vt - ve)[m].mean(),
                s_at_8=hit.min() if len(hit) else float('nan'),
                thr=thr[m].mean(), vt_mean=vt[m].mean(), vtrue_mean=vtrue[m].mean())


def main():
    print(f"{'run':>10} {'laps':>5} {'rst':>4} {'best':>7} {'med':>7} {'mean':>7} {'worst':>7} {'std':>6}")
    tele = {}
    for arg in sys.argv[1:]:
        name, _, src = arg.partition('=')
        log, _, tel = src.partition(':')
        L = laps(log)
        if not L:
            print(f"{name:>10}    no laps yet"); continue
        t = [x for _, x in L]
        r, _ = events(log)
        print(f"{name:>10} {len(t):5d} {r:4d} {min(t):7.3f} {st.median(t):7.3f} "
              f"{st.mean(t):7.3f} {max(t):7.3f} {st.pstdev(t):6.3f}")
        if tel:
            try: tele[name] = straight(tel)
            except Exception as e: print(f"  ({name} telemetry: {e})")
    if tele:
        print(f"\nSTRAIGHT s={STRAIGHT[0]}-{STRAIGHT[1]} m")
        print(f"{'run':>10} {'v_peak':>7} {'v_mean':>7} {'tgt_mean':>9} {'lag':>6} {'s@8.0':>7} {'thr':>6}")
        for n, d in tele.items():
            if d is None: print(f"{n:>10}   (no straight data)"); continue
            print(f"{n:>10} {d['v_peak']:7.2f} {d['vtrue_mean']:7.2f} {d['vt_mean']:9.2f} "
                  f"{d['lag']:6.2f} {d['s_at_8']:7.2f} {d['thr']:6.3f}")


if __name__ == '__main__':
    main()
