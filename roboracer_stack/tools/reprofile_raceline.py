#!/usr/bin/env python3

"""Re-plan the velocity profile of an existing raceline CSV. Standard library only.

The GEOMETRY of a raceline (columns 0-6: s, x, y, psi, kappa, w_right, w_left)
comes from the minimum-curvature solve in `raceline/optimize_raceline.py` on the
main branch. This tool does not touch it -- columns 0-6 are copied through
byte-for-byte -- and rewrites only column 7, `v_mps`, the speed profile the
follower tracks when `use_path_speed` is true.

WHY THIS EXISTS
---------------
The shipped profiles are conservative in ONE specific place. Measured off
raceline_a7.0.csv:

    lateral limit         7.00 m/s^2      <- at the tyre's peak (mu 0.72 * g = 7.06)
    braking limit         ~7.0 m/s^2      <- also at the peak
    acceleration limit    3.93 m/s^2      <- HALF of what the tyre can take
    power limit           ~20.5 W/kg      <- binds above 5.1 m/s

Cornering and braking are already at the friction limit, so there is nothing to
win there; the tyre model in pure_pursuit.py (`_mu`, TIRE_MU_PEAK 0.72) says
6.3-7.0 m/s^2 is available longitudinally at slips 0.08-0.15, and the profile
only ever asks for 3.93. Raising `--a-accel` spends that headroom on corner
exits, which is what carries onto the straights.

The power limit is the real ceiling above ~5 m/s and is a property of the
drivetrain, not the plan: at 6.8 m/s the profile's own acceleration is
20.5/6.8 = 3.0 m/s^2 no matter what `--a-accel` says. Raise `--power` only if
you have measured the car pulling harder than that at speed -- unlike a_accel
there is no evidence in the repo that the headroom exists.

THE MODEL
---------
Forward-backward passes over the closed loop, with a friction circle shared
between lateral and longitudinal demand:

    v_curve = sqrt(a_lat / |kappa|)                       cornering limit
    circle  = sqrt(1 - (v^2 |kappa| / a_lat)^2)           grip left over for a_x
    accel   = min(a_accel * circle, power / v)            forward pass
    brake   = a_brake * circle                            backward pass

The defaults ARE the limits recovered from raceline_a7.0.csv above, so running
with no flags re-plans that profile onto itself: rms 0.03 m/s, worst 0.09 m/s at
the lap seam, where this closes the loop slightly more conservatively than the
original did. A run with one flag changed therefore isolates that one flag; the
residual is well under the deltas worth driving.

WHAT THIS DOES NOT MODEL
------------------------
Aerodynamic/rolling drag (the follower's observer uses 0.273*v, which at 7.3 m/s
is 2.0 m/s^2 of decel the forward pass here ignores) and any transient in the
tyre's build-up of slip. Both make the real car slightly slower than the plan on
the straights and slightly better under braking. The follower's slip band
(`slip_accel`, `slip_brake` in pure_pursuit.yaml, 0.08 = 6.35 m/s^2) is a
SEPARATE saturation and will clip whatever this plans beyond it -- raise it to
0.10-0.12 alongside a big `--a-accel`, and never past 0.15, where the friction
curve turns over and more slip means less force.

USAGE
-----
    # what the input profile actually assumes, no file written
    ./reprofile_raceline.py raceline_a7.0.csv

    # a little more acceleration, everything else held
    ./reprofile_raceline.py raceline_a7.0.csv raceline_a7.0_acc5.csv --a-accel 5.0

Then drive it:

    ros2 launch roboracer_stack race.launch.py \
        path_csv:=<pkg>/raceline/raceline_a7.0_acc5.csv

KEEP --a-lat AT THE INPUT'S VALUE unless you mean to change corner speeds. The
follower reads `profile_a_lat` back out of the CSV as max(v^2 |kappa|) and the
delay derate (derate_a_lat, pure_pursuit.yaml) scales targets by
sqrt(a_lat_eff / profile_a_lat) -- so a profile planned at a higher a_lat is
derated harder on a slow loop, and the two changes partly cancel in a way that
is very hard to read from a lap time.
"""

import argparse
import math
import sys

# Defaults recovered from raceline_a7.0.csv; see THE MODEL above.
A_LAT = 7.0        # m/s^2, lateral. Tyre peak is mu 0.72 * g = 7.06.
A_ACCEL = 4.0      # m/s^2, longitudinal, at zero lateral demand
A_BRAKE = 7.0      # m/s^2, longitudinal, at zero lateral demand
POWER = 20.5       # W/kg, i.e. a_x * v. Binds above a_accel/power = 5.1 m/s.
                   # NOT the sim's physics: see --drag. Kept as the default only
                   # because it is what raceline_a7.0.csv was planned with.
DRAG = 0.0         # 1/s. 0.273 is the simulator's own Rigidbody linear drag.
V_MAX = 8.0        # m/s. Matches v_max in pure_pursuit.yaml.
V_MIN = 1.0        # m/s. Matches v_min in pure_pursuit.yaml.

PASSES = 4         # forward+backward sweeps; the loop closes in 2 on this track


def read_csv(path):
    """Return (comments, rows) where each row is the list of column STRINGS.

    The numeric columns are kept as text so the geometry can be written back
    exactly as it came in -- reformatting a float through Python would silently
    perturb a line that took an optimizer to produce.
    """
    comments, rows = [], []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            if line.startswith('#'):
                comments.append(line)
            else:
                rows.append(line.split(','))
    if len(rows) < 3:
        raise SystemExit(f'{path}: need at least 3 points, found {len(rows)}')
    if len(rows[0]) < 7:
        raise SystemExit(f'{path}: expected >= 7 columns (s,x,y,psi,kappa,w_r,w_l), '
                         f'found {len(rows[0])}')
    # Every row must have the same width, and every field must be a number.
    # np.loadtxt in the follower's Raceline enforces both, but it raises at
    # ROS start-up on the car; catching it here says which line is wrong. A
    # ragged file is the signature of two writes landing on one path -- run the
    # tool twice into the same output and this is what you get.
    width = len(rows[0])
    for i, row in enumerate(rows):
        if len(row) != width:
            raise SystemExit(f'{path}: line {i + 1} has {len(row)} columns, '
                             f'expected {width}. The file is corrupt (concurrent '
                             f'writes to one path?); regenerate it.')
        for j, field in enumerate(row):
            try:
                float(field)
            except ValueError:
                raise SystemExit(f'{path}: line {i + 1}, column {j} is not a '
                                 f'number: {field!r}. The file is corrupt; '
                                 f'regenerate it.')
    return comments, rows


def arc_steps(s):
    """Per-point distance to the NEXT point, closing the loop at the seam.

    Taken from the s column, which is the arc length the optimizer measured
    along the spline; the seam segment (last point back to the first) has no s
    to difference, so it gets the mean of the others. The line is sampled
    uniformly, so that is exact to the width of the rounding in the file.
    """
    n = len(s)
    ds = [s[i + 1] - s[i] for i in range(n - 1)]
    mean = sum(ds) / len(ds)
    if any(d <= 0.0 for d in ds):
        raise SystemExit('the s column must increase monotonically')
    return ds + [mean]


def plan(kappa, ds, a_lat, a_accel, a_brake, power, drag, v_max, v_min):
    """Forward-backward velocity profile on a closed loop.

    Both passes step with v^2 = v0^2 + 2 a ds, the closed form of constant
    acceleration over a segment, which is exact for the piecewise-constant
    limit and avoids the timestep a numerical integration would need.

    The available longitudinal acceleration is evaluated at the point the
    segment STARTS from (forward) or ENDS at (backward), i.e. always at the
    speed and curvature where the grip is actually being spent.

    Drag is a force the car always fights, so it comes OFF the forward pass and
    is ADDED to the backward one -- it shortens braking distances as much as it
    flattens acceleration.
    """
    n = len(kappa)

    def circle(v, k):
        """Fraction of the friction limit left for longitudinal use at (v, k)."""
        used = v * v * abs(k) / a_lat
        # used > 1 means the point is already over the cornering limit, which
        # the seed below prevents; clamped rather than trusted.
        return math.sqrt(max(0.0, 1.0 - min(used, 1.0) ** 2))

    # Seed: the cornering limit at every point, which no pass may ever raise.
    v = [min(v_max, math.sqrt(a_lat / max(abs(k), 1e-6))) for k in kappa]
    v = [max(x, v_min) for x in v]

    for _ in range(PASSES):
        before = list(v)
        # Forward: how fast can the car BE here, having accelerated to it.
        for i in range(n):
            j = (i + 1) % n
            a = a_accel * circle(v[i], kappa[i])
            if power > 0.0:
                a = min(a, power / max(v[i], 1e-3))
            a -= drag * v[i]
            v[j] = min(v[j], math.sqrt(max(0.0, v[i] ** 2 + 2.0 * a * ds[i])))
        # Backward: how fast may the car be here and still stop for what's next.
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            a = a_brake * circle(v[j], kappa[j]) + drag * v[j]
            v[i] = min(v[i], math.sqrt(max(0.0, v[j] ** 2 + 2.0 * a * ds[i])))
        v = [max(x, v_min) for x in v]
        if max(abs(a - b) for a, b in zip(v, before)) < 1e-9:
            break
    return v


def stats(v, kappa, ds):
    """What a profile actually demands -- the numbers the docstring quotes."""
    n = len(v)
    lap = sum(ds)
    a_long = [(v[(i + 1) % n] ** 2 - v[i] ** 2) / (2.0 * ds[i]) for i in range(n)]
    # Time over each segment at the mean of its endpoint speeds. On a profile
    # sampled every 7 cm the error against the exact constant-a integral is
    # under a millisecond a lap.
    t = sum(ds[i] / max(0.5 * (v[i] + v[(i + 1) % n]), 1e-3) for i in range(n))
    return {
        'v_min': min(v), 'v_max': max(v), 'v_mean': sum(v) / n,
        'a_lat': max(v[i] ** 2 * abs(kappa[i]) for i in range(n)),
        'a_accel': max(a_long), 'a_brake': min(a_long),
        'power': max(a_long[i] * v[i] for i in range(n)),
        'lap': lap, 'time': t,
    }


def show(label, st):
    print(f'  {label:<10} '
          f'v {st["v_min"]:.2f}-{st["v_max"]:.2f} (mean {st["v_mean"]:.2f}) m/s   '
          f'a_lat {st["a_lat"]:.2f}   accel {st["a_accel"]:.2f}   '
          f'brake {st["a_brake"]:.2f} m/s^2   power {st["power"]:.1f} W/kg   '
          f'lap {st["time"]:.3f} s')


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Re-plan the v_mps column of a raceline CSV; geometry untouched.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('input', help='raceline CSV to read')
    ap.add_argument('output', nargs='?',
                    help='CSV to write; omit to only report on the input')
    ap.add_argument('--a-accel', type=float, default=A_ACCEL,
                    help='longitudinal acceleration limit at zero lateral demand '
                         '[m/s^2]. THE ONE TO TURN. The tyre offers 6.3-7.0')
    ap.add_argument('--a-lat', type=float, default=A_LAT,
                    help='lateral (cornering) limit [m/s^2]; sets corner speeds. '
                         'Changing it also moves the follower delay derate')
    ap.add_argument('--a-brake', type=float, default=A_BRAKE,
                    help='braking limit at zero lateral demand [m/s^2]')
    ap.add_argument('--power', type=float, default=POWER,
                    help='power/mass limit [W/kg], i.e. a_x * v; 0 disables it. This '
                         'is NOT in the simulator, which limits force by tyre slip '
                         'alone -- pass 0 with --drag 0.273 for the real model')
    ap.add_argument('--drag', type=float, default=DRAG,
                    help="linear drag [1/s], subtracted as drag*v from acceleration "
                         "and added to braking. The sim's Rigidbody drag is 0.273 "
                         "(DRAG_LIN in pure_pursuit.py)")
    ap.add_argument('--v-max', type=float, default=V_MAX,
                    help='speed cap [m/s]; keep <= v_max in pure_pursuit.yaml')
    ap.add_argument('--v-min', type=float, default=V_MIN, help='speed floor [m/s]')
    args = ap.parse_args(argv)

    for name in ('a_accel', 'a_lat', 'a_brake', 'v_max'):
        if getattr(args, name) <= 0.0:
            raise SystemExit(f'--{name.replace("_", "-")} must be positive')
    if args.power < 0.0 or args.drag < 0.0:
        raise SystemExit('--power and --drag must not be negative')
    if args.v_min >= args.v_max:
        raise SystemExit('--v-min must be below --v-max')

    comments, rows = read_csv(args.input)
    s = [float(r[0]) for r in rows]
    kappa = [float(r[4]) for r in rows]
    ds = arc_steps(s)
    has_v = len(rows[0]) > 7

    print(f'{args.input}: {len(rows)} points, {sum(ds):.2f} m lap')
    if has_v:
        show('input', stats([float(r[7]) for r in rows], kappa, ds))

    v = plan(kappa, ds, args.a_lat, args.a_accel, args.a_brake,
             args.power, args.drag, args.v_max, args.v_min)
    st = stats(v, kappa, ds)
    show('planned', st)

    if has_v:
        v_in = [float(r[7]) for r in rows]
        dev = max(abs(a - b) for a, b in zip(v, v_in))
        t_in = stats(v_in, kappa, ds)['time']
        print(f'  vs input: max |dv| {dev:.3f} m/s, '
              f'lap {st["time"] - t_in:+.3f} s ({(st["time"] / t_in - 1.0) * 100:+.1f} %)')
        print('  NOTE: the planned lap time is what the PLAN takes, not what the car '
              'drives.\n        Drag and tyre transients are not modelled; treat the '
              'delta as the\n        signal and the absolute number as optimistic.')

    if not args.output:
        print('\nNo output path given, nothing written.')
        return 0

    if args.output == args.input:
        raise SystemExit('refusing to overwrite the input; give a new output path')

    # Columns 0-6 are copied as the strings they came in as; only v is rewritten,
    # in the same %.5f the exporter uses.
    with open(args.output, 'w') as fh:
        for line in comments:
            fh.write(line + '\n')
        for row, vi in zip(rows, v):
            fh.write(','.join(row[:7]) + f',{vi:.5f}\n')
    print(f'\nwrote {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
