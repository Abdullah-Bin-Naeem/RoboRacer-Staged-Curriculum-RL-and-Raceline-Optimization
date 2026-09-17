#!/usr/bin/env python3

"""Edit a raceline by hand, on the map, in a browser. Standard library only.

    ./scripts/raceline_editor.py                        # the default line (frames.DEFAULT_RACELINE)
    ./scripts/raceline_editor.py raceline_a7.0_old.csv  # a named line in roboracer_stack/raceline/
    ./scripts/raceline_editor.py --port 8848 --no-browser

Ported from main_track_algorithm (5df3bbe). That branch has a multi-track
registry in common/frames.py; this branch has one track, so the grids are every
georeferenced .pgm beside frames.DEFAULT_MAP_YAML and the lines live in
frames.RACELINE_DIR.

Loads a track's occupancy grid and one raceline CSV, serves a single page, and
writes a NEW CSV when you save. The geometry is refit to a periodic cubic spline
through a few dozen draggable control points; the velocity profile is replanned
from the same forward/backward passes tools/reprofile_raceline.py uses, so what
comes out is comparable with what that tool produces.

WHAT THE PAGE DOES
------------------
    constraints   Velocity limits are refused above the car's physical caps
                  (tyre peaks, top speed), all derived from VEHICLE_GUIDE below,
                  which is the competition guide's vehicle table verbatim. The
                  simulator's linear drag is applied, not optional. A drag may
                  not push curvature past the steering lock or the body
                  footprint into a wall; steering rate caps the profile. A
                  checklist shows every cap against the line, worst point first.
    vehicle       The guide's parameters in a drawer on the left edge.
    precision     Wheel zoom and pan, shift-drag for 10x finer moves, arrow-key
                  nudges down to 1 mm, typed coordinates, normal-only dragging.
    overlays      The line coloured by clearance, speed, accel/brake, lateral,
                  friction-ellipse or steering-rate use; the profile's lower
                  strip plots the same against their limits.
    benchmark     Any CSV in the raceline directory (the TUM lines, the
                  centreline, older edits), or a min-curvature line generated
                  in the page, timed under the SAME limits; split view, a
                  time-delta trace along the lap, and a race replay.
    map specks    Isolated blobs of a few non-free cells are freed by default
                  (track_clean has five; one sits 0.13 m off the start-straight
                  line and read as a wall). Toggle in the side panel.

WHY THIS IS A WEB PAGE
----------------------
The line optimizer (raceline/optimize_raceline.py, on main and multi-track)
needs numpy, scipy, skimage and trajectory_planning_helpers. This host has
Python 3.14 with the standard library, PyYAML and Pillow, and no pip -- so it
cannot run, and neither can matplotlib, tkinter or Qt. A browser is the only
interactive surface actually available here.

So the split is: PYTHON DOES I/O ONLY -- read the PGM, the YAML and the CSV,
hand them over, write the result back and refuse to write a bad one. Every piece
of geometry and physics runs in JavaScript, in raceline_editor.html beside this
file. Nothing is fetched from a network; there is no build step.

WHAT IT REFUSES TO DO
---------------------
Overwrite. Save always writes a new name, because the shipped ladders are
reference artefacts and the line you opened has to stay recoverable.

Write a line the car cannot drive: |kappa| past the steering lock, speed past
the top speed, or lateral/traction/braking demand past the tyre's peak
(physics_problems). Shipped lines are still LOADED when they break these --
several brake harder than the tyre allows -- because you may be opening one to
fix it.

Write a file the car cannot read. Everything the browser sends is re-checked
here -- eight columns, all numeric, s monotonic, ds uniform, the loop closed
without a duplicated seam point -- because the alternative is a CSV that fails
inside rclpy at start-up on the car, where the message is a numpy traceback and
not "your line is malformed".

THE GRID IS SENT RAW, NOT AS AN IMAGE
-------------------------------------
/grid/<layer>.bin is w*h bytes straight out of the PGM. The page needs the
actual occupancy values to ray-cast wall distances, and a PNG round trip through
the browser's colour management is not guaranteed to preserve 205 (unknown)
against 254 (free). Raw bytes are exact, and the page builds its own ImageData
for display -- which also means no PNG encoder here.

SEE ALSO
--------
    tools/reprofile_raceline.py (main_track_algorithm)   the velocity model, ported to JS
    raceline/VEHICLE_MODEL.md (multi-track)       the car, from the simulator source
    roboracer_stack/roboracer_stack/planning/raceline.py   the CSV contract
    roboracer_stack/tools/MAPPING.md              where the grids come from
"""

import argparse
import json
import math
import os
import posixpath
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer

# Where the map and the lines live. frames imports with no ROS and no numpy (its
# ament lookup is already wrapped in try/except), so a host-only tool can reuse
# it rather than re-deriving the paths.
_STACK = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'roboracer_stack')
if _STACK not in sys.path:
    sys.path.insert(0, _STACK)
# On the host there is no ament index, and frames' source-tree fallback on this
# branch resolves one directory too high (<repo>/maps). Point it at the package's
# own maps/ and raceline/ unless the caller already chose.
os.environ.setdefault('RACER_MAPS_DIR', os.path.join(_STACK, 'maps'))
os.environ.setdefault('RACER_RACELINE_DIR', os.path.join(_STACK, 'raceline'))

from roboracer_stack.common import frames  # noqa: E402

# From planning/raceline.py:25. Half the car's width -- what a corner-cutting
# chord may not consume of the free space on the inside of the curve.
HALF_WIDTH = 0.135

# From tools/clean_map.py:20. The three values a saved occupancy grid holds.
UNKNOWN, WALL, FREE = 205, 0, 254

GUIDE_URL = 'https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/'

# The car, as the competition guide publishes it (section 1.3, fetched
# 2026-09-17). Values are copied verbatim; the page shows them in its vehicle
# drawer and derives every physical cap from them, so this table is the ONE
# place a spec change has to be made.
VEHICLE_GUIDE = {
    'car_length': 0.5000,          # m
    'car_width': 0.2700,           # m
    'wheelbase': 0.3240,           # m
    'track_width': 0.2360,         # m
    'front_overhang': 0.0900,      # m, ahead of the front axle
    'rear_overhang': 0.0800,       # m, behind the rear axle
    'wheel_radius': 0.0590,        # m
    'wheel_width': 0.0450,         # m
    'total_mass': 3.906,           # kg
    'sprung_mass': 3.470,          # kg
    'unsprung_mass': 0.436,        # kg
    'com': [0.15532, 0.00000, 0.01434],   # m, from the rear axle centre
    'suspension_spring': 500,      # N/m
    'suspension_damper': 100,      # Ns/m
    'long_tire_extremum': [0.15, 0.72],   # (slip, force coefficient)
    'long_tire_asymptote': [0.25, 0.464],
    'lat_tire_extremum': [0.01, 1.00],
    'lat_tire_asymptote': [0.10, 0.500],
    'drive_type': 'All wheel drive',
    'throttle_limits': [-1, 1],
    'motor_torque': 428,           # Nm
    'top_speed': 22.88,            # m/s
    'steer_type': 'Ackermann steering',
    'steering_limits': [-1, 1],
    'steer_angle_max': 0.5236,     # rad
    'steer_rate': 3.2,             # rad/s
}

# NOT in the guide. Read out of the simulator's Unity source by
# raceline/VEHICLE_MODEL.md (multi-track branch, section 2). Shown separately on
# the page so nobody mistakes them for published figures.
VEHICLE_SOURCE = {
    'linear_drag': 0.273,          # 1/s, Rigidbody.drag: a = -0.273 v
    'angular_drag': 0.1,           # 1/s
    'g': 9.81,                     # m/s^2
    'brake_torque_idle': 428,      # Nm per wheel at throttle exactly 0
    'u_per_throttle': 25.25,       # m/s of wheel surface speed per unit throttle
    'min_long_slip_den': 4.0,      # m/s, PhysX minLongSlipDenominator
}

# Velocity model defaults. Every one of these is a PLANNING choice and sits
# below a physical cap the page derives from VEHICLE_GUIDE (tyre peaks, top
# speed); the page refuses values above the cap. Drag is not a choice: it is
# the simulator's own and is applied as-is.
#   a_lat 7.0   = steer_a_lat_max in pure_pursuit.yaml; tyre peak is 9.81
#   a_accel 6.0 / a_brake 5.0 = the fastest clean ICRA rung (VEHICLE_MODEL.md
#               run 22-23); tyre peak is 0.72 g = 7.06
#   power 0     = off. The 428 Nm motor never limits; the old 20.5 W/kg was a
#               stand-in for the drag that is now modelled directly.
LIMITS = {
    'a_lat': 7.0,
    'a_accel': 6.0,
    'a_brake': 5.0,
    'power': 0.0,
    'drag': VEHICLE_SOURCE['linear_drag'],
    'v_max': 8.5,
    'v_min': 1.0,
}

FOLLOWER_YAML = os.path.join(_STACK, 'config', 'pure_pursuit.yaml')


def follower_params():
    """The few pure_pursuit.yaml values a line has to respect, without PyYAML.

    The follower clips speed at v_max and curvature at steer_a_lat_max/v^2, so a
    line planned beyond either is driven as something else. Keys missing from
    the file fall back to the values the file carried on 2026-09-17.
    """
    out = {'v_max': 8.5, 'steer_a_lat_max': 7.0, 'lookahead_max': 2.2, 'v_min': 1.0}
    try:
        with open(FOLLOWER_YAML) as fh:
            for raw in fh:
                key, _, val = raw.split('#')[0].strip().partition(':')
                if key in out and val.strip():
                    try:
                        out[key] = float(val)
                    except ValueError:
                        pass
    except OSError:
        pass
    return out

# The CSV's own header, byte-for-byte what optimize_raceline.export_csv writes.
HEADER = '# s_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m,v_mps'

# Uniform-ds tolerance for a line we are willing to write. planning/raceline.py
# derives lap_len as s[-1] + (s[1]-s[0]) and pure_pursuit turns that into a
# metres-per-index conversion, so non-uniform spacing mis-places the speed
# target silently. %.5f rounding alone accounts for 1e-5; this leaves room for
# the resampler's own residual without admitting a genuinely ragged line.
DS_TOL = 2e-4


def read_pgm(path):
    """Binary P5 -> (width, height, bytes). The reader from tools/clean_map.py:23,
    without numpy: the payload stays a bytes object and the page indexes it.
    """
    with open(path, 'rb') as fh:
        magic = fh.readline().strip()
        if magic != b'P5':
            raise SystemExit(f'{path}: not a binary PGM (magic {magic!r}, expected P5)')
        line = fh.readline()
        while line.startswith(b'#'):
            line = fh.readline()
        w, h = (int(v) for v in line.split())
        fh.readline()                       # maxval, discarded
        data = fh.read(w * h)
    if len(data) != w * h:
        raise SystemExit(f'{path}: header says {w}x{h} = {w * h} bytes, '
                         f'found {len(data)}. The file is truncated.')
    return w, h, data


def read_map_yaml(path):
    """resolution + origin out of a map YAML, without PyYAML.

    These files are four scalars and a three-element list, written by
    nav2_map_server and by tools/clean_map.py. Parsing them directly keeps this
    tool at zero third-party imports; PyYAML is present on this host but is not
    in the submission image, and there is no reason for this to be the one tool
    that would not run there.
    """
    out = {}
    with open(path) as fh:
        for raw in fh:
            line = raw.split('#')[0].strip()
            if ':' not in line:
                continue
            key, _, val = line.partition(':')
            key, val = key.strip(), val.strip()
            if val.startswith('['):
                out[key] = [float(v) for v in val.strip('[]').split(',')]
            else:
                try:
                    out[key] = float(val)
                except ValueError:
                    out[key] = val
    for need in ('resolution', 'origin'):
        if need not in out:
            raise SystemExit(f'{path}: no {need}: key. Not a map YAML?')
    return out


def read_csv(path):
    """Return (comments, rows) with rows as lists of floats.

    The validation is lifted from tools/reprofile_raceline.py:99-139 and exists
    for the same reason: a ragged or non-numeric file is the signature of two
    writes landing on one path, and catching it here says which line is wrong
    instead of raising inside np.loadtxt at ROS start-up on the car.
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
    width = len(rows[0])
    if width < 7:
        raise SystemExit(f'{path}: expected >= 7 columns (s,x,y,psi,kappa,w_r,w_l), '
                         f'found {width}')
    out = []
    for i, row in enumerate(rows):
        if len(row) != width:
            raise SystemExit(f'{path}: line {i + 1} has {len(row)} columns, expected '
                             f'{width}. The file is corrupt; regenerate it.')
        try:
            out.append([float(v) for v in row])
        except ValueError as exc:
            raise SystemExit(f'{path}: line {i + 1} is not all numbers: {exc}')
    return comments, out


def validate(rows):
    """Everything planning/raceline.py and pure_pursuit.py silently assume.

    Returns a list of complaints; empty means the line is safe to write. Each
    of these is a real failure mode, not a style rule:

      8 columns        the follower reads v_mps at index 7 and falls back to a
                       curvature-derived speed without it
      s monotonic      lap_len and every index<->metre conversion come off s
      ds uniform       pure_pursuit uses lap_len/n as metres-per-index for the
                       lookahead scan and the speed-target preview
      seam not doubled Raceline.a central-differences with wraparound; a
                       zero-length seam segment divides by ~0 and spikes it
    """
    bad = []
    if len(rows[0]) != 8:
        bad.append(f'expected 8 columns, got {len(rows[0])}')
        return bad
    for i, r in enumerate(rows):
        for j, v in enumerate(r):
            if not math.isfinite(v):
                bad.append(f'row {i}, column {j} is not finite ({v})')
                return bad
    s = [r[0] for r in rows]
    if abs(s[0]) > 1e-9:
        bad.append(f's must start at 0, starts at {s[0]:.6f}')
    ds = [s[i + 1] - s[i] for i in range(len(s) - 1)]
    if any(d <= 0.0 for d in ds):
        bad.append('s must increase monotonically')
    else:
        spread = max(ds) - min(ds)
        if spread > DS_TOL:
            bad.append(f'ds is not uniform: {min(ds):.6f}..{max(ds):.6f} '
                       f'(spread {spread:.6f} > {DS_TOL})')
    seam = math.hypot(rows[0][1] - rows[-1][1], rows[0][2] - rows[-1][2])
    if ds and seam < 0.5 * min(ds):
        bad.append(f'the last point duplicates the first (seam gap {seam:.6f} m); '
                   'the loop must close with one ds to spare, not by repeating a point')
    return bad


def physics_problems(rows, tol=0.03):
    """What the car cannot do at all, whatever the planning limits say.

    Checked on save only: shipped lines are loaded regardless (several brake
    harder than the tyre allows), and the page re-plans before it saves, so a
    line reaching here in violation means the page's caps were bypassed. `tol`
    absorbs the %.5f rounding and finite differences, never real excess.
    """
    g = VEHICLE_SOURCE['g']
    drag = VEHICLE_SOURCE['linear_drag']
    k_lock = math.tan(VEHICLE_GUIDE['steer_angle_max']) / VEHICLE_GUIDE['wheelbase']
    ax_peak = VEHICLE_GUIDE['long_tire_extremum'][1] * g
    ay_peak = VEHICLE_GUIDE['lat_tire_extremum'][1] * g
    v_top = VEHICLE_GUIDE['top_speed']
    n = len(rows)
    ds = rows[1][0] - rows[0][0]
    worst = {}

    def note(key, i, val):
        if key not in worst or val > worst[key][1]:
            worst[key] = (i, val)

    for i, r in enumerate(rows):
        j = rows[(i + 1) % n]
        k, v, vj = abs(r[4]), r[7], j[7]
        if k > k_lock * (1 + tol):
            note('kappa', i, k)
        if v < 0 or v > v_top:
            note('speed', i, v)
        if v * v * k > ay_peak * (1 + tol):
            note('lateral', i, v * v * k)
        ax = (vj * vj - v * v) / (2 * ds)
        if ax > 0 and ax + drag * v > ax_peak * (1 + tol):
            note('traction', i, ax + drag * v)
        if ax < 0 and -ax - drag * vj > ax_peak * (1 + tol):
            note('braking', i, -ax - drag * vj)
    text = {
        'kappa': lambda i, x: f'|kappa| {x:.3f} 1/m at s {rows[i][0]:.2f} is past the '
                              f'steering lock {k_lock:.3f} 1/m',
        'speed': lambda i, x: f'v {x:.2f} m/s at s {rows[i][0]:.2f} is outside [0, {v_top}]',
        'lateral': lambda i, x: f'lateral {x:.2f} m/s^2 at s {rows[i][0]:.2f} exceeds the '
                                f'tyre peak {ay_peak:.2f}',
        'traction': lambda i, x: f'traction {x:.2f} m/s^2 at s {rows[i][0]:.2f} exceeds the '
                                 f'tyre peak {ax_peak:.2f}',
        'braking': lambda i, x: f'braking {x:.2f} m/s^2 (tyre share) at s {rows[i][0]:.2f} '
                                f'exceeds the tyre peak {ax_peak:.2f}',
    }
    return [text[key](*worst[key]) for key in text if key in worst]


def write_csv(path, rows, comments):
    """Header + %.5f, matching optimize_raceline.export_csv exactly.

    Written to a temporary file beside the target and renamed into place, which
    on POSIX is atomic. A raceline is read by np.loadtxt, which needs every row
    to be the same width, so a half-written file is not a partial line -- it is
    the ragged file whose diagnostic reprofile_raceline.read_csv:127 attributes
    to "concurrent writes to one path". Renaming means a reader sees the old
    file or the new one and never the middle of a write.
    """
    tmp = path + '.partial'
    try:
        with open(tmp, 'w') as fh:
            for line in comments:
                fh.write(line + '\n')
            for r in rows:
                fh.write(','.join(f'{v:.5f}' for v in r) + '\n')
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


# This branch races one track; the name is only a label on the page.
TRACK = 'iros-compete'


def track_layers():
    """Every grid beside the default map that can actually be placed in the world.

    A .pgm with no .yaml has no resolution and no origin, so it cannot be
    georeferenced and is not offered. Returns [{name, pgm, yaml}], the default
    grid (frames.DEFAULT_MAP_YAML) first, since that is the one AMCL localizes
    against.
    """
    default = os.path.basename(frames.DEFAULT_MAP_YAML)[:-len('.yaml')]
    d = os.path.dirname(frames.DEFAULT_MAP_YAML)
    found = []
    for f in sorted(os.listdir(d)):
        if not f.endswith('.pgm'):
            continue
        name = f[:-len('.pgm')]
        yml = os.path.join(d, name + '.yaml')
        if os.path.isfile(yml):
            found.append({'name': name, 'pgm': os.path.join(d, f), 'yaml': yml})
    if not found:
        raise SystemExit(f'{d}: no .pgm with a matching .yaml')
    found.sort(key=lambda layer: layer['name'] != default)
    return found


def raceline_path(name):
    """A bare filename in the raceline directory, refusing anything else."""
    if os.path.basename(name) != name:
        raise SystemExit(f'{name!r}: give a bare filename in {frames.RACELINE_DIR}')
    path = os.path.join(frames.RACELINE_DIR, name)
    if not os.path.isfile(path):
        have = sorted(f for f in os.listdir(frames.RACELINE_DIR) if f.endswith('.csv'))
        raise SystemExit(f'{path} does not exist. Lines here: {have}')
    return path


class Session:
    """Everything the page needs, resolved once at start-up.

    Held in memory rather than re-read per request so that editing a CSV on
    disk underneath a live session cannot half-swap the line the page is
    working on.
    """

    def __init__(self, line_name):
        self.track = TRACK
        self.layers = track_layers()
        self.dir = frames.RACELINE_DIR
        self.grids = {}
        self.meta = []
        for layer in self.layers:
            w, h, data = read_pgm(layer['pgm'])
            info = read_map_yaml(layer['yaml'])
            self.grids[layer['name']] = data
            self.meta.append({
                'name': layer['name'], 'w': w, 'h': h,
                'res': info['resolution'],
                'origin': info['origin'][:2],
            })
        self.path = raceline_path(line_name or os.path.basename(frames.DEFAULT_RACELINE))
        self.comments, self.rows = read_csv(self.path)
        problems = validate(self.rows)
        if problems:
            # Loading is not blocked -- you may well be opening a line BECAUSE
            # it is wrong -- but saying so up front beats discovering it in the
            # save dialog after twenty minutes of dragging.
            print(f'[editor] NOTE {os.path.basename(self.path)} does not satisfy '
                  'every assumption the follower makes:')
            for p in problems:
                print(f'[editor]      {p}')
            print('[editor]      Saving re-derives all of these, so a round trip '
                  'through this editor will fix them.')

    def payload(self):
        return {
            'track': self.track,
            'layers': self.meta,
            'line': {
                'name': os.path.basename(self.path),
                'path': self.path,
                'rows': self.rows,
            },
            'dir': self.dir,
            'existing': sorted(f for f in os.listdir(self.dir) if f.endswith('.csv')),
            'limits': LIMITS,
            'vehicle': VEHICLE_GUIDE,
            'vehicle_source': VEHICLE_SOURCE,
            'guide_url': GUIDE_URL,
            'follower': follower_params(),
            'half_width': HALF_WIDTH,
            'free': FREE,
            'header': HEADER,
        }

    def other_line(self, name):
        """Any line in the raceline directory, for comparison. Read fresh each
        time -- unlike the line being edited, a reference is allowed to change
        on disk between two looks. Returns (ok, rows-or-message)."""
        if os.path.basename(name) != name or not name.endswith('.csv'):
            return False, f'{name!r}: a bare .csv filename in {self.dir}'
        path = os.path.join(self.dir, name)
        if not os.path.isfile(path):
            return False, f'{name} does not exist in {self.dir}'
        try:
            _, rows = read_csv(path)
        except SystemExit as exc:          # read_csv reports by exiting; not here
            return False, str(exc)
        return True, rows

    def save(self, name, rows):
        """Write a new line. Returns (ok, message)."""
        if not name.endswith('.csv'):
            name += '.csv'
        if os.path.basename(name) != name:
            return False, f'{name!r} must be a bare filename, not a path'
        dest = os.path.join(self.dir, name)
        if os.path.exists(dest):
            return False, (f'{name} already exists in {self.dir}. Saving never '
                           'overwrites -- the shipped ladders are reference '
                           'artefacts. Choose another name.')
        problems = validate(rows)
        if problems:
            # The page derives all of this itself, so reaching here means the
            # two implementations disagree. Refusing is the only safe answer:
            # the alternative is a file that raises inside rclpy on the car.
            return False, 'refusing to write a line the follower cannot use: ' \
                          + '; '.join(problems)
        problems = physics_problems(rows)
        if problems:
            return False, 'refusing to write a line the car cannot drive: ' \
                          + '; '.join(problems)
        write_csv(dest, rows, [HEADER])
        return True, dest


class Handler(BaseHTTPRequestHandler):
    session = None
    page = None

    def log_message(self, fmt, *args):       # noqa: A003 - BaseHTTPRequestHandler API
        """Silence the per-request access log; this is a single-user tool."""

    def _send(self, code, body, ctype):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        # The page is regenerated on every start; a cached grid or session would
        # silently show the previous line.
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):                        # noqa: N802 - BaseHTTPRequestHandler API
        path = posixpath.normpath(self.path.split('?')[0])
        if path in ('/', '/index.html'):
            return self._send(200, self.page, 'text/html; charset=utf-8')
        if path == '/session.json':
            return self._send(200, json.dumps(self.session.payload()),
                              'application/json')
        if path.startswith('/line/'):
            ok, out = self.session.other_line(path[len('/line/'):])
            return self._send(200 if ok else 400,
                              json.dumps({'ok': ok, 'rows': out} if ok
                                         else {'ok': False, 'message': out}),
                              'application/json')
        if path.startswith('/grid/') and path.endswith('.bin'):
            name = path[len('/grid/'):-len('.bin')]
            grid = self.session.grids.get(name)
            if grid is None:
                return self._send(404, f'no such layer: {name}', 'text/plain')
            return self._send(200, grid, 'application/octet-stream')
        return self._send(404, 'not found', 'text/plain')

    def do_POST(self):                       # noqa: N802 - BaseHTTPRequestHandler API
        if posixpath.normpath(self.path.split('?')[0]) != '/save':
            return self._send(404, 'not found', 'text/plain')
        try:
            n = int(self.headers.get('Content-Length', 0))
            req = json.loads(self.rfile.read(n) or b'{}')
            ok, msg = self.session.save(str(req.get('name', '')),
                                        [[float(v) for v in r] for r in req['rows']])
        except Exception as exc:             # noqa: BLE001 - report, never crash the server
            ok, msg = False, f'{type(exc).__name__}: {exc}'
        if ok:
            print(f'[editor] wrote {msg}')
        return self._send(200 if ok else 400,
                          json.dumps({'ok': ok, 'message': msg}), 'application/json')


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n')[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('raceline', nargs='?', default=None,
                    help='line to open, by bare filename within '
                         f'{frames.RACELINE_DIR}; omit for the default line')
    ap.add_argument('--port', type=int, default=8848, help='port to serve on')
    ap.add_argument('--no-browser', action='store_true',
                    help='do not open a browser; just print the URL')
    args = ap.parse_args(argv)


    here = os.path.dirname(os.path.abspath(__file__))
    page_path = os.path.join(here, 'raceline_editor.html')
    if not os.path.isfile(page_path):
        raise SystemExit(f'{page_path} is missing; it ships beside this script')

    Handler.session = Session(args.raceline)
    Handler.page = open(page_path).read()

    n = len(Handler.session.rows)
    lap = Handler.session.rows[-1][0] + Handler.session.rows[1][0]
    print(f'[editor] track {TRACK}, {len(Handler.session.layers)} grid layer(s): '
          f'{", ".join(m["name"] for m in Handler.session.meta)}')
    print(f'[editor] {os.path.basename(Handler.session.path)}: {n} points, {lap:.2f} m lap')

    try:
        srv = HTTPServer(('127.0.0.1', args.port), Handler)
    except OSError as exc:
        raise SystemExit(f'cannot serve on port {args.port}: {exc}. '
                         'Another editor may already be running; try --port.')
    url = f'http://127.0.0.1:{args.port}/'
    print(f'[editor] {url}   (ctrl-c to stop)')
    if not args.no_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n[editor] stopped')
    return 0


if __name__ == '__main__':
    sys.exit(main())
