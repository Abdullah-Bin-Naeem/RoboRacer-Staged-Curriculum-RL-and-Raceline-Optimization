#!/usr/bin/env python3

"""Edit a raceline by hand, on the map, in a browser. Standard library only.

    ./scripts/raceline_editor.py                        # RACER_TRACK's default line
    ./scripts/raceline_editor.py raceline_a4.0.csv      # a named line on that track
    RACER_TRACK=porto ./scripts/raceline_editor.py raceline_a6.5.csv
    ./scripts/raceline_editor.py --port 8848 --no-browser

Loads a track's occupancy grid and one raceline CSV, serves a single page, and
writes a NEW CSV when you save. The geometry is refit to a periodic cubic spline
through a few dozen draggable control points; the velocity profile is replanned
from the same forward/backward passes tools/reprofile_raceline.py uses, so what
comes out is comparable with what that tool produces.

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
    roboracer_stack/tools/reprofile_raceline.py   the velocity model, ported to JS
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

# The track registry: which map, which spawn, which lines. It imports with no
# ROS and no numpy (its ament lookup is already wrapped in try/except), so a
# host-only tool can reuse it rather than re-deriving where anything lives.
_STACK = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'roboracer_stack')
if _STACK not in sys.path:
    sys.path.insert(0, _STACK)

from roboracer_stack.common import frames  # noqa: E402

# From planning/raceline.py:25. Half the car's width -- what a corner-cutting
# chord may not consume of the free space on the inside of the curve.
HALF_WIDTH = 0.135

# From tools/clean_map.py:20. The three values a saved occupancy grid holds.
UNKNOWN, WALL, FREE = 205, 0, 254

# Velocity model defaults, from tools/reprofile_raceline.py:86-96. Kept in the
# same units and under the same names so a profile planned here and one planned
# by that tool are the same object.
LIMITS = {
    'a_lat': 7.0,      # m/s^2 lateral. Tyre peak is mu 0.72 * g = 7.06.
    'a_accel': 4.0,    # m/s^2 longitudinal at zero lateral demand
    'a_brake': 7.0,    # m/s^2
    'power': 20.5,     # W/kg, i.e. a_x * v. Binds above a_accel/power = 5.1 m/s.
    'drag': 0.0,       # 1/s. 0.273 is the simulator's Rigidbody linear drag.
    'v_max': 8.0,      # matches v_max in config/pure_pursuit.yaml
    'v_min': 1.0,
}

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


def track_layers(track):
    """Every grid for this track that can actually be placed in the world.

    A .pgm with no .yaml has no resolution and no origin, so it cannot be
    georeferenced and is not offered -- maps/icra/track_new.pgm is one, a GIMP
    intermediate. Returns [{name, pgm, yaml}], the track's default grid first,
    since that is the one AMCL localizes against.
    """
    spec = frames.track_spec(track)
    default = os.path.basename(spec['map_yaml'])[:-len('.yaml')]
    d = os.path.dirname(os.path.join(frames.MAPS_DIR, spec['map_yaml']))
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


def raceline_dir(track):
    """The directory this track's lines live in."""
    spec = frames.track_spec(track)
    return os.path.dirname(os.path.join(frames.RACELINE_DIR, spec['raceline']))


class Session:
    """Everything the page needs, resolved once at start-up.

    Held in memory rather than re-read per request so that editing a CSV on
    disk underneath a live session cannot half-swap the line the page is
    working on.
    """

    def __init__(self, track, line_name):
        self.track = track
        self.layers = track_layers(track)
        self.dir = raceline_dir(track)
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
        self.path = frames.raceline_path(line_name, track) if line_name \
            else frames.raceline_path(os.path.basename(frames.DEFAULT_RACELINE), track)
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
            'half_width': HALF_WIDTH,
            'free': FREE,
            'header': HEADER,
        }

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
                    help='line to open, by bare filename within the track\'s '
                         'raceline directory; omit for the track default')
    ap.add_argument('--track', default=None,
                    help=f'which track; default is $RACER_TRACK, currently '
                         f'{frames.TRACK!r}. Known: {sorted(frames.TRACKS)}')
    ap.add_argument('--port', type=int, default=8848, help='port to serve on')
    ap.add_argument('--no-browser', action='store_true',
                    help='do not open a browser; just print the URL')
    args = ap.parse_args(argv)

    track = args.track or frames.TRACK
    if track not in frames.TRACKS:
        raise SystemExit(f'unknown track {track!r}; known: {sorted(frames.TRACKS)}')

    here = os.path.dirname(os.path.abspath(__file__))
    page_path = os.path.join(here, 'raceline_editor.html')
    if not os.path.isfile(page_path):
        raise SystemExit(f'{page_path} is missing; it ships beside this script')

    Handler.session = Session(track, args.raceline)
    Handler.page = open(page_path).read()

    n = len(Handler.session.rows)
    lap = Handler.session.rows[-1][0] + Handler.session.rows[1][0]
    print(f'[editor] track {track}, {len(Handler.session.layers)} grid layer(s): '
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
