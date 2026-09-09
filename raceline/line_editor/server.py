#!/usr/bin/env python
"""Interactive raceline editor: drag the line on the map, shape its speed, save.

    cd raceline && ../.venv-rl/bin/python -m line_editor --track icra2026 --line raceline_a7.0zv_hard_l6.0_corners_h.csv

Run it from .venv-rl (it imports optimize_raceline, hence trajectory_planning_
helpers); it serves one page on localhost and opens it. Everything the page
shows comes from the pipeline's own functions -- the same splines, widths,
physics and zone semantics -- so what it saves is what optimize_raceline would
have exported for that geometry under those arguments. Each save writes

    raceline/<track>/<name>.csv         s,x,y,psi,kappa,w_r,w_l,v   (what pure_pursuit reads)
    raceline/<track>/<name>.edit.json   the source line and the arguments that built the profile

The .edit.json is what a later session reloads, and its a_lat / a_long /
a_brake / v_max / v_zones / lat_zones are optimize_raceline's flags, so the
profile is reproducible from the command line. A line without a sidecar gets
its arguments inferred: the rung and the a_long from the file name, the zones
from the registry when the name carries the z / v suffix, v_max from the
fastest point outside any speed zone. The page reports the lap time of the
file's own speed column next to the recomputed one, so a wrong inference shows.
"""
import argparse
import base64
import io
import json
import re
import sys
import threading
import traceback
import webbrowser
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
from PIL import Image

TOOL_DIR = Path(__file__).resolve().parent          # raceline/line_editor/
RACELINE = TOOL_DIR.parent                          # raceline/: the pipeline and the per-track line folders
REPO = RACELINE.parent
STATIC = TOOL_DIR / "static"
sys.path.insert(0, str(RACELINE))
sys.path.insert(0, str(REPO / "devkit_ws/src/racer_common"))
from optimize_raceline import (PHYS, ProfileLimits, TrackMap, apply_v_zones, enforce_long,  # noqa: E402
                               heading, map_base, resample_closed, spline_geometry,
                               steering_rate_required, velocity_profile)
from racer_common import frames  # noqa: E402  (plain Python, no ROS needed)

CSV_HEADER = "s_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m,v_mps"
PARAM_KEYS = ("a_lat", "a_long", "a_brake", "v_max")


def parse_zones(spec):
    """'s0:s1:v,...' or [[s0, s1, v], ...] -> [(s0, s1, v), ...]."""
    if isinstance(spec, str):
        return [tuple(float(v) for v in z.split(":")) for z in spec.split(",") if z.strip()]
    return [tuple(float(v) for v in z) for z in (spec or [])]


def zones_str(zones):
    return ",".join(f"{s0:g}:{s1:g}:{v:g}" for s0, s1, v in zones)


def lap_time(el, v):
    """The pipeline's own number: segment length over segment mean speed, closed."""
    return float(np.sum(2.0 * el / (v + np.roll(v, -1))))


def rounded(a, nd=5):
    return np.asarray(a, float).round(nd).tolist()


class Editor:
    def __init__(self, track, line, safety):
        self.track = track
        self.reg = frames.TRACKS.get(track, {})
        self.dir = RACELINE / track
        self.safety = safety
        self.tm = TrackMap(map_base(track))
        self.map_png = self._map_png()
        self.line = self.load(line or self.reg.get("raceline"))

    # ---- map --------------------------------------------------------------
    def _map_png(self):
        tm = self.tm
        rgb = np.full((tm.H, tm.W, 3), 160, np.uint8)      # unknown
        rgb[tm.img == 0] = 20                                # wall
        rgb[tm.free] = 255                                   # free
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, "PNG")
        return base64.b64encode(buf.getvalue()).decode()

    def files(self):
        return sorted(p.name for p in self.dir.glob("*.csv"))

    # ---- the profile, exactly as the pipeline's ladder builds it -----------
    # dv: optional per-point speed edits in m/s, added on top of the physics
    # profile. They are the user's word, not the tire's: nothing re-imposes
    # feasibility unless p["resweep_edits"] says so, and the implied
    # longitudinal demand is reported so an unreachable step is visible.
    def profile(self, x, y, p, dv=None, fast=False):
        x, y = np.asarray(x, float), np.asarray(y, float)
        if len(x) < 8 or len(x) != len(y):
            raise ValueError("need at least 8 points")
        g = spline_geometry(x, y)
        el, kappa = g["el"], g["kappa"]
        s = np.concatenate([[0.0], np.cumsum(el)[:-1]])
        psi = heading(x, y)
        # fast: the live estimate during a drag; the wall raycast is the slow part
        w_r, w_l = (None, None) if fast else self.tm.widths(x, y, psi)
        body = self.tm.clearance(x, y) - PHYS.width / 2.0

        rung, a_long, a_brake, v_max = (float(p[k]) for k in PARAM_KEYS)
        lat_zones, v_zones = parse_zones(p.get("lat_zones")), parse_zones(p.get("v_zones"))
        mu = None
        if lat_zones:
            mu = np.ones(len(s))
            for s0, s1, a in lat_zones:
                mu[(s >= s0) & (s <= s1)] = a / rung
        lr = ProfileLimits(a_lat=rung, a_long=a_long, v_max=v_max)
        resweep = bool(v_zones) or abs(a_brake - a_long) > 1e-9
        if resweep:
            top = ProfileLimits(a_lat=rung, a_long=a_long,
                                v_max=max([v_max] + [z[2] for z in v_zones]))
            v, _, _ = velocity_profile(kappa, el, top, mu=mu)
            if v_zones:
                v = apply_v_zones(v, el, s, v_zones, lr)
            v = enforce_long(v, el, a_long, a_brake)
        else:
            v, _, _ = velocity_profile(kappa, el, lr, mu=mu)
        v_phys = v.copy()
        edits = np.zeros(len(v))
        if dv is not None and len(dv) == len(v):
            edits = np.nan_to_num(np.asarray(dv, float))
            if np.any(edits != 0.0):
                v = np.maximum(v + edits, 0.3)
                if p.get("resweep_edits"):
                    v = enforce_long(v, el, a_long, a_brake)
                    edits = v - v_phys                        # what survived, so the page shows the truth
        a_lat = v ** 2 * np.abs(kappa)
        ax = np.gradient(v ** 2) / (2.0 * np.maximum(el, 1e-6))     # the pipeline's own estimate
        sr = steering_rate_required(kappa, el, v)
        ib, ia = int(np.argmin(body)), int(np.argmax(a_lat))
        return dict(s=rounded(s), psi=rounded(psi), kappa=rounded(kappa),
                    w_r=None if fast else rounded(w_r), w_l=None if fast else rounded(w_l),
                    v=rounded(v), v_phys=rounded(v_phys), dv=rounded(edits, 3), fast=fast,
                    body=rounded(body), a_lat=rounded(a_lat),
                    t=lap_time(el, v), length=float(el.sum()),
                    body_min=float(body[ib]), body_min_s=float(s[ib]),
                    lat_max=float(a_lat[ia]), lat_max_s=float(s[ia]),
                    ax_max=float(ax.max()), ax_min=float(ax.min()),
                    steer_rate_max=float(sr.max()), kmax=float(np.abs(kappa).max()),
                    _raw=dict(s=s, psi=psi, kappa=kappa, w_r=w_r, w_l=w_l, v=v, el=el))

    # ---- arguments a line was built with --------------------------------
    def default_params(self, name, s, v_file):
        side = self.dir / (Path(name).stem + ".edit.json")
        if side.exists():
            a = json.loads(side.read_text())["args"]
            return dict(a_lat=float(a["a_lat"]), a_long=float(a["a_long"]), a_brake=float(a["a_brake"]),
                        v_max=float(a["v_max"]), v_zones=[list(z) for z in parse_zones(a["v_zones"])],
                        lat_zones=[list(z) for z in parse_zones(a["lat_zones"])],
                        resweep_edits=bool(a.get("resweep_edits", False)))
        m = re.search(r"_a(\d+\.\d+)(z?)(v?)", name)
        rung = float(m.group(1)) if m else ProfileLimits.a_lat
        lat_zones = parse_zones(self.reg.get("lat_zones", "")) if m and m.group(2) else []
        v_zones = parse_zones(self.reg.get("v_zones", "")) if m and m.group(3) else []
        ml = re.search(r"_l(\d+\.\d+)", name)
        # _l6.0 names the longitudinal rung; those lines also carry the registry's
        # brake budget. Older names were built at the profile default, brake = accel.
        a_long = float(ml.group(1)) if ml else PHYS.a_long_profile
        a_brake = float(self.reg.get("a_brake", a_long)) if ml else a_long
        if v_file is not None and v_zones:
            # The registry's zones are today's; an older line may have been built
            # with the straight at 8 rather than 9. Inside a speed zone the ceiling
            # binds, so the file's fastest point there IS the level it was built with.
            v_zones = [(s0, s1, float(np.round(v_file[(s >= s0) & (s <= s1)].max(), 1))
                        if ((s >= s0) & (s <= s1)).any() else v) for s0, s1, v in v_zones]
        if v_file is not None:
            outside = np.ones(len(s), bool)
            for s0, s1, _ in v_zones:
                outside &= ~((s >= s0) & (s <= s1))
            v_max = float(np.round(v_file[outside].max() if outside.any() else v_file.max(), 2))
        else:
            v_max = float(self.reg.get("follower", {}).get("v_max", ProfileLimits.v_max))
        return dict(a_lat=rung, a_long=a_long, a_brake=a_brake, v_max=v_max,
                    v_zones=[list(z) for z in v_zones], lat_zones=[list(z) for z in lat_zones],
                    resweep_edits=False)

    @staticmethod
    def params(p):
        out = {k: float(p[k]) for k in PARAM_KEYS}
        out["v_zones"] = [list(z) for z in parse_zones(p.get("v_zones"))]
        out["lat_zones"] = [list(z) for z in parse_zones(p.get("lat_zones"))]
        out["resweep_edits"] = bool(p.get("resweep_edits", False))
        return out

    def sidecar_dv(self, name, s):
        """Speed edits recorded with a saved line, re-sampled onto s (exact when the
        geometry is the saved one, which it is on a reload)."""
        side = self.dir / (Path(name).stem + ".edit.json")
        if not side.exists():
            return None
        e = json.loads(side.read_text()).get("speed_edits")
        if not e or not e.get("s"):
            return None
        return np.interp(s, np.asarray(e["s"], float), np.asarray(e["dv"], float), period=float(s[-1]) + 0.1)

    # ---- requests -----------------------------------------------------------
    def load(self, name):
        path = self.dir / name
        if not path.exists():
            raise FileNotFoundError(f"{path} does not exist; known: {self.files()}")
        D = np.loadtxt(path, delimiter=",")
        x, y = D[:, 1], D[:, 2]
        v_file = D[:, 7] if D.shape[1] > 7 else None
        p = self.default_params(name, D[:, 0], v_file)
        r = self.profile(x, y, p, self.sidecar_dv(name, D[:, 0]))
        t_file = lap_time(r["_raw"]["el"], v_file) if v_file is not None else None
        r.pop("_raw")
        r.update(name=name, stem=path.stem, x=rounded(x), y=rounded(y), params=p, t_file=t_file,
                 v_file=rounded(v_file) if v_file is not None else None)
        self.line = r
        return r

    def init(self):
        tm = self.tm
        return dict(track=self.track, safety=self.safety, files=self.files(),
                    map=dict(png=self.map_png, ox=tm.ox, oy=tm.oy, res=tm.res, W=tm.W, H=tm.H),
                    phys=dict(width=PHYS.width, a_lat_robust=PHYS.a_lat_robust, kappa_car=PHYS.kappa_car),
                    line=self.line)

    def profile_req(self, body):
        r = self.profile(body["x"], body["y"], self.params(body["params"]), body.get("dv"), bool(body.get("fast")))
        r.pop("_raw")
        return r

    def resample(self, body):
        n = int(body.get("n") or len(body["x"]))
        x, y = resample_closed(np.asarray(body["x"], float), np.asarray(body["y"], float),
                               n, float(body.get("smooth") or 0.0))
        return dict(x=rounded(x), y=rounded(y))

    def save(self, body):
        name = str(body.get("name", "")).strip()
        if not name.endswith(".csv"):
            name += ".csv"
        if "/" in name or name.startswith(".") or name == ".csv":
            raise ValueError(f"bad name {name!r}")
        path = self.dir / name
        if path.exists() and not body.get("overwrite"):
            raise FileExistsError(name)
        p = self.params(body["params"])
        x, y = np.asarray(body["x"], float), np.asarray(body["y"], float)
        r = self.profile(x, y, p, body.get("dv"))
        raw = r.pop("_raw")
        np.savetxt(path, np.column_stack([raw["s"], x, y, raw["psi"], raw["kappa"], raw["w_r"], raw["w_l"], raw["v"]]),
                   delimiter=",", fmt="%.5f", header=CSV_HEADER, comments="# ")
        edited = [i for i, d in enumerate(r["dv"]) if d != 0.0]
        side = dict(source=self.line["name"], track=self.track, saved=datetime.now().isoformat(timespec="seconds"),
                    args={**{k: p[k] for k in PARAM_KEYS}, "v_zones": zones_str(p["v_zones"]),
                          "lat_zones": zones_str(p["lat_zones"]), "resweep_edits": p["resweep_edits"]},
                    # the hand-set speed layer, in full, so a reload restores it exactly
                    speed_edits=dict(s=r["s"], dv=r["dv"]) if edited else None,
                    edited_points=len(edited), points=len(x), lap_on_paper=r["t"], length=r["length"],
                    body_min=r["body_min"], lat_max=r["lat_max"], ax_max=r["ax_max"], ax_min=r["ax_min"],
                    steer_rate_max=r["steer_rate_max"])
        (self.dir / (path.stem + ".edit.json")).write_text(json.dumps(side, indent=2) + "\n")
        return dict(path=str(path), t=r["t"], body_min=r["body_min"], lat_max=r["lat_max"], files=self.files())


MIME = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8", ".css": "text/css; charset=utf-8",
        ".png": "image/png", ".svg": "image/svg+xml", ".ico": "image/x-icon"}


def make_handler(ed: Editor, verbose: bool):
    routes = {"/api/profile": ed.profile_req, "/api/save": ed.save,
              "/api/load": lambda b: ed.load(b["name"]), "/api/resample": ed.resample}

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, payload, ctype="application/json"):
            data = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _static(self, name):
            # read per request, so an edit to the page shows on reload; never leave static/
            f = (STATIC / name).resolve()
            if not f.is_file() or STATIC not in f.parents:
                return self._send(404, {"error": "not found"})
            self._send(200, f.read_bytes(), MIME.get(f.suffix, "application/octet-stream"))

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._static("index.html")
            elif self.path.startswith("/static/"):
                self._static(self.path[len("/static/"):].split("?")[0])
            elif self.path == "/api/init":
                self._send(200, ed.init())
            elif self.path == "/favicon.ico":
                self._send(204, b"", "image/x-icon")
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
                fn = routes.get(self.path)
                if fn is None:
                    return self._send(404, {"error": "not found"})
                self._send(200, fn(body))
            except FileExistsError as e:
                self._send(409, {"error": f"{e} exists"})
            except Exception as e:  # noqa: BLE001  -- report to the page, keep serving
                traceback.print_exc()
                self._send(400, {"error": f"{type(e).__name__}: {e}"})

        def log_message(self, fmt, *args):
            if verbose:
                super().log_message(fmt, *args)

    return Handler


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--track", default=frames.TRACK, help=f"track name; default {frames.TRACK} (RACER_TRACK)")
    p.add_argument("--line", default=None, help="CSV in raceline/<track>/; default: the registry's line")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--safety", type=float, default=0.15, help="body-to-wall margin the page flags [m]")
    p.add_argument("--no-browser", action="store_true")
    p.add_argument("-v", "--verbose", action="store_true", help="log every request")
    a = p.parse_args(argv)

    ed = Editor(a.track, a.line, a.safety)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(ed, a.verbose))
    url = f"http://127.0.0.1:{a.port}/"
    ln = ed.line
    print(f"line editor: {a.track} / {ln['name']}: {ln['length']:.2f} m, {ln['t']:.3f} s on paper"
          + (f" (file {ln['t_file']:.3f})" if ln["t_file"] else "") + f"\n  {url}   (Ctrl-C to stop)")
    if not a.no_browser:
        threading.Timer(0.5, webbrowser.open, [url]).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
