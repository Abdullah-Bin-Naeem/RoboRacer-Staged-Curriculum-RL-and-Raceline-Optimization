# Line editor

Interactive editing of a raceline on the track map, with the speed profile
shaped by hand, saved in the layout `pure_pursuit` reads.

```bash
cd raceline
../.venv-rl/bin/python -m line_editor --track icra2026 --line raceline_a7.0zv_hard_l6.0_corners_h.csv
```

Run it from `.venv-rl`: it imports `optimize_raceline`, hence
`trajectory_planning_helpers`. It serves one page on `localhost:8765` and opens
it. `--line` defaults to the track registry's line, `--safety` (0.15 m) is the
body-to-wall margin the page flags in red, `--no-browser` and `--port` do what
they say. Everything the page computes goes through the pipeline's own
functions, so a saved line is what `optimize_raceline` would have exported for
that geometry under those arguments, plus whatever was drawn by hand.

## The page

Two full-window views, keys `1` and `2`, one state and one undo stack. The
header carries the lap time on paper (marked *est.* while a stroke is in
progress and computed locally, confirmed by the server on release), the length,
the tightest body margin, the peak lateral demand, the peak acceleration and
braking the profile implies, and the steering rate the path needs. Red means
past the safety margin, past the plan's own lateral limit, or past the accel
and brake budgets. `?` opens the key list.

**Map.** Drag a point; neighbours follow with a brush falloff. Points you
dragged form the *edited region*, tinted amber on the map and on the curvature
strip below; *Smooth edited region* runs a Gaussian on x and y over it, reaching
two radii past its edges (that is where a brush drag leaves its kinks) and
blending to zero, so nothing else moves. Apply again for more. *Resample all*
restores uniform spacing; with `spline smooth > 0` it fits a smoothing spline
through the whole line. The curvature strip shows κ against s with the car's
full-lock limit; hover it to find the spot on the map. With *live* ticked the
profile is recomputed while you drag (a fast pass without the wall widths),
the full pass on release.

**Speed.** The profile in three panels: speed, lateral demand v²|κ|, and the
longitudinal demand with the accel and brake budgets. Grey is the file's own
speed column, dashed blue the physics profile under your edits, amber ticks the
edited points. The lap is a closed loop, so the s-axis carries a shaded wrap
margin at both ends where the loop continues: the curve closes on itself and a
stroke can be drawn across the start line. A minimap in the sidebar shows where
the chart cursor is on the track, with the edited points in amber and the
brush's reach ringed; hover or click it to jump the cursor. Tools:

| tool | key | does |
|---|---|---|
| Draw | `D` | the curve follows the mouse over the s-range you sweep; the stroke is Gaussian-smoothed on release (`stroke smoothing`) |
| Smooth | `S` | smooths your edit layer under the brush; keep moving for more |
| Erase | `E` | fades edits under the brush back to the profile |
| Zone | `Z` | drag a level on the speed panel for a speed zone, on the lateral panel for a lateral zone; shift-click a band removes it |

The edit layer is yours, not the tire's: a step stays a step unless *re-impose
the accel / brake budgets* is ticked, which turns it back into the ramp. The
numeric *from s … to … by …* bump adds an exact amount over a range. *Use
file's speeds* sets the layer so the curve equals the file's own column, the
cleanest start for small changes to a validated line.

**Profile.** The `optimize_raceline` flags: the lateral rung, the accel and
brake budgets, the global ceiling, and the speed and lateral zones, editable as
tables. Every change re-solves the profile.

**File.** Save writes `raceline/<track>/<name>.csv` (`s,x,y,psi,kappa,w_r,w_l,v`)
and `<name>.edit.json` with the source line, every argument and the full edit
layer, so a reload restores the state and the profile is reproducible from
the command line. `Ctrl-S`, `Ctrl-Z`, `Ctrl-Y`.

A line without a sidecar gets its arguments inferred (rung and `a_long` from
the name, zones from the registry, the speed-zone level and `v_max` from the
file's own speed column). The page shows the lap time of the file's column next
to the recomputed one and says in red when they disagree.

## Files

| file | role |
|---|---|
| `server.py` | `Editor` (load, profile, resample, save) and the HTTP server; `python -m line_editor` |
| `static/index.html`, `style.css` | the page and its design tokens |
| `static/app.js` | state, server calls, undo, the estimate, the controls both views share |
| `static/map.js` | the map view: grid, line, drag brush, edited region and its smoother, curvature strip |
| `static/speed.js` | the speed view: three panels and the draw / smooth / erase / zone tools |
| `static/main.js` | boot |
| `test_editor.py` | Selenium end-to-end test in headless Chrome: `../.venv-rl/bin/python line_editor/test_editor.py --shots /tmp/shots` |

Static files are read per request, so an edit to the page shows on reload.

## What the follower does with a saved line

Speed is the CSV's `v` at the delay-matched point ahead, clipped to the
follower's `v_max` (8.0 in the registry; pass `v_max:=` to raise it).
Acceleration has no number: throttle is a wheel speed inside the tire's slip
band, so the car accelerates at what the tire gives whatever the profile asks.
Steering is capped at 7.0 m/s² over v², so a corner asked for more than the
tire runs wide. The derate reads the CSV's peak v²|κ| and scales every target
down on a slow loop, so an edit that inflates that peak is a hidden lever.
