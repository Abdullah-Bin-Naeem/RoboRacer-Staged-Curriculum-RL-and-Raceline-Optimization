#!/usr/bin/env python3

"""A tiny HTML+SVG report writer, standard library only.

analyze_localization.py needs charts, and the container this all runs in has
numpy (from ROS) and nothing else -- no matplotlib, no plotly. Rather than add
a dependency to an image that ships in a competition submission, the charts are
emitted as inline SVG into one self-contained HTML file. It opens from the
mounted runs/ directory with no server, no CDN and no network.

WHAT IS DELIBERATE HERE
-----------------------
Colour is assigned by the job the data is doing, not by series index:

    sequential (one hue, light->dark)   magnitude -- error binned by lap
                                        position, by speed, by curvature, and
                                        the track map. Most charts here are
                                        magnitude, so most are one blue ramp.
    categorical (fixed hue order)       identity -- only where several series
                                        share one plot (err_dist vs err_cross
                                        vs dr_err). Slots are taken in order
                                        and never cycled.
    diverging (blue <-> red, gray mid)  polarity -- signed cross-track error,
                                        where left-of-truth and right-of-truth
                                        are opposite things and zero is neutral.

Only the first three categorical slots are used. That is the documented cap for
charts where every pair of series can appear together: past three, the palette
stops being separable under colour-vision deficiency, and no chart here needs a
fourth line.

Hover is native. Every mark carries an SVG <title>, which browsers render as a
tooltip with no JavaScript at all; line charts get invisible full-height hit
bands so the tooltip reports every series at that x, plus a CSS-only crosshair.
The one piece of script in the output is the light/dark toggle.

Themes are selected, not flipped: the dark values are their own steps against
the dark surface. Both are defined as CSS custom properties, so every chart is
written against roles (--series-1, --ink-muted) rather than raw hex.
"""

import html
import math

# --- palette -----------------------------------------------------------------
# Categorical slots 1-3 and the blue sequential ramp, taken unmodified from the
# validated reference palette. The ordering is the colour-vision-deficiency
# safety mechanism, not decoration -- do not reorder, and do not add a fourth
# slot to an all-pairs chart.
LIGHT = {
    'surface': '#fcfcfb', 'panel': '#ffffff', 'edge': '#e6e5e0',
    'ink': '#0b0b0b', 'ink2': '#52514e', 'ink3': '#7d7c76',
    'grid': '#ecebe6',
    'series1': '#2a78d6', 'series2': '#eb6834', 'series3': '#1baf7a',
    'pos': '#2a78d6', 'neg': '#e34948', 'mid': '#f0efec',
    'warn': '#eda100', 'bad': '#e34948', 'good': '#008300',
}
DARK = {
    'surface': '#1a1a19', 'panel': '#222221', 'edge': '#38383550',
    'ink': '#ffffff', 'ink2': '#c3c2b7', 'ink3': '#8e8d85',
    'grid': '#2e2e2c',
    'series1': '#3987e5', 'series2': '#d95926', 'series3': '#199e70',
    'pos': '#3987e5', 'neg': '#e66767', 'mid': '#383835',
    'warn': '#c98500', 'bad': '#e66767', 'good': '#008300',
}
# Sequential blue, light -> dark. Index 0 is nearest the surface and means
# "near zero"; it is only ever used for continuous magnitude, never as an
# ordinal step (which would have to start no lighter than RAMP[2]).
RAMP_LIGHT = ['#cde2fb', '#9ec5f4', '#86b6ef', '#5598e7', '#3987e5',
              '#2a78d6', '#256abf', '#1c5cab', '#184f95', '#104281']
RAMP_DARK = ['#104281', '#184f95', '#1c5cab', '#256abf', '#2a78d6',
             '#3987e5', '#5598e7', '#86b6ef', '#9ec5f4', '#cde2fb']


def esc(s):
    return html.escape(str(s), quote=True)


def _fmt(v, nd=3):
    if v is None:
        return '--'
    try:
        f = float(v)
    except (TypeError, ValueError):
        return esc(v)
    if f != f:                                   # NaN
        return '--'
    if math.isinf(f):
        return 'inf'
    return f'{f:.{nd}f}'


def nice_ticks(lo, hi, n=5):
    """Round tick values spanning [lo, hi]. Never returns an empty list."""
    if not (lo == lo and hi == hi):              # NaN in, nothing out
        return [0.0, 1.0]
    if hi <= lo:
        hi = lo + 1.0
    raw = (hi - lo) / max(n, 1)
    mag = 10 ** math.floor(math.log10(raw)) if raw > 0 else 1.0
    for m in (1, 2, 2.5, 5, 10):
        if raw / mag <= m:
            step = m * mag
            break
    else:
        step = 10 * mag
    first = math.ceil(lo / step) * step
    out, v = [], first
    while v <= hi + step * 1e-6:
        out.append(round(v, 10))
        v += step
    return out or [lo, hi]


class Axes:
    """Pixel mapping plus the frame: grid, ticks, axis titles.

    One class rather than a helper per chart, because every chart in this
    report shares the same margins and the same recessive grid; the charts
    then only have to emit their own marks.
    """

    def __init__(self, w=760, h=280, pad=(52, 16, 40, 58)):
        self.w, self.h = w, h
        self.pt, self.pr, self.pb, self.pl = pad
        self.iw = w - self.pl - self.pr
        self.ih = h - self.pt - self.pb
        self.x0 = self.x1 = self.y0 = self.y1 = 0.0

    def domain(self, xs, ys, ymin=None, ymax=None, xmin=None, xmax=None):
        xs = [v for v in xs if v == v]
        ys = [v for v in ys if v == v]
        self.x0 = xmin if xmin is not None else (min(xs) if xs else 0.0)
        self.x1 = xmax if xmax is not None else (max(xs) if xs else 1.0)
        self.y0 = ymin if ymin is not None else (min(ys) if ys else 0.0)
        self.y1 = ymax if ymax is not None else (max(ys) if ys else 1.0)
        if self.x1 <= self.x0:
            self.x1 = self.x0 + 1.0
        if self.y1 <= self.y0:
            self.y1 = self.y0 + 1.0
        else:                                    # a little headroom for labels
            self.y1 += (self.y1 - self.y0) * 0.06
        return self

    def px(self, x):
        return self.pl + (x - self.x0) / (self.x1 - self.x0) * self.iw

    def py(self, y):
        return self.pt + self.ih - (y - self.y0) / (self.y1 - self.y0) * self.ih

    def frame(self, xlabel='', ylabel='', xticks=None, yticks=None, xfmt=None):
        xt = xticks if xticks is not None else nice_ticks(self.x0, self.x1, 6)
        yt = yticks if yticks is not None else nice_ticks(self.y0, self.y1, 5)
        fx = xfmt or (lambda v: f'{v:g}')
        o = []
        for v in yt:
            y = self.py(v)
            o.append(f'<line class="grid" x1="{self.pl:.1f}" y1="{y:.1f}" '
                     f'x2="{self.pl + self.iw:.1f}" y2="{y:.1f}"/>')
            o.append(f'<text class="tick" x="{self.pl - 8:.1f}" y="{y + 4:.1f}" '
                     f'text-anchor="end">{v:g}</text>')
        for v in xt:
            x = self.px(v)
            o.append(f'<text class="tick" x="{x:.1f}" '
                     f'y="{self.pt + self.ih + 18:.1f}" '
                     f'text-anchor="middle">{esc(fx(v))}</text>')
        o.append(f'<line class="axis" x1="{self.pl:.1f}" '
                 f'y1="{self.pt + self.ih:.1f}" x2="{self.pl + self.iw:.1f}" '
                 f'y2="{self.pt + self.ih:.1f}"/>')
        if xlabel:
            o.append(f'<text class="axlabel" x="{self.pl + self.iw / 2:.1f}" '
                     f'y="{self.h - 6:.1f}" text-anchor="middle">'
                     f'{esc(xlabel)}</text>')
        if ylabel:
            o.append(f'<text class="axlabel" transform="translate(13,'
                     f'{self.pt + self.ih / 2:.1f}) rotate(-90)" '
                     f'text-anchor="middle">{esc(ylabel)}</text>')
        return ''.join(o)

    def rule(self, y, label, cls='rule'):
        """A horizontal reference line -- a wall margin, or fit_ratio = 1."""
        if not (self.y0 <= y <= self.y1):
            return ''
        yy = self.py(y)
        return (f'<line class="{cls}" x1="{self.pl:.1f}" y1="{yy:.1f}" '
                f'x2="{self.pl + self.iw:.1f}" y2="{yy:.1f}"/>'
                f'<text class="rulelabel" x="{self.pl + self.iw - 4:.1f}" '
                f'y="{yy - 5:.1f}" text-anchor="end">{esc(label)}</text>')

    def open(self, extra=''):
        return (f'<svg viewBox="0 0 {self.w} {self.h}" width="100%" '
                f'preserveAspectRatio="xMidYMid meet" role="img" {extra}>')


def _path(ax, xs, ys):
    """Polyline with NaN treated as a break, not as zero."""
    d, pen = [], False
    for x, y in zip(xs, ys):
        if x != x or y != y:
            pen = False
            continue
        d.append(f'{"L" if pen else "M"}{ax.px(x):.1f} {ax.py(y):.1f}')
        pen = True
    return ''.join(d)


def line_chart(xs, series, xlabel='', ylabel='', rules=(), height=280,
               ymin=None, ymax=None, bands=28, xfmt=None):
    """Multi-series line chart with a native-tooltip hover layer.

    `series` is [(label, values, role)] with role in series1..3 -- taken in
    order, never cycled. A single series needs no legend: the caption names it.
    """
    ax = Axes(h=height)
    ally = [v for _, vals, _ in series for v in vals]
    ax.domain(xs, ally, ymin=ymin, ymax=ymax)
    o = [ax.open(), ax.frame(xlabel, ylabel, xfmt=xfmt)]
    for lab, val in rules:
        o.append(ax.rule(val, lab))
    for _, vals, role in series:
        o.append(f'<path class="ln" stroke="var(--{role})" '
                 f'd="{_path(ax, xs, vals)}"/>')

    # Hover: invisible full-height bands. The <title> is the tooltip, so the
    # reader gets every series at that x with no script.
    n = len(xs)
    if n > 1 and bands:
        step = max(1, n // bands)
        bw = ax.iw / max(1, n // step)
        for i in range(0, n, step):
            x = xs[i]
            if x != x:
                continue
            cx = ax.px(x)
            parts = [f'{esc(xlabel.split("[")[0].strip() or "x")} '
                     f'{(xfmt(x) if xfmt else f"{x:g}")}']
            for lab, vals, _ in series:
                parts.append(f'{lab}: {_fmt(vals[i])}')
            o.append(
                f'<g class="band"><rect x="{cx - bw / 2:.1f}" y="{ax.pt}" '
                f'width="{bw:.1f}" height="{ax.ih}" fill="transparent">'
                f'<title>{esc(chr(10).join(parts))}</title></rect>'
                f'<line class="cross" x1="{cx:.1f}" y1="{ax.pt}" '
                f'x2="{cx:.1f}" y2="{ax.pt + ax.ih}"/></g>')
    o.append('</svg>')
    legend = ''
    if len(series) > 1:
        legend = '<div class="legend">' + ''.join(
            f'<span class="lg"><i style="background:var(--{r})"></i>'
            f'{esc(lab)}</span>' for lab, _, r in series) + '</div>'
    return ''.join(o) + legend


def bar_chart(labels, values, xlabel='', ylabel='', height=280, rules=(),
              notes=None, sort_desc=False, ymax=None):
    """Magnitude by category -- one hue, more-is-darker (sequential).

    Darkness carries the same information as height on purpose: it is what
    makes the worst bins findable at a glance in a 40-bin chart.
    """
    rows = list(zip(labels, values, notes or [''] * len(labels)))
    if sort_desc:
        rows.sort(key=lambda r: (-(r[1] if r[1] == r[1] else -1e9)))
    labels = [r[0] for r in rows]
    values = [r[1] for r in rows]
    notes = [r[2] for r in rows]

    ax = Axes(h=height)
    ax.domain(range(len(values)), list(values) + [0.0], ymin=0.0, ymax=ymax,
              xmin=-0.5, xmax=len(values) - 0.5)
    o = [ax.open()]
    yt = nice_ticks(0.0, ax.y1, 5)
    o.append(ax.frame(xlabel, ylabel, xticks=[], yticks=yt))
    for lab, val in rules:
        o.append(ax.rule(val, lab))

    finite = [v for v in values if v == v]
    vmax = max(finite) if finite else 1.0
    vmin = min(finite) if finite else 0.0
    bw = ax.iw / max(len(values), 1)
    for i, v in enumerate(values):
        if v != v:
            continue
        x = ax.px(i) - bw / 2 + 1.0                 # 2px surface gap between bars
        y = ax.py(v)
        h = ax.py(0.0) - y
        frac = (v - vmin) / (vmax - vmin) if vmax > vmin else 1.0
        step = min(9, max(0, int(frac * 9)))
        tip = f'{labels[i]}\n{ylabel or "value"}: {_fmt(v)}'
        if notes[i]:
            tip += f'\n{notes[i]}'
        o.append(
            f'<rect class="bar" x="{x:.1f}" y="{y:.1f}" '
            f'width="{max(bw - 2.0, 1.0):.1f}" height="{max(h, 0.5):.1f}" '
            f'rx="4" fill="var(--ramp{step})"><title>{esc(tip)}</title></rect>')

    # Label only the ends and a few interior ticks: a number on every bar is
    # noise at 40 bins.
    if labels:
        idx = sorted({0, len(labels) - 1, len(labels) // 2,
                      len(labels) // 4, 3 * len(labels) // 4})
        for i in idx:
            o.append(f'<text class="tick" x="{ax.px(i):.1f}" '
                     f'y="{ax.pt + ax.ih + 18:.1f}" text-anchor="middle">'
                     f'{esc(labels[i])}</text>')
    o.append('</svg>')
    return ''.join(o)


def diverging_bar(labels, values, xlabel='', ylabel='', height=280, notes=None):
    """Signed magnitude around a neutral zero -- blue above, red below.

    Used for cross-track error, where "estimate is left of the car" and
    "estimate is right of the car" are opposite failures and must not share
    a hue.
    """
    ax = Axes(h=height)
    finite = [v for v in values if v == v] or [0.0]
    lim = max(abs(min(finite)), abs(max(finite)), 1e-6) * 1.12
    ax.domain(range(len(values)), finite, ymin=-lim, ymax=lim,
              xmin=-0.5, xmax=len(values) - 0.5)
    ax.y1 = lim                                    # undo domain()'s headroom
    o = [ax.open(), ax.frame(xlabel, ylabel, xticks=[])]
    zero = ax.py(0.0)
    o.append(f'<line class="axis" x1="{ax.pl:.1f}" y1="{zero:.1f}" '
             f'x2="{ax.pl + ax.iw:.1f}" y2="{zero:.1f}"/>')
    bw = ax.iw / max(len(values), 1)
    notes = notes or [''] * len(values)
    for i, v in enumerate(values):
        if v != v:
            continue
        x = ax.px(i) - bw / 2 + 1.0
        y = min(ax.py(v), zero)
        h = abs(ax.py(v) - zero)
        role = 'pos' if v >= 0 else 'neg'
        tip = f'{labels[i]}\n{ylabel or "value"}: {_fmt(v)}'
        if notes[i]:
            tip += f'\n{notes[i]}'
        o.append(f'<rect class="bar" x="{x:.1f}" y="{y:.1f}" '
                 f'width="{max(bw - 2.0, 1.0):.1f}" height="{max(h, 0.5):.1f}" '
                 f'rx="3" fill="var(--{role})"><title>{esc(tip)}</title></rect>')
    if labels:
        for i in sorted({0, len(labels) // 2, len(labels) - 1}):
            o.append(f'<text class="tick" x="{ax.px(i):.1f}" '
                     f'y="{ax.pt + ax.ih + 18:.1f}" text-anchor="middle">'
                     f'{esc(labels[i])}</text>')
    o.append('</svg>')
    return ''.join(o)


def track_map(line_x, line_y, line_err, true_x=None, true_y=None,
              est_x=None, est_y=None, height=520, err_label='error [m]'):
    """The racing line drawn in map coordinates, coloured by error magnitude.

    The one chart that answers "WHERE is it bad" instead of "how bad is it".
    Aspect ratio is preserved -- a squashed track is a misleading track.
    """
    xs = [v for v in line_x if v == v]
    ys = [v for v in line_y if v == v]
    if not xs or not ys:
        return '<p class="muted">no geometry to draw</p>'
    pad = 0.6
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad
    span_x, span_y = x1 - x0, y1 - y0
    w = 760
    inner_w, inner_h = w - 70, height - 60
    scale = min(inner_w / span_x, inner_h / span_y)
    ox = 35 + (inner_w - span_x * scale) / 2
    oy = 20 + (inner_h - span_y * scale) / 2

    def sx(x):
        return ox + (x - x0) * scale

    def sy(y):                                     # map y is up, SVG y is down
        return oy + (y1 - y) * scale

    o = [f'<svg viewBox="0 0 {w} {height}" width="100%" '
         f'preserveAspectRatio="xMidYMid meet" role="img">']

    if true_x is not None:
        o.append(f'<path class="traj-true" d="{_poly(sx, sy, true_x, true_y)}"/>')
    if est_x is not None:
        o.append(f'<path class="traj-est" d="{_poly(sx, sy, est_x, est_y)}"/>')

    finite = [v for v in line_err if v == v]
    lo = min(finite) if finite else 0.0
    hi = max(finite) if finite else 1.0
    n = len(line_x)
    for i in range(n - 1):
        e = line_err[i]
        if line_x[i] != line_x[i] or line_x[i + 1] != line_x[i + 1]:
            continue
        step = 4 if e != e else min(9, max(0, int((e - lo) / (hi - lo) * 9))) \
            if hi > lo else 4
        o.append(
            f'<line class="seg" stroke="var(--ramp{step})" '
            f'x1="{sx(line_x[i]):.1f}" y1="{sy(line_y[i]):.1f}" '
            f'x2="{sx(line_x[i + 1]):.1f}" y2="{sy(line_y[i + 1]):.1f}">'
            f'<title>{esc(f"{err_label}: {_fmt(e)}")}</title></line>')

    # Ramp legend, low -> high.
    lx, ly = w - 190, height - 26
    for k in range(10):
        o.append(f'<rect x="{lx + k * 14}" y="{ly}" width="13" height="9" '
                 f'fill="var(--ramp{k})"/>')
    o.append(f'<text class="tick" x="{lx}" y="{ly - 5}">{_fmt(lo, 2)}</text>'
             f'<text class="tick" x="{lx + 140}" y="{ly - 5}" '
             f'text-anchor="end">{_fmt(hi, 2)}</text>'
             f'<text class="tick" x="{lx + 146}" y="{ly + 8}">'
             f'{esc(err_label)}</text>')
    o.append('</svg>')
    legend = ('<div class="legend">'
              '<span class="lg"><i class="sw-true"></i>true path</span>'
              '<span class="lg"><i class="sw-est"></i>estimated path</span>'
              '<span class="lg"><i class="sw-ramp"></i>racing line, '
              'shaded by error</span></div>')
    return ''.join(o) + legend


def _poly(sx, sy, xs, ys):
    d, pen = [], False
    for x, y in zip(xs, ys):
        if x != x or y != y:
            pen = False
            continue
        d.append(f'{"L" if pen else "M"}{sx(x):.1f} {sy(y):.1f}')
        pen = True
    return ''.join(d)


def cdf_chart(values, xlabel='error [m]', rules=(), height=280):
    """Share of the run below a given error. Reads off as 'p95 = x'."""
    v = sorted(x for x in values if x == x)
    if not v:
        return '<p class="muted">no samples</p>'
    xs = v
    ys = [100.0 * (i + 1) / len(v) for i in range(len(v))]
    ax = Axes(h=height)
    ax.domain(xs, ys, ymin=0.0, ymax=100.0)
    ax.y1 = 100.0
    o = [ax.open(), ax.frame(xlabel, 'share of run [%]',
                             yticks=[0, 25, 50, 75, 100])]
    for lab, val in rules:                         # vertical: a margin in x
        if ax.x0 <= val <= ax.x1:
            x = ax.px(val)
            o.append(f'<line class="rule" x1="{x:.1f}" y1="{ax.pt}" '
                     f'x2="{x:.1f}" y2="{ax.pt + ax.ih}"/>'
                     f'<text class="rulelabel" x="{x + 5:.1f}" '
                     f'y="{ax.pt + 12}">{esc(lab)}</text>')
    o.append(f'<path class="ln" stroke="var(--series1)" '
             f'd="{_path(ax, xs, ys)}"/>')
    for q in (50, 90, 95, 99):
        i = min(len(v) - 1, int(len(v) * q / 100))
        o.append(f'<circle class="dot" cx="{ax.px(xs[i]):.1f}" '
                 f'cy="{ax.py(ys[i]):.1f}" r="5" fill="var(--series1)">'
                 f'<title>p{q}: {_fmt(xs[i])} m</title></circle>')
    o.append('</svg>')
    return ''.join(o)


def multi_cdf(runs, xlabel='error [m]', rules=(), height=320, best=0):
    """Several runs' error distributions on one axis, by EMPHASIS.

    Nine variants cannot be nine colours -- past three hues no palette stays
    separable under colour-vision deficiency, and inventing more is how a chart
    stops meaning anything. So the winner is drawn in the accent hue and
    everything else in the de-emphasis grey: the reader's question is "is the
    winner actually ahead across the whole distribution, or only at the mean",
    and that is answered by one curve against a band of context, not by nine
    competing colours.

    `runs` is [(label, values)]; `best` indexes the one to emphasise.
    """
    series = []
    for lab, vals in runs:
        v = sorted(x for x in vals if x == x)
        if v:
            series.append((lab, v, [100.0 * (i + 1) / len(v) for i in range(len(v))]))
    if not series:
        return '<p class="muted">no samples</p>'
    ax = Axes(h=height)
    ax.domain([x for _, v, _ in series for x in v], [0.0, 100.0],
              ymin=0.0, ymax=100.0)
    ax.y1 = 100.0
    o = [ax.open(), ax.frame(xlabel, 'share of run [%]',
                             yticks=[0, 25, 50, 75, 100])]
    for lab, val in rules:
        if ax.x0 <= val <= ax.x1:
            x = ax.px(val)
            o.append(f'<line class="rule" x1="{x:.1f}" y1="{ax.pt}" '
                     f'x2="{x:.1f}" y2="{ax.pt + ax.ih}"/>'
                     f'<text class="rulelabel" x="{x + 5:.1f}" '
                     f'y="{ax.pt + 12}">{esc(lab)}</text>')
    for i, (lab, xs, ys) in enumerate(series):
        if i == best:
            continue
        p95 = xs[min(len(xs) - 1, int(len(xs) * 0.95))]
        o.append(f'<path class="ln ctx" stroke="var(--ink3)" '
                 f'd="{_path(ax, xs, ys)}">'
                 f'<title>{esc(f"{lab}  p95 {p95:.3f} m")}</title></path>')
    lab, xs, ys = series[best]
    p95 = xs[min(len(xs) - 1, int(len(xs) * 0.95))]
    o.append(f'<path class="ln" stroke="var(--series1)" '
             f'd="{_path(ax, xs, ys)}">'
             f'<title>{esc(f"{lab}  p95 {p95:.3f} m")}</title></path>')
    o.append('</svg>')
    return ''.join(o) + (
        '<div class="legend">'
        f'<span class="lg"><i style="background:var(--series1)"></i>'
        f'{esc(series[best][0])} (best)</span>'
        f'<span class="lg"><i style="background:var(--ink3)"></i>'
        f'the other {len(series) - 1} — hover any curve to name it</span></div>')


def hist_chart(values, bins=40, xlabel='', height=250):
    v = [x for x in values if x == x]
    if not v:
        return '<p class="muted">no samples</p>'
    lo, hi = min(v), max(v)
    if hi <= lo:
        hi = lo + 1e-6
    w = (hi - lo) / bins
    counts = [0] * bins
    for x in v:
        counts[min(bins - 1, int((x - lo) / w))] += 1
    labels = [f'{lo + i * w:.3g}' for i in range(bins)]
    notes = [f'{c} samples ({100.0 * c / len(v):.1f} %)' for c in counts]
    # Bins are categorical in x, so a reference line at a DATA value (e.g.
    # fit_ratio = 1) cannot be drawn by Axes.rule -- the caller gets the bin
    # edges in the tick labels and the tooltip instead.
    return bar_chart(labels, counts, xlabel=xlabel, ylabel='samples',
                     height=height, notes=notes)


class Report:
    """Accumulates sections, then renders one standalone HTML file."""

    def __init__(self, title, subtitle=''):
        self.title = title
        self.subtitle = subtitle
        self.body = []
        self.toc = []

    def section(self, heading, anchor=None):
        a = anchor or heading.lower().replace(' ', '-').replace('/', '')
        self.toc.append((heading, a))
        self.body.append(f'<h2 id="{esc(a)}">{esc(heading)}</h2>')
        return self

    def p(self, text, cls=''):
        self.body.append(f'<p class="{cls}">{text}</p>')
        return self

    def verdict(self, level, text):
        """level: good | warn | bad | info. Never colour alone -- each ships
        with a word, because a red block is meaningless in greyscale."""
        word = {'good': 'OK', 'warn': 'CHECK', 'bad': 'PROBLEM',
                'info': 'NOTE'}[level]
        self.body.append(
            f'<div class="verdict v-{level}"><b>{word}</b><span>{text}</span></div>')
        return self

    def tiles(self, items):
        """items: [(label, value, note)] -- headline numbers as stat tiles
        rather than as a one-bar bar chart."""
        cells = ''.join(
            f'<div class="tile"><div class="tl">{esc(lab)}</div>'
            f'<div class="tv">{val}</div>'
            f'<div class="tn">{esc(note)}</div></div>'
            for lab, val, note in items)
        self.body.append(f'<div class="tiles">{cells}</div>')
        return self

    def table(self, headers, rows, cls=''):
        h = ''.join(f'<th>{esc(x)}</th>' for x in headers)
        r = ''.join('<tr>' + ''.join(f'<td>{esc(c)}</td>' for c in row) + '</tr>'
                    for row in rows)
        self.body.append(
            f'<div class="tw"><table class="{cls}"><thead><tr>{h}</tr></thead>'
            f'<tbody>{r}</tbody></table></div>')
        return self

    def figure(self, svg, caption='', note=''):
        self.body.append(
            f'<figure>{svg}<figcaption><b>{esc(caption)}</b>'
            + (f' {esc(note)}' if note else '') + '</figcaption></figure>')
        return self

    def raw(self, h):
        self.body.append(h)
        return self

    def render(self):
        ramps_l = '\n'.join(f'  --ramp{i}: {c};'
                            for i, c in enumerate(RAMP_LIGHT))
        ramps_d = '\n'.join(f'    --ramp{i}: {c};'
                            for i, c in enumerate(RAMP_DARK))
        base_l = '\n'.join(f'  --{k}: {v};' for k, v in LIGHT.items())
        base_d = '\n'.join(f'    --{k}: {v};' for k, v in DARK.items())
        nav = ''.join(f'<a href="#{esc(a)}">{esc(t)}</a>' for t, a in self.toc)
        return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{esc(self.title)}</title>
<style>
:root {{
  color-scheme: light;
{base_l}
{ramps_l}
}}
@media (prefers-color-scheme: dark) {{
  :root:not([data-theme="light"]) {{
    color-scheme: dark;
{base_d}
{ramps_d}
  }}
}}
:root[data-theme="dark"] {{
  color-scheme: dark;
{base_d}
{ramps_d}
}}
* {{ box-sizing: border-box; }}
body {{ margin:0; background:var(--surface); color:var(--ink);
  font:14px/1.55 ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
  -webkit-text-size-adjust:100%; }}
.wrap {{ max-width:1000px; margin:0 auto; padding:28px 20px 80px; }}
header {{ display:flex; align-items:flex-start; gap:16px;
  border-bottom:1px solid var(--edge); padding-bottom:16px; margin-bottom:8px; }}
h1 {{ font-size:22px; margin:0 0 4px; letter-spacing:-.01em; }}
.sub {{ color:var(--ink2); font-size:13px; margin:0; }}
h2 {{ font-size:15px; margin:34px 0 10px; letter-spacing:.04em;
  text-transform:uppercase; color:var(--ink2); font-weight:600; }}
p {{ margin:8px 0; color:var(--ink2); max-width:74ch; }}
p.lead {{ color:var(--ink); }}
.muted {{ color:var(--ink3); }}
code {{ font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:.92em;
  background:var(--mid); padding:1px 5px; border-radius:4px; }}
nav {{ display:flex; flex-wrap:wrap; gap:4px 14px; font-size:12px;
  padding:10px 0 2px; border-bottom:1px solid var(--edge); }}
nav a {{ color:var(--ink2); text-decoration:none; }}
nav a:hover {{ color:var(--series1); text-decoration:underline; }}
button.tt {{ margin-left:auto; background:var(--panel); color:var(--ink2);
  border:1px solid var(--edge); border-radius:7px; padding:6px 11px;
  font:inherit; font-size:12px; cursor:pointer; white-space:nowrap; }}
.tiles {{ display:grid; gap:10px; margin:14px 0;
  grid-template-columns:repeat(auto-fit,minmax(158px,1fr)); }}
.tile {{ background:var(--panel); border:1px solid var(--edge);
  border-radius:10px; padding:12px 14px; }}
.tl {{ font-size:11px; letter-spacing:.05em; text-transform:uppercase;
  color:var(--ink3); }}
.tv {{ font-size:26px; font-weight:600; margin:3px 0 1px; color:var(--ink);
  font-variant-numeric:tabular-nums; letter-spacing:-.02em; }}
.tn {{ font-size:12px; color:var(--ink2); }}
figure {{ margin:16px 0 22px; background:var(--panel);
  border:1px solid var(--edge); border-radius:12px; padding:14px 12px 10px; }}
figcaption {{ font-size:12.5px; color:var(--ink2); margin-top:8px;
  padding:0 4px; max-width:none; }}
figcaption b {{ color:var(--ink); font-weight:600; }}
svg {{ display:block; overflow:visible; }}
text {{ font:11px ui-sans-serif,system-ui,sans-serif; fill:var(--ink3); }}
text.tick {{ font-variant-numeric:tabular-nums; }}
text.axlabel {{ fill:var(--ink2); font-size:11.5px; }}
text.rulelabel {{ fill:var(--ink3); font-size:10.5px; }}
.grid {{ stroke:var(--grid); stroke-width:1; }}
.axis {{ stroke:var(--edge); stroke-width:1; }}
.rule {{ stroke:var(--warn); stroke-width:1.5; stroke-dasharray:5 4; }}
.ln {{ fill:none; stroke-width:2; stroke-linejoin:round; stroke-linecap:round; }}
.ln.ctx {{ stroke-width:1.5; opacity:.42; }}
.ln.ctx:hover {{ opacity:.95; stroke-width:2.5; }}
.seg {{ stroke-width:5; stroke-linecap:round; }}
.dot {{ stroke:var(--panel); stroke-width:2; }}
.traj-true {{ fill:none; stroke:var(--ink3); stroke-width:1.5; opacity:.55; }}
.traj-est {{ fill:none; stroke:var(--series2); stroke-width:1.5;
  stroke-dasharray:4 3; opacity:.85; }}
.band .cross {{ stroke:var(--ink3); stroke-width:1; opacity:0; }}
.band:hover .cross {{ opacity:.5; }}
.bar:hover {{ opacity:.75; }}
.legend {{ display:flex; flex-wrap:wrap; gap:6px 18px; padding:10px 4px 0;
  font-size:12px; color:var(--ink2); }}
.lg {{ display:inline-flex; align-items:center; gap:6px; }}
.lg i {{ width:13px; height:13px; border-radius:3px; display:inline-block;
  flex:none; }}
.sw-true {{ background:var(--ink3); }}
.sw-est {{ background:var(--series2); }}
.sw-ramp {{ background:linear-gradient(90deg,var(--ramp0),var(--ramp9)); }}
.tw {{ overflow-x:auto; margin:12px 0; }}
table {{ border-collapse:collapse; font-size:13px; width:100%;
  font-variant-numeric:tabular-nums; }}
th, td {{ text-align:right; padding:6px 12px;
  border-bottom:1px solid var(--edge); white-space:nowrap; }}
th:first-child, td:first-child {{ text-align:left; }}
th {{ color:var(--ink3); font-weight:600; font-size:11px;
  letter-spacing:.05em; text-transform:uppercase; }}
.verdict {{ display:flex; gap:12px; align-items:baseline; margin:8px 0;
  padding:11px 14px; border-radius:9px; border:1px solid var(--edge);
  background:var(--panel); border-left-width:4px; }}
.verdict b {{ font-size:11px; letter-spacing:.06em; flex:none; min-width:62px; }}
.verdict span {{ color:var(--ink2); }}
.v-good {{ border-left-color:var(--good); }} .v-good b {{ color:var(--good); }}
.v-warn {{ border-left-color:var(--warn); }} .v-warn b {{ color:var(--warn); }}
.v-bad  {{ border-left-color:var(--bad);  }} .v-bad  b {{ color:var(--bad); }}
.v-info {{ border-left-color:var(--ink3); }} .v-info b {{ color:var(--ink3); }}
@media (max-width:640px) {{ .tv {{ font-size:22px; }} header {{ flex-wrap:wrap; }} }}
</style></head><body><div class="wrap">
<header><div><h1>{esc(self.title)}</h1>
<p class="sub">{esc(self.subtitle)}</p></div>
<button class="tt" onclick="var r=document.documentElement,
d=r.getAttribute('data-theme')==='dark';r.setAttribute('data-theme',d?'light':'dark')">
light / dark</button></header>
<nav>{nav}</nav>
{''.join(self.body)}
</div></body></html>"""

    def write(self, path):
        with open(path, 'w') as f:
            f.write(self.render())
        return path
