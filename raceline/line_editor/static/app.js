'use strict';
// Shared state, server calls, undo, the controls that both views use.
const $ = id => document.getElementById(id);
const E = window.editor = {};
const S = E.S = {
  track: '', name: '', stem: '', safety: 0.15, phys: null,
  map: null, meta: null,                   // background image and its world placement
  x: [], y: [],                            // the geometry being edited
  orig: null, origV: null, origS: null, origParams: null,   // as loaded
  prof: null,                              // last profile: s, psi, kappa, w_r, w_l, v, v_phys, body, a_lat, t, ...
  params: null,                            // a_lat, a_long, a_brake, v_max, v_zones, lat_zones, resweep_edits
  dv: [],                                  // hand-set speed edits per point, m/s, on top of the profile
  region: [],                              // per-point weight of the geometry edits since the last "forget"
  t_file: null,
  view: 'map', tool: 'draw',
  hover: -1, busy: false, req: 0, dragging: false,
  fastBusy: false, fastPending: false, fastCount: 0,
  est: null,                               // client-side estimate while a stroke is in progress
  undo: [], redo: [],
};

// ---- helpers ------------------------------------------------------------------
E.api = async function (path, body) {
  const r = await fetch(path, body ? { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                      body: JSON.stringify(body) } : undefined);
  const j = await r.json().catch(() => ({ error: r.statusText }));
  if (!r.ok) { const e = new Error(j.error || r.statusText); e.status = r.status; throw e; }
  return j;
};
E.clone = o => JSON.parse(JSON.stringify(o));
E.fmt = (v, d = 2) => (v == null || Number.isNaN(v)) ? '—' : v.toFixed(d);
E.turbo = function (t) {
  t = Math.min(1, Math.max(0, t));
  const r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));
  const g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));
  const b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));
  const c = v => Math.round(255 * Math.min(1, Math.max(0, v)));
  return `rgb(${c(r)},${c(g)},${c(b)})`;
};
E.bump = u => u < 1 ? (1 - u * u) ** 2 : 0;
E.vRange = () => { const v = S.prof && S.prof.v; return v ? [Math.min(...v), Math.max(...v)] : [0, 1]; };
E.error = msg => { $('status').innerHTML = `<span class="warn">${msg}</span>`; };

// arc length along the current geometry (closed): per-point s, segment lengths, total
E.arc = function () {
  const n = S.x.length, s = new Float64Array(n), el = new Float64Array(n); let acc = 0;
  for (let i = 0; i < n; i++) { const j = (i + 1) % n; s[i] = acc; el[i] = Math.hypot(S.x[j] - S.x[i], S.y[j] - S.y[i]); acc += el[i]; }
  return { s, el, L: acc };
};
// arc distance from point i0 to every other point, the shorter way round
E.arcDist = function (i0) {
  const n = S.x.length, d = new Float64Array(n).fill(Infinity); d[i0] = 0;
  let acc = 0;
  for (let k = 1; k < n; k++) { const a = (i0 + k - 1) % n, b = (i0 + k) % n; acc += Math.hypot(S.x[b] - S.x[a], S.y[b] - S.y[a]); d[b] = Math.min(d[b], acc); }
  acc = 0;
  for (let k = 1; k < n; k++) { const a = (i0 - k + 1 + n) % n, b = (i0 - k + n) % n; acc += Math.hypot(S.x[b] - S.x[a], S.y[b] - S.y[a]); d[b] = Math.min(d[b], acc); }
  return d;
};
E.brushWeights = function (i0, R) {
  const d = E.arcDist(i0), w = new Float64Array(d.length);
  for (let k = 0; k < d.length; k++) w[k] = E.bump(d[k] / R);
  return w;
};
// Gaussian smoothing along s of a closed array, taken at each point by weight w[i] (0 = untouched)
E.gaussSmooth = function (arr, sigma, w, s, L) {
  const n = arr.length, out = arr.slice(); if (sigma <= 0) return out;
  const k = Math.min(n >> 1, Math.ceil(3 * sigma / (L / n)));
  for (let i = 0; i < n; i++) {
    if (!(w[i] > 0)) continue;
    let num = 0, den = 0;
    for (let j = -k; j <= k; j++) {
      const idx = (i + j + n) % n; let d = Math.abs(s[idx] - s[i]); if (d > L / 2) d = L - d;
      const g = Math.exp(-0.5 * (d / sigma) ** 2); num += g * arr[idx]; den += g;
    }
    out[i] = arr[i] + Math.min(1, w[i]) * (num / den - arr[i]);
  }
  return out;
};
E.indexAtS = function (s) {           // nearest index by s on the last profile
  const a = S.prof.s; let lo = 0, hi = a.length - 1;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (a[mid] < s) lo = mid + 1; else hi = mid; }
  if (lo > 0 && Math.abs(a[lo - 1] - s) < Math.abs(a[lo] - s)) lo--;
  return lo;
};
// lap time and the demands implied by the current v, computed here so a stroke reads instantly
E.estimate = function () {
  const p = S.prof; if (!p) return null;
  const { el } = E.arc(), v = p.v, n = v.length, ax = new Float64Array(n);
  let t = 0, axmax = -Infinity, axmin = Infinity, latmax = 0, lats = 0;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n, h = (i - 1 + n) % n;
    t += 2 * el[i] / (v[i] + v[j]);
    ax[i] = (v[j] ** 2 - v[h] ** 2) / (4 * Math.max(el[i], 1e-6));      // np.gradient(v^2) / (2 el)
    axmax = Math.max(axmax, ax[i]); axmin = Math.min(axmin, ax[i]);
    const a = v[i] ** 2 * Math.abs(p.kappa[i]); if (a > latmax) { latmax = a; lats = p.s[i]; }
  }
  return { t, ax, ax_max: axmax, ax_min: axmin, lat_max: latmax, lat_max_s: lats };
};
E.axArray = () => E.estimate().ax;
// after the edit layer changes: the curve, its lateral demand and the estimate, without the server
E.localUpdate = function () {
  const p = S.prof; if (!p) return;
  for (let i = 0; i < p.v.length; i++) { p.v[i] = Math.max(0.3, p.v_phys[i] + S.dv[i]); p.a_lat[i] = p.v[i] ** 2 * Math.abs(p.kappa[i]); }
  S.est = E.estimate();
};

// ---- profile round trips ----------------------------------------------------------
let timer = null;
E.scheduleRecompute = (ms = 250) => { clearTimeout(timer); timer = setTimeout(() => E.recompute(), ms); };
E.recompute = async function (fast = false) {
  clearTimeout(timer);
  if (fast) { if (S.fastBusy) { S.fastPending = true; return; } S.fastBusy = true; S.fastCount++; }
  else { S.busy = true; E.updateStatus(); }
  const id = ++S.req;
  try {
    const r = await E.api('/api/profile', { x: S.x, y: S.y, params: S.params, dv: S.dv, fast });
    if (id === S.req) {
      if (fast && S.prof) { r.w_r = S.prof.w_r; r.w_l = S.prof.w_l; }     // the fast pass skips the widths
      S.prof = r; S.est = null;
    }
  } catch (e) { E.error(e.message); }
  finally {
    if (fast) { S.fastBusy = false; if (S.fastPending) { S.fastPending = false; if (S.dragging) E.recompute(true); } }
    else if (id === S.req) S.busy = false;
  }
  E.draw(); E.readout(S.hover);
};

// ---- undo ------------------------------------------------------------------------------
E.current = () => ({ x: S.x.slice(), y: S.y.slice(), dv: S.dv.slice(), region: S.region.slice(), params: E.clone(S.params) });
E.snapshot = () => { S.undo.push(E.current()); if (S.undo.length > 100) S.undo.shift(); S.redo.length = 0; };
E.restore = snap => { S.x = snap.x.slice(); S.y = snap.y.slice(); S.dv = snap.dv.slice(); S.region = snap.region.slice(); S.params = E.clone(snap.params); E.syncInputs(); E.renderZones(); E.recompute(); };
E.undo = () => { if (!S.undo.length) return; S.redo.push(E.current()); E.restore(S.undo.pop()); };
E.redo = () => { if (!S.redo.length) return; S.undo.push(E.current()); E.restore(S.redo.pop()); };

// ---- status, readout ------------------------------------------------------------------
E.updateStatus = function () {
  const p = S.prof, el = $('status'); if (!p || !S.params) { el.textContent = ''; return; }
  const e = S.est, t = e ? e.t : p.t, axmax = e ? e.ax_max : p.ax_max, axmin = e ? e.ax_min : p.ax_min;
  const latmax = e ? e.lat_max : p.lat_max, lats = e ? e.lat_max_s : p.lat_max_s;
  const bad = p.body_min < S.safety, overA = axmax > S.params.a_long + 0.1, overB = -axmin > S.params.a_brake + 0.1;
  // red when the demand exceeds what the plan itself allows anywhere (rung or the highest lateral zone)
  const tire = latmax > Math.max(S.params.a_lat, ...S.params.lat_zones.map(z => z[2])) + 0.15;
  el.innerHTML =
    `<span class="stat" title="lap time on paper: segment length over segment mean speed"><b>${t.toFixed(3)} s</b>${e ? ' <span class="est">est.</span>' : ''}${S.t_file != null ? ` <i>file ${S.t_file.toFixed(3)}</i>` : ''}</span>` +
    `<span class="stat">${p.length.toFixed(2)} m</span>` +
    `<span class="stat" title="tightest body-to-wall margin and where">body <span class="${bad ? 'warn' : 'ok'}">${p.body_min.toFixed(2)}</span> @ ${p.body_min_s.toFixed(1)}</span>` +
    `<span class="stat" title="peak lateral demand v²|κ| and where; the tire holds about 7">lat <span class="${tire ? 'warn' : ''}">${latmax.toFixed(2)}</span> @ ${lats.toFixed(1)}</span>` +
    `<span class="stat" title="peak acceleration / braking the profile asks for, against a_long / a_brake">long <span class="${overA ? 'warn' : ''}">+${axmax.toFixed(1)}</span> / <span class="${overB ? 'warn' : ''}">${axmin.toFixed(1)}</span></span>` +
    `<span class="stat" title="steering rate the path needs at profile speed, rad/s">steer ${p.steer_rate_max.toFixed(1)}</span>` +
    (S.busy ? '<span class="stat"><b>computing…</b></span>' : '');
  const k = S.dv.filter(d => d !== 0).length;
  $('e_info').textContent = k ? `${k} points edited, ${Math.min(...S.dv).toFixed(2)} to +${Math.max(...S.dv).toFixed(2)} m/s`
                              + (S.params.resweep_edits ? '; budgets re-imposed, a step shows as its ramp' : '; taken as drawn')
                            : 'no edits';
  const r = S.region.filter(w => w > 0).length;
  $('g_region').textContent = r ? `edited region: ${r} points` : 'no edited region yet: drag the line to create one';
};
E.readout = function (i) {
  const p = S.prof, f = E.fmt;
  if (i < 0 || !p) { $('readout').textContent = 'hover the line'; return; }
  const w = p.w_r ? `width R ${f(p.w_r[i])}  L ${f(p.w_l[i])} m   ` : '';
  $('readout').textContent =
    `s ${f(p.s[i])} m    x ${f(S.x[i], 3)}   y ${f(S.y[i], 3)}\n` +
    `v ${f(p.v[i])} m/s${S.dv[i] ? `   (profile ${f(p.v_phys[i])} ${S.dv[i] > 0 ? '+' : ''}${f(S.dv[i])})` : ''}\n` +
    `kappa ${p.kappa[i] >= 0 ? '+' : ''}${f(p.kappa[i], 3)} 1/m   demand ${f(p.a_lat[i])} m/s²\n` +
    `${w}body ${f(p.body[i])} m`;
};
E.draw = function () {
  if (S.view === 'map') { E.drawMap(); E.drawCurv(); } else E.drawSpeed();
  E.updateStatus();
};

// ---- controls shared by both views ---------------------------------------------------
E.syncInputs = function () {
  for (const k of ['a_lat', 'a_long', 'a_brake', 'v_max']) $(k).value = S.params[k];
  $('resweep').checked = !!S.params.resweep_edits;
};
E.renderZones = function () {
  for (const [key, tbl] of [['v_zones', $('vz')], ['lat_zones', $('lz')]]) {
    tbl.innerHTML = '';
    S.params[key].forEach((z, i) => {
      const tr = document.createElement('tr');
      z.forEach((val, j) => {
        const td = document.createElement('td'), inp = document.createElement('input');
        inp.type = 'number'; inp.step = '0.1'; inp.value = val;
        inp.addEventListener('change', () => { E.snapshot(); S.params[key][i][j] = parseFloat(inp.value) || 0; E.scheduleRecompute(); });
        td.appendChild(inp); tr.appendChild(td);
        if (j < 2) { const sep = document.createElement('td'); sep.textContent = ':'; tr.appendChild(sep); }
      });
      const td = document.createElement('td'), btn = document.createElement('button');
      btn.textContent = '×'; btn.title = 'remove'; btn.className = 'link';
      btn.addEventListener('click', () => { E.snapshot(); S.params[key].splice(i, 1); E.renderZones(); E.recompute(); });
      td.appendChild(btn); tr.appendChild(td); tbl.appendChild(tr);
    });
  }
};
E.addZone = function (key, s0, s1, lvl) {
  E.snapshot();
  S.params[key].push([Math.round(s0 * 10) / 10, Math.round(s1 * 10) / 10, Math.round(lvl * 10) / 10]);
  E.renderZones(); E.recompute();
};
E.removeZoneAt = function (key, s) {
  const zs = S.params[key];
  for (let i = zs.length - 1; i >= 0; i--) if (s >= zs[i][0] && s <= zs[i][1]) { E.snapshot(); zs.splice(i, 1); E.renderZones(); E.recompute(); return true; }
  return false;
};
E.resampleArray = function (arr, n) {            // an edit layer follows a resample by fractional index
  const m = arr.length; if (!m) return new Array(n).fill(0);
  return Array.from({ length: n }, (_, i) => { const u = i * m / n, a = Math.floor(u) % m, b = (a + 1) % m, f = u - Math.floor(u); return arr[a] * (1 - f) + arr[b] * f; });
};
E.bumpRange = function (s0, s1, d) {
  const lo = Math.min(s0, s1), hi = Math.max(s0, s1);
  S.prof.s.forEach((s, i) => { if (s >= lo && s <= hi) S.dv[i] = Math.round((S.dv[i] + d) * 1000) / 1000; });
};

// ---- file -------------------------------------------------------------------------------
E.fillFiles = function (files) {
  const sel = $('files'); sel.innerHTML = '';
  for (const f of files) { const o = document.createElement('option'); o.value = o.textContent = f; sel.appendChild(o); }
  sel.value = S.name;
};
E.applyLine = function (r) {
  S.x = r.x.slice(); S.y = r.y.slice();
  S.orig = { x: r.x.slice(), y: r.y.slice() }; S.origV = r.v_file; S.origS = r.s.slice();
  S.prof = r; S.params = E.clone(r.params); S.origParams = E.clone(r.params);
  S.dv = (r.dv && r.dv.length === r.x.length) ? r.dv.slice() : new Array(r.x.length).fill(0);
  S.region = new Array(r.x.length).fill(0);
  S.name = r.name; S.stem = r.stem; S.t_file = r.t_file; S.hover = -1; S.est = null;
  S.undo.length = 0; S.redo.length = 0;
  $('title').textContent = `${S.track} / ${r.name}`;
  $('name').value = r.stem.replace(/_edit\d*$/, '') + '_edit';
  $('n').value = r.x.length;
  E.syncInputs(); E.renderZones(); E.readout(-1);
  const off = r.t_file != null ? Math.abs(r.t - r.t_file) : 0;
  $('note').innerHTML = off > 0.05
    ? `<span class="warn">arguments inferred, not recorded: recomputed under them this line is ${off.toFixed(3)} s off its own speed column. Set the zones and budgets to what built it, or accept the new profile, before editing.</span>`
    : (r.t_file != null ? `profile reproduces the file (${off.toFixed(3)} s apart)` : 'no speed column in this file: the profile is computed from the arguments');
};
E.load = async function (name) {
  try { E.applyLine(await E.api('/api/load', { name })); $('files').value = name; $('saved').textContent = ''; E.draw(); }
  catch (e) { E.error(e.message); }
};
E.save = async function (overwrite) {
  const name = $('name').value.trim(); if (!name) return;
  try {
    const r = await E.api('/api/save', { name, x: S.x, y: S.y, params: S.params, dv: S.dv, overwrite: !!overwrite });
    $('saved').textContent = `saved ${r.path}: ${r.t.toFixed(3)} s on paper, body min ${r.body_min.toFixed(2)} m, lat max ${r.lat_max.toFixed(2)}`;
    E.fillFiles(r.files); $('files').value = S.name;
  } catch (e) {
    if (e.status === 409) { if (confirm(`${name} exists. Overwrite?`)) return E.save(true); return; }
    $('saved').innerHTML = `<span class="warn">save failed: ${e.message}</span>`;
  }
};

// ---- views, tools, keys ----------------------------------------------------------------
const HINTS = {
  map: 'drag a point to move the line (brush falloff) · drag empty space or right-drag to pan · wheel to zoom · the strip below is curvature: hover it to find a spot on the map',
  draw: 'draw on the speed panel: the curve follows the mouse over the s-range you sweep and is smoothed on release · lateral panel: drag a level for a lateral zone',
  smooth: 'drag over the speed panel to smooth your edits under the brush; keep moving for more',
  erase: 'drag over the speed panel to erase edits under the brush, back to the physics profile',
  zone: 'drag a level across the speed panel for a speed zone, across the lateral panel for a lateral zone · shift-click a band to remove it',
};
E.setView = function (v) {
  S.view = v; S.hover = -1;
  document.querySelectorAll('.view').forEach(el => el.classList.toggle('active', el.id === 'view-' + v));
  document.querySelectorAll('.tab').forEach(el => el.classList.toggle('active', el.dataset.view === v));
  document.querySelectorAll('.for-map').forEach(el => el.hidden = v !== 'map');
  document.querySelectorAll('.for-speed').forEach(el => el.hidden = v !== 'speed');
  $('hint').textContent = v === 'map' ? HINTS.map : HINTS[S.tool];
  E.resize();
};
E.setTool = function (t) {
  S.tool = t;
  document.querySelectorAll('.tool').forEach(el => el.classList.toggle('active', el.dataset.tool === t));
  if (S.view === 'speed') $('hint').textContent = HINTS[t];
  if (S.view === 'speed') E.drawSpeed();
};
E.resize = function () { if (S.view === 'map') E.resizeMap(); else E.resizeSpeed(); E.draw(); };

function bindControls() {
  for (const k of ['a_lat', 'a_long', 'a_brake', 'v_max'])
    $(k).addEventListener('change', () => { E.snapshot(); S.params[k] = parseFloat($(k).value) || S.params[k]; E.recompute(); });
  $('recompute').addEventListener('click', () => E.recompute());
  $('vz_add').addEventListener('click', () => E.addZone('v_zones', 0, 5, S.params.v_max));
  $('lz_add').addEventListener('click', () => E.addZone('lat_zones', 0, 5, S.params.a_lat));
  $('e_apply').addEventListener('click', () => {
    const d = parseFloat($('e_dv').value); if (!S.prof || !d) return;
    E.snapshot(); E.bumpRange(parseFloat($('e_s0').value) || 0, parseFloat($('e_s1').value) || 0, d); E.recompute();
  });
  $('e_clear').addEventListener('click', () => { if (!S.dv.some(d => d !== 0)) return; E.snapshot(); S.dv.fill(0); E.recompute(); });
  $('e_file').addEventListener('click', () => {
    if (!S.origV || !S.prof || S.origV.length !== S.prof.v_phys.length) return;
    E.snapshot(); S.dv = S.origV.map((v, i) => Math.round((v - S.prof.v_phys[i]) * 1000) / 1000); E.recompute();
  });
  $('resweep').addEventListener('change', () => { E.snapshot(); S.params.resweep_edits = $('resweep').checked; E.recompute(); });
  $('undo').addEventListener('click', E.undo);
  $('redo').addEventListener('click', E.redo);
  $('fit').addEventListener('click', () => E.fit());
  $('reset').addEventListener('click', () => {
    E.snapshot(); S.x = S.orig.x.slice(); S.y = S.orig.y.slice(); S.dv.fill(0); S.region.fill(0);
    S.params = E.clone(S.origParams); E.syncInputs(); E.renderZones(); E.recompute();
  });
  $('resample').addEventListener('click', async () => {
    E.snapshot();
    try {
      const r = await E.api('/api/resample', { x: S.x, y: S.y, n: parseInt($('n').value) || S.x.length, smooth: parseFloat($('smooth').value) || 0 });
      S.dv = E.resampleArray(S.dv, r.x.length); S.region = E.resampleArray(S.region, r.x.length);
      S.x = r.x; S.y = r.y; S.hover = -1; E.recompute();
    } catch (e) { E.error(e.message); }
  });
  $('load').addEventListener('click', () => E.load($('files').value));
  $('save').addEventListener('click', () => E.save(false));
  document.querySelectorAll('.tab').forEach(el => el.addEventListener('click', () => E.setView(el.dataset.view)));
  document.querySelectorAll('.tool').forEach(el => el.addEventListener('click', () => E.setTool(el.dataset.tool)));
  for (const [id, out, unit] of [['brush', 'brushv', ' m'], ['g_radius', 'g_radiusv', ' m'], ['g_strength', 'g_strengthv', ''],
                                 ['s_radius', 's_radiusv', ' m'], ['s_strength', 's_strengthv', ''], ['s_stroke', 's_strokev', ' m']])
    $(id).addEventListener('input', () => { $(out).textContent = (unit ? (+$(id).value).toFixed(1) : (+$(id).value).toFixed(2)) + unit; if (S.view === 'speed') E.drawSpeed(); });
  const help = on => { $('helpbox').hidden = !on; };
  $('help').addEventListener('click', () => help(true));
  $('help_close').addEventListener('click', () => help(false));
  $('helpbox').addEventListener('click', e => { if (e.target === $('helpbox')) help(false); });
  window.addEventListener('keydown', e => {
    const inInput = /INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName);
    const k = e.key.toLowerCase();
    if ((e.ctrlKey || e.metaKey) && k === 's') { e.preventDefault(); E.save(false); return; }
    if (inInput) return;
    if ((e.ctrlKey || e.metaKey) && k === 'z') { e.preventDefault(); e.shiftKey ? E.redo() : E.undo(); return; }
    if ((e.ctrlKey || e.metaKey) && k === 'y') { e.preventDefault(); E.redo(); return; }
    if (e.ctrlKey || e.metaKey || e.altKey) return;
    if (k === '1') E.setView('map'); else if (k === '2') E.setView('speed');
    else if (k === 'f') E.fit();
    else if (k === '?') help(true); else if (k === 'escape') help(false);
    else if (S.view === 'speed' && { d: 'draw', s: 'smooth', e: 'erase', z: 'zone' }[k]) E.setTool({ d: 'draw', s: 'smooth', e: 'erase', z: 'zone' }[k]);
  });
}
E.bindControls = bindControls;
