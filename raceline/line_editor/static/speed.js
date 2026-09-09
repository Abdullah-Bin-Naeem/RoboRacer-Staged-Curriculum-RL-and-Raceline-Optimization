'use strict';
// Speed view: the profile full-window in three panels (speed, lateral demand,
// longitudinal demand) with the draw / smooth / erase / zone tools, and a
// linked minimap in the sidebar. The lap is a closed loop, so the s-axis
// carries a wrap margin at both ends where the loop continues: the curve
// closes on itself and a stroke can cross the start line. Strokes update the
// curve and the estimate locally; the server confirms on release.
(() => {
  const S = E.S;
  const C = $('speed'), ctx = C.getContext('2d');
  const MC = $('minimap'), mctx = MC.getContext('2d');
  const G = E.speedGeom = { x0: 52, x1: 0, W: 0, H: 0, pan: {}, vTop: 10, aTop: 8, axTop: 8, L: 1, m: 3 };
  const MM = { w: 0, h: 0, k: 1, sx: 0, sy: 0 };          // minimap: world -> canvas
  let stroke = null, paint = null, cursorS = null;

  E.resizeSpeed = function () {
    const dpr = window.devicePixelRatio || 1, host = $('view-speed');
    G.W = host.clientWidth; G.H = host.clientHeight;
    C.width = G.W * dpr; C.height = G.H * dpr; C.style.width = G.W + 'px'; C.style.height = G.H + 'px';
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    G.x1 = G.W - 20;
    // each panel gets a 38 px gap above it: the previous panel's s labels and this one's title
    const avail = G.H - 3 * 38 - 10, top = 38;
    G.pan.v = { y0: top, h: avail * 0.48, name: 'v' };
    G.pan.lat = { y0: top + avail * 0.48 + 38, h: avail * 0.26, name: 'lat' };
    G.pan.ax = { y0: top + avail * 0.74 + 76, h: avail * 0.26, name: 'ax' };
    E.resizeMini();
  };
  E.resizeMini = function () {
    const m = S.meta; if (!m) return;
    const dpr = window.devicePixelRatio || 1, w = MC.clientWidth || 360;
    const h = Math.min(380, Math.round(w * (m.H / m.W)));
    MC.width = w * dpr; MC.height = h * dpr; MC.style.height = h + 'px';
    mctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    MM.w = w; MM.h = h;
    MM.k = 0.94 * Math.min(w / (m.W * m.res), h / (m.H * m.res));
    MM.sx = (w - m.W * m.res * MM.k) / 2; MM.sy = (h - m.H * m.res * MM.k) / 2;
  };
  const mini = (x, y) => [MM.sx + (x - S.meta.ox) * MM.k, MM.sy + (S.meta.oy + S.meta.H * S.meta.res - y) * MM.k];
  E.miniOf = i => mini(S.x[i], S.y[i]);

  // ---- axis with the wrap margin ---------------------------------------------------------
  const sToX = s => G.x0 + (s + G.m) / (G.L + 2 * G.m) * (G.x1 - G.x0);
  const xToS = px => (px - G.x0) / (G.x1 - G.x0) * (G.L + 2 * G.m) - G.m;
  const wrap = s => ((s % G.L) + G.L) % G.L;
  const yOf = (val, p, top, bottom = 0) => p.y0 + p.h - (val - bottom) / (top - bottom) * p.h;
  const valOf = (py, p, top, bottom = 0) => bottom + (p.y0 + p.h - py) / p.h * (top - bottom);
  const panelAt = py => Object.values(G.pan).find(p => py >= p.y0 && py <= p.y0 + p.h) || null;
  const range = p => p.name === 'v' ? [G.vTop, 0] : p.name === 'lat' ? [G.aTop, 0] : [G.axTop, -G.axTop];
  E.speedXY = (s, val, name) => { const p = G.pan[name], [top, bot] = range(p); return [sToX(s), yOf(val, p, top, bot)]; };

  // ---- drawing --------------------------------------------------------------------------
  function panel(p, top, bottom, label, yStep) {
    ctx.fillStyle = '#2b2e34'; ctx.fillRect(G.x0, p.y0, G.x1 - G.x0, p.h);
    ctx.fillStyle = 'rgba(0,0,0,0.28)';                                   // the wrap margins
    ctx.fillRect(G.x0, p.y0, sToX(0) - G.x0, p.h); ctx.fillRect(sToX(G.L), p.y0, G.x1 - sToX(G.L), p.h);
    ctx.font = '11px system-ui'; ctx.fillStyle = '#9aa0a8'; ctx.textAlign = 'right';
    for (let v = Math.ceil(bottom / yStep) * yStep; v <= top; v += yStep) {
      const y = yOf(v, p, top, bottom);
      ctx.strokeStyle = v === 0 ? '#5a5f68' : '#3a3e46'; ctx.beginPath(); ctx.moveTo(G.x0, y); ctx.lineTo(G.x1, y); ctx.stroke();
      ctx.fillText(v.toFixed(0), G.x0 - 6, y + 4);
    }
    ctx.textAlign = 'center';
    for (let s = 0; s <= G.L; s += 5) {
      const x = sToX(s); ctx.strokeStyle = '#3a3e46'; ctx.beginPath(); ctx.moveTo(x, p.y0); ctx.lineTo(x, p.y0 + p.h); ctx.stroke();
      ctx.fillText(s.toFixed(0), x, p.y0 + p.h + 14);
    }
    for (const s of [0, G.L]) {                                            // the start line, both ends
      const x = sToX(s); ctx.strokeStyle = '#8b90a0'; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(x, p.y0); ctx.lineTo(x, p.y0 + p.h); ctx.stroke(); ctx.setLineDash([]);
    }
    ctx.fillStyle = '#6b7280'; ctx.font = '10px system-ui';
    ctx.fillText('↺', (G.x0 + sToX(0)) / 2, p.y0 + p.h - 6); ctx.fillText('↺', (sToX(G.L) + G.x1) / 2, p.y0 + p.h - 6);
    ctx.textAlign = 'left'; ctx.fillStyle = '#d5d8de'; ctx.font = '600 12px system-ui'; ctx.fillText(label, G.x0, p.y0 - 8);
  }
  function band(p, top, bottom, s0, s1, lvl, color, text) {
    ctx.fillStyle = color.replace(/[\d.]+\)$/, '0.16)'); ctx.fillRect(sToX(s0), p.y0, sToX(s1) - sToX(s0), p.h);
    const y = yOf(Math.min(lvl, top), p, top, bottom);
    ctx.setLineDash([5, 3]); ctx.strokeStyle = color; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(sToX(s0), y); ctx.lineTo(sToX(s1), y); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = '#e8eaee'; ctx.font = '10px system-ui'; ctx.textAlign = 'left'; ctx.fillText(text, sToX(s0) + 3, y - 3);
  }
  // the closed loop, drawn three times (shifted by -L, 0, +L) and clipped, so the margins show the continuation
  function curve(p, top, bottom, ys, color, width = 1.8, dash = [], s = S.prof.s) {
    ctx.save(); ctx.beginPath(); ctx.rect(G.x0, p.y0, G.x1 - G.x0, p.h); ctx.clip();
    ctx.setLineDash(dash); ctx.strokeStyle = color; ctx.lineWidth = width;
    for (const k of [-1, 0, 1]) {
      ctx.beginPath();
      for (let i = 0; i < s.length; i++) { const x = sToX(s[i] + k * G.L), y = yOf(ys[i], p, top, bottom); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }
      ctx.lineTo(sToX(s[0] + (k + 1) * G.L), yOf(ys[0], p, top, bottom));
      ctx.stroke();
    }
    ctx.setLineDash([]); ctx.restore();
  }
  function hline(p, top, bottom, v, color, dash, label) {
    if (v > top || v < bottom) return;
    const y = yOf(v, p, top, bottom);
    ctx.setLineDash(dash); ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(G.x0, y); ctx.lineTo(G.x1, y); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = color; ctx.font = '10px system-ui'; ctx.textAlign = 'right'; ctx.fillText(label, G.x1 - 4, y - 3);
  }

  E.drawSpeed = function () {
    ctx.fillStyle = '#23262b'; ctx.fillRect(0, 0, G.W, G.H);
    const p = S.prof; if (!p || !S.params || !G.pan.v) return;
    G.L = p.length; G.m = Math.min(4, 0.06 * G.L);
    const zv = S.params.v_zones, zl = S.params.lat_zones, hasEdits = S.dv.some(d => d !== 0);
    const est = S.est || E.estimate(), ax = est.ax;
    G.vTop = Math.ceil(Math.max(S.params.v_max, ...zv.map(z => z[2]), ...p.v, S.origV ? Math.max(...S.origV) : 0) + 0.5);
    G.aTop = Math.ceil(Math.max(S.params.a_lat, ...zl.map(z => z[2]), ...p.a_lat, S.phys.a_lat_robust) + 0.5);
    G.axTop = Math.ceil(Math.max(S.params.a_long, S.params.a_brake, Math.min(20, Math.max(...ax.map(Math.abs)))) + 1);
    const pv = G.pan.v, pl = G.pan.lat, pa = G.pan.ax;
    panel(pv, G.vTop, 0, 'speed the plan demands [m/s]', 1);
    panel(pl, G.aTop, 0, 'lateral demand v²|κ| [m/s²]', 2);
    panel(pa, G.axTop, -G.axTop, 'longitudinal demand [m/s²]  (+ accelerating, − braking)', G.axTop > 12 ? 5 : 2);
    for (const [s0, s1, v] of zv) band(pv, G.vTop, 0, Math.max(0, s0), Math.min(G.L, s1), v, 'rgba(96,165,250,0.9)', `v ${v.toFixed(1)}`);
    for (const [s0, s1, a] of zl) band(pl, G.aTop, 0, Math.max(0, s0), Math.min(G.L, s1), a, 'rgba(248,113,113,0.9)', `lat ${a.toFixed(1)}`);
    hline(pv, G.vTop, 0, S.params.v_max, '#9aa0a8', [4, 3], 'v_max ' + S.params.v_max);
    hline(pl, G.aTop, 0, S.params.a_lat, '#e8eaee', [2, 3], 'rung ' + S.params.a_lat);
    hline(pl, G.aTop, 0, S.phys.a_lat_robust, '#9aa0a8', [6, 3], 'tire asymptote ' + S.phys.a_lat_robust.toFixed(2));
    hline(pa, G.axTop, -G.axTop, S.params.a_long, '#86efac', [4, 3], 'a_long ' + S.params.a_long);
    hline(pa, G.axTop, -G.axTop, -S.params.a_brake, '#fca5a5', [4, 3], 'a_brake ' + S.params.a_brake);
    if (S.origV && S.origS) curve(pv, G.vTop, 0, S.origV, 'rgba(160,160,160,0.55)', 1.2, [], S.origS);
    if (hasEdits) curve(pv, G.vTop, 0, p.v_phys, 'rgba(96,165,250,0.55)', 1.2, [4, 3]);
    curve(pv, G.vTop, 0, p.v, '#60a5fa', 2);
    curve(pl, G.aTop, 0, p.a_lat, '#f87171', 1.6);
    curve(pa, G.axTop, -G.axTop, Array.from(ax, a => Math.max(-G.axTop, Math.min(G.axTop, a))), '#fbbf24', 1.4);
    if (hasEdits) {                                                        // edited points, marked under the speed curve
      ctx.fillStyle = 'rgba(245,158,11,0.7)';
      for (let i = 0; i < p.s.length; i++) if (S.dv[i] !== 0) ctx.fillRect(sToX(p.s[i]) - 0.5, pv.y0 + pv.h - 3, 1.5, 3);
    }
    if (cursorS != null && (S.tool === 'smooth' || S.tool === 'erase') && !stroke) {   // brush preview
      const R = parseFloat($('s_radius').value);
      ctx.fillStyle = S.tool === 'smooth' ? 'rgba(245,158,11,0.15)' : 'rgba(248,113,113,0.15)';
      ctx.fillRect(sToX(cursorS - R), pv.y0, sToX(cursorS + R) - sToX(cursorS - R), pv.h);
    }
    if (S.hover >= 0) {
      const x = sToX(p.s[S.hover]);
      ctx.strokeStyle = '#fff'; ctx.setLineDash([2, 2]); ctx.beginPath(); ctx.moveTo(x, pv.y0); ctx.lineTo(x, pa.y0 + pa.h); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = '#fff';
      for (const [pn, top, bot, ys] of [[pv, G.vTop, 0, p.v], [pl, G.aTop, 0, p.a_lat], [pa, G.axTop, -G.axTop, ax]])
        { ctx.beginPath(); ctx.arc(x, yOf(Math.max(bot, Math.min(top, ys[S.hover])), pn, top, bot), 3.5, 0, 7); ctx.fill(); }
      ctx.font = '11px system-ui'; ctx.textAlign = 'left'; ctx.fillText(`s ${p.s[S.hover].toFixed(1)}`, x + 6, pv.y0 + 14);
    }
    if (paint) {
      const pn = paint.panel, [top, bot] = range(pn), y = yOf(paint.level, pn, top, bot), a = sToX(paint.s0), b = sToX(paint.s1);
      ctx.fillStyle = 'rgba(245,158,11,0.25)'; ctx.fillRect(Math.min(a, b), pn.y0, Math.abs(b - a), pn.h);
      ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(a, y); ctx.lineTo(b, y); ctx.stroke();
      ctx.fillStyle = '#fde68a'; ctx.font = '11px system-ui'; ctx.textAlign = 'left';
      ctx.fillText(`${Math.min(paint.s0, paint.s1).toFixed(1)}–${Math.max(paint.s0, paint.s1).toFixed(1)} m @ ${paint.level.toFixed(1)}`, Math.min(a, b) + 4, y - 5);
    }
    E.drawMini();
  };

  // ---- the minimap: where an s is on the track -------------------------------------------
  E.drawMini = function () {
    const p = S.prof; mctx.fillStyle = '#23262b'; mctx.fillRect(0, 0, MM.w, MM.h);
    if (!p || !S.map) return;
    const m = S.meta, [ix, iy] = mini(m.ox, m.oy + m.H * m.res);
    mctx.imageSmoothingEnabled = false; mctx.drawImage(S.map, ix, iy, m.W * m.res * MM.k, m.H * m.res * MM.k);
    const n = S.x.length, [vmin, vmax] = E.vRange();
    mctx.lineWidth = 2; mctx.lineCap = 'round';
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n, [ax, ay] = mini(S.x[i], S.y[i]), [bx, by] = mini(S.x[j], S.y[j]);
      mctx.strokeStyle = E.turbo((p.v[i] - vmin) / ((vmax - vmin) || 1));
      mctx.beginPath(); mctx.moveTo(ax, ay); mctx.lineTo(bx, by); mctx.stroke();
    }
    mctx.fillStyle = 'rgba(245,158,11,0.9)';                               // edited points
    for (let i = 0; i < n; i++) if (S.dv[i] !== 0) { const [x, y] = mini(S.x[i], S.y[i]); mctx.fillRect(x - 1, y - 1, 2, 2); }
    if (cursorS != null && (S.tool === 'smooth' || S.tool === 'erase')) {  // the brush's reach
      const R = parseFloat($('s_radius').value), c = wrap(cursorS);
      mctx.strokeStyle = 'rgba(255,255,255,0.8)'; mctx.lineWidth = 1;
      for (let i = 0; i < n; i++) { let d = Math.abs(p.s[i] - c); if (d > G.L / 2) d = G.L - d; if (d <= R) { const [x, y] = mini(S.x[i], S.y[i]); mctx.beginPath(); mctx.arc(x, y, 3, 0, 7); mctx.stroke(); } }
    }
    const [zx, zy] = mini(S.x[0], S.y[0]);                                 // s = 0
    mctx.fillStyle = '#fff'; mctx.beginPath(); mctx.moveTo(zx, zy - 5); mctx.lineTo(zx + 5, zy); mctx.lineTo(zx, zy + 5); mctx.lineTo(zx - 5, zy); mctx.fill();
    if (S.hover >= 0) {
      const [x, y] = mini(S.x[S.hover], S.y[S.hover]);
      mctx.strokeStyle = '#fff'; mctx.lineWidth = 3; mctx.beginPath(); mctx.arc(x, y, 7, 0, 7); mctx.stroke();
      mctx.strokeStyle = '#000'; mctx.lineWidth = 1; mctx.beginPath(); mctx.arc(x, y, 7, 0, 7); mctx.stroke();
      mctx.fillStyle = '#fff'; mctx.font = '11px system-ui'; mctx.textAlign = 'left';
      mctx.fillText(`s ${p.s[S.hover].toFixed(1)}  v ${p.v[S.hover].toFixed(2)}`, Math.min(x + 10, MM.w - 110), y + 4);
    }
  };
  function miniNearest(px, py) {
    let best = -1, bd = 14 * 14;
    for (let i = 0; i < S.x.length; i++) { const [x, y] = mini(S.x[i], S.y[i]), dd = (x - px) ** 2 + (y - py) ** 2; if (dd < bd) { bd = dd; best = i; } }
    return best;
  }
  MC.addEventListener('mousemove', e => {
    if (!S.prof) return;
    const i = miniNearest(e.offsetX, e.offsetY);
    if (i >= 0 && i !== S.hover) { S.hover = i; E.readout(i); E.drawSpeed(); }
  });
  MC.addEventListener('click', e => { const i = miniNearest(e.offsetX, e.offsetY); if (i >= 0) { S.hover = i; E.readout(i); E.drawSpeed(); } });

  // ---- tools ------------------------------------------------------------------------------
  function brushAt(s, R, strength) {
    const p = S.prof, n = p.s.length, w = new Float64Array(n), c = wrap(s);
    for (let i = 0; i < n; i++) { let d = Math.abs(p.s[i] - c); if (d > G.L / 2) d = G.L - d; w[i] = E.bump(d / R) * strength; }
    return w;
  }
  // draw: every point between the previous mouse sample and this one, as drawn; s may run past either end
  function applyDraw(s1, v1) {
    const { s: s0, v: v0 } = stroke.last, p = S.prof, lo = Math.min(s0, s1), hi = Math.max(s0, s1);
    for (let i = 0; i < p.s.length; i++) {
      for (const k of [-1, 0, 1]) {
        const si = p.s[i] + k * G.L; if (si < lo || si > hi) continue;
        const f = s1 !== s0 ? (si - s0) / (s1 - s0) : 0;
        S.dv[i] = Math.round((v0 + (v1 - v0) * f - p.v_phys[i]) * 1000) / 1000;
        break;
      }
    }
    stroke.lo = Math.min(stroke.lo, lo); stroke.hi = Math.max(stroke.hi, hi);
  }
  // on release: Gaussian-smooth the stroke, fading into the untouched curve beyond its ends (periodic)
  function smoothStroke() {
    const sigma = parseFloat($('s_stroke').value); if (sigma <= 0) return;
    const p = S.prof, w = new Float64Array(p.s.length);
    for (let i = 0; i < p.s.length; i++) {
      let out = Infinity;
      for (const k of [-1, 0, 1]) { const si = p.s[i] + k * G.L; out = Math.min(out, si < stroke.lo ? stroke.lo - si : si > stroke.hi ? si - stroke.hi : 0); }
      w[i] = Math.max(0, 1 - out / (2 * sigma));
    }
    S.dv = E.gaussSmooth(S.dv, sigma, w, p.s, G.L).map(d => Math.round(d * 1000) / 1000);
  }
  function applySmooth(s) {
    const R = parseFloat($('s_radius').value), k = parseFloat($('s_strength').value);
    S.dv = E.gaussSmooth(S.dv, R / 2, brushAt(s, R, k), S.prof.s, G.L).map(d => Math.round(d * 1000) / 1000);
  }
  function applyErase(s) {
    const w = brushAt(s, parseFloat($('s_radius').value), parseFloat($('s_strength').value));
    for (let i = 0; i < S.dv.length; i++) if (w[i] > 0) S.dv[i] = Math.abs(S.dv[i]) < 1e-3 ? 0 : Math.round(S.dv[i] * (1 - w[i]) * 1000) / 1000;
  }
  const clampAxis = s => Math.min(G.L + G.m, Math.max(-G.m, s));

  C.addEventListener('mousedown', e => {
    if (!S.prof || e.button !== 0) return;
    const pn = panelAt(e.offsetY); if (!pn) return;
    const [top, bot] = range(pn), s = clampAxis(xToS(e.offsetX));
    if (e.shiftKey && pn.name !== 'ax') { E.removeZoneAt(pn.name === 'v' ? 'v_zones' : 'lat_zones', wrap(s)); return; }
    const level = valOf(e.offsetY, pn, top, bot);
    if (S.tool === 'zone' || pn.name === 'lat') {
      if (pn.name === 'ax') return;
      const sc = Math.min(G.L, Math.max(0, s));
      paint = { panel: pn, s0: sc, s1: sc, level: Math.max(0.5, level) }; return;
    }
    if (pn.name !== 'v') return;
    E.snapshot(); S.dragging = true;
    stroke = { tool: S.tool, last: { s, v: Math.max(0.3, level) }, lo: s, hi: s, lastApply: -1e9 };
    if (S.tool === 'draw') applyDraw(s, stroke.last.v);
    else if (S.tool === 'smooth') { applySmooth(s); stroke.lastApply = s; }
    else if (S.tool === 'erase') { applyErase(s); stroke.lastApply = s; }
    E.localUpdate(); E.drawSpeed(); E.updateStatus();
  });
  C.addEventListener('mousemove', e => {
    if (!S.prof) return;
    const s = clampAxis(xToS(e.offsetX));
    cursorS = s;
    if (stroke) {
      const [top, bot] = range(G.pan.v), v = Math.max(0.3, valOf(e.offsetY, G.pan.v, top, bot));
      if (stroke.tool === 'draw') { applyDraw(s, v); stroke.last = { s, v }; }
      else if (Math.abs(s - stroke.lastApply) >= 0.05) { (stroke.tool === 'smooth' ? applySmooth : applyErase)(s); stroke.lastApply = s; }
      E.localUpdate(); E.drawSpeed(); E.updateStatus(); return;
    }
    if (paint) { paint.s1 = Math.min(G.L, Math.max(0, s)); E.drawSpeed(); return; }
    const i = panelAt(e.offsetY) ? E.indexAtS(wrap(s)) : -1;
    if (i !== S.hover) { S.hover = i; E.readout(i); }
    E.drawSpeed();
  });
  function endStroke() {
    if (stroke) {
      if (stroke.tool === 'draw') smoothStroke();
      stroke = null; S.dragging = false; E.localUpdate(); E.recompute();
    }
    if (paint) {
      const { panel: pn, s0, s1, level } = paint; paint = null;
      if (Math.abs(s1 - s0) >= 0.3) E.addZone(pn.name === 'v' ? 'v_zones' : 'lat_zones', Math.min(s0, s1), Math.max(s0, s1), level);
      else E.drawSpeed();
    }
  }
  C.addEventListener('mouseup', endStroke);
  C.addEventListener('mouseleave', () => { endStroke(); cursorS = null; if (S.hover >= 0) { S.hover = -1; E.readout(-1); } E.drawSpeed(); });
})();
