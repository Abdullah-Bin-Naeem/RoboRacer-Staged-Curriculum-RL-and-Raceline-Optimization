'use strict';
// Map view: the occupancy grid, the line coloured by speed, the drag brush, the
// edited region and its smoother, and the curvature strip underneath.
(() => {
  const S = E.S;
  const mapC = $('map'), curvC = $('curv'), wrap = $('mapwrap');
  const mctx = mapC.getContext('2d'), cctx = curvC.getContext('2d');
  const V = E.mapView = { cx: 0, cy: 0, scale: 40 };      // world centre of the canvas, px per metre
  const CV = { x0: 44, x1: 0, y0: 22, h: 100 };            // curvature strip layout
  let drag = null, pan = null;

  E.w2s = (x, y) => [(x - V.cx) * V.scale + mapC.clientWidth / 2, (V.cy - y) * V.scale + mapC.clientHeight / 2];
  E.s2w = (px, py) => [(px - mapC.clientWidth / 2) / V.scale + V.cx, V.cy - (py - mapC.clientHeight / 2) / V.scale];
  E.screenOf = i => E.w2s(S.x[i], S.y[i]);

  E.fit = function () {
    const m = S.meta; if (!m) return;
    if (S.view !== 'map') E.setView('map');
    V.scale = 0.95 * Math.min(mapC.clientWidth / (m.W * m.res), mapC.clientHeight / (m.H * m.res));
    V.cx = m.ox + m.W * m.res / 2; V.cy = m.oy + m.H * m.res / 2;
    E.draw();
  };
  E.resizeMap = function () {
    const dpr = window.devicePixelRatio || 1;
    mapC.width = wrap.clientWidth * dpr; mapC.height = wrap.clientHeight * dpr;
    mapC.style.width = wrap.clientWidth + 'px'; mapC.style.height = wrap.clientHeight + 'px';
    mctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    curvC.width = curvC.clientWidth * dpr; curvC.height = 150 * dpr;
    cctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  };

  function nearestPoint(px, py, maxPx) {
    let best = -1, bd = maxPx * maxPx;
    for (let i = 0; i < S.x.length; i++) {
      const [sx, sy] = E.w2s(S.x[i], S.y[i]), dd = (sx - px) ** 2 + (sy - py) ** 2;
      if (dd < bd) { bd = dd; best = i; }
    }
    return best;
  }

  // ---- drawing -----------------------------------------------------------------------
  E.drawMap = function () {
    const cw = mapC.clientWidth, ch = mapC.clientHeight, ctx = mctx;
    ctx.fillStyle = '#23262b'; ctx.fillRect(0, 0, cw, ch);
    if (S.map) {
      const m = S.meta, [sx, sy] = E.w2s(m.ox, m.oy + m.H * m.res);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(S.map, sx, sy, m.W * m.res * V.scale, m.H * m.res * V.scale);
    }
    const n = S.x.length; if (!n) return;
    if (S.orig) {
      ctx.setLineDash([4, 4]); ctx.strokeStyle = 'rgba(80,80,80,0.9)'; ctx.lineWidth = 1; ctx.beginPath();
      for (let i = 0; i <= n; i++) { const k = i % n, [px, py] = E.w2s(S.orig.x[k], S.orig.y[k]); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); }
      ctx.stroke(); ctx.setLineDash([]);
    }
    const v = S.prof && S.prof.v, [vmin, vmax] = E.vRange();
    ctx.lineWidth = Math.max(2, Math.min(7, V.scale * 0.07)); ctx.lineCap = 'round';
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n, [ax, ay] = E.w2s(S.x[i], S.y[i]), [bx, by] = E.w2s(S.x[j], S.y[j]);
      ctx.strokeStyle = v ? E.turbo((v[i] - vmin) / ((vmax - vmin) || 1)) : '#00b050';
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
    }
    if (S.region.some(w => w > 0)) {                        // the edited region, for the smoother
      ctx.fillStyle = 'rgba(245,158,11,0.75)';
      for (let i = 0; i < n; i++) if (S.region[i] > 0) { const [px, py] = E.w2s(S.x[i], S.y[i]); ctx.beginPath(); ctx.arc(px, py, 1.5 + 2.5 * S.region[i], 0, 7); ctx.fill(); }
    }
    if (V.scale > 30) {
      ctx.fillStyle = 'rgba(0,0,0,0.55)';
      for (let i = 0; i < n; i++) { const [px, py] = E.w2s(S.x[i], S.y[i]); ctx.beginPath(); ctx.arc(px, py, 2, 0, 7); ctx.fill(); }
    }
    if (S.prof) {
      ctx.strokeStyle = '#e11'; ctx.lineWidth = 1.5;
      for (let i = 0; i < n; i++) if (S.prof.body[i] < S.safety) { const [px, py] = E.w2s(S.x[i], S.y[i]); ctx.beginPath(); ctx.arc(px, py, 5, 0, 7); ctx.stroke(); }
    }
    if (drag) {
      ctx.fillStyle = 'rgba(255,255,255,0.9)';
      for (let i = 0; i < n; i++) if (drag.w[i] > 0) { const [px, py] = E.w2s(S.x[i], S.y[i]); ctx.beginPath(); ctx.arc(px, py, 1.5 + 2.5 * drag.w[i], 0, 7); ctx.fill(); }
    }
    const [zx, zy] = E.w2s(S.x[0], S.y[0]);
    ctx.fillStyle = '#000'; ctx.beginPath(); ctx.moveTo(zx, zy - 6); ctx.lineTo(zx + 6, zy); ctx.lineTo(zx, zy + 6); ctx.lineTo(zx - 6, zy); ctx.fill();
    if (S.hover >= 0) {
      const [px, py] = E.w2s(S.x[S.hover], S.y[S.hover]);
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(px, py, 6, 0, 7); ctx.stroke();
      ctx.strokeStyle = '#000'; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(px, py, 6, 0, 7); ctx.stroke();
    }
    if (v) {
      const bx = 12, by = 12, bw = 12, bh = 120;
      for (let k = 0; k < bh; k++) { ctx.fillStyle = E.turbo(1 - k / bh); ctx.fillRect(bx, by + k, bw, 1); }
      ctx.fillStyle = '#eee'; ctx.font = '11px system-ui';
      ctx.fillText(vmax.toFixed(1) + ' m/s', bx + bw + 5, by + 9); ctx.fillText(vmin.toFixed(1), bx + bw + 5, by + bh);
    }
  };

  E.drawCurv = function () {
    const W = curvC.clientWidth, ctx = cctx; ctx.clearRect(0, 0, W, 150);
    const p = S.prof; if (!p) return;
    CV.x1 = W - 12;
    const L = p.length, kcar = S.phys ? S.phys.kappa_car : 1.78, kmax = Math.max(...p.kappa.map(Math.abs));
    const top = Math.max(kcar, kmax) * 1.08;
    const sx = s => CV.x0 + s / L * (CV.x1 - CV.x0), ky = k => CV.y0 + CV.h / 2 - k / top * CV.h / 2;
    ctx.fillStyle = '#ccc'; ctx.font = '11px system-ui'; ctx.textAlign = 'left';
    ctx.fillText(`curvature κ [1/m]   |κ| max ${kmax.toFixed(2)}   full lock ${kcar.toFixed(2)}`, CV.x0, 14);
    ctx.strokeStyle = '#3d4149'; ctx.lineWidth = 1;
    for (let s = 0; s <= L; s += 5) { const x = sx(s); ctx.beginPath(); ctx.moveTo(x, CV.y0); ctx.lineTo(x, CV.y0 + CV.h); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(s.toFixed(0), x, CV.y0 + CV.h + 14); }
    ctx.textAlign = 'right';
    for (const k of [-kcar, 0, kcar]) {
      const y = ky(k); ctx.setLineDash(k ? [4, 3] : []); ctx.strokeStyle = k ? '#e11' : '#666';
      ctx.beginPath(); ctx.moveTo(CV.x0, y); ctx.lineTo(CV.x1, y); ctx.stroke(); ctx.fillText(k.toFixed(1), CV.x0 - 4, y + 4);
    }
    ctx.setLineDash([]);
    if (S.region.some(w => w > 0)) {
      ctx.fillStyle = 'rgba(245,158,11,0.18)';
      for (let i = 0; i < p.s.length; i++) if (S.region[i] > 0) { const j = (i + 1) % p.s.length; ctx.fillRect(sx(p.s[i]), CV.y0, Math.max(1, sx(p.s[j]) - sx(p.s[i])), CV.h); }
    }
    ctx.strokeStyle = '#7dd3fc'; ctx.lineWidth = 1.5; ctx.beginPath();
    for (let i = 0; i < p.s.length; i++) { const x = sx(p.s[i]), y = ky(p.kappa[i]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }
    ctx.stroke();
    if (S.hover >= 0) {
      const x = sx(p.s[S.hover]);
      ctx.strokeStyle = '#fff'; ctx.setLineDash([2, 2]); ctx.beginPath(); ctx.moveTo(x, CV.y0); ctx.lineTo(x, CV.y0 + CV.h); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = '#fff'; ctx.beginPath(); ctx.arc(x, ky(p.kappa[S.hover]), 3.5, 0, 7); ctx.fill();
    }
  };

  // ---- the region smoother ----------------------------------------------------------
  // Smoothing weight 1 over the edited region and tapering to 0 over two radii
  // beyond it: a brush drag leaves its kinks at the region's EDGES, where the
  // falloff meets the untouched line, so the smoother has to reach past them.
  E.regionWeights = function (sigma) {
    const n = S.x.length, { el } = E.arc();
    if (!S.region.some(r => r > 0)) return null;
    const d = new Float64Array(n).fill(Infinity);
    for (let i = 0; i < n; i++) if (S.region[i] > 0) d[i] = 0;
    for (let pass = 0; pass < 2; pass++) {                    // arc distance to the region, both ways round
      for (let k = 1; k <= n; k++) { const i = k % n, h = (i - 1 + n) % n; d[i] = Math.min(d[i], d[h] + el[h]); }
      for (let k = n - 1; k >= -1; k--) { const i = (k + n) % n, j = (i + 1) % n; d[i] = Math.min(d[i], d[j] + el[i]); }
    }
    return Array.from(d, v => Math.max(0, 1 - v / (2 * sigma)));
  };
  E.smoothRegion = function () {
    const strength = parseFloat($('g_strength').value), radius = parseFloat($('g_radius').value);
    const base = E.regionWeights(radius); if (!base) return false;
    const w = base.map(v => v * strength);
    E.snapshot();
    const { s, L } = E.arc();
    S.x = E.gaussSmooth(S.x, radius, w, s, L); S.y = E.gaussSmooth(S.y, radius, w, s, L);
    for (let i = 0; i < S.region.length; i++) if (w[i] > 0) S.region[i] = Math.max(S.region[i], w[i] / strength);   // the region now includes what moved
    E.recompute();
    return true;
  };
  $('g_smooth').addEventListener('click', () => E.smoothRegion());
  $('g_forget').addEventListener('click', () => { S.region.fill(0); E.draw(); });

  // ---- mouse ----------------------------------------------------------------------------
  mapC.addEventListener('mousedown', e => {
    const px = e.offsetX, py = e.offsetY;
    if (e.button === 0) {
      const i = nearestPoint(px, py, 12);
      if (i >= 0) {
        E.snapshot();
        const [wx, wy] = E.s2w(px, py);
        drag = { i0: i, w: E.brushWeights(i, parseFloat($('brush').value)), lx: wx, ly: wy, moved: false };
        S.dragging = true; S.hover = i; E.drawMap(); return;
      }
    }
    pan = { px, py, cx: V.cx, cy: V.cy }; mapC.style.cursor = 'grabbing';
  });
  mapC.addEventListener('mousemove', e => {
    const px = e.offsetX, py = e.offsetY;
    if (drag) {
      const [wx, wy] = E.s2w(px, py), dx = wx - drag.lx, dy = wy - drag.ly;
      for (let k = 0; k < S.x.length; k++) if (drag.w[k] > 0) {
        S.x[k] += drag.w[k] * dx; S.y[k] += drag.w[k] * dy; S.region[k] = Math.max(S.region[k], drag.w[k]);
      }
      drag.lx = wx; drag.ly = wy; drag.moved = true;
      if ($('live').checked) E.recompute(true); else E.drawMap();
      return;
    }
    if (pan) { V.cx = pan.cx - (px - pan.px) / V.scale; V.cy = pan.cy + (py - pan.py) / V.scale; E.drawMap(); return; }
    const i = nearestPoint(px, py, 12);
    if (i !== S.hover) { S.hover = i; E.draw(); E.readout(i); }
  });
  function endDrag() {
    if (drag) { const moved = drag.moved; drag = null; S.dragging = false; if (moved) E.recompute(); else { S.undo.pop(); E.draw(); } }
    if (pan) { pan = null; mapC.style.cursor = 'crosshair'; }
  }
  mapC.addEventListener('mouseup', endDrag);
  mapC.addEventListener('mouseleave', () => { endDrag(); if (S.hover >= 0) { S.hover = -1; E.draw(); E.readout(-1); } });
  mapC.addEventListener('contextmenu', e => e.preventDefault());
  mapC.addEventListener('wheel', e => {
    e.preventDefault();
    const [wx, wy] = E.s2w(e.offsetX, e.offsetY), f = Math.exp(-e.deltaY * 0.0015);
    V.scale = Math.min(2000, Math.max(5, V.scale * f));
    const [nx, ny] = E.s2w(e.offsetX, e.offsetY);
    V.cx += wx - nx; V.cy += wy - ny; E.draw();
  }, { passive: false });
  curvC.addEventListener('mousemove', e => {
    if (!S.prof) return;
    const s = (e.offsetX - CV.x0) / (CV.x1 - CV.x0) * S.prof.length;
    const i = (s >= 0 && s <= S.prof.length) ? E.indexAtS(s) : -1;
    if (i !== S.hover) { S.hover = i; E.draw(); E.readout(i); }
  });
  curvC.addEventListener('mouseleave', () => { if (S.hover >= 0) { S.hover = -1; E.draw(); E.readout(-1); } });
})();
