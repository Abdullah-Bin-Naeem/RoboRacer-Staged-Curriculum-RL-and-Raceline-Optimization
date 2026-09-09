'use strict';
// Boot: fetch the track and the line, wire the controls, size the canvases.
(async () => {
  const S = E.S;
  E.bindControls();
  try {
    const r = await E.api('/api/init');
    S.track = r.track; S.safety = r.safety; S.phys = r.phys; S.meta = r.map;
    const img = new Image();
    img.onload = () => { S.map = img; E.fit(); };
    img.src = 'data:image/png;base64,' + r.map.png;
    E.fillFiles(r.files); E.applyLine(r.line); $('files').value = r.line.name;
    E.setView('map'); E.setTool('draw');
    new ResizeObserver(() => E.resize()).observe($('view-map'));
    new ResizeObserver(() => E.resize()).observe($('view-speed'));
    E.fit();
  } catch (e) {
    $('title').textContent = 'failed to load: ' + e.message;
  }
})();
