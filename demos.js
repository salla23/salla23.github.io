/* ============================================================================
   Demos — interactive teaching tools (dependency-free, ES5-safe)
   (1) 2D -> 3D back-projection via the pinhole model + ground-plane intersection
   (2) a 1-layer "neuron per letter" OCR intuition demo
   Each init is guarded so a missing element or any error is a silent no-op.
   ============================================================================ */
(function () {
  'use strict';
  function clamp(v, a, b) { return Math.max(a, Math.min(b, v)); }
  function fix(v, n) { return (Math.round(v * Math.pow(10, n)) / Math.pow(10, n)).toFixed(n); }

  /* ------------------------------------------------------------------ */
  /* (1) BACK-PROJECTION                                                */
  /* ------------------------------------------------------------------ */
  function initBackproj() {
    var cam = document.getElementById('bp-cam');
    var side = document.getElementById('bp-side');
    if (!cam || !side) return;
    var cc = cam.getContext('2d'), sc = side.getContext('2d');
    if (!cc || !sc) return;

    var IMW = 640, IMH = 480;                 // image size (matches the repo's camera assumption)
    var P = { f: 500, cx: 320, cy: 240, h: 1.2, pitch: 10, u: 384, v: 332 };

    // palette tuned for the dark instrument panel (same in both themes)
    var COL = {
      grid: 'rgba(110,140,255,0.42)', grid2: 'rgba(110,140,255,0.15)',
      horizon: 'rgba(244,162,74,0.85)', cone: '#f4a24a', ray: '#6e8bff',
      hit: '#f4a24a', txt: 'rgba(205,214,236,0.92)', dim: 'rgba(150,164,196,0.72)',
      groundfill: 'rgba(110,140,255,0.05)', cam: '#dde5ff'
    };
    var cones = [[3, 0], [5, 1.6], [4.2, -1.4], [7, 0.6]];   // (forward, lateral) m

    function Rmat(phiDeg) {
      var p = phiDeg * Math.PI / 180, c = Math.cos(p), s = Math.sin(p);
      return [[0, -s, c], [-1, 0, 0], [0, -c, -s]];
    }
    function mul(R, v) {
      return [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
              R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
              R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]];
    }
    function mulT(R, v) { // R^T v  (world -> camera)
      return [R[0][0]*v[0]+R[1][0]*v[1]+R[2][0]*v[2],
              R[0][1]*v[0]+R[1][1]*v[1]+R[2][1]*v[2],
              R[0][2]*v[0]+R[1][2]*v[1]+R[2][2]*v[2]];
    }
    function project(Pw) { // world -> pixel {u,v,z} or null if behind
      var R = Rmat(P.pitch), pc = mulT(R, [Pw[0], Pw[1], Pw[2] - P.h]);
      if (pc[2] <= 0.02) return null;
      return { u: P.f * (pc[0] / pc[2]) + P.cx, v: P.f * (pc[1] / pc[2]) + P.cy, z: pc[2] };
    }
    function backproj(u, v) { // pixel -> {hit, dw, lam, Pw}
      var R = Rmat(P.pitch), C = [0, 0, P.h];
      var dc = [(u - P.cx) / P.f, (v - P.cy) / P.f, 1];
      var dw = mul(R, dc);
      if (dw[2] >= -1e-6) return { hit: false, dw: dw };
      var lam = -C[2] / dw[2];
      return { hit: true, dw: dw, lam: lam, Pw: [C[0]+lam*dw[0], C[1]+lam*dw[1], C[2]+lam*dw[2]] };
    }
    function horizonV() { return P.cy - P.f * Math.tan(P.pitch * Math.PI / 180); }

    // ---- sizing ----
    var camS = 1, sideW = 0, sideH = 0;
    function size(canvas, ctx) {
      var r = canvas.getBoundingClientRect(), dpr = Math.min(2, window.devicePixelRatio || 1);
      canvas.width = Math.max(1, Math.round(r.width * dpr));
      canvas.height = Math.max(1, Math.round(r.height * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      return r;
    }
    function sizeAll() {
      var rc = size(cam, cc); camS = rc.width / IMW;
      var rs = size(side, sc); sideW = rs.width; sideH = rs.height;
    }
    function CX(u) { return u * camS; }
    function CY(v) { return v * camS; }

    // ---- camera-view render ----
    function lineWorld(a, b, col, w) {
      var pa = project(a), pb = project(b); if (!pa || !pb) return;
      cc.strokeStyle = col; cc.lineWidth = w || 1;
      cc.beginPath(); cc.moveTo(CX(pa.u), CY(pa.v)); cc.lineTo(CX(pb.u), CY(pb.v)); cc.stroke();
    }
    function renderCam() {
      cc.clearRect(0, 0, cam.width, cam.height);
      var vh = horizonV();
      // faint ground fill below horizon
      if (vh < IMH) { cc.fillStyle = COL.groundfill; cc.fillRect(0, CY(Math.max(0, vh)), CX(IMW), CY(IMH) - CY(Math.max(0, vh))); }
      // ground grid: lateral lines (const X) and depth lines (const Y)
      var x, y, latW = 5;
      for (x = 1; x <= 12; x++) lineWorld([x, -latW, 0], [x, latW, 0], (x % 5 === 0) ? COL.grid : COL.grid2, (x % 5 === 0) ? 1.2 : 1);
      for (y = -latW; y <= latW; y++) lineWorld([0.5, y, 0], [13, y, 0], (y === 0) ? COL.grid : COL.grid2, (y === 0) ? 1.2 : 1);
      // horizon
      if (vh > 0 && vh < IMH) {
        cc.strokeStyle = COL.horizon; cc.lineWidth = 1; cc.setLineDash([5, 4]);
        cc.beginPath(); cc.moveTo(0, CY(vh)); cc.lineTo(CX(IMW), CY(vh)); cc.stroke(); cc.setLineDash([]);
        cc.fillStyle = COL.horizon; cc.font = '600 ' + (10) + 'px ui-monospace, monospace';
        cc.fillText('horizon', 8, CY(vh) - 6);
      }
      // cones (ICURO nod) — base on the ground, apex up
      for (var i = 0; i < cones.length; i++) {
        var base = project([cones[i][0], cones[i][1], 0]), apex = project([cones[i][0], cones[i][1], 0.4]);
        if (!base || !apex) continue;
        var bx = CX(base.u), by = CY(base.v), ax = CX(apex.u), ay = CY(apex.v);
        var wdt = Math.max(2, (by - ay) * 0.36);
        cc.fillStyle = COL.cone; cc.globalAlpha = 0.85;
        cc.beginPath(); cc.moveTo(ax, ay); cc.lineTo(bx - wdt, by); cc.lineTo(bx + wdt, by); cc.closePath(); cc.fill();
        cc.globalAlpha = 1;
      }
      // current sample: crosshair + the recovered ground point
      var r = backproj(P.u, P.v);
      var sx = CX(P.u), sy = CY(P.v);
      if (r.hit) {
        var pp = project(r.Pw);
        if (pp) { cc.strokeStyle = COL.hit; cc.lineWidth = 1.4; cc.beginPath(); cc.arc(CX(pp.u), CY(pp.v), 5, 0, Math.PI * 2); cc.stroke(); }
      }
      cc.strokeStyle = r.hit ? COL.ray : COL.dim; cc.lineWidth = 1.4;
      cc.beginPath(); cc.moveTo(sx - 9, sy); cc.lineTo(sx + 9, sy); cc.moveTo(sx, sy - 9); cc.lineTo(sx, sy + 9); cc.stroke();
      cc.fillStyle = r.hit ? COL.ray : COL.dim; cc.beginPath(); cc.arc(sx, sy, 2.4, 0, Math.PI * 2); cc.fill();
    }

    // ---- side-view schematic (X = range, Z = height) ----
    function renderSide(r) {
      sc.clearRect(0, 0, side.width, side.height);
      var mL = 38, mR = 16, mT = 16, mB = 26;
      var xf = (r.hit ? r.Pw[0] : 6);
      var Xspan = clamp(Math.max(xf * 1.2, 4), 4, 26), Zspan = Math.max(P.h * 1.7, 1.4);
      function SX(X) { return mL + (X / Xspan) * (sideW - mL - mR); }
      function SY(Z) { return (sideH - mB) - (Z / Zspan) * (sideH - mT - mB); }
      // ground
      sc.strokeStyle = COL.grid; sc.lineWidth = 1.4;
      sc.beginPath(); sc.moveTo(SX(0), SY(0)); sc.lineTo(SX(Xspan), SY(0)); sc.stroke();
      // range ticks
      sc.fillStyle = COL.dim; sc.font = '10px ui-monospace, monospace'; sc.strokeStyle = COL.grid2; sc.lineWidth = 1;
      for (var m = 1; m <= Xspan; m++) {
        if (m % (Xspan > 12 ? 4 : 2) !== 0) continue;
        sc.beginPath(); sc.moveTo(SX(m), SY(0)); sc.lineTo(SX(m), SY(0) + 4); sc.stroke();
        sc.fillText(m + 'm', SX(m) - 6, SY(0) + 16);
      }
      // camera + height
      var camX = SX(0), camZ = SY(P.h);
      sc.strokeStyle = COL.dim; sc.setLineDash([3, 3]); sc.beginPath(); sc.moveTo(camX, camZ); sc.lineTo(camX, SY(0)); sc.stroke(); sc.setLineDash([]);
      sc.fillStyle = COL.dim; sc.fillText('h=' + fix(P.h, 2) + 'm', camX + 5, (camZ + SY(0)) / 2);
      // horizontal (horizon direction) from camera
      sc.strokeStyle = 'rgba(244,162,74,0.5)'; sc.setLineDash([4, 4]);
      sc.beginPath(); sc.moveTo(camX, camZ); sc.lineTo(SX(Xspan * 0.5), camZ); sc.stroke(); sc.setLineDash([]);
      // ray
      if (r.hit) {
        sc.strokeStyle = COL.ray; sc.lineWidth = 1.6;
        sc.beginPath(); sc.moveTo(camX, camZ); sc.lineTo(SX(r.Pw[0]), SY(0)); sc.stroke();
        // intersection
        sc.fillStyle = COL.hit; sc.beginPath(); sc.arc(SX(r.Pw[0]), SY(0), 4, 0, Math.PI * 2); sc.fill();
        sc.fillStyle = COL.txt; sc.font = '600 11px ui-monospace, monospace';
        sc.fillText('range ' + fix(Math.sqrt(r.Pw[0]*r.Pw[0] + r.Pw[1]*r.Pw[1]), 2) + ' m', SX(r.Pw[0]) - 30, SY(0) - 8);
      } else {
        sc.strokeStyle = COL.dim; sc.lineWidth = 1.4; sc.setLineDash([5, 4]);
        sc.beginPath(); sc.moveTo(camX, camZ); sc.lineTo(SX(Xspan * 0.7), SY(Zspan * 0.95)); sc.stroke(); sc.setLineDash([]);
        sc.fillStyle = COL.dim; sc.fillText('ray above horizon', camX + 8, SY(Zspan * 0.7));
      }
      // camera glyph
      sc.fillStyle = COL.cam; sc.fillRect(camX - 5, camZ - 4, 10, 8);
      sc.fillStyle = COL.ray; sc.fillRect(camX + 4, camZ - 2, 4, 4);
    }

    // ---- DOM (matrix, equations, readout) ----
    function setText(id, t) { var e = document.getElementById(id); if (e) e.textContent = t; }
    function updateDOM(r) {
      setText('k-f', fix(P.f, 0)); setText('k-f2', fix(P.f, 0)); setText('k-cx', fix(P.cx, 0)); setText('k-cy', fix(P.cy, 0));
      var dc = [(P.u - P.cx) / P.f, (P.v - P.cy) / P.f, 1];
      setText('eq-dc', '[ ' + fix(dc[0], 3) + ', ' + fix(dc[1], 3) + ', 1 ]');
      setText('eq-dw', '[ ' + fix(r.dw[0], 3) + ', ' + fix(r.dw[1], 3) + ', ' + fix(r.dw[2], 3) + ' ]');
      setText('eq-hz', fix(horizonV(), 1) + ' px');
      // readout
      setText('ro-uv', '(' + Math.round(P.u) + ', ' + Math.round(P.v) + ') px');
      if (r.hit) {
        setText('eq-lam', fix(r.lam, 3));
        setText('eq-pw', 'X=' + fix(r.Pw[0], 2) + ' m   Y=' + fix(r.Pw[1], 2) + ' m');
        setText('ro-ground', 'X ' + fix(r.Pw[0], 2) + ' m  ·  Y ' + fix(r.Pw[1], 2) + ' m');
        setText('ro-range', fix(Math.sqrt(r.Pw[0]*r.Pw[0] + r.Pw[1]*r.Pw[1]), 2) + ' m');
        var st = document.getElementById('ro-status'); if (st) { st.textContent = 'on ground'; st.className = 'hot'; }
        // sensitivity |dlam/dv| in metres per pixel
        var c = Math.cos(P.pitch * Math.PI / 180), sens = P.h * c / (P.f * r.dw[2] * r.dw[2]);
        setText('ro-sens', fix(Math.abs(sens), 3) + ' m / px');
      } else {
        setText('eq-lam', '—'); setText('eq-pw', 'no intersection');
        setText('ro-ground', '—'); setText('ro-range', '∞ (above horizon)');
        var st2 = document.getElementById('ro-status'); if (st2) { st2.textContent = 'above horizon'; st2.className = 'warn'; }
        setText('ro-sens', '—');
      }
    }

    function renderAll() { var r = backproj(P.u, P.v); renderCam(); renderSide(r); updateDOM(r); }

    // ---- controls ----
    var ids = ['f', 'cx', 'cy', 'h', 'pitch'];
    for (var i = 0; i < ids.length; i++) (function (key) {
      var el = document.getElementById('bp-' + key); if (!el) return;
      el.value = P[key];
      var out = document.getElementById('bp-' + key + '-v'); if (out) out.textContent = el.value + (el.getAttribute('data-unit') || '');
      el.addEventListener('input', function () {
        P[key] = parseFloat(el.value);
        if (out) out.textContent = el.value + (el.getAttribute('data-unit') || '');
        renderAll();
      });
    })(ids[i]);

    // ---- click + keyboard on the camera view ----
    function pick(ev) {
      var r = cam.getBoundingClientRect();
      P.u = clamp((ev.clientX - r.left) / camS, 0, IMW);
      P.v = clamp((ev.clientY - r.top) / camS, 0, IMH);
      renderAll();
    }
    cam.addEventListener('click', pick);
    cam.addEventListener('keydown', function (ev) {
      var step = ev.shiftKey ? 20 : 4, k = ev.key;
      if (k === 'ArrowLeft') P.u = clamp(P.u - step, 0, IMW);
      else if (k === 'ArrowRight') P.u = clamp(P.u + step, 0, IMW);
      else if (k === 'ArrowUp') P.v = clamp(P.v - step, 0, IMH);
      else if (k === 'ArrowDown') P.v = clamp(P.v + step, 0, IMH);
      else return;
      ev.preventDefault(); renderAll();
    });

    var rz; window.addEventListener('resize', function () { clearTimeout(rz); rz = setTimeout(function () { sizeAll(); renderAll(); }, 120); });
    sizeAll(); renderAll();
  }

  /* ------------------------------------------------------------------ */
  /* (2) OCR — one output neuron per letter (template matching)         */
  /* ------------------------------------------------------------------ */
  function initOCR() {
    var gridEl = document.getElementById('ocr-grid');
    if (!gridEl) return;
    var ROWS = 7, COLS = 5, N = ROWS * COLS;
    var FONT = {
      A: '.###.#...##...########...##...##...#',
      C: '.#######....#....#....#....#.....####',
      E: '######....#....####.#....#....######',
      H: '#...##...##...########...##...##...#',
      I: '#####..#....#....#....#....#..#####',
      L: '#....#....#....#....#....#....######',
      O: '.###.#...##...##...##...##...#.###.',
      T: '#####..#....#....#....#....#....#..',
      U: '#...##...##...##...##...##...#.###.'
    };
    // normalise any accidental length to exactly N
    var protos = {};
    for (var L in FONT) if (FONT.hasOwnProperty(L)) {
      var s = FONT[L].replace(/[^#.]/g, ''); s = (s + '...................................').slice(0, N);
      var arr = []; for (var j = 0; j < N; j++) arr.push(s.charAt(j) === '#' ? 1 : 0);
      protos[L] = arr;
    }
    var letters = Object.keys(protos);
    var grid = protos.A.slice();
    var target = 'A';

    // build cells
    var cells = [];
    gridEl.style.gridTemplateColumns = 'repeat(' + COLS + ', auto)';
    for (var idx = 0; idx < N; idx++) (function (k) {
      var b = document.createElement('button');
      b.type = 'button'; b.className = 'ocr-cell';
      b.setAttribute('aria-label', 'pixel ' + (Math.floor(k / COLS) + 1) + ',' + (k % COLS + 1));
      b.addEventListener('click', function () { grid[k] = grid[k] ? 0 : 1; sync(); });
      cells.push(b); gridEl.appendChild(b);
    })(idx);

    function score(proto) { // correlation in [-1,1] over +/-1 pixels
      var s = 0; for (var k = 0; k < N; k++) s += (2 * grid[k] - 1) * (2 * proto[k] - 1);
      return s / N;
    }
    function predict() {
      var best = null, bestS = -2, all = [];
      for (var i = 0; i < letters.length; i++) {
        var L = letters[i], sv = score(protos[L]); all.push([L, sv]);
        if (sv > bestS) { bestS = sv; best = L; }
      }
      all.sort(function (a, b) { return b[1] - a[1]; });
      return { best: best, bestS: bestS, all: all };
    }
    function sync() {
      for (var k = 0; k < N; k++) cells[k].className = 'ocr-cell' + (grid[k] ? ' on' : '');
      var pr = predict();
      var bigEl = document.getElementById('ocr-pred-big'); if (bigEl) bigEl.textContent = pr.best || '—';
      var subEl = document.getElementById('ocr-pred-sub'); if (subEl) subEl.textContent = 'activation ' + fix((pr.bestS + 1) / 2, 2) + ' · winning neuron';
      var bars = document.getElementById('ocr-bars'); if (!bars) return;
      bars.innerHTML = '';
      for (var i = 0; i < pr.all.length; i++) {
        var L = pr.all[i][0], a = (pr.all[i][1] + 1) / 2;
        var row = document.createElement('div'); row.className = 'ocr-bar-row' + (L === pr.best ? ' win' : '');
        row.innerHTML = '<span>' + L + '</span><span class="ocr-bar-track"><span class="ocr-bar-fill" style="width:' +
          Math.round(a * 100) + '%"></span></span><span class="ocr-val">' + fix(a, 2) + '</span>';
        bars.appendChild(row);
      }
    }

    // letter selector
    var selWrap = document.getElementById('ocr-letters');
    if (selWrap) for (var li = 0; li < letters.length; li++) (function (L) {
      var b = document.createElement('button');
      b.type = 'button'; b.className = 'ocr-btn' + (L === target ? ' sel' : ''); b.textContent = L;
      b.addEventListener('click', function () {
        target = L;
        var kids = selWrap.querySelectorAll('.ocr-btn');
        for (var q = 0; q < kids.length; q++) kids[q].className = 'ocr-btn' + (kids[q].textContent === L ? ' sel' : '');
      });
      selWrap.appendChild(b);
    })(letters[li]);

    function bind(id, fn) { var e = document.getElementById(id); if (e) e.addEventListener('click', fn); }
    bind('ocr-clear', function () { for (var k = 0; k < N; k++) grid[k] = 0; sync(); });
    bind('ocr-load', function () { grid = protos[target].slice(); sync(); });
    bind('ocr-teach', function () { protos[target] = grid.slice(); sync(); });

    sync();
  }

  function boot() { try { initBackproj(); } catch (e) {} try { initOCR(); } catch (e) {} }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot); else boot();
})();
