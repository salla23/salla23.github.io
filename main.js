/* ============================================================================
   Sudhakar Alla — portfolio interactions
   (1) live EKF-localization viewport (the hero signature)
   (2) theme toggle  (3) graceful broken-image fallback
   Vanilla, dependency-free, ES5-safe. Everything is wrapped so any failure
   leaves the static, styled page fully intact.
   ============================================================================ */
(function () {
  'use strict';

  /* ---- theme toggle (session-scoped; system preference is the default) ---- */
  function initTheme() {
    var root = document.documentElement;
    var btn = document.querySelector('.theme-toggle');
    if (!btn) return;
    var mq = window.matchMedia ? window.matchMedia('(prefers-color-scheme: dark)') : null;
    function dark() {
      var a = root.getAttribute('data-theme');
      if (a === 'dark') return true;
      if (a === 'light') return false;
      return mq ? mq.matches : false;
    }
    function sync() {
      var d = dark();
      btn.setAttribute('aria-pressed', String(d));
      btn.setAttribute('aria-label', d ? 'Switch to light theme' : 'Switch to dark theme');
    }
    btn.addEventListener('click', function () {
      root.setAttribute('data-theme', dark() ? 'light' : 'dark');
      sync();
      try { window.dispatchEvent(new Event('themechange')); } catch (e) {}
    });
    if (mq && mq.addEventListener) {
      mq.addEventListener('change', function () { sync(); try { window.dispatchEvent(new Event('themechange')); } catch (e) {} });
    }
    sync();
  }

  /* ---- broken-image fallback → keep the graceful grey placeholder ---- */
  function initImageFallback() {
    function fail(img) {
      if (img.getAttribute('data-broken')) return;
      img.setAttribute('data-broken', '1');
      if (img.classList.contains('photo')) {
        var div = document.createElement('div');
        div.className = 'photo';
        div.setAttribute('role', 'img');
        div.setAttribute('aria-label', img.getAttribute('alt') || '');
        if (img.parentNode) img.parentNode.replaceChild(div, img);
      } else {
        var wrap = img.closest ? img.closest('.fig-img-wrap') : null;
        if (wrap) { wrap.classList.add('is-broken'); wrap.textContent = img.getAttribute('alt') || 'Figure unavailable'; }
      }
    }
    var imgs = document.querySelectorAll('.fig img, img.photo');
    Array.prototype.forEach.call(imgs, function (img) {
      img.addEventListener('error', function () { fail(img); });
      if (img.complete && img.naturalWidth === 0 && img.getAttribute('src')) fail(img);
    });
  }

  /* ---- EKF localization viewport ------------------------------------------
     A robot patrols a corridor (occupancy grid + walls). Its covariance
     ellipse grows while it coasts on odometry and collapses at each fiducial
     fix; a LiDAR fan rakes the walls. This is the project named in the first
     card — rendered live. Colours are read from CSS so it follows the theme.
  --------------------------------------------------------------------------- */
  function initSim() {
    var canvas = document.getElementById('sim-canvas');
    if (!canvas) return;
    var ctx;
    try { ctx = canvas.getContext('2d'); } catch (e) { return; }
    if (!ctx) return;

    var reduce = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;

    var C = {};
    function readColors() {
      var cs = getComputedStyle(document.documentElement);
      function g(n, f) { var v = cs.getPropertyValue(n); return (v && v.trim()) || f; }
      C.grid = g('--sim-grid', 'rgba(124,140,176,.13)');
      C.wall = g('--sim-wall', 'rgba(154,170,202,.42)');
      C.track = g('--sim-track', '#6e8bff');
      C.ray = g('--sim-ray', 'rgba(120,150,255,.16)');
      C.fid = g('--sim-fid', '#f4a24a');
      C.cov = g('--sim-cov', '#f4a24a');
      C.covF = g('--sim-cov-fill', 'rgba(244,162,74,.07)');
      C.robot = g('--sim-robot', '#dde5ff');
      C.trail = g('--sim-trail', 'rgba(120,150,255,.28)');
    }
    readColors();

    /* world in normalised [0,1] space */
    var pathPts = [[0.16,0.30],[0.38,0.18],[0.64,0.20],[0.84,0.34],[0.82,0.62],[0.62,0.80],[0.36,0.80],[0.17,0.64]];
    var walls = [
      [0.06,0.08,0.94,0.08],[0.94,0.08,0.94,0.92],[0.94,0.92,0.06,0.92],[0.06,0.92,0.06,0.08],
      [0.42,0.40,0.58,0.40],[0.58,0.40,0.58,0.58],[0.58,0.58,0.42,0.58],[0.42,0.58,0.42,0.40]
    ];
    var fiducials = [[0.50,0.10],[0.92,0.48],[0.50,0.90],[0.08,0.46]];

    function catmull(p0,p1,p2,p3,t){
      var t2=t*t, t3=t2*t;
      return [
        0.5*((2*p1[0])+(-p0[0]+p2[0])*t+(2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2+(-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3),
        0.5*((2*p1[1])+(-p0[1]+p2[1])*t+(2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2+(-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3)
      ];
    }
    var samples = [];
    (function () {
      var n = pathPts.length, per = 36;
      for (var i = 0; i < n; i++) {
        var p0=pathPts[(i-1+n)%n], p1=pathPts[i], p2=pathPts[(i+1)%n], p3=pathPts[(i+2)%n];
        for (var s = 0; s < per; s++) samples.push(catmull(p0,p1,p2,p3,s/per));
      }
    })();
    var cum = [0], total = 0;
    for (var i = 1; i < samples.length; i++) {
      var dx = samples[i][0]-samples[i-1][0], dy = samples[i][1]-samples[i-1][1];
      total += Math.sqrt(dx*dx+dy*dy); cum.push(total);
    }
    function atDist(dd) {
      dd = ((dd % total) + total) % total;
      var lo = 0, hi = cum.length - 1;
      while (lo < hi) { var mid = (lo+hi) >> 1; if (cum[mid] < dd) lo = mid+1; else hi = mid; }
      var i1 = Math.max(1, lo), i0 = i1 - 1;
      var seg = cum[i1]-cum[i0] || 1e-6, f = (dd-cum[i0])/seg;
      var a = samples[i0], b = samples[i1];
      return [a[0]+(b[0]-a[0])*f, a[1]+(b[1]-a[1])*f];
    }

    var W=0,H=0,dpr=1,pad=0;
    function resize() {
      var r = canvas.getBoundingClientRect();
      W = Math.max(1, r.width); H = Math.max(1, r.height);
      dpr = Math.min(2, window.devicePixelRatio || 1);
      canvas.width = Math.round(W*dpr); canvas.height = Math.round(H*dpr);
      ctx.setTransform(dpr,0,0,dpr,0,0);
      pad = Math.max(14, Math.min(W,H)*0.07);
    }
    function MX(x){ return pad + x*(W-2*pad); }
    function MY(y){ return pad + y*(H-2*pad); }
    function SC(){ return Math.min(W,H)-2*pad; }

    function castRay(ox,oy,ang,maxd){
      var dx=Math.cos(ang), dy=Math.sin(ang), best=maxd, hx=ox+dx*maxd, hy=oy+dy*maxd, hit=false;
      for (var i=0;i<walls.length;i++){
        var w=walls[i], x1=w[0],y1=w[1], sx=w[2]-w[0], sy=w[3]-w[1];
        var den=dx*sy-dy*sx; if (Math.abs(den)<1e-9) continue;
        var t=((x1-ox)*sy-(y1-oy)*sx)/den, u=((x1-ox)*dy-(y1-oy)*dx)/den;
        if (t>0.001 && t<best && u>=0 && u<=1){ best=t; hx=ox+dx*t; hy=oy+dy*t; hit=true; }
      }
      return [hx,hy,hit];
    }

    var d=0, speed=0.08, sigma=0.35, sigMin=0.16, sigMax=1.0, heading=0;
    var telEl = document.querySelector('.telemetry'), telAccum=0;

    function step(dt) {
      var pos=atDist(d), ahead=atDist(d+0.004);
      heading=Math.atan2(ahead[1]-pos[1], ahead[0]-pos[0]); d+=speed*dt;
      var nearest=1e9, fix=false;
      for (var i=0;i<fiducials.length;i++){ var f=fiducials[i], dist=Math.sqrt((f[0]-pos[0])*(f[0]-pos[0])+(f[1]-pos[1])*(f[1]-pos[1])); if (dist<nearest) nearest=dist; }
      if (nearest<0.20){ sigma-=1.6*dt*(1-nearest/0.20); fix=true; }
      sigma+=0.16*dt; sigma=Math.max(sigMin, Math.min(sigMax, sigma));
      return { pos:pos, fix:fix };
    }

    function drawGrid(){ ctx.lineWidth=1; ctx.strokeStyle=C.grid; var n=10,i,gx,gy;
      for (i=0;i<=n;i++){ gx=MX(i/n); ctx.beginPath(); ctx.moveTo(gx,MY(0)); ctx.lineTo(gx,MY(1)); ctx.stroke();
        gy=MY(i/n); ctx.beginPath(); ctx.moveTo(MX(0),gy); ctx.lineTo(MX(1),gy); ctx.stroke(); } }
    function drawWalls(){ ctx.lineWidth=1.4; ctx.strokeStyle=C.wall; ctx.lineCap='round';
      for (var i=0;i<walls.length;i++){ var w=walls[i]; ctx.beginPath(); ctx.moveTo(MX(w[0]),MY(w[1])); ctx.lineTo(MX(w[2]),MY(w[3])); ctx.stroke(); } }
    function drawTrail(){ ctx.lineWidth=1.2; ctx.strokeStyle=C.trail; ctx.beginPath();
      for (var i=0;i<samples.length;i++){ var p=samples[i]; if (i===0) ctx.moveTo(MX(p[0]),MY(p[1])); else ctx.lineTo(MX(p[0]),MY(p[1])); }
      ctx.closePath(); ctx.stroke(); }
    function drawFiducials(pos){
      for (var i=0;i<fiducials.length;i++){
        var f=fiducials[i], fx=MX(f[0]), fy=MY(f[1]);
        var dist=Math.sqrt((f[0]-pos[0])*(f[0]-pos[0])+(f[1]-pos[1])*(f[1]-pos[1])), vis=dist<0.20;
        ctx.save(); ctx.translate(fx,fy); ctx.strokeStyle=C.fid; ctx.fillStyle=C.fid;
        ctx.globalAlpha=vis?1:0.5; ctx.lineWidth=1.4; var s=5.5; ctx.strokeRect(-s,-s,2*s,2*s);
        ctx.globalAlpha=vis?0.9:0.35; ctx.fillRect(-2,-2,4,4); ctx.restore();
        if (vis){ ctx.save(); ctx.strokeStyle=C.fid; ctx.globalAlpha=0.5; ctx.lineWidth=1; ctx.setLineDash([2,3]);
          ctx.beginPath(); ctx.moveTo(MX(pos[0]),MY(pos[1])); ctx.lineTo(fx,fy); ctx.stroke(); ctx.restore(); }
      }
    }
    function drawRays(pos){ var ox=pos[0],oy=pos[1],n=40,maxd=0.34; ctx.strokeStyle=C.ray; ctx.lineWidth=1;
      for (var i=0;i<n;i++){ var ang=(i/n)*Math.PI*2, r=castRay(ox,oy,ang,maxd);
        ctx.beginPath(); ctx.moveTo(MX(ox),MY(oy)); ctx.lineTo(MX(r[0]),MY(r[1])); ctx.stroke();
        if (r[2]){ ctx.fillStyle=C.wall; ctx.fillRect(MX(r[0])-1,MY(r[1])-1,2,2); } } }
    function drawRobot(pos){
      var rx=MX(pos[0]), ry=MY(pos[1]), sc=SC();
      var a=(0.018+sigma*0.085)*sc, b=(0.014+sigma*0.045)*sc;
      ctx.save(); ctx.translate(rx,ry); ctx.rotate(heading);
      ctx.beginPath(); ctx.ellipse(0,0,a,b,0,0,Math.PI*2); ctx.fillStyle=C.covF; ctx.fill();
      ctx.lineWidth=1.3; ctx.strokeStyle=C.cov; ctx.globalAlpha=0.9; ctx.stroke();
      ctx.globalAlpha=0.35; ctx.beginPath(); ctx.ellipse(0,0,a*0.6,b*0.6,0,0,Math.PI*2); ctx.stroke();
      ctx.restore();
      ctx.save(); ctx.fillStyle=C.robot; ctx.beginPath(); ctx.arc(rx,ry,3.4,0,Math.PI*2); ctx.fill();
      ctx.strokeStyle=C.track; ctx.lineWidth=1.6; ctx.beginPath(); ctx.moveTo(rx,ry);
      ctx.lineTo(rx+Math.cos(heading)*11, ry+Math.sin(heading)*11); ctx.stroke(); ctx.restore();
    }
    function frame(info){ ctx.clearRect(0,0,W,H); drawGrid(); drawTrail(); drawWalls(); drawRays(info.pos); drawFiducials(info.pos); drawRobot(info.pos); }
    function telemetry(info){ if (!telEl) return;
      var th=Math.round((((heading*180/Math.PI)%360)+360)%360);
      telEl.innerHTML =
        '<div class="t-row"><span>X <b>'+(info.pos[0]*12).toFixed(2)+'</b></span><span>Y <b>'+(info.pos[1]*9).toFixed(2)+'</b></span><span>&theta; <b>'+th+'&deg;</b></span></div>'+
        '<div class="t-row"><span>&sigma; <b>'+sigma.toFixed(2)+'</b></span><span class="t-stat">'+(info.fix?'&#9679; FIX':'&#9675; COAST')+'</span></div>';
    }
    function drawStatic(){
      ctx.clearRect(0,0,W,H); drawGrid(); drawTrail(); drawWalls();
      var k=10, i, j;
      for (i=0;i<k;i++){
        var p=atDist(total*i/k), a2=atDist(total*i/k+0.004), hd=Math.atan2(a2[1]-p[1],a2[0]-p[0]);
        var nd=1e9; for (j=0;j<fiducials.length;j++){ var f=fiducials[j]; nd=Math.min(nd, Math.sqrt((f[0]-p[0])*(f[0]-p[0])+(f[1]-p[1])*(f[1]-p[1]))); }
        var sg=Math.max(0.18, Math.min(0.9, nd*2.2)), sc=SC(), aa=(0.016+sg*0.08)*sc, bb=(0.012+sg*0.04)*sc;
        ctx.save(); ctx.translate(MX(p[0]),MY(p[1])); ctx.rotate(hd);
        ctx.beginPath(); ctx.ellipse(0,0,aa,bb,0,0,Math.PI*2); ctx.fillStyle=C.covF; ctx.fill();
        ctx.strokeStyle=C.cov; ctx.globalAlpha=0.55; ctx.lineWidth=1; ctx.stroke(); ctx.restore();
      }
      var pos=atDist(total*0.18), a3=atDist(total*0.18+0.004); heading=Math.atan2(a3[1]-pos[1],a3[0]-pos[0]);
      drawRays(pos); drawFiducials(pos); drawRobot(pos);
      if (telEl) telEl.innerHTML='<div class="t-row"><span>X <b>'+(pos[0]*12).toFixed(2)+'</b></span><span>Y <b>'+(pos[1]*9).toFixed(2)+'</b></span></div><div class="t-row"><span>&sigma; <b>0.31</b></span><span class="t-stat">&#9679; FIX</span></div>';
    }

    var raf=null, last=0, running=false, visible=true, onscreen=true;
    function tick(ts){ if (!running) return; if (!last) last=ts;
      var dt=Math.min(0.05,(ts-last)/1000); last=ts; var info=step(dt); frame(info);
      telAccum+=dt; if (telAccum>0.1){ telemetry(info); telAccum=0; } raf=requestAnimationFrame(tick); }
    function start(){ if (running||reduce) return; running=true; last=0; raf=requestAnimationFrame(tick); }
    function stop(){ running=false; if (raf) cancelAnimationFrame(raf); raf=null; }
    function update(){ if (reduce){ resize(); drawStatic(); return; } if (visible&&onscreen) start(); else stop(); }

    resize(); if (reduce) drawStatic(); else start();

    var rsz; window.addEventListener('resize', function(){ clearTimeout(rsz); rsz=setTimeout(function(){ resize(); if (reduce) drawStatic(); }, 120); });
    document.addEventListener('visibilitychange', function(){ visible=!document.hidden; update(); });
    if ('IntersectionObserver' in window){ new IntersectionObserver(function(es){ onscreen=es[0].isIntersecting; update(); }, {threshold:0.05}).observe(canvas); }
    window.addEventListener('themechange', function(){ readColors(); if (reduce) drawStatic(); });
  }

  function boot(){ try { initTheme(); } catch(e){} try { initImageFallback(); } catch(e){} try { initSim(); } catch(e){} }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot); else boot();
})();
