// Minimal browser UI: live stream with a mirror-circle / 72-ray overlay so the
// omnidirectional field of view can be checked without any PC-side tooling.
#pragma once

#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Timer Camera F - omni check</title>
<style>
 body{font-family:sans-serif;margin:8px;background:#111;color:#ddd}
 #wrap{position:relative;display:inline-block}
 #cam{display:block;max-width:100%}
 #ovl{position:absolute;left:0;top:0;pointer-events:none}
 fieldset{border:1px solid #444;margin:6px 0}
 label{display:inline-block;width:5.5em}
 input[type=range]{width:220px;vertical-align:middle}
 .v{display:inline-block;width:3.5em;text-align:right}
 #st{font-family:monospace;white-space:pre}
</style>
<div id="wrap"><img id="cam"><canvas id="ovl"></canvas></div>
<fieldset><legend>mirror circle</legend>
 <div><label>cx</label><input type="range" id="cx" min="0" max="1600" value="400"><span class="v" id="cxv"></span></div>
 <div><label>cy</label><input type="range" id="cy" min="0" max="1200" value="300"><span class="v" id="cyv"></span></div>
 <div><label>r inner</label><input type="range" id="ri" min="0" max="800" value="40"><span class="v" id="riv"></span></div>
 <div><label>r outer</label><input type="range" id="ro" min="0" max="800" value="280"><span class="v" id="rov"></span></div>
 <div><label>theta0</label><input type="range" id="t0" min="0" max="359" value="0"><span class="v" id="t0v"></span></div>
</fieldset>
<fieldset><legend>camera</legend>
 <div>
  <!-- framesize_t indices, verified against the esp32-camera bundled with
       arduino-esp32 3.3.7; older releases number these differently. -->
  <select id="framesize">
   <option value="6">QVGA 320x240</option><option value="9">CIF 400x296</option>
   <option value="10" selected>VGA 640x480</option><option value="11">SVGA 800x600</option>
   <option value="12">XGA 1024x768</option><option value="14">SXGA 1280x1024</option>
   <option value="15">UXGA 1600x1200</option>
  </select>
  <label style="width:auto"> quality</label><input type="range" id="quality" min="8" max="40" value="12">
  <label style="width:auto"> sat</label><input type="range" id="saturation" min="-2" max="2" value="0">
 </div>
 <div>
  <label style="width:auto">frame interval</label>
  <input type="range" id="interval" min="0" max="1000" step="25" value="150">
  <span class="v" id="intervalv"></span> ms
 </div>
 <div>
  <label style="width:auto"><input type="checkbox" id="awb" checked>AWB</label>
  <label style="width:auto"><input type="checkbox" id="aec" checked>AEC</label>
  <label style="width:auto"><input type="checkbox" id="agc" checked>AGC</label>
  <label style="width:auto"><input type="checkbox" id="hmirror">hmirror</label>
  <label style="width:auto"><input type="checkbox" id="vflip">vflip</label>
 </div>
</fieldset>
<div id="st"></div>
<script>
const $=id=>document.getElementById(id);
const img=$('cam'), ovl=$('ovl'), ctx=ovl.getContext('2d');
img.src='http://'+location.hostname+':81/stream';

const P=['cx','cy','ri','ro','t0'];
P.forEach(k=>{const s=localStorage.getItem('omni_'+k); if(s!==null)$(k).value=s;
             $(k).oninput=()=>{localStorage.setItem('omni_'+k,$(k).value);draw();};});

function draw(){
  const w=img.naturalWidth||img.width, h=img.naturalHeight||img.height;
  if(!w||!h){requestAnimationFrame(draw);return;}
  // Overlay is drawn in native sensor pixels, then CSS-scaled onto the image.
  if(ovl.width!=w||ovl.height!=h){ovl.width=w;ovl.height=h;
    $('cx').max=w;$('cy').max=h;$('ri').max=Math.round(Math.hypot(w,h)/2);$('ro').max=$('ri').max;}
  ovl.style.width=img.clientWidth+'px'; ovl.style.height=img.clientHeight+'px';
  const cx=+$('cx').value, cy=+$('cy').value, ri=+$('ri').value, ro=+$('ro').value, t0=+$('t0').value;
  P.forEach(k=>$(k+'v').textContent=$(k).value);

  ctx.clearRect(0,0,w,h);
  ctx.lineWidth=Math.max(1,w/500);
  ctx.strokeStyle='#0f0'; ctx.beginPath(); ctx.arc(cx,cy,ro,0,7); ctx.stroke();
  ctx.strokeStyle='#f80'; ctx.beginPath(); ctx.arc(cx,cy,ri,0,7); ctx.stroke();
  // 72 rays at 5 deg; every 6th (30 deg) is highlighted.
  for(let i=0;i<72;i++){
    const a=(t0+i*5)*Math.PI/180;
    ctx.strokeStyle=(i%6==0)?'rgba(255,255,0,.9)':'rgba(0,255,255,.35)';
    ctx.beginPath();
    ctx.moveTo(cx+ri*Math.cos(a),cy+ri*Math.sin(a));
    ctx.lineTo(cx+ro*Math.cos(a),cy+ro*Math.sin(a));
    ctx.stroke();
  }
  const inside = cx-ro>=0 && cy-ro>=0 && cx+ro<=w && cy+ro<=h;
  $('st').textContent='frame '+w+'x'+h+'   outer circle inside frame: '+(inside?'YES':'NO (FOV is clipped)');
  requestAnimationFrame(draw);
}
draw();

function ctrl(v,val){fetch('/ctrl?var='+v+'&val='+val);}
['framesize','quality','saturation','interval'].forEach(k=>
  $(k).onchange=$(k).oninput=()=>{ctrl(k,$(k).value); const l=$(k+'v'); if(l)l.textContent=$(k).value;});
$('intervalv').textContent=$('interval').value;
['awb','aec','agc','hmirror','vflip'].forEach(k=>$(k).onchange=()=>ctrl(k,$(k).checked?1:0));
fetch('/status').then(r=>r.json()).then(s=>{
  for(const k in s){const e=$(k); if(!e)continue;
    if(e.type=='checkbox')e.checked=!!s[k]; else e.value=s[k];}
}).catch(()=>{});
</script>
)HTML";
