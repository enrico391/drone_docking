"""Small localhost dashboard for tuning the landing PID."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import queue
import threading
import time
from typing import Any


AXES = ("x", "y", "z", "yaw")
HOLD_STATES = ("none", "PID_ALIGN", "DESCEND_OUTER", "ACQUIRE_CENTRAL",
               "DESCEND_CENTRAL", "HOLD_CENTRAL")


def parse_gain_request(payload: Any) -> tuple[str, float, float, float]:
    if not isinstance(payload, dict) or payload.get("axis") not in AXES:
        raise ValueError("axis must be x, y, z, or yaw")
    values = []
    for name in ("kp", "ki", "kd"):
        value = payload.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} must be a number")
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        values.append(value)
    return payload["axis"], *values


def parse_distance_request(payload: Any) -> float:
    value = payload.get("distance_m") if isinstance(payload, dict) else None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("dock distance must be a number in metres")
    value = float(value)
    if not math.isfinite(value) or not 0.3 <= value <= 20.0:
        raise ValueError("dock distance must be between 0.3 and 20 m")
    return value


def parse_hold_request(payload: Any) -> str:
    value = payload.get("pid_hold_at_state") if isinstance(payload, dict) else None
    if not isinstance(value, str) or value not in HOLD_STATES:
        raise ValueError("pid_hold_at_state must be one of " + ", ".join(HOLD_STATES))
    return value


class TelemetryStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._telemetry: dict[str, Any] | None = None
        self._received_at: float | None = None

    def update(self, raw: str) -> bool:
        try:
            value = json.loads(raw)
        except (TypeError, ValueError):
            return False
        if not isinstance(value, dict):
            return False
        with self._lock:
            self._telemetry = value
            self._received_at = time.monotonic()
        return True

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            value = deepcopy(self._telemetry)
            received_at = self._received_at
        fresh = received_at is not None and time.monotonic() - received_at < 2.0
        return {"connected": fresh, "telemetry": value if fresh else None}


@dataclass
class _GainWork:
    parameters: dict[str, float | str]
    done: threading.Event
    result: dict[str, Any] | None = None


class _WebApp:
    def __init__(self, telemetry: TelemetryStore, submit: Any) -> None:
        self.telemetry = telemetry
        self.submit = submit


def _handler_for(app: _WebApp) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def _json(self, status: int, value: Any) -> None:
            body = json.dumps(value).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/api/state":
                self._json(200, app.telemetry.snapshot())
                return
            if self.path != "/":
                self.send_error(404)
                return
            body = DASHBOARD_HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self) -> None:  # noqa: N802
            if self.path not in ("/api/gains", "/api/distance", "/api/hold"):
                self.send_error(404)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 4096:
                    raise ValueError("invalid request size")
                payload = json.loads(self.rfile.read(length))
                if self.path == "/api/gains":
                    axis, kp, ki, kd = parse_gain_request(payload)
                    parameters = {f"pid_{axis}_{name}": value for name, value in
                                  zip(("kp", "ki", "kd"), (kp, ki, kd))}
                elif self.path == "/api/distance":
                    parameters = {"pid_align_distance_setpoint_m": parse_distance_request(payload)}
                else:
                    parameters = {"pid_hold_at_state": parse_hold_request(payload)}
            except (ValueError, TypeError) as exc:
                self._json(400, {"ok": False, "message": str(exc)})
                return
            result = app.submit(parameters)
            self._json(200 if result.get("ok") else 503, result)

        def log_message(self, _format: str, *_args: Any) -> None:
            pass

    return Handler


DASHBOARD_HTML = r"""<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width">
<title>Landing PID tuner</title><style>
body{font:15px system-ui;background:#111827;color:#e5e7eb;margin:0;padding:24px}
main{max-width:1320px;margin:auto}
.dashboard{display:grid;grid-template-columns:minmax(320px,1fr) minmax(0,1.25fr);gap:16px;align-items:start}
.dashboard .card{margin:0 0 16px}
@media(max-width:850px){.dashboard{grid-template-columns:minmax(0,1fr)}}
.card{background:#1f2937;border-radius:10px;padding:18px;margin:14px 0}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:end}
button,input,select{font:inherit;border-radius:6px;border:1px solid #4b5563;padding:8px}
button,input,select{background:#111827;color:#fff}button{cursor:pointer}
.axis.on,.apply{background:#2563eb}label{display:grid;gap:5px}input{width:105px}
.badge{padding:5px 9px;border-radius:12px;background:#374151}
.bad{background:#b91c1c}.good{background:#047857}
canvas{width:100%;height:230px;background:#0b1020;border-radius:6px}
#plot{height:auto;aspect-ratio:720/270;display:block;margin-top:8px}
#position{height:auto;aspect-ratio:820/430;display:block}
.readings{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px;margin:12px 0}
.reading{min-width:0;padding:9px;background:#111827;border:1px solid #374151;border-radius:6px}
.reading.on{border-color:#60a5fa;color:#bfdbfe}
.reading strong{display:block;font-size:20px;font-variant-numeric:tabular-nums}
small{color:#9ca3af}#message{min-height:20px}
</style></head><body><main><h2>Landing PID tuner</h2>
<div class="row">
<span id="link" class="badge bad">waiting for PID data</span>
<span id="state" class="badge">state —</span>
<span id="sat" class="badge">not saturated</span>
<span id="pause" class="badge">descent active</span></div>
<div class="dashboard"><div class="dashboard-left">
<div class="card"><h3>Position selection and PID gains</h3><div class="row" id="axes"></div>
<div class="row" style="margin-top:14px">
<label>Kp<input id="kp" type="number" min="0" step="0.01"></label>
<label>Ki<input id="ki" type="number" min="0" step="0.01"></label>
<label>Kd<input id="kd" type="number" min="0" step="0.01"></label>
<button class="apply" id="apply">Apply</button>
<button id="undo">Undo</button><button id="ponly">P-only</button>
</div><p id="message"></p></div>
<div class="card"><h3>Landing hold</h3>
<div class="row"><label>Hold at state
<select id="hold" disabled>
<option value="none">none (continue landing)</option>
<option value="PID_ALIGN">PID_ALIGN</option>
<option value="DESCEND_OUTER">DESCEND_OUTER</option>
<option value="ACQUIRE_CENTRAL">ACQUIRE_CENTRAL</option>
<option value="DESCEND_CENTRAL">DESCEND_CENTRAL</option>
<option value="HOLD_CENTRAL">HOLD_CENTRAL (before touchdown)</option>
</select></label><button class="apply" id="apply-hold" disabled>Apply hold</button></div>
<p id="hold-status">Waiting for live PID data</p><p id="hold-message" role="status"></p>
<small>Changing a hold already passed or after touchdown commitment is rejected. Releasing HOLD_CENTRAL requires fresh central vision and stable alignment.</small>
</div>
<div class="card" id="distance-card" hidden>
<div class="row"><label>Dock distance setpoint (m)
<input id="distance" type="number" min="0.3" max="20" step="0.05"></label>
<button class="apply" id="apply-distance">Apply Z setpoint</button></div>
<p id="distance-status">Waiting for dock distance</p>
<small>Larger distance climbs; smaller distance descends. Available only during alignment debug.</small>
</div>
<div class="card"><h3>Tracking errors</h3>
<div id="readings" class="readings" aria-live="off"></div>
<small>X forward · Y left · Z dock-distance setpoint minus measured distance (not height) · Yaw counter-clockwise.</small>
</div>
<div class="card"><h3>Selected-axis history</h3>
<div class="row"><label>Error range ± (<span id="error-unit">m</span>)
<input id="error-scale" type="number" min="0.01" max="1000" step="any" value="2"></label>
<label>Command range ± (<span id="command-unit">m/s</span>)
<input id="command-scale" type="number" min="0.01" max="1000" step="any" value="1"></label>
<button id="reset-history-scale">Reset axis scales</button></div>
<p id="history-scale-message" role="status"></p>
<canvas id="plot" width="720" height="270" role="img" aria-label="Fixed-scale tracking error and command history"></canvas>
<small>Tracking error (blue, left scale) · command (orange, right scale). Scales stay fixed until changed; out-of-range values are clipped and flagged. Yaw in degrees and degrees/s.</small></div>
<button id="download">Download YAML</button>
</div><div class="dashboard-right">
<div class="card"><h3>Dock-relative error surface (estimated)</h3>
<div class="row"><label>Surface range ± (m)
<input id="surface-scale" type="number" min="0.01" max="1000" step="any" value="2"></label>
<button id="reset-surface-scale">Reset surface scale</button></div>
<p id="surface-scale-message" role="status"></p>
<canvas id="position" width="820" height="430" role="img" aria-label="Estimated 3D wireframe of dock-relative errors with live error marker"></canvas>
<small>X forward · Y left · Z distance-setpoint error (not height). Adjustable fixed scale; out-of-range positions are clipped and flagged. Colored wireframe interpolates only between nearby sampled errors; blank areas are unknown. Marker is the current error, not an absolute drone position. Yaw is shown by the arrow.</small>
</div></div></div></main><script>
const names=['x','y','z','yaw'], fields=['kp','ki','kd'];
const plotRanges={x:{error:2,command:1,unit:'m',commandUnit:'m/s'},
  y:{error:2,command:1,unit:'m',commandUnit:'m/s'},
  z:{error:3,command:1,unit:'m',commandUnit:'m/s'},
  yaw:{error:180,command:60,unit:'°',commandUnit:'°/s'}};
const defaultPlotRanges=JSON.parse(JSON.stringify(plotRanges));
let surfaceScale=2; // Metres per axis; do not zoom as errors improve.
let axis='x', latest=null, dirty=false, previous=null, holdDirty=false;
const history={}; names.forEach(a=>history[a]={e:[],c:[]});
const surfaceSamples=[];
let surfaceSetpoint=null, lastSurfaceTime=null;
const axes=document.querySelector('#axes'), readings=document.querySelector('#readings');
names.forEach(a=>{
  const item=document.createElement('div');item.className='reading';item.id='reading-'+a;
  const label=document.createElement('span');label.textContent=a.toUpperCase()+' error';
  const value=document.createElement('strong');value.textContent='—';
  item.append(label,value);readings.appendChild(item);
});
names.forEach(a=>{
  let b=document.createElement('button'); b.textContent=a.toUpperCase();
  b.className='axis'; b.onclick=()=>select(a); b.id='axis-'+a; axes.appendChild(b);
});
fields.forEach(n=>document.querySelector('#'+n).oninput=()=>dirty=true);
function showScaleControls(){
  const range=plotRanges[axis];
  document.querySelector('#error-scale').value=range.error;
  document.querySelector('#command-scale').value=range.command;
  document.querySelector('#error-unit').textContent=range.unit;
  document.querySelector('#command-unit').textContent=range.commandUnit;
  document.querySelector('#history-scale-message').textContent='';
}
function setScale(input,limit,message,apply){
  const raw=input.value.trim(),value=Number(raw);
  if(!raw || !Number.isFinite(value) || value<0.01 || value>1000){
    input.value=limit;
    message.textContent='Scale must be between 0.01 and 1000.';
    return;
  }
  apply(value);message.textContent='';
}
document.querySelector('#error-scale').onchange=()=>{
  const input=document.querySelector('#error-scale');
  setScale(input,plotRanges[axis].error,document.querySelector('#history-scale-message'),
    value=>{plotRanges[axis].error=value;draw()});
};
document.querySelector('#command-scale').onchange=()=>{
  const input=document.querySelector('#command-scale');
  setScale(input,plotRanges[axis].command,document.querySelector('#history-scale-message'),
    value=>{plotRanges[axis].command=value;draw()});
};
document.querySelector('#surface-scale').onchange=()=>{
  const input=document.querySelector('#surface-scale');
  setScale(input,surfaceScale,document.querySelector('#surface-scale-message'),
    value=>{surfaceScale=value;drawPosition()});
};
document.querySelector('#reset-history-scale').onclick=()=>{
  plotRanges[axis].error=defaultPlotRanges[axis].error;
  plotRanges[axis].command=defaultPlotRanges[axis].command;
  showScaleControls();draw();
};
document.querySelector('#reset-surface-scale').onclick=()=>{
  surfaceScale=2;document.querySelector('#surface-scale').value=surfaceScale;
  document.querySelector('#surface-scale-message').textContent='';drawPosition();
};
function select(a){
  axis=a; dirty=false;
  showScaleControls();draw();
  document.querySelector('#distance-card').hidden=a!=='z';
  document.querySelectorAll('.axis').forEach(
    b=>b.classList.toggle('on',b.id==='axis-'+a));
  fill();drawPosition();
}
function fill(){
  let d=latest?.axes?.[axis]; if(!d||dirty)return;
  fields.forEach(n=>document.querySelector('#'+n).value=d[n]??0);
}
function values(){
  let v={axis};
  fields.forEach(n=>v[n]=Number(document.querySelector('#'+n).value)); return v;
}
async function send(v){
  let m=document.querySelector('#message');
  try{
    let r=await fetch('/api/gains',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify(v)});
    let x=await r.json(); m.textContent=x.message;
    if(x.ok){dirty=false;return true}
  }catch(e){m.textContent=e.message} return false;
}
document.querySelector('#apply').onclick=async()=>{
  let v=values(), old=latest?.axes?.[axis];
  if(await send(v)&&old)previous={axis,kp:old.kp,ki:old.ki,kd:old.kd};
};
document.querySelector('#undo').onclick=async()=>{
  if(previous){axis=previous.axis;select(axis);await send(previous)}
};
document.querySelector('#ponly').onclick=()=>{
  document.querySelector('#ki').value=0;document.querySelector('#kd').value=0;
  dirty=true;
};
document.querySelector('#apply-distance').onclick=async()=>{
  let m=document.querySelector('#message');
  let v=Number(document.querySelector('#distance').value);
  if(!Number.isFinite(v)||v<0.3||v>20){m.textContent='Dock distance must be between 0.3 and 20 m';return}
  try{
    let r=await fetch('/api/distance',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({distance_m:v})});
    m.textContent=(await r.json()).message;
  }catch(e){m.textContent=e.message}
};
const holdSelect=document.querySelector('#hold');
holdSelect.onchange=()=>holdDirty=true;
document.querySelector('#apply-hold').onclick=async()=>{
  const button=document.querySelector('#apply-hold'),message=document.querySelector('#hold-message');
  button.disabled=true;
  try{
    const r=await fetch('/api/hold',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({pid_hold_at_state:holdSelect.value})});
    const result=await r.json();message.textContent=result.message;
    if(result.ok && latest?.pid_hold_at_state===holdSelect.value)holdDirty=false;
  }catch(e){message.textContent=e.message}
  button.disabled=!latest;
};
// Keep only distinct, fresh observations under the same distance setpoint.
// Z is a distance-setpoint error, not a measured vertical separation.
function recordSurfaceSample(){
  const e=latest?.error;
  if(!e || !Number.isFinite(e.x) || !Number.isFinite(e.y) ||
     !Number.isFinite(e.z) || !Number.isFinite(e.yaw) ||
     !Number.isFinite(latest.dock_age_s) || latest.dock_age_s >= 1){
    surfaceSamples.length=0;surfaceSetpoint=null;lastSurfaceTime=null;return;
  }
  const setpoint=latest.distance_setpoint_m;
  if(!Number.isFinite(setpoint)){
    surfaceSamples.length=0;surfaceSetpoint=null;lastSurfaceTime=null;return;
  }
  if(surfaceSetpoint!==setpoint){surfaceSamples.length=0;surfaceSetpoint=setpoint;lastSurfaceTime=null}
  if(Number.isFinite(latest.time_s)){
    if(lastSurfaceTime===latest.time_s)return;
    if(lastSurfaceTime!==null && latest.time_s<lastSurfaceTime)surfaceSamples.length=0;
    lastSurfaceTime=latest.time_s;
  }
  const previous=surfaceSamples[surfaceSamples.length-1];
  if(previous && Math.hypot(e.x-previous.x,e.y-previous.y)<0.025 &&
     Math.abs(e.z-previous.z)<0.025)return;
  surfaceSamples.push({x:e.x,y:e.y,z:e.z});
  if(surfaceSamples.length>160)surfaceSamples.shift();
}
// Interpolate only inside small triangles of observed XY locations; never extrapolate.
function surfaceHeight(x,y,cell){
  if(surfaceSamples.length<3)return null;
  const nearby=surfaceSamples.map(p=>({p,d:Math.hypot(p.x-x,p.y-y)}))
    .filter(v=>v.d<=cell*4 && [v.p.x,v.p.y,v.p.z].every(n=>Math.abs(n)<=surfaceScale))
    .sort((a,b)=>a.d-b.d).slice(0,7);
  let best=null,bestArea=Infinity;
  for(let a=0;a<nearby.length;a++)for(let b=a+1;b<nearby.length;b++)
    for(let c=b+1;c<nearby.length;c++){
      const p=nearby[a].p,q=nearby[b].p,r=nearby[c].p;
      if(Math.hypot(p.x-q.x,p.y-q.y)>cell*4 ||
         Math.hypot(p.x-r.x,p.y-r.y)>cell*4 ||
         Math.hypot(q.x-r.x,q.y-r.y)>cell*4)continue;
      const cross=(q.y-r.y)*(p.x-r.x)+(r.x-q.x)*(p.y-r.y);
      if(Math.abs(cross)<cell*cell*.06)continue;
      const u=((q.y-r.y)*(x-r.x)+(r.x-q.x)*(y-r.y))/cross;
      const v=((r.y-p.y)*(x-r.x)+(p.x-r.x)*(y-r.y))/cross;
      const t=1-u-v;
      if(u < -1e-8 || v < -1e-8 || t < -1e-8)continue;
      const area=Math.abs(cross);
      if(area<bestArea){bestArea=area;best=u*p.z+v*q.z+t*r.z}
    }
  return best;
}
// Isometric projection of BODY_FLU X (forward), Y (left), and Z error.
function drawPosition(){
  const canvas=document.querySelector('#position'), g=canvas.getContext('2d');
  const w=canvas.width,h=canvas.height,origin=[w/2,h*.57];
  g.clearRect(0,0,w,h);
  const error=latest?.error;
  const valid=error && Number.isFinite(error.x) && Number.isFinite(error.y) &&
    Number.isFinite(error.yaw) && Number.isFinite(latest.dock_age_s) &&
    latest.dock_age_s < 1;
  const hasZ=valid && Number.isFinite(error.z);
  names.forEach(a=>{
    const item=document.querySelector('#reading-'+a);
    item.classList.toggle('on',a===axis);
    item.querySelector('strong').textContent=valid?
      (a==='yaw'?(error.yaw*180/Math.PI).toFixed(1)+'°':
        (a==='z'&&!hasZ?'—':error[a].toFixed(2)+' m')):'—';
  });
  const scale=surfaceScale;
  // Keep the current marker and surface within the fixed reference box.
  const clamp=v=>Math.max(-scale,Math.min(scale,v));
  function project(x,y,z){x=clamp(x);y=clamp(y);z=clamp(z);
    return [origin[0]+(x-y)*70/scale,origin[1]+(x+y)*32/scale-z*80/scale]}
  function line(a,b,color,width=2,dash=[]){
    g.beginPath();g.setLineDash(dash);g.moveTo(...a);g.lineTo(...b);
    g.strokeStyle=color;g.lineWidth=width;g.stroke();g.setLineDash([]);
  }
  const colors={x:'#60a5fa',y:'#34d399',z:'#fbbf24'};
  // Base grid and box give the surface a stable reference even before samples arrive.
  for(let i=-4;i<=4;i++){
    const v=i*scale/4;
    line(project(v,-scale,0),project(v,scale,0),'#334155',1);
    line(project(-scale,v,0),project(scale,v,0),'#334155',1);
  }
  for(const x of [-scale,scale])for(const y of [-scale,scale])
    line(project(x,y,-scale),project(x,y,scale),'#334155',1);
  const n=16,cell=2*scale/n;
  const grid=surfaceSamples.length<3?[]:
    Array.from({length:n+1},(_,j)=>Array.from({length:n+1},(_,i)=>{
      const x=-scale+i*cell,y=-scale+j*cell;
      return {x,y,z:surfaceHeight(x,y,cell)};
    }));
  // Paint farther cells first. A cell needs all four supported corners.
  const cells=[];
  for(let j=0;j<grid.length-1;j++)for(let i=0;i<n;i++){
    const corners=[grid[j][i],grid[j][i+1],grid[j+1][i+1],grid[j+1][i]];
    if(corners.every(p=>p.z!==null))cells.push({corners,depth:i+j});
  }
  cells.sort((a,b)=>a.depth-b.depth);
  for(const {corners} of cells){
    const avg=corners.reduce((sum,p)=>sum+p.z,0)/4;
    const hue=210-170*(avg/scale+1)/2;
    g.beginPath();corners.forEach((p,i)=>{
      const pt=project(p.x,p.y,p.z);
      i?g.lineTo(...pt):g.moveTo(...pt);
    });g.closePath();
    g.fillStyle=`hsla(${hue},85%,50%,.72)`;g.fill();
    g.strokeStyle='rgba(226,232,240,.65)';g.lineWidth=.8;g.stroke();
  }
  for(const a of ['x','y','z']){
    const end=project(a==='x'?scale:0,a==='y'?scale:0,a==='z'?scale:0);
    line(origin,end,colors[a],a===axis?3:1.5);
    g.fillStyle=colors[a];g.font='bold 15px system-ui';g.fillText(a.toUpperCase(),end[0]+5,end[1]-5);
  }
  g.fillStyle='#e5e7eb';g.beginPath();g.arc(...origin,5,0,2*Math.PI);g.fill();
  g.font='13px system-ui';g.fillText('dock / target',origin[0]+10,origin[1]+16);
  g.fillStyle='#cbd5e1';g.font='12px system-ui';
  g.fillText('estimated surface · '+surfaceSamples.length+' samples · '+cells.length+' mesh cells',12,h-30);
  if(!valid){
    g.textAlign='center';g.fillStyle='#fca5a5';g.font='16px system-ui';
    g.fillText('Waiting for fresh X, Y and yaw errors',w/2,35);
    g.textAlign='left';return;
  }
  const z=hasZ?error.z:0;
  if(!hasZ){g.fillStyle=colors.z;g.fillText('Z error unavailable (no setpoint); marker at Z=0 for display',12,28)}
  if([error.x,error.y,z].some(v=>Math.abs(v)>scale)){
    g.fillStyle='#fca5a5';g.font='13px system-ui';
    g.fillText('Position outside ±'+scale+' m view (marker clipped)',12,hasZ?28:46);
  }
  const point=project(error.x,error.y,z);
  line(origin,point,'#e5e7eb',2,[5,4]);
  for(const a of ['x','y','z']){
    if(a==='z'&&!hasZ)continue;
    const vec={x:[error.x,0,0],y:[0,error.y,0],z:[0,0,z]}[a];
    line(origin,project(...vec),colors[a],a===axis?4:2);
  }
  g.fillStyle='#f8fafc';g.beginPath();g.arc(...point,9,0,2*Math.PI);g.fill();
  g.strokeStyle=axis==='yaw'?'#fb7185':'#94a3b8';g.lineWidth=3;g.stroke();
  // Yaw rotates the forward arrow toward body-left (counter-clockwise).
  const tip=project(error.x+Math.cos(error.yaw)*scale*.27,
    error.y+Math.sin(error.yaw)*scale*.27,z);
  line(point,tip,'#fb7185',axis==='yaw'?4:3);
  const angle=Math.atan2(tip[1]-point[1],tip[0]-point[0]);
  for(const side of [-1,1]){
    line(tip,[tip[0]-10*Math.cos(angle+side*.5),
      tip[1]-10*Math.sin(angle+side*.5)],'#fb7185',3);
  }
  g.fillStyle='#f8fafc';g.font='bold 14px system-ui';
  g.fillText('error'+(hasZ?'':' (Z unknown)'),point[0]+12,point[1]-12);
  g.fillStyle='#9ca3af';g.font='12px system-ui';
  g.fillText('scale: '+scale.toFixed(1)+' m per axis',12,h-12);
}
function draw(){
  let c=document.querySelector('#plot'),g=c.getContext('2d'),h=history[axis];
  const range=plotRanges[axis],left=72,right=c.width-90,top=35,bottom=c.height-32;
  const yFor=(value,limit)=>bottom-(value/limit+1)*(bottom-top)/2;
  g.clearRect(0,0,c.width,c.height);
  g.font='12px system-ui';g.textAlign='left';g.fillStyle='#60a5fa';
  g.fillText('error ('+range.unit+')',left,18);
  g.fillStyle='#fb923c';g.fillText('command ('+range.commandUnit+')',right-95,18);
  for(const fraction of [-1,-.5,0,.5,1]){
    const y=yFor(fraction,1);
    g.strokeStyle=fraction===0?'#94a3b8':'#334155';g.lineWidth=fraction===0?2:1;
    g.beginPath();g.moveTo(left,y);g.lineTo(right,y);g.stroke();
    g.fillStyle='#93c5fd';g.textAlign='right';
    g.fillText((fraction*range.error).toFixed(axis==='yaw'?0:1),left-8,y+4);
    g.fillStyle='#fdba74';g.textAlign='left';
    g.fillText((fraction*range.command).toFixed(axis==='yaw'?0:1),right+8,y+4);
  }
  g.fillStyle='#9ca3af';g.textAlign='left';
  g.fillText('oldest',left,bottom+20);g.textAlign='right';g.fillText('latest',right,bottom+20);
  g.save();g.beginPath();g.rect(left,top,right-left,bottom-top);g.clip();
  [[h.e,'#60a5fa',range.error],[h.c,'#fb923c',range.command]].forEach(([values,color,limit])=>{
    g.strokeStyle=color;g.lineWidth=2;g.beginPath();
    values.forEach((value,i)=>{
      const displayed=axis==='yaw'?value*180/Math.PI:value;
      const px=left+i*(right-left)/119,py=yFor(displayed,limit);
      i?g.lineTo(px,py):g.moveTo(px,py);
    });g.stroke();
  });g.restore();
  const errorClipped=h.e.some(v=>Math.abs(axis==='yaw'?v*180/Math.PI:v)>range.error);
  const commandClipped=h.c.some(v=>Math.abs(axis==='yaw'?v*180/Math.PI:v)>range.command);
  if(errorClipped||commandClipped){
    g.fillStyle='#fca5a5';g.textAlign='right';
    g.fillText((errorClipped?'error':'')+(errorClipped&&commandClipped?' / ':'')+
      (commandClipped?'command':'')+' outside scale',right,top-9);
  }
  g.textAlign='left';
}
async function poll(){
  try{
    let s=await(await fetch('/api/state')).json();latest=s.connected?s.telemetry:null;
    let ok=Boolean(latest),link=document.querySelector('#link');
    link.textContent=ok?'PID data live':'waiting for PID data';
    link.className='badge '+(ok?'good':'bad');
    if(ok){
      holdSelect.disabled=false;
      document.querySelector('#apply-hold').disabled=false;
      if(holdDirty && latest.pid_hold_at_state===holdSelect.value)holdDirty=false;
      if(!holdDirty && latest.pid_hold_at_state)
        holdSelect.value=latest.pid_hold_at_state;
      document.querySelector('#hold-status').textContent=
        'Active hold: '+(latest.pid_hold_at_state??'unknown');
      document.querySelector('#state').textContent='state '+(latest.state??'—');
      let d=latest.axes?.[axis]||{},sat=document.querySelector('#sat');
      sat.textContent=d.saturated?'SATURATED':'not saturated';
      sat.className='badge '+(d.saturated?'bad':'');
      let pause=document.querySelector('#pause');
      pause.textContent=latest.descent_paused?'DESCENT PAUSED':'descent active';
      pause.className='badge '+(latest.descent_paused?'bad':'');fill();
      let card=document.querySelector('#distance-card');
      card.hidden=axis!=='z';
      let input=document.querySelector('#distance');
      let enabled=latest.alignment_only&&latest.state==='PID_ALIGN'&&
        Number.isFinite(latest.dock_distance_m)&&Number.isFinite(latest.distance_setpoint_m);
      document.querySelector('#apply-distance').disabled=!enabled;
      if(enabled&&!input.matches(':focus'))input.value=latest.distance_setpoint_m;
      document.querySelector('#distance-status').textContent=enabled?
        `Measured: ${latest.dock_distance_m.toFixed(2)} m · Target: ${latest.distance_setpoint_m.toFixed(2)} m · Error: ${(latest.error?.z??0).toFixed(2)} m · Command: ${(latest.command_body_flu?.[2]??0).toFixed(2)} m/s`:
        'Waiting for alignment and a valid dock observation';
      let i=names.indexOf(axis),e=latest.error?.[axis];
      let c=latest.command_body_flu?.[i];
      if(Number.isFinite(e)&&Number.isFinite(c)&&
         Number.isFinite(latest.dock_age_s)&&latest.dock_age_s<1){
        let h=history[axis];h.e.push(e);h.c.push(c);
        if(h.e.length>120){h.e.shift();h.c.shift()}
      }draw();
    }else{
      holdSelect.disabled=true;
      document.querySelector('#apply-hold').disabled=true;
      document.querySelector('#hold-status').textContent='Waiting for live PID data';
      document.querySelector('#state').textContent='state —';
      document.querySelector('#sat').textContent='not saturated';
      document.querySelector('#pause').textContent='descent unknown';
      document.querySelector('#apply-distance').disabled=true;
      document.querySelector('#distance-status').textContent='Waiting for alignment and a valid dock observation';
    }
    recordSurfaceSample();drawPosition();
  }catch(e){
    latest=null;document.querySelector('#link').textContent='waiting for PID data';
    document.querySelector('#link').className='badge bad';
    holdSelect.disabled=true;document.querySelector('#apply-hold').disabled=true;
    document.querySelector('#hold-status').textContent='Waiting for live PID data';
    recordSurfaceSample();drawPosition();
  }setTimeout(poll,200);
}
document.querySelector('#download').onclick=()=>{
  if(!latest?.axes)return;
  let y='flight_cycle_manager:\n  ros__parameters:\n';
  names.forEach(a=>fields.forEach(n=>
    y+=`    pid_${a}_${n}: ${latest.axes[a]?.[n]??0}\n`));
  let u=URL.createObjectURL(new Blob([y],{type:'text/yaml'}));
  let a=document.createElement('a');a.href=u;a.download='pid_tuned.yaml';a.click();
  URL.revokeObjectURL(u);
};
select('x');poll();
</script></body></html>"""


def main(args: list[str] | None = None) -> None:
    import rclpy
    from rcl_interfaces.srv import SetParametersAtomically
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    class PidTunerNode(Node):
        def __init__(self) -> None:
            super().__init__("pid_tuner")
            self.declare_parameter("port", 8765)
            self.telemetry = TelemetryStore()
            self._works: queue.Queue[_GainWork] = queue.Queue()
            self._active: tuple[_GainWork, Any] | None = None
            self._client = self.create_client(
                SetParametersAtomically,
                "/flight_cycle_manager/set_parameters_atomically",
            )
            self.create_subscription(
                String, "/flight_cycle_manager/pid_debug",
                lambda message: self.telemetry.update(message.data), 10,
            )
            self.create_timer(0.05, self._process_work)

        def submit(self, parameters: dict[str, float | str]) -> dict[str, Any]:
            work = _GainWork(parameters, threading.Event())
            self._works.put(work)
            if not work.done.wait(3.0):
                return {"ok": False, "message": "parameter update timed out"}
            return work.result or {"ok": False, "message": "parameter update failed"}

        def _process_work(self) -> None:
            if self._active is not None:
                work, future = self._active
                if not future.done():
                    return
                try:
                    outcome = future.result().result
                    work.result = {
                        "ok": bool(outcome.successful),
                        "message": "parameters applied" if outcome.successful else outcome.reason,
                    }
                except Exception as exc:  # service transport failure
                    work.result = {"ok": False, "message": str(exc)}
                work.done.set()
                self._active = None
            try:
                work = self._works.get_nowait()
            except queue.Empty:
                return
            if not self._client.service_is_ready():
                work.result = {"ok": False, "message": "flight_cycle_manager is unavailable"}
                work.done.set()
                return
            request = SetParametersAtomically.Request()
            request.parameters = [
                Parameter(name, value=value).to_parameter_msg()
                for name, value in work.parameters.items()
            ]
            self._active = work, self._client.call_async(request)

    rclpy.init(args=args)
    node = PidTunerNode()
    port = int(node.get_parameter("port").value)
    server = ThreadingHTTPServer(
        ("127.0.0.1", port), _handler_for(_WebApp(node.telemetry, node.submit))
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f"PID tuner: http://127.0.0.1:{port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
