"""Small localhost dashboard for tuning the landing PID."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import queue
import threading
from typing import Any


AXES = ("x", "y", "z", "yaw")


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


class TelemetryStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._telemetry: dict[str, Any] | None = None

    def update(self, raw: str) -> bool:
        try:
            value = json.loads(raw)
        except (TypeError, ValueError):
            return False
        if not isinstance(value, dict):
            return False
        with self._lock:
            self._telemetry = value
        return True

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            value = deepcopy(self._telemetry)
        return {"connected": value is not None, "telemetry": value}


@dataclass
class _GainWork:
    parameters: dict[str, float]
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
            if self.path not in ("/api/gains", "/api/distance"):
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
                else:
                    parameters = {"pid_align_distance_setpoint_m": parse_distance_request(payload)}
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
main{max-width:760px;margin:auto}
.card{background:#1f2937;border-radius:10px;padding:18px;margin:14px 0}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:end}
button,input{font:inherit;border-radius:6px;border:1px solid #4b5563;padding:8px}
button,input{background:#111827;color:#fff}button{cursor:pointer}
.axis.on,.apply{background:#2563eb}label{display:grid;gap:5px}input{width:105px}
.badge{padding:5px 9px;border-radius:12px;background:#374151}
.bad{background:#b91c1c}.good{background:#047857}
canvas{width:100%;height:230px;background:#0b1020;border-radius:6px}
small{color:#9ca3af}#message{min-height:20px}
</style></head><body><main><h2>Landing PID tuner</h2>
<div class="row">
<span id="link" class="badge bad">waiting for PID data</span>
<span id="state" class="badge">state —</span>
<span id="sat" class="badge">not saturated</span>
<span id="pause" class="badge">descent active</span></div>
<div class="card"><div class="row" id="axes"></div>
<div class="row" style="margin-top:14px">
<label>Kp<input id="kp" type="number" min="0" step="0.01"></label>
<label>Ki<input id="ki" type="number" min="0" step="0.01"></label>
<label>Kd<input id="kd" type="number" min="0" step="0.01"></label>
<button class="apply" id="apply">Apply</button>
<button id="undo">Undo</button><button id="ponly">P-only</button>
</div><p id="message"></p></div>
<div class="card" id="distance-card" hidden>
<div class="row"><label>Dock distance setpoint (m)
<input id="distance" type="number" min="0.3" max="20" step="0.05"></label>
<button class="apply" id="apply-distance">Apply Z setpoint</button></div>
<p id="distance-status">Waiting for dock distance</p>
<small>Larger distance climbs; smaller distance descends. Available only during alignment debug.</small>
</div>
<div class="card"><canvas id="plot" width="720" height="230"></canvas>
<small>error (blue) · command (orange), auto-scaled</small></div>
<button id="download">Download YAML</button></main><script>
const names=['x','y','z','yaw'], fields=['kp','ki','kd'];
let axis='x', latest=null, dirty=false, previous=null;
const history={}; names.forEach(a=>history[a]={e:[],c:[]});
const axes=document.querySelector('#axes');
names.forEach(a=>{
  let b=document.createElement('button'); b.textContent=a.toUpperCase();
  b.className='axis'; b.onclick=()=>select(a); b.id='axis-'+a; axes.appendChild(b);
});
fields.forEach(n=>document.querySelector('#'+n).oninput=()=>dirty=true);
function select(a){
  axis=a; dirty=false;
  document.querySelector('#distance-card').hidden=a!=='z';
  document.querySelectorAll('.axis').forEach(
    b=>b.classList.toggle('on',b.id==='axis-'+a));
  fill();
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
function draw(){
  let c=document.querySelector('#plot'),g=c.getContext('2d'),h=history[axis];
  let all=h.e.concat(h.c),scale=Math.max(.001,...all.map(Math.abs));
  g.clearRect(0,0,c.width,c.height);g.strokeStyle='#475569';g.beginPath();
  g.moveTo(0,c.height/2);g.lineTo(c.width,c.height/2);g.stroke();
  [[h.e,'#60a5fa'],[h.c,'#fb923c']].forEach(([v,col])=>{
    g.strokeStyle=col;g.beginPath();v.forEach((x,i)=>{
      let px=i*c.width/119,py=c.height/2-x/scale*c.height*.45;
      i?g.lineTo(px,py):g.moveTo(px,py);
    });g.stroke();
  });
}
async function poll(){
  try{
    let s=await(await fetch('/api/state')).json();latest=s.telemetry;
    let ok=s.connected,link=document.querySelector('#link');
    link.textContent=ok?'PID data live':'waiting for PID data';
    link.className='badge '+(ok?'good':'bad');
    if(ok){
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
      if(Number.isFinite(e)&&Number.isFinite(c)){
        let h=history[axis];h.e.push(e);h.c.push(c);
        if(h.e.length>120){h.e.shift();h.c.shift()}
      }draw();
    }
  }catch(e){}setTimeout(poll,200);
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

        def submit(self, parameters: dict[str, float]) -> dict[str, Any]:
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
