"""Local HTTP PID tuner with pose-only telemetry, adapted from debug_refactor."""

from copy import deepcopy
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import queue
import threading
import time

from .px4_control import LIVE_TRANSITION_PARAMETERS, PID_AXES


FLIGHT_ACTIONS = ("start_cycle", "request_dock")


def parse_action_request(payload):
    if (not isinstance(payload, dict) or set(payload) != {"action", "confirmed"}
            or payload["action"] not in FLIGHT_ACTIONS or payload["confirmed"] is not True):
        raise ValueError("a supported flight action and confirmed: true are required")
    return payload["action"]


def parse_gain_request(payload):
    """Update one complete axis in the base, near or far gain bank."""
    if not isinstance(payload, dict) or payload.get("axis") not in PID_AXES:
        raise ValueError("axis must be x, y, z, or yaw")
    bank = payload.get("bank", "base")
    if bank not in ("base", "near", "far"):
        raise ValueError("bank must be base, near, or far")
    prefix = (f"pid_{payload['axis']}" if bank == "base"
              else f"pid_gain_{bank}_{payload['axis']}")
    parameters = {}
    for gain in ("kp", "ki", "kd"):
        value = payload.get(gain)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or (value < 0 and not (bank != "base" and value == -1))):
            raise ValueError(f"{gain} must be finite and nonnegative (-1 inherits base in near/far)")
        parameters[f"{prefix}_{gain}"] = float(value)
    return parameters


def parse_transition_request(payload):
    """Allow only explicitly selected positive transition settings, never arbitrary ROS params."""
    parameters = payload.get("parameters") if isinstance(payload, dict) else None
    if not isinstance(parameters, dict) or not parameters:
        raise ValueError("parameters must be a nonempty object")
    if parameters.keys() - LIVE_TRANSITION_PARAMETERS:
        raise ValueError("unsupported transition parameter")
    for name, value in parameters.items():
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0):
            raise ValueError(f"{name} must be finite and positive")
    return {name: float(value) for name, value in parameters.items()}


class TelemetryStore:
    """Thread-safe latest sample; never display stale tracking as live."""

    def __init__(self, *, now=time.monotonic):
        self._now = now
        self._lock = threading.Lock()
        self._telemetry = None
        self._received_at = -math.inf

    def update(self, raw):
        try:
            value = json.loads(raw)
            if not isinstance(value, dict):
                return False
            json.dumps(value, allow_nan=False)
        except (TypeError, ValueError, OverflowError):
            return False
        with self._lock:
            self._telemetry = value
            self._received_at = self._now()
        return True

    def snapshot(self):
        with self._lock:
            fresh = self._now() - self._received_at < 2.0
            return {"connected": fresh,
                    "telemetry": deepcopy(self._telemetry) if fresh else None}


def _handler_for(telemetry, submit, submit_action=None):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(3.0)

        def _reply(self, status, payload, content_type="application/json"):
            body = (json.dumps(payload, allow_nan=False) if content_type == "application/json"
                    else payload).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802
            if self.path == "/":
                self._reply(200, DASHBOARD_HTML, "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._reply(200, telemetry.snapshot())
            else:
                self.send_error(404)

        def do_POST(self):  # noqa: N802
            if self.path not in ("/api/gains", "/api/transition", "/api/action"):
                self.send_error(404)
                return
            # Do not let another website issue control requests to localhost.
            origin = self.headers.get("Origin")
            if (self.headers.get("Sec-Fetch-Site") == "cross-site"
                    or (origin is not None and origin != "http://" + self.headers.get("Host", ""))):
                self._reply(403, {"ok": False, "message": "same-origin requests only"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError("invalid request size")
                parse = {"/api/gains": parse_gain_request, "/api/transition": parse_transition_request,
                         "/api/action": parse_action_request}[self.path]
                parameters = parse(json.loads(self.rfile.read(length)))
            except (TypeError, ValueError, OverflowError, OSError) as error:
                self._reply(400, {"ok": False, "message": str(error)})
                return
            if self.path == "/api/action":
                result = (submit_action(parameters) if submit_action else
                          {"ok": False, "message": "flight actions unavailable"})
            else:
                result = submit(parameters)
            self._reply(200 if result.get("ok") else 503, result)

        def log_message(self, _format, *_args):
            pass

    return Handler


@dataclass
class _GainWork:
    parameters: dict
    deadline: float = field(default_factory=lambda: time.monotonic() + 3.0)
    done: threading.Event = field(default_factory=threading.Event)
    result: dict | None = None


class FlightActionBridge:
    """Single-flight HTTP-to-ROS Trigger bridge; dispatch only in the ROS executor."""

    def __init__(self, telemetry, clients, request_factory, *, now=time.monotonic):
        self._telemetry, self._clients = telemetry, clients
        self._request_factory, self._now = request_factory, now
        self._lock = threading.RLock()
        self._work = self._action = self._future = None

    def _blocker(self, action):
        state = self._telemetry.snapshot()
        if not state["connected"]:
            return "fresh manager telemetry required"
        info = (state["telemetry"].get("flight_actions") or {}).get(action) or {}
        if info.get("allowed") is not True:
            return info.get("reason") or "manager has not enabled this action"
        return ""

    def submit(self, action):
        if action not in FLIGHT_ACTIONS:
            return {"ok": False, "message": "unsupported flight action"}
        with self._lock:
            if self._work is not None:
                return {"ok": False, "message": "a flight request is already pending; do not retry"}
            reason = self._blocker(action)
            if reason:
                return {"ok": False, "message": reason}
            work = _GainWork({}, deadline=self._now() + 3.0)
            self._work, self._action = work, action
        try:
            got = work.done.wait(3.2)
        except Exception:
            got = False
        if not got:
            with self._lock:
                terminal = (
                    work.result is not None
                    or (self._work is work and self._now() >= work.deadline and self._future is None)
                )
            if terminal:
                return work.result
            return self._unknown_result()
        if work.result is None:
            return self._unknown_result()
        return work.result

    @staticmethod
    def _unknown_result():
        return {"ok": False, "uncertain": True,
                "message": "Flight request outcome unknown; inspect vehicle and manager state. "
                           "Do not retry automatically."}

    def process(self):
        with self._lock:
            work = self._work
            if work is None:
                return
            if self._future is None:
                reason = self._blocker(self._action)
                client = self._clients[self._action]
                if self._now() >= work.deadline or reason or not client.service_is_ready():
                    work.result = {"ok": False, "message": reason or "service unavailable or request expired; not sent"}
                else:
                    try:
                        self._future = client.call_async(self._request_factory())
                    except Exception:
                        work.result = self._unknown_result()
            if self._future is not None:
                if not self._future.done() and self._now() < work.deadline:
                    return
                try:
                    if not self._future.done():
                        self._future.cancel()
                        work.result = self._unknown_result()
                    else:
                        response = self._future.result()
                        if response is None or not hasattr(response, "success"):
                            work.result = self._unknown_result()
                        else:
                            work.result = {"ok": bool(response.success),
                                           "message": getattr(response, "message", "") or ""}
                except Exception:
                    work.result = self._unknown_result()
            if work.result is not None:
                work.done.set()
                self._work = self._action = self._future = None


def main(args=None):
    import rclpy
    from rcl_interfaces.srv import SetParametersAtomically
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class PidTunerNode(Node):
        def __init__(self):
            super().__init__("pid_tuner")
            self.declare_parameter("port", 8765)
            self.declare_parameter("host", "127.0.0.1")
            self.declare_parameter("manager_node", "flight_cycle_manager")
            target = str(self.get_parameter("manager_node").value).rstrip("/")
            self.telemetry = TelemetryStore()
            self._works = queue.Queue(maxsize=16)
            self._active = None
            self._client = self.create_client(
                SetParametersAtomically, target + "/set_parameters_atomically")
            self.actions = FlightActionBridge(self.telemetry, {
                action: self.create_client(Trigger, target + "/" + action)
                for action in FLIGHT_ACTIONS
            }, Trigger.Request)
            self.create_subscription(String, target + "/pid_debug",
                                     lambda message: self.telemetry.update(message.data), 10)
            self.create_timer(0.05, self._process_work,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.create_timer(0.05, self.actions.process,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))

        def submit(self, parameters):
            work = _GainWork(parameters)
            try:
                self._works.put_nowait(work)
            except queue.Full:
                return {"ok": False, "message": "tuner busy; retry"}
            if not work.done.wait(3.0):
                return {"ok": False, "message": "update timed out; check live parameters before retrying"}
            return work.result

        def _process_work(self):
            if self._active is not None:
                work, future = self._active
                if not future.done() and time.monotonic() < work.deadline:
                    return
                try:
                    if not future.done():
                        future.cancel()
                        raise TimeoutError("parameter service timed out; check live parameters")
                    outcome = future.result().result
                    work.result = {"ok": outcome.successful,
                                   "message": "parameters applied" if outcome.successful else outcome.reason}
                except Exception as error:  # ROS transport failure must not stop the HTTP server.
                    work.result = {"ok": False, "message": str(error)}
                work.done.set()
                self._active = None
            try:
                work = self._works.get_nowait()
            except queue.Empty:
                return
            if time.monotonic() >= work.deadline or not self._client.service_is_ready():
                work.result = {"ok": False, "message": "flight_cycle_manager unavailable or request expired"}
                work.done.set()
                return
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter(name, value=value).to_parameter_msg()
                                  for name, value in work.parameters.items()]
            self._active = work, self._client.call_async(request)

    rclpy.init(args=args)
    node = server = thread = None
    try:
        node = PidTunerNode()
        port = node.get_parameter("port").value
        if not 1 <= port <= 65535:
            raise ValueError("port must be in [1, 65535]")
        host = node.get_parameter("host").value
        server = ThreadingHTTPServer((host, port), _handler_for(
            node.telemetry, node.submit, node.actions.submit))
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        node.get_logger().info(f"Pose PID tuner: http://{host}:{server.server_port}")
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if server is not None:
            if thread is not None:
                server.shutdown()
                thread.join(timeout=2)
            server.server_close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


DASHBOARD_HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Dock pose PID tuner</title><style>
body{font:15px system-ui;background:#111827;color:#e5e7eb;margin:0;padding:20px}
main{max-width:1200px;margin:auto}.card{background:#1f2937;padding:18px;border-radius:10px;margin:14px 0}
.row{display:flex;gap:12px;flex-wrap:wrap;align-items:end}.plots{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.axis-controls{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px;margin:14px 0}
.axis-panel{min-width:0;border:1px solid #475569;border-radius:8px;padding:14px}.axis-panel h3{margin-top:0}
.axis-panel input{width:90px}.axis-panel label{min-width:0}.axis-panel p{overflow-wrap:anywhere}
label{display:grid;gap:6px}input,select,button{font:inherit;padding:8px;background:#111827;color:white;border:1px solid #64748b;border-radius:5px}
input{width:110px}button{cursor:pointer}button:disabled{opacity:.5;cursor:default}canvas{width:100%;height:auto;background:#0b1020}
small,p{color:#cbd5e1}strong{color:#60a5fa}.bad{color:#fca5a5}table{width:100%;text-align:left;border-collapse:collapse}td,th{padding:8px}
@media(max-width:750px){.plots,.axis-controls{grid-template-columns:1fr}}
</style></head><body><main>
<h1>Dock pose PID tuner</h1><p id="connection" role="status">Waiting for flight_cycle_manager</p>
<section class="card"><h2>Flight actions</h2>
<p class="bad">WARNING: these controls initiate real vehicle movement. Keep the flight area clear and the RC override available.</p>
<div class="row"><button id="action-start_cycle" disabled>Start cycle — arm &amp; take off</button>
<button id="action-request_dock" disabled>Request dock — search &amp; land</button></div>
<p id="action-availability-start_cycle">Waiting for manager prerequisites</p>
<p id="action-availability-request_dock">Waiting for manager prerequisites</p>
<p id="action-status" role="status">No flight request sent</p>
<button id="action-acknowledge" hidden>Acknowledge: I checked the vehicle and manager state</button>
<small>Start cycle takes off and hovers; docking requires a separate request. Request dock can acquire Offboard from manual flight.
Each action requires confirmation. Acceptance means queued/started, NOT completed. No automatic retries.
These are not emergency-stop controls; use the configured pilot override for intervention.</small></section>
<section class="card"><h2>Live gains</h2>
<div class="axis-controls" id="axis-controls"></div>
<p><button id="download" disabled>Download YAML</button></p>
<small>Changes apply live, with the controller's existing gain ramp. Near/far banks interpolate by dock distance;
-1 inherits the base gain. Base changes do not override explicit near/far values.</small></section>
<section class="card"><h2>DESCEND_CENTRAL → TOUCHDOWN_APPROACH</h2>
<p id="transition-summary" role="status">Waiting for transition diagnostics</p>
<p id="transition-reset">No dwell interruption recorded</p>
<table><thead><tr><th>Gate</th><th>Measured / elapsed</th><th>Required</th><th>Status</th></tr></thead>
<tbody id="transition-checks"></tbody></table>
<small>X/Y/yaw use the active control reference, which can differ from camera pose. Speeds are measured, not commanded.
X and Y each must be within tolerance. A pose or horizontal-speed excursion, or loss of central vision, resets BOTH dwell timers.
Height and vertical speed are separate gates and do not reset the fine-alignment dwell.</small>
<h3>Live transition parameters</h3><div class="row" id="transition-controls"></div>
<p><button id="apply-transition" disabled>Apply transition parameters</button>
<button id="reset-transition" disabled>Reload live values</button></p>
<p id="transition-status" role="status"></p>
<small>WARNING: these settings change landing acceptance criteria on real hardware. Apply only deliberate changes.
All edits apply atomically, with existing configuration constraints; both dwell timers restart.
Changes after touchdown commitment are rejected. Yaw inputs are radians (diagnostics also show degrees).
Commit height is camera-to-dock distance, not absolute altitude. Touchdown speed also sets the committed descent speed.
Alignment dwell is shared with earlier alignment stages; central tolerances also control descent pausing.
Use Download YAML to persist these values; live changes alone are not saved.</small></section>
<section class="card"><h2>Dock-relative pose tracking</h2>
<p id="pose-status">No fresh dock pose</p><table><thead><tr><th>Axis</th><th>Measured pose</th><th>Target</th><th>Tracking error</th></tr></thead><tbody id="readings"></tbody></table>
<small>X forward · Y left · Z measured dock distance, not absolute altitude · yaw counter-clockwise.
Errors are the active controller's errors (target minus measurement in the control convention), which may use a frozen reference.
Z has no error when distance control is inactive; missing values are gaps, not zero.</small></section>
<div class="row"><label>Position error range ± m<input id="position-scale" type="number" min="0.01" max="1000" step="any" value="2"></label>
<label>Yaw error range ± °<input id="yaw-scale" type="number" min="0.01" max="1000" step="any" value="180"></label>
<button id="clear">Clear history</button><span id="scale-status" role="status"></span></div>
<p>Last 60 seconds of pose tracking error. Fixed scales; out-of-range values are clipped and flagged.</p>
<div class="plots" id="plots"></div></main><script>
const axes=['x','y','z','yaw'], gains=['kp','ki','kd'];
const $=id=>document.getElementById(id), finite=v=>typeof v==='number'&&Number.isFinite(v);
const display=(v,a)=>finite(v)?(a==='yaw'?v*180/Math.PI:v).toFixed(3)+(a==='yaw'?' °':' m'):'—';
let latest=null, lastTime=null, samples=[];
const flightActions={start_cycle:'Start cycle — arm & take off',request_dock:'Request dock — search & land'};
const flightRequest={pending:false,uncertain:false};
function renderFlightActions(){
  Object.entries(flightActions).forEach(([action,label])=>{
    const info=latest?.flight_actions?.[action];
    $(`action-${action}`).disabled=!latest||info?.allowed!==true||flightRequest.pending||flightRequest.uncertain;
    $(`action-availability-${action}`).textContent=label+': '+(!latest?'Disconnected / stale telemetry':
      info?.allowed===true?'Manager prerequisites satisfied':info?.reason||'Action eligibility unavailable');
  });
  $('action-acknowledge').hidden=!flightRequest.uncertain;
}
async function requestFlightAction(action){
  if(!Object.hasOwn(flightActions,action)||$(`action-${action}`).disabled)return;
  const warning=action==='start_cycle'?
    'ARM AND TAKE OFF? The vehicle will acquire Offboard, arm, take off and hover. Confirm the area is clear and RC override is ready.':
    'START DOCKING? The airborne vehicle may switch to Offboard, search for the dock and land. Confirm the area is clear and RC override is ready.';
  if(!window.confirm(warning))return;
  flightRequest.pending=true;renderFlightActions();
  $('action-status').textContent='Sending '+flightActions[action]+'; do not repeat the request.';
  try{
    const response=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({action,confirmed:true}),signal:AbortSignal.timeout(5000)});
    const result=await response.json();
    flightRequest.uncertain=result.uncertain===true;
    $('action-status').textContent=(result.uncertain?'OUTCOME UNKNOWN: ':response.ok&&result.ok?'ACCEPTED (not completed): ':'NOT ACCEPTED: ')+result.message;
  }catch(error){
    flightRequest.uncertain=true;
    $('action-status').textContent='OUTCOME UNKNOWN: '+error.message+'. Inspect vehicle and manager state before any further request; do not retry automatically.';
  }finally{flightRequest.pending=false;renderFlightActions()}
}
Object.keys(flightActions).forEach(action=>$(`action-${action}`).onclick=()=>requestFlightAction(action));
$('action-acknowledge').onclick=()=>{flightRequest.uncertain=false;renderFlightActions()};
const editors=Object.fromEntries(axes.map(a=>[a,{dirty:false,applying:false}]));
const transitionFields=[
  ['touchdown_commit_height_m','Approach height (m)'],
  ['approach_height_tolerance_m','Height arrival tolerance (m)'],
  ['touchdown_speed_m_s','Vertical speed limit / touchdown descent (m/s)'],
  ['touchdown_horizontal_tolerance_m','Fine X/Y tolerance (m)'],
  ['touchdown_yaw_tolerance_rad','Fine yaw tolerance (rad)'],
  ['alignment_stable_time_s','Alignment dwell (s)'],
  ['touchdown_horizontal_speed_tolerance_m_s','Horizontal speed limit (m/s)'],
  ['touchdown_horizontal_speed_stable_time_s','Low-speed dwell (s)'],
  ['central_horizontal_tolerance_m','Central descent X/Y tolerance (m)'],
  ['central_yaw_tolerance_rad','Central descent yaw tolerance (rad)'],
];
const transitionEditor={dirty:new Set(),applying:false};
transitionFields.forEach(([field,label])=>{
  const control=document.createElement('label');
  control.innerHTML=`${label}<input id="transition-${field}" type="number" min="0" step="any" title="pid_${field}">`;
  $('transition-controls').appendChild(control);
  $(`transition-${field}`).oninput=()=>transitionEditor.dirty.add(field);
});
let positionScale=2,yawScale=180;
axes.forEach(a=>{
  const editor=document.createElement('section');editor.className='axis-panel';
  editor.setAttribute('aria-labelledby',`heading-${a}`);
  editor.innerHTML=`<h3 id="heading-${a}">${a.toUpperCase()} gains</h3><div class="row">
    <label>Gain bank<select id="bank-${a}"><option value="base">Base</option><option value="near">Near</option><option value="far">Far</option></select></label>
    ${gains.map(g=>`<label>${g==='kp'?'Kp':g==='ki'?'Ki':'Kd'}<input id="${g}-${a}" type="number" min="0" step="any"></label>`).join('')}
    <button id="apply-${a}" disabled>Apply ${a.toUpperCase()} gains</button></div>
    <p id="gain-status-${a}" role="status"></p><p id="effective-${a}">Effective gains: —</p>`;
  $('axis-controls').appendChild(editor);
  gains.forEach(g=>$(`${g}-${a}`).oninput=()=>{editors[a].dirty=true});
  $(`bank-${a}`).onchange=()=>{editors[a].dirty=false;$(`gain-status-${a}`).textContent='';fill();render()};
  $(`apply-${a}`).onclick=()=>applyGains(a);
  const row=document.createElement('tr');row.innerHTML=`<th>${a.toUpperCase()}</th><td id="measured-${a}">—</td><td id="target-${a}">—</td><td id="error-${a}">—</td>`;
  $('readings').appendChild(row);
  const card=document.createElement('section');card.className='card';
  card.innerHTML=`<h3>${a.toUpperCase()} tracking error (${a==='yaw'?'°':'m'})</h3><canvas id="plot-${a}" width="560" height="240" role="img" aria-label="${a} pose tracking error history"></canvas>`;
  $('plots').appendChild(card);
});
function prefix(a){return $(`bank-${a}`).value==='base'?`pid_${a}`:`pid_gain_${$(`bank-${a}`).value}_${a}`}
function fill(){
  axes.forEach(a=>{
    gains.forEach(g=>{$(`${g}-${a}`).min=$(`bank-${a}`).value==='base'?0:-1});
    if(!latest?.parameters||editors[a].dirty||editors[a].applying)return;
    gains.forEach(g=>{$(`${g}-${a}`).value=latest.parameters[`${prefix(a)}_${g}`]??''});
  });
  transitionFields.forEach(([field])=>{
    if(!latest?.parameters||transitionEditor.applying||transitionEditor.dirty.has(field))return;
    $(`transition-${field}`).value=latest.parameters[`pid_${field}`]??'';
  });
}
function renderTransition(){
  const d=latest?.touchdown_transition;
  const committed=['TOUCHDOWN_APPROACH','DISARM','COMPLETE','FAILED'].includes(latest?.state);
  const available=!!latest?.parameters&&transitionFields.every(([f])=>finite(latest.parameters[`pid_${f}`]));
  $('apply-transition').disabled=!available||committed||transitionEditor.applying;
  $('reset-transition').disabled=!available||transitionEditor.applying;
  transitionFields.forEach(([field])=>$(`transition-${field}`).disabled=!available||committed||transitionEditor.applying);
  $('transition-checks').innerHTML='';
  if(!d){$('transition-summary').textContent=latest?'Transition diagnostics inactive in '+latest.state:'Waiting for fresh manager telemetry';$('transition-reset').textContent='No live dwell diagnostics';return}
  const checks=d.checks??{};
  const blockers=(d.blockers??[]).map(key=>checks[key]?.label??key);
  $('transition-summary').textContent=`fine_alignment_stable: ${d.fine_alignment_stable} · height_ready: ${d.height_ready} · `+
    (d.ready?'READY':`BLOCKED: ${blockers.join(', ')||'gate evaluation pending'}`)+
    (d.evaluated?'':' · not evaluated this cycle')+
    (finite(d.distance_m)?` · distance ${d.distance_m.toFixed(3)} m / approach ${d.approach_height_m.toFixed(3)} m`:'')+
    (d.reason?' · '+d.reason:'');
  $('transition-summary').className=d.ready?'':'bad';
  const fmt=(value,unit)=>finite(value)?value.toFixed(6)+' '+unit+(unit==='rad'?` (${(value*180/Math.PI).toFixed(3)} °)`:''):'unavailable';
  const interruption=d.last_dwell_interruption;
  $('transition-reset').textContent=interruption?'Last dwell interruption: '+Object.values(interruption.checks).map(c=>
    c.label+(c.unit?` ${fmt(c.value,c.unit)}; required <= ${fmt(c.limit,c.unit)}`:': '+c.detail)).join('; ')+
    (finite(latest.time_s)?` · ${Math.max(0,latest.time_s-interruption.time_s).toFixed(2)} s ago`:''):'No dwell interruption recorded';
  Object.values(checks).forEach(check=>{
    const row=document.createElement('tr');
    const values=[check.label,check.unit?fmt(check.value,check.unit):check.detail??'—',
      check.unit?`${check.comparison} ${fmt(check.limit,check.unit)} (${check.parameter})`:'required',
      check.passed?'PASS':'BLOCKED'];
    values.forEach(text=>{const cell=document.createElement('td');cell.textContent=text;row.appendChild(cell)});
    row.className=check.passed?'':'bad';$('transition-checks').appendChild(row);
  });
}
function render(){
  renderFlightActions();
  const p=latest?.measured_pose,e=latest?.error;
  axes.forEach(a=>{
    $(`measured-${a}`).textContent=display(p?.[a],a);
    $(`error-${a}`).textContent=display(e?.[a],a);
    // Z setpoint is a distance. X/Y/yaw errors may refer to a frozen pose.
    $(`target-${a}`).textContent=a==='z'?display(latest?.distance_setpoint_m,a):'0 (alignment)';
  });
  $('pose-status').textContent=latest?.pose_valid?'Fresh dock pose · receipt age '+latest.dock_age_s.toFixed(2)+' s':'No fresh dock pose; measured pose unavailable';
  axes.forEach(a=>{
    const d=latest?.axes?.[a];
    $(`effective-${a}`).textContent=d?`Effective ${a.toUpperCase()}: Kp=${d.kp.toFixed(4)} Ki=${d.ki.toFixed(4)} Kd=${d.kd.toFixed(4)} · near ${latest.gain_near_distance_m} m / far ${latest.gain_far_distance_m} m`:'Effective gains: waiting for docking';
    $(`apply-${a}`).disabled=!latest?.parameters||editors[a].applying;
    $(`bank-${a}`).disabled=editors[a].applying;
    gains.forEach(g=>{$(`${g}-${a}`).disabled=editors[a].applying});
  });
  $('download').disabled=!latest?.parameters;
  renderTransition();
  draw();
}
function draw(){
  axes.forEach(a=>{
    const c=$(`plot-${a}`),g=c.getContext('2d'),limit=a==='yaw'?yawScale:positionScale;
    const left=65,right=c.width-18,top=28,bottom=c.height-30;
    const y=v=>bottom-(v/limit+1)*(bottom-top)/2;
    g.clearRect(0,0,c.width,c.height);g.font='12px system-ui';
    [-1,-.5,0,.5,1].forEach(f=>{
      g.strokeStyle=f===0?'#94a3b8':'#334155';g.beginPath();g.moveTo(left,y(f*limit));g.lineTo(right,y(f*limit));g.stroke();
      g.fillStyle='#cbd5e1';g.fillText((f*limit).toFixed(2),5,y(f*limit)+4);
    });
    g.fillStyle='#cbd5e1';g.fillText('-60 s',left,c.height-8);g.fillText('now',right-25,c.height-8);
    const end=lastTime??0;let clipped=false,pen=false;
    g.save();g.beginPath();g.rect(left,top,right-left,bottom-top);g.clip();g.strokeStyle='#60a5fa';g.lineWidth=2;g.beginPath();
    samples.forEach(s=>{
      let value=s.error?.[a];
      if(!finite(value)){pen=false;return}
      if(a==='yaw')value*=180/Math.PI;
      clipped=clipped||Math.abs(value)>limit;
      const x=right+(s.time-end)/60*(right-left),py=y(value);
      if(pen)g.lineTo(x,py);else{g.moveTo(x,py);g.fillStyle='#60a5fa';g.fillRect(x-2,py-2,4,4)}pen=true;
    });g.stroke();g.restore();
    if(clipped){g.fillStyle='#fca5a5';g.fillText('Outside displayed range (clipped)',left,18)}
  });
}
['position','yaw'].forEach(kind=>$(kind+'-scale').onchange=()=>{
  const input=$(kind+'-scale'),v=Number(input.value);
  if(!input.value.trim()||!finite(v)||v<.01||v>1000){$('scale-status').textContent='Range must be 0.01–1000';return}
  if(kind==='yaw')yawScale=v;else positionScale=v;
  $('scale-status').textContent='';draw();
});
$('clear').onclick=()=>{samples=[];draw()};
async function applyGains(a){
  if(!latest?.parameters||editors[a].applying)return;
  const payload={axis:a,bank:$(`bank-${a}`).value};
  for(const gain of gains){
    const raw=$(`${gain}-${a}`).value.trim(),value=Number(raw);
    if(!raw||!finite(value)||(value<0&&!(payload.bank!=='base'&&value===-1))){$(`gain-status-${a}`).textContent='Use nonnegative gains, or -1 to inherit in near/far';return}
    payload[gain]=value;
  }
  editors[a].applying=true;render();
  try{
    const response=await fetch('/api/gains',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
    const result=await response.json();$(`gain-status-${a}`).textContent=result.message;
    if(response.ok&&result.ok)editors[a].dirty=false;
  }catch(error){$(`gain-status-${a}`).textContent='Update failed: '+error.message}
  finally{editors[a].applying=false;render()}
}
$('reset-transition').onclick=()=>{transitionEditor.dirty.clear();$('transition-status').textContent='';fill()};
$('apply-transition').onclick=async()=>{
  if($('apply-transition').disabled||transitionEditor.applying)return;
  const parameters={};
  for(const field of transitionEditor.dirty){
    const raw=$(`transition-${field}`).value.trim(),value=Number(raw);
    if(!raw||!finite(value)||value<=0){$('transition-status').textContent='Use finite positive values';return}
    parameters[`pid_${field}`]=value;
  }
  if(!Object.keys(parameters).length){$('transition-status').textContent='No edited transition parameters';return}
  transitionEditor.applying=true;render();
  try{
    const response=await fetch('/api/transition',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({parameters})});
    const result=await response.json();$('transition-status').textContent=result.message;
    if(response.ok&&result.ok)transitionEditor.dirty.clear();
  }catch(error){$('transition-status').textContent='Update failed: '+error.message}
  finally{transitionEditor.applying=false;render()}
};
$('download').onclick=()=>{
  if(!latest?.parameters)return;
  let text='flight_cycle_manager:\n  ros__parameters:\n';
  Object.entries(latest.parameters).forEach(([name,value])=>text+=`    ${name}: ${Number.isInteger(value)?value.toFixed(1):value}\n`);
  const url=URL.createObjectURL(new Blob([text],{type:'text/yaml'})),a=document.createElement('a');
  a.href=url;a.download='pid_tuned.yaml';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
};
async function poll(){
  try{
    const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(1500)});
    if(!response.ok)throw new Error('HTTP '+response.status);
    const state=await response.json();latest=state.connected?state.telemetry:null;
    $('connection').textContent=latest?`Manager: ${latest.cycle_state} · PID: ${latest.state}`:'Disconnected / stale telemetry';
    $('connection').className=latest?'':'bad';
    if(latest&&finite(latest.time_s)&&latest.time_s!==lastTime){
      if(lastTime!==null&&latest.time_s<lastTime)samples=[];
      if(lastTime!==null&&latest.time_s-lastTime>1)samples.push({time:latest.time_s,error:null});
      lastTime=latest.time_s;samples.push({time:lastTime,error:latest.error});
      samples=samples.filter(s=>s.time>=lastTime-60).slice(-1200);
    }
    fill();render();
  }catch(error){
    latest=null;$('connection').textContent='Disconnected: '+error.message;$('connection').className='bad';render();
  }finally{setTimeout(poll,200)}
}
render();poll();
</script></body></html>"""


if __name__ == "__main__":
    main()