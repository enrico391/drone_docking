"""Local web controls for a running disturbance injector."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import queue
import signal
import threading


BOOL_NAMES = {"use_sim_time", "enabled", "turbulence.enabled", "gust.enabled", "ground_effect.enabled"}
VECTOR_NAMES = {
    "constant.force", "constant.torque", "turbulence.force_stddev",
    "turbulence.torque_stddev", "gust.force", "gust.torque", "limits.force", "limits.torque",
}
FLOAT_NAMES = {
    "update_rate", "turbulence.correlation_time", "gust.duration", "gust.interval",
    "ground_effect.surface_z", "ground_effect.rotor_plane_offset",
    "ground_effect.minimum_height", "ground_effect.maximum_height",
    "ground_effect.maximum_upward_force", "ground_effect.turbulence_gain",
}
PARAM_NAMES = (
    "use_sim_time", "enabled", "target_model", "update_rate", "frame", "random_seed",
    "constant.force", "constant.torque", "turbulence.enabled",
    "turbulence.force_stddev", "turbulence.torque_stddev", "turbulence.correlation_time",
    "gust.enabled", "gust.force", "gust.torque", "gust.duration", "gust.interval",
    "ground_effect.enabled", "ground_effect.surface_z", "ground_effect.rotor_plane_offset",
    "ground_effect.minimum_height", "ground_effect.maximum_height",
    "ground_effect.maximum_upward_force", "ground_effect.turbulence_gain",
    "limits.force", "limits.torque",
)


def parse_change(payload):
    if not isinstance(payload, dict) or payload.get("name") not in PARAM_NAMES:
        raise ValueError("unknown parameter")
    name, value = payload["name"], payload.get("value")
    if name in BOOL_NAMES:
        if not isinstance(value, bool):
            raise ValueError(f"{name} must be a boolean")
    elif name in VECTOR_NAMES:
        if not isinstance(value, list) or len(value) != 3 or any(
            isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
            for x in value
        ):
            raise ValueError(f"{name} must have three finite numbers")
        value = [float(x) for x in value]
    elif name == "random_seed":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("random_seed must be an integer")
    elif name in FLOAT_NAMES:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        value = float(value)
        if name == "update_rate" and value <= 0:
            raise ValueError("update_rate must be positive")
    elif name == "frame":
        if value not in ("body", "world"):
            raise ValueError("frame must be body or world")
    elif not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be nonempty text")
    return name, value


HTML = """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Disturbance tuner</title><style>
body{font:15px system-ui;background:#111827;color:#e5e7eb;margin:0;padding:24px}
main{max-width:850px;margin:auto}fieldset{border:1px solid #374151;border-radius:9px;margin:14px 0;padding:16px}
legend{font-weight:700}label{display:grid;grid-template-columns:minmax(180px,1fr) minmax(160px,2fr) auto;
gap:10px;align-items:center;margin:8px 0}input,select,button{font:inherit;padding:7px;border-radius:5px}
input,select{background:#1f2937;color:white;border:1px solid #4b5563;min-width:0}input[type=checkbox]{justify-self:start}
button{background:#2563eb;color:white;border:0;cursor:pointer}button:disabled{opacity:.5;cursor:default}
#toggle{font-weight:700;background:#047857}#toggle.on{background:#b91c1c}#status{margin-left:12px}
small{color:#9ca3af}#message{min-height:1.3em}
</style></head><body><main><h2>Disturbance tuner</h2>
<button id="toggle" disabled>Enable disturbances</button><span id="status">Waiting for injector</span>
<p id="message"></p><div id="fields"></div><button id="download" disabled>Download YAML</button>
<p><small>Changes apply immediately. Download YAML to keep them for the next launch.</small></p>
</main><script>
const groups=['General','Constant','Turbulence','Gust','Ground effect','Limits'];
const fieldsets={};let current=null, busy=false, editing=new Set();
for(const name of groups){let f=document.createElement('fieldset'),l=document.createElement('legend');
 l.textContent=name;f.appendChild(l);document.querySelector('#fields').appendChild(f);fieldsets[name]=f}
function group(n){return n.startsWith('constant.')?'Constant':n.startsWith('turbulence.')?'Turbulence':
 n.startsWith('gust.')?'Gust':n.startsWith('ground_effect.')?'Ground effect':n.startsWith('limits.')?'Limits':'General'}
function makeField(n,v){let row=document.createElement('label'),title=document.createElement('span'),input;
 title.textContent=n;row.appendChild(title);
 if(typeof v==='boolean'){input=document.createElement('input');input.type='checkbox'}
 else if(n==='frame'){input=document.createElement('select');for(let s of ['body','world']){
   let o=document.createElement('option');o.value=s;o.textContent=s;input.appendChild(o)}}
 else{input=document.createElement('input');input.type=Array.isArray(v)||typeof v==='string'?'text':'number';
   if(input.type==='number')input.step=n==='random_seed'?'1':'any'}
 input.id='p-'+n;input.oninput=()=>editing.add(n);input.onchange=()=>editing.add(n);row.appendChild(input);
 let button=document.createElement('button');button.textContent='Apply';button.type='button';button.onclick=()=>apply(n);
 row.appendChild(button);fieldsets[group(n)].appendChild(row);return input}
function valueOf(n){let input=document.getElementById('p-'+n),old=current[n];
 if(typeof old==='boolean')return input.checked;
 if(Array.isArray(old)){let parts=input.value.split(',').map(s=>s.trim());if(parts.length!==3||parts.some(s=>s===''))throw Error('Enter three comma-separated numbers');
  let values=parts.map(Number);if(values.some(x=>!Number.isFinite(x)))throw Error('Enter three finite numbers');return values}
 if(typeof old==='number'){if(input.value.trim()==='')throw Error('Enter a number');let x=Number(input.value);
  if(!Number.isFinite(x))throw Error('Enter a finite number');return x}
 return input.value}
async function send(n,v){let m=document.querySelector('#message');busy=true;
 try{let r=await fetch('/api/parameter',{method:'POST',headers:{'Content-Type':'application/json'},
 body:JSON.stringify({name:n,value:v})});let result=await r.json();m.textContent=result.message;
 if(result.ok){editing.delete(n);await poll()}return result.ok}
 catch(e){m.textContent=e.message;return false}finally{busy=false}}
async function apply(n){try{await send(n,valueOf(n))}catch(e){document.querySelector('#message').textContent=e.message}}
document.querySelector('#toggle').onclick=()=>send('enabled',!current.enabled);
function fill(params){current=params;for(let [n,v] of Object.entries(params)){
 let input=document.getElementById('p-'+n)||makeField(n,v);if(editing.has(n))continue;
 if(typeof v==='boolean')input.checked=v;else input.value=Array.isArray(v)?v.join(', '):v}
 let b=document.querySelector('#toggle');b.disabled=false;b.textContent=params.enabled?'Disable disturbances':'Enable disturbances';
 b.classList.toggle('on',params.enabled)}
async function poll(){try{let s=await(await fetch('/api/state')).json();
 document.querySelector('#status').textContent=s.connected?'Injector connected':'Waiting for injector';
 document.querySelector('#download').disabled=!s.connected;
 if(s.connected&&!busy)fill(s.parameters);if(!s.connected)document.querySelector('#toggle').disabled=true
 }catch(e){document.querySelector('#status').textContent='Dashboard disconnected'} }
document.querySelector('#download').onclick=()=>{if(!current)return;
 let y='/**:\\n  ros__parameters:\\n';for(let [n,v] of Object.entries(current)){
  y+='    '+n+': '+(typeof v==='string'?JSON.stringify(v):JSON.stringify(v))+'\\n'}
 let url=URL.createObjectURL(new Blob([y],{type:'text/yaml'})),a=document.createElement('a');
 a.href=url;a.download='disturbances_tuned.yaml';a.click();URL.revokeObjectURL(url)};
poll();setInterval(poll,500);
</script></body></html>"""


def _handler_for(node):
    class Handler(BaseHTTPRequestHandler):
        def _json(self, status, data):
            body = json.dumps(data).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/api/state":
                self._json(200, node.snapshot())
            elif self.path == "/":
                body = HTML.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path != "/api/parameter":
                self.send_error(404)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 4096:
                    raise ValueError("invalid request size")
                change = parse_change(json.loads(self.rfile.read(length)))
            except (ValueError, TypeError) as exc:
                self._json(400, {"ok": False, "message": str(exc)})
                return
            result = node.submit(change)
            self._json(200 if result["ok"] else 503, result)

        def log_message(self, *args):
            pass

    return Handler


def main(args=None):
    import rclpy
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
    from rclpy.node import Node
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rclpy.signals import SignalHandlerOptions

    class Tuner(Node):
        def __init__(self):
            super().__init__("disturbance_tuner")
            self.declare_parameter("port", 8766)
            self.declare_parameter("target_node", "/disturbance_injector")
            target = str(self.get_parameter("target_node").value).rstrip("/")
            self._get = self.create_client(GetParameters, target + "/get_parameters")
            self._set = self.create_client(SetParametersAtomically, target + "/set_parameters_atomically")
            self._lock = threading.Lock()
            self._latest = None
            self._work = queue.Queue()
            self._active = None
            self._reading = None
            self.create_timer(0.05, self._tick)

        def snapshot(self):
            with self._lock:
                params = self._latest
            return {"connected": params is not None and self._get.service_is_ready(),
                    "parameters": params}

        def submit(self, change):
            done = threading.Event()
            item = [change, done, None]
            self._work.put(item)
            if not done.wait(3.0):
                return {"ok": False, "message": "parameter update timed out"}
            return item[2]

        def _tick(self):
            if self._reading is not None and self._reading.done():
                try:
                    response = self._reading.result()
                    values = [parameter_value_to_python(v) for v in response.values]
                    with self._lock:
                        self._latest = dict(zip(PARAM_NAMES, values))
                except Exception:
                    with self._lock:
                        self._latest = None
                self._reading = None
            if self._reading is None and self._get.service_is_ready():
                request = GetParameters.Request()
                request.names = list(PARAM_NAMES)
                self._reading = self._get.call_async(request)
            if self._active is not None:
                item, future = self._active
                if not future.done():
                    return
                try:
                    outcome = future.result().result
                    item[2] = {"ok": bool(outcome.successful),
                               "message": "parameter applied" if outcome.successful else outcome.reason}
                except Exception as exc:
                    item[2] = {"ok": False, "message": str(exc)}
                item[1].set()
                self._active = None
            try:
                item = self._work.get_nowait()
            except queue.Empty:
                return
            if not self._set.service_is_ready():
                item[2] = {"ok": False, "message": "disturbance injector is unavailable"}
                item[1].set()
                return
            name, value = item[0]
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter(name, value=value).to_parameter_msg()]
            self._active = item, self._set.call_async(request)

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Tuner()
    port = int(node.get_parameter("port").value)
    server = ThreadingHTTPServer(("127.0.0.1", port), _handler_for(node))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f"Disturbance tuner: http://127.0.0.1:{port}")
    previous_handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}

    def request_shutdown(signum, frame):
        del signum, frame
        raise KeyboardInterrupt

    for signum in previous_handlers:
        signal.signal(signum, request_shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    main()
