from __future__ import annotations

import hmac
import json
import os
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


HTML = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Sparky Tour Control</title>
<style>
:root{font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;color-scheme:dark}
body{margin:0;background:#101214;color:#f4f6f8}.wrap{max-width:900px;margin:auto;padding:18px}
.card{background:#191d21;border:1px solid #30363d;border-radius:16px;padding:16px;margin:12px 0;box-shadow:0 8px 28px #0005}
h1{font-size:24px;margin:0 0 8px}h2{font-size:18px;margin:0 0 8px}.muted{color:#aeb6bf;font-size:13px}.row{display:flex;gap:10px;flex-wrap:wrap}
input,textarea,select,button{font:inherit;border-radius:12px;border:1px solid #3a424a;background:#111519;color:#fff;padding:12px}
input,textarea,select{box-sizing:border-box;width:100%}textarea{min-height:92px;resize:vertical}select{min-height:46px}
button{cursor:pointer;min-height:46px;flex:1}button.primary{background:#e8edf2;color:#111;border-color:#e8edf2;font-weight:700}
button.danger{background:#642222;border-color:#a83a3a}button.safe{background:#153c2d;border-color:#2d775b}.admin{border-color:#725f24;background:#211e14}
pre{white-space:pre-wrap;word-break:break-word;background:#0d1013;padding:12px;border-radius:12px;max-height:320px;overflow:auto}.badge{display:inline-block;padding:4px 8px;border-radius:999px;background:#2a3036;font-size:12px;margin-right:5px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}@media(max-width:650px){.grid2{grid-template-columns:1fr}}
</style>
</head>
<body><div class="wrap">
<h1>Sparky Tour Control</h1>
<div class="muted">The phone performs speech recognition. Only text is sent to the DGX. Motion still passes through Sparky's safety/confirmation gate.</div>
<div class="card">
<label>Demo PIN / API token</label><input id="token" type="password" inputmode="numeric" autocomplete="off" placeholder="0000">
<div class="muted" style="margin-top:7px">For the supervised demo you can use SPARKY_WEB_TOKEN=0000. The same value is used as the Admin PIN by default.</div>
</div>
<div class="card">
<label>Phone speech recognition</label>
<div class="row" style="margin-top:10px"><button class="primary" onclick="startSpeech('command')">🎤 Speak command</button><button onclick="startSpeech('query')">🎤 Ask question</button><button onclick="stopSpeech()">Stop listening</button></div>
<div class="muted" id="speechStatus" style="margin-top:10px">Tap a microphone button and allow microphone access.</div>
<pre id="speechTranscript" style="min-height:44px;max-height:120px">No transcript yet.</pre>
<label style="display:flex;gap:8px;align-items:center"><input id="autoSendSpeech" type="checkbox" checked style="width:auto"> Automatically send final transcript</label>
</div>
<div class="card">
<label>Command</label><textarea id="command" placeholder="Welcome guests / Start the tour / Find the chair / Stop"></textarea>
<div class="row"><button class="primary" onclick="sendCommand()">Send command</button><button class="safe" onclick="command('yes')">Confirm / Yes</button><button onclick="command('no')">Reject / No</button><button class="danger" onclick="command('stop')">STOP</button></div>
</div>
<div class="card">
<label>Guest Tour Host</label>
<div class="row"><button class="primary" onclick="quick('welcome guests')">Welcome Guests</button><button onclick="quick('introduce the lab')">Lab Intro</button><button onclick="quick('introduce yourself')">Introduce Sparky</button></div>
<div class="row" style="margin-top:10px"><button onclick="quick('tell us your capabilities')">Capabilities</button><button class="safe" onclick="quick('show some moves')">Show Safe Moves</button><button onclick="quick('start the tour')">Start Saved Tour</button></div>
<div class="row" style="margin-top:10px"><button onclick="quick('wave')">Wave</button><button onclick="quick('stretch')">Stretch</button><button onclick="quick('sit')">Sit</button><button onclick="quick('stand')">Stand</button><button onclick="quick('heart')">Heart</button></div>
</div>
<div class="card">
<label>Ask Sparky (read-only)</label><textarea id="query" placeholder="What do you see? / What do you remember? / How many chairs do you remember?"></textarea>
<button class="primary" onclick="sendQuery()">Ask</button><div id="result" class="muted" style="margin-top:10px"></div>
</div>

<div class="card admin">
<h2>Admin — Tour Builder</h2>
<div class="muted">Admin operations require both the API token and Admin PIN. Save Stop captures the robot's current map pose and writes places.yaml + route.yaml + unified memory.</div>
<div class="row" style="margin-top:10px"><button class="primary" onclick="refreshAdmin()">Refresh Session Memory</button><button onclick="writeCheckpoint()">Write Memory Checkpoint Now</button></div>
<pre id="adminSummary">Admin data not loaded.</pre>
<div class="grid2">
<div><label>Stop name</label><input id="stopName" placeholder="robot_demo_area"></div>
<div><label>Room</label><input id="stopRoom" placeholder="digital_twin_lab"></div>
</div>
<label>Speech / narration at this stop</label><div class="row"><button onclick="startSpeech('stop_script')">🎤 Dictate narration</button></div><textarea id="stopScript" placeholder="Here we demonstrate ..."></textarea>
<div class="grid2"><div><label>Fact / short description</label><input id="stopFact" placeholder="Short factual description"></div><div><label>Pause seconds</label><input id="stopPause" type="number" step="0.5" value="4"></div></div>
<div class="row" style="margin-top:10px"><button class="primary" onclick="saveStop()">Save Current Pose as Tour Stop</button><button onclick="updateStop()">Update Selected Stop Speech</button><button class="danger" onclick="deleteStop()">Delete Selected Stop</button></div>
<label style="margin-top:12px;display:block">Existing stops</label><select id="stopSelect" onchange="loadSelectedStop()"><option value="">No stops loaded</option></select>
<label style="margin-top:12px;display:block">Stop order — one stop name per line</label><textarea id="stopOrder" placeholder="entrance\nrobot_demo_area\nexit"></textarea><button onclick="saveOrder()">Save Stop Order</button>
<label style="margin-top:12px;display:block">tour_host.yaml — editable welcome/intro scripts</label><textarea id="tourYaml" style="min-height:280px" placeholder="scripts:\n  welcome:\n    ..."></textarea><button onclick="saveTourYaml()">Validate + Save tour_host.yaml</button>
<div id="adminResult" class="muted" style="margin-top:10px"></div>
</div>

<div class="card"><div class="row"><span class="badge" id="tourBadge">tour: ?</span><span class="badge" id="voiceBadge">voice: ?</span><span class="badge" id="navBadge">nav: ?</span></div><pre id="state">Connecting...</pre></div>
</div>
<script>
const tokenEl=document.getElementById('token'); tokenEl.value=localStorage.getItem('sparkyToken')||'';
tokenEl.addEventListener('change',()=>localStorage.setItem('sparkyToken',tokenEl.value));
let adminCache=null;
async function api(path,method='GET',body=null,admin=false){
 const headers={'X-Sparky-Token':tokenEl.value}; if(admin)headers['X-Sparky-Admin-Pin']=tokenEl.value; if(body)headers['Content-Type']='application/json';
 const r=await fetch(path,{method,headers,body:body?JSON.stringify(body):null}); const t=await r.text(); let v; try{v=JSON.parse(t)}catch{v={text:t}}; if(!r.ok)throw new Error(v.error||t||r.statusText); return v;
}
async function command(text,confidence=1.0,source='phone_web'){try{const v=await api('/api/command','POST',{text,confidence,source});document.getElementById('result').textContent='Command accepted: '+(v.text||text)}catch(e){document.getElementById('result').textContent='ERROR: '+e.message}}
function sendCommand(){command(document.getElementById('command').value)} function quick(t){document.getElementById('command').value=t;command(t)}
async function sendQuery(confidence=1.0,source='phone_web_query'){const text=document.getElementById('query').value;try{const v=await api('/api/query','POST',{text,confidence,source});document.getElementById('result').textContent='Question sent: '+(v.text||text)}catch(e){document.getElementById('result').textContent='ERROR: '+e.message}}
let activeRecognition=null;
function speechCtor(){return window.SpeechRecognition||window.webkitSpeechRecognition||null} function setSpeechStatus(t){document.getElementById('speechStatus').textContent=t}
function startSpeech(mode){const Ctor=speechCtor();if(!Ctor){setSpeechStatus('Browser SpeechRecognition unavailable. Use the text box or phone keyboard microphone.');return}stopSpeech();const rec=new Ctor();activeRecognition=rec;rec.lang='en-US';rec.continuous=false;rec.interimResults=true;rec.maxAlternatives=1;rec.onstart=()=>setSpeechStatus('Listening on phone…');rec.onerror=e=>{setSpeechStatus('Speech error: '+e.error);activeRecognition=null};rec.onend=()=>{if(activeRecognition===rec){setSpeechStatus('Stopped listening.');activeRecognition=null}};rec.onresult=async e=>{let interim='',finalText='',confidence=1.0;for(let i=e.resultIndex;i<e.results.length;i++){const alt=e.results[i][0];if(e.results[i].isFinal){finalText+=alt.transcript;if(Number.isFinite(alt.confidence)&&alt.confidence>0)confidence=alt.confidence}else interim+=alt.transcript}const shown=(finalText||interim).trim();if(shown)document.getElementById('speechTranscript').textContent=shown;if(!finalText.trim())return;const final=finalText.trim();setSpeechStatus('Recognized: '+final);if(mode==='command')document.getElementById('command').value=final;else if(mode==='query')document.getElementById('query').value=final;else if(mode==='stop_script')document.getElementById('stopScript').value=final;if(document.getElementById('autoSendSpeech').checked){if(mode==='command')await command(final,confidence,'phone_chrome_speech');else if(mode==='query')await sendQuery(confidence,'phone_chrome_speech_query')}};try{rec.start()}catch(e){setSpeechStatus('Could not start microphone: '+e.message);activeRecognition=null}}
function stopSpeech(){if(activeRecognition){try{activeRecognition.stop()}catch(e){}activeRecognition=null}}
function short(v){if(v==null)return'?';if(typeof v==='string')return v.slice(0,60);return JSON.stringify(v).slice(0,60)}
async function refresh(){try{const s=await api('/api/state');document.getElementById('state').textContent=JSON.stringify(s,null,2);document.getElementById('tourBadge').textContent='tour: '+short(s.latest?.tour_status?.value);document.getElementById('voiceBadge').textContent='voice: '+short(s.latest?.voice_state?.value);document.getElementById('navBadge').textContent='nav: '+short(s.latest?.semantic_nav_status?.value)}catch(e){document.getElementById('state').textContent='Status error: '+e.message}}
function adminResult(t){document.getElementById('adminResult').textContent=t}
async function refreshAdmin(){try{const v=await api('/api/admin/session','GET',null,true);adminCache=v;document.getElementById('adminSummary').textContent=JSON.stringify(v.memory,null,2);const sel=document.getElementById('stopSelect');sel.innerHTML='<option value="">Select a stop</option>';for(const stop of (v.route?.stops||[])){const o=document.createElement('option');o.value=stop.name||stop.place_name;o.textContent=(stop.name||stop.place_name)+' → '+(stop.place_name||'');o.dataset.stop=JSON.stringify(stop);sel.appendChild(o)}document.getElementById('stopOrder').value=(v.route?.stops||[]).map(x=>x.name||x.place_name).join('\n');document.getElementById('tourYaml').value=v.tour_host_yaml||'';adminResult('Loaded session '+v.session_name)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
function loadSelectedStop(){const sel=document.getElementById('stopSelect');const opt=sel.options[sel.selectedIndex];if(!opt||!opt.dataset.stop)return;const s=JSON.parse(opt.dataset.stop);document.getElementById('stopName').value=s.name||s.place_name||'';document.getElementById('stopScript').value=s.script||'';document.getElementById('stopFact').value=s.fact||'';document.getElementById('stopPause').value=s.pause_seconds??4}
async function saveStop(){try{const body={name:document.getElementById('stopName').value,room:document.getElementById('stopRoom').value,script:document.getElementById('stopScript').value,fact:document.getElementById('stopFact').value,pause_seconds:Number(document.getElementById('stopPause').value||4),tags:['tour_stop']};const v=await api('/api/admin/save_stop','POST',body,true);adminResult(v.message||'Save requested');setTimeout(refreshAdmin,700)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
async function updateStop(){try{const name=document.getElementById('stopSelect').value||document.getElementById('stopName').value;const body={name,script:document.getElementById('stopScript').value,fact:document.getElementById('stopFact').value,pause_seconds:Number(document.getElementById('stopPause').value||4)};const v=await api('/api/admin/update_stop','POST',body,true);adminResult(v.message||'Update requested');setTimeout(refreshAdmin,700)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
async function deleteStop(){try{const name=document.getElementById('stopSelect').value||document.getElementById('stopName').value;if(!name)throw new Error('Select a stop first');if(!confirm('Delete Tour stop '+name+'?'))return;const v=await api('/api/admin/delete_stop','POST',{name},true);adminResult(v.message||'Delete requested');setTimeout(refreshAdmin,700)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
async function saveOrder(){try{const order=document.getElementById('stopOrder').value.split(/\n+/).map(x=>x.trim()).filter(Boolean);const v=await api('/api/admin/reorder_stops','POST',{order},true);adminResult(v.message||'Order requested');setTimeout(refreshAdmin,700)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
async function writeCheckpoint(){try{const v=await api('/api/admin/write_checkpoint','POST',{},true);adminResult(v.message||'Checkpoint requested');setTimeout(refreshAdmin,700)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
async function saveTourYaml(){try{const v=await api('/api/admin/tour_host_yaml','POST',{yaml:document.getElementById('tourYaml').value},true);adminResult(v.message||'YAML saved')}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
setInterval(refresh,1000);refresh();
</script></body></html>'''


class PhoneWebGatewayNode(Node):
    """Browser gateway for commands, read-only Q&A, and PIN-protected Tour administration."""

    def __init__(self) -> None:
        super().__init__("go2_phone_web_gateway")
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("port", 8765)
        self.declare_parameter("require_token", True)
        self.declare_parameter("api_token", os.environ.get("SPARKY_WEB_TOKEN", ""))
        self.declare_parameter("admin_pin", os.environ.get("SPARKY_ADMIN_PIN", "0000"))
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "auto")
        self.declare_parameter("wake_word", "Sparky")
        self.declare_parameter("auto_prefix_wake_word", True)
        self.declare_parameter("transcript_topic", "/go2_voice/transcript")
        self.declare_parameter("query_topic", "/go2_agent/query")
        self.declare_parameter("semantic_nav_command_topic", "/semantic_nav/command")
        self.declare_parameter("snapshot_topic", "/go2_memory/write_snapshot_now")

        self.transcript_pub = self.create_publisher(String, str(self.get_parameter("transcript_topic").value), 10)
        self.query_pub = self.create_publisher(String, str(self.get_parameter("query_topic").value), 10)
        self.semantic_command_pub = self.create_publisher(String, str(self.get_parameter("semantic_nav_command_topic").value), 10)
        self.snapshot_pub = self.create_publisher(String, str(self.get_parameter("snapshot_topic").value), 10)
        self._lock = threading.Lock()
        self._latest: dict[str, dict[str, Any]] = {}

        topics = {
            "tour_status": "/go2_tour/status", "tour_host_status": "/go2_tour/host_status", "tour_narration": "/go2_tour/narration",
            "voice_state": "/go2_voice/verification_state", "voice_request": "/go2_voice/verification_request",
            "semantic_nav_status": "/semantic_nav/status", "semantic_nav_event": "/semantic_nav/event",
            "agent_status": "/go2_agent/status", "agent_speech": "/go2_agent/speech", "agent_stream": "/go2_agent/stream",
            "tts_status": "/go2_tts/status", "omi_status": "/omi/status", "collision_monitor": "/collision_monitor_state",
            "object_inventory": "/go2_memory/object_inventory", "world_status": "/go2_memory/world_status",
        }
        for key, topic in topics.items():
            self.create_subscription(String, topic, lambda msg, k=key: self._capture(k, msg), 10)

        require_token = self._bool_param("require_token")
        token = str(self.get_parameter("api_token").value or "")
        if require_token and not token:
            raise RuntimeError("phone web gateway requires SPARKY_WEB_TOKEN (or api_token parameter) when require_token=true")

        host = str(self.get_parameter("bind_host").value)
        port = int(self.get_parameter("port").value)
        node = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "SparkyPhoneGateway/2.0"
            def log_message(self, fmt: str, *args: Any) -> None:
                node.get_logger().debug("web: " + (fmt % args))
            def _json(self, status: int, payload: Any) -> None:
                data = json.dumps(payload, sort_keys=True, default=str).encode("utf-8")
                self.send_response(status); self.send_header("Content-Type", "application/json; charset=utf-8"); self.send_header("Cache-Control", "no-store"); self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data)
            def _authorized(self) -> bool:
                if not node._bool_param("require_token"): return True
                expected=str(node.get_parameter("api_token").value or ""); provided=self.headers.get("X-Sparky-Token", "")
                return bool(expected) and hmac.compare_digest(expected, provided)
            def _admin_authorized(self) -> bool:
                expected=str(node.get_parameter("admin_pin").value or ""); provided=self.headers.get("X-Sparky-Admin-Pin", "")
                return bool(expected) and hmac.compare_digest(expected, provided)
            def _need_auth(self) -> bool:
                if self._authorized(): return False
                self._json(HTTPStatus.UNAUTHORIZED,{"error":"invalid or missing X-Sparky-Token"}); return True
            def _need_admin(self) -> bool:
                if self._need_auth(): return True
                if self._admin_authorized(): return False
                self._json(HTTPStatus.FORBIDDEN,{"error":"invalid or missing Admin PIN"}); return True
            def _read_json(self) -> dict[str, Any]:
                try:
                    n=min(int(self.headers.get("Content-Length","0") or 0),262144); raw=self.rfile.read(n).decode("utf-8",errors="replace"); value=json.loads(raw or "{}"); return value if isinstance(value,dict) else {}
                except Exception: return {}
            def do_GET(self) -> None:
                path=urlparse(self.path).path
                if path=="/":
                    data=HTML.encode("utf-8"); self.send_response(HTTPStatus.OK); self.send_header("Content-Type","text/html; charset=utf-8"); self.send_header("Cache-Control","no-store"); self.send_header("Content-Length",str(len(data))); self.end_headers(); self.wfile.write(data); return
                if path=="/api/health": self._json(HTTPStatus.OK,{"ok":True,"node":node.get_name()}); return
                if path=="/api/state":
                    if self._need_auth(): return
                    self._json(HTTPStatus.OK,node.state_snapshot()); return
                if path=="/api/admin/session":
                    if self._need_admin(): return
                    try: self._json(HTTPStatus.OK,node.admin_session_snapshot())
                    except Exception as exc: self._json(HTTPStatus.INTERNAL_SERVER_ERROR,{"error":str(exc)})
                    return
                self._json(HTTPStatus.NOT_FOUND,{"error":"not found"})
            def do_POST(self) -> None:
                path=urlparse(self.path).path
                if path.startswith("/api/admin/"):
                    if self._need_admin(): return
                    body=self._read_json()
                    try:
                        result=node.handle_admin_api(path,body); self._json(HTTPStatus.ACCEPTED if result.get("queued") else HTTPStatus.OK,result)
                    except ValueError as exc: self._json(HTTPStatus.BAD_REQUEST,{"error":str(exc)})
                    except Exception as exc: self._json(HTTPStatus.INTERNAL_SERVER_ERROR,{"error":str(exc)})
                    return
                if self._need_auth(): return
                body=self._read_json(); text=str(body.get("text") or body.get("command") or body.get("query") or "").strip()
                if not text: self._json(HTTPStatus.BAD_REQUEST,{"error":"missing text"}); return
                if len(text)>2000: self._json(HTTPStatus.BAD_REQUEST,{"error":"text too long"}); return
                try: confidence=max(0.0,min(1.0,float(body.get("confidence",1.0))))
                except Exception: confidence=1.0
                source=str(body.get("source") or "phone_web").strip()[:80]
                if path=="/api/command":
                    sent=node.publish_command(text,confidence=confidence,source=source); self._json(HTTPStatus.ACCEPTED,{"ok":True,"kind":"command","text":sent}); return
                if path=="/api/query":
                    node.publish_query(text,confidence=confidence,source=source); self._json(HTTPStatus.ACCEPTED,{"ok":True,"kind":"read_only_query","text":text}); return
                self._json(HTTPStatus.NOT_FOUND,{"error":"not found"})

        self._server=ThreadingHTTPServer((host,port),Handler); self._server.daemon_threads=True
        self._thread=threading.Thread(target=self._server.serve_forever,name="sparky-phone-web",daemon=True); self._thread.start()
        self.get_logger().info(f"Phone web gateway listening on http://{host}:{port}; token_required={require_token}; admin_enabled=true")

    def _bool_param(self,name:str)->bool:
        value=self.get_parameter(name).value; return value if isinstance(value,bool) else str(value).strip().lower() in {"1","true","yes","on"}
    def _capture(self,key:str,msg:String)->None:
        raw=msg.data
        try: value:Any=json.loads(raw)
        except Exception: value=raw
        with self._lock: self._latest[key]={"value":value,"received_unix":time.time()}
    def state_snapshot(self)->dict[str,Any]:
        with self._lock: latest=dict(self._latest)
        return {"ok":True,"time_unix":time.time(),"latest":latest}
    def publish_command(self,text:str,confidence:float=1.0,source:str="phone_web")->str:
        stripped=text.strip(); low=stripped.lower().strip(" .!?\t\r\n"); no_prefix=low in {"yes","confirm","go ahead","proceed","no","reject","cancel","stop","halt","freeze"}
        if self._bool_param("auto_prefix_wake_word") and not no_prefix:
            wake=str(self.get_parameter("wake_word").value or "Sparky").strip()
            # SPARKY_WAKE_PREFIX_NORMALIZATION_V4
            wake_low = wake.lower()
            already_prefixed = (
                low == wake_low
                or low.startswith(wake_low + " ")
                or low.startswith(wake_low + ",")
                or low.startswith(wake_low + ":")
            )
            if wake and not already_prefixed:
                stripped = f"{wake}, {stripped}"
        payload={"text":stripped,"confidence":max(0.0,min(1.0,float(confidence))),"source":source or "phone_web","input_kind":"command"}; self.transcript_pub.publish(String(data=json.dumps(payload,sort_keys=True))); return stripped
    def publish_query(self,text:str,confidence:float=1.0,source:str="phone_web_query")->None:
        payload={"text":text.strip(),"user_text":text.strip(),"confidence":max(0.0,min(1.0,float(confidence))),"source":source or "phone_web_query","read_only":True,"verified":True,"input_kind":"query"}; self.query_pub.publish(String(data=json.dumps(payload,sort_keys=True)))

    def _session_root(self)->Path:
        return Path(os.path.expanduser(str(self.get_parameter("session_root").value))).resolve()
    def _admin_session_dir(self)->Path:
        root=self._session_root(); requested=str(self.get_parameter("session_name").value or "").strip()
        if requested and requested not in {"auto","latest","__latest_created__"}:
            p=root/requested
            if not p.is_dir(): raise RuntimeError(f"session not found: {requested}")
            return p
        candidates=[p for p in root.iterdir() if p.is_dir()] if root.is_dir() else []
        usable=[p for p in candidates if (p/"map.yaml").is_file()]
        pool=usable or candidates
        if not pool: raise RuntimeError(f"no sessions found under {root}")
        return max(pool,key=lambda p:p.stat().st_mtime)
    @staticmethod
    def _read_yaml(path:Path)->dict[str,Any]:
        if not path.is_file(): return {}
        value=yaml.safe_load(path.read_text(encoding="utf-8")) or {}; return value if isinstance(value,dict) else {}
    @staticmethod
    def _jsonl_rows(path:Path)->list[dict[str,Any]]:
        rows=[]
        if not path.is_file(): return rows
        for line in path.read_text(encoding="utf-8",errors="replace").splitlines():
            try:
                v=json.loads(line)
                if isinstance(v,dict): rows.append(v)
            except Exception: pass
        return rows
    def admin_session_snapshot(self)->dict[str,Any]:
        session=self._admin_session_dir(); memory=session/"memory"
        places_payload=self._read_yaml(session/"places.yaml"); route_payload=self._read_yaml(session/"route.yaml"); route=route_payload.get("route") if isinstance(route_payload.get("route"),dict) else route_payload
        objects=self._jsonl_rows(memory/"objects.jsonl"); confirmed={}
        for i,row in enumerate(objects):
            data=row.get("data") if isinstance(row.get("data"),dict) else row
            if not bool(data.get("confirmed",True)) or not bool(data.get("countable",True)): continue
            oid=str(row.get("id") or data.get("object_id") or data.get("id") or i); confirmed[oid]=row
        def n(rel:str)->int: return len(self._jsonl_rows(session/rel))
        tour_yaml_path=session/"tour_host.yaml"; tour_yaml=tour_yaml_path.read_text(encoding="utf-8") if tour_yaml_path.is_file() else "scripts:\n  welcome:\n    - say: Welcome to the Digital Twin Lab.\n"
        with self._lock: live=dict(self._latest)
        return {
            "ok":True,"session_name":session.name,"session_dir":str(session),
            "memory":{
                "confirmed_objects":len(confirmed),"object_observations":n("memory/object_observations.jsonl"),"background_checkpoints":n("memory/checkpoints.jsonl"),"vlm_checkpoints":n("memory/vlm_checkpoints.jsonl"),"unified_places":n("memory/places.jsonl"),"unified_tour_stops":n("memory/tour_stops.jsonl"),"semantic_places":len(places_payload.get("places") or []),"route_stops":len((route or {}).get("stops") or []),"map_saved":(session/"map.yaml").is_file(),"live_object_inventory":(live.get("object_inventory") or {}).get("value"),
            },
            "places":places_payload.get("places") or [],"route":route or {},"tour_host_yaml":tour_yaml,
        }
    def _publish_semantic_admin(self,payload:dict[str,Any])->None:
        self.semantic_command_pub.publish(String(data=json.dumps(payload,sort_keys=True)))
    def handle_admin_api(self,path:str,body:dict[str,Any])->dict[str,Any]:
        if path=="/api/admin/save_stop":
            name=str(body.get("name") or "").strip()
            if not name: raise ValueError("stop name is required")
            payload={"type":"save_tour_stop","name":name,"room":str(body.get("room") or ""),"script":str(body.get("script") or ""),"fact":str(body.get("fact") or ""),"pause_seconds":float(body.get("pause_seconds",4.0) or 4.0),"tags":body.get("tags") or ["tour_stop"]}; self._publish_semantic_admin(payload); return {"ok":True,"queued":True,"message":f"Saving current robot pose as Tour stop '{name}'."}
        if path=="/api/admin/update_stop":
            name=str(body.get("name") or "").strip()
            if not name: raise ValueError("select a stop first")
            self._publish_semantic_admin({"type":"update_tour_stop","name":name,"script":str(body.get("script") or ""),"fact":str(body.get("fact") or ""),"pause_seconds":float(body.get("pause_seconds",4.0) or 4.0)}); return {"ok":True,"queued":True,"message":f"Updating Tour stop '{name}'."}
        if path=="/api/admin/delete_stop":
            name=str(body.get("name") or "").strip()
            if not name: raise ValueError("select a stop first")
            self._publish_semantic_admin({"type":"delete_tour_stop","name":name,"delete_place":bool(body.get("delete_place",False))}); return {"ok":True,"queued":True,"message":f"Deleting Tour stop '{name}'."}
        if path=="/api/admin/reorder_stops":
            order=body.get("order")
            if not isinstance(order,list): raise ValueError("order must be a list")
            self._publish_semantic_admin({"type":"reorder_tour_stops","order":[str(x).strip() for x in order if str(x).strip()]}); return {"ok":True,"queued":True,"message":"Tour stop order update queued."}
        if path=="/api/admin/write_checkpoint":
            self.snapshot_pub.publish(String(data=json.dumps({"source":"phone_admin","reason":"manual_admin_checkpoint","stamp_unix":time.time()}))); return {"ok":True,"queued":True,"message":"Memory checkpoint requested at current pose."}
        if path=="/api/admin/tour_host_yaml":
            raw=str(body.get("yaml") or "")
            if len(raw)>131072: raise ValueError("tour_host.yaml is too large")
            parsed=yaml.safe_load(raw) or {}
            if not isinstance(parsed,dict) or not isinstance(parsed.get("scripts"),dict): raise ValueError("YAML must contain a top-level 'scripts:' mapping")
            session=self._admin_session_dir(); target=session/"tour_host.yaml"; tmp=target.with_suffix(".yaml.tmp"); tmp.write_text(raw,encoding="utf-8"); os.replace(tmp,target); return {"ok":True,"queued":False,"message":f"Saved and validated {target}. Tour Host reloads it on the next script."}
        raise ValueError(f"unknown admin endpoint: {path}")
    def destroy_node(self)->bool:
        try: self._server.shutdown(); self._server.server_close()
        except Exception: pass
        return super().destroy_node()


def main(args=None)->None:
    rclpy.init(args=args); node=PhoneWebGatewayNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
