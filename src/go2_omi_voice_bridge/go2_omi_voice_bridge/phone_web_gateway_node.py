from __future__ import annotations

import hmac
import json
import math
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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


HTML = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Sparky Tour Control</title>
<style>
:root{font-family:-apple-system,BlinkMacSystemFont,"SF Pro Display","SF Pro Text",system-ui,sans-serif;color-scheme:light dark;--bg:#f5f5f7;--glass:rgba(255,255,255,.72);--text:#1d1d1f;--muted:#6e6e73;--line:rgba(0,0,0,.09);--blue:#007aff;--green:#34c759;--orange:#ff9f0a;--red:#ff3b30;--shadow:0 18px 55px rgba(0,0,0,.08)}
@media(prefers-color-scheme:dark){:root{--bg:#000;--glass:rgba(28,28,30,.76);--text:#f5f5f7;--muted:#98989d;--line:rgba(255,255,255,.12);--shadow:0 20px 60px rgba(0,0,0,.45)}}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 15% 0%,rgba(0,122,255,.10),transparent 34%),var(--bg);color:var(--text);min-height:100vh}.wrap{max-width:1100px;margin:auto;padding:32px 20px 70px}h1{font-size:38px;letter-spacing:-1.2px;margin:0;font-weight:730}h2{font-size:20px;letter-spacing:-.35px;margin:0 0 12px}.muted{color:var(--muted);font-size:13px;line-height:1.45}.hero{display:flex;justify-content:space-between;align-items:flex-end;gap:16px;margin:8px 2px 24px}.eyebrow{font-size:12px;color:var(--blue);font-weight:700;text-transform:uppercase;letter-spacing:.08em}.card{background:var(--glass);backdrop-filter:blur(28px) saturate(150%);-webkit-backdrop-filter:blur(28px) saturate(150%);border:1px solid var(--line);border-radius:24px;padding:20px;margin:14px 0;box-shadow:var(--shadow)}.row{display:flex;gap:10px;flex-wrap:wrap}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:12px}.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}.telemetry{display:grid;grid-template-columns:1.1fr .9fr;gap:14px}label{display:block;font-size:13px;font-weight:650;margin:4px 0 8px}input,textarea,select,button{font:inherit;border-radius:14px;border:1px solid var(--line);padding:12px 14px;color:var(--text)}input,textarea,select{width:100%;background:rgba(127,127,127,.08);outline:none}input:focus,textarea:focus,select:focus{border-color:rgba(0,122,255,.55);box-shadow:0 0 0 4px rgba(0,122,255,.10)}textarea{min-height:92px;resize:vertical}button{cursor:pointer;min-height:44px;background:rgba(127,127,127,.10);font-weight:620;flex:1;transition:transform .12s ease,background .12s ease}button:active{transform:scale(.98)}button.primary{background:var(--blue);color:#fff;border-color:transparent}button.safe{background:rgba(52,199,89,.14);color:var(--green)}button.danger{background:var(--red);color:#fff;border-color:transparent}.admin{box-shadow:none}.badge,.status-pill{display:inline-flex;align-items:center;gap:6px;padding:7px 10px;border-radius:999px;background:rgba(127,127,127,.10);font-size:12px;font-weight:650}.status-pill:before{content:"";width:7px;height:7px;border-radius:50%;background:var(--muted)}.status-pill.good:before{background:var(--green)}.status-pill.warn:before{background:var(--orange)}.status-pill.bad:before{background:var(--red)}pre{white-space:pre-wrap;word-break:break-word;background:rgba(127,127,127,.07);padding:14px;border-radius:16px;max-height:330px;overflow:auto;border:1px solid var(--line);font:12px ui-monospace,SFMono-Regular,Menlo,monospace}.status-strip{display:flex;gap:8px;flex-wrap:wrap;margin-top:13px}.big-answer{font-size:15px;line-height:1.5;padding:14px;border-radius:16px;background:rgba(0,122,255,.08);min-height:52px}.section-note{margin-top:-5px;margin-bottom:12px}details>summary{cursor:pointer;font-weight:700;padding:4px 0 10px}.kpi{padding:14px;border-radius:18px;background:rgba(127,127,127,.07);border:1px solid var(--line)}.kpi b{display:block;font-size:12px;color:var(--muted);margin-bottom:8px}.kpi span{font-size:15px;font-weight:650}@media(max-width:760px){.grid2,.grid3,.telemetry{grid-template-columns:1fr}.wrap{padding:20px 14px 55px}h1{font-size:32px}.hero{display:block}.hero .muted{margin-top:8px}}
</style>
</head>
<body><div class="wrap">
<div class="hero"><div><div class="eyebrow">Unitree Go2 · Live Mission Console</div><h1>Sparky</h1><div class="muted">Navigation, perception, memory, speech and tour orchestration.</div></div><span class="status-pill" id="systemPill">Connecting</span></div>
<div class="muted">The phone performs speech recognition. Only text is sent to the DGX. Phone Sit/Stand/Wave/Dance/Front Flip gestures are one-tap. Navigation and object-finding stay confirmation-gated.</div>
<div class="card">
<label>Demo PIN / API token</label><input id="token" type="password" inputmode="numeric" autocomplete="off" placeholder="0000">
<div class="muted" style="margin-top:7px">For the supervised demo you can use SPARKY_WEB_TOKEN=0000. The same value is used as the Admin PIN by default.</div>
</div>
<div class="card" id="agentConsoleCard">
<!-- SPARKY_PHONE_UNIFIED_AGENT_UI_V12_8 -->
<label>Talk to Sparky</label>
<textarea id="agentInput" placeholder="Ask anything or give a command: What do you see? / Start the tour / Find the chair / Sit / Return to spawn"></textarea>
<div class="row"><button class="primary" onclick="sendAgentInput()">Send to Sparky</button><button onclick="startSpeech('agent')">🎤 Speak</button><button onclick="stopSpeech()">Stop listening</button><button class="danger" onclick="quick('stop')">STOP</button></div>
<div class="muted" id="speechStatus" style="margin-top:10px">One input goes through the same intent → VLM/memory/tool → action pipeline.</div>
<pre id="speechTranscript" style="min-height:44px;max-height:120px">No transcript yet.</pre>
<label style="display:flex;gap:8px;align-items:center"><input id="autoSendSpeech" type="checkbox" checked style="width:auto"> Automatically send final speech transcript</label>
<div id="result" class="muted" style="margin-top:10px"></div>
</div>
<div class="card">
<label>Guest Tour Host</label>
<div class="row"><button class="primary" onclick="host('welcome')">Welcome Guests</button><button onclick="host('lab_intro')">Lab Intro</button><button onclick="host('sparky_intro')">Introduce Sparky</button></div>
<div class="row" style="margin-top:10px"><button onclick="host('capabilities')">Capabilities</button><button class="safe" onclick="host('safe_moves')">Show Safe Moves</button><button onclick="quick('start the tour')">Start Saved Tour</button></div>
<div class="row" style="margin-top:10px"><button onclick="quick('wave')">Wave</button><button onclick="quick('dance')">Dance</button><button onclick="quick('stretch')">Stretch</button><button onclick="quick('sit')">Sit</button><button onclick="quick('stand')">Stand</button><button onclick="quick('heart')">Heart</button><button class="danger" title="Immediate phone gesture; blocked while semantic navigation is active" onclick="quick('front flip')">Front Flip</button></div>
</div>
<div class="card">
<h2>Map & localization</h2>
<div class="section-note muted">Live saved map with AMCL position and heading. Use this while issuing commands below or from the Command panel.</div>
<canvas id="mapCanvas" width="198" height="232" style="width:100%;max-height:520px;object-fit:contain;border-radius:18px;background:rgba(127,127,127,.08);border:1px solid var(--line)"></canvas>
<div class="row" style="margin-top:10px"><span class="status-pill" id="mapPill">Map</span><span class="badge" id="mapPose">pose: waiting</span></div>
</div>
<div class="card">
<h2>Live system</h2><div class="section-note muted">Every command is traceable from intent → tool → ROS dispatch → completion.</div>
<div class="status-strip"><span class="status-pill" id="agentPill">Agent</span><span class="status-pill" id="navPill">Nav2</span><span class="status-pill" id="visionPill">Vision</span><span class="status-pill" id="speechPill">Speech</span><span class="status-pill" id="motionPill">Motion</span><span class="status-pill" id="tourPill">Tour</span></div>
<div class="telemetry" style="margin-top:14px"><div><label>Live VLM</label><div id="vlmLive" class="big-answer">No live VLM call yet.</div></div><div><label>Current orchestration</label><pre id="orchestration">Waiting for a request…</pre></div></div>
<details style="margin-top:12px"><summary>Recent ROS / agent events</summary><pre id="eventHistory">No events yet.</pre></details>
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
async function command(text,confidence=1.0,source='phone_web'){try{const clean=(text||'').trim();if(!clean)return;const v=await api('/api/command','POST',{text:clean,confidence,source});document.getElementById('result').textContent='Sent to Sparky: '+(v.text||clean)}catch(e){document.getElementById('result').textContent='ERROR: '+e.message}}
function sendAgentInput(){command(document.getElementById('agentInput').value)} function quick(t){document.getElementById('agentInput').value=t;command(t)}
async function host(script){try{document.getElementById('result').textContent='Starting Tour Host: '+script;const v=await api('/api/tour_host','POST',{text:script,source:'phone_web_tour_host'});document.getElementById('result').textContent='Tour Host started: '+(v.script||script)}catch(e){document.getElementById('result').textContent='ERROR: '+e.message}}
async function sendQuery(confidence=1.0,source='phone_web_query'){return command(document.getElementById('agentInput').value,confidence,source)}
let activeRecognition=null;
function speechCtor(){return window.SpeechRecognition||window.webkitSpeechRecognition||null} function setSpeechStatus(t){document.getElementById('speechStatus').textContent=t}
function startSpeech(mode='agent'){const Ctor=speechCtor();if(!Ctor){setSpeechStatus('Browser SpeechRecognition unavailable.\nUse the text box or phone keyboard microphone.');return}stopSpeech();const rec=new Ctor();activeRecognition=rec;rec.lang='en-US';rec.continuous=false;rec.interimResults=true;rec.maxAlternatives=1;rec.onstart=()=>setSpeechStatus('Listening on phone…');rec.onerror=e=>{setSpeechStatus('Speech error: '+e.error);activeRecognition=null};rec.onend=()=>{if(activeRecognition===rec){setSpeechStatus('Stopped listening.');activeRecognition=null}};rec.onresult=async e=>{let interim='',finalText='',confidence=1.0;for(let i=e.resultIndex;i<e.results.length;i++){const alt=e.results[i][0];if(e.results[i].isFinal){finalText+=alt.transcript;if(Number.isFinite(alt.confidence)&&alt.confidence>0)confidence=alt.confidence}else interim+=alt.transcript}const shown=(finalText||interim).trim();if(shown)document.getElementById('speechTranscript').textContent=shown;if(!finalText.trim())return;const final=finalText.trim();setSpeechStatus('Recognized: '+final);if(mode==='stop_script'){document.getElementById('stopScript').value=final;return}document.getElementById('agentInput').value=final;if(document.getElementById('autoSendSpeech').checked)await command(final,confidence,'phone_chrome_speech')};try{rec.start()}catch(e){setSpeechStatus('Could not start microphone: '+e.message);activeRecognition=null}}
function stopSpeech(){if(activeRecognition){try{activeRecognition.stop()}catch(e){}activeRecognition=null}}
function short(v){if(v==null)return'?';if(typeof v==='string')return v.slice(0,90);return JSON.stringify(v).slice(0,90)}
function value(s,k){return s.latest?.[k]?.value}
function age(s,k){return Number(s.latest?.[k]?.age_sec??999)}
function setPill(id,label,ok,warn=false){const e=document.getElementById(id);if(!e)return;e.textContent=label;e.className='status-pill '+(ok?'good':warn?'warn':'bad')}
async function refresh(){try{
 const s=await api('/api/state'); const agent=value(s,'agent_interaction')||value(s,'agent_status'); const nav=value(s,'semantic_nav_status'); const vlm=value(s,'vlm_live'); const speech=value(s,'speech_status'); const speechReq=value(s,'speech_request'); const motion=value(s,'motion_status'); const tour=value(s,'tour_host_status')||value(s,'tour_status');
 const navFresh=age(s,'semantic_nav_status')<8, agentFresh=age(s,'agent_interaction')<15||age(s,'agent_status')<15, visionFresh=age(s,'vlm_live')<30||age(s,'object_inventory')<5;
 setPill('agentPill','Agent '+short(agent),agentFresh,!agentFresh); setPill('navPill','Nav2 '+short(nav),navFresh,!navFresh); setPill('visionPill','Vision '+(vlm?.provider||'detector'),visionFresh,!visionFresh); setPill('speechPill','Speech '+short(speech),age(s,'speech_status')<20,age(s,'speech_status')>=20); setPill('motionPill','Motion '+short(motion),age(s,'motion_status')<20,age(s,'motion_status')>=20); setPill('tourPill','Tour '+short(tour),age(s,'tour_host_status')<30||age(s,'tour_status')<30,true);
 setPill('systemPill',(agentFresh&&navFresh?'System ready':'System needs attention'),agentFresh&&navFresh,true);
 document.getElementById('vlmLive').textContent=vlm?(vlm.success===false?'VLM error: '+(vlm.error||'unknown'):((vlm.summary||'No summary')+'  ·  '+(vlm.provider||'?')+' / '+(vlm.model||'?'))):'No live VLM call yet. Ask “What do you see in front of me?”';
 if(speechReq?.text)document.getElementById('result').textContent=speechReq.text; document.getElementById('orchestration').textContent=JSON.stringify({agent,nav,speech,motion,tour},null,2);
 document.getElementById('eventHistory').textContent=(s.history||[]).slice(-24).reverse().map(x=>new Date((x.received_unix||0)*1000).toLocaleTimeString()+'  '+x.key+'  '+short(x.value)).join('\n')||'No events yet.';
 document.getElementById('state').textContent=JSON.stringify(s,null,2); document.getElementById('tourBadge').textContent='tour: '+short(value(s,'tour_status')); document.getElementById('voiceBadge').textContent='agent: '+short(agent); document.getElementById('navBadge').textContent='nav: '+short(nav);
}catch(e){document.getElementById('state').textContent='Status error: '+e.message;setPill('systemPill','Disconnected',false)}}
async function refreshMap(){try{
 const v=await api('/api/map'); const c=document.getElementById('mapCanvas'),ctx=c?.getContext('2d'); if(!ctx)return;
 const g=v.map,p=v.robot_pose; if(!g||!g.width||!g.height){setPill('mapPill','Map waiting',false,true);return}
 c.width=g.width;c.height=g.height;const img=ctx.createImageData(g.width,g.height),d=img.data;
 for(let y=0;y<g.height;y++){for(let x=0;x<g.width;x++){const src=y*g.width+x,dst=((g.height-1-y)*g.width+x)*4,val=Number(g.data[src]);let q=184;if(val===0)q=246;else if(val>50)q=24;d[dst]=q;d[dst+1]=q;d[dst+2]=q;d[dst+3]=255}}
 ctx.putImageData(img,0,0);
 if(p){const oy=Number(g.origin?.yaw||0),dx=p.x-Number(g.origin?.x||0),dy=p.y-Number(g.origin?.y||0),co=Math.cos(-oy),si=Math.sin(-oy),mx=(co*dx-si*dy)/g.resolution,my=(si*dx+co*dy)/g.resolution,px=mx,py=g.height-1-my,hy=p.yaw-oy;
  ctx.save();ctx.strokeStyle='#ff3b30';ctx.fillStyle='#ff3b30';ctx.lineWidth=Math.max(1,g.width/130);ctx.beginPath();ctx.arc(px,py,Math.max(2,g.width/80),0,Math.PI*2);ctx.fill();ctx.beginPath();ctx.moveTo(px,py);ctx.lineTo(px+Math.cos(hy)*Math.max(10,g.width/10),py-Math.sin(hy)*Math.max(10,g.width/10));ctx.stroke();ctx.restore();
  document.getElementById('mapPose').textContent='map pose: x='+p.x.toFixed(2)+'  y='+p.y.toFixed(2)+'  yaw='+(p.yaw*180/Math.PI).toFixed(1)+'°';
 }
 setPill('mapPill','Map '+g.width+'×'+g.height,true,false);
}catch(e){setPill('mapPill','Map error',false,true);const el=document.getElementById('mapPose');if(el)el.textContent='map: '+e.message}}
function adminResult(t){document.getElementById('adminResult').textContent=t}
async function refreshAdmin(){try{const v=await api('/api/admin/session','GET',null,true);adminCache=v;document.getElementById('adminSummary').textContent=JSON.stringify(v.memory,null,2);const sel=document.getElementById('stopSelect');sel.innerHTML='<option value="">Select a stop</option>';for(const stop of (v.route?.stops||[])){const o=document.createElement('option');o.value=stop.name||stop.place_name;o.textContent=(stop.name||stop.place_name)+' → '+(stop.place_name||'');o.dataset.stop=JSON.stringify(stop);sel.appendChild(o)}document.getElementById('stopOrder').value=(v.route?.stops||[]).map(x=>x.name||x.place_name).join('\n');document.getElementById('tourYaml').value=v.tour_host_yaml||'';adminResult('Loaded session '+v.session_name)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
function loadSelectedStop(){const sel=document.getElementById('stopSelect');const opt=sel.options[sel.selectedIndex];if(!opt||!opt.dataset.stop)return;const s=JSON.parse(opt.dataset.stop);document.getElementById('stopName').value=s.name||s.place_name||'';document.getElementById('stopScript').value=s.script||'';document.getElementById('stopFact').value=s.fact||'';document.getElementById('stopPause').value=s.pause_seconds??4}
async function saveStop(){try{const body={name:document.getElementById('stopName').value,room:document.getElementById('stopRoom').value,script:document.getElementById('stopScript').value,fact:document.getElementById('stopFact').value,pause_seconds:Number(document.getElementById('stopPause').value||4),tags:['tour_stop']};const v=await api('/api/admin/save_stop','POST',body,true);adminResult(v.message||'Save requested');setTimeout(refreshAdmin,700)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
async function updateStop(){try{const name=document.getElementById('stopSelect').value||document.getElementById('stopName').value;const body={name,script:document.getElementById('stopScript').value,fact:document.getElementById('stopFact').value,pause_seconds:Number(document.getElementById('stopPause').value||4)};const v=await api('/api/admin/update_stop','POST',body,true);adminResult(v.message||'Update requested');setTimeout(refreshAdmin,700)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
async function deleteStop(){try{const name=document.getElementById('stopSelect').value||document.getElementById('stopName').value;if(!name)throw new Error('Select a stop first');if(!confirm('Delete Tour stop '+name+'?'))return;const v=await api('/api/admin/delete_stop','POST',{name},true);adminResult(v.message||'Delete requested');setTimeout(refreshAdmin,700)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
async function saveOrder(){try{const order=document.getElementById('stopOrder').value.split(/\n+/).map(x=>x.trim()).filter(Boolean);const v=await api('/api/admin/reorder_stops','POST',{order},true);adminResult(v.message||'Order requested');setTimeout(refreshAdmin,700)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
async function writeCheckpoint(){try{const v=await api('/api/admin/write_checkpoint','POST',{},true);adminResult(v.message||'Checkpoint requested');setTimeout(refreshAdmin,700)}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
async function saveTourYaml(){try{const v=await api('/api/admin/tour_host_yaml','POST',{yaml:document.getElementById('tourYaml').value},true);adminResult(v.message||'YAML saved')}catch(e){adminResult('ADMIN ERROR: '+e.message)}}
setInterval(refresh,1000);refresh();setInterval(refreshMap,1800);refreshMap();
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
        self.declare_parameter("vlm_query_topic", "/go2_vlm/query")
        self.declare_parameter("semantic_nav_command_topic", "/semantic_nav/command")
        self.declare_parameter("snapshot_topic", "/go2_memory/write_snapshot_now")
        self.declare_parameter("tour_host_command_topic", "/go2_tour/host_command")

        self.transcript_pub = self.create_publisher(String, str(self.get_parameter("transcript_topic").value), 10)
        self.query_pub = self.create_publisher(String, str(self.get_parameter("query_topic").value), 10)
        self.vlm_query_pub = self.create_publisher(String, str(self.get_parameter("vlm_query_topic").value), 10)
        self.semantic_command_pub = self.create_publisher(String, str(self.get_parameter("semantic_nav_command_topic").value), 10)
        self.snapshot_pub = self.create_publisher(String, str(self.get_parameter("snapshot_topic").value), 10)
        self.tour_host_pub = self.create_publisher(String, str(self.get_parameter("tour_host_command_topic").value), 10)
        self._lock = threading.Lock()
        self._latest: dict[str, dict[str, Any]] = {}
        self._map_snapshot: dict[str, Any] | None = None
        self._pose_snapshot: dict[str, Any] | None = None
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_cb, 10)
        self._history: list[dict[str, Any]] = []

        topics = {
            "tour_status": "/go2_tour/status", "tour_host_status": "/go2_tour/host_status", "tour_narration": "/go2_tour/narration",
            "voice_state": "/go2_voice/verification_state", "voice_request": "/go2_voice/verification_request",
            "semantic_nav_status": "/semantic_nav/status", "semantic_nav_event": "/semantic_nav/event",
            "agent_status": "/go2_agent/status", "agent_interaction": "/go2_agent/interaction_state", "agent_events": "/go2_agent/events",
            "speech_status": "/go2_speech/status", "speech_request": "/go2_speech/request", "motion_status": "/motion_skills/status", "vlm_live": "/go2_vlm/query_result",
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
                if path=="/api/map":
                    if self._need_auth(): return
                    self._json(HTTPStatus.OK,node.map_snapshot()); return
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
                if path=="/api/tour_host":
                    node.publish_tour_host(text,source=source); self._json(HTTPStatus.ACCEPTED,{"ok":True,"kind":"tour_host","script":text}); return
                if path=="/api/command":
                    sent=node.publish_command(text,confidence=confidence,source=source); self._json(HTTPStatus.ACCEPTED,{"ok":True,"kind":"command","text":sent}); return
                if path=="/api/query":
                    node.publish_query(text,confidence=confidence,source=source); self._json(HTTPStatus.ACCEPTED,{"ok":True,"kind":"unified_intent","text":text}); return
                self._json(HTTPStatus.NOT_FOUND,{"error":"not found"})

        self._server=ThreadingHTTPServer((host,port),Handler); self._server.daemon_threads=True
        self._thread=threading.Thread(target=self._server.serve_forever,name="sparky-phone-web",daemon=True); self._thread.start()
        self.get_logger().info(f"Phone web gateway listening on http://{host}:{port}; token_required={require_token}; admin_enabled=true")

    def _bool_param(self,name:str)->bool:
        value=self.get_parameter(name).value; return value if isinstance(value,bool) else str(value).strip().lower() in {"1","true","yes","on"}
    @staticmethod
    def _yaw_from_quaternion(q: Any) -> float:
        return math.atan2(2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)), 1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2))

    def _map_cb(self, msg: OccupancyGrid) -> None:
        origin = msg.info.origin
        snap = {
            "width": int(msg.info.width), "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin": {"x": float(origin.position.x), "y": float(origin.position.y), "yaw": self._yaw_from_quaternion(origin.orientation)},
            "frame_id": str(msg.header.frame_id or "map"),
            "stamp_sec": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
            "data": [int(v) for v in msg.data],
        }
        with self._lock:
            self._map_snapshot = snap

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        snap = {
            "x": float(pose.position.x), "y": float(pose.position.y),
            "yaw": self._yaw_from_quaternion(pose.orientation),
            "frame_id": str(msg.header.frame_id or "map"),
            "stamp_sec": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
        }
        with self._lock:
            self._pose_snapshot = snap

    def map_snapshot(self) -> dict[str, Any]:
        with self._lock:
            grid = dict(self._map_snapshot) if isinstance(self._map_snapshot, dict) else None
            pose = dict(self._pose_snapshot) if isinstance(self._pose_snapshot, dict) else None
        return {"ok": bool(grid), "map": grid, "robot_pose": pose, "time_unix": time.time()}

    def _capture(self,key:str,msg:String)->None:
        raw=msg.data
        try: value:Any=json.loads(raw)
        except Exception: value=raw
        now=time.time()
        with self._lock:
            self._latest[key]={"value":value,"received_unix":now}
            self._history.append({"key":key,"value":value,"received_unix":now})
            self._history=self._history[-100:]
    def state_snapshot(self)->dict[str,Any]:
        now=time.time()
        with self._lock:
            latest={k:dict(v) for k,v in self._latest.items()}
            history=list(self._history[-60:])
        for item in latest.values():
            item["age_sec"]=max(0.0,now-float(item.get("received_unix") or now))
        return {"ok":True,"time_unix":now,"latest":latest,"history":history}
    def publish_command(self,text:str,confidence:float=1.0,source:str="phone_web")->str:
        stripped=str(text or "").strip()
        low=stripped.lower().strip(" .!?\t\r\n")
        no_prefix=low in {"yes","confirm","go ahead","proceed","no","reject","cancel","stop","halt","freeze"}
        if self._bool_param("auto_prefix_wake_word") and not no_prefix:
            wake=str(self.get_parameter("wake_word").value or "Sparky").strip()
            wake_low=wake.lower()
            already_prefixed=(low==wake_low or low.startswith(wake_low+" ") or low.startswith(wake_low+",") or low.startswith(wake_low+":"))
            if wake and not already_prefixed:
                stripped=f"{wake}, {stripped}"
        now=time.time()
        payload={
            "text":stripped,
            "confidence":max(0.0,min(1.0,float(confidence))),
            "source":source or "phone_web",
            "input_kind":"unified",
            "request_id":f"phone_{time.time_ns()}",
            "command_received_unix":now,
        }
        self.transcript_pub.publish(String(data=json.dumps(payload,sort_keys=True)))
        return stripped
    @staticmethod
    def _is_live_visual_query(text:str)->bool:
        low=" ".join(str(text or "").lower().split())
        phrases=(
            "what do you see", "what can you see", "what are you seeing",
            "look in front", "look around", "current camera", "camera view",
            "describe what you see", "describe the scene", "observe",
        )
        return any(p in low for p in phrases)
    def publish_query(self,text:str,confidence:float=1.0,source:str="phone_web_query")->None:
        # Backward-compatible API endpoint; classification happens once in the unified gate.
        self.publish_command(text, confidence=confidence, source=source or "phone_web_query")

    def publish_tour_host(self,script:str,source:str="phone_web_tour_host")->None:
        script=str(script or "").strip()
        if not script:
            return
        phrases={
            "full_intro":"start the introduction", "welcome":"welcome the guests",
            "lab_intro":"introduce the digital twin lab", "research_intro":"tell us about the research",
            "sparky_intro":"introduce yourself", "capabilities":"tell us your capabilities",
            "safe_moves":"show us some moves",
        }
        self.publish_command(phrases.get(script, script.replace("_", " ")), confidence=1.0, source=source)
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
