#!/usr/bin/env python3

import json
import math
import os
import re
import threading
import time
import traceback
from collections import deque
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


HTML = r"""
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Go2 ROS Topic Dashboard</title>
  <style>
    body {
      margin: 0;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #101216;
      color: #e8eaf0;
    }
    header {
      padding: 10px 16px;
      background: #1b1f2a;
      border-bottom: 1px solid #343a4a;
      display: flex;
      align-items: center;
      gap: 16px;
    }
    header h1 {
      font-size: 18px;
      margin: 0;
      white-space: nowrap;
    }
    header .badge {
      background: #24344f;
      border: 1px solid #3b5a8b;
      color: #d9e8ff;
      border-radius: 999px;
      padding: 4px 10px;
      font-size: 12px;
    }
    header a {
      color: #9ecbff;
      text-decoration: none;
      font-size: 13px;
    }
    #layout {
      display: grid;
      grid-template-columns: 360px 1fr;
      height: calc(100vh - 48px);
    }
    #sidebar {
      border-right: 1px solid #343a4a;
      overflow: auto;
      background: #151923;
    }
    #main {
      overflow: auto;
      padding: 12px;
    }
    .panel {
      background: #181d28;
      border: 1px solid #343a4a;
      border-radius: 10px;
      margin-bottom: 12px;
      overflow: hidden;
    }
    .panel h2 {
      font-size: 14px;
      margin: 0;
      padding: 10px 12px;
      background: #202737;
      border-bottom: 1px solid #343a4a;
    }
    .panel .content {
      padding: 10px 12px;
    }
    input {
      width: calc(100% - 24px);
      background: #0f131b;
      border: 1px solid #343a4a;
      color: #e8eaf0;
      padding: 8px 10px;
      border-radius: 8px;
      margin: 10px 12px;
      outline: none;
    }
    button {
      background: #2f6fed;
      border: 0;
      color: white;
      padding: 7px 10px;
      border-radius: 7px;
      cursor: pointer;
      margin-right: 6px;
    }
    button.secondary {
      background: #303848;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 12px;
    }
    th, td {
      padding: 6px 8px;
      border-bottom: 1px solid #2a3040;
      vertical-align: top;
    }
    th {
      text-align: left;
      color: #a9b4ca;
      background: #1d2331;
      position: sticky;
      top: 0;
    }
    tr.topic-row {
      cursor: pointer;
    }
    tr.topic-row:hover {
      background: #20283a;
    }
    tr.selected {
      background: #24344f !important;
    }
    .rate {
      color: #73d67a;
      font-weight: 600;
    }
    .stale {
      color: #f7c96b;
    }
    .bad {
      color: #ff7b7b;
    }
    .small {
      color: #9aa6bf;
      font-size: 11px;
    }
    pre {
      background: #0c0f15;
      border: 1px solid #2a3040;
      border-radius: 8px;
      padding: 10px;
      overflow: auto;
      max-height: 420px;
      font-size: 12px;
      line-height: 1.35;
    }
    .logline {
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
      border-bottom: 1px solid #262c3a;
      padding: 6px 8px;
      white-space: pre-wrap;
    }
    .logline.warn {
      color: #f7c96b;
    }
    .logline.error {
      color: #ff7b7b;
    }
    .logline.info {
      color: #dbe4ff;
    }
    .pill {
      display: inline-block;
      padding: 2px 7px;
      border-radius: 999px;
      background: #2a3142;
      margin-right: 4px;
      margin-bottom: 4px;
      font-size: 11px;
    }
  </style>
</head>
<body>
<header>
  <h1>Go2 ROS Topic Dashboard</h1>
  <span class="badge" id="status">connecting...</span>
  <span class="badge" id="topic-count">0 topics</span>
  <span class="badge" id="msg-count">0 msgs</span>
  <a href="/api/download" target="_blank">download JSONL log</a>
</header>

<div id="layout">
  <aside id="sidebar">
    <input id="topic-filter" placeholder="filter topics, e.g. nav, mrkl, scan, cmd_vel"/>
    <table>
      <thead>
        <tr>
          <th>Topic</th>
          <th>Hz</th>
          <th>Count</th>
        </tr>
      </thead>
      <tbody id="topics"></tbody>
    </table>
  </aside>

  <main id="main">
    <div class="panel">
      <h2>Selected Topic</h2>
      <div class="content">
        <div id="selected-title" class="small">none</div>
        <div style="margin-top: 8px;">
          <button onclick="pauseToggle()" id="pause-btn">Pause</button>
          <button class="secondary" onclick="refreshNow()">Refresh</button>
        </div>
      </div>
    </div>

    <div class="panel">
      <h2>Important System Topics</h2>
      <div class="content" id="important"></div>
    </div>

    <div class="panel">
      <h2>Recent Messages for Selected Topic</h2>
      <div class="content">
        <pre id="topic-json">Select a topic on the left.</pre>
      </div>
    </div>

    <div class="panel">
      <h2>Recent Global Events</h2>
      <div id="global-events"></div>
    </div>
  </main>
</div>

<script>
let selectedTopic = "";
let paused = false;
let latestState = null;

function fmtRate(v) {
  if (v === null || v === undefined) return "0.0";
  return Number(v).toFixed(1);
}

function clsForTopic(t) {
  const age = t.age_sec || 999;
  if (age > 5 && t.count > 0) return "stale";
  return "rate";
}

function esc(s) {
  return String(s).replace(/[&<>"']/g, c => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;", "'": "&#39;"
  }[c]));
}

async function fetchJson(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${r.status} ${r.statusText}`);
  return await r.json();
}

async function refreshState() {
  if (paused) return;
  try {
    latestState = await fetchJson("/api/state");
    document.getElementById("status").textContent = "live";
    document.getElementById("topic-count").textContent = latestState.topic_count + " topics";
    document.getElementById("msg-count").textContent = latestState.total_messages + " msgs";
    renderTopics(latestState);
    renderImportant(latestState);
    renderGlobal(latestState);
    if (selectedTopic) await refreshTopic();
  } catch (e) {
    document.getElementById("status").textContent = "offline: " + e;
  }
}

function renderTopics(state) {
  const filter = document.getElementById("topic-filter").value.toLowerCase();
  const tbody = document.getElementById("topics");
  tbody.innerHTML = "";

  const topics = state.topics
    .filter(t => !filter || t.name.toLowerCase().includes(filter) || t.type.toLowerCase().includes(filter))
    .sort((a, b) => a.name.localeCompare(b.name));

  for (const t of topics) {
    const tr = document.createElement("tr");
    tr.className = "topic-row" + (t.name === selectedTopic ? " selected" : "");
    tr.onclick = () => {
      selectedTopic = t.name;
      document.getElementById("selected-title").innerHTML =
        `<b>${esc(t.name)}</b><br/><span class="small">${esc(t.type)}</span>`;
      refreshTopic();
      renderTopics(latestState);
    };
    tr.innerHTML = `
      <td>
        <div>${esc(t.name)}</div>
        <div class="small">${esc(t.type.split("/").slice(-2).join("/"))}</div>
      </td>
      <td class="${clsForTopic(t)}">${fmtRate(t.rate_hz)}</td>
      <td>${t.count}</td>
    `;
    tbody.appendChild(tr);
  }
}

function renderImportant(state) {
  const wanted = [
    "/mrkl_explorer/trace",
    "/mrkl_explorer/status",
    "/object_explorer/state",
    "/object_explorer/llm_decision",
    "/object_explorer/detections",
    "/object_explorer/sam2_detections",
    "/cmd_vel_nav2",
    "/cmd_vel_omi",
    "/cmd_vel_nav",
    "/cmd_vel_out",
    "/scan_nav",
    "/plan",
    "/rosout"
  ];
  const byName = {};
  for (const t of state.topics) byName[t.name] = t;

  let html = "";
  for (const name of wanted) {
    const t = byName[name];
    if (!t) {
      html += `<span class="pill bad">${esc(name)}: missing</span>`;
    } else {
      html += `<span class="pill">${esc(name)}: ${fmtRate(t.rate_hz)} Hz, ${t.count}</span>`;
    }
  }
  document.getElementById("important").innerHTML = html;
}

async function refreshTopic() {
  if (!selectedTopic) return;
  const data = await fetchJson("/api/topic?name=" + encodeURIComponent(selectedTopic));
  document.getElementById("topic-json").textContent = JSON.stringify(data.messages, null, 2);
}

function renderGlobal(state) {
  const el = document.getElementById("global-events");
  el.innerHTML = "";

  for (const m of state.global_events.slice().reverse()) {
    const text = JSON.stringify(m.summary);
    let cls = "logline info";
    if (text.toLowerCase().includes("error") || text.toLowerCase().includes("failed")) cls = "logline error";
    else if (text.toLowerCase().includes("warn") || text.toLowerCase().includes("stop") || text.toLowerCase().includes("invalid")) cls = "logline warn";

    const div = document.createElement("div");
    div.className = cls;
    div.textContent = `[${new Date(m.t * 1000).toLocaleTimeString()}] ${m.topic}: ${text}`;
    el.appendChild(div);
  }
}

function pauseToggle() {
  paused = !paused;
  document.getElementById("pause-btn").textContent = paused ? "Resume" : "Pause";
}

function refreshNow() {
  paused = false;
  document.getElementById("pause-btn").textContent = "Pause";
  refreshState();
}

document.getElementById("topic-filter").addEventListener("input", () => {
  if (latestState) renderTopics(latestState);
});

setInterval(refreshState, 1000);
refreshState();
</script>
</body>
</html>
"""


def parse_bool(v):
    if isinstance(v, bool):
        return v
    return str(v).strip().lower() in ("1", "true", "yes", "on")


def json_safe(obj, max_items=40, depth=0):
    if depth > 5:
        return str(type(obj).__name__)

    if obj is None or isinstance(obj, (bool, int, float, str)):
        if isinstance(obj, float) and (math.isnan(obj) or math.isinf(obj)):
            return str(obj)
        if isinstance(obj, str) and len(obj) > 2000:
            return obj[:2000] + "...<truncated>"
        return obj

    if isinstance(obj, bytes):
        return {"bytes_len": len(obj)}

    if isinstance(obj, dict):
        out = {}
        for k, v in list(obj.items())[:max_items]:
            out[str(k)] = json_safe(v, max_items=max_items, depth=depth + 1)
        if len(obj) > max_items:
            out["_truncated_keys"] = len(obj) - max_items
        return out

    if isinstance(obj, (list, tuple)):
        if len(obj) > max_items:
            return {
                "_sequence_len": len(obj),
                "_sample": [json_safe(x, max_items=max_items, depth=depth + 1) for x in obj[:max_items]],
            }
        return [json_safe(x, max_items=max_items, depth=depth + 1) for x in obj]

    return str(obj)


def header_summary(msg):
    if not hasattr(msg, "header"):
        return None
    h = msg.header
    return {
        "frame_id": getattr(h, "frame_id", ""),
        "stamp": {
            "sec": int(getattr(h.stamp, "sec", 0)),
            "nanosec": int(getattr(h.stamp, "nanosec", 0)),
        },
    }


def summarize_message(msg, type_name):
    h = header_summary(msg)

    if type_name == "sensor_msgs/msg/Image":
        return {
            "header": h,
            "height": int(msg.height),
            "width": int(msg.width),
            "encoding": str(msg.encoding),
            "step": int(msg.step),
            "data_len": len(msg.data),
        }

    if type_name == "sensor_msgs/msg/PointCloud2":
        return {
            "header": h,
            "height": int(msg.height),
            "width": int(msg.width),
            "fields": [f.name for f in msg.fields],
            "point_step": int(msg.point_step),
            "row_step": int(msg.row_step),
            "data_len": len(msg.data),
        }

    if type_name == "sensor_msgs/msg/LaserScan":
        vals = [float(x) for x in msg.ranges if math.isfinite(float(x))]
        return {
            "header": h,
            "angle_min": float(msg.angle_min),
            "angle_max": float(msg.angle_max),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "ranges_len": len(msg.ranges),
            "finite_min": min(vals) if vals else None,
            "finite_max": max(vals) if vals else None,
        }

    if type_name == "nav_msgs/msg/OccupancyGrid":
        return {
            "header": h,
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin": {
                "x": float(msg.info.origin.position.x),
                "y": float(msg.info.origin.position.y),
            },
            "data_len": len(msg.data),
        }

    if type_name == "nav_msgs/msg/Path":
        n = len(msg.poses)
        last = None
        if n:
            p = msg.poses[-1].pose.position
            last = {"x": float(p.x), "y": float(p.y), "z": float(p.z)}
        return {
            "header": h,
            "poses_len": n,
            "last_pose": last,
        }

    try:
        return json_safe(message_to_ordereddict(msg))
    except Exception:
        return {"repr": repr(msg)[:4000]}


class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_json(self, obj):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_html(self, html):
        data = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        node = self.server.dashboard_node
        parsed = urlparse(self.path)

        if parsed.path == "/":
            self.send_html(HTML)
            return

        if parsed.path == "/api/state":
            self.send_json(node.snapshot_state())
            return

        if parsed.path == "/api/topic":
            qs = parse_qs(parsed.query)
            name = unquote(qs.get("name", [""])[0])
            self.send_json(node.snapshot_topic(name))
            return

        if parsed.path == "/api/download":
            try:
                with open(node.log_path, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "application/x-ndjson")
                self.send_header(
                    "Content-Disposition",
                    f'attachment; filename="{os.path.basename(node.log_path)}"',
                )
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            except Exception as exc:
                self.send_json({"error": str(exc)})
            return

        self.send_response(404)
        self.end_headers()


class TopicWebDashboardNode(Node):
    def __init__(self):
        super().__init__("topic_web_dashboard_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8766)
        self.declare_parameter("auto_discover", True)
        self.declare_parameter("topic_allow_regex", ".*")
        self.declare_parameter("topic_deny_regex", "^/parameter_events$")
        self.declare_parameter("max_messages_per_topic", 100)
        self.declare_parameter("max_global_events", 300)
        self.declare_parameter("discover_period_sec", 2.0)
        self.declare_parameter("log_dir", "~/.ros/go2_object_explorer/topic_web_logs")

        self.lock = threading.RLock()
        self.subs = {}
        self.topic_types = {}
        self.topic_stats = {}
        self.topic_buffers = {}
        self.global_events = deque(maxlen=int(self.get_parameter("max_global_events").value))
        self.total_messages = 0

        log_dir = os.path.expanduser(str(self.get_parameter("log_dir").value))
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, time.strftime("go2_topics_%Y%m%d_%H%M%S.jsonl"))
        self.log_f = open(self.log_path, "a", buffering=1, encoding="utf-8")

        self.discover_timer = self.create_timer(
            float(self.get_parameter("discover_period_sec").value),
            self.discover_topics,
        )

        self.start_http_server()

        self.get_logger().info(f"Topic Web Dashboard listening at http://localhost:{self.get_parameter('port').value}")
        self.get_logger().info(f"Logging JSONL to {self.log_path}")

    def qos(self):
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

    def start_http_server(self):
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)

        self.httpd = ThreadingHTTPServer((host, port), DashboardHandler)
        self.httpd.dashboard_node = self
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()

    def topic_allowed(self, topic):
        allow = str(self.get_parameter("topic_allow_regex").value)
        deny = str(self.get_parameter("topic_deny_regex").value)

        if deny and re.search(deny, topic):
            return False
        if allow and not re.search(allow, topic):
            return False
        return True

    def discover_topics(self):
        if not parse_bool(self.get_parameter("auto_discover").value):
            return

        for topic, types in self.get_topic_names_and_types():
            if not types:
                continue
            if topic in self.subs:
                continue
            if not self.topic_allowed(topic):
                continue

            type_name = types[0]

            try:
                msg_cls = get_message(type_name)
                sub = self.create_subscription(
                    msg_cls,
                    topic,
                    lambda msg, topic=topic, type_name=type_name: self.on_message(topic, type_name, msg),
                    self.qos(),
                )
                with self.lock:
                    self.subs[topic] = sub
                    self.topic_types[topic] = type_name
                    self.topic_buffers[topic] = deque(
                        maxlen=int(self.get_parameter("max_messages_per_topic").value)
                    )
                    self.topic_stats[topic] = {
                        "name": topic,
                        "type": type_name,
                        "count": 0,
                        "rate_hz": 0.0,
                        "last_time": None,
                        "age_sec": None,
                        "error": None,
                    }
                self.get_logger().info(f"dashboard subscribed: {topic} [{type_name}]")
            except Exception as exc:
                self.get_logger().warn(f"could not subscribe to {topic} [{type_name}]: {exc}")

    def on_message(self, topic, type_name, msg):
        now = time.time()

        try:
            summary = summarize_message(msg, type_name)
        except Exception as exc:
            summary = {
                "error": str(exc),
                "traceback": traceback.format_exc(limit=2),
            }

        event = {
            "t": now,
            "topic": topic,
            "type": type_name,
            "summary": summary,
        }

        with self.lock:
            st = self.topic_stats.get(topic)
            if st is None:
                return

            last_time = st.get("last_time")
            st["count"] += 1
            st["last_time"] = now
            st["age_sec"] = 0.0
            st["error"] = None

            if last_time is not None:
                dt = max(1e-6, now - last_time)
                inst = 1.0 / dt
                old = float(st.get("rate_hz") or 0.0)
                st["rate_hz"] = inst if old <= 0.0 else 0.85 * old + 0.15 * inst

            self.topic_buffers[topic].append(event)
            self.global_events.append(event)
            self.total_messages += 1

        try:
            self.log_f.write(json.dumps(event, ensure_ascii=False) + "\n")
        except Exception:
            pass

    def snapshot_state(self):
        now = time.time()
        with self.lock:
            topics = []
            for topic, st in self.topic_stats.items():
                x = dict(st)
                if x["last_time"] is not None:
                    x["age_sec"] = now - x["last_time"]
                topics.append(x)

            return {
                "now": now,
                "topic_count": len(topics),
                "total_messages": self.total_messages,
                "log_path": self.log_path,
                "topics": topics,
                "global_events": list(self.global_events)[-int(self.get_parameter("max_global_events").value):],
            }

    def snapshot_topic(self, name):
        with self.lock:
            return {
                "name": name,
                "type": self.topic_types.get(name),
                "stats": self.topic_stats.get(name),
                "messages": list(self.topic_buffers.get(name, [])),
            }

    def destroy_node(self):
        try:
            self.log_f.close()
        except Exception:
            pass
        try:
            self.httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TopicWebDashboardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
