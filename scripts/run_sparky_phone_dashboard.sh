#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

if [[ -f "$ROOT_DIR/.env.local" ]]; then
  set -a
  . "$ROOT_DIR/.env.local"
  set +a
fi
if [[ -f "$SCRIPT_DIR/sparky_ros_env.sh" ]]; then
  . "$SCRIPT_DIR/sparky_ros_env.sh"
fi

if [[ "${1:-}" == "--demo" ]]; then
  export SPARKY_WEB_TOKEN="${SPARKY_WEB_TOKEN:-0000}"
  export SPARKY_ADMIN_PIN="${SPARKY_ADMIN_PIN:-$SPARKY_WEB_TOKEN}"
  echo "WARNING: --demo uses a simple trusted-LAN demo credential." >&2
fi

if [[ -z "${SPARKY_WEB_TOKEN:-}" ]]; then
  cat >&2 <<'MSG'
SPARKY_WEB_TOKEN is required because dashboard authentication is enabled.
For a supervised trusted-LAN demo:
  export SPARKY_WEB_TOKEN=0000
  export SPARKY_ADMIN_PIN=0000
Then rerun, or use --demo.
MSG
  exit 2
fi

# The dashboard UI currently uses the same entered value for API token + Admin PIN.
export SPARKY_ADMIN_PIN="${SPARKY_ADMIN_PIN:-$SPARKY_WEB_TOKEN}"

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROOT_DIR/src/.venv/bin/activate" ]]; then source "$ROOT_DIR/src/.venv/bin/activate"; fi
source "$ROOT_DIR/install/setup.bash"
set -u

PORT="${SPARKY_WEB_PORT:-8765}"
HOST="${SPARKY_WEB_BIND_HOST:-0.0.0.0}"
LAN_IP="$(hostname -I 2>/dev/null | awk '{print $1}' || true)"

echo "Sparky phone dashboard gateway"
echo "  local health: http://127.0.0.1:${PORT}/api/health"
if [[ -n "$LAN_IP" ]]; then
  echo "  phone URL:    http://${LAN_IP}:${PORT}/"
else
  echo "  phone URL:    http://<THIS_COMPUTER_LAN_IP>:${PORT}/"
fi
echo "  auth:         use SPARKY_WEB_TOKEN in the dashboard PIN/token field"

exec ros2 run go2_omi_voice_bridge phone_web_gateway_node --ros-args \
  -p bind_host:="$HOST" \
  -p port:="$PORT" \
  -p require_token:=true

