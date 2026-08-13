#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
[[ -f "${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}/install/setup.bash" ]] && source "${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}/install/setup.bash"
set -u

count_node_endpoints() {
  local topic="$1" kind="$2" node="$3"
  ros2 topic info "$topic" --verbose 2>/dev/null | awk -v kind="$kind" -v node="$node" '
    BEGIN{section=""; count=0}
    /^Publishers:/{section="pub"; next}
    /^Subscribers:/{section="sub"; next}
    /^Node name:/{
      name=$0; sub(/^Node name:[[:space:]]*/, "", name)
      if (section==kind && name==node) count++
    }
    END{print count}
  '
}

nav2_subs="$(count_node_endpoints /cmd_vel_nav2 sub go2_motion_arbiter)"
escape_subs="$(count_node_endpoints /cmd_vel_escape sub go2_motion_arbiter)"
nav_pubs="$(count_node_endpoints /cmd_vel_nav pub go2_motion_arbiter)"

printf 'go2_motion_arbiter endpoints:\n'
printf '  /cmd_vel_nav2 subscriptions:  %s\n' "$nav2_subs"
printf '  /cmd_vel_escape subscriptions:%s\n' "$escape_subs"
printf '  /cmd_vel_nav publishers:      %s\n' "$nav_pubs"

if [[ "$nav2_subs" != "1" || "$escape_subs" != "1" || "$nav_pubs" != "1" ]]; then
  echo
  echo 'FAIL: motion command ownership is duplicated.' >&2
  echo 'Process candidates:' >&2
  pgrep -af 'motion_arbiter|semantic_nav_resume.launch.py' >&2 || true
  exit 2
fi

echo 'PASS: exactly one motion arbiter owns the Nav2/escape -> /cmd_vel_nav path.'

echo
printf '/cmd_vel_out publishers (2 is expected when collision_monitor + unified emergency-stop publisher are active):\n'
ros2 topic info /cmd_vel_out --verbose 2>/dev/null | awk '
  /^Publishers:/{p=1; next}
  /^Subscribers:/{p=0}
  p && /^Node name:/{print "  " $0}
'
