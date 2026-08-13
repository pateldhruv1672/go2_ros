#!/usr/bin/env bash
# Source this in every Sparky terminal before ros2 CLI commands.
export ROBOT_IP="${ROBOT_IP:-192.168.12.1}"
export CONN_TYPE="${CONN_TYPE:-webrtc}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Do not override a user/site CycloneDDS configuration. If no custom config is
# supplied, only increase the auto participant-index ceiling. This prevents
# RViz + dashboard + ros2 CLI tools from exhausting a small auto-index range.
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
  export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><Discovery><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
fi
