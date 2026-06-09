#!/usr/bin/env bash

# Source this before running ROS 2 CLI commands against Sparky.
# It keeps the DDS and ROS discovery settings consistent across terminals.

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'
