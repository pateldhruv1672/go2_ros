#!/usr/bin/env bash
set -eo pipefail
DURATION="${1:-20}"
echo "============================================================"
echo " SPARKY V11.6 COMMAND -> RESPONSE MONITOR (READ ONLY)"
echo "============================================================"
echo "duration=${DURATION}s"
echo
echo "This monitor publishes NO velocity commands."
echo "Start it immediately before one supervised short motion test."
echo
python3 "$(dirname "$0")/check_sparky_command_response_v11_6.py" --duration "$DURATION"
