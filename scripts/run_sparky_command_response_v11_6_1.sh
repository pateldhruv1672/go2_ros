#!/usr/bin/env bash
set -eo pipefail
DURATION="${1:-20}"
echo "============================================================"
echo " SPARKY V11.6.1 COMMAND / RESPONSE MONITOR - READ ONLY"
echo "============================================================"
echo "duration=${DURATION}s"
echo "Publishes NO motion commands."
echo
python3 "$(dirname "$0")/check_sparky_command_response_v11_6_1.py" --duration "$DURATION"
