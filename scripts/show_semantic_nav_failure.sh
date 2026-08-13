#!/usr/bin/env bash
set -u
LOGROOT="$(ls -td "$HOME/.ros/log"/* 2>/dev/null | head -1 || true)"
echo "latest_log_dir=$LOGROOT"
if [[ -n "$LOGROOT" && -d "$LOGROOT" ]]; then
  grep -RniE 'semantic_nav_node|Traceback|process has died|RuntimeError|ERROR' "$LOGROOT" 2>/dev/null | tail -120 || true
fi
