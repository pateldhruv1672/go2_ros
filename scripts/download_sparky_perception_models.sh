#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-$(pwd)}"
ROOT="$(cd "$ROOT" && pwd)"
MODEL_DIR="${SPARKY_MODEL_DIR:-$HOME/.cache/sparky_models}"
mkdir -p "$MODEL_DIR"

if [[ -n "${VIRTUAL_ENV:-}" && -x "$VIRTUAL_ENV/bin/python" ]]; then
  PY="$VIRTUAL_ENV/bin/python"
elif [[ -x "$ROOT/src/.venv/bin/python" ]]; then
  PY="$ROOT/src/.venv/bin/python"
elif [[ -x "$ROOT/.venv/bin/python" ]]; then
  PY="$ROOT/.venv/bin/python"
else
  PY="$(command -v python3 || command -v python)"
fi

echo "Using Python: $PY"
echo "Model cache:  $MODEL_DIR"

if ! "$PY" -c 'from ultralytics import YOLO, SAM' >/dev/null 2>&1; then
  echo "ultralytics is missing; installing it in the selected Python environment..."
  "$PY" -m pip install -U ultralytics
fi

MODEL_DIR="$MODEL_DIR" "$PY" - <<'PY_MODELS'
import os
from pathlib import Path
from ultralytics import YOLO, SAM

out = Path(os.environ['MODEL_DIR']).expanduser().resolve()
out.mkdir(parents=True, exist_ok=True)
os.chdir(out)

models = [
    ('YOLO', 'yolov8n.pt', YOLO),
    ('SAM2', 'sam2_t.pt', SAM),
]
for label, filename, cls in models:
    path = out / filename
    if path.exists() and path.stat().st_size > 1024 * 1024:
        print(f'OK   {label}: already present -> {path}')
        continue
    print(f'DOWNLOAD {label}: {filename}')
    cls(filename)
    if not path.exists():
        # Ultralytics may resolve a cache path internally. Ask the constructed model for its source.
        obj = cls(filename)
        candidate = Path(str(getattr(obj, 'ckpt_path', '') or getattr(obj, 'model_name', ''))).expanduser()
        if candidate.exists() and candidate.resolve() != path:
            path.write_bytes(candidate.read_bytes())
    if not path.exists() or path.stat().st_size < 1024 * 1024:
        raise RuntimeError(f'{label} weight was not downloaded to {path}')
    print(f'OK   {label}: {path} ({path.stat().st_size/1024/1024:.1f} MiB)')
PY_MODELS

# Compatibility with the existing VLN node defaults (yolov8n.pt / sam2_t.pt).
ln -sfn "$MODEL_DIR/yolov8n.pt" "$ROOT/yolov8n.pt"
ln -sfn "$MODEL_DIR/sam2_t.pt" "$ROOT/sam2_t.pt"

echo
echo "Perception models are ready:"
echo "  $MODEL_DIR/yolov8n.pt"
echo "  $MODEL_DIR/sam2_t.pt"
echo "Workspace compatibility symlinks:"
echo "  $ROOT/yolov8n.pt"
echo "  $ROOT/sam2_t.pt"
