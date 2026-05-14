from __future__ import annotations

import csv
import json
import logging
import os
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict

import yaml


LOGGER = logging.getLogger('go2_colmap_pipeline')


def configure_logging(verbose: bool = False) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(level=level, format='[%(levelname)s] %(message)s')


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def read_yaml(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def write_yaml(path: Path, data: Dict[str, Any]) -> None:
    with path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle, sort_keys=False)


def write_json(path: Path, data: Dict[str, Any]) -> None:
    with path.open('w', encoding='utf-8') as handle:
        json.dump(data, handle, indent=2)


def copy_file(src: Path, dst: Path) -> None:
    ensure_dir(dst.parent)
    shutil.copy2(src, dst)


def detect_storage_id(bag_path: Path) -> str:
    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.exists():
        return 'sqlite3'
    metadata = read_yaml(metadata_path)
    return metadata.get('rosbag2_bagfile_information', {}).get('storage_identifier', 'sqlite3')


def run_command(cmd: list[str], cwd: Path | None = None, env: dict[str, str] | None = None) -> None:
    LOGGER.info('Running: %s', ' '.join(cmd))
    completed = subprocess.run(cmd, cwd=str(cwd) if cwd else None, env=env, check=False)
    if completed.returncode != 0:
        raise RuntimeError('Command failed with exit code {}: {}'.format(completed.returncode, ' '.join(cmd)))


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    ensure_dir(path.parent)
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def which_or_raise(name: str) -> str:
    executable = shutil.which(name)
    if executable is None:
        raise FileNotFoundError(f'Required executable not found on PATH: {name}')
    return executable
