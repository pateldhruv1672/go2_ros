from __future__ import annotations

import argparse
from pathlib import Path

from .common import LOGGER, configure_logging, ensure_dir


def convert_with_open3d(input_path: Path, output_path: Path) -> None:
    try:
        import open3d as o3d
    except ImportError as exc:  # pragma: no cover
        raise ImportError('open3d is required for mesh conversion. pip install open3d') from exc

    mesh = o3d.io.read_triangle_mesh(str(input_path))
    if mesh.is_empty():
        raise RuntimeError(f'Failed to read mesh from {input_path}')
    ensure_dir(output_path.parent)
    ok = o3d.io.write_triangle_mesh(str(output_path), mesh, write_triangle_uvs=True)
    if not ok:
        raise RuntimeError(f'Failed to write OBJ mesh to {output_path}')


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Convert a reconstructed mesh into OBJ for Isaac Sim import.')
    parser.add_argument('--input-mesh', required=True)
    parser.add_argument('--output-obj', required=True)
    parser.add_argument('--verbose', action='store_true')
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    configure_logging(args.verbose)
    input_path = Path(args.input_mesh)
    output_path = Path(args.output_obj)
    LOGGER.info('Converting %s -> %s', input_path, output_path)
    convert_with_open3d(input_path, output_path)


if __name__ == '__main__':
    main()
