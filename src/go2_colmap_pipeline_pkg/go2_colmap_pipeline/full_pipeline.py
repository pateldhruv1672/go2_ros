from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from .common import LOGGER, configure_logging


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Run bag export and COLMAP reconstruction end to end.')
    parser.add_argument('--bag', required=True)
    parser.add_argument('--dataset-dir', required=True)
    parser.add_argument('--workspace-dir', required=True)
    parser.add_argument('--image-topic', default='/camera/image_raw')
    parser.add_argument('--camera-info-topic', default='/camera/camera_info')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--camera-info-yaml', default='')
    parser.add_argument('--camera-extrinsics', default='')
    parser.add_argument('--align-with-priors', action='store_true')
    parser.add_argument('--skip-dense', action='store_true')
    parser.add_argument('--verbose', action='store_true')
    return parser


def run_step(module: str, args: list[str]) -> None:
    cmd = [sys.executable, '-m', module, *args]
    LOGGER.info('Running: %s', ' '.join(cmd))
    subprocess.run(cmd, check=True)


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    configure_logging(args.verbose)

    bag_args = [
        '--bag', args.bag,
        '--output-dir', args.dataset_dir,
        '--image-topic', args.image_topic,
        '--camera-info-topic', args.camera_info_topic,
        '--odom-topic', args.odom_topic,
    ]
    if args.camera_info_yaml:
        bag_args += ['--camera-info-yaml', args.camera_info_yaml]
    if args.camera_extrinsics:
        bag_args += ['--camera-extrinsics', args.camera_extrinsics]
    if args.verbose:
        bag_args += ['--verbose']

    colmap_args = [
        '--dataset-dir', args.dataset_dir,
        '--workspace-dir', args.workspace_dir,
    ]
    if args.align_with_priors:
        colmap_args += ['--align-with-priors']
    if args.skip_dense:
        colmap_args += ['--skip-dense']
    if args.verbose:
        colmap_args += ['--verbose']

    run_step('go2_colmap_pipeline.bag_to_dataset', bag_args)
    run_step('go2_colmap_pipeline.run_colmap', colmap_args)

    LOGGER.info('Done. Dataset: %s Workspace: %s', Path(args.dataset_dir), Path(args.workspace_dir))


if __name__ == '__main__':
    main()
