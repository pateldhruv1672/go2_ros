from __future__ import annotations

import argparse
from pathlib import Path

from .colmap_camera import camera_info_to_colmap
from .common import LOGGER, configure_logging, ensure_dir, read_yaml, run_command, which_or_raise


class ColmapRunner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.colmap = which_or_raise(args.colmap_bin)
        self.dataset_dir = Path(args.dataset_dir)
        self.workspace_dir = Path(args.workspace_dir)
        self.image_dir = self.dataset_dir / 'images'
        self.database_path = self.workspace_dir / 'database.db'
        self.sparse_dir = self.workspace_dir / 'sparse'
        self.dense_dir = self.workspace_dir / 'dense'
        self.aligned_sparse_dir = self.workspace_dir / 'sparse_aligned'
        ensure_dir(self.workspace_dir)
        ensure_dir(self.sparse_dir)

    def _camera_args(self) -> tuple[str, str]:
        if self.args.camera_model and self.args.camera_params:
            return self.args.camera_model, self.args.camera_params
        camera_info = read_yaml(self.dataset_dir / 'camera_info.yaml')
        return camera_info_to_colmap(camera_info)

    def feature_extractor(self) -> None:
        camera_model, camera_params = self._camera_args()
        cmd = [
            self.colmap,
            'feature_extractor',
            '--database_path', str(self.database_path),
            '--image_path', str(self.image_dir),
            '--ImageReader.single_camera', '1',
            '--ImageReader.camera_model', camera_model,
            '--ImageReader.camera_params', camera_params,
        ]
        run_command(cmd)

    def matcher(self) -> None:
        matcher_name = 'sequential_matcher' if self.args.matcher == 'sequential' else 'exhaustive_matcher'
        cmd = [self.colmap, matcher_name, '--database_path', str(self.database_path)]
        if matcher_name == 'sequential_matcher':
            cmd.extend(['--SequentialMatching.overlap', str(self.args.sequential_overlap)])
        run_command(cmd)

    def mapper(self) -> None:
        cmd = [
            self.colmap,
            'mapper',
            '--database_path', str(self.database_path),
            '--image_path', str(self.image_dir),
            '--output_path', str(self.sparse_dir),
        ]
        run_command(cmd)

    def maybe_align(self) -> Path:
        sparse_model_dir = self.sparse_dir / '0'
        ref_images = self.dataset_dir / 'ref_images.txt'
        if not self.args.align_with_priors or not ref_images.exists():
            return sparse_model_dir
        ensure_dir(self.aligned_sparse_dir)
        cmd = [
            self.colmap,
            'model_aligner',
            '--input_path', str(sparse_model_dir),
            '--output_path', str(self.aligned_sparse_dir),
            '--ref_images_path', str(ref_images),
            '--ref_is_gps', '0',
            '--alignment_type', 'custom',
            '--robust_alignment', '1',
            '--robust_alignment_max_error', str(self.args.robust_alignment_max_error),
        ]
        run_command(cmd)
        return self.aligned_sparse_dir

    def dense(self, sparse_input: Path) -> None:
        ensure_dir(self.dense_dir)
        run_command([
            self.colmap,
            'image_undistorter',
            '--image_path', str(self.image_dir),
            '--input_path', str(sparse_input),
            '--output_path', str(self.dense_dir),
            '--output_type', 'COLMAP',
        ])
        run_command([
            self.colmap,
            'patch_match_stereo',
            '--workspace_path', str(self.dense_dir),
        ])
        run_command([
            self.colmap,
            'stereo_fusion',
            '--workspace_path', str(self.dense_dir),
            '--output_path', str(self.dense_dir / 'fused.ply'),
        ])
        mesher = 'poisson_mesher' if self.args.mesh_method == 'poisson' else 'delaunay_mesher'
        run_command([
            self.colmap,
            mesher,
            '--input_path', str(self.dense_dir / 'fused.ply'),
            '--output_path', str(self.dense_dir / self.args.mesh_name),
        ])

    def run(self) -> None:
        if not self.image_dir.exists():
            raise FileNotFoundError(f'Image directory not found: {self.image_dir}')
        self.feature_extractor()
        self.matcher()
        self.mapper()
        sparse_input = self.maybe_align()
        if not self.args.skip_dense:
            self.dense(sparse_input)
        LOGGER.info('COLMAP workspace ready at %s', self.workspace_dir)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Run the COLMAP reconstruction pipeline.')
    parser.add_argument('--dataset-dir', required=True)
    parser.add_argument('--workspace-dir', required=True)
    parser.add_argument('--colmap-bin', default='colmap')
    parser.add_argument('--matcher', default='sequential', choices=['sequential', 'exhaustive'])
    parser.add_argument('--sequential-overlap', default=10, type=int)
    parser.add_argument('--skip-dense', action='store_true')
    parser.add_argument('--mesh-method', default='poisson', choices=['poisson', 'delaunay'])
    parser.add_argument('--mesh-name', default='scene_poisson.ply')
    parser.add_argument('--align-with-priors', action='store_true')
    parser.add_argument('--robust-alignment-max-error', default=0.50, type=float)
    parser.add_argument('--camera-model', default='')
    parser.add_argument('--camera-params', default='')
    parser.add_argument('--verbose', action='store_true')
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    configure_logging(args.verbose)
    runner = ColmapRunner(args)
    runner.run()


if __name__ == '__main__':
    main()
