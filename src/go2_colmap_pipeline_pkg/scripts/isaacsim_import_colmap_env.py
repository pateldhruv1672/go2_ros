from __future__ import annotations

import argparse
import asyncio
from pathlib import Path

import omni.kit.asset_converter
import omni.usd
from pxr import Sdf, UsdGeom


async def convert_mesh(input_asset: str, output_usd: str) -> None:
    converter = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    context.ignore_animations = True
    context.ignore_camera = True
    context.ignore_light = True
    task = converter.create_converter_task(input_asset, output_usd, None, context)
    ok = await task.wait_until_finished()
    if not ok:
        raise RuntimeError(task.get_error_message())


def create_stage(asset_usd: str, stage_usd: str, env_prim_path: str) -> None:
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    world = stage.DefinePrim('/World', 'Xform')
    env = stage.DefinePrim(env_prim_path, 'Xform')
    env.GetReferences().AddReference(asset_usd)
    stage.SetDefaultPrim(world)
    stage.GetRootLayer().Export(stage_usd)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Convert a COLMAP mesh into USD and build an Isaac Sim stage.')
    parser.add_argument('--input-asset', required=True, help='OBJ, FBX, STL, glTF, or another mesh format supported by the asset converter.')
    parser.add_argument('--converted-usd', required=True, help='Path to the converted asset USD file.')
    parser.add_argument('--stage-usd', required=True, help='Path to the top-level stage USD file.')
    parser.add_argument('--env-prim-path', default='/World/Environment')
    return parser.parse_args()


async def main_async() -> None:
    args = parse_args()
    input_asset = str(Path(args.input_asset).resolve())
    converted_usd = str(Path(args.converted_usd).resolve())
    stage_usd = str(Path(args.stage_usd).resolve())

    await convert_mesh(input_asset, converted_usd)
    create_stage(converted_usd, stage_usd, args.env_prim_path)
    print(f'Wrote converted USD to {converted_usd}')
    print(f'Wrote stage USD to {stage_usd}')


def main() -> None:
    asyncio.get_event_loop().run_until_complete(main_async())


if __name__ == '__main__':
    main()
