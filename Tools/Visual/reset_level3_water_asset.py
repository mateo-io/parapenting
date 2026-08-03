"""Delete only the generated Level 3 water material before rebuilding it."""

import unreal


ASSET = "/Game/Materials/M_WaterSurface"

if unreal.EditorAssetLibrary.does_asset_exist(ASSET):
    if not unreal.EditorAssetLibrary.delete_asset(ASSET):
        raise RuntimeError(f"Could not delete {ASSET}")
