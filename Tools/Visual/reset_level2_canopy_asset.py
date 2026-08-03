"""Delete only the generated canopy material before a deliberate rebuild."""

import unreal


ASSET = "/Game/Materials/M_CanopyFabric"

if unreal.EditorAssetLibrary.does_asset_exist(ASSET):
    if not unreal.EditorAssetLibrary.delete_asset(ASSET):
        raise RuntimeError(f"Could not delete {ASSET}")
