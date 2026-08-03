"""Delete only M_TerrainLit before regenerating its material graph.

Run this in one Unreal Python commandlet process, then run
create_level1_materials.py in a fresh process. Unreal cannot safely remove
expressions from a loaded material graph in place.
"""

import unreal


TERRAIN_MATERIAL = "/Game/Materials/M_TerrainLit"

if unreal.EditorAssetLibrary.does_asset_exist(TERRAIN_MATERIAL):
    if not unreal.EditorAssetLibrary.delete_asset(TERRAIN_MATERIAL):
        raise RuntimeError(f"Could not delete {TERRAIN_MATERIAL}")

unreal.log("Parapenting Level 1 terrain material reset successfully")
