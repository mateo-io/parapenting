"""Rebuild only the generated canopy fabric material.

Kept separate from the full material-library generator so a fabric iteration
does not resave unrelated material instances.
"""

import runpy
import unreal


ASSET = "/Game/Materials/M_CanopyFabric"
SOURCE = "/Users/pachosky/projects/parapenting/Tools/Visual/create_level1_materials.py"

if unreal.EditorAssetLibrary.does_asset_exist(ASSET):
    if not unreal.EditorAssetLibrary.delete_asset(ASSET):
        raise RuntimeError(f"Could not delete {ASSET}")

definitions = runpy.run_path(SOURCE, run_name="parapenting_material_definitions")
definitions["make_canopy_fabric"]()
unreal.log("Parapenting canopy fabric material rebuilt successfully")
