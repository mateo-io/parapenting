"""Rebuild only the generated air-mote material after graph changes."""

import runpy

import unreal


ASSET = "/Game/Materials/M_AirMotes"
SOURCE = "/Users/pachosky/projects/parapenting/Tools/Visual/create_level1_materials.py"

if unreal.EditorAssetLibrary.does_asset_exist(ASSET):
    if not unreal.EditorAssetLibrary.delete_asset(ASSET):
        raise RuntimeError(f"Could not delete {ASSET}")

definitions = runpy.run_path(SOURCE, run_name="parapenting_material_definitions")
definitions["make_air_motes"]()
unreal.log("Parapenting air-mote material rebuilt successfully")
