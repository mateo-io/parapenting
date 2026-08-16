"""Register the bundled mannequin bridge after it is copied into Content/.

The UE template supplies a licensed, fully rigged pilot bridge while the
MetaHuman body is being imported. Files copied into Content outside the editor
need an explicit Asset Registry scan before a commandlet/game process can
resolve their /Game object paths.
"""

import unreal


ROOT = "/Game/Characters/Mannequins"
MESH = f"{ROOT}/Meshes/SK_Mannequin.SK_Mannequin"
MESH_FILE = unreal.Paths.convert_relative_path_to_full(
    unreal.Paths.project_content_dir()
    + "Characters/Mannequins/Meshes/SK_Mannequin.uasset"
)

registry = unreal.AssetRegistryHelpers.get_asset_registry()
registry.scan_paths_synchronous([ROOT], True)
registry.scan_files_synchronous([MESH_FILE], True)
if not unreal.EditorAssetLibrary.does_asset_exist(MESH):
    raise RuntimeError(
        f"Asset Registry did not discover copied mannequin mesh: {MESH_FILE}"
    )
mesh = unreal.load_object(None, MESH)
if mesh is None:
    raise RuntimeError(f"Could not register mannequin mesh: {MESH}")

unreal.log(f"Parapenting mannequin bridge registered: {mesh.get_path_name()}")
