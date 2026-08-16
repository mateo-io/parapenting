"""Build the project-owned technical nylon material for the pilot harness.

Run with UnrealEditor-Cmd from the project root. The source bitmap is checked
in so the imported texture and cooked material are reproducible on a clean
machine.
"""

import os

import unreal


ROOT = "/Game/Materials"
TEXTURE_PATH = f"{ROOT}/Textures/T_HarnessNylon_Albedo"
MATERIAL_PATH = f"{ROOT}/M_HarnessFabric"
SOURCE = os.path.join(
    unreal.Paths.project_dir(),
    "Content/ArtSource/Harness/T_HarnessNylon_Albedo_Source_v1.png",
)


def connect(source, output, target, target_pin):
    unreal.MaterialEditingLibrary.connect_material_expressions(
        source, output, target, target_pin
    )


def scalar(material, name, value, x, y):
    node = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionScalarParameter, x, y
    )
    node.set_editor_property("parameter_name", name)
    node.set_editor_property("default_value", value)
    return node


def import_texture():
    texture = unreal.EditorAssetLibrary.load_asset(TEXTURE_PATH)
    if texture:
        return texture
    if not unreal.Paths.file_exists(SOURCE):
        raise RuntimeError(f"Harness source is missing: {SOURCE}")
    unreal.EditorAssetLibrary.make_directory(f"{ROOT}/Textures")
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", SOURCE)
    task.set_editor_property("destination_path", f"{ROOT}/Textures")
    task.set_editor_property("destination_name", "T_HarnessNylon_Albedo")
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", False)
    task.set_editor_property("save", True)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    texture = unreal.EditorAssetLibrary.load_asset(TEXTURE_PATH)
    if not texture:
        raise RuntimeError("Could not import harness nylon texture")
    texture.set_editor_property("srgb", True)
    unreal.EditorAssetLibrary.save_loaded_asset(texture, False)
    return texture


texture = import_texture()
material = unreal.EditorAssetLibrary.load_asset(MATERIAL_PATH)
if material:
    unreal.EditorAssetLibrary.delete_asset(MATERIAL_PATH)
material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
    "M_HarnessFabric", ROOT, unreal.Material, unreal.MaterialFactoryNew()
)
material.set_editor_property("two_sided", True)
material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)

uv = unreal.MaterialEditingLibrary.create_material_expression(
    material, unreal.MaterialExpressionTextureCoordinate, -500, 0
)
uv.set_editor_property("u_tiling", 3.5)
uv.set_editor_property("v_tiling", 3.5)
sample = unreal.MaterialEditingLibrary.create_material_expression(
    material, unreal.MaterialExpressionTextureSampleParameter2D, -270, 0
)
sample.set_editor_property("parameter_name", "NylonAlbedo")
sample.set_editor_property("texture", texture)
connect(uv, "", sample, "UVs")

tint = unreal.MaterialEditingLibrary.create_material_expression(
    material, unreal.MaterialExpressionVectorParameter, -270, 160
)
tint.set_editor_property("parameter_name", "NylonTint")
tint.set_editor_property("default_value", unreal.LinearColor(0.40, 0.50, 0.64, 1))
base_color = unreal.MaterialEditingLibrary.create_material_expression(
    material, unreal.MaterialExpressionMultiply, 0, 0
)
connect(sample, "RGB", base_color, "A")
connect(tint, "RGB", base_color, "B")
unreal.MaterialEditingLibrary.connect_material_property(
    base_color, "", unreal.MaterialProperty.MP_BASE_COLOR
)
roughness = scalar(material, "NylonRoughness", 0.78, 0, 150)
unreal.MaterialEditingLibrary.connect_material_property(
    roughness, "", unreal.MaterialProperty.MP_ROUGHNESS
)
metallic = scalar(material, "Metallic", 0.0, 0, 220)
unreal.MaterialEditingLibrary.connect_material_property(
    metallic, "", unreal.MaterialProperty.MP_METALLIC
)
unreal.MaterialEditingLibrary.recompile_material(material)
unreal.EditorAssetLibrary.save_loaded_asset(material, False)
print("Built M_HarnessFabric with project-owned technical nylon source")
