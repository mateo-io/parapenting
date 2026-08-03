"""Create the checked-in Level 1 material library in Unreal Editor.

Run from the project root with:
  UnrealEditor-Cmd Parapenting.uproject \
    -run=pythonscript -script=Tools/Visual/create_level1_materials.py \
    -unattended -nop4

The script is intentionally idempotent. It updates the small set of authored
material assets instead of relying on opaque manual editor work.
"""

import unreal


ROOT = "/Game/Materials"


def load_or_create(name, asset_class, factory):
    path = f"{ROOT}/{name}"
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        return unreal.EditorAssetLibrary.load_asset(path), False
    return (
        unreal.AssetToolsHelpers.get_asset_tools().create_asset(
            name, ROOT, asset_class, factory
        ),
        True,
    )


def scalar(material, name, value, x, y):
    node = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionScalarParameter, x, y
    )
    node.set_editor_property("parameter_name", name)
    node.set_editor_property("default_value", value)
    return node


def vector(material, name, value, x, y):
    node = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionVectorParameter, x, y
    )
    node.set_editor_property("parameter_name", name)
    node.set_editor_property("default_value", value)
    return node


def connect(node, output, material, prop):
    unreal.MaterialEditingLibrary.connect_material_property(node, output, prop)


def make_vertex_lit():
    material, created = load_or_create(
        "M_VertexLit", unreal.Material, unreal.MaterialFactoryNew()
    )
    if not created:
        return material
    material.set_editor_property("two_sided", True)
    material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)

    color = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionVertexColor, -420, -80
    )
    roughness = scalar(material, "Roughness", 0.92, -420, 100)
    metallic = scalar(material, "Metallic", 0.0, -420, 180)
    connect(color, "RGB", material, unreal.MaterialProperty.MP_BASE_COLOR)
    connect(roughness, "", material, unreal.MaterialProperty.MP_ROUGHNESS)
    connect(metallic, "", material, unreal.MaterialProperty.MP_METALLIC)
    unreal.MaterialEditingLibrary.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material, False)
    return material


def make_error_material():
    material, created = load_or_create(
        "M_VisualError", unreal.Material, unreal.MaterialFactoryNew()
    )
    if not created:
        return material
    material.set_editor_property("two_sided", True)
    magenta = vector(
        material, "ErrorColor", unreal.LinearColor(1.0, 0.0, 0.72, 1.0),
        -420, -80
    )
    roughness = scalar(material, "Roughness", 0.35, -420, 100)
    connect(magenta, "RGB", material, unreal.MaterialProperty.MP_BASE_COLOR)
    connect(magenta, "RGB", material, unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    connect(roughness, "", material, unreal.MaterialProperty.MP_ROUGHNESS)
    unreal.MaterialEditingLibrary.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material, False)
    return material


def make_surface_master():
    material, created = load_or_create(
        "M_SurfaceMaster", unreal.Material, unreal.MaterialFactoryNew()
    )
    if not created:
        return material
    material.set_editor_property("two_sided", False)

    base = vector(
        material, "BaseColor", unreal.LinearColor(0.18, 0.20, 0.16, 1.0),
        -700, -180
    )
    tint = vector(
        material, "MacroTint", unreal.LinearColor(0.88, 0.94, 0.84, 1.0),
        -700, -80
    )
    multiply = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionMultiply, -380, -140
    )
    unreal.MaterialEditingLibrary.connect_material_expressions(
        base, "RGB", multiply, "A"
    )
    unreal.MaterialEditingLibrary.connect_material_expressions(
        tint, "RGB", multiply, "B"
    )

    roughness = scalar(material, "Roughness", 0.82, -700, 80)
    wetness = scalar(material, "Wetness", 0.0, -700, 160)
    wet_scale = scalar(material, "WetRoughnessReduction", 0.55, -700, 240)
    wet_multiply = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionMultiply, -420, 150
    )
    unreal.MaterialEditingLibrary.connect_material_expressions(
        wetness, "", wet_multiply, "A"
    )
    unreal.MaterialEditingLibrary.connect_material_expressions(
        wet_scale, "", wet_multiply, "B"
    )
    one_minus = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionOneMinus, -220, 150
    )
    unreal.MaterialEditingLibrary.connect_material_expressions(
        wet_multiply, "", one_minus, ""
    )
    final_roughness = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionMultiply, 0, 100
    )
    unreal.MaterialEditingLibrary.connect_material_expressions(
        roughness, "", final_roughness, "A"
    )
    unreal.MaterialEditingLibrary.connect_material_expressions(
        one_minus, "", final_roughness, "B"
    )

    metallic = scalar(material, "Metallic", 0.0, -700, 340)
    specular = scalar(material, "Specular", 0.5, -700, 420)
    # These parameters establish the shared contract for the later terrain and
    # fabric shaders. They are deliberately inert until those levels supply
    # textures/geometry; having named defaults prevents each asset family from
    # inventing incompatible units.
    scalar(material, "DetailNormalStrength", 1.0, -700, 520)
    scalar(material, "WorldTextureScaleM", 1.0, -700, 600)
    scalar(material, "SnowAmount", 0.0, -700, 680)
    scalar(material, "WindResponse", 0.0, -700, 760)
    scalar(material, "DebugOverride", 0.0, -700, 840)

    connect(multiply, "", material, unreal.MaterialProperty.MP_BASE_COLOR)
    connect(
        final_roughness, "", material, unreal.MaterialProperty.MP_ROUGHNESS
    )
    connect(metallic, "", material, unreal.MaterialProperty.MP_METALLIC)
    connect(specular, "", material, unreal.MaterialProperty.MP_SPECULAR)
    unreal.MaterialEditingLibrary.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material, False)
    return material


SWATCHES = {
    "MI_Grass": ((0.09, 0.24, 0.045, 1.0), 0.92, 0.0),
    "MI_Soil": ((0.16, 0.085, 0.035, 1.0), 0.96, 0.0),
    "MI_Limestone": ((0.34, 0.33, 0.30, 1.0), 0.88, 0.0),
    "MI_Snow": ((0.78, 0.82, 0.86, 1.0), 0.72, 0.0),
    "MI_Water": ((0.018, 0.12, 0.16, 1.0), 0.08, 0.0),
    "MI_RipstopNylon": ((0.86, 0.12, 0.035, 1.0), 0.58, 0.0),
    "MI_Webbing": ((0.035, 0.038, 0.042, 1.0), 0.9, 0.0),
    "MI_Metal": ((0.32, 0.34, 0.36, 1.0), 0.3, 1.0),
    "MI_Clothing": ((0.055, 0.075, 0.10, 1.0), 0.86, 0.0),
}


def make_swatches(parent):
    factory = unreal.MaterialInstanceConstantFactoryNew()
    for name, (color, roughness, metallic) in SWATCHES.items():
        instance, _ = load_or_create(
            name, unreal.MaterialInstanceConstant, factory
        )
        instance.set_editor_property("parent", parent)
        unreal.MaterialEditingLibrary.set_material_instance_vector_parameter_value(
            instance, "BaseColor", unreal.LinearColor(*color)
        )
        unreal.MaterialEditingLibrary.set_material_instance_vector_parameter_value(
            instance, "MacroTint", unreal.LinearColor(1.0, 1.0, 1.0, 1.0)
        )
        unreal.MaterialEditingLibrary.set_material_instance_scalar_parameter_value(
            instance, "Roughness", roughness
        )
        unreal.MaterialEditingLibrary.set_material_instance_scalar_parameter_value(
            instance, "Metallic", metallic
        )
        unreal.EditorAssetLibrary.save_loaded_asset(instance, False)


unreal.EditorAssetLibrary.make_directory(ROOT)
make_vertex_lit()
make_error_material()
surface = make_surface_master()
make_swatches(surface)
unreal.EditorAssetLibrary.save_directory(ROOT, False, True)
unreal.log("Parapenting Level 1 material library created successfully")
