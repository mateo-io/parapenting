"""Report exposed parameters on the bundled Manny material instance."""

import unreal


material = unreal.load_asset(
    "/Game/Characters/Mannequins/Materials/Manny/MI_Manny_01_New")
if not material:
    raise RuntimeError("MI_Manny_01_New is missing")

for getter in (
    unreal.MaterialEditingLibrary.get_scalar_parameter_names,
    unreal.MaterialEditingLibrary.get_vector_parameter_names,
    unreal.MaterialEditingLibrary.get_texture_parameter_names,
):
    try:
        unreal.log(f"MANNY PARAMS {getter.__name__}: {getter(material)}")
    except Exception as error:
        unreal.log(f"MANNY PARAMS unavailable: {error}")
