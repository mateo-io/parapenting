"""Report poses available in the bundled Mannequin pose asset."""

import unreal


asset = unreal.load_asset("/Game/Characters/Mannequins/Rigs/PA_Mannequin")
if not asset:
    raise RuntimeError("PA_Mannequin is missing")

unreal.log(f"Pose Asset: {asset.get_path_name()}")
for property_name in ("pose_names", "pose_name_array"):
    try:
        poses = asset.get_editor_property(property_name)
        unreal.log(f"{property_name}: {poses}")
    except Exception as error:
        unreal.log(f"{property_name}: unavailable ({error})")
