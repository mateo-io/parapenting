"""Report the controls exposed by the bundled Mannequin Control Rig."""

import unreal


rig = unreal.load_asset("/Game/Characters/Mannequins/Rigs/CR_Mannequin_Body")
if not rig:
    raise RuntimeError("CR_Mannequin_Body is missing")

unreal.log(f"Control Rig: {rig.get_path_name()}")
hierarchy = rig.get_hierarchy()
for key in hierarchy.get_all_keys():
    if key.type != unreal.RigElementType.CONTROL:
        continue
    unreal.log(f"CONTROL {key.name}")
