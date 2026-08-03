"""Delete only the generated Level 1 surface library before regeneration.

Unreal 5.8 cannot safely rewrite a loaded material graph from commandlet mode.
Run this commandlet, close it, then run create_level1_materials.py in a fresh
commandlet process. M_VertexLit is intentionally excluded because the startup
world loads it before Python executes.
"""

import unreal


ROOT = "/Game/Materials"
GENERATED_SURFACES = [
    "MI_Clothing",
    "MI_Grass",
    "MI_Limestone",
    "MI_Metal",
    "MI_RipstopNylon",
    "MI_Snow",
    "MI_Soil",
    "MI_Water",
    "MI_Webbing",
    "M_SurfaceMaster",
    "M_VisualError",
]

for name in GENERATED_SURFACES:
    path = f"{ROOT}/{name}"
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        if not unreal.EditorAssetLibrary.delete_asset(path):
            raise RuntimeError(f"Could not delete generated asset {path}")

unreal.log("Parapenting Level 1 generated surface assets reset successfully")
