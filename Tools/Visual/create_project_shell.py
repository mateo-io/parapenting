"""Create the minimal checked-in flight map required by the visual plan.

The game world itself is spawned by AParapentingGameMode, so this map is
deliberately empty: it establishes a stable /Game entry point without copying
runtime actors into an authored level.
"""

import unreal


MAP_PATH = "/Game/Maps/L_FlightLab"


def main():
    unreal.EditorAssetLibrary.make_directory("/Game/Maps")
    if unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH):
        unreal.log("Project shell already exists: {}".format(MAP_PATH))
        return
    level_editor = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    if not level_editor.new_level(MAP_PATH):
        raise RuntimeError("Could not create {}".format(MAP_PATH))
    unreal.log("Created project shell: {}".format(MAP_PATH))


main()
