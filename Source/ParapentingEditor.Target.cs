using UnrealBuildTool;

public class ParapentingEditorTarget : TargetRules
{
    public ParapentingEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V7;
        IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
        ExtraModuleNames.Add("Parapenting");
    }
}
