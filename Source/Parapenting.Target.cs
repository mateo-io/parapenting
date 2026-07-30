using UnrealBuildTool;

public class ParapentingTarget : TargetRules
{
    public ParapentingTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V7;
        IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
        ExtraModuleNames.Add("Parapenting");
    }
}
