using UnrealBuildTool;

public class Parapenting : ModuleRules
{
    public Parapenting(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        CppStandard = CppStandardVersion.Cpp20;
        PublicDependencyModuleNames.AddRange(new[] {
            "Core", "CoreUObject", "Engine", "InputCore", "RenderCore",
            "ProceduralMeshComponent", "AudioMixer", "HTTP", "Json"
        });
    }
}
