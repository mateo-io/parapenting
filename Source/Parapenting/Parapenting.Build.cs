using UnrealBuildTool;

public class Parapenting : ModuleRules
{
    public Parapenting(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        CppStandard = CppStandardVersion.Cpp20;
        // The Physics/ sources are self-contained translation units that each
        // keep their own helpers in an anonymous namespace: Profiles,
        // SmoothStep01, Clamp, RadToDeg and friends recur across several
        // files. That is legal per translation unit, but unity builds
        // concatenate them into one file where the names collide and the
        // module stops compiling entirely. Build them separately.
        bUseUnity = false;
        PublicDependencyModuleNames.AddRange(new[] {
            "Core", "CoreUObject", "Engine", "InputCore", "RenderCore",
            "ProceduralMeshComponent", "AudioMixer", "HTTP", "Json"
        });
    }
}
