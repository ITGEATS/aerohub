using UnrealBuildTool;
public class AerohubSceneControl : ModuleRules
{
    public AerohubSceneControl(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine" });
        PrivateDependencyModuleNames.AddRange(new[] { "LevelSequence", "MovieScene", "HTTPServer", "Json" });
    }
}
