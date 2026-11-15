using UnrealBuildTool;
public class AerohubDisplay : ModuleRules
{
    public AerohubDisplay(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine" });
        PrivateDependencyModuleNames.AddRange(new[] { "InputCore", "HTTPServer", "Json", "JsonUtilities", "SlateCore", "Projects", "RenderCore" });
        RuntimeDependencies.Add("$(PluginDir)/Resources/Fonts/...", StagedFileType.NonUFS);
    }
}
