using UnrealBuildTool;
using System.IO;
public class AerohubSceneControl : ModuleRules
{
    public AerohubSceneControl(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine" });
        PrivateDependencyModuleNames.AddRange(new[] { "LevelSequence", "MovieScene", "HTTPServer", "Json", "Landscape" });
        string SimPlugin = Path.GetFullPath(Path.Combine(GetModuleDirectory("ProjectAirSim"), "../.."));
        PrivateIncludePaths.Add(Path.Combine(SimPlugin, "SimLibs/core_sim/include"));
        if (Target.Type == TargetType.Editor && Target.Platform == UnrealTargetPlatform.Linux)
        {
            // A module dependency also propagates libcore_sim.a into this DSO,
            // creating a separate SimClock singleton. Import the existing DSO
            // instead. The uplugin declares its load dependency on ProjectAirSim.
            PublicAdditionalLibraries.Add(Path.Combine(SimPlugin, "Binaries/Linux/libUnrealEditor-ProjectAirSim.so"));
        }
        else
        {
            if (Target.Type == TargetType.Editor)
                throw new BuildException("AerohubSceneControl's modular SimClock bridge is currently validated for Linux only.");
            // Packaged Linux games link modules into a single executable.
            PrivateDependencyModuleNames.Add("ProjectAirSim");
        }
    }
}
