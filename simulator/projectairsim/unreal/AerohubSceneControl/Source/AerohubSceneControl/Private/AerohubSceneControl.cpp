#include "AerohubSceneControl.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Components/SkeletalMeshComponent.h"
#include "LevelSequenceActor.h"
#include "LevelSequencePlayer.h"
#include "MovieSceneSequencePlayer.h"
#include "MovieSceneTimeController.h"
#include "LandscapeSubsystem.h"
#include "LandscapeProxy.h"
#include "LandscapeComponent.h"
#include "Materials/Material.h"
#include "Materials/MaterialInstanceConstant.h"
#include "Misc/Crc.h"
#include "core_sim/clock.hpp"
#include "HttpServerModule.h"
#include "HttpServerRequest.h"
#include "HttpServerResponse.h"
#include "IHttpRouter.h"
#include "HAL/PlatformMisc.h"
#include "Misc/FileHelper.h"
#include "Modules/ModuleManager.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

IMPLEMENT_MODULE(FDefaultModuleImpl, AerohubSceneControl)

namespace {
int64 SimNanos() { return microsoft::projectairsim::SimClock::Get()->NowSimNanos(); }
// Integral of smoothstep speed: exact even when render frames are irregular.
double RampDistance(double Seconds, double RampSeconds)
{
    if (RampSeconds <= 0. || Seconds >= RampSeconds) return Seconds - RampSeconds * .5;
    const double X = Seconds / RampSeconds;
    return RampSeconds * (X * X * X - .5 * X * X * X * X);
}
}

struct FRunnerSimTimeController : FMovieSceneTimeController
{
    double RampSeconds = 0., ElapsedSeconds = 0., OffsetSeconds = 0.;
    int64 LastNanos = 0;
    void BeginRun(double Ramp) { RampSeconds = Ramp; ElapsedSeconds = 0.; }
    virtual void OnStartPlaying(const FQualifiedFrameTime&) override
    {
        LastNanos = SimNanos(); OffsetSeconds = 0.;
    }
    virtual FFrameTime OnRequestCurrentTime(const FQualifiedFrameTime& Current, float) override
    {
        if (!GetPlaybackStartTime().IsSet()) return Current.Time;
        const int64 Now = SimNanos();
        if (Now < 0) { LastNanos = -1; return Current.Time; }
        // A scene reload can replace/reset SimClock. Rebase without moving backwards.
        const double Delta = LastNanos >= 0 && Now >= LastNanos ? double(Now - LastNanos) * 1.e-9 : 0.;
        LastNanos = Now;
        OffsetSeconds += RampDistance(ElapsedSeconds + Delta, RampSeconds)
            - RampDistance(ElapsedSeconds, RampSeconds);
        ElapsedSeconds += Delta;
        return GetPlaybackStartTime()->ConvertTo(Current.Rate) + OffsetSeconds * Current.Rate;
    }
};

int32 UAerohubSceneControlWorld::OutdatedLandscapeMaterials(UObject* WorldContext)
{
#if WITH_EDITOR
    auto* World = WorldContext ? WorldContext->GetWorld() : nullptr;
    auto* Landscape = World ? World->GetSubsystem<ULandscapeSubsystem>() : nullptr;
    return Landscape ? Landscape->GetOudatedPhysicalMaterialComponentsCount() : -1;
#else
    return -1;
#endif
}
int32 UAerohubSceneControlWorld::RebuildLandscape(UObject* WorldContext)
{
#if WITH_EDITOR
    auto* World = WorldContext ? WorldContext->GetWorld() : nullptr;
    if (auto* Landscape = World ? World->GetSubsystem<ULandscapeSubsystem>() : nullptr) Landscape->BuildPhysicalMaterial();
#endif
    return OutdatedLandscapeMaterials(WorldContext);
}

void UAerohubSceneControlWorld::InitializeLandscapeSavedHashes()
{
#if WITH_EDITOR && ENGINE_MAJOR_VERSION == 5 && ENGINE_MINOR_VERSION == 2
    // UE5.2 initializes this transient saved-hash field only when GIsEditor is
    // true (Landscape.cpp, ULandscapeComponent::PostLoad). UnrealEditor -game
    // skips it and reports every clean, already-built component as unsaved.
    // Restore only validated cached hashes, never acknowledge stale build data.
    if (!GIsEditor && GetWorld()->IsGameWorld()) {
        for (TActorIterator<ALandscapeProxy> It(GetWorld()); It; ++It) {
            for (ULandscapeComponent* Component : It->LandscapeComponents) {
                if (!Component || Component->LastSavedPhysicalMaterialHash != 0
                    || Component->GetPackage()->IsDirty() || !Component->GetCollisionComponent()) continue;
                uint32 Hash = 0;
                UMaterialInterface* Material = Component->GetLandscapeMaterial();
                for (auto* Instance = Cast<UMaterialInstanceConstant>(Material); Instance;
                     Instance = Cast<UMaterialInstanceConstant>(Material)) {
                    Hash = FCrc::TypeCrc32(Instance->ParameterStateId, Hash);
                    Material = Instance->Parent;
                }
                auto* Base = Cast<UMaterial>(Material);
                if (!Base) continue;
                Hash = FCrc::TypeCrc32(Base->StateId, Hash);
                if (Hash != 0 && Component->PhysicalMaterialHash == Hash) {
                    Component->LastSavedPhysicalMaterialHash = Hash;
                    ++RestoredLandscapeHashes;
                }
            }
        }
        UE_LOG(LogTemp, Display, TEXT("Scene control restored %d validated UE5.2 landscape saved-hash caches"), RestoredLandscapeHashes);
    }
#endif
}

bool UAerohubSceneControlWorld::DoesSupportWorldType(EWorldType::Type Type) const
{
    return Type == EWorldType::Game || Type == EWorldType::PIE;
}
TStatId UAerohubSceneControlWorld::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(AerohubSceneControlWorld, STATGROUP_Tickables);
}
bool UAerohubSceneControlWorld::FindTarget()
{
    AActor* FoundRunner = nullptr;
    ALevelSequenceActor* FoundSequence = nullptr;
    int32 Runners = 0, Sequences = 0;
    for (TActorIterator<AActor> It(GetWorld()); It; ++It) {
        if (It->IsActorBeingDestroyed()) continue;
        if (It->ActorHasTag(TEXT("WestRunner"))) { FoundRunner = *It; ++Runners; }
        if (It->ActorHasTag(TEXT("WestRunnerSequence"))) {
            if (auto* Actor = Cast<ALevelSequenceActor>(*It)) { FoundSequence = Actor; ++Sequences; }
        }
    }
    if (Runners != 1 || Sequences != 1 || !FoundSequence->GetSequencePlayer()) {
        Runner.Reset(); Sequence.Reset(); bTargetInitialized = false; bHavePreviousPosition = false;
        Velocity = FVector::ZeroVector;
        return false;
    }
    if (Runner.Get() != FoundRunner || Sequence.Get() != FoundSequence) bTargetInitialized = false;
    Runner = FoundRunner; Sequence = FoundSequence;
    if (!bTargetInitialized) {
        auto* Player = FoundSequence->GetSequencePlayer();
        Player->Pause();
        SimTimeController = MakeShared<FRunnerSimTimeController>();
        Player->SetTimeController(SimTimeController);
        Player->SetPlayRate(1.f); bStarting = false; StartupElapsedSeconds = 0.f;
        Player->SetPlaybackPosition(FMovieSceneSequencePlaybackParams(0.f, EUpdatePositionMethod::Jump));
        Freeze(true);
        bTargetInitialized = true; bHavePreviousPosition = false; LastSimNanos = -1;
    }
    return true;
}
void UAerohubSceneControlWorld::Freeze(bool bFreeze)
{
    if (!Runner.IsValid()) return;
    TInlineComponentArray<USkeletalMeshComponent*> Meshes;
    Runner->GetComponents(Meshes);
    for (auto* Mesh : Meshes) Mesh->bPauseAnims = bFreeze;
}
TSharedPtr<FJsonObject> UAerohubSceneControlWorld::State() const
{
    auto Data = MakeShared<FJsonObject>();
    Data->SetStringField(TEXT("actor_id"), TEXT("runner"));
    Data->SetStringField(TEXT("clock_source"), TEXT("projectairsim_sim_clock"));
    const int64 Now = SimNanos();
    Data->SetBoolField(TEXT("clock_available"), Now >= 0);
    Data->SetNumberField(TEXT("sim_time_seconds"), Now >= 0 ? double(Now) * 1.e-9 : -1.);
    Data->SetNumberField(TEXT("landscape_saved_hashes_restored"), RestoredLandscapeHashes);
#if WITH_EDITOR
    Data->SetNumberField(TEXT("landscape_materials_outdated"), GetWorld()->GetSubsystem<ULandscapeSubsystem>()->GetOudatedPhysicalMaterialComponentsCount());
#endif
    const bool bAvailable = Runner.IsValid() && Sequence.IsValid() && Sequence->GetSequencePlayer();
    Data->SetBoolField(TEXT("available"), bAvailable);
    Data->SetStringField(TEXT("state"), !bAvailable ? TEXT("unavailable")
        : Sequence->GetSequencePlayer()->IsPlaying() ? TEXT("running") : TEXT("paused"));
    if (bAvailable) {
        auto* Player = Sequence->GetSequencePlayer();
        Data->SetNumberField(TEXT("play_rate"), Player->GetPlayRate());
        Data->SetNumberField(TEXT("startup_ramp_seconds"), StartupRampSeconds);
        Data->SetBoolField(TEXT("starting"), bStarting);
        Data->SetNumberField(TEXT("time_seconds"), Player->GetCurrentTime().AsSeconds());
        Data->SetNumberField(TEXT("duration_seconds"), Player->GetDuration().AsSeconds());
        const auto Location = Runner->GetActorLocation();
        auto Position = MakeShared<FJsonObject>();
        Position->SetNumberField(TEXT("x"), Location.X); Position->SetNumberField(TEXT("y"), Location.Y); Position->SetNumberField(TEXT("z"), Location.Z);
        Data->SetObjectField(TEXT("location_ue_cm"), Position);
        Data->SetNumberField(TEXT("speed_mps"), Velocity.Size()/100.0);
    }
    return Data;
}
void UAerohubSceneControlWorld::InitializeRouter()
{
    bRouterInitialized = true;
    FString Text;
    const FString Config = FPlatformMisc::GetEnvironmentVariable(TEXT("AEROHUB_SCENE_CONTROL_CONFIG"));
    TSharedPtr<FJsonObject> Settings;
    if (!Config.IsEmpty() && FFileHelper::LoadFileToString(Text, *Config)
        && FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Settings) && Settings.IsValid()) {
        double Value;
        if (Settings->TryGetNumberField(TEXT("startup_ramp_seconds"), Value)
            && FMath::IsFinite(Value) && Value >= 0. && Value <= 10.) StartupRampSeconds = float(Value);
        if (Settings->TryGetNumberField(TEXT("port"), Value) && Value >= 1 && Value <= 65535) Port = int32(Value);
    }
    Router = FHttpServerModule::Get().GetHttpRouter(Port, true);
    if (!Router.IsValid()) { UE_LOG(LogTemp, Error, TEXT("Scene control cannot bind %d"), Port); return; }
    Route = Router->BindRoute(FHttpPath(TEXT("/aerohub/scene/rpc")), EHttpServerRequestVerbs::VERB_POST,
        [this](const FHttpServerRequest& Request, const FHttpResultCallback& Done) {
            TSharedPtr<FJsonObject> Input;
            auto Output = MakeShared<FJsonObject>(); Output->SetStringField(TEXT("jsonrpc"), TEXT("2.0"));
            FString Error, Method, Version, ActorId, Event;
            int32 Code = -32602;
            const FUTF8ToTCHAR Converted(reinterpret_cast<const ANSICHAR*>(Request.Body.GetData()), Request.Body.Num());
            const FString Body(Converted.Length(), Converted.Get());
            const TSharedPtr<FJsonObject>* Params = nullptr;
            if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Body), Input) || !Input.IsValid()) {
                Error = TEXT("Parse error"); Code = -32700;
            } else {
                Output->SetField(TEXT("id"), Input->HasField(TEXT("id")) ? Input->Values[TEXT("id")] : MakeShared<FJsonValueNull>());
                if (!Input->HasField(TEXT("id")) || !Input->TryGetStringField(TEXT("jsonrpc"), Version) || Version != TEXT("2.0")
                    || !Input->TryGetStringField(TEXT("method"), Method)) { Error = TEXT("Invalid Request"); Code = -32600; }
                else if (Method != TEXT("GetActorState") && Method != TEXT("ControlActor")) { Error = TEXT("Unknown method"); Code = -32601; }
                else if (!Input->TryGetObjectField(TEXT("params"), Params) || !(*Params)->TryGetStringField(TEXT("actor_id"), ActorId)
                    || ActorId != TEXT("runner")) Error = TEXT("Expected actor_id runner");
                else if (Method == TEXT("ControlActor") && (!(*Params)->TryGetStringField(TEXT("event"), Event)
                    || (Event != TEXT("start") && Event != TEXT("stop") && Event != TEXT("reset")))) Error = TEXT("Expected start/stop/reset");
                else if (!FindTarget()) { Error = TEXT("Actor unavailable or tag is ambiguous"); Code = -32001; }
                else {
                    auto* Player = Sequence->GetSequencePlayer();
                    if (Method == TEXT("ControlActor")) {
                        if (Event == TEXT("start")) {
                            if (!Player->IsPlaying()) {
                                StartupElapsedSeconds = 0.f;
                                bStarting = StartupRampSeconds > 0.f;
                                SimTimeController->BeginRun(StartupRampSeconds);
                                LastSimNanos = SimNanos(); bHavePreviousPosition = false;
                                Player->SetPlayRate(bStarting ? 0.f : 1.f);
                                Freeze(false); Player->Play();
                            }
                        }
                        else {
                            Player->Pause();
                            bStarting = false; StartupElapsedSeconds = 0.f; Player->SetPlayRate(1.f);
                            if (Event == TEXT("reset")) Player->SetPlaybackPosition(FMovieSceneSequencePlaybackParams(0.f, EUpdatePositionMethod::Jump));
                            Freeze(true); Velocity = FVector::ZeroVector; bHavePreviousPosition = false;
                        }
                    }
                    Output->SetObjectField(TEXT("result"), State());
                }
            }
            if (!Error.IsEmpty()) {
                auto Data = MakeShared<FJsonObject>(); Data->SetNumberField(TEXT("code"), Code); Data->SetStringField(TEXT("message"), Error); Output->SetObjectField(TEXT("error"), Data);
            }
            FString Response; FJsonSerializer::Serialize(Output, TJsonWriterFactory<>::Create(&Response));
            Done(FHttpServerResponse::Create(Response, TEXT("application/json"))); return true;
        });
    FHttpServerModule::Get().StartAllListeners();
}
void UAerohubSceneControlWorld::Tick(float DeltaSeconds)
{
    if (!GetWorld()->HasBegunPlay()) return;
    if (!bLandscapeInitialized) { InitializeLandscapeSavedHashes(); bLandscapeInitialized = true; }
    if (!bRouterInitialized) InitializeRouter();
    if (FindTarget()) {
        const int64 Now = SimNanos();
        const double SimDelta = LastSimNanos >= 0 && Now >= LastSimNanos
            ? double(Now - LastSimNanos) * 1.e-9 : 0.;
        const bool bClockReset = LastSimNanos >= 0 && Now < LastSimNanos;
        LastSimNanos = Now;
        if (bClockReset) bHavePreviousPosition = false;
        auto* Player = Sequence->GetSequencePlayer();
        if (bStarting && Player->IsPlaying()) {
            StartupElapsedSeconds += SimDelta;
            const float Alpha = FMath::Clamp(StartupElapsedSeconds / StartupRampSeconds, 0.f, 1.f);
            // Smoothstep: zero acceleration at both ends, unchanged route and loop.
            Player->SetPlayRate(Alpha * Alpha * (3.f - 2.f * Alpha));
            if (Alpha >= 1.f) bStarting = false;
        }
        const auto Position = Runner->GetActorLocation();
        Velocity = bHavePreviousPosition && SimDelta > SMALL_NUMBER && Player->IsPlaying()
            ? (Position-PreviousPosition)/SimDelta : FVector::ZeroVector;
        PreviousPosition = Position; bHavePreviousPosition = true;
    }
}
void UAerohubSceneControlWorld::Deinitialize()
{
    if (Router.IsValid() && Route.IsValid()) Router->UnbindRoute(Route);
    Route.Reset(); Router.Reset(); Super::Deinitialize();
}
