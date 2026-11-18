#pragma once
#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "HttpRouteHandle.h"
#include "AerohubSceneControl.generated.h"
class IHttpRouter;
class ALevelSequenceActor;
class ULevelSequencePlayer;
class FJsonObject;
struct FRunnerSimTimeController;

UCLASS()
class AEROHUBSCENECONTROL_API UAerohubSceneControlWorld : public UTickableWorldSubsystem
{
    GENERATED_BODY()
public:
    virtual bool DoesSupportWorldType(EWorldType::Type Type) const override;
    virtual void Tick(float DeltaSeconds) override;
    virtual TStatId GetStatId() const override;
    virtual void Deinitialize() override;
    // Editor utility: build real landscape data, then report remaining stale components.
    UFUNCTION(BlueprintCallable, Category="Aerohub|Scene", meta=(WorldContext="WorldContext"))
    static int32 RebuildLandscape(UObject* WorldContext);
    UFUNCTION(BlueprintCallable, Category="Aerohub|Scene", meta=(WorldContext="WorldContext"))
    static int32 OutdatedLandscapeMaterials(UObject* WorldContext);
private:
    bool FindTarget();
    void Freeze(bool bFreeze);
    void InitializeRouter();
    void InitializeLandscapeSavedHashes();
    TSharedPtr<FJsonObject> State() const;
    TWeakObjectPtr<AActor> Runner;
    TWeakObjectPtr<ALevelSequenceActor> Sequence;
    bool bRouterInitialized = false;
    bool bTargetInitialized = false;
    bool bHavePreviousPosition = false;
    float StartupRampSeconds = 1.f;
    float StartupElapsedSeconds = 0.f;
    bool bStarting = false;
    bool bLandscapeInitialized = false;
    int32 RestoredLandscapeHashes = 0;
    int64 LastSimNanos = -1;
    TSharedPtr<FRunnerSimTimeController> SimTimeController;
    FVector PreviousPosition = FVector::ZeroVector;
    FVector Velocity = FVector::ZeroVector;
    int32 Port = 19892;
    TSharedPtr<IHttpRouter> Router;
    FHttpRouteHandle Route;
};
