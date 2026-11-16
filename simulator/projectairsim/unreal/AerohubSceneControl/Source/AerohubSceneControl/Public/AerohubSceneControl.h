#pragma once
#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "HttpRouteHandle.h"
#include "AerohubSceneControl.generated.h"
class IHttpRouter;
class ALevelSequenceActor;
class ULevelSequencePlayer;
class FJsonObject;

UCLASS()
class AEROHUBSCENECONTROL_API UAerohubSceneControlWorld : public UTickableWorldSubsystem
{
    GENERATED_BODY()
public:
    virtual bool DoesSupportWorldType(EWorldType::Type Type) const override;
    virtual void Tick(float DeltaSeconds) override;
    virtual TStatId GetStatId() const override;
    virtual void Deinitialize() override;
private:
    bool FindTarget();
    void Freeze(bool bFreeze);
    void InitializeRouter();
    TSharedPtr<FJsonObject> State() const;
    TWeakObjectPtr<AActor> Runner;
    TWeakObjectPtr<ALevelSequenceActor> Sequence;
    bool bRouterInitialized = false;
    bool bTargetInitialized = false;
    bool bHavePreviousPosition = false;
    FVector PreviousPosition = FVector::ZeroVector;
    FVector Velocity = FVector::ZeroVector;
    int32 Port = 19892;
    TSharedPtr<IHttpRouter> Router;
    FHttpRouteHandle Route;
};
