#include "AerohubDisplay.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/Canvas.h"
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/InputComponent.h"
#include "GameFramework/PlayerInput.h"
#include "Engine/Font.h"
#include "CanvasItem.h"
#include "GlobalRenderResources.h"
#include "Interfaces/IPluginManager.h"
#include "Engine/Engine.h"
#include "Engine/TextureRenderTarget2D.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "HttpServerModule.h"
#include "HttpServerRequest.h"
#include "HttpServerResponse.h"
#include "IHttpRouter.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformMisc.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "UObject/UObjectIterator.h"

IMPLEMENT_MODULE(FDefaultModuleImpl, AerohubDisplay)

bool UAerohubDisplayWorld::DoesSupportWorldType(EWorldType::Type Type) const
{
    return Type == EWorldType::Game || Type == EWorldType::PIE;
}
TStatId UAerohubDisplayWorld::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(AerohubDisplayWorld, STATGROUP_Tickables);
}
void UAerohubDisplayWorld::Tick(float DeltaSeconds)
{
    if (!GetWorld()->HasBegunPlay()) return;
    APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (PC && PC->IsLocalController() && !Cast<AAerohubHUD>(PC->GetHUD()))
        PC->ClientSetHUD(AAerohubHUD::StaticClass());
}

void AAerohubHUD::ReadConfig()
{
    FString Text;
    FString ConfigPath = FPlatformMisc::GetEnvironmentVariable(TEXT("AEROHUB_DISPLAY_CONFIG"));
    if (ConfigPath.IsEmpty()) ConfigPath = FPaths::ProjectSavedDir() / TEXT("Aerohub/display.json");
    if (!FFileHelper::LoadFileToString(Text, *ConfigPath)) return;
    TSharedPtr<FJsonObject> Data;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Data) || !Data.IsValid()) return;
    Data->TryGetBoolField(TEXT("enabled"), bEnabled);
    Data->TryGetBoolField(TEXT("camera_enabled"), bCameraEnabled);
    // Opt-in only: default Global remains the UAV-centroid overhead view.
    GlobalCameraTag.Empty();
    Data->TryGetStringField(TEXT("global_camera_tag"), GlobalCameraTag);
    double Width;
    if (Data->TryGetNumberField(TEXT("camera_width"), Width)) CameraWidth = FMath::Clamp(Width, 160.0, 640.0);
    double RequestedSpeedMax;
    if (Data->TryGetNumberField(TEXT("speed_max_mps"), RequestedSpeedMax))
        SpeedMaxMps = FMath::Clamp(RequestedSpeedMax, 1.0, 100.0);
    double RequestedPort;
    if (Data->TryGetNumberField(TEXT("port"), RequestedPort) && !Router.IsValid()) Port = int32(RequestedPort);
}

void AAerohubHUD::BeginPlay()
{
    Super::BeginPlay();
    PrimaryActorTick.bCanEverTick = true;
    const FString Fonts = IPluginManager::Get().FindPlugin(TEXT("AerohubDisplay"))->GetBaseDir() / TEXT("Resources/Fonts");
    auto MakeFont = [this, &Fonts](const TCHAR* Filename) {
        UFont* Result = NewObject<UFont>(this);
        Result->FontCacheType = EFontCacheType::Runtime;
        Result->LegacyFontSize = 24;
        Result->CompositeFont.DefaultTypeface.Fonts.Add(FTypefaceEntry(
            TEXT("Default"), Fonts / Filename, EFontHinting::Default, EFontLoadingPolicy::LazyLoad));
        return Result;
    };
    MonoRegular = MakeFont(TEXT("NimbusMonoPS-Regular.otf"));
    MonoBold = MakeFont(TEXT("NimbusMonoPS-Bold.otf"));
    TextFont = MakeFont(TEXT("NotoSansCJK-Regular.ttc"));
    TitleFont = MakeFont(TEXT("NotoSansCJK-Bold.ttc"));
    ReadConfig();
    ObservationCamera = GetWorld()->SpawnActor<ACameraActor>();
    if (APlayerController* PC = GetWorld()->GetFirstPlayerController()) {
        EnableInput(PC);
        InputComponent->Priority = 100;
        auto& Tab = InputComponent->BindKey(EKeys::Tab, IE_Pressed, this, &AAerohubHUD::NextView);
        Tab.bConsumeInput = true;
        auto& Global = InputComponent->BindKey(EKeys::G, IE_Pressed, this, &AAerohubHUD::ToggleGlobal);
        Global.bConsumeInput = true;
    }
    Router = FHttpServerModule::Get().GetHttpRouter(Port, true);
    if (!Router.IsValid()) {
        UE_LOG(LogTemp, Error, TEXT("Aerohub display control cannot bind port %d"), Port);
        return;
    }
    Routes.Add(Router->BindRoute(FHttpPath(TEXT("/aerohub/status")), EHttpServerRequestVerbs::VERB_GET,
        [this](const FHttpServerRequest&, const FHttpResultCallback& Done) {
            Done(FHttpServerResponse::Create(StatusJson(), TEXT("application/json")));
            return true;
        }));
    Routes.Add(Router->BindRoute(FHttpPath(TEXT("/aerohub/telemetry")), EHttpServerRequestVerbs::VERB_POST,
        [this](const FHttpServerRequest& Request, const FHttpResultCallback& Done) {
            const FUTF8ToTCHAR Converted(reinterpret_cast<const ANSICHAR*>(Request.Body.GetData()), Request.Body.Num());
            TSharedPtr<FJsonObject> Data;
            double Value;
            FString Vehicle;
            if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(FString(Converted.Length(), Converted.Get())), Data)
                || !Data.IsValid() || !Data->TryGetStringField(TEXT("vehicle"), Vehicle) || Vehicle.IsEmpty() || Vehicle == TEXT("__global__")
                || !Data->TryGetNumberField(TEXT("speed_mps"), Value) || !FMath::IsFinite(Value) || Value < 0) {
                auto Response = FHttpServerResponse::Create(TEXT("{\"error\":\"invalid telemetry\"}"), TEXT("application/json"));
                Response->Code = EHttpServerResponseCodes::BadRequest;
                Done(MoveTemp(Response)); return true;
            }
            auto& State = VehicleStates.FindOrAdd(Vehicle);
            State.Speed = Value; State.Peak = FMath::Max(State.Peak, Value);
            State.Received = FPlatformTime::Seconds();
            if (Vehicle == RobotName) {
                Speed = State.Speed; PeakSpeed = State.Peak; LastTelemetrySeconds = State.Received;
            }
            Done(FHttpServerResponse::Create(StatusJson(), TEXT("application/json"))); return true;
        }));
    Routes.Add(Router->BindRoute(FHttpPath(TEXT("/aerohub/mission/:event")), EHttpServerRequestVerbs::VERB_POST,
        [this](const FHttpServerRequest& Request, const FHttpResultCallback& Done) {
            const FString Event = Request.PathParams.FindRef(TEXT("event"));
            const FString Error = TimerEvent(RobotName, Event);
            auto Response = FHttpServerResponse::Create(Error.IsEmpty() ? StatusJson() : TEXT("{\"error\":\"Invalid event or transition\"}"), TEXT("application/json"));
            if (!Error.IsEmpty()) Response->Code = EHttpServerResponseCodes::BadRequest;
            Done(MoveTemp(Response)); return true;
        }));
    // Independent JSON-RPC transport; not the native ProjectAirSim NNG service.
    Routes.Add(Router->BindRoute(FHttpPath(TEXT("/aerohub/rpc")), EHttpServerRequestVerbs::VERB_POST,
        [this](const FHttpServerRequest& Request, const FHttpResultCallback& Done) {
            const FUTF8ToTCHAR Converted(reinterpret_cast<const ANSICHAR*>(Request.Body.GetData()), Request.Body.Num());
            TSharedPtr<FJsonObject> Input;
            auto Output = MakeShared<FJsonObject>();
            Output->SetStringField(TEXT("jsonrpc"), TEXT("2.0"));
            FString Method, Version, Error;
            const TSharedPtr<FJsonObject>* Params = nullptr;
            int32 ErrorCode = -32602;
            if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(FString(Converted.Length(), Converted.Get())), Input)
                || !Input.IsValid()) {
                Error = TEXT("Parse error"); ErrorCode = -32700;
            } else {
                Output->SetField(TEXT("id"), Input->HasField(TEXT("id")) ? Input->Values[TEXT("id")] : MakeShared<FJsonValueNull>());
                if (!Input->TryGetStringField(TEXT("jsonrpc"), Version) || Version != TEXT("2.0")
                    || !Input->TryGetStringField(TEXT("method"), Method)) {
                    Error = TEXT("Invalid Request"); ErrorCode = -32600;
                } else if (Method == TEXT("GetDisplayState")) {
                    TSharedPtr<FJsonObject> State;
                    FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(StatusJson()), State);
                    Output->SetObjectField(TEXT("result"), State);
                } else if (Method == TEXT("RegisterViews")) {
                    const TArray<TSharedPtr<FJsonValue>>* Entries = nullptr;
                    TArray<FView> NewViews;
                    if (!Input->TryGetObjectField(TEXT("params"), Params)
                        || !(*Params)->TryGetArrayField(TEXT("views"), Entries) || Entries->Num() > 256) {
                        Error = TEXT("Expected views array (max 256)");
                    } else {
                        for (const auto& Entry : *Entries) {
                            const TSharedPtr<FJsonObject>* Obj = nullptr;
                            FView View;
                            if (!Entry->TryGetObject(Obj)
                                || !(*Obj)->TryGetStringField(TEXT("vehicle_id"), View.Vehicle)
                                || !(*Obj)->TryGetStringField(TEXT("camera_id"), View.Camera)
                                || !(*Obj)->TryGetStringField(TEXT("image_type"), View.Image)
                                || !(*Obj)->TryGetStringField(TEXT("component"), View.Component)
                                || View.Vehicle.IsEmpty() || View.Vehicle == TEXT("__global__")
                                || View.Camera.IsEmpty() || View.Component.IsEmpty()) {
                                Error = TEXT("Invalid view identity"); break;
                            }
                            NewViews.Add(View);
                        }
                        if (Error.IsEmpty()) {
                            Views = MoveTemp(NewViews); ViewIndex = INDEX_NONE;
                            if (!Views.IsEmpty()) SelectView(0);
                            else { bGlobal = true; bOwnsView = true; }
                        }
                    }
                } else if (Method == TEXT("SetView")) {
                    FString Vehicle, CameraID, Image;
                    if (!Input->TryGetObjectField(TEXT("params"), Params)
                        || !(*Params)->TryGetStringField(TEXT("vehicle_id"), Vehicle)) {
                        Error = TEXT("Expected vehicle_id");
                    } else if (Vehicle == TEXT("__global__")) {
                        bGlobal = true; bOwnsView = true;
                    } else if (!(*Params)->TryGetStringField(TEXT("camera_id"), CameraID)
                        || !(*Params)->TryGetStringField(TEXT("image_type"), Image)) {
                        Error = TEXT("Expected camera_id and image_type");
                    } else {
                        int32 Found = Views.IndexOfByPredicate([&](const FView& V) {
                            return V.Vehicle == Vehicle && V.Camera == CameraID && V.Image == Image;
                        });
                        if (Found == INDEX_NONE || !SelectView(Found)) Error = TEXT("View not available");
                    }
                } else if (Method == TEXT("ControlTimer") || Method == TEXT("ControlRun")) {
                    FString Vehicle, Event;
                    if (!Input->TryGetObjectField(TEXT("params"), Params)
                        || !(*Params)->TryGetStringField(TEXT("vehicle_id"), Vehicle)
                        || !(*Params)->TryGetStringField(TEXT("event"), Event)) Error = TEXT("Expected vehicle_id and event");
                    else Error = Method == TEXT("ControlRun") ? RunEvent(Vehicle, Event) : TimerEvent(Vehicle, Event);
                } else if (Method == TEXT("SetRunState")) {
                    FString Vehicle, State;
                    if (!Input->TryGetObjectField(TEXT("params"), Params)
                        || !(*Params)->TryGetStringField(TEXT("vehicle_id"), Vehicle)
                        || Vehicle.IsEmpty() || Vehicle == TEXT("__global__")
                        || !(*Params)->TryGetStringField(TEXT("state"), State)
                        || (State != TEXT("idle") && State != TEXT("running") && State != TEXT("finished"))) {
                        Error = TEXT("Expected vehicle_id and idle/running/finished state");
                    } else {
                        VehicleStates.FindOrAdd(Vehicle).RunState = State;
                    }
                } else if (Method == TEXT("ClearMessage")) {
                    CenterMessage.Empty(); MessageDeadline = 0;
                } else if (Method == TEXT("SetTitle") || Method == TEXT("ShowMessage")) {
                    FString Text;
                    double Duration = 3;
                    if (!Input->TryGetObjectField(TEXT("params"), Params)
                        || !(*Params)->TryGetStringField(TEXT("text"), Text) || Text.Len() > 512) {
                        Error = TEXT("Expected text of at most 512 characters");
                    } else if (Method == TEXT("SetTitle")) {
                        FString Side = TEXT("left");
                        if ((*Params)->HasField(TEXT("side"))
                            && (!(*Params)->TryGetStringField(TEXT("side"), Side)
                                || (Side != TEXT("left") && Side != TEXT("right")))) {
                            Error = TEXT("side must be left or right");
                        } else if (Side == TEXT("right")) RightTitle = Text;
                        else DisplayTitle = Text;
                    } else if ((*Params)->HasField(TEXT("duration_seconds"))
                        && (!(*Params)->TryGetNumberField(TEXT("duration_seconds"), Duration)
                            || !FMath::IsFinite(Duration) || Duration < 0 || Duration > 86400)) {
                        Error = TEXT("Invalid duration_seconds");
                    } else {
                        CenterMessage = Text;
                        MessageDeadline = Duration > 0 ? FPlatformTime::Seconds() + Duration : 0;
                    }
                } else {
                    Error = TEXT("Method not found"); ErrorCode = -32601;
                }
            }
            if (!Output->HasField(TEXT("id"))) Output->SetField(TEXT("id"), MakeShared<FJsonValueNull>());
            if (!Error.IsEmpty()) {
                auto Detail = MakeShared<FJsonObject>();
                Detail->SetNumberField(TEXT("code"), ErrorCode);
                Detail->SetStringField(TEXT("message"), Error);
                Output->SetObjectField(TEXT("error"), Detail);
            } else if (!Output->HasField(TEXT("result"))) {
                Output->SetBoolField(TEXT("result"), true);
            }
            FString Body;
            FJsonSerializer::Serialize(Output, TJsonWriterFactory<>::Create(&Body));
            Done(FHttpServerResponse::Create(Body, TEXT("application/json")));
            return true;
        }));
    FHttpServerModule::Get().StartAllListeners();
}

void AAerohubHUD::EndPlay(const EEndPlayReason::Type Reason)
{
    if (Router.IsValid()) for (const auto& Route : Routes) if (Route.IsValid()) Router->UnbindRoute(Route);
    Routes.Empty(); Router.Reset();
    if (ObservationCamera) ObservationCamera->Destroy();
    Super::EndPlay(Reason);
}

double AAerohubHUD::Elapsed() const
{
    const auto* State = VehicleStates.Find(RobotName);
    if (!State || bGlobal) return 0;
    return State->Timer == TEXT("running") ? FPlatformTime::Seconds() - State->Start : State->Frozen;
}
FString AAerohubHUD::StatusJson() const
{
    auto Data = MakeShared<FJsonObject>();
    Data->SetStringField(TEXT("title"), DisplayTitle);
    Data->SetStringField(TEXT("title_left"), DisplayTitle);
    Data->SetStringField(TEXT("title_right"), RightTitle);
    Data->SetStringField(TEXT("message"), MessageDeadline > 0 && FPlatformTime::Seconds() >= MessageDeadline ? TEXT("") : CenterMessage);
    Data->SetStringField(TEXT("vehicle"), RobotName);
    Data->SetStringField(TEXT("vehicle_label"), bGlobal ? TEXT("Global") : RobotName);
    Data->SetStringField(TEXT("image_type"), ImageType);
    Data->SetBoolField(TEXT("global"), bGlobal);
    Data->SetStringField(TEXT("global_mode"), GlobalCameraTag.IsEmpty() ? TEXT("uav_centroid") : TEXT("actor_follow"));
    Data->SetStringField(TEXT("global_camera_tag"), GlobalCameraTag);
    Data->SetBoolField(TEXT("global_target_present"), bGlobalFollowAvailable);
    Data->SetBoolField(TEXT("view_available"), bGlobal
        ? (ObservationCamera != nullptr && (GlobalCameraTag.IsEmpty() || bGlobalFollowAvailable)) : Camera.IsValid());
    Data->SetBoolField(TEXT("vehicle_present"), Robot.IsValid());
    const auto* VehicleState = VehicleStates.Find(RobotName);
    Data->SetStringField(TEXT("run_state"), VehicleState ? VehicleState->RunState : TEXT(""));
    Data->SetStringField(TEXT("mission"), VehicleState ? VehicleState->Timer : TEXT("idle"));
    Data->SetNumberField(TEXT("elapsed_wall_seconds"), Elapsed());
    if (Robot.IsValid() && FPlatformTime::Seconds() - LastTelemetrySeconds < 1) Data->SetNumberField(TEXT("observed_speed_mps"), Speed);
    else Data->SetField(TEXT("observed_speed_mps"), MakeShared<FJsonValueNull>());
    Data->SetStringField(TEXT("speed_source"), TEXT("projectairsim_ground_truth_kinematics"));
    Data->SetNumberField(TEXT("peak_observed_speed_mps"), PeakSpeed);
    Data->SetStringField(TEXT("camera"), CameraName);
    Data->SetBoolField(TEXT("camera_capture_active"), Camera.IsValid() && Camera->IsActive());
    Data->SetBoolField(TEXT("camera_target_present"), Camera.IsValid() && Camera->TextureTarget != nullptr);
    FString Text;
    FJsonSerializer::Serialize(Data, TJsonWriterFactory<>::Create(&Text));
    return Text;
}

void AAerohubHUD::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    ConfigWait -= DeltaSeconds;
    if (ConfigWait <= 0) { ReadConfig(); ConfigWait = 0.5f; }
    UpdateView();
    const auto* State = VehicleStates.Find(RobotName);
    Speed = State ? State->Speed : 0;
    PeakSpeed = State ? State->Peak : 0;
    LastTelemetrySeconds = State ? State->Received : 0;
    MissionState = State ? State->Timer : TEXT("idle");
}

// v5 geometry is expressed at 1920 x 1080; the review viewport is 1280 x 720.
void AAerohubHUD::DrawLabel(const FString& Text, FVector2D Position, float Size,
                            FLinearColor Color, bool bCentered, bool bBold)
{
    FCanvasTextItem Item(Position, FText::FromString(Text), bBold ? MonoBold : MonoRegular, Color);
    Item.Scale = FVector2D(Size / 32.f);
    Item.bCentreX = bCentered;
    Item.bCentreY = bCentered;
    Canvas->DrawItem(Item);
}

void AAerohubHUD::DrawHUD()
{
    Super::DrawHUD();
    if (!bEnabled || !Canvas || !MonoBold) return;
    const float S = FMath::Min(Canvas->SizeX / 1920.f, Canvas->SizeY / 1080.f);
    DrawTextBlock(DisplayTitle, {24*S,24*S}, 36*S, Canvas->SizeX*.42f, 1, false, true);
    DrawTextBlock(RightTitle, {Canvas->SizeX-24*S,24*S}, 36*S, Canvas->SizeX*.42f, 1, false, true, true);
    if (MessageDeadline == 0 || FPlatformTime::Seconds() < MessageDeadline)
        DrawTextBlock(CenterMessage, {Canvas->SizeX/2.f,Canvas->SizeY/2.f}, 42*S,
                      Canvas->SizeX*.7f, 3, true);
    // Keep the accepted v5 bar proportions. Vehicle is primary; camera and
    // image format are subordinate metadata, not an equal-weight breadcrumb.
    const float W = 825*S, H = 57*S;
    const float X = (Canvas->SizeX-W)/2.f, Y = 21*S;
    const FLinearColor Primary = FLinearColor::FromSRGBColor(FColor(237,244,250));
    const FLinearColor Secondary = FLinearColor::FromSRGBColor(FColor(205,221,235));
    const FLinearColor Border(.75f,.81f,.86f,.85f);
    const auto* State = VehicleStates.Find(RobotName);
    const FString Run = !bGlobal && State && !State->RunState.IsEmpty() ? State->RunState.ToUpper() : TEXT("--");
    const FLinearColor Accent = FLinearColor::FromSRGBColor(Run == TEXT("RUNNING")
        ? FColor(53,226,194) : Run == TEXT("FINISHED") ? FColor(98,169,255) : FColor(164,184,205));
    auto Fit = [&](FString Text, float Size, float MaxWidth, bool Bold) {
        auto Width = [&](const FString& Value) {
            float TW,TH; Canvas->StrLen(Bold ? MonoBold : MonoRegular,Value,TW,TH);
            return TW*Size/32.f;
        };
        if (Width(Text)<=MaxWidth) return Text;
        while (!Text.IsEmpty() && Width(Text+TEXT("..."))>MaxWidth) Text.LeftChopInline(1);
        return Text+TEXT("...");
    };
    FLinearColor Panel = FLinearColor::FromSRGBColor(FColor(28,33,41));
    Panel.A = 136.f/255.f;
    DrawRect(Panel,X,Y,W,H);
    DrawRect(Border,X,Y,W,3*S); DrawRect(Border,X,Y+H-3*S,W,3*S);
    DrawRect(Border,X,Y,3*S,H); DrawRect(Border,X+W-3*S,Y,3*S,H);
    const FString VehicleLabel = bGlobal ? TEXT("Global") : RobotName;
    DrawLabel(TEXT("[ ") + Fit(VehicleLabel,21*S,180*S,true) + TEXT(" ]"),
              {X+25.5f*S,Y+13.5f*S},21*S,Primary);
    DrawRect(Accent,X+231*S,Y+10.5f*S,219*S,36*S);
    DrawLabel(bGlobal ? TEXT("GLOBAL") : Run,{X+340.5f*S,Y+28.5f*S},21*S,
              FLinearColor(.01f,.02f,.03f),true);
    DrawRect(Border,X+465*S,Y+12*S,1.5f*S,33*S);
    DrawLabel(bGlobal ? TEXT("VIEW") : TEXT("WALL"),{X+495*S,Y+13.5f*S},
              21*S,Secondary,false,true);
    DrawLabel(bGlobal ? (Robot.IsValid() ? TEXT("LIVE") : TEXT("--"))
                      : FString::Printf(TEXT("%05.1f s"),Elapsed()),
              {X+595.5f*S,Y+13.5f*S},21*S,Accent);

    // Smaller, quieter camera strip: context label, raw sensor name, format tag.
    const FString CameraText = Fit(CameraName,22.5f*S,360*S,true);
    const FString FormatText = Fit(ImageType,19.5f*S,220*S,true);
    float CW,CH,IW,IH;
    Canvas->StrLen(MonoBold,CameraText,CW,CH); CW *= 22.5f*S/32.f;
    Canvas->StrLen(MonoBold,FormatText,IW,IH); IW *= 19.5f*S/32.f;
    const float BadgeW = FMath::Max(48*S,IW+21*S);
    const float StripW = 75*S+CW+24*S+BadgeW+12*S;
    const float SX = (Canvas->SizeX-StripW)/2.f, SY = Y+66*S, SH = 31.5f*S;
    FLinearColor SubPanel = Panel; SubPanel.A = .78f;
    DrawRect(SubPanel,SX,SY,StripW,SH);
    DrawLabel(TEXT("CAM"),{SX+12*S,SY+6*S},19.5f*S,Secondary,false,true);
    DrawLabel(CameraText,{SX+63*S,SY+4.5f*S},22.5f*S,Primary,false,true);
    const float BX = SX+75*S+CW+12*S;
    const FLinearColor FormatColor = FLinearColor::FromSRGBColor(
        ImageType == TEXT("RGB") ? FColor(158,232,255) : FColor(255,222,157));
    FLinearColor BadgeFill = FormatColor; BadgeFill.A = .20f;
    DrawRect(BadgeFill,BX,SY+3*S,BadgeW,25.5f*S);
    DrawLabel(FormatText,{BX+BadgeW/2,SY+15.75f*S},19.5f*S,FormatColor,true);

    const FString Health = bGlobal ? (!GlobalCameraTag.IsEmpty()
        ? (bGlobalFollowAvailable ? TEXT("") : TEXT("NO FOLLOW TARGET"))
        : (Robot.IsValid() ? TEXT("") : TEXT("NO VEHICLES")))
        : !Robot.IsValid() ? TEXT("NO VEHICLE") : !Camera.IsValid() ? TEXT("NO CAMERA")
        : LastTelemetrySeconds <= 0 ? TEXT("WAITING DATA")
        : FPlatformTime::Seconds()-LastTelemetrySeconds >= 1 ? TEXT("STALE DATA") : TEXT("");
    if (!Health.IsEmpty()) DrawLabel(Health,{Canvas->SizeX/2.f,SY+SH+18*S},
        16.5f*S,FLinearColor(1,.7f,.3f),true);
    if (!bGlobal) DrawSpeedGauge(S);
}

void AAerohubHUD::DrawSpeedGauge(float S)
{
    const float R = 198*S;
    const FVector2D C(Canvas->SizeX-R-36*S,Canvas->SizeY-R+6*S);
    auto Point = [&](float Degrees,float Radius) {
        const float A=FMath::DegreesToRadians(Degrees);
        return C + FVector2D(FMath::Cos(A),FMath::Sin(A))*Radius;
    };
    auto Ring = [&](float Inner,float Outer,float Start,float End,FLinearColor Color) {
        TArray<FCanvasUVTri> Triangles;
        for(int I=0;I<4;++I) {
            const float A=FMath::Lerp(Start,End,I/4.f), B=FMath::Lerp(Start,End,(I+1)/4.f);
            const FVector2D P0=Point(A,Inner),P1=Point(A,Outer),P2=Point(B,Outer),P3=Point(B,Inner);
            FCanvasUVTri T; T.V0_Pos=P0;T.V1_Pos=P1;T.V2_Pos=P2;
            T.V0_Color=T.V1_Color=T.V2_Color=Color; Triangles.Add(T);
            T.V1_Pos=P2;T.V2_Pos=P3;Triangles.Add(T);
        }
        FCanvasTriangleItem Item(Triangles,GWhiteTexture);Item.BlendMode=SE_BLEND_Translucent;Canvas->DrawItem(Item);
    };
    auto Gradient = [](float T) {
        const float Stops[]={0,.24f,.48f,.72f,1};
        const FColor Colors[]={FColor(19,40,180),FColor(20,126,239),FColor(21,208,189),FColor(112,225,38),FColor(255,210,22)};
        for(int I=0;I<4;++I) if(T<=Stops[I+1]) return FMath::Lerp(FLinearColor::FromSRGBColor(Colors[I]),FLinearColor::FromSRGBColor(Colors[I+1]),(T-Stops[I])/(Stops[I+1]-Stops[I]));
        return FLinearColor::FromSRGBColor(Colors[4]);
    };
    const bool bFresh = Robot.IsValid() && LastTelemetrySeconds > 0 && FPlatformTime::Seconds()-LastTelemetrySeconds < 1;
    const float Fraction=bFresh ? FMath::Clamp(float(Speed)/SpeedMaxMps,0.f,1.f) : 0.f;
    const int Active=FMath::RoundToInt(Fraction*60);
    for(int I=0;I<60;++I) {
        Ring(R-15*S,R,150+4.f*I,150+4.f*(I+.78f),FLinearColor(.62f,.83f,.92f,.86f));
        if(I<Active) Ring(R-52.5f*S,R-25.5f*S,150+4.f*I,150+4.f*(I+1.03f),Gradient(I/59.f));
    }
    for(int I=0;I<=10;++I) {
        const float A=150+24.f*I;
        const FVector2D P=Point(A,R-13.5f*S),Q=Point(A,R+4.5f*S);
        DrawLine(P.X,P.Y,Q.X,Q.Y,FLinearColor(.85f,.91f,.96f),3*S);
        DrawLabel(FString::Printf(TEXT("%.0f"),SpeedMaxMps*I/10.f),Point(A,R+25.5f*S),15*S,FLinearColor(.85f,.91f,.96f),true);
    }
    const FVector2D P=Point(150+240*Fraction,R-57*S),Q=Point(150+240*Fraction,R+4.5f*S);
    if (bFresh) DrawLine(P.X,P.Y,Q.X,Q.Y,Gradient(Fraction),6*S);
    const FLinearColor Cyan=FLinearColor::FromSRGBColor(FColor(34,243,236));
    const FLinearColor Red=FLinearColor::FromSRGBColor(FColor(255,89,100));
    DrawLabel(TEXT("SIM GT  m/s"),C+FVector2D(0,-51*S),19.5f*S,Cyan,true);
    DrawLabel(bFresh?FString::Printf(TEXT("%.1f"),Speed):TEXT("--"),C+FVector2D(0,-12*S),46.5f*S,Cyan,true);
    DrawLabel(TEXT("PEAK  m/s"),C+FVector2D(0,34.5f*S),16.5f*S,Red,true);
    DrawLabel(Robot.IsValid()?FString::Printf(TEXT("%.1f"),PeakSpeed):TEXT("--"),C+FVector2D(0,67.5f*S),33*S,Red,true);

}

FString AAerohubHUD::RunEvent(const FString& Vehicle, const FString& Event)
{
    if (Vehicle.IsEmpty() || Vehicle == TEXT("__global__")) return TEXT("Expected vehicle identity");
    if (Event != TEXT("start") && Event != TEXT("finish") && Event != TEXT("reset"))
        return TEXT("Expected start/finish/reset event");
    // Validate before inserting or mutating state; rejected transitions have no effects.
    const auto* Existing = VehicleStates.Find(Vehicle);
    if (Event == TEXT("finish") && (!Existing || Existing->Timer == TEXT("idle")))
        return TEXT("Cannot finish an idle run");
    auto& State = VehicleStates.FindOrAdd(Vehicle);
    if (Event == TEXT("start")) {
        if (State.Timer != TEXT("running")) {
            State.Start = FPlatformTime::Seconds();
            State.Frozen = State.Peak = 0;
        }
        State.Timer = State.RunState = TEXT("running");
    } else if (Event == TEXT("finish")) {
        if (State.Timer == TEXT("running")) State.Frozen = FPlatformTime::Seconds()-State.Start;
        State.Timer = State.RunState = TEXT("finished");
    } else {
        State.Timer = State.RunState = TEXT("idle");
        State.Start = State.Frozen = State.Peak = 0;
    }
    return TEXT("");
}

FString AAerohubHUD::TimerEvent(const FString& Vehicle, const FString& Event)
{
    if (Vehicle.IsEmpty() || Vehicle == TEXT("__global__")) return TEXT("Expected vehicle identity");
    auto& State = VehicleStates.FindOrAdd(Vehicle);
    if (Event == TEXT("start") && State.Timer != TEXT("running")) {
        State.Timer = TEXT("running"); State.Start = FPlatformTime::Seconds(); State.Frozen = 0;
    } else if (Event == TEXT("finish") && State.Timer == TEXT("running")) {
        State.Frozen = FPlatformTime::Seconds()-State.Start; State.Timer = TEXT("finished");
    } else if (Event == TEXT("reset")) {
        State.Timer = TEXT("idle"); State.Start = State.Frozen = State.Peak = 0;
    } else return TEXT("Invalid timer transition");
    return TEXT("");
}

bool AAerohubHUD::SelectView(int32 Index)
{
    if (!Views.IsValidIndex(Index)) return false;
    const auto& View = Views[Index];
    USceneCaptureComponent2D* Match = nullptr;
    for (TObjectIterator<USceneCaptureComponent2D> It; It; ++It) {
        if (IsValid(*It) && It->IsRegistered() && It->GetWorld() == GetWorld() && IsValid(It->GetOwner())
            && !It->GetOwner()->IsActorBeingDestroyed()
            && It->GetOwner()->GetName() == View.Vehicle
            && It->GetOuter()->GetName() == View.Camera && It->GetName() == View.Component) {
            Match = *It; break;
        }
    }
    if (!IsValid(Match) || !Match->IsRegistered()) return false;
    ViewIndex = Index; bGlobal = false; bOwnsView = true;
    RobotName = View.Vehicle; CameraName = View.Camera; ImageType = View.Image;
    Robot = Match->GetOwner(); Camera = Match;
    return true;
}

void AAerohubHUD::NextView()
{
    if (Views.IsEmpty()) return;
    for (int32 Offset=1; Offset<=Views.Num(); ++Offset)
        if (SelectView((FMath::Max(ViewIndex,0)+Offset)%Views.Num())) return;
}
void AAerohubHUD::ToggleGlobal()
{
    if (bGlobal && SelectView(FMath::Max(ViewIndex,0))) return;
    bGlobal = true; bOwnsView = true;
}

void AAerohubHUD::UpdateView()
{
    if (!bOwnsView || !ObservationCamera) return;
    auto* PC = GetWorld()->GetFirstPlayerController();
    if (!PC) return;
    auto* Lens = ObservationCamera->GetCameraComponent();
    if (bGlobal) {
        RobotName = TEXT("__global__"); CameraName = TEXT("TopCamera"); ImageType = TEXT("RGB");
        Camera.Reset(); Robot.Reset();
        TSet<FString> Names;
        for (const auto& View : Views) Names.Add(View.Vehicle);
        TArray<FVector> Positions;
        FVector Center = FVector::ZeroVector;
        for (TActorIterator<AActor> It(GetWorld()); It; ++It)
            if (Names.Contains(It->GetName())) { Positions.Add(It->GetActorLocation()); Center += It->GetActorLocation(); Robot = *It; }
        bGlobalFollowAvailable = false;
        if (!GlobalCameraTag.IsEmpty()) {
            // Copy the reviewed scene camera; do not attach/teleport native robots.
            ACameraActor* Follow = nullptr;
            int32 Matches = 0;
            for (TActorIterator<ACameraActor> It(GetWorld()); It; ++It) {
                if (*It != ObservationCamera && !It->IsActorBeingDestroyed()
                    && It->ActorHasTag(FName(*GlobalCameraTag))) { Follow = *It; ++Matches; }
            }
            if (Matches == 1 && Follow && Follow->GetCameraComponent()) {
                const auto* Source = Follow->GetCameraComponent();
                ObservationCamera->SetActorLocationAndRotation(Source->GetComponentLocation(), Source->GetComponentRotation());
                Lens->FieldOfView = Source->FieldOfView;
                Lens->PostProcessSettings = Source->PostProcessSettings;
                Lens->PostProcessBlendWeight = Source->PostProcessBlendWeight;
                bGlobalFollowAvailable = true;
            }
            CameraName = TEXT("FollowCamera");
        } else if (!Positions.IsEmpty()) {
            Center /= Positions.Num();
            float Radius = 1000;
            for (const auto& Position : Positions) Radius = FMath::Max(Radius, FVector::Dist(Center,Position));
            const float Aspect = PC->PlayerCameraManager->GetCameraCacheView().AspectRatio;
            const float Height = Radius*1.3f*FMath::Max(Aspect,1.f);
            ObservationCamera->SetActorLocationAndRotation(Center+FVector(0,0,Height),FRotator(-90,0,0));
        }
        if (GlobalCameraTag.IsEmpty()) {
            Lens->FieldOfView = 90;
            Lens->PostProcessSettings = FPostProcessSettings(); Lens->PostProcessBlendWeight = 0;
        }
    } else {
        USceneCaptureComponent2D* Selected = Camera.Get();
        if (!IsValid(Selected) || !Selected->IsRegistered() || !IsValid(Selected->GetOwner())
            || Selected->GetOwner()->IsActorBeingDestroyed()) {
            Camera.Reset(); Robot.Reset();
            SelectView(ViewIndex);
            Selected = Camera.Get();
        }
        if (!IsValid(Selected) || !Selected->IsRegistered()) return;
        FMinimalViewInfo Info;
        Selected->GetCameraView(0,Info);
        ObservationCamera->SetActorLocationAndRotation(Info.Location,Info.Rotation);
        Lens->FieldOfView = Info.FOV;
        Lens->PostProcessSettings = Selected->PostProcessSettings;
        Lens->PostProcessBlendWeight = Selected->PostProcessBlendWeight;
    }
    Lens->bConstrainAspectRatio = false;
    if (PC->GetViewTarget() != ObservationCamera) PC->SetViewTarget(ObservationCamera);
}

void AAerohubHUD::DrawTextBlock(const FString& Text, FVector2D Position, float Size,
                                float MaxWidth, int32 Lines, bool Center, bool bTitle, bool bRight)
{
    UFont* Font = bTitle ? TitleFont : TextFont;
    if (Text.IsEmpty() || !Font) return;
    TArray<FString> Rows;
    FString Row;
    auto Width = [&](const FString& Value) {
        float W,H; Canvas->StrLen(Font,Value,W,H); return W*Size/32.f;
    };
    for (int32 I=0; I<Text.Len(); ++I) {
        if (Text[I] == '\r') continue;
        const FString Next = Row + Text.Mid(I,1);
        if (Text[I] == '\n' || Width(Next)>MaxWidth) {
            if (Rows.Num() == Lines-1) {
                while (!Row.IsEmpty() && Width(Row+TEXT("…"))>MaxWidth) Row.LeftChopInline(1);
                Rows.Add(Row+TEXT("…")); Row.Empty(); break;
            }
            Rows.Add(Row); Row = Text[I] == '\n' ? TEXT("") : Text.Mid(I,1);
        } else Row = Next;
    }
    if (!Row.IsEmpty() && Rows.Num()<Lines) Rows.Add(Row);
    float Y = Position.Y-(Center ? (Rows.Num()-1)*Size*.7f : 0);
    for (const auto& Line : Rows) {
        const FLinearColor Fill = bTitle
            ? FLinearColor::FromSRGBColor(bRight ? FColor(110,224,255) : FColor(255,209,102))
            : FLinearColor::White;
        FCanvasTextItem Item({bRight ? Position.X-Width(Line) : Position.X,Y},FText::FromString(Line),Font,Fill);
        if (bTitle) {
            Item.bOutlined = true;
            Item.OutlineColor = FLinearColor::FromSRGBColor(FColor(24,20,16));
        }
        Item.Scale = FVector2D(Size/32.f); Item.bCentreX = Center; Item.bCentreY = Center;
        Item.EnableShadow(FLinearColor::Black);
        Canvas->DrawItem(Item); Y += Size*1.4f;
    }
}
