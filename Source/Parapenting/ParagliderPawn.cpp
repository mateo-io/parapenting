#include "ParagliderPawn.h"
#include "ParaglidingAudioComponent.h"
#include "ParapentingMaterials.h"
#include "ParapentingTerrain.h"
#include "Physics/CanopyGeometry.h"
#include "Physics/ParagliderCoordinateSystem.h"
#include "Physics/SuspensionSystem.h"
#include "Physics/ParagliderSolverClock.h"
#include "Physics/TerrainModel.h"
#include "Physics/CameraFeedback.h"
#include "Physics/PilotPose.h"
#include "Physics/CanopyLoadPose.h"
#include "Physics/SuspensionSystem.h"
#include "Physics/LandingCircuitModel.h"
#include "Physics/GroundLaunchModel.h"
#include "Physics/FlightDebrief.h"
#include "Physics/PreflightBriefing.h"
#include "Physics/FlightNavigation.h"
#include "Physics/AerodynamicPolar.h"
#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "HAL/PlatformFileManager.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/ConfigCacheIni.h"
#include "HttpModule.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/BufferArchive.h"
#include "Serialization/MemoryReader.h"
#include "Misc/Paths.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/InputSettings.h"
#include "InputCoreTypes.h"
#include "ProceduralMeshComponent.h"
#include "Scalability.h"
#include "UObject/ConstructorHelpers.h"
#include "Materials/MaterialInstanceDynamic.h"

AParagliderPawn::AParagliderPawn()
{
    PrimaryActorTick.bCanEverTick = true;
    Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(Root);

    static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereFinder(
        TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(
        TEXT("/Engine/BasicShapes/Cube.Cube"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderFinder(
        TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    CanopyVisual = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("CanopyVisual"));
    CanopyVisual->SetupAttachment(Root);
    CanopyVisual->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    SuspensionVisual = CreateDefaultSubobject<UProceduralMeshComponent>(
        TEXT("SuspensionVisual"));
    SuspensionVisual->SetupAttachment(Root);
    SuspensionVisual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    SuspensionVisual->SetCastShadow(true);

    PilotVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PilotVisual"));
    PilotVisual->SetupAttachment(Root);
    PilotVisual->SetStaticMesh(SphereFinder.Object);
    PilotVisual->SetRelativeScale3D(FVector(0.38, 0.38, 0.72));
    PilotVisual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    PilotVisual->SetVisibility(false);

    PilotRig = CreateDefaultSubobject<USceneComponent>(TEXT("PilotRig"));
    PilotRig->SetupAttachment(Root);

    const auto ConfigurePart = [this](
        UStaticMeshComponent* Part, UStaticMesh* Mesh)
    {
        Part->SetupAttachment(PilotRig);
        Part->SetStaticMesh(Mesh);
        Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Part->SetCastShadow(true);
    };

    PilotTorso = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PilotTorso"));
    ConfigurePart(PilotTorso, CubeFinder.Object);
    PilotHead = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PilotHead"));
    ConfigurePart(PilotHead, SphereFinder.Object);
    HarnessVisual =
        CreateDefaultSubobject<UStaticMeshComponent>(TEXT("HarnessVisual"));
    ConfigurePart(HarnessVisual, CubeFinder.Object);
    LeftUpperArm =
        CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LeftUpperArm"));
    ConfigurePart(LeftUpperArm, CylinderFinder.Object);
    RightUpperArm =
        CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RightUpperArm"));
    ConfigurePart(RightUpperArm, CylinderFinder.Object);
    LeftForearm =
        CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LeftForearm"));
    ConfigurePart(LeftForearm, CylinderFinder.Object);
    RightForearm =
        CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RightForearm"));
    ConfigurePart(RightForearm, CylinderFinder.Object);
    LeftThigh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LeftThigh"));
    ConfigurePart(LeftThigh, CylinderFinder.Object);
    RightThigh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RightThigh"));
    ConfigurePart(RightThigh, CylinderFinder.Object);
    LeftShin = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LeftShin"));
    ConfigurePart(LeftShin, CylinderFinder.Object);
    RightShin = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RightShin"));
    ConfigurePart(RightShin, CylinderFinder.Object);
    LeftBrakeHandle =
        CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LeftBrakeHandle"));
    ConfigurePart(LeftBrakeHandle, SphereFinder.Object);
    RightBrakeHandle =
        CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RightBrakeHandle"));
    ConfigurePart(RightBrakeHandle, SphereFinder.Object);

    GhostCanopyVisual =
        CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("GhostCanopyVisual"));
    GhostCanopyVisual->SetupAttachment(Root);
    GhostCanopyVisual->SetUsingAbsoluteLocation(true);
    GhostCanopyVisual->SetUsingAbsoluteRotation(true);
    GhostCanopyVisual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    GhostCanopyVisual->SetVisibility(false);

    GhostPilotVisual =
        CreateDefaultSubobject<UStaticMeshComponent>(TEXT("GhostPilotVisual"));
    GhostPilotVisual->SetupAttachment(Root);
    GhostPilotVisual->SetStaticMesh(SphereFinder.Object);
    GhostPilotVisual->SetUsingAbsoluteLocation(true);
    GhostPilotVisual->SetUsingAbsoluteRotation(true);
    GhostPilotVisual->SetRelativeScale3D(FVector(0.28, 0.28, 0.55));
    GhostPilotVisual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    GhostPilotVisual->SetVisibility(false);

    BuildCanopyMesh();

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(Root);
    Camera->SetRelativeLocation(FVector(-850.0, 0.0, 260.0));
    Camera->SetRelativeRotation(FRotator(-5.0, 0.0, 0.0));

    FlightAudio = CreateDefaultSubobject<UParaglidingAudioComponent>(TEXT("FlightAudio"));
}

void AParagliderPawn::BeginPlay()
{
    Super::BeginPlay();
    LineGraph = Parapenting::Physics::BuildSuspensionGraph(
        Canopy, Parapenting::Physics::Epic2MlLinePlan());
    SolveSuspensionGraph();
    FlightAudio->Start();
    LoadPilotProgress();
    ApplyKeyboardLayout();
    ApplyGraphicsProfile();
    ApplyEquipmentConfiguration();
    AirModel.SetPreset(Parapenting::Physics::WeatherPresetId::MorningCalm);
    ManualWindFromDegrees = AirModel.GetSnapshot().windFromDegrees;
    ManualWindSpeedMps = AirModel.GetSnapshot().windSpeedMps;
    ResetFlight();
    RefreshReplayCatalogue();

    const auto TintPart = [](UStaticMeshComponent* Part, const FLinearColor& Color)
    {
        if (!Part) return;
        if (UMaterialInstanceDynamic* Material =
                Part->CreateAndSetMaterialInstanceDynamic(0))
        {
            Material->SetVectorParameterValue(TEXT("Color"), Color);
        }
    };
    TintPart(PilotTorso, FLinearColor(0.06f, 0.20f, 0.38f));
    TintPart(PilotHead, FLinearColor(0.78f, 0.49f, 0.31f));
    TintPart(HarnessVisual, FLinearColor(0.025f, 0.035f, 0.055f));
    for (UStaticMeshComponent* Limb : {
        LeftUpperArm.Get(), RightUpperArm.Get(),
        LeftForearm.Get(), RightForearm.Get()})
        TintPart(Limb, FLinearColor(0.06f, 0.20f, 0.38f));
    for (UStaticMeshComponent* Limb : {
        LeftThigh.Get(), RightThigh.Get(), LeftShin.Get(), RightShin.Get()})
        TintPart(Limb, FLinearColor(0.035f, 0.045f, 0.065f));
    TintPart(LeftBrakeHandle, FLinearColor(0.85f, 0.04f, 0.02f));
    TintPart(RightBrakeHandle, FLinearColor(0.85f, 0.04f, 0.02f));
}

void AParagliderPawn::ResetFlight()
{
    const auto& Route =
        Parapenting::Physics::GetRouteProfileByIndex(SelectedRouteIndex);
    const auto Launch = Parapenting::Physics::RouteLaunchLocalM(Route);
    const auto Landing = Parapenting::Physics::RouteLandingLocalM(Route);
    LandingTargetXM = Landing.x;
    LandingTargetYM = Landing.y;
    // Draw the region this route flies in. Every route change comes through
    // here, and Interlaken and Grindelwald are 20 km apart, so the terrain
    // actor rebuilds rather than stretching one mesh over both. It no-ops
    // when the new route is in the region already drawn.
    if (UWorld* World = GetWorld())
    {
        for (TActorIterator<AParapentingTerrain> It(World); It; ++It)
            It->BuildForRegionAt(Launch.x, Launch.y);
    }
    const double LaunchGround =
        Parapenting::Physics::TerrainModel::HeightM(Launch.x, Launch.y);
    auto NavigationLaunch = Launch;
    auto NavigationLanding = Landing;
    NavigationLaunch.z = LaunchGround;
    NavigationLanding.z =
        Parapenting::Physics::TerrainModel::HeightM(Landing.x, Landing.y);
    NavigationRoute = Parapenting::Physics::BuildNavigationRoute(
        NavigationLaunch, NavigationLanding);
    NavigationProgress = {};
    FVector LocationM(
        Launch.x, Launch.y, LaunchGround + 25.0);
    const double RouteDX = Landing.x - Launch.x;
    const double RouteDY = Landing.y - Launch.y;
    const double RouteLength = FMath::Max(
        1.0, FMath::Sqrt(RouteDX * RouteDX + RouteDY * RouteDY));
    double ForwardX = RouteDX / RouteLength;
    double ForwardY = RouteDY / RouteLength;
    const auto ScenarioId =
        Parapenting::Physics::GetTrainingScenarioByIndex(
            SelectedScenarioIndex).id;
    if (ScenarioId == Parapenting::Physics::TrainingScenarioId::LandingFlare)
    {
        const double LandingGround =
            Parapenting::Physics::TerrainModel::HeightM(
                Landing.x, Landing.y);
        const auto SurfaceAir = AirModel.Sample(
            {Landing.x, Landing.y, LandingGround + 6.0}, 0.0);
        const double SurfaceSpeed = FMath::Sqrt(
            SurfaceAir.windWorldMps.x * SurfaceAir.windWorldMps.x
            + SurfaceAir.windWorldMps.y * SurfaceAir.windWorldMps.y);
        if (SurfaceSpeed > 0.5)
        {
            ForwardX = -SurfaceAir.windWorldMps.x / SurfaceSpeed;
            ForwardY = -SurfaceAir.windWorldMps.y / SurfaceSpeed;
        }
        const double StartX = Landing.x - ForwardX * 420.0;
        const double StartY = Landing.y - ForwardY * 420.0;
        const double StartGround =
            Parapenting::Physics::TerrainModel::HeightM(StartX, StartY);
        LocationM = FVector(StartX, StartY, StartGround + 55.0);
    }
    const double YawRad = FMath::Atan2(ForwardY, ForwardX);
    SetActorLocation(LocationM * 100.0);
    SetActorRotation(FRotator(
        0.0, FMath::RadiansToDegrees(YawRad), 0.0));
    State = {};
    State.positionWorldM = {LocationM.X, LocationM.Y, LocationM.Z};
    State.velocityWorldMps = {ForwardX * 10.5, ForwardY * 10.5, -1.2};
    State.attitude = {
        FMath::Cos(YawRad * 0.5), 0.0, 0.0,
        FMath::Sin(YawRad * 0.5)
    };
    Controls = {};
    ControllerControls = {};
    AppliedControls = {};
    SolverClock.Reset();
    SimulationTimeSeconds = 0.0;
    bLanded = false;
    bGroundLaunching = false;
    bLaunchHeld = false;
    bHardLanding = false;
    bRolloutFinalized = false;
    RolloutState = {};
    RolloutOutput = {};
    LandingDistanceM = 0.0;
    TouchdownVerticalSpeedMps = 0.0;
    TouchdownHorizontalSpeedMps = 0.0;
    LastCameraVelocityMps = FVector(
        State.velocityWorldMps.x, State.velocityWorldMps.y,
        State.velocityWorldMps.z);
    SmoothedBodyAccelerationMps2 = FVector::ZeroVector;
    HapticModel.Reset();
    Challenge.Reset(
        Parapenting::Physics::GetTrainingScenarioByIndex(
            SelectedScenarioIndex).id);
    bChallengeResultRecorded = false;
    Debrief.Reset();
}

void AParagliderPawn::PrepareGroundLaunch()
{
    ResetFlight();
    const auto& Route =
        Parapenting::Physics::GetRouteProfileByIndex(SelectedRouteIndex);
    const auto Launch = Parapenting::Physics::RouteLaunchLocalM(Route);
    const auto Landing = Parapenting::Physics::RouteLandingLocalM(Route);
    const double Ground = Parapenting::Physics::TerrainModel::HeightM(
        Launch.x, Launch.y);
    const double DX = Landing.x - Launch.x;
    const double DY = Landing.y - Launch.y;
    const double Length = FMath::Max(1.0, FMath::Sqrt(DX * DX + DY * DY));
    const double ForwardX = DX / Length;
    const double ForwardY = DY / Length;
    const double YawRad = FMath::Atan2(ForwardY, ForwardX);
    State.positionWorldM = {Launch.x, Launch.y, Ground + 1.0};
    State.velocityWorldMps = {};
    State.attitude = {
        FMath::Cos(YawRad * 0.5), 0.0, 0.0,
        FMath::Sin(YawRad * 0.5)};
    State.canopyPressure = 0.0;
    LaunchModel.Reset(LaunchState, false);
    LaunchOutput = {};
    bGroundLaunching = true;
    bLaunchHeld = false;
}

void AParagliderPawn::PrepareReverseGroundLaunch()
{
    PrepareGroundLaunch();
    LaunchModel.Reset(LaunchState, true);
    LaunchOutput.pilotFacingYawOffsetRad =
        LaunchState.pilotFacingYawOffsetRad;
    LaunchOutput.brakeSidesCrossed = true;
}

void AParagliderPawn::StartLaunchRun()
{
    if (bGroundLaunching)
        bLaunchHeld = true;
}

void AParagliderPawn::StopLaunchRun()
{
    bLaunchHeld = false;
}

void AParagliderPawn::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    UpdateBindingCapture();
    // Fixed-step spine. The clock owns the accumulator so the step count is
    // derived from total delivered time rather than a residual, and so
    // simulation time is an exact multiple of the step instead of a running
    // sum. See ParagliderSolverClock.h.
    const int DueSteps = SolverClock.BeginFrame(DeltaSeconds);
    for (int StepIndex = 0; StepIndex < DueSteps; ++StepIndex)
    {
        const double PreviousSimulationTime = SimulationTimeSeconds;
        // Half-span comes from the canopy geometry, not from a second
        // hardcoded aspect ratio. sqrt(area * 5.2) gave 5.92 m, which was
        // approximating the *developed* half-span of 5.90 m - but this samples
        // the air at the physical wing halves, so it wants the projected
        // half-span, 4.65 m. The old figure placed the sample points 27%
        // further out than the tips actually are.
        const double HalfSpanM = 0.5 * Canopy.ProjectedSpanM();
        const Parapenting::Physics::Atmosphere Atmosphere =
            AirModel.SampleCanopy(
                State.positionWorldM,
                State.attitude.Rotate({0.0, 1.0, 0.0}),
                HalfSpanM, SimulationTimeSeconds);
        if (bPlayingReplay && ReplayFrames.IsValidIndex(ReplayFrameIndex))
        {
            AppliedControls = ReplayFrames[ReplayFrameIndex++];
        }
        else
        {
            if (bPlayingReplay) bPlayingReplay = false;
            AppliedControls.leftBrake =
                FMath::Max(Controls.leftBrake, ControllerControls.leftBrake);
            AppliedControls.rightBrake =
                FMath::Max(Controls.rightBrake, ControllerControls.rightBrake);
            AppliedControls.weightShift = FMath::Clamp(
                Controls.weightShift + ControllerControls.weightShift, -1.0, 1.0);
            AppliedControls.accelerator = FMath::Max(
                Controls.accelerator, ControllerControls.accelerator);
        }
        if (bLanded)
        {
            if (bRecordingReplay) ReplayFrames.Add(AppliedControls);
            const auto SurfaceAir = AirModel.Sample(
                State.positionWorldM
                    + Parapenting::Physics::Vec3{0.0, 0.0, 2.0},
                SimulationTimeSeconds);
            RolloutOutput = RolloutModel.Step(
                RolloutState,
                {
                    AppliedControls.leftBrake,
                    AppliedControls.rightBrake,
                    SurfaceAir.windWorldMps
                },
                PhysicsStepSeconds);
            bHardLanding = bHardLanding || RolloutState.hardImpact;
            if (!bRolloutFinalized
                && RolloutState.phase
                    == Parapenting::Physics::LandingRolloutPhase::Settled)
            {
                Challenge.FinalizeRollout(
                    RolloutState.runoutDistanceM,
                    RolloutState.hardImpact);
                bRolloutFinalized = true;
            }
            State.positionWorldM += RolloutOutput.displacementWorldM;
            const double NewGround =
                Parapenting::Physics::TerrainModel::HeightM(
                    State.positionWorldM.x, State.positionWorldM.y);
            State.positionWorldM.z = NewGround + 0.8;
            State.velocityWorldMps = RolloutState.velocityWorldMps;
            State.canopyPressure = RolloutState.canopyPressure;
            State.angularVelocityBodyRadps = {};
            SolverClock.EndStep();
            SimulationTimeSeconds = SolverClock.SimulationTimeSeconds();
            continue;
        }
        if (bGroundLaunching)
        {
            const auto& Route =
                Parapenting::Physics::GetRouteProfileByIndex(
                    SelectedRouteIndex);
            const auto Launch =
                Parapenting::Physics::RouteLaunchLocalM(Route);
            const auto Landing =
                Parapenting::Physics::RouteLandingLocalM(Route);
            const Parapenting::Physics::Vec3 Forward =
                Parapenting::Physics::Normalized(Parapenting::Physics::Vec3{
                    Landing.x - Launch.x, Landing.y - Launch.y, 0.0});
            const double Ground =
                Parapenting::Physics::TerrainModel::HeightM(
                    State.positionWorldM.x, State.positionWorldM.y);
            const double AheadGround =
                Parapenting::Physics::TerrainModel::HeightM(
                    State.positionWorldM.x + Forward.x * 12.0,
                    State.positionWorldM.y + Forward.y * 12.0);
            const auto SurfaceAir = AirModel.Sample(
                State.positionWorldM
                    + Parapenting::Physics::Vec3{0.0, 0.0, 2.0},
                SimulationTimeSeconds);
            Parapenting::Physics::GroundLaunchInput LaunchInput;
            LaunchInput.launchHeld = bLaunchHeld;
            LaunchInput.surfaceWindMps = SurfaceAir.windWorldMps;
            LaunchInput.launchDirection = Forward;
            LaunchInput.slopeDownRadians = FMath::Atan2(
                Ground - AheadGround, 12.0);
            LaunchInput.leftBrake = AppliedControls.leftBrake;
            LaunchInput.rightBrake = AppliedControls.rightBrake;
            LaunchInput.weightShift = AppliedControls.weightShift;
            LaunchInput.allUpMassKg = Dynamics.Parameters().allUpMassKg;
            LaunchInput.wingAreaM2 = Dynamics.Parameters().areaM2;
            LaunchOutput = LaunchModel.Step(
                LaunchState, LaunchInput, PhysicsStepSeconds);
            State.positionWorldM +=
                LaunchOutput.pilotVelocityWorldMps * PhysicsStepSeconds;
            const double NewGround =
                Parapenting::Physics::TerrainModel::HeightM(
                    State.positionWorldM.x, State.positionWorldM.y);
            State.positionWorldM.z = NewGround + 1.0;
            State.velocityWorldMps = LaunchOutput.pilotVelocityWorldMps;
            State.canopyPressure = LaunchOutput.canopyPressure;
            Parapenting::Physics::FlightDebriefSample DebriefSample;
            DebriefSample.positionWorldM = State.positionWorldM;
            DebriefSample.airspeedMps = LaunchOutput.apparentWindMps;
            DebriefSample.canopyPressure = LaunchOutput.canopyPressure;
            DebriefSample.leftBrake = AppliedControls.leftBrake;
            DebriefSample.rightBrake = AppliedControls.rightBrake;
            DebriefSample.weightShift = AppliedControls.weightShift;
            DebriefSample.groundClearanceM = 1.0;
            DebriefSample.distanceToLandingM = FMath::Sqrt(
                FMath::Square(State.positionWorldM.x - LandingTargetXM)
                + FMath::Square(State.positionWorldM.y - LandingTargetYM));
            DebriefSample.groundLaunching = true;
            Debrief.Step(DebriefSample, PhysicsStepSeconds);
            if (LaunchOutput.liftOff)
            {
                bGroundLaunching = false;
                State.positionWorldM.z = NewGround + 1.25;
                State.velocityWorldMps.z = 0.8;
                State.canopyPressure = FMath::Max(
                    State.canopyPressure, 0.9);
            }
            SolverClock.EndStep();
            SimulationTimeSeconds = SolverClock.SimulationTimeSeconds();
            continue;
        }
        if (bRecordingReplay) ReplayFrames.Add(AppliedControls);
        Dynamics.Step(State, AppliedControls, Atmosphere, PhysicsStepSeconds);
        if (bRecordingReplay && (++GhostCaptureStep % 12) == 0)
        {
            GhostFrames.Add(FTransform(
                FQuat(State.attitude.x, State.attitude.y,
                      State.attitude.z, State.attitude.w),
                FVector(State.positionWorldM.x, State.positionWorldM.y,
                        State.positionWorldM.z) * 100.0));
        }
        const auto& T = Dynamics.LastTelemetry();
        const auto& Q = State.attitude;
        const double YawRad = FMath::Atan2(
            2.0 * (Q.w * Q.z + Q.x * Q.y),
            1.0 - 2.0 * (Q.y * Q.y + Q.z * Q.z));
        const double ChallengeGround =
            Parapenting::Physics::TerrainModel::HeightM(
                State.positionWorldM.x, State.positionWorldM.y);
        Parapenting::Physics::ChallengeSample Sample;
        Sample.verticalSpeedMps = State.velocityWorldMps.z;
        Sample.thermalLiftMps = T.thermalLiftMps;
        Sample.rotorStrength = T.rotorStrength;
        Sample.airspeedMps = T.airspeedMps;
        Sample.yawRad = YawRad;
        Sample.rollRateRadps = State.angularVelocityBodyRadps.x;
        Sample.pitchRateRadps = State.angularVelocityBodyRadps.y;
        Sample.yawRateRadps = State.angularVelocityBodyRadps.z;
        Sample.loadFactor = T.loadFactor;
        Sample.highLoadDeformation = T.highLoadDeformation;
        Sample.leftCollapse = T.leftCollapse;
        Sample.rightCollapse = T.rightCollapse;
        Sample.frontalCollapse = T.frontalCollapse;
        Sample.canopyPressure = T.canopyPressure;
        Sample.recoverySurge = T.recoverySurge;
        Sample.deepStall = T.deepStall;
        Sample.spin = T.spin;
        Sample.leftCravat = T.leftCravat;
        Sample.rightCravat = T.rightCravat;
        Sample.leftBrake = AppliedControls.leftBrake;
        Sample.rightBrake = AppliedControls.rightBrake;
        Sample.distanceToLandingM = FMath::Sqrt(
            FMath::Square(State.positionWorldM.x - LandingTargetXM)
            + FMath::Square(State.positionWorldM.y - LandingTargetYM));
        Sample.groundClearanceM = State.positionWorldM.z - ChallengeGround;
        const auto& Route =
            Parapenting::Physics::GetRouteProfileByIndex(SelectedRouteIndex);
        const Parapenting::Physics::Vec3 LandingTarget{
            LandingTargetXM, LandingTargetYM,
            Parapenting::Physics::TerrainModel::HeightM(
                LandingTargetXM, LandingTargetYM)};
        const auto SurfaceAir = AirModel.Sample(
            LandingTarget + Parapenting::Physics::Vec3{0.0, 0.0, 6.0},
            SimulationTimeSeconds);
        // The published Lehn circuit is right-hand in the normal valley
        // breeze; Hoehematte is the opposite. Reverse it in mountain flow.
        const bool ValleyFlow = SurfaceAir.windWorldMps.x <= 0.0;
        const bool ValleyRightHand = !Route.advancedLanding;
        const bool RightHand = ValleyFlow
            ? ValleyRightHand : !ValleyRightHand;
        LandingGuidance = Parapenting::Physics::EvaluateLandingApproach(
            Parapenting::Physics::BuildLandingCircuit(
                LandingTarget, SurfaceAir.windWorldMps, RightHand),
            State.positionWorldM, State.velocityWorldMps,
            Sample.groundClearanceM);
        Sample.landingPhase = LandingGuidance.phase;
        Sample.approachQuality = LandingGuidance.approachQuality;
        Sample.stabilizedApproach = LandingGuidance.stabilized;
        Sample.groundEffect = T.groundEffect;
        Sample.flareEnergy = T.flareEnergy;
        Sample.flareAuthority = T.flareAuthority;
        Challenge.Step(Sample, PhysicsStepSeconds);
        Parapenting::Physics::FlightDebriefSample DebriefSample;
        DebriefSample.positionWorldM = State.positionWorldM;
        DebriefSample.verticalSpeedMps = State.velocityWorldMps.z;
        DebriefSample.airspeedMps = T.airspeedMps;
        DebriefSample.loadFactor = T.loadFactor;
        DebriefSample.canopyPressure = T.canopyPressure;
        DebriefSample.thermalLiftMps = T.thermalLiftMps;
        DebriefSample.rotorStrength = T.rotorStrength;
        DebriefSample.turbulence = T.turbulence;
        DebriefSample.leftCollapse = T.leftCollapse;
        DebriefSample.rightCollapse = T.rightCollapse;
        DebriefSample.frontalCollapse = T.frontalCollapse;
        DebriefSample.leftCravat = T.leftCravat;
        DebriefSample.rightCravat = T.rightCravat;
        DebriefSample.deepStall = T.deepStall;
        DebriefSample.spin = T.spin;
        DebriefSample.leftBrake = AppliedControls.leftBrake;
        DebriefSample.rightBrake = AppliedControls.rightBrake;
        DebriefSample.weightShift = AppliedControls.weightShift;
        DebriefSample.groundClearanceM = Sample.groundClearanceM;
        DebriefSample.distanceToLandingM = Sample.distanceToLandingM;
        DebriefSample.approachQuality = LandingGuidance.approachQuality;
        DebriefSample.stabilizedApproach = LandingGuidance.stabilized;
        Debrief.Step(DebriefSample, PhysicsStepSeconds);
        Parapenting::Physics::UpdateNavigationProgress(
            NavigationProgress, NavigationRoute, State.positionWorldM);
        SolverClock.EndStep();
        SimulationTimeSeconds = SolverClock.SimulationTimeSeconds();
        ApplyIncidentCue(Parapenting::Physics::ScenarioCueCrossed(
            Parapenting::Physics::GetTrainingScenarioByIndex(
                SelectedScenarioIndex),
            PreviousSimulationTime, SimulationTimeSeconds));
    }
    if (bRecordingTelemetry)
    {
        TelemetryAccumulatorSeconds += DeltaSeconds;
        while (TelemetryAccumulatorSeconds >= 0.1)
        {
            RecordTelemetrySample();
            TelemetryAccumulatorSeconds -= 0.1;
        }
    }

    const double GroundHeight = Parapenting::Physics::TerrainModel::HeightM(
        State.positionWorldM.x, State.positionWorldM.y);
    if (!bLanded && State.positionWorldM.z <= GroundHeight + 0.8)
    {
        const double HorizontalSpeed = FMath::Sqrt(
            State.velocityWorldMps.x * State.velocityWorldMps.x
            + State.velocityWorldMps.y * State.velocityWorldMps.y);
        TouchdownVerticalSpeedMps = State.velocityWorldMps.z;
        TouchdownHorizontalSpeedMps = HorizontalSpeed;
        bHardLanding = State.velocityWorldMps.z < -4.0 || HorizontalSpeed > 12.0;
        LandingDistanceM = FMath::Sqrt(
            FMath::Square(State.positionWorldM.x - LandingTargetXM)
            + FMath::Square(State.positionWorldM.y - LandingTargetYM));
        Challenge.FinalizeLanding(LandingDistanceM,
            TouchdownVerticalSpeedMps, TouchdownHorizontalSpeedMps);
        Debrief.FinalizeLanding(LandingDistanceM,
            TouchdownVerticalSpeedMps, TouchdownHorizontalSpeedMps);
        SaveFlightDebrief();
        RolloutModel.Begin(
            RolloutState, State.velocityWorldMps, State.canopyPressure);
        bHardLanding = RolloutState.hardImpact;
        bLanded = true;
        State.positionWorldM.z = GroundHeight + 0.8;
        State.velocityWorldMps = RolloutState.velocityWorldMps;
        State.angularVelocityBodyRadps = {};
    }
    const bool bAwaitingRolloutScore =
        IsLandingFlareScenario() && bLanded && !bRolloutFinalized;
    if (Challenge.IsComplete() && !bChallengeResultRecorded
        && !bAwaitingRolloutScore)
    {
        ChallengeBestScores[SelectedScenarioIndex] = FMath::Max(
            ChallengeBestScores[SelectedScenarioIndex], Challenge.Score());
        bChallengeResultRecorded = true;
        SavePilotProgress();
    }

    SetActorLocation(FVector(
        State.positionWorldM.x * 100.0,
        State.positionWorldM.y * 100.0,
        State.positionWorldM.z * 100.0));
    SetActorRotation(FQuat(
        State.attitude.x, State.attitude.y, State.attitude.z, State.attitude.w));

    const auto& Telemetry = Dynamics.LastTelemetry();
    const float AudioAirspeed = bLanded
        ? static_cast<float>(Parapenting::Physics::Length(
            RolloutState.velocityWorldMps))
        : (bGroundLaunching
        ? static_cast<float>(LaunchOutput.apparentWindMps)
        : static_cast<float>(Telemetry.airspeedMps));
    const float AudioCanopyPressure = bLanded
        ? static_cast<float>(State.canopyPressure)
        : (bGroundLaunching
        ? static_cast<float>(LaunchOutput.canopyPressure)
        : static_cast<float>(Telemetry.canopyPressure));
    const float AudioLineLoad = bGroundLaunching
        ? static_cast<float>(LaunchState.lineLoadN)
        : static_cast<float>(Telemetry.lineLoadTotalN);
    FlightAudio->SetFlightAudio(
        static_cast<float>(State.velocityWorldMps.z),
        AudioAirspeed,
        static_cast<float>(Telemetry.turbulence),
        bGroundLaunching ? 0.0f : static_cast<float>(FMath::Max(
            Telemetry.leftCollapse, Telemetry.frontalCollapse)),
        bGroundLaunching ? 0.0f : static_cast<float>(FMath::Max(
            Telemetry.rightCollapse, Telemetry.frontalCollapse)),
        bGroundLaunching ? 0.0f : static_cast<float>(Telemetry.leftCravat),
        bGroundLaunching ? 0.0f : static_cast<float>(Telemetry.rightCravat),
        AudioCanopyPressure,
        AudioLineLoad,
        static_cast<float>(Telemetry.recoverySurge),
        static_cast<float>(Telemetry.leftBrakeForceN),
        static_cast<float>(Telemetry.rightBrakeForceN),
        static_cast<float>(Telemetry.thermalCoreStrength),
        static_cast<float>(Telemetry.aerodynamicUnloading),
        static_cast<float>(Telemetry.highLoadDeformation),
        static_cast<float>(Telemetry.highFrequencyGustMps));
    UpdateControllerHaptics();
    UpdateCanopyMesh();
    if (bGroundLaunching)
    {
        const float Inflation =
            static_cast<float>(LaunchState.inflation);
        CanopyVisual->SetRelativeLocation(FVector(
            -820.0f * (1.0f - Inflation),
            0.0f,
            -610.0f * (1.0f - Inflation)));
        CanopyVisual->SetRelativeRotation(FRotator(
            0.0f,
            FMath::RadiansToDegrees(
                static_cast<float>(LaunchState.canopyHeadingErrorRad)),
            0.0f));
    }
    else if (bLanded)
    {
        const float Deflation =
            1.0f - static_cast<float>(State.canopyPressure);
        CanopyVisual->SetRelativeLocation(FVector(
            -220.0f * Deflation, 0.0f, -560.0f * Deflation));
        CanopyVisual->SetRelativeRotation(FRotator(
            -48.0f * Deflation, 0.0f, 0.0f));
    }
    else
    {
        // The canopy swings on its lines, and where it ends up is decided by
        // the physics rather than by a rotation convention.
        //
        // Rotating the component about its own origin - which sits at the
        // pilot, 7.3 m below the mesh - does sweep the right arc, and that is
        // what this used to rely on. But whether it sweeps FORWARD or AFT
        // depends on the handedness of FRotator's pitch, which no suite here
        // can check: a wing that surges backwards looks exactly as convincing
        // as one that surges forwards until someone flies it. This project has
        // shipped two convention errors that survived review for dozens of
        // commits.
        //
        // So the arc is computed in the physics layer, where it is tested
        // (`EvaluateCanopySwingOffset`, checked for sign and magnitude in the
        // geometry suite), and the component is then translated by whatever it
        // takes to land there. The rotation still supplies the canopy's tilt;
        // it just no longer decides which way the wing goes.
        const FRotator SwingRotation(
            FMath::RadiansToDegrees(static_cast<float>(
                Telemetry.canopyRelativePitchRad)),
            0.0f,
            FMath::RadiansToDegrees(static_cast<float>(
                Telemetry.canopyRelativeRollRad)));
        CanopyVisual->SetRelativeRotation(SwingRotation);

        constexpr float MetresToCm = static_cast<float>(
            Parapenting::Physics::WorldAxes::MetresToUnrealUnits);
        constexpr float SuspensionRiseCm = 730.0f;
        const Parapenting::Physics::CanopySwingOffset Swing =
            Parapenting::Physics::EvaluateCanopySwingOffset(
                Telemetry.canopyRelativePitchRad,
                SuspensionRiseCm / MetresToCm);
        // Where the rotation alone would put the canopy, and where it should
        // be. The difference is the correction, so the convention cancels out
        // of the answer instead of being trusted.
        const FVector RestCentre(0.0f, 0.0f, SuspensionRiseCm);
        const FVector Rotated = SwingRotation.RotateVector(RestCentre);
        const FVector Target(
            RestCentre.X + Swing.forwardM * MetresToCm,
            Rotated.Y,
            RestCentre.Z + Swing.riseM * MetresToCm);
        CanopyVisual->SetRelativeLocation(FVector(
            Target.X - Rotated.X, 0.0f, Target.Z - Rotated.Z));
    }
    const int32 GhostIndex =
        FMath::FloorToInt(SimulationTimeSeconds * 10.0);
    const bool bShowGhost = bGhostVisible && GhostFrames.IsValidIndex(GhostIndex)
        && !bRecordingReplay && !bPlayingReplay
        && SelectedRouteIndex == ReplayRouteIndex
        && SelectedScenarioIndex == ReplayScenarioIndex;
    GhostCanopyVisual->SetVisibility(bShowGhost);
    GhostPilotVisual->SetVisibility(bShowGhost);
    if (bShowGhost)
    {
        const FTransform& Ghost = GhostFrames[GhostIndex];
        GhostCanopyVisual->SetWorldTransform(Ghost);
        GhostPilotVisual->SetWorldTransform(Ghost);
        GhostPilotVisual->AddWorldOffset(FVector(0.0, 0.0, 40.0));
    }
    UpdatePilotVisual();

    const auto& Accessibility =
        Parapenting::Physics::GetAccessibilityProfile(AccessibilityProfile);
    const float MotionScale =
        static_cast<float>(Accessibility.inertialCameraScale);
    const FVector CurrentVelocity(
        State.velocityWorldMps.x, State.velocityWorldMps.y,
        State.velocityWorldMps.z);
    const FVector WorldAcceleration = DeltaSeconds > 0.0001f
        ? (CurrentVelocity - LastCameraVelocityMps) / DeltaSeconds
        : FVector::ZeroVector;
    LastCameraVelocityMps = CurrentVelocity;
    const auto BodyAcceleration = State.attitude.InverseRotate({
        WorldAcceleration.X, WorldAcceleration.Y, WorldAcceleration.Z});
    SmoothedBodyAccelerationMps2 = FMath::VInterpTo(
        SmoothedBodyAccelerationMps2,
        FVector(BodyAcceleration.x, BodyAcceleration.y, BodyAcceleration.z),
        DeltaSeconds, 3.2f);
    const auto CameraResponse =
        Parapenting::Physics::EvaluateCameraFeedback(
            Telemetry,
            {SmoothedBodyAccelerationMps2.X,
             SmoothedBodyAccelerationMps2.Y,
             SmoothedBodyAccelerationMps2.Z},
            SimulationTimeSeconds, Accessibility);
    const FVector InertialOffset(
        CameraResponse.positionOffsetCm.x,
        CameraResponse.positionOffsetCm.y,
        CameraResponse.positionOffsetCm.z);
    FVector CameraBase(-1050.0f, 0.0f, 220.0f);
    float BaseFov = 88.0f;
    if (CameraMode == 1)
    {
        CameraBase = FVector(-570.0f, 0.0f, 105.0f);
        BaseFov = 94.0f;
    }
    else if (CameraMode == 2)
    {
        CameraBase = FVector(115.0f, 0.0f, 85.0f);
        BaseFov = 103.0f;
    }
    const FVector CameraTarget = CameraBase + InertialOffset + FVector(
        -static_cast<float>((Telemetry.airspeedMps - 10.5) * 7.0),
        static_cast<float>(-Telemetry.harnessRollRad * 95.0) * MotionScale,
        static_cast<float>(Telemetry.harnessPitchRad * 120.0) * MotionScale);
    Camera->SetRelativeLocation(FMath::VInterpTo(
        Camera->GetRelativeLocation(), CameraTarget, DeltaSeconds, 2.8f));
    const float CameraBasePitch = CameraMode == 2 ? -2.0f : 3.0f;
    const FRotator CameraRotationTarget(
        CameraBasePitch + static_cast<float>(CameraResponse.pitchDegrees),
        static_cast<float>(CameraResponse.yawDegrees),
        static_cast<float>(CameraResponse.rollDegrees));
    Camera->SetRelativeRotation(FMath::RInterpTo(
        Camera->GetRelativeRotation(), CameraRotationTarget, DeltaSeconds, 3.6f));
    Camera->SetFieldOfView(FMath::FInterpTo(
        Camera->FieldOfView,
        BaseFov + static_cast<float>(
            CameraResponse.fieldOfViewDeltaDegrees),
        DeltaSeconds, 2.0f));

    // Render a load-responsive suspension fan. The line endpoints follow the
    // same collapse/cravat contraction as the canopy mesh, while unloaded
    // lines sag and flutter instead of remaining rigid during an incident.
    BeginSuspensionMesh();
    const FTransform ActorTransform = GetActorTransform();
    const auto SuspensionLoadPose =
        Parapenting::Physics::EvaluateCanopyLoadPose(
            Telemetry.highLoadDeformation, Telemetry.loadFactor);
    const auto& SuspensionGeometry =
        Parapenting::Physics::Epic2MlSuspensionGeometry();
    // Suspension endpoints must use the exact component transform that poses
    // the canopy. Reconstructing pitch/roll with RotateAngleAxis uses
    // different sign/order conventions from FRotator and visibly detaches the
    // lines during large pendular excursions.
    const FTransform CanopyRelativeTransform =
        CanopyVisual->GetRelativeTransform();
    // Keep the risers legible as separate load paths. Width is driven by the
    // filtered group tension, so an incident replay exposes which side and
    // which cascade was carrying load rather than merely colouring a line.
    const FColor RiserColors[4] = {
        FColor(235, 45, 38), FColor(245, 205, 35),
        FColor(55, 105, 225), FColor(40, 190, 90)};
    for (int32 Side = -1; Side <= 1; Side += 2)
    {
        const bool bLeft = Side < 0;
        const FVector CarabinerLocal = CarabinerLocalCm(bLeft);
        const FVector MaillonSide(0.0f, 3.2f, 0.0f);
        const FVector MaillonUp(0.0f, 0.0f, 5.0f);
        const FColor MaillonColor(185, 190, 198);
        AddSuspensionSegment(
            CarabinerLocal - MaillonSide - MaillonUp,
            CarabinerLocal + MaillonSide - MaillonUp,
            MaillonColor, 0.9f);
        AddSuspensionSegment(
            CarabinerLocal + MaillonSide - MaillonUp,
            CarabinerLocal + MaillonSide + MaillonUp,
            MaillonColor, 0.9f);
        AddSuspensionSegment(
            CarabinerLocal + MaillonSide + MaillonUp,
            CarabinerLocal - MaillonSide + MaillonUp,
            MaillonColor, 0.9f);
        AddSuspensionSegment(
            CarabinerLocal - MaillonSide + MaillonUp,
            CarabinerLocal - MaillonSide - MaillonUp,
            MaillonColor, 0.9f);
        // The maillon and the harness webbing the carabiner hangs on, so the
        // load path is visibly continuous from the pilot up rather than
        // starting in mid air beside them.
        const FVector HarnessTop = PilotRigToActor().TransformPosition(
            FVector(-2.0f, bLeft ? -6.0f : 6.0f, 8.0f));
        AddSuspensionSegment(
            HarnessTop, CarabinerLocal, FColor(60, 62, 68), 1.5f);
        const FVector ShoulderWebbing = PilotRigToActor().TransformPosition(
            FVector(-9.0f, bLeft ? -12.0f : 12.0f, 34.0f));
        const FVector SeatWebbing = PilotRigToActor().TransformPosition(
            FVector(7.0f, bLeft ? -15.0f : 15.0f, -24.0f));
        AddSuspensionSegment(
            ShoulderWebbing, HarnessTop, FColor(38, 42, 50), 1.7f);
        AddSuspensionSegment(
            HarnessTop, SeatWebbing, FColor(38, 42, 50), 1.9f);
        for (int32 Group = 0; Group < 4; ++Group)
        {
            const float TensionN = static_cast<float>(bLeft
                ? Telemetry.leftLineTensionN[Group]
                : Telemetry.rightLineTensionN[Group]);
            const float Load01 = FMath::Clamp(
                FMath::Sqrt(FMath::Max(0.0f, TensionN) / 180.0f),
                0.0f, 1.0f);
            const FVector RiserTop = RiserTopLocalCm(Group, bLeft);
            // Webbing, not thread: a riser is a flat band about 2 cm wide, so
            // it is drawn as two rails with a maillon bar across the top. At
            // one hairline per group the four groups read as a single smear of
            // lines going the same way, which is the main reason the rig did
            // not look like a rig.
            const FVector Lateral = PilotRigToActor().TransformVector(
                FVector(0.0f, 1.0f, 0.0f)) * 1.1f;
            const float Width = FMath::Lerp(1.1f, 2.6f, Load01);
            for (int32 Rail = -1; Rail <= 1; Rail += 2)
            {
                AddSuspensionSegment(
                    CarabinerLocal + Lateral * Rail,
                    RiserTop + Lateral * Rail,
                    RiserColors[Group], Width * 0.5f);
            }
            AddSuspensionSegment(
                RiserTop - Lateral, RiserTop + Lateral,
                RiserColors[Group], Width * 0.5f);
        }
    }
    for (const auto& Attachment : SuspensionGeometry.attachments)
    {
        const float Span01 = static_cast<float>(Attachment.spanFraction);
        const float Chord01 = static_cast<float>(Attachment.chordFraction);
        const float AbsSpan = FMath::Abs(Span01);
        const float TipBlend = FMath::SmoothStep(0.42f, 1.0f, AbsSpan);
        const bool bLeft = Span01 < 0.0f;
        const float Collapse = static_cast<float>(
            bGroundLaunching ? 0.0
            : (bLeft ? Telemetry.leftCollapse : Telemetry.rightCollapse))
            * TipBlend;
        const float Cravat = static_cast<float>(
            bGroundLaunching ? 0.0
            : (bLeft ? Telemetry.leftCravat : Telemetry.rightCravat))
            * TipBlend;
        const float Brake = static_cast<float>(bLeft
            ? AppliedControls.leftBrake : AppliedControls.rightBrake);
        const float SuspensionPressure = bLanded
            ? static_cast<float>(State.canopyPressure)
            : (bGroundLaunching
            ? static_cast<float>(LaunchOutput.canopyPressure)
            : static_cast<float>(Telemetry.canopyPressure));
        const float Pressure = FMath::Clamp(
            SuspensionPressure
                * (1.0f - 0.78f * Collapse - 0.9f * Cravat),
            0.0f, 1.0f);
        // Same canopy geometry the mesh is built from, so a line endpoint is
        // by construction a point on the rendered surface. These formulas were
        // previously duplicated here with their own copies of the half span,
        // arch and chord constants, which meant lines terminated wherever
        // those constants happened to agree with the mesh's.
        constexpr float MetresToCm = static_cast<float>(
            Parapenting::Physics::WorldAxes::MetresToUnrealUnits);
        constexpr float SuspensionRiseCm = 730.0f;
        const Parapenting::Physics::RibStation AttachStation =
            Canopy.StationAt(Span01);
        const float ContractedSpan =
            static_cast<float>(AttachStation.positionM.y) * MetresToCm
            * (1.0f - 0.28f * Collapse - 0.38f * Cravat)
            * static_cast<float>(SuspensionLoadPose.spanScale);
        const float Arch = SuspensionRiseCm
            + static_cast<float>(AttachStation.positionM.z) * MetresToCm
            - static_cast<float>(SuspensionLoadPose.extraArchDropCm)
                * FMath::Pow(AbsSpan, 1.65f)
            + static_cast<float>(SuspensionLoadPose.lineStretchCm);
        const float Drop = 255.0f * Collapse + 330.0f * Cravat;
        const float Flutter = (1.0f - Pressure)
            * 24.0f * FMath::Sin(static_cast<float>(
                SimulationTimeSeconds * 12.0 + Span01 * 9.0));
        // Which riser this group hangs from. RiserGroup indexes the same
        // A/A'/B/C ordering the riser draw uses, so a main line starts exactly
        // where its riser ends; TensionGroup indexes the telemetry, which
        // reports A/B/C plus brakes and has no separate A' entry.
        int32 RiserGroup = 0;
        int32 TensionGroup = 0;
        FColor GroupColor(235, 45, 38);
        if (Attachment.group == Parapenting::Physics::SuspensionGroup::BabyA)
        {
            RiserGroup = 1;
            GroupColor = FColor(245, 80, 45);
        }
        else if (Attachment.group == Parapenting::Physics::SuspensionGroup::B)
        {
            RiserGroup = 2;
            TensionGroup = 1;
            GroupColor = FColor(245, 205, 35);
        }
        else if (Attachment.group == Parapenting::Physics::SuspensionGroup::C)
        {
            RiserGroup = 3;
            TensionGroup = 2;
            GroupColor = FColor(55, 105, 225);
        }
        const float LineSlack = static_cast<float>(bLeft
            ? Telemetry.leftLineSlack[TensionGroup]
            : Telemetry.rightLineSlack[TensionGroup]);
        const float LineTensionN = static_cast<float>(bLeft
            ? Telemetry.leftLineTensionN[TensionGroup]
            : Telemetry.rightLineTensionN[TensionGroup]);
        const float LineLoad01 = FMath::Clamp(
            FMath::Sqrt(FMath::Max(0.0f, LineTensionN) / 180.0f),
            0.0f, 1.0f);
        // A main line starts at the top of its riser, not at the carabiner.
        // Starting them all at the carabiner drew the risers as four stubs
        // with the lines running past them from a common point below - which
        // is why the attachments looked wrong and the lines looked parallel.
        const FVector PilotLocal = RiserTopLocalCm(RiserGroup, bLeft);
        const float LoadedChord =
            static_cast<float>(AttachStation.chordM) * MetresToCm
            * static_cast<float>(SuspensionLoadPose.chordScale);
        FVector CanopyLocal(
            (0.48f - Chord01) * LoadedChord
                + static_cast<float>(
                    (bGroundLaunching ? 0.0 : Telemetry.recoverySurge)
                        * 350.0),
            ContractedSpan + Flutter,
            Arch - Drop);
        CanopyLocal =
            CanopyRelativeTransform.TransformPosition(CanopyLocal);
        // The cascade junction, placed where the solver put it: the fraction
        // along the riser-to-attachment run and the sag off that line are the
        // Level 2 solution's, not a drawn curve. Slack still deepens the
        // droop, because a line that has stopped carrying load hangs under its
        // own weight and the rigid-canopy solver has no collapse to show it.
        const int32 ShapeRow = Attachment.group
                == Parapenting::Physics::SuspensionGroup::BabyA
            ? static_cast<int32>(Parapenting::Physics::LineRow::ABaby)
            : (Attachment.group
                   == Parapenting::Physics::SuspensionGroup::B
               ? static_cast<int32>(Parapenting::Physics::LineRow::B)
               : (Attachment.group
                      == Parapenting::Physics::SuspensionGroup::C
                  ? static_cast<int32>(Parapenting::Physics::LineRow::C)
                  : static_cast<int32>(Parapenting::Physics::LineRow::A)));
        const FVector RunLocal = CanopyLocal - PilotLocal;
        FVector SolvedOffset =
            LineShape[ShapeRow].OffsetFromRunM * MetresToCm;
        if (bLeft) SolvedOffset.Y = -SolvedOffset.Y;
        const FVector MidLocal =
            PilotLocal + RunLocal * LineShape[ShapeRow].splitAlongRun
            + SolvedOffset
            // A line that has stopped carrying load hangs under its own
            // weight. The rigid-canopy solver has no collapse to show that
            // with, so slack still deepens the droop here.
            + FVector(0.0f, Flutter * (0.35f + 1.15f * LineSlack),
                      -95.0f * FMath::Square(LineSlack));
        const FColor LineColor = FLinearColor::LerpUsingHSV(
            FLinearColor(0.16f, 0.17f, 0.18f),
            FLinearColor(GroupColor), 1.0f - 0.72f * LineSlack)
                .ToFColor(true);
        const float LineRadius = FMath::Lerp(
            0.28f, 0.72f, LineLoad01 * (1.0f - LineSlack));
        AddSuspensionSegment(PilotLocal, MidLocal, LineColor, LineRadius);
        AddSuspensionSegment(MidLocal, CanopyLocal, LineColor, LineRadius);
    }
    // Brake fans are independent of the three load-bearing risers and pull
    // the trailing edge down toward the pilot's hands.
    for (int32 Side = -1; Side <= 1; Side += 2)
    {
        const bool bLeft = Side < 0;
        const float Brake = static_cast<float>(bLeft
            ? AppliedControls.leftBrake : AppliedControls.rightBrake);
        const float BrakeSlack = static_cast<float>(bLeft
            ? Telemetry.leftLineSlack[3] : Telemetry.rightLineSlack[3]);
        const float BrakeTensionN = static_cast<float>(bLeft
            ? Telemetry.leftLineTensionN[3]
            : Telemetry.rightLineTensionN[3]);
        const float BrakeLoad01 = FMath::Clamp(
            FMath::Sqrt(FMath::Max(0.0f, BrakeTensionN) / 55.0f),
            0.0f, 1.0f);
        // The fist the forearm is drawn to. The brake line and the hand
        // holding it were computed independently before, so pulling a brake
        // moved the arm down and left the line ending beside it.
        const FVector HandLocal = BrakeHandLocalCm(bLeft);
        for (int32 Branch = 0; Branch < 4; ++Branch)
        {
            const float Span01 = Side * (0.24f + Branch * 0.20f);
            const float AbsSpan = FMath::Abs(Span01);
            const float TipBlend = FMath::SmoothStep(
                0.42f, 1.0f, AbsSpan);
            const float Collapse = static_cast<float>(
                bGroundLaunching ? 0.0
                : (bLeft ? Telemetry.leftCollapse
                         : Telemetry.rightCollapse)) * TipBlend;
            const float Cravat = static_cast<float>(
                bGroundLaunching ? 0.0
                : (bLeft ? Telemetry.leftCravat
                         : Telemetry.rightCravat)) * TipBlend;
            constexpr float MetresToCm = static_cast<float>(
                Parapenting::Physics::WorldAxes::MetresToUnrealUnits);
            constexpr float SuspensionRiseCm = 730.0f;
            const Parapenting::Physics::RibStation BrakeStation =
                Canopy.StationAt(Span01);
            const float LoadedChord =
                static_cast<float>(BrakeStation.chordM) * MetresToCm
                * static_cast<float>(SuspensionLoadPose.chordScale);
            const float ContractedSpan =
                static_cast<float>(BrakeStation.positionM.y) * MetresToCm
                * (1.0f - 0.28f * Collapse - 0.38f * Cravat)
                * static_cast<float>(SuspensionLoadPose.spanScale);
            const float Arch = SuspensionRiseCm
                + static_cast<float>(BrakeStation.positionM.z) * MetresToCm
                - static_cast<float>(SuspensionLoadPose.extraArchDropCm)
                    * FMath::Pow(AbsSpan, 1.65f)
                + static_cast<float>(SuspensionLoadPose.lineStretchCm);
            const float CollapseDrop =
                255.0f * Collapse + 330.0f * Cravat;
            const float RotorFlutter = static_cast<float>(
                bGroundLaunching ? 0.0 : Telemetry.rotorStrength)
                * TipBlend * 15.0f * FMath::Sin(static_cast<float>(
                    SimulationTimeSeconds * 11.0 + Span01 * 8.0));
            FVector TrailingEdge(
                -0.52f * LoadedChord
                    + static_cast<float>(
                        (bGroundLaunching ? 0.0 : Telemetry.recoverySurge)
                            * 350.0),
                ContractedSpan,
                Arch - CollapseDrop - Brake * 55.0f + RotorFlutter
                    - static_cast<float>(
                        bGroundLaunching ? 0.0 : Telemetry.deepStall)
                        * 75.0f);
            TrailingEdge =
                CanopyRelativeTransform.TransformPosition(TrailingEdge);
            const FVector Cascade = FMath::Lerp(
                HandLocal, TrailingEdge, 0.62f)
                + FVector(
                    28.0f * BrakeSlack,
                    FMath::Sin(static_cast<float>(
                        SimulationTimeSeconds * 10.0 + Span01 * 8.0))
                        * 14.0f * BrakeSlack,
                    -135.0f * BrakeSlack * BrakeSlack);
            const float BrakeRadius = FMath::Lerp(
                0.28f, 0.68f, BrakeLoad01 * (1.0f - BrakeSlack));
            AddSuspensionSegment(
                HandLocal, Cascade, FColor(40, 190, 90), BrakeRadius);
            AddSuspensionSegment(
                Cascade, TrailingEdge, FColor(40, 190, 90),
                BrakeRadius * 0.88f);
        }
    }
    CommitSuspensionMesh();
    // The old always-on 5.2 m debug sphere made every landing field readable
    // by marker rather than by terrain. Keep it only in the explicit geometry
    // diagnostic view; the production path must stand on field palette,
    // approach references and later authored wear/edge overlays.
    if (bGeometryVisualization)
    {
        DrawDebugSphere(
            GetWorld(),
            FVector(LandingTargetXM, LandingTargetYM,
                    Parapenting::Physics::TerrainModel::HeightM(
                        LandingTargetXM, LandingTargetYM) + 1.2) * 100.0,
            260.0f, 24, FColor::White, false, 0.0f, 0, 7.0f);
    }
    if (!NavigationProgress.complete)
    {
        const std::size_t WaypointIndex = FMath::Min<std::size_t>(
            NavigationProgress.activeWaypoint,
            NavigationRoute.waypoints.size() - 1);
        const auto& Waypoint =
            NavigationRoute.waypoints[WaypointIndex];
        const FVector WaypointWorld(
            Waypoint.positionWorldM.x * 100.0,
            Waypoint.positionWorldM.y * 100.0,
            Waypoint.positionWorldM.z * 100.0);
        DrawDebugSphere(
            GetWorld(), WaypointWorld, 1200.0f, 20,
            FColor(35, 220, 255), false, 0.0f, 0, 6.0f);
        DrawDebugLine(
            GetWorld(), WaypointWorld - FVector(0.0f, 0.0f, 5000.0f),
            WaypointWorld + FVector(0.0f, 0.0f, 5000.0f),
            FColor(35, 220, 255), false, 0.0f, 0, 5.0f);
    }
    if (GetDistanceToTargetM() < 1700.0)
    {
        const auto GatePoint = [](const Parapenting::Physics::Vec3& Point)
        {
            return FVector(Point.x, Point.y, Point.z) * 100.0;
        };
        const auto& Circuit = LandingGuidance.circuit;
        const FVector Downwind = GatePoint(Circuit.downwindGate);
        const FVector Base = GatePoint(Circuit.baseGate);
        const FVector Final = GatePoint(Circuit.finalGate);
        const FVector Target = GatePoint(Circuit.target);
        DrawDebugDirectionalArrow(
            GetWorld(), Downwind, Base, 900.0f, FColor(80, 170, 255),
            false, 0.0f, 0, 10.0f);
        DrawDebugDirectionalArrow(
            GetWorld(), Base, Final, 900.0f, FColor(255, 190, 45),
            false, 0.0f, 0, 10.0f);
        DrawDebugDirectionalArrow(
            GetWorld(), Final, Target, 900.0f, FColor(75, 255, 125),
            false, 0.0f, 0, 12.0f);
        DrawDebugSphere(
            GetWorld(), Downwind, 650.0f, 16, FColor(80, 170, 255),
            false, 0.0f, 0, 5.0f);
        DrawDebugSphere(
            GetWorld(), Base, 650.0f, 16, FColor(255, 190, 45),
            false, 0.0f, 0, 5.0f);
        DrawDebugSphere(
            GetWorld(), Final, 650.0f, 16,
            LandingGuidance.stabilized
                ? FColor(75, 255, 125) : FColor(255, 100, 60),
            false, 0.0f, 0, 6.0f);
    }

    if (bGeometryVisualization)
    {
        DrawCanopyGeometryDebug();
    }
    if (bAirflowVisualization)
    {
        // A compact Eulerian probe volume that makes the authored air field
        // inspectable in flight. Arrows show actual sampled velocity rather
        // than decorative particles or camera shake.
        constexpr double HorizontalSpacingM = 145.0;
        constexpr double VerticalSpacingM = 110.0;
        for (int32 GX = -1; GX <= 1; ++GX)
        {
            for (int32 GY = -1; GY <= 1; ++GY)
            {
                for (int32 GZ = -1; GZ <= 1; ++GZ)
                {
                    const Parapenting::Physics::Vec3 Probe{
                        State.positionWorldM.x + GX * HorizontalSpacingM,
                        State.positionWorldM.y + GY * HorizontalSpacingM,
                        FMath::Max(
                            Parapenting::Physics::TerrainModel::HeightM(
                                State.positionWorldM.x + GX * HorizontalSpacingM,
                                State.positionWorldM.y + GY * HorizontalSpacingM)
                                + 12.0,
                            State.positionWorldM.z + GZ * VerticalSpacingM)
                    };
                    const auto Air = AirModel.Sample(
                        Probe, SimulationTimeSeconds);
                    const FVector Start(
                        Probe.x * 100.0, Probe.y * 100.0, Probe.z * 100.0);
                    FVector Flow(
                        Air.windWorldMps.x,
                        Air.windWorldMps.y,
                        Air.windWorldMps.z);
                    const float Speed = Flow.Size();
                    Flow = Flow.GetSafeNormal()
                        * FMath::Clamp(Speed * 1800.0f, 2200.0f, 12500.0f);
                    FColor Color(65, 205, 255);
                    if (Air.rotorStrength > 0.35)
                        Color = FColor(235, 55, 255);
                    else if (Air.thermalLiftMps > 0.35)
                        Color = FColor(255, 155, 35);
                    else if (Air.sinkRingMps > 0.20
                             || Air.windWorldMps.z < -0.45)
                        Color = FColor(60, 90, 255);
                    DrawDebugDirectionalArrow(
                        GetWorld(), Start, Start + Flow, 420.0f, Color,
                        false, 0.0f, 0, 3.2f);
                }
            }
        }
    }
}

void AParagliderPawn::PosePilotSegment(
    UStaticMeshComponent* Segment,
    const FVector& Start, const FVector& End, float RadiusScale)
{
    if (!Segment) return;
    const FVector Direction = End - Start;
    const float LengthCm = FMath::Max(1.0f, Direction.Size());
    Segment->SetRelativeLocation((Start + End) * 0.5f);
    Segment->SetRelativeRotation(FQuat::FindBetweenNormals(
        FVector::UpVector, Direction / LengthCm));
    // Engine BasicShapes/Cylinder is 100 cm high and 100 cm in diameter.
    Segment->SetRelativeScale3D(FVector(
        RadiusScale, RadiusScale, LengthCm / 100.0f));
}

FTransform AParagliderPawn::PilotRigToActor() const
{
    return PilotRig ? PilotRig->GetRelativeTransform() : FTransform::Identity;
}

FVector AParagliderPawn::CarabinerLocalCm(bool bLeft) const
{
    // Rig space: the carabiners are hardware on the harness, one to each side,
    // half the harness's own carabiner separation apart. That separation is the
    // lever the payload's weight acts on in Level 3, so the drawn rig and the
    // solved statics describe the same hardware rather than two similar ones.
    //
    // The 34 cm height is the physics model's own carabiner height. Its
    // telemetry also carried a `22 sin(roll)` lateral shift and a
    // `10 (1 - cos(roll))` drop, which is the harness rolling - and PilotRig is
    // already rotated by that same roll. Two descriptions of one motion, which
    // is why the pilot and the lines disagreed on screen. The rig supplies it
    // once now, and the telemetry fields remain for CSV and analysis.
    constexpr float MetresToCm = static_cast<float>(
        Parapenting::Physics::WorldAxes::MetresToUnrealUnits);
    const float HalfSeparationCm = 0.5f * MetresToCm * static_cast<float>(
        Parapenting::Physics::HarnessGeometryFor(Equipment)
            .carabinerSeparationM);
    const FVector InRig(
        -2.0f, bLeft ? -HalfSeparationCm : HalfSeparationCm, 34.0f);
    return PilotRigToActor().TransformPosition(InRig);
}

FVector AParagliderPawn::RiserTopLocalCm(int32 Group, bool bLeft) const
{
    // A, A', B, C front to back. Real risers are about 45 cm of webbing with
    // the groups separated by a few centimetres fore and aft; that separation
    // is what makes the mains fan instead of running as one parallel sheet.
    static constexpr float ForeAftCm[4] = {6.0f, 1.0f, -5.0f, -11.0f};
    static constexpr float RiserLengthCm = 45.0f;
    const int32 Index = FMath::Clamp(Group, 0, 3);
    const FVector Carabiner = CarabinerLocalCm(bLeft);
    const FVector Up = PilotRigToActor().TransformVector(
        FVector(0.0f, 0.0f, 1.0f));
    const FVector Fore = PilotRigToActor().TransformVector(
        FVector(1.0f, 0.0f, 0.0f));
    return Carabiner + Up * RiserLengthCm + Fore * ForeAftCm[Index];
}

FVector AParagliderPawn::BrakeHandLocalCm(bool bLeft) const
{
    // The hand the arm is actually drawn to. This used to be a point built
    // from the carabiner and the brake input, so the brake lines converged
    // somewhere near the pilot rather than into their fists.
    const Parapenting::Physics::Vec3& Hand = bLeft
        ? LastPilotPose.leftHandCm : LastPilotPose.rightHandCm;
    return PilotRigToActor().TransformPosition(
        FVector(Hand.x, Hand.y, Hand.z));
}

void AParagliderPawn::UpdatePilotVisual()
{
    const auto& T = Dynamics.LastTelemetry();
    const auto Pose = Parapenting::Physics::EvaluatePilotPose({
        T.harnessRollRad,
        T.harnessPitchRad,
        AppliedControls.weightShift,
        AppliedControls.leftBrake,
        AppliedControls.rightBrake,
        T.leftBrakeForceN,
        T.rightBrakeForceN,
        FMath::Max(FMath::Max(
            T.leftCollapse, T.rightCollapse), T.frontalCollapse),
        T.recoverySurge
    });
    // Cached for the suspension draw, which hangs the risers off this same
    // body rather than off an independently reconstructed anchor.
    LastPilotPose = Pose;
    const auto ToFVector = [](const Parapenting::Physics::Vec3& Value)
    {
        return FVector(Value.x, Value.y, Value.z);
    };
    const bool bPilotFallen = bLanded
        && RolloutState.phase
            == Parapenting::Physics::LandingRolloutPhase::Fallen;
    PilotRig->SetRelativeLocation(
        bPilotFallen ? FVector(0.0f, 0.0f, -32.0f)
        : (bGroundLaunching || bLanded
            ? FVector::ZeroVector : ToFVector(Pose.rigOffsetCm)));
    PilotRig->SetRelativeRotation(FRotator(
        bGroundLaunching || bLanded ? 0.0 : Pose.rigRotationDegrees.x,
        bGroundLaunching
            ? FMath::RadiansToDegrees(static_cast<float>(
                LaunchOutput.pilotFacingYawOffsetRad))
            : (bPilotFallen ? 0.0 : Pose.rigRotationDegrees.y),
        bPilotFallen
            ? static_cast<float>(RolloutOutput.pilotFallRollDegrees)
            : (bGroundLaunching || bLanded
                ? 0.0 : Pose.rigRotationDegrees.z)));

    HarnessVisual->SetRelativeLocation(FVector(-2.0f, 0.0f, -17.0f));
    HarnessVisual->SetRelativeScale3D(FVector(0.52f, 0.48f, 0.30f));
    HarnessVisual->SetRelativeRotation(FRotator(-8.0f, 0.0f, 0.0f));
    PilotTorso->SetRelativeLocation(FVector(-9.0f, 0.0f, 17.0f));
    PilotTorso->SetRelativeScale3D(FVector(0.32f, 0.29f, 0.48f));
    PilotTorso->SetRelativeRotation(FRotator(
        bGroundLaunching
            ? 4.0f
            : -10.0f - static_cast<float>(T.recoverySurge) * 10.0f,
        0.0f,
        static_cast<float>(AppliedControls.weightShift) * -4.0f));
    PilotHead->SetRelativeLocation(FVector(-12.0f, 0.0f, 53.0f));
    PilotHead->SetRelativeScale3D(FVector(0.18f, 0.18f, 0.20f));

    const FVector LeftShoulder = ToFVector(Pose.leftShoulderCm);
    const FVector RightShoulder = ToFVector(Pose.rightShoulderCm);
    const FVector LeftHand = ToFVector(Pose.leftHandCm);
    const FVector RightHand = ToFVector(Pose.rightHandCm);
    const FVector LeftElbow = ToFVector(Pose.leftElbowCm);
    const FVector RightElbow = ToFVector(Pose.rightElbowCm);
    PosePilotSegment(LeftUpperArm, LeftShoulder, LeftElbow, 0.075f);
    PosePilotSegment(RightUpperArm, RightShoulder, RightElbow, 0.075f);
    PosePilotSegment(LeftForearm, LeftElbow, LeftHand, 0.065f);
    PosePilotSegment(RightForearm, RightElbow, RightHand, 0.065f);
    LeftBrakeHandle->SetRelativeLocation(LeftHand);
    RightBrakeHandle->SetRelativeLocation(RightHand);
    LeftBrakeHandle->SetRelativeScale3D(FVector(0.075f, 0.055f, 0.11f));
    RightBrakeHandle->SetRelativeScale3D(FVector(0.075f, 0.055f, 0.11f));

    const bool bPilotRunning = bGroundLaunching
        || (bLanded && RolloutState.phase
            == Parapenting::Physics::LandingRolloutPhase::Running);
    const double PilotRunSpeed = bGroundLaunching
        ? LaunchState.pilotRunSpeedMps
        : Parapenting::Physics::Length(RolloutState.velocityWorldMps);
    const float RunCycle = bPilotRunning
        ? FMath::Sin(static_cast<float>(
            SimulationTimeSeconds * (4.0 + PilotRunSpeed)))
        : 0.0f;
    const FVector LeftHip(1.0f, -14.0f, -13.0f);
    const FVector RightHip(1.0f, 14.0f, -13.0f);
    const FVector LeftKnee = bPilotRunning
        ? FVector(14.0f + 22.0f * RunCycle, -17.0f, -48.0f)
        : FVector(31.0f, -17.0f, -31.0f);
    const FVector RightKnee = bPilotRunning
        ? FVector(14.0f - 22.0f * RunCycle, 17.0f, -48.0f)
        : FVector(31.0f, 17.0f, -31.0f);
    const FVector LeftBoot = bPilotRunning
        ? FVector(16.0f - 34.0f * RunCycle, -18.0f, -88.0f)
        : FVector(60.0f, -18.0f, -47.0f);
    const FVector RightBoot = bPilotRunning
        ? FVector(16.0f + 34.0f * RunCycle, 18.0f, -88.0f)
        : FVector(60.0f, 18.0f, -47.0f);
    PosePilotSegment(LeftThigh, LeftHip, LeftKnee, 0.095f);
    PosePilotSegment(RightThigh, RightHip, RightKnee, 0.095f);
    PosePilotSegment(LeftShin, LeftKnee, LeftBoot, 0.085f);
    PosePilotSegment(RightShin, RightKnee, RightBoot, 0.085f);
}

void AParagliderPawn::BeginSuspensionMesh()
{
    SuspensionVertices.Reset();
    SuspensionTriangles.Reset();
    SuspensionNormals.Reset();
    SuspensionUVs.Reset();
    SuspensionColors.Reset();
}

void AParagliderPawn::AddSuspensionSegment(
    const FVector& Start, const FVector& End,
    const FColor& Color, float RadiusCm)
{
    const FVector Axis = End - Start;
    if (Axis.IsNearlyZero()) return;

    const FVector Direction = Axis.GetSafeNormal();
    const FVector Reference = FMath::Abs(Direction.Z) < 0.92f
        ? FVector::UpVector : FVector::RightVector;
    const FVector Side = FVector::CrossProduct(Direction, Reference)
        .GetSafeNormal() * RadiusCm;
    const FVector Up = FVector::CrossProduct(Side, Direction)
        .GetSafeNormal() * RadiusCm;
    const int32 Base = SuspensionVertices.Num();
    const FVector Radials[4] = {Side, Up, -Side, -Up};
    for (int32 Ring = 0; Ring < 2; ++Ring)
    {
        const FVector Centre = Ring == 0 ? Start : End;
        for (int32 Corner = 0; Corner < 4; ++Corner)
        {
            SuspensionVertices.Add(Centre + Radials[Corner]);
            SuspensionNormals.Add(Radials[Corner].GetSafeNormal());
            SuspensionUVs.Add(FVector2D(
                static_cast<float>(Corner) / 4.0f,
                static_cast<float>(Ring)));
            SuspensionColors.Add(Color);
        }
    }
    for (int32 Corner = 0; Corner < 4; ++Corner)
    {
        const int32 Next = (Corner + 1) % 4;
        SuspensionTriangles.Append({
            Base + Corner, Base + 4 + Corner, Base + 4 + Next,
            Base + Corner, Base + 4 + Next, Base + Next
        });
    }
}

void AParagliderPawn::CommitSuspensionMesh()
{
    TArray<FProcMeshTangent> Tangents;
    if (!bSuspensionMeshInitialized)
    {
        SuspensionVisual->CreateMeshSection(
            0, SuspensionVertices, SuspensionTriangles, SuspensionNormals,
            SuspensionUVs, SuspensionColors, Tangents, false);
        bSuspensionMeshInitialized = true;
    }
    else
    {
        SuspensionVisual->UpdateMeshSection(
            0, SuspensionVertices, SuspensionNormals,
            SuspensionUVs, SuspensionColors, Tangents);
    }
}

void AParagliderPawn::BuildCanopyMesh()
{
    // Rendering follows the 47-cell production-wing cadence while physics
    // remains free to use its lower station count. StationAt interpolates the
    // same authoritative canopy shape at this denser visual resolution.
    constexpr int32 SpanCount = 47;
    constexpr int32 ChordCount = 9;
    constexpr int32 SurfaceVertexCount = SpanCount * ChordCount;
    TArray<FVector> Vertices;
    TArray<int32> Triangles;
    TArray<FVector> Normals;
    TArray<FVector2D> UVs;
    TArray<FColor> Colors;
    TArray<FProcMeshTangent> Tangents;
    Vertices.SetNumZeroed(SurfaceVertexCount * 2);
    Normals.Init(FVector::UpVector, SurfaceVertexCount * 2);
    UVs.SetNum(SurfaceVertexCount * 2);
    Colors.SetNum(SurfaceVertexCount * 2);

    for (int32 S = 0; S < SpanCount; ++S)
    {
        for (int32 C = 0; C < ChordCount; ++C)
        {
            const int32 Index = S * ChordCount + C;
            const float Span01 =
                2.0f * static_cast<float>(S) / (SpanCount - 1) - 1.0f;
            const float Chord01 =
                static_cast<float>(C) / (ChordCount - 1);
            const float AbsSpan = FMath::Abs(Span01);
            constexpr float MetresToCm = static_cast<float>(
                Parapenting::Physics::WorldAxes::MetresToUnrealUnits);
            constexpr float SuspensionRiseCm = 730.0f;
            const Parapenting::Physics::RibStation RestStation =
                Canopy.StationAt(Span01);
            const float LocalChord =
                static_cast<float>(RestStation.chordM) * MetresToCm;
            Vertices[Index] = FVector(
                (0.48f - Chord01) * LocalChord,
                static_cast<float>(RestStation.positionM.y) * MetresToCm,
                SuspensionRiseCm
                    + static_cast<float>(RestStation.positionM.z) * MetresToCm
                    + static_cast<float>(
                        Canopy.InflatedSectionAt(Chord01).sagittaM)
                        * MetresToCm);
            // Ram-air intakes retain a real leading-edge gap. The old sine
            // section collapsed both skins to the same point at chord zero,
            // which made the wing read as a solid foam crescent.
            const float Thickness =
                FMath::Sin(Chord01 * PI) * (38.0f - 12.0f * AbsSpan)
                + FMath::Pow(1.0f - Chord01, 8.0f)
                    * (18.0f - 6.0f * AbsSpan);
            Vertices[Index + SurfaceVertexCount] =
                Vertices[Index] - FVector(0.0f, 0.0f, Thickness);
            UVs[Index] = FVector2D(
                static_cast<float>(C) / (ChordCount - 1),
                static_cast<float>(S) / (SpanCount - 1));
            UVs[Index + SurfaceVertexCount] = UVs[Index];
            const float TipPanel = FMath::SmoothStep(0.72f, 1.0f, AbsSpan);
            const float CentrePanel = 1.0f - FMath::SmoothStep(
                0.04f, 0.22f, AbsSpan);
            const float PanelValue = (S % 2) == 0 ? 1.0f : 0.91f;
            FLinearColor Panel = FMath::Lerp(
                FLinearColor(0.92f, 0.16f, 0.035f),
                FLinearColor(1.0f, 0.56f, 0.035f), CentrePanel);
            Panel = FMath::Lerp(
                Panel, FLinearColor(0.055f, 0.10f, 0.18f), TipPanel);
            Panel *= PanelValue;
            Colors[Index] = Panel.ToFColor(false);
            Colors[Index + SurfaceVertexCount] = Colors[Index];
        }
    }
    // Dark intake back-walls sit just behind the open leading edge. One quad
    // per rendered cell gives the 47-cell cadence a geometric cue without
    // painting fake holes onto the skin.
    for (int32 S = 0; S < SpanCount - 1; ++S)
    {
        const int32 UpperA = S * ChordCount;
        const int32 UpperB = (S + 1) * ChordCount;
        const int32 LowerA = UpperA + SurfaceVertexCount;
        const int32 LowerB = UpperB + SurfaceVertexCount;
        const int32 Base = Vertices.Num();
        Vertices.Append({
            FMath::Lerp(Vertices[UpperA], Vertices[UpperA + 1], 0.48f),
            FMath::Lerp(Vertices[UpperB], Vertices[UpperB + 1], 0.48f),
            FMath::Lerp(Vertices[LowerB], Vertices[LowerB + 1], 0.48f),
            FMath::Lerp(Vertices[LowerA], Vertices[LowerA + 1], 0.48f)
        });
        Triangles.Append({Base, Base + 1, Base + 2, Base, Base + 2, Base + 3});
        for (int32 Corner = 0; Corner < 4; ++Corner)
        {
            Normals.Add(FVector::ForwardVector);
            UVs.Add(FVector2D(
                Corner == 1 || Corner == 2 ? 1.0f : 0.0f,
                Corner >= 2 ? 1.0f : 0.0f));
            Colors.Add(FLinearColor(0.012f, 0.016f, 0.022f).ToFColor(false));
        }
    }
    for (int32 S = 0; S < SpanCount - 1; ++S)
    {
        for (int32 C = 0; C < ChordCount - 1; ++C)
        {
            const int32 A = S * ChordCount + C;
            const int32 B = (S + 1) * ChordCount + C;
            Triangles.Append({A, B, B + 1, A, B + 1, A + 1});
            const int32 LowerA = A + SurfaceVertexCount;
            const int32 LowerB = B + SurfaceVertexCount;
            Triangles.Append({
                LowerA, LowerB + 1, LowerB,
                LowerA, LowerA + 1, LowerB + 1
            });
        }
    }
    // Close both wing tips so the canopy silhouettes as an inflated volume.
    for (const int32 TipSpan : {0, SpanCount - 1})
    {
        for (int32 C = 0; C < ChordCount - 1; ++C)
        {
            const int32 UpperA = TipSpan * ChordCount + C;
            const int32 UpperB = UpperA + 1;
            const int32 LowerA = UpperA + SurfaceVertexCount;
            const int32 LowerB = UpperB + SurfaceVertexCount;
            if (TipSpan == 0)
                Triangles.Append({
                    UpperA, UpperB, LowerB,
                    UpperA, LowerB, LowerA
                });
            else
                Triangles.Append({
                    UpperA, LowerB, UpperB,
                    UpperA, LowerA, LowerB
                });
        }
    }
    CanopyVisual->CreateMeshSection(
        0, Vertices, Triangles, Normals, UVs, Colors, Tangents, false);
    TArray<FColor> GhostColors;
    GhostColors.Init(FColor(40, 210, 255, 150), Vertices.Num());
    GhostCanopyVisual->CreateMeshSection(
        0, Vertices, Triangles, Normals, UVs, GhostColors, Tangents, false);
    if (UMaterialInterface* Material =
        Parapenting::LoadCanopyMaterial())
    {
        CanopyVisual->SetMaterial(0, Material);
        GhostCanopyVisual->SetMaterial(0, Material);
    }
    if (UMaterialInterface* LineMaterial = Parapenting::LoadLineMaterial())
        SuspensionVisual->SetMaterial(0, LineMaterial);
    UpdateCanopyMesh();
}

void AParagliderPawn::UpdateCanopyMesh()
{
    constexpr int32 SpanCount = 47;
    constexpr int32 ChordCount = 9;
    constexpr int32 SurfaceVertexCount = SpanCount * ChordCount;
    // Base shape comes from the canopy geometry, which is derived from the
    // published EPIC 2 ML specification. The constants that used to live here
    // - half span 465 cm, root chord 280 cm, arch 650 cm dropping 150 cm at
    // the tips - were a second, unrelated description of the same wing.
    // Deformation below (collapse, cravat, load pose, brake, rotor) still
    // scales this base; only the undeformed shape moved.
    constexpr float MetresToCm =
        static_cast<float>(Parapenting::Physics::WorldAxes::MetresToUnrealUnits);
    // Height of the canopy above the harness, from the suspension geometry.
    constexpr float SuspensionRiseCm = 730.0f;
    const auto& T = Dynamics.LastTelemetry();
    const double VisualCanopyPressure = bLanded
        ? State.canopyPressure
        : (bGroundLaunching ? LaunchOutput.canopyPressure : T.canopyPressure);
    const auto LoadPose = Parapenting::Physics::EvaluateCanopyLoadPose(
        bGroundLaunching ? 0.0 : T.highLoadDeformation,
        bGroundLaunching ? 1.0 : T.loadFactor);
    TArray<FVector> Vertices;
    TArray<FVector> Normals;
    TArray<FVector2D> UVs;
    TArray<FColor> Colors;
    TArray<FProcMeshTangent> Tangents;
    Vertices.SetNumZeroed(SurfaceVertexCount * 2);
    Normals.Init(FVector::ZeroVector, SurfaceVertexCount * 2);
    UVs.SetNum(SurfaceVertexCount * 2);
    Colors.SetNum(SurfaceVertexCount * 2);

    for (int32 S = 0; S < SpanCount; ++S)
    {
        const float Span01 = 2.0f * static_cast<float>(S) / (SpanCount - 1) - 1.0f;
        const float AbsSpan = FMath::Abs(Span01);
        const float TipBlend = FMath::SmoothStep(0.42f, 1.0f, AbsSpan);
        const float Collapse = static_cast<float>(
            bGroundLaunching ? 0.0
            : (Span01 < 0.0f ? T.leftCollapse : T.rightCollapse)) * TipBlend;
        const float Cravat = static_cast<float>(
            bGroundLaunching ? 0.0
            : (Span01 < 0.0f ? T.leftCravat : T.rightCravat)) * TipBlend;
        const Parapenting::Physics::RibStation Station =
            Canopy.StationAt(Span01);
        const float ContractedSpan =
            static_cast<float>(Station.positionM.y) * MetresToCm
            * (1.0f - 0.28f * Collapse - 0.38f * Cravat)
            * static_cast<float>(LoadPose.spanScale);
        const float LoadedChord =
            static_cast<float>(Station.chordM) * MetresToCm
            * static_cast<float>(LoadPose.chordScale);
        // Station z is measured from the centre-cell apex and is negative
        // outboard, so the tips hang below the middle without a separate
        // droop term.
        const float Arch = SuspensionRiseCm
            + static_cast<float>(Station.positionM.z) * MetresToCm
            - static_cast<float>(LoadPose.extraArchDropCm)
                * FMath::Pow(AbsSpan, 1.65f)
            + static_cast<float>(LoadPose.lineStretchCm);
        const float CollapseDrop = 255.0f * Collapse + 330.0f * Cravat;
        const float RotorFlutter = static_cast<float>(
            bGroundLaunching ? 0.0 : T.rotorStrength) * TipBlend
            * 15.0f * FMath::Sin(
                static_cast<float>(SimulationTimeSeconds * 11.0 + Span01 * 8.0));
        for (int32 C = 0; C < ChordCount; ++C)
        {
            const float Chord01 = static_cast<float>(C) / (ChordCount - 1);
            const float Frontal = static_cast<float>(T.frontalCollapse)
                * (bGroundLaunching ? 0.0f : 1.0f)
                * (1.0f - FMath::SmoothStep(0.0f, 0.45f, Chord01));
            // Cell bulge, solved from the seam allowance and cell pressure
            // rather than a fixed amplitude. Scaled by the live canopy
            // pressure so a deflating cell loses its section.
            const float Camber = static_cast<float>(
                    Canopy.InflatedSectionAt(Chord01).sagittaM)
                * MetresToCm
                * static_cast<float>(VisualCanopyPressure)
                * static_cast<float>(LoadPose.camberScale)
                * (1.0f - 0.65f * Collapse) * (1.0f - 0.75f * Frontal);
            const float Brake = Span01 < 0.0f
                ? static_cast<float>(AppliedControls.leftBrake)
                : static_cast<float>(AppliedControls.rightBrake);
            const float TrailingEdge = FMath::SmoothStep(0.68f, 1.0f, Chord01)
                * Brake * 55.0f;
            const int32 Index = S * ChordCount + C;
            const FVector UpperVertex(
                (0.48f - Chord01) * LoadedChord
                    + static_cast<float>(
                        (bGroundLaunching ? 0.0 : T.recoverySurge) * 350.0)
                    - Frontal * 65.0f,
                ContractedSpan,
                Arch + Camber - CollapseDrop - TrailingEdge + RotorFlutter
                    + static_cast<float>(LoadPose.rippleAmplitudeCm)
                        * TipBlend * FMath::Sin(static_cast<float>(
                            SimulationTimeSeconds * 24.0
                            + Span01 * 18.0 + Chord01 * 3.0))
                    - Frontal * 125.0f
                    - static_cast<float>(
                        bGroundLaunching ? 0.0 : T.deepStall) * 75.0f);
            const float LocalPressure = FMath::Clamp(
                static_cast<float>(VisualCanopyPressure)
                    * (1.0f - 0.72f * Collapse - 0.82f * Cravat)
                    * (1.0f - 0.65f * Frontal),
                0.0f, 1.0f);
            const float TipPanel = FMath::SmoothStep(0.72f, 1.0f, AbsSpan);
            const float CentrePanel = 1.0f - FMath::SmoothStep(
                0.04f, 0.22f, AbsSpan);
            const float PanelValue = (S % 2) == 0 ? 1.0f : 0.91f;
            FLinearColor Inflated = FMath::Lerp(
                FLinearColor(0.92f, 0.16f, 0.035f),
                FLinearColor(1.0f, 0.56f, 0.035f), CentrePanel);
            Inflated = FMath::Lerp(
                Inflated, FLinearColor(0.055f, 0.10f, 0.18f), TipPanel);
            Inflated *= PanelValue;
            const FLinearColor Unloaded(0.08f, 0.055f, 0.045f);
            // Linear, not sRGB: mesh vertex colours reach the material
            // unconverted, so encoding here would gamma the skin twice and
            // render the striped canopy as near-white. See ParapentingTerrain.
            const FColor SkinColor = FLinearColor::LerpUsingHSV(
                Unloaded, Inflated,
                0.18f + 0.82f * LocalPressure).ToFColor(false);
            const float CellThickness = (
                FMath::Sin(Chord01 * PI) * (38.0f - 12.0f * AbsSpan)
                + FMath::Pow(1.0f - Chord01, 8.0f)
                    * (18.0f - 6.0f * AbsSpan)) * LocalPressure;
            Vertices[Index] = UpperVertex;
            Vertices[Index + SurfaceVertexCount] =
                UpperVertex - FVector(0.0f, 0.0f, CellThickness);
            UVs[Index] = FVector2D(Chord01,
                static_cast<float>(S) / (SpanCount - 1));
            UVs[Index + SurfaceVertexCount] = UVs[Index];
            Colors[Index] = SkinColor;
            Colors[Index + SurfaceVertexCount] = SkinColor;
        }
    }
    for (int32 S = 0; S < SpanCount - 1; ++S)
    {
        const int32 UpperA = S * ChordCount;
        const int32 UpperB = (S + 1) * ChordCount;
        const int32 LowerA = UpperA + SurfaceVertexCount;
        const int32 LowerB = UpperB + SurfaceVertexCount;
        Vertices.Append({
            FMath::Lerp(Vertices[UpperA], Vertices[UpperA + 1], 0.48f),
            FMath::Lerp(Vertices[UpperB], Vertices[UpperB + 1], 0.48f),
            FMath::Lerp(Vertices[LowerB], Vertices[LowerB + 1], 0.48f),
            FMath::Lerp(Vertices[LowerA], Vertices[LowerA + 1], 0.48f)
        });
        for (int32 Corner = 0; Corner < 4; ++Corner)
        {
            Normals.Add(FVector::ForwardVector);
            UVs.Add(FVector2D(
                Corner == 1 || Corner == 2 ? 1.0f : 0.0f,
                Corner >= 2 ? 1.0f : 0.0f));
            Colors.Add(FLinearColor(0.012f, 0.016f, 0.022f).ToFColor(false));
        }
    }
    auto AccumulateTriangleNormal =
        [&Vertices, &Normals](int32 A, int32 B, int32 C)
    {
        const FVector N = FVector::CrossProduct(
            Vertices[B] - Vertices[A],
            Vertices[C] - Vertices[A]).GetSafeNormal();
        Normals[A] += N;
        Normals[B] += N;
        Normals[C] += N;
    };
    for (int32 S = 0; S < SpanCount - 1; ++S)
    {
        for (int32 C = 0; C < ChordCount - 1; ++C)
        {
            const int32 A = S * ChordCount + C;
            const int32 B = (S + 1) * ChordCount + C;
            const int32 D = A + 1;
            const int32 E = B + 1;
            AccumulateTriangleNormal(A, B, E);
            AccumulateTriangleNormal(A, E, D);
            const int32 LowerA = A + SurfaceVertexCount;
            const int32 LowerB = B + SurfaceVertexCount;
            const int32 LowerD = D + SurfaceVertexCount;
            const int32 LowerE = E + SurfaceVertexCount;
            AccumulateTriangleNormal(LowerA, LowerE, LowerB);
            AccumulateTriangleNormal(LowerA, LowerD, LowerE);
        }
    }
    for (const int32 TipSpan : {0, SpanCount - 1})
    {
        for (int32 C = 0; C < ChordCount - 1; ++C)
        {
            const int32 UpperA = TipSpan * ChordCount + C;
            const int32 UpperB = UpperA + 1;
            const int32 LowerA = UpperA + SurfaceVertexCount;
            const int32 LowerB = UpperB + SurfaceVertexCount;
            if (TipSpan == 0)
            {
                AccumulateTriangleNormal(UpperA, UpperB, LowerB);
                AccumulateTriangleNormal(UpperA, LowerB, LowerA);
            }
            else
            {
                AccumulateTriangleNormal(UpperA, LowerB, UpperB);
                AccumulateTriangleNormal(UpperA, LowerA, LowerB);
            }
        }
    }
    for (FVector& Normal : Normals)
    {
        Normal.Normalize();
    }
    CanopyVisual->UpdateMeshSection(
        0, Vertices, Normals, UVs, Colors, Tangents);
}

void AParagliderPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);
    PlayerInputComponent->BindAction(
        TEXT("WeightShiftLeftStep"), IE_Pressed, this, &AParagliderPawn::StepWeightShiftLeft);
    PlayerInputComponent->BindAction(
        TEXT("WeightShiftRightStep"), IE_Pressed, this, &AParagliderPawn::StepWeightShiftRight);
    PlayerInputComponent->BindAction(
        TEXT("CenterWeightShift"), IE_Pressed, this,
        &AParagliderPawn::CenterWeightShift);
    PlayerInputComponent->BindAction(
        TEXT("LeftBrakeStep"), IE_Pressed, this, &AParagliderPawn::StepLeftBrake);
    PlayerInputComponent->BindAction(
        TEXT("RightBrakeStep"), IE_Pressed, this, &AParagliderPawn::StepRightBrake);
    PlayerInputComponent->BindAction(
        TEXT("BothBrakesMore"), IE_Pressed, this, &AParagliderPawn::StepBothBrakesMore);
    PlayerInputComponent->BindAction(
        TEXT("BrakesRelease"), IE_Pressed, this, &AParagliderPawn::StepBrakesRelease);
    PlayerInputComponent->BindAction(
        TEXT("ResetFlight"), IE_Pressed, this, &AParagliderPawn::ResetFlight);
    PlayerInputComponent->BindAction(
        TEXT("PrepareGroundLaunch"), IE_Pressed,
        this, &AParagliderPawn::PrepareGroundLaunch);
    PlayerInputComponent->BindAction(
        TEXT("PrepareReverseGroundLaunch"), IE_Pressed,
        this, &AParagliderPawn::PrepareReverseGroundLaunch);
    PlayerInputComponent->BindAction(
        TEXT("LaunchRun"), IE_Pressed, this, &AParagliderPawn::StartLaunchRun);
    PlayerInputComponent->BindAction(
        TEXT("LaunchRun"), IE_Released, this, &AParagliderPawn::StopLaunchRun);
    PlayerInputComponent->BindAction(
        TEXT("WeatherChill"), IE_Pressed, this, &AParagliderPawn::SetWeatherChill);
    PlayerInputComponent->BindAction(
        TEXT("WeatherRidge"), IE_Pressed, this, &AParagliderPawn::SetWeatherRidge);
    PlayerInputComponent->BindAction(
        TEXT("WeatherLocalizedRotor"), IE_Pressed, this, &AParagliderPawn::SetWeatherLocalizedRotor);
    PlayerInputComponent->BindAction(
        TEXT("WeatherRotorEverywhere"), IE_Pressed, this, &AParagliderPawn::SetWeatherRotorEverywhere);
    PlayerInputComponent->BindAction(
        TEXT("WingTraining"), IE_Pressed, this, &AParagliderPawn::SelectTrainingWing);
    PlayerInputComponent->BindAction(
        TEXT("WingEpic"), IE_Pressed, this, &AParagliderPawn::SelectEpicWing);
    PlayerInputComponent->BindAction(
        TEXT("WingSport"), IE_Pressed, this, &AParagliderPawn::SelectSportWing);
    PlayerInputComponent->BindAction(
        TEXT("WingEpsilon"), IE_Pressed, this, &AParagliderPawn::SelectEpsilonWing);
    PlayerInputComponent->BindAction(
        TEXT("CycleWing"), IE_Pressed, this, &AParagliderPawn::CycleWing);
    PlayerInputComponent->BindAction(
        TEXT("InjectLeftCollapse"), IE_Pressed, this, &AParagliderPawn::InjectLeftCollapse);
    PlayerInputComponent->BindAction(
        TEXT("InjectFrontalCollapse"), IE_Pressed, this, &AParagliderPawn::InjectFrontalCollapse);
    PlayerInputComponent->BindAction(
        TEXT("InjectRightCollapse"), IE_Pressed, this, &AParagliderPawn::InjectRightCollapse);
    PlayerInputComponent->BindAction(
        TEXT("PreviousRoute"), IE_Pressed, this, &AParagliderPawn::PreviousRoute);
    PlayerInputComponent->BindAction(
        TEXT("NextRoute"), IE_Pressed, this, &AParagliderPawn::NextRoute);
    PlayerInputComponent->BindAction(
        TEXT("ToggleTelemetry"), IE_Pressed, this, &AParagliderPawn::ToggleTelemetryRecording);
    PlayerInputComponent->BindAction(
        TEXT("ToggleReplayRecording"), IE_Pressed, this, &AParagliderPawn::ToggleReplayRecording);
    PlayerInputComponent->BindAction(
        TEXT("PlayReplay"), IE_Pressed, this, &AParagliderPawn::PlayReplay);
    PlayerInputComponent->BindAction(
        TEXT("PreviousReplay"), IE_Pressed, this, &AParagliderPawn::PreviousReplay);
    PlayerInputComponent->BindAction(
        TEXT("NextReplay"), IE_Pressed, this, &AParagliderPawn::NextReplay);
    PlayerInputComponent->BindAction(
        TEXT("ToggleGhost"), IE_Pressed, this, &AParagliderPawn::ToggleGhost);
    PlayerInputComponent->BindAction(
        TEXT("ToggleAirflow"), IE_Pressed,
        this, &AParagliderPawn::ToggleAirflowVisualization);
    PlayerInputComponent->BindAction(
        TEXT("ToggleGeometry"), IE_Pressed,
        this, &AParagliderPawn::ToggleGeometryVisualization);
    PlayerInputComponent->BindAction(
        TEXT("CycleCamera"), IE_Pressed, this, &AParagliderPawn::CycleCameraMode);
    PlayerInputComponent->BindAction(
        TEXT("CycleAccessibility"), IE_Pressed,
        this, &AParagliderPawn::CycleAccessibilityProfile);
    PlayerInputComponent->BindAction(
        TEXT("CycleKeyboardLayout"), IE_Pressed,
        this, &AParagliderPawn::CycleKeyboardLayout);
    PlayerInputComponent->BindAction(
        TEXT("CycleBindingAction"), IE_Pressed,
        this, &AParagliderPawn::CycleBindingAction);
    PlayerInputComponent->BindAction(
        TEXT("BeginBindingCapture"), IE_Pressed,
        this, &AParagliderPawn::BeginBindingCapture);
    PlayerInputComponent->BindAction(
        TEXT("CycleGraphicsProfile"), IE_Pressed,
        this, &AParagliderPawn::CycleGraphicsProfile);
    PlayerInputComponent->BindAction(
        TEXT("CycleHudMode"), IE_Pressed,
        this, &AParagliderPawn::CycleHudMode);
    PlayerInputComponent->BindAction(
        TEXT("TogglePreflightBriefing"), IE_Pressed,
        this, &AParagliderPawn::TogglePreflightBriefing);
    PlayerInputComponent->BindAction(
        TEXT("NextScenario"), IE_Pressed, this, &AParagliderPawn::NextTrainingScenario);
    PlayerInputComponent->BindAction(
        TEXT("WindSpeedDown"), IE_Pressed, this, &AParagliderPawn::WindSpeedDown);
    PlayerInputComponent->BindAction(
        TEXT("WindSpeedUp"), IE_Pressed, this, &AParagliderPawn::WindSpeedUp);
    PlayerInputComponent->BindAction(
        TEXT("WindRotateLeft"), IE_Pressed, this, &AParagliderPawn::WindRotateLeft);
    PlayerInputComponent->BindAction(
        TEXT("WindRotateRight"), IE_Pressed, this, &AParagliderPawn::WindRotateRight);
    PlayerInputComponent->BindAction(
        TEXT("AcceleratorMore"), IE_Pressed, this, &AParagliderPawn::AcceleratorMore);
    PlayerInputComponent->BindAction(
        TEXT("AcceleratorLess"), IE_Pressed, this, &AParagliderPawn::AcceleratorLess);
    PlayerInputComponent->BindAction(
        TEXT("CycleHarness"), IE_Pressed, this, &AParagliderPawn::CycleHarness);
    PlayerInputComponent->BindAction(
        TEXT("PilotMassDown"), IE_Pressed, this, &AParagliderPawn::PilotMassDown);
    PlayerInputComponent->BindAction(
        TEXT("PilotMassUp"), IE_Pressed, this, &AParagliderPawn::PilotMassUp);
    PlayerInputComponent->BindAction(
        TEXT("CycleBallast"), IE_Pressed, this, &AParagliderPawn::CycleBallast);
    PlayerInputComponent->BindAction(
        TEXT("CycleWingSize"), IE_Pressed, this, &AParagliderPawn::CycleWingSize);
    PlayerInputComponent->BindAction(
        TEXT("CycleBrakeTravel"), IE_Pressed, this, &AParagliderPawn::CycleBrakeTravel);
    PlayerInputComponent->BindAction(
        TEXT("CycleWeatherPreset"), IE_Pressed, this, &AParagliderPawn::CycleWeatherPreset);
    PlayerInputComponent->BindAction(
        TEXT("CycleTimeOfDay"), IE_Pressed,
        this, &AParagliderPawn::CycleTimeOfDay);
    PlayerInputComponent->BindAction(
        TEXT("FetchLiveWeather"), IE_Pressed, this, &AParagliderPawn::FetchLiveWeather);
    PlayerInputComponent->BindAxis(
        TEXT("ControllerLeftBrake"), this, &AParagliderPawn::SetControllerLeftBrake);
    PlayerInputComponent->BindAxis(
        TEXT("ControllerRightBrake"), this, &AParagliderPawn::SetControllerRightBrake);
    PlayerInputComponent->BindAxis(
        TEXT("ControllerWeightShift"), this, &AParagliderPawn::SetControllerWeightShift);
    PlayerInputComponent->BindAxis(
        TEXT("ControllerAccelerator"), this, &AParagliderPawn::SetControllerAccelerator);
}

bool AParagliderPawn::IsKeyDown(const FKey& Key) const
{
    const APlayerController* Controller = Cast<APlayerController>(GetController());
    return Controller && Controller->IsInputKeyDown(Key);
}

void AParagliderPawn::StepWeightShiftLeft()
{
    Controls.weightShift = FMath::Clamp(Controls.weightShift - ControlStep, -1.0, 1.0);
}

void AParagliderPawn::StepWeightShiftRight()
{
    Controls.weightShift = FMath::Clamp(Controls.weightShift + ControlStep, -1.0, 1.0);
}

void AParagliderPawn::CenterWeightShift()
{
    Controls.weightShift = 0.0;
    ControllerControls.weightShift = 0.0;
}

void AParagliderPawn::StepLeftBrake()
{
    if (IsKeyDown(EKeys::Up))
        Controls.leftBrake = FMath::Max(0.0, Controls.leftBrake - ControlStep);
    else
        Controls.leftBrake = FMath::Min(1.0, Controls.leftBrake + ControlStep);
}

void AParagliderPawn::StepRightBrake()
{
    if (IsKeyDown(EKeys::Up))
        Controls.rightBrake = FMath::Max(0.0, Controls.rightBrake - ControlStep);
    else
        Controls.rightBrake = FMath::Min(1.0, Controls.rightBrake + ControlStep);
}

void AParagliderPawn::StepBothBrakesMore()
{
    if (IsKeyDown(EKeys::Left) || IsKeyDown(EKeys::Right)) return;
    Controls.leftBrake = FMath::Min(1.0, Controls.leftBrake + ControlStep);
    Controls.rightBrake = FMath::Min(1.0, Controls.rightBrake + ControlStep);
}

void AParagliderPawn::StepBrakesRelease()
{
    const bool LeftHeld = IsKeyDown(EKeys::Left);
    const bool RightHeld = IsKeyDown(EKeys::Right);

    if (LeftHeld || RightHeld)
    {
        if (LeftHeld)
            Controls.leftBrake = FMath::Max(0.0, Controls.leftBrake - ControlStep);
        if (RightHeld)
            Controls.rightBrake = FMath::Max(0.0, Controls.rightBrake - ControlStep);
        return;
    }

    Controls.leftBrake = FMath::Max(0.0, Controls.leftBrake - ControlStep);
    Controls.rightBrake = FMath::Max(0.0, Controls.rightBrake - ControlStep);
}

void AParagliderPawn::SetControllerLeftBrake(float Value)
{
    ControllerControls.leftBrake = FMath::Clamp(static_cast<double>(Value), 0.0, 1.0);
}

void AParagliderPawn::AcceleratorMore()
{
    Controls.accelerator = FMath::Min(1.0, Controls.accelerator + ControlStep);
}

void AParagliderPawn::AcceleratorLess()
{
    Controls.accelerator = FMath::Max(0.0, Controls.accelerator - ControlStep);
}

void AParagliderPawn::SetControllerAccelerator(float Value)
{
    // Right stick forward is full bar and the dead-zone leaves trim untouched.
    ControllerControls.accelerator = FMath::Clamp(
        static_cast<double>(Value), 0.0, 1.0);
}

void AParagliderPawn::SetControllerRightBrake(float Value)
{
    ControllerControls.rightBrake = FMath::Clamp(static_cast<double>(Value), 0.0, 1.0);
}

void AParagliderPawn::SetControllerWeightShift(float Value)
{
    const double Shift = FMath::Abs(Value) < 0.12f ? 0.0 : static_cast<double>(Value);
    ControllerControls.weightShift = FMath::Clamp(Shift, -1.0, 1.0);
}

void AParagliderPawn::SetWeatherChill()
{
    AirModel.SetMode(Parapenting::Physics::WeatherMode::Chill);
    bLiveWeatherActive = false;
    LiveWeatherStatus = TEXT("MANUAL WEATHER");
}

void AParagliderPawn::SetWeatherRidge()
{
    AirModel.SetMode(Parapenting::Physics::WeatherMode::Ridge);
    bLiveWeatherActive = false;
    LiveWeatherStatus = TEXT("MANUAL WEATHER");
}

void AParagliderPawn::SetWeatherLocalizedRotor()
{
    AirModel.SetMode(Parapenting::Physics::WeatherMode::LocalizedRotor);
    bLiveWeatherActive = false;
    LiveWeatherStatus = TEXT("MANUAL WEATHER");
}

void AParagliderPawn::SetWeatherRotorEverywhere()
{
    AirModel.SetMode(Parapenting::Physics::WeatherMode::RotorEverywhere);
    bLiveWeatherActive = false;
    LiveWeatherStatus = TEXT("MANUAL WEATHER");
}

void AParagliderPawn::SelectWing(Parapenting::Physics::WingProfileId Id)
{
    SelectedWing = Id;
    ApplyEquipmentConfiguration();
    ResetFlight();
}

void AParagliderPawn::ApplyEquipmentConfiguration()
{
    const auto& Wing = Parapenting::Physics::GetWingProfile(SelectedWing);
    const auto Configured = Parapenting::Physics::ConfigureWing(
        Wing, SelectedWingSize, SelectedBrakeTravel);
    const double WingMass = Parapenting::Physics::ConfiguredWingMassKg(
        Wing, SelectedWingSize);
    Dynamics.SetParameters(Parapenting::Physics::ApplyEquipmentSetup(
        Configured, Equipment, WingMass));
    Dynamics.SetHarnessParameters(
        Parapenting::Physics::HarnessParametersFor(Equipment));
    // Level 3: the harness the pilot is strapped into and what they weigh.
    // Weight-shift authority now comes from this geometry, so cycling the
    // harness changes the handling because the mass can move further, not
    // because an authority multiplier was raised.
    Dynamics.SetHarnessGeometry(
        Parapenting::Physics::HarnessGeometryFor(Equipment));
    Dynamics.SetPayloadMass(
        Parapenting::Physics::PayloadMassFor(Equipment));
    LineGraph.plan.harness =
        Parapenting::Physics::HarnessGeometryFor(Equipment);
    // Every pendulum period in the flight model comes from this one length,
    // and it is measured on the built suspension rather than written down as
    // a constant beside it.
    Dynamics.SetSuspensionLengthM(
        Parapenting::Physics::SuspensionPendulumLengthM(LineGraph));
    SolveSuspensionGraph();
}

void AParagliderPawn::CycleWingSize()
{
    SelectedWingSize = static_cast<Parapenting::Physics::WingSize>(
        (static_cast<int32>(SelectedWingSize) + 1) % 3);
    ApplyEquipmentConfiguration();
    ResetFlight();
}

void AParagliderPawn::CycleBrakeTravel()
{
    SelectedBrakeTravel = static_cast<Parapenting::Physics::BrakeTravel>(
        (static_cast<int32>(SelectedBrakeTravel) + 1) % 3);
    ApplyEquipmentConfiguration();
    ResetFlight();
}

void AParagliderPawn::CycleWeatherPreset()
{
    const auto& Presets = Parapenting::Physics::GetWeatherPresets();
    int32 Current = INDEX_NONE;
    for (int32 Index = 0; Index < static_cast<int32>(Presets.size()); ++Index)
    {
        if (Presets[Index].id == AirModel.GetPresetId())
        {
            Current = Index;
            break;
        }
    }
    const int32 Next = (Current + 1) % static_cast<int32>(Presets.size());
    AirModel.SetPreset(Presets[Next].id);
    ManualWindFromDegrees = AirModel.GetSnapshot().windFromDegrees;
    ManualWindSpeedMps = AirModel.GetSnapshot().windSpeedMps;
    bLiveWeatherActive = false;
    LiveWeatherStatus = TEXT("OFFLINE PRESET");
    ResetFlight();
}

void AParagliderPawn::CycleTimeOfDay()
{
    AirModel.SetStartLocalHour(
        AirModel.GetStartLocalHour() + 3.0);
    bLiveWeatherActive = false;
    LiveWeatherStatus = TEXT("OFFLINE TIME OVERRIDE");
    ResetFlight();
}

void AParagliderPawn::SetVisualQALocalHour(double LocalHour)
{
    AirModel.SetStartLocalHour(LocalHour);
    bLiveWeatherActive = false;
    LiveWeatherStatus = TEXT("VISUAL QA TIME OVERRIDE");
    ResetFlight();
}

void AParagliderPawn::FetchLiveWeather()
{
    LiveWeatherStatus = TEXT("FETCHING OPEN-METEO MODEL...");
    TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Request =
        FHttpModule::Get().CreateRequest();
    Request->SetVerb(TEXT("GET"));
    Request->SetURL(
        TEXT("https://api.open-meteo.com/v1/forecast?"
             "latitude=46.6863&longitude=7.8632&"
             "current=wind_speed_10m,wind_direction_10m,wind_gusts_10m&"
             "wind_speed_unit=ms&timeformat=unixtime&timezone=UTC"));
    Request->SetHeader(TEXT("User-Agent"), TEXT("Parapenting-Simulator/0.4"));
    Request->OnProcessRequestComplete().BindUObject(
        this, &AParagliderPawn::HandleLiveWeatherResponse);
    if (!Request->ProcessRequest())
        LiveWeatherStatus = TEXT("REQUEST COULD NOT START");
}

void AParagliderPawn::HandleLiveWeatherResponse(
    FHttpRequestPtr Request, FHttpResponsePtr Response, bool bSucceeded)
{
    (void)Request;
    if (bSucceeded && Response.IsValid()
        && EHttpResponseCodes::IsOk(Response->GetResponseCode())
        && ApplyLiveWeatherJson(Response->GetContentAsString(), false))
    {
        const FString Directory = FPaths::Combine(
            FPaths::ProjectSavedDir(), TEXT("Weather"));
        FPlatformFileManager::Get().GetPlatformFile().CreateDirectoryTree(
            *Directory);
        FFileHelper::SaveStringToFile(
            Response->GetContentAsString(),
            *FPaths::Combine(Directory, TEXT("interlaken-current.json")));
        return;
    }

    FString Cached;
    const FString CachePath = FPaths::Combine(
        FPaths::ProjectSavedDir(),
        TEXT("Weather/interlaken-current.json"));
    if (FFileHelper::LoadFileToString(Cached, *CachePath)
        && ApplyLiveWeatherJson(Cached, true))
        return;
    LiveWeatherStatus = TEXT("LIVE WEATHER UNAVAILABLE - PRESET RETAINED");
}

bool AParagliderPawn::ApplyLiveWeatherJson(
    const FString& Json, bool bFromCache)
{
    TSharedPtr<FJsonObject> RootObject;
    const TSharedRef<TJsonReader<>> Reader =
        TJsonReaderFactory<>::Create(Json);
    if (!FJsonSerializer::Deserialize(Reader, RootObject)
        || !RootObject.IsValid())
        return false;

    const TSharedPtr<FJsonObject>* Current = nullptr;
    if (!RootObject->TryGetObjectField(TEXT("current"), Current)
        || !Current || !Current->IsValid())
        return false;

    double Time = 0.0;
    double Speed = 0.0;
    double Direction = 0.0;
    double Gust = 0.0;
    if (!(*Current)->TryGetNumberField(TEXT("time"), Time)
        || !(*Current)->TryGetNumberField(TEXT("wind_speed_10m"), Speed)
        || !(*Current)->TryGetNumberField(
            TEXT("wind_direction_10m"), Direction))
        return false;
    if (!(*Current)->TryGetNumberField(TEXT("wind_gusts_10m"), Gust))
        Gust = Speed;
    if (!FMath::IsFinite(Speed) || !FMath::IsFinite(Direction)
        || !FMath::IsFinite(Gust)
        || Speed < 0.0 || Speed > 45.0 || Gust < 0.0 || Gust > 70.0)
        return false;

    Parapenting::Physics::WeatherSnapshot Snapshot;
    Snapshot.source = "open-meteo";
    Snapshot.displayName = bFromCache
        ? "CACHED OPEN-METEO MODEL" : "LIVE OPEN-METEO MODEL";
    Snapshot.observedUnixSeconds = Time;
    Snapshot.windFromDegrees = Direction;
    Snapshot.windSpeedMps = Speed;
    Snapshot.gustSpeedMps = FMath::Max(Speed, Gust);
    Snapshot.thermalTopMslM = 0.0;
    Snapshot.deterministicSeed =
        static_cast<unsigned int>(FMath::Max(1.0, FMath::Fmod(Time, 4294967294.0)));
    AirModel.ApplySnapshot(Snapshot);
    ManualWindFromDegrees = Direction;
    ManualWindSpeedMps = Speed;
    LiveWeatherObservedUnixSeconds = Time;
    bLiveWeatherActive = true;
    LiveWeatherStatus = bFromCache
        ? TEXT("CACHED MODEL - NETWORK UNAVAILABLE")
        : TEXT("LIVE MODEL LOADED");
    ResetFlight();
    return true;
}

void AParagliderPawn::CycleHarness()
{
    const int32 Next =
        (static_cast<int32>(Equipment.harness) + 1) % 3;
    Equipment.harness =
        static_cast<Parapenting::Physics::HarnessType>(Next);
    ApplyEquipmentConfiguration();
    ResetFlight();
}

void AParagliderPawn::PilotMassDown()
{
    Equipment.pilotMassKg = FMath::Max(50.0, Equipment.pilotMassKg - 5.0);
    ApplyEquipmentConfiguration();
    ResetFlight();
}

void AParagliderPawn::PilotMassUp()
{
    Equipment.pilotMassKg = FMath::Min(120.0, Equipment.pilotMassKg + 5.0);
    ApplyEquipmentConfiguration();
    ResetFlight();
}

void AParagliderPawn::CycleBallast()
{
    Equipment.ballastKg = Equipment.ballastKg >= 10.0
        ? 0.0 : Equipment.ballastKg + 5.0;
    ApplyEquipmentConfiguration();
    ResetFlight();
}

const char* AParagliderPawn::GetHarnessDisplayName() const
{
    return Parapenting::Physics::GetHarnessProfile(
        Equipment.harness).displayName;
}

const char* AParagliderPawn::GetWingSizeDisplayName() const
{
    return Parapenting::Physics::GetWingSizeVariant(
        SelectedWingSize).displayName;
}

const char* AParagliderPawn::GetBrakeTravelDisplayName() const
{
    return Parapenting::Physics::GetBrakeTravelVariant(
        SelectedBrakeTravel).displayName;
}

const char* AParagliderPawn::GetWeatherPresetDisplayName() const
{
    if (AirModel.GetPresetId()
        == Parapenting::Physics::WeatherPresetId::Custom)
    {
        return bLiveWeatherActive
            ? AirModel.GetSnapshot().displayName : "CUSTOM";
    }
    return Parapenting::Physics::GetWeatherPreset(
        AirModel.GetPresetId()).displayName;
}

FString AParagliderPawn::GetLocalTimeDisplay() const
{
    const double HourValue = GetLocalTimeHours();
    int32 Hour = FMath::FloorToInt(HourValue);
    int32 Minute = FMath::RoundToInt((HourValue - Hour) * 60.0);
    if (Minute >= 60)
    {
        Minute = 0;
        Hour = (Hour + 1) % 24;
    }
    return FString::Printf(TEXT("%02d:%02d"), Hour, Minute);
}

double AParagliderPawn::GetLiveWeatherAgeMinutes() const
{
    if (LiveWeatherObservedUnixSeconds <= 0.0) return 0.0;
    return FMath::Max(0.0,
        (static_cast<double>(FDateTime::UtcNow().ToUnixTimestamp())
         - LiveWeatherObservedUnixSeconds) / 60.0);
}

bool AParagliderPawn::IsWithinRecommendedWingRange() const
{
    const auto& Wing = Parapenting::Physics::GetWingProfile(SelectedWing);
    const double AllUp = Parapenting::Physics::AllUpMassKg(
        Equipment, Parapenting::Physics::ConfiguredWingMassKg(
            Wing, SelectedWingSize));
    return AllUp >= Parapenting::Physics::ConfiguredRangeMinKg(
            Wing, SelectedWingSize)
        && AllUp <= Parapenting::Physics::ConfiguredRangeMaxKg(
            Wing, SelectedWingSize);
}

void AParagliderPawn::SelectTrainingWing()
{
    SelectWing(Parapenting::Physics::WingProfileId::TrainingA);
}

void AParagliderPawn::SelectEpicWing()
{
    SelectWing(Parapenting::Physics::WingProfileId::Epic2MLResearch);
}

void AParagliderPawn::SelectSportWing()
{
    SelectWing(Parapenting::Physics::WingProfileId::SportB);
}

void AParagliderPawn::SelectEpsilonWing()
{
    SelectWing(
        Parapenting::Physics::WingProfileId::AdvanceEpsilonDls28Research);
}

void AParagliderPawn::CycleWing()
{
    const auto& Profiles = Parapenting::Physics::GetWingProfiles();
    std::size_t CurrentIndex = 0;
    for (std::size_t Index = 0; Index < Profiles.size(); ++Index)
    {
        if (Profiles[Index].id == SelectedWing)
        {
            CurrentIndex = Index;
            break;
        }
    }
    SelectWing(Profiles[(CurrentIndex + 1) % Profiles.size()].id);
}

const char* AParagliderPawn::GetWingDisplayName() const
{
    return Parapenting::Physics::GetWingProfile(SelectedWing).displayName;
}

const char* AParagliderPawn::GetRouteDisplayName() const
{
    return Parapenting::Physics::GetRouteProfileByIndex(
        SelectedRouteIndex).displayName;
}

Parapenting::Physics::SiteWindAssessment
AParagliderPawn::GetSiteWindAssessment() const
{
    const auto& Route = Parapenting::Physics::GetRouteProfileByIndex(
        SelectedRouteIndex);
    const auto Wind = AirModel.GetBaseWind();
    return Parapenting::Physics::AssessRouteWind(
        Route,
        Parapenting::Physics::MeteorologicalDirectionFromWindVector(Wind),
        std::hypot(Wind.x, Wind.y));
}

Parapenting::Physics::PreflightBriefing
AParagliderPawn::GetPreflightBriefing() const
{
    const auto& Route = Parapenting::Physics::GetRouteProfileByIndex(
        SelectedRouteIndex);
    auto Launch = Parapenting::Physics::RouteLaunchLocalM(Route);
    auto Landing = Parapenting::Physics::RouteLandingLocalM(Route);
    Launch.z = Parapenting::Physics::TerrainModel::HeightM(
        Launch.x, Launch.y) + 2.0;
    Landing.z = Parapenting::Physics::TerrainModel::HeightM(
        Landing.x, Landing.y) + 6.0;
    Parapenting::Physics::Vec3 Cruise =
        (Launch + Landing) * 0.5;
    Cruise.z = Parapenting::Physics::TerrainModel::HeightM(
        Cruise.x, Cruise.y) + 450.0;
    return Parapenting::Physics::EvaluatePreflightBriefing(
        Route, AirModel.GetSnapshot(), AirModel.GetParameters(),
        AirModel.GetVolumes(),
        AirModel.SampleCloudField(SimulationTimeSeconds),
        AirModel.Sample(Launch, SimulationTimeSeconds),
        AirModel.Sample(Landing, SimulationTimeSeconds),
        AirModel.Sample(Cruise, SimulationTimeSeconds));
}

Parapenting::Physics::GlideNavigationSolution
AParagliderPawn::GetNavigationSolution() const
{
    const std::size_t Index = FMath::Min<std::size_t>(
        NavigationProgress.activeWaypoint,
        NavigationRoute.waypoints.size() - 1);
    const double SymmetricBrake =
        0.5 * (AppliedControls.leftBrake + AppliedControls.rightBrake);
    const auto& Parameters = Dynamics.Parameters();
    const double EffectiveBrake = FMath::Clamp(
        (SymmetricBrake - Parameters.brakeFreePlayFraction)
            / FMath::Max(0.1, 1.0 - Parameters.brakeFreePlayFraction),
        0.0, 1.0);
    const auto Polar =
        Parapenting::Physics::EstimateSteadyPolarPoint(
            Parameters, EffectiveBrake);
    const auto Air = AirModel.Sample(
        State.positionWorldM, SimulationTimeSeconds);
    return Parapenting::Physics::EvaluateGlideNavigation(
        State.positionWorldM, NavigationRoute.waypoints[Index],
        Air.windWorldMps, Polar, Dynamics.LastTelemetry().airspeedMps,
        Air.windWorldMps.z);
}

const char* AParagliderPawn::GetActiveWaypointName() const
{
    if (NavigationProgress.complete) return "ROUTE COMPLETE";
    const std::size_t Index = FMath::Min<std::size_t>(
        NavigationProgress.activeWaypoint,
        NavigationRoute.waypoints.size() - 1);
    return NavigationRoute.waypoints[Index].displayName;
}

const char* AParagliderPawn::GetSiteWindAssessmentName() const
{
    return Parapenting::Physics::SiteWindAssessmentName(
        GetSiteWindAssessment());
}

const char* AParagliderPawn::GetLaunchHazardText() const
{
    return Parapenting::Physics::GetRouteProfileByIndex(
        SelectedRouteIndex).launchHazard;
}

const char* AParagliderPawn::GetLandingCircuitText() const
{
    return Parapenting::Physics::GetRouteProfileByIndex(
        SelectedRouteIndex).landingCircuit;
}

const char* AParagliderPawn::GetLandingDisplayName() const
{
    return Parapenting::Physics::GetRouteProfileByIndex(
        SelectedRouteIndex).landing.displayName;
}

double AParagliderPawn::GetLandingElevationM() const
{
    // All regional simulation lanes retain the primary 565 m MSL vertical
    // datum even when translated horizontally for sparse rendering.
    return 565.0;
}

void AParagliderPawn::PreviousRoute()
{
    const std::size_t Count = Parapenting::Physics::RouteProfileCount();
    SelectedRouteIndex = (SelectedRouteIndex + Count - 1) % Count;
    ResetFlight();
}

void AParagliderPawn::NextRoute()
{
    SelectedRouteIndex =
        (SelectedRouteIndex + 1) % Parapenting::Physics::RouteProfileCount();
    ResetFlight();
}

void AParagliderPawn::ToggleTelemetryRecording()
{
    bRecordingTelemetry = !bRecordingTelemetry;
    TelemetryAccumulatorSeconds = 0.0;
    if (!bRecordingTelemetry) return;

    const FString Directory = FPaths::Combine(
        FPaths::ProjectSavedDir(), TEXT("Telemetry"));
    IPlatformFile& PlatformFile =
        FPlatformFileManager::Get().GetPlatformFile();
    PlatformFile.CreateDirectoryTree(*Directory);
    TelemetryFilePath = FPaths::Combine(
        Directory,
        FString::Printf(TEXT("flight-%s.csv"),
            *FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S"))));
    FFileHelper::SaveStringToFile(
        TEXT("time_s,route,wing,scenario,weather_preset,x_m,y_m,altitude_msl_m,agl_m,vx_mps,vy_mps,"
             "vz_mps,airspeed_mps,left_brake,right_brake,weight_shift,"
             "accelerator,"
             "brake_load_left,brake_load_right,load_factor,thermal_mps,rotor,"
             "left_collapse,right_collapse,frontal_collapse,left_cravat,"
             "right_cravat,deep_stall,spin,canopy_pressure,"
             "spanwise_load_asymmetry,a_riser,b_riser,c_riser,d_riser,"
             "line_load_n,all_up_mass_kg,wing_loading_kg_m2,harness_drag_n,"
             "brake_travel_left_mm,brake_travel_right_mm,"
             "challenge_score,challenge_progress,left_reinflation,"
             "right_reinflation,surge_containment,recovery_surge,"
             "brake_force_left_n,brake_force_right_n,high_load_deformation,"
             "spanwise_airflow_shear_mps,left_airflow_disturbance,"
             "right_airflow_disturbance,low_frequency_gust_mps,"
             "high_frequency_gust_mps,gust_energy_mps,ground_effect,"
             "flare_energy,flare_authority,flare_lift_delta_cl,"
             "collapse_resistance,passive_reinflation_rate,"
             "frontal_reinflation_rate,cravat_susceptibility,rollout_phase,"
             "runout_distance_m,flight_phase,"
             "safety_rating,efficiency_rating,thermal_rating,"
             "landing_rating,overall_rating\n"),
        *TelemetryFilePath);
}

void AParagliderPawn::ToggleReplayRecording()
{
    if (bRecordingReplay)
    {
        bRecordingReplay = false;
        bGhostVisible = !GhostFrames.IsEmpty();
        SaveReplayManifest();
        return;
    }

    bPlayingReplay = false;
    bGhostVisible = false;
    ReplayFrames.Reset();
    GhostFrames.Reset();
    GhostCaptureStep = 0;
    ReplayWing = SelectedWing;
    ReplayRouteIndex = SelectedRouteIndex;
    ReplayScenarioIndex = SelectedScenarioIndex;
    ReplayWeatherMode = AirModel.GetMode();
    ReplayWeatherPreset = AirModel.GetPresetId();
    ReplayStartLocalHour = AirModel.GetStartLocalHour();
    ReplayWindFromDegrees = ManualWindFromDegrees;
    ReplayWindSpeedMps = ManualWindSpeedMps;
    ReplayEquipment = Equipment;
    ReplayWingSize = SelectedWingSize;
    ReplayBrakeTravel = SelectedBrakeTravel;
    ResetFlight();
    bRecordingReplay = true;
}

void AParagliderPawn::PlayReplay()
{
    if (ReplayFrames.IsEmpty()) return;
    bRecordingReplay = false;
    bGhostVisible = false;
    SelectedRouteIndex = ReplayRouteIndex;
    SelectedScenarioIndex = ReplayScenarioIndex;
    SelectWing(ReplayWing);
    if (ReplayWeatherPreset
        == Parapenting::Physics::WeatherPresetId::Custom)
        AirModel.SetMode(ReplayWeatherMode);
    else
        AirModel.SetPreset(ReplayWeatherPreset);
    AirModel.SetStartLocalHour(ReplayStartLocalHour);
    ManualWindFromDegrees = ReplayWindFromDegrees;
    ManualWindSpeedMps = ReplayWindSpeedMps;
    Equipment = ReplayEquipment;
    SelectedWingSize = ReplayWingSize;
    SelectedBrakeTravel = ReplayBrakeTravel;
    ApplyEquipmentConfiguration();
    if (ReplayWeatherPreset
        == Parapenting::Physics::WeatherPresetId::Custom)
        ApplyManualWind();
    ResetFlight();
    ReplayFrameIndex = 0;
    bPlayingReplay = true;
}

void AParagliderPawn::PreviousReplay()
{
    if (ReplayFiles.IsEmpty() || bRecordingReplay) return;
    SelectedReplayFileIndex =
        (SelectedReplayFileIndex + ReplayFiles.Num() - 1) % ReplayFiles.Num();
    LoadReplayFile(ReplayFiles[SelectedReplayFileIndex]);
}

void AParagliderPawn::NextReplay()
{
    if (ReplayFiles.IsEmpty() || bRecordingReplay) return;
    SelectedReplayFileIndex =
        (SelectedReplayFileIndex + 1) % ReplayFiles.Num();
    LoadReplayFile(ReplayFiles[SelectedReplayFileIndex]);
}

void AParagliderPawn::ToggleGhost()
{
    if (!GhostFrames.IsEmpty() && !bRecordingReplay)
        bGhostVisible = !bGhostVisible;
}

void AParagliderPawn::DrawCanopyGeometryDebug()
{
    UWorld* World = GetWorld();
    if (!World) return;

    constexpr float MetresToCm = static_cast<float>(
        Parapenting::Physics::WorldAxes::MetresToUnrealUnits);
    constexpr float SuspensionRiseCm = 730.0f;
    const FTransform ActorTransform = GetActorTransform();
    const auto ToWorld = [&](const Parapenting::Physics::Vec3& body)
    {
        return ActorTransform.TransformPosition(FVector(
            static_cast<float>(body.x) * MetresToCm,
            static_cast<float>(body.y) * MetresToCm,
            SuspensionRiseCm + static_cast<float>(body.z) * MetresToCm));
    };

    // Rib stations: the developed wing's skeleton, leading edge to trailing
    // edge at every rib. This is the shape everything else is derived from.
    const auto& Ribs = Canopy.Ribs();
    for (std::size_t Index = 0; Index < Ribs.size(); ++Index)
    {
        const double Span = Ribs[Index].spanFraction;
        const FVector LeadingEdge =
            ToWorld(Canopy.SurfacePointM(Span, 0.0, true));
        const FVector TrailingEdge =
            ToWorld(Canopy.SurfacePointM(Span, 1.0, true));
        // Centre rib and tips picked out so the arc apex is unmistakable.
        const bool bLandmark = Index == 0 || Index + 1 == Ribs.size()
            || std::fabs(Span) < 1.0 / static_cast<double>(Ribs.size());
        DrawDebugLine(World, LeadingEdge, TrailingEdge,
            bLandmark ? FColor(255, 220, 60) : FColor(90, 130, 170),
            false, 0.0f, 0, bLandmark ? 2.2f : 0.8f);
    }

    // Leading and trailing edges, and the solved cell section at mid-chord.
    // The section is drawn at its true sagitta, so a seam allowance change is
    // visible directly rather than only in a number.
    FVector PreviousLeading = FVector::ZeroVector;
    FVector PreviousTrailing = FVector::ZeroVector;
    FVector PreviousCrown = FVector::ZeroVector;
    for (int32 Step = 0; Step <= 96; ++Step)
    {
        const double Span = -1.0 + 2.0 * static_cast<double>(Step) / 96.0;
        const FVector Leading = ToWorld(Canopy.SurfacePointM(Span, 0.0, true));
        const FVector Trailing = ToWorld(Canopy.SurfacePointM(Span, 1.0, true));
        const FVector Crown =
            ToWorld(Canopy.InflatedSurfacePointM(Span, 0.45, 0.5, true));
        if (Step > 0)
        {
            DrawDebugLine(World, PreviousLeading, Leading,
                FColor(255, 120, 40), false, 0.0f, 0, 2.0f);
            DrawDebugLine(World, PreviousTrailing, Trailing,
                FColor(120, 200, 255), false, 0.0f, 0, 1.6f);
            DrawDebugLine(World, PreviousCrown, Crown,
                FColor(160, 255, 160), false, 0.0f, 0, 1.2f);
        }
        PreviousLeading = Leading;
        PreviousTrailing = Trailing;
        PreviousCrown = Crown;
    }

    // Attachment nodes, coloured and labelled by row. The exit gate is that
    // every line ends on the canopy, so drawing the node where the geometry
    // says it is makes a mismatch immediately visible.
    const auto& Suspension =
        Parapenting::Physics::Epic2MlSuspensionGeometry();
    for (const auto& Attachment : Suspension.attachments)
    {
        const FVector Node = ToWorld(Canopy.SurfacePointM(
            Attachment.spanFraction, Attachment.chordFraction, false));
        FColor Colour = FColor::White;
        const TCHAR* Label = TEXT("?");
        switch (Attachment.group)
        {
        case Parapenting::Physics::SuspensionGroup::A:
            Colour = FColor(255, 70, 60); Label = TEXT("A"); break;
        case Parapenting::Physics::SuspensionGroup::BabyA:
            Colour = FColor(255, 150, 60); Label = TEXT("A'"); break;
        case Parapenting::Physics::SuspensionGroup::B:
            Colour = FColor(70, 130, 255); Label = TEXT("B"); break;
        case Parapenting::Physics::SuspensionGroup::C:
            Colour = FColor(90, 220, 120); Label = TEXT("C"); break;
        case Parapenting::Physics::SuspensionGroup::Brake:
            Colour = FColor(230, 230, 90); Label = TEXT("BR"); break;
        }
        DrawDebugSphere(World, Node, 9.0f, 8, Colour, false, 0.0f, 0, 1.4f);
        DrawDebugString(World, Node + FVector(0.0f, 0.0f, 16.0f),
            FString(Label), nullptr, Colour, 0.0f, true, 1.1f);
    }

    // Level 2: the solved suspension network. Node positions are the solver's,
    // not a drawn curve - the cascade junctions hang where the tensions and
    // the lines' own weight put them, so the sag on screen is the sag the
    // solver found.
    {
        // The graph is solved in the payload frame, where the canopy body
        // origin sits at riser length plus canopy-to-riser. Shift it so that
        // origin lands on the same actor-local height the geometry above is
        // drawn at, and the attachment nodes coincide by construction.
        const float PayloadOffsetCm = SuspensionRiseCm
            - static_cast<float>(LineGraph.canopyDesignOriginM.z) * MetresToCm;
        const auto ToWorldPayload =
            [&](const Parapenting::Physics::Vec3& payload)
        {
            return ActorTransform.TransformPosition(FVector(
                static_cast<float>(payload.x) * MetresToCm,
                static_cast<float>(payload.y) * MetresToCm,
                PayloadOffsetCm
                    + static_cast<float>(payload.z) * MetresToCm));
        };
        const auto RowColour = [](Parapenting::Physics::LineRow Row)
        {
            switch (Row)
            {
            case Parapenting::Physics::LineRow::A:
                return FColor(255, 70, 60);
            case Parapenting::Physics::LineRow::ABaby:
                return FColor(255, 150, 60);
            case Parapenting::Physics::LineRow::B:
                return FColor(70, 130, 255);
            case Parapenting::Physics::LineRow::C:
                return FColor(90, 220, 120);
            case Parapenting::Physics::LineRow::Brake:
                return FColor(230, 230, 90);
            }
            return FColor::White;
        };

        for (const auto& Cable : LineSolution.cables)
        {
            const FVector Start = ToWorldPayload(
                LineSolution.nodePositionM[
                    static_cast<std::size_t>(Cable.nodeA)]);
            const FVector End = ToWorldPayload(
                LineSolution.nodePositionM[
                    static_cast<std::size_t>(Cable.nodeB)]);
            // Width is tension, colour fades out when the line is slack. A
            // slack line carries exactly zero, and it should look like it.
            const float Load01 = FMath::Clamp(
                FMath::Sqrt(static_cast<float>(Cable.tensionN) / 120.0f),
                0.0f, 1.0f);
            const FColor Colour = Cable.slack
                ? FColor(70, 70, 75) : RowColour(Cable.row);
            DrawDebugLine(World, Start, End, Colour, false, 0.0f, 0,
                FMath::Lerp(0.5f, 3.0f, Load01));
        }
        for (std::size_t Index = 0; Index < LineGraph.nodes.size(); ++Index)
        {
            const auto& Node = LineGraph.nodes[Index];
            const bool bJunction = Node.kind
                == Parapenting::Physics::SuspensionNodeKind::CascadeJunction;
            const bool bCarabiner = Node.kind
                == Parapenting::Physics::SuspensionNodeKind::Carabiner;
            if (!bJunction && !bCarabiner) continue;
            DrawDebugSphere(World,
                ToWorldPayload(LineSolution.nodePositionM[Index]),
                bCarabiner ? 6.0f : 3.5f, 8,
                bCarabiner ? FColor(205, 205, 210) : RowColour(Node.row),
                false, 0.0f, 0, 1.2f);
        }

        DrawDebugString(World,
            ActorTransform.TransformPosition(
                FVector(0.0f, 0.0f, PayloadOffsetCm - 60.0f)),
            FString::Printf(
                TEXT("lines  A %.0f%%  A' %.0f%%  B %.0f%%  C %.0f%%  ")
                TEXT("brake %.0f%%\n")
                TEXT("carabiner %.0f / %.0f N   stretch %.1f cm   ")
                TEXT("incidence %+.2f deg\n")
                TEXT("slack %d of %d   residual %.3f N / %.3f Nm"),
                100.0 * LineSolution.rowLoadFraction[0],
                100.0 * LineSolution.rowLoadFraction[1],
                100.0 * LineSolution.rowLoadFraction[2],
                100.0 * LineSolution.rowLoadFraction[3],
                100.0 * LineSolution.rowLoadFraction[4],
                LineSolution.leftCarabinerLoadN,
                LineSolution.rightCarabinerLoadN,
                100.0 * LineSolution.lineStretchM,
                FMath::RadiansToDegrees(LineSolution.incidenceChangeRad),
                LineSolution.slackCableCount,
                static_cast<int32>(LineSolution.cables.size()),
                LineSolution.canopyForceResidualN,
                LineSolution.canopyMomentResidualNm),
            nullptr, FColor(235, 240, 255), 0.0f, true, 1.0f);
    }

    // Summary, so the derived figures can be read against the published ones
    // without leaving the cockpit.
    const auto Section = Canopy.InflatedSectionAt(0.45);
    DrawDebugString(World,
        ActorTransform.TransformPosition(
            FVector(0.0f, 0.0f, SuspensionRiseCm + 340.0f)),
        FString::Printf(
            TEXT("flat %.2f m / %.1f m2   proj %.2f m / %.1f m2   AR %.2f\n")
            TEXT("cells %d   root chord %.2f m   tip %.2f m\n")
            TEXT("seam %.1f%%   sagitta %.0f mm   hoop %.0f N/m"),
            Canopy.DevelopedSpanM(), Canopy.DevelopedAreaM2(),
            Canopy.ProjectedSpanM(), Canopy.ProjectedAreaM2(),
            Canopy.FlatAspectRatio(), Canopy.Spec().cellCount,
            Canopy.StationAt(0.0).chordM, Canopy.Ribs().front().chordM,
            Canopy.Spec().billowFraction * 100.0,
            Section.sagittaM * 1000.0, Section.hoopTensionNPerM),
        nullptr, FColor(235, 240, 255), 0.0f, true, 1.0f);
}

void AParagliderPawn::SolveSuspensionGraph()
{
    Parapenting::Physics::SuspensionSolveInput Input;
    // Steady-flight load: what the lines are actually holding up right now.
    // Level 4 replaces this with the VSM resultant and Level 7 puts the solve
    // inside the step; until then the graph is driven by the flight model
    // rather than driving it.
    const double LoadFactor =
        FMath::Max(0.1, Dynamics.LastTelemetry().loadFactor);
    Input.aeroForceN = {0.0, 0.0,
        LoadFactor * Dynamics.Parameters().allUpMassKg * 9.80665};
    Input.canopyWeightN = Dynamics.Parameters().canopyMassKg * 9.80665;
    Input.accelerator = AppliedControls.accelerator;
    Input.leftBrake = AppliedControls.leftBrake;
    Input.rightBrake = AppliedControls.rightBrake;
    Input.weightShift = AppliedControls.weightShift;
    LineSolution = Parapenting::Physics::SolveSuspension(LineGraph, Input);

    // Reduce the solved network to a shape the line rendering can apply to the
    // canopy as it is actually drawn - deformed by collapse, cravat and load,
    // none of which the Level 2 rigid-canopy solver models. The sag is the
    // solver's; the endpoints stay the render path's, so lines still terminate
    // on the surface the pilot can see.
    FVector offsetSum[Parapenting::Physics::LineRowCount]{};
    double splitSum[Parapenting::Physics::LineRowCount]{};
    int shapeCount[Parapenting::Physics::LineRowCount]{};
    for (std::size_t Index = 0; Index < LineGraph.nodes.size(); ++Index)
    {
        const auto& Node = LineGraph.nodes[Index];
        if (Node.kind
            != Parapenting::Physics::SuspensionNodeKind::CascadeJunction)
            continue;

        // Find the run this junction sits on: its main line below and the
        // mean of the uppers above.
        Parapenting::Physics::Vec3 Lower{};
        Parapenting::Physics::Vec3 UpperMean{};
        int Uppers = 0;
        bool bFoundLower = false;
        for (const auto& Cable : LineSolution.cables)
        {
            if (Cable.nodeB == static_cast<int>(Index))
            {
                Lower = LineSolution.nodePositionM[
                    static_cast<std::size_t>(Cable.nodeA)];
                bFoundLower = true;
            }
            else if (Cable.nodeA == static_cast<int>(Index))
            {
                UpperMean += LineSolution.nodePositionM[
                    static_cast<std::size_t>(Cable.nodeB)];
                ++Uppers;
            }
        }
        if (!bFoundLower || Uppers == 0) continue;
        UpperMean = UpperMean / static_cast<double>(Uppers);

        const Parapenting::Physics::Vec3 Run = UpperMean - Lower;
        const double RunLength = Parapenting::Physics::Length(Run);
        if (RunLength < 1e-6) continue;
        const Parapenting::Physics::Vec3 Junction =
            LineSolution.nodePositionM[Index];
        const Parapenting::Physics::Vec3 Offset = Junction - Lower;
        const double Along =
            Parapenting::Physics::Dot(Offset, Run) / (RunLength * RunLength);
        const Parapenting::Physics::Vec3 Perpendicular =
            Offset - Run * Along;

        const auto Row = static_cast<std::size_t>(Node.row);
        splitSum[Row] += Along;
        // Mirror the left half onto the right so the two average together
        // instead of cancelling.
        offsetSum[Row] += FVector(
            static_cast<float>(Perpendicular.x),
            static_cast<float>(Perpendicular.y * (Node.side < 0.0 ? -1.0 : 1.0)),
            static_cast<float>(Perpendicular.z));
        ++shapeCount[Row];
    }
    for (int Row = 0; Row < Parapenting::Physics::LineRowCount; ++Row)
    {
        if (shapeCount[Row] == 0) continue;
        LineShape[Row].splitAlongRun = static_cast<float>(
            splitSum[Row] / shapeCount[Row]);
        LineShape[Row].OffsetFromRunM =
            offsetSum[Row] / static_cast<float>(shapeCount[Row]);
    }
}

void AParagliderPawn::ToggleGeometryVisualization()
{
    bGeometryVisualization = !bGeometryVisualization;
    // Re-solve on the way in, so the view shows the load path for the
    // controls being held rather than a stale one.
    if (bGeometryVisualization) SolveSuspensionGraph();
}

void AParagliderPawn::ToggleAirflowVisualization()
{
    bAirflowVisualization = !bAirflowVisualization;
}

void AParagliderPawn::CycleCameraMode()
{
    CameraMode = (CameraMode + 1) % 3;
}

void AParagliderPawn::CycleAccessibilityProfile()
{
    AccessibilityProfile =
        static_cast<Parapenting::Physics::AccessibilityProfileId>(
            (static_cast<int32>(AccessibilityProfile) + 1)
            % static_cast<int32>(
                Parapenting::Physics::AccessibilityProfileCount));
    SavePilotProgress();
}

const char* AParagliderPawn::GetAccessibilityProfileName() const
{
    return Parapenting::Physics::GetAccessibilityProfile(
        AccessibilityProfile).displayName;
}

const char* AParagliderPawn::GetKeyboardLayoutName() const
{
    switch (KeyboardLayoutIndex)
    {
        case 1: return "COMPACT";
        case 2: return "RIGHT HAND";
        case 3: return "CUSTOM";
        default: return "STANDARD";
    }
}

void AParagliderPawn::CycleKeyboardLayout()
{
    KeyboardLayoutIndex = (KeyboardLayoutIndex + 1) % 4;
    ApplyKeyboardLayout();
    SavePilotProgress();
}

FString AParagliderPawn::GetBindingCaptureText() const
{
    const auto Action =
        static_cast<Parapenting::Physics::FlightBindingAction>(
            FMath::Clamp(SelectedBindingAction, 0,
                static_cast<int32>(
                    Parapenting::Physics::FlightBindingActionCount) - 1));
    const FString ActionName = ANSI_TO_TCHAR(
        Parapenting::Physics::FlightBindingActionDisplayName(Action));
    const FString KeyName = ANSI_TO_TCHAR(
        CustomBindings.Key(Action).c_str());
    return bCapturingBinding
        ? FString::Printf(TEXT("PRESS KEY FOR %s  [ESC CANCEL]"), *ActionName)
        : FString::Printf(TEXT("%s = %s  [F6 SELECT / F7 BIND]  %s"),
            *ActionName, *KeyName, *BindingCaptureStatus);
}

void AParagliderPawn::CycleBindingAction()
{
    if (bCapturingBinding) return;
    SelectedBindingAction = (SelectedBindingAction + 1)
        % static_cast<int32>(
            Parapenting::Physics::FlightBindingActionCount);
    BindingCaptureStatus = TEXT("READY");
}

void AParagliderPawn::BeginBindingCapture()
{
    bCapturingBinding = true;
    BindingCaptureStatus = TEXT("LISTENING");
}

void AParagliderPawn::UpdateBindingCapture()
{
    if (!bCapturingBinding) return;
    APlayerController* PC = Cast<APlayerController>(GetController());
    if (!PC) return;
    if (PC->WasInputKeyJustPressed(EKeys::Escape))
    {
        bCapturingBinding = false;
        BindingCaptureStatus = TEXT("CANCELLED");
        return;
    }

    TArray<FKey> Keys;
    EKeys::GetAllKeys(Keys);
    const TArray<FName> RemappedActions{
        TEXT("WeightShiftLeftStep"), TEXT("WeightShiftRightStep"),
        TEXT("LeftBrakeStep"), TEXT("RightBrakeStep"),
        TEXT("BothBrakesMore"), TEXT("BrakesRelease")
    };
    const TArray<FInputActionKeyMapping> Existing =
        UInputSettings::GetInputSettings()->GetActionMappings();
    for (const FKey& Key : Keys)
    {
        if (!PC->WasInputKeyJustPressed(Key)
            || Key.IsGamepadKey() || Key.IsMouseButton())
            continue;
        const FString Candidate = Key.GetFName().ToString();
        bool bUsedByProtectedAction = false;
        for (const FInputActionKeyMapping& Mapping : Existing)
        {
            if (Mapping.Key == Key
                && !RemappedActions.Contains(Mapping.ActionName))
            {
                bUsedByProtectedAction = true;
                break;
            }
        }
        if (bUsedByProtectedAction)
        {
            BindingCaptureStatus =
                FString::Printf(TEXT("%s IS USED BY ANOTHER ACTION"),
                    *Candidate);
            continue;
        }
        const auto Result = CustomBindings.Rebind(
            static_cast<Parapenting::Physics::FlightBindingAction>(
                SelectedBindingAction),
            TCHAR_TO_UTF8(*Candidate));
        if (Result == Parapenting::Physics::RebindResult::RejectedProtected
            || Result == Parapenting::Physics::RebindResult::RejectedInvalid)
        {
            BindingCaptureStatus =
                FString::Printf(TEXT("%s IS PROTECTED"), *Candidate);
            continue;
        }
        KeyboardLayoutIndex = 3;
        bCapturingBinding = false;
        BindingCaptureStatus =
            Result == Parapenting::Physics::RebindResult::Swapped
            ? TEXT("BOUND / CONFLICT SWAPPED") : TEXT("BOUND");
        ApplyKeyboardLayout();
        SavePilotProgress();
        return;
    }
}

void AParagliderPawn::ApplyKeyboardLayout()
{
    UInputSettings* Settings = UInputSettings::GetInputSettings();
    if (!Settings) return;
    static const TArray<FName> RemappedActions{
        TEXT("WeightShiftLeftStep"), TEXT("WeightShiftRightStep"),
        TEXT("LeftBrakeStep"), TEXT("RightBrakeStep"),
        TEXT("BothBrakesMore"), TEXT("BrakesRelease")
    };
    const TArray<FInputActionKeyMapping> Existing =
        Settings->GetActionMappings();
    for (const FInputActionKeyMapping& Mapping : Existing)
    {
        if (RemappedActions.Contains(Mapping.ActionName)
            && !Mapping.Key.IsGamepadKey())
            Settings->RemoveActionMapping(Mapping, false);
        if (Mapping.ActionName == TEXT("CenterWeightShift")
            || (Mapping.ActionName == TEXT("AcceleratorLess")
                && Mapping.Key == EKeys::S))
            Settings->RemoveActionMapping(Mapping, false);
    }

    const Parapenting::Physics::InputBindingProfile* Profile =
        &CustomBindings;
    Parapenting::Physics::InputBindingProfile Preset;
    if (KeyboardLayoutIndex == 0)
    {
        Preset = Parapenting::Physics::InputBindingProfile::Standard();
        Profile = &Preset;
    }
    else if (KeyboardLayoutIndex == 1)
    {
        Preset = Parapenting::Physics::InputBindingProfile::Compact();
        Profile = &Preset;
    }
    else if (KeyboardLayoutIndex == 2)
    {
        Preset = Parapenting::Physics::InputBindingProfile::RightHand();
        Profile = &Preset;
    }
    TArray<FKey> Keys;
    for (std::size_t Index = 0;
         Index < Parapenting::Physics::FlightBindingActionCount; ++Index)
    {
        const auto Action =
            static_cast<Parapenting::Physics::FlightBindingAction>(Index);
        Keys.Add(FKey(FName(ANSI_TO_TCHAR(
            Profile->Key(Action).c_str()))));
    }
    for (int32 Index = 0; Index < RemappedActions.Num(); ++Index)
        Settings->AddActionMapping(
            FInputActionKeyMapping(RemappedActions[Index], Keys[Index]),
            false);
    Settings->AddActionMapping(
        FInputActionKeyMapping(TEXT("CenterWeightShift"), EKeys::S),
        false);
    Settings->SaveKeyMappings();
    Settings->ForceRebuildKeymaps();
}

const char* AParagliderPawn::GetGraphicsProfileName() const
{
    return Parapenting::Physics::GetGraphicsProfile(
        GraphicsProfile).displayName;
}

const char* AParagliderPawn::GetHudModeName() const
{
    static constexpr const char* Names[] = {
        "COMPACT", "EXPANDED", "MINIMAL"};
    return Names[FMath::Clamp(HudMode, 0, 2)];
}

void AParagliderPawn::CycleHudMode()
{
    HudMode = (HudMode + 1) % 3;
    SavePilotProgress();
}

void AParagliderPawn::TogglePreflightBriefing()
{
    bBriefingVisible = !bBriefingVisible;
}

void AParagliderPawn::CycleGraphicsProfile()
{
    GraphicsProfile =
        static_cast<Parapenting::Physics::GraphicsProfileId>(
            (static_cast<int32>(GraphicsProfile) + 1)
            % static_cast<int32>(
                Parapenting::Physics::GraphicsProfileCount));
    ApplyGraphicsProfile();
    SavePilotProgress();
}

void AParagliderPawn::ApplyGraphicsProfile()
{
    const auto& Profile =
        Parapenting::Physics::GetGraphicsProfile(GraphicsProfile);
    Scalability::FQualityLevels Levels;
    Levels.SetFromSingleQualityLevel(Profile.qualityLevel);
    Levels.ResolutionQuality = static_cast<float>(Profile.resolutionScale);
    Scalability::SetQualityLevels(Levels, true);
    Scalability::SaveState(GIsEditor ? GEditorSettingsIni : GGameUserSettingsIni);
}

const char* AParagliderPawn::GetCameraModeDisplayName() const
{
    switch (CameraMode)
    {
        case 1: return "CLOSE CHASE";
        case 2: return "PILOT";
        default: return "CINEMATIC CHASE";
    }
}

const char* AParagliderPawn::GetPilotRankName() const
{
    return Parapenting::Physics::EvaluatePilotProgression(
        ChallengeBestScores).rankName;
}

double AParagliderPawn::GetPilotExperience() const
{
    return Parapenting::Physics::EvaluatePilotProgression(
        ChallengeBestScores).experience;
}

double AParagliderPawn::GetPilotRankProgress() const
{
    return Parapenting::Physics::EvaluatePilotProgression(
        ChallengeBestScores).rankProgress;
}

int32 AParagliderPawn::GetPilotMedalCount() const
{
    const auto Progress = Parapenting::Physics::EvaluatePilotProgression(
        ChallengeBestScores);
    return Progress.bronzeMedals + Progress.silverMedals
        + Progress.goldMedals;
}

void AParagliderPawn::LoadPilotProgress()
{
    const FString ProgressPath = FPaths::Combine(
        FPaths::ProjectSavedDir(), TEXT("PlayerProgress.ini"));
    for (int32 Index = 0;
         Index < static_cast<int32>(ChallengeBestScores.size()); ++Index)
    {
        double Score = 0.0;
        GConfig->GetDouble(TEXT("FlightLab"),
            *FString::Printf(TEXT("Scenario%dBest"), Index),
            Score, ProgressPath);
        ChallengeBestScores[Index] = FMath::Clamp(Score, 0.0, 1000.0);
    }
    int32 SavedAccessibility = 0;
    GConfig->GetInt(TEXT("Accessibility"), TEXT("Profile"),
        SavedAccessibility, ProgressPath);
    SavedAccessibility = FMath::Clamp(
        SavedAccessibility, 0,
        static_cast<int32>(
            Parapenting::Physics::AccessibilityProfileCount) - 1);
    AccessibilityProfile =
        static_cast<Parapenting::Physics::AccessibilityProfileId>(
            SavedAccessibility);
    GConfig->GetInt(TEXT("Accessibility"), TEXT("KeyboardLayout"),
        KeyboardLayoutIndex, ProgressPath);
    KeyboardLayoutIndex = FMath::Clamp(KeyboardLayoutIndex, 0, 3);
    for (std::size_t Index = 0;
         Index < Parapenting::Physics::FlightBindingActionCount; ++Index)
    {
        FString KeyName;
        if (GConfig->GetString(TEXT("CustomBindings"),
            *FString::Printf(TEXT("Action%d"), static_cast<int32>(Index)),
            KeyName, ProgressPath))
        {
            CustomBindings.keyNames[Index] = TCHAR_TO_UTF8(*KeyName);
        }
    }
    GConfig->GetInt(TEXT("Interface"), TEXT("HudMode"),
        HudMode, ProgressPath);
    HudMode = FMath::Clamp(HudMode, 0, 2);
    int32 SavedGraphics =
        static_cast<int32>(Parapenting::Physics::GraphicsProfileId::High);
    GConfig->GetInt(TEXT("Graphics"), TEXT("Profile"),
        SavedGraphics, ProgressPath);
    SavedGraphics = FMath::Clamp(
        SavedGraphics, 0,
        static_cast<int32>(
            Parapenting::Physics::GraphicsProfileCount) - 1);
    GraphicsProfile =
        static_cast<Parapenting::Physics::GraphicsProfileId>(SavedGraphics);
}

void AParagliderPawn::SavePilotProgress() const
{
    const FString ProgressPath = FPaths::Combine(
        FPaths::ProjectSavedDir(), TEXT("PlayerProgress.ini"));
    for (int32 Index = 0;
         Index < static_cast<int32>(ChallengeBestScores.size()); ++Index)
    {
        GConfig->SetDouble(TEXT("FlightLab"),
            *FString::Printf(TEXT("Scenario%dBest"), Index),
            ChallengeBestScores[Index], ProgressPath);
    }
    GConfig->SetInt(TEXT("Accessibility"), TEXT("Profile"),
        static_cast<int32>(AccessibilityProfile), ProgressPath);
    GConfig->SetInt(TEXT("Accessibility"), TEXT("KeyboardLayout"),
        KeyboardLayoutIndex, ProgressPath);
    for (std::size_t Index = 0;
         Index < Parapenting::Physics::FlightBindingActionCount; ++Index)
    {
        GConfig->SetString(TEXT("CustomBindings"),
            *FString::Printf(TEXT("Action%d"), static_cast<int32>(Index)),
            ANSI_TO_TCHAR(CustomBindings.keyNames[Index].c_str()),
            ProgressPath);
    }
    GConfig->SetInt(TEXT("Interface"), TEXT("HudMode"),
        HudMode, ProgressPath);
    GConfig->SetInt(TEXT("Graphics"), TEXT("Profile"),
        static_cast<int32>(GraphicsProfile), ProgressPath);
    GConfig->Flush(false, ProgressPath);
}

void AParagliderPawn::SaveReplayManifest()
{
    const FString Directory = FPaths::Combine(
        FPaths::ProjectSavedDir(), TEXT("Replays"));
    FPlatformFileManager::Get().GetPlatformFile().CreateDirectoryTree(*Directory);
    const FDateTime Now = FDateTime::Now();
    const FString Timestamp = FString::Printf(
        TEXT("%s-%03d"), *Now.ToString(TEXT("%Y%m%d-%H%M%S")),
        Now.GetMillisecond());
    FString Csv = TEXT("frame,time_s,left_brake,right_brake,weight_shift,accelerator\n");
    for (int32 Index = 0; Index < ReplayFrames.Num(); ++Index)
    {
        const auto& Frame = ReplayFrames[Index];
        Csv += FString::Printf(TEXT("%d,%.6f,%.3f,%.3f,%.3f,%.3f\n"),
            Index, Index * PhysicsStepSeconds, Frame.leftBrake,
            Frame.rightBrake, Frame.weightShift, Frame.accelerator);
    }
    const FString Path = FPaths::Combine(
        Directory,
        FString::Printf(TEXT("replay-%s.csv"),
            *Timestamp));
    FFileHelper::SaveStringToFile(Csv, *Path);
    FString GhostCsv = TEXT("sample,time_s,x_cm,y_cm,z_cm,qx,qy,qz,qw\n");
    for (int32 Index = 0; Index < GhostFrames.Num(); ++Index)
    {
        const FTransform& Frame = GhostFrames[Index];
        const FVector P = Frame.GetLocation();
        const FQuat Q = Frame.GetRotation();
        GhostCsv += FString::Printf(
            TEXT("%d,%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f\n"),
            Index, Index * 0.1, P.X, P.Y, P.Z,
            Q.X, Q.Y, Q.Z, Q.W);
    }
    const FString GhostPath = FPaths::Combine(
        Directory,
        FString::Printf(TEXT("ghost-%s.csv"),
            *Timestamp));
    FFileHelper::SaveStringToFile(GhostCsv, *GhostPath);
    const auto& DebriefSummary = Debrief.Summary();
    const FString AnalysisCsv = FString::Printf(
        TEXT("phase,duration_s,distance_m,altitude_gain_m,thermal_gain_m,"
             "rotor_exposure_s,collapse_events,cravat_events,stall_spin_events,"
             "safety,efficiency,thermal,landing,overall\n"
             "\"%s\",%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,"
             "%.1f,%.1f,%.1f,%.1f,%.1f\n"),
        ANSI_TO_TCHAR(GetFlightPhaseName()),
        DebriefSummary.durationS, DebriefSummary.horizontalDistanceM,
        DebriefSummary.altitudeGainM, DebriefSummary.thermalGainM,
        DebriefSummary.rotorExposureS,
        DebriefSummary.asymmetricCollapseEvents
            + DebriefSummary.frontalCollapseEvents,
        DebriefSummary.cravatEvents, DebriefSummary.stallOrSpinEvents,
        DebriefSummary.safetyRating, DebriefSummary.efficiencyRating,
        DebriefSummary.thermalRating, DebriefSummary.landingRating,
        DebriefSummary.overallRating);
    const FString AnalysisPath = FPaths::Combine(
        Directory,
        FString::Printf(TEXT("analysis-%s.csv"), *Timestamp));
    FFileHelper::SaveStringToFile(AnalysisCsv, *AnalysisPath);

    // Versioned native replay stores everything required for deterministic
    // playback. CSV exports remain alongside it for analysis and portability.
    FBufferArchive Archive;
    uint32 Magic = 0x50524752; // PRGR
    uint32 Version = 2;
    Archive << Magic;
    Archive << Version;
    int32 Wing = static_cast<int32>(ReplayWing);
    int32 Route = static_cast<int32>(ReplayRouteIndex);
    int32 Scenario = static_cast<int32>(ReplayScenarioIndex);
    int32 WeatherMode = static_cast<int32>(ReplayWeatherMode);
    int32 WeatherPreset = static_cast<int32>(ReplayWeatherPreset);
    int32 WingSize = static_cast<int32>(ReplayWingSize);
    int32 BrakeTravel = static_cast<int32>(ReplayBrakeTravel);
    int32 Harness = static_cast<int32>(ReplayEquipment.harness);
    Archive << Wing << Route << Scenario << WeatherMode << WeatherPreset;
    Archive << WingSize << BrakeTravel << Harness;
    Archive << ReplayWindFromDegrees << ReplayWindSpeedMps;
    Archive << ReplayStartLocalHour;
    Archive << ReplayEquipment.pilotMassKg;
    Archive << ReplayEquipment.reserveAndEquipmentKg;
    Archive << ReplayEquipment.ballastKg;
    int32 ControlCount = ReplayFrames.Num();
    Archive << ControlCount;
    for (const auto& Frame : ReplayFrames)
    {
        double Left = Frame.leftBrake;
        double Right = Frame.rightBrake;
        double Shift = Frame.weightShift;
        double Accelerator = Frame.accelerator;
        Archive << Left << Right << Shift << Accelerator;
    }
    int32 GhostCount = GhostFrames.Num();
    Archive << GhostCount;
    for (const FTransform& Frame : GhostFrames)
    {
        FVector Position = Frame.GetLocation();
        FQuat Rotation = Frame.GetRotation();
        Archive << Position << Rotation;
    }
    const FString NativePath = FPaths::Combine(
        Directory, FString::Printf(TEXT("flight-%s.pgr"), *Timestamp));
    FFileHelper::SaveArrayToFile(Archive, *NativePath);
    Archive.FlushCache();
    Archive.Empty();
    RefreshReplayCatalogue();
    SelectedReplayFileIndex = ReplayFiles.IndexOfByKey(NativePath);
    if (SelectedReplayFileIndex == INDEX_NONE)
        SelectedReplayFileIndex = FMath::Max(0, ReplayFiles.Num() - 1);
    ReplayDisplayLabel = FPaths::GetBaseFilename(NativePath);
}

void AParagliderPawn::RefreshReplayCatalogue()
{
    const FString Directory = FPaths::Combine(
        FPaths::ProjectSavedDir(), TEXT("Replays"));
    TArray<FString> Names;
    IFileManager::Get().FindFiles(
        Names, *FPaths::Combine(Directory, TEXT("*.pgr")), true, false);
    Names.Sort();
    ReplayFiles.Reset();
    for (const FString& Name : Names)
        ReplayFiles.Add(FPaths::Combine(Directory, Name));
    if (ReplayFiles.IsEmpty())
    {
        SelectedReplayFileIndex = 0;
        ReplayDisplayLabel = TEXT("NO SAVED REPLAY");
        return;
    }
    SelectedReplayFileIndex = ReplayFiles.Num() - 1;
    LoadReplayFile(ReplayFiles[SelectedReplayFileIndex]);
}

bool AParagliderPawn::LoadReplayFile(const FString& Path)
{
    TArray<uint8> Bytes;
    if (!FFileHelper::LoadFileToArray(Bytes, *Path)) return false;
    FMemoryReader Reader(Bytes, true);
    uint32 Magic = 0;
    uint32 Version = 0;
    Reader << Magic << Version;
    if (Magic != 0x50524752 || (Version != 1 && Version != 2)) return false;

    int32 Wing = 0, Route = 0, Scenario = 0;
    int32 WeatherMode = 0, WeatherPreset = 0;
    int32 WingSize = 0, BrakeTravel = 0, Harness = 0;
    Reader << Wing << Route << Scenario << WeatherMode << WeatherPreset;
    Reader << WingSize << BrakeTravel << Harness;
    double WindFrom = 0.0, WindSpeed = 0.0;
    Reader << WindFrom << WindSpeed;
    double StartLocalHour = 13.0;
    if (Version >= 2) Reader << StartLocalHour;
    Parapenting::Physics::EquipmentSetup LoadedEquipment;
    Reader << LoadedEquipment.pilotMassKg;
    Reader << LoadedEquipment.reserveAndEquipmentKg;
    Reader << LoadedEquipment.ballastKg;
    int32 ControlCount = 0;
    Reader << ControlCount;
    const bool bMetadataValid =
        Wing >= 0 && Wing < static_cast<int32>(
            Parapenting::Physics::WingProfileCount)
        && Route >= 0 && Route < static_cast<int32>(
            Parapenting::Physics::RouteProfileCount())
        && Scenario >= 0 && Scenario < static_cast<int32>(
            Parapenting::Physics::TrainingScenarioCount())
        && WeatherMode >= 0 && WeatherMode <= 3
        && WeatherPreset >= 0
        && WeatherPreset <= static_cast<int32>(
            Parapenting::Physics::WeatherPresetId::EveningDrainage)
        && WingSize >= 0 && WingSize <= 2
        && BrakeTravel >= 0 && BrakeTravel <= 2
        && Harness >= 0 && Harness <= 2
        && ControlCount >= 0 && ControlCount <= 5000000
        && FMath::IsFinite(WindFrom) && FMath::IsFinite(WindSpeed)
        && FMath::IsFinite(StartLocalHour)
        && StartLocalHour >= 0.0 && StartLocalHour < 24.0;
    if (!bMetadataValid) return false;

    TArray<Parapenting::Physics::ControlInput> LoadedControls;
    LoadedControls.SetNum(ControlCount);
    for (auto& Frame : LoadedControls)
        Reader << Frame.leftBrake << Frame.rightBrake
               << Frame.weightShift << Frame.accelerator;
    int32 GhostCount = 0;
    Reader << GhostCount;
    if (GhostCount < 0 || GhostCount > 500000) return false;
    TArray<FTransform> LoadedGhosts;
    LoadedGhosts.Reserve(GhostCount);
    for (int32 Index = 0; Index < GhostCount; ++Index)
    {
        FVector Position;
        FQuat Rotation;
        Reader << Position << Rotation;
        LoadedGhosts.Add(FTransform(Rotation, Position));
    }
    if (Reader.IsError()) return false;

    LoadedEquipment.harness =
        static_cast<Parapenting::Physics::HarnessType>(Harness);
    ReplayFrames = MoveTemp(LoadedControls);
    GhostFrames = MoveTemp(LoadedGhosts);
    ReplayWing = static_cast<Parapenting::Physics::WingProfileId>(Wing);
    ReplayRouteIndex = static_cast<std::size_t>(Route);
    ReplayScenarioIndex = static_cast<std::size_t>(Scenario);
    ReplayWeatherMode =
        static_cast<Parapenting::Physics::WeatherMode>(WeatherMode);
    ReplayWeatherPreset =
        static_cast<Parapenting::Physics::WeatherPresetId>(WeatherPreset);
    ReplayWingSize = static_cast<Parapenting::Physics::WingSize>(WingSize);
    ReplayBrakeTravel =
        static_cast<Parapenting::Physics::BrakeTravel>(BrakeTravel);
    ReplayEquipment = LoadedEquipment;
    ReplayWindFromDegrees = WindFrom;
    ReplayWindSpeedMps = WindSpeed;
    if (Version == 1
        && ReplayWeatherPreset
            != Parapenting::Physics::WeatherPresetId::Custom)
    {
        StartLocalHour = Parapenting::Physics::GetWeatherPreset(
            ReplayWeatherPreset).startLocalHour;
    }
    ReplayStartLocalHour = StartLocalHour;
    ReplayDisplayLabel = FPaths::GetBaseFilename(Path);
    bPlayingReplay = false;
    bGhostVisible = false;
    return true;
}

void AParagliderPawn::RecordTelemetrySample()
{
    const auto& T = Dynamics.LastTelemetry();
    FString Line = FString::Printf(
        TEXT("%.2f,\"%s\",\"%s\",\"%s\",\"%s\",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
             "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
             "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
             "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n"),
        SimulationTimeSeconds,
        ANSI_TO_TCHAR(GetRouteDisplayName()),
        ANSI_TO_TCHAR(GetWingDisplayName()),
        ANSI_TO_TCHAR(GetScenarioDisplayName()),
        ANSI_TO_TCHAR(GetWeatherPresetDisplayName()),
        State.positionWorldM.x, State.positionWorldM.y,
        State.positionWorldM.z + GetLandingElevationM(),
        GetGroundClearanceM(),
        State.velocityWorldMps.x, State.velocityWorldMps.y,
        State.velocityWorldMps.z, T.airspeedMps,
        AppliedControls.leftBrake, AppliedControls.rightBrake,
        AppliedControls.weightShift, T.accelerator, T.leftBrakePressure,
        T.rightBrakePressure, T.loadFactor, T.thermalLiftMps,
        T.rotorStrength, T.leftCollapse, T.rightCollapse,
        T.frontalCollapse, T.leftCravat, T.rightCravat,
        T.deepStall, T.spin, bLanded ? State.canopyPressure : T.canopyPressure,
        T.spanwiseLoadAsymmetry, T.aRiserLoad, T.bRiserLoad,
        T.cRiserLoad, T.dRiserLoad, T.lineLoadTotalN,
        T.allUpMassKg, T.wingLoadingKgM2, T.harnessDragN,
        T.brakeTravelLeftMm, T.brakeTravelRightMm);
    Line.RemoveFromEnd(TEXT("\n"));
    Line += FString::Printf(
        TEXT(",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
             "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
             "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n"),
        Challenge.Score(), Challenge.Progress(),
        T.leftReinflationAuthority, T.rightReinflationAuthority,
        T.surgeContainment, T.recoverySurge,
        T.leftBrakeForceN, T.rightBrakeForceN,
        T.highLoadDeformation, T.spanwiseAirflowShearMps,
        T.leftAirflowDisturbance, T.rightAirflowDisturbance,
        T.lowFrequencyGustMps, T.highFrequencyGustMps,
        T.gustEnergyMps, T.groundEffect, T.flareEnergy,
        T.flareAuthority, T.flareBoost,
        T.collapseResistance, T.passiveReinflationRate,
        T.frontalReinflationRate, T.cravatSusceptibility);
    Line.RemoveFromEnd(TEXT("\n"));
    const auto& D = Debrief.Summary();
    Line += FString::Printf(
        TEXT(",\"%s\",%.3f,\"%s\",%.2f,%.2f,%.2f,%.2f,%.2f\n"),
        ANSI_TO_TCHAR(GetLandingRolloutPhaseName()),
        GetRunoutDistanceM(),
        ANSI_TO_TCHAR(GetFlightPhaseName()),
        D.safetyRating, D.efficiencyRating, D.thermalRating,
        D.landingRating, D.overallRating);
    FFileHelper::SaveStringToFile(
        Line, *TelemetryFilePath,
        FFileHelper::EEncodingOptions::AutoDetect,
        &IFileManager::Get(), FILEWRITE_Append);
}

void AParagliderPawn::SaveFlightDebrief() const
{
    const FString Directory = FPaths::Combine(
        FPaths::ProjectSavedDir(), TEXT("Debriefs"));
    FPlatformFileManager::Get().GetPlatformFile().CreateDirectoryTree(
        *Directory);
    const auto& D = Debrief.Summary();
    FString Csv = TEXT(
        "route,wing,weather,duration_s,distance_m,altitude_gain_m,"
        "altitude_loss_m,thermal_gain_m,thermal_time_s,rotor_exposure_s,"
        "severe_rotor_s,max_climb_mps,max_sink_mps,max_airspeed_mps,"
        "max_load_g,min_pressure,asymmetric_collapses,frontal_collapses,"
        "cravats,stall_spin_events,stable_final_s,approach_quality,"
        "landing_distance_m,touchdown_vz_mps,touchdown_speed_mps,"
        "safety,efficiency,thermal,landing,overall,focus\n");
    Csv += FString::Printf(
        TEXT("\"%s\",\"%s\",\"%s\",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
             "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%d,%d,%d,%d,"
             "%.2f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,\"%s\"\n"),
        ANSI_TO_TCHAR(GetRouteDisplayName()),
        ANSI_TO_TCHAR(GetWingDisplayName()),
        ANSI_TO_TCHAR(GetWeatherPresetDisplayName()),
        D.durationS, D.horizontalDistanceM, D.altitudeGainM,
        D.altitudeLossM, D.thermalGainM, D.thermalTimeS,
        D.rotorExposureS, D.severeRotorExposureS,
        D.maximumClimbMps, D.maximumSinkMps, D.maximumAirspeedMps,
        D.maximumLoadFactor, D.minimumCanopyPressure,
        D.asymmetricCollapseEvents, D.frontalCollapseEvents,
        D.cravatEvents, D.stallOrSpinEvents,
        D.stabilizedFinalS, D.averageApproachQuality,
        D.landingDistanceM, D.touchdownVerticalSpeedMps,
        D.touchdownHorizontalSpeedMps,
        D.safetyRating, D.efficiencyRating, D.thermalRating,
        D.landingRating, D.overallRating,
        ANSI_TO_TCHAR(GetDebriefFocusText()));
    const FDateTime Now = FDateTime::Now();
    const FString Path = FPaths::Combine(
        Directory,
        FString::Printf(TEXT("debrief-%s-%03d.csv"),
            *Now.ToString(TEXT("%Y%m%d-%H%M%S")), Now.GetMillisecond()));
    FFileHelper::SaveStringToFile(Csv, *Path);
}

const char* AParagliderPawn::GetScenarioDisplayName() const
{
    return Parapenting::Physics::GetTrainingScenarioByIndex(
        SelectedScenarioIndex).displayName;
}

const char* AParagliderPawn::GetScenarioObjective() const
{
    return Parapenting::Physics::GetTrainingScenarioByIndex(
        SelectedScenarioIndex).objective;
}

void AParagliderPawn::NextTrainingScenario()
{
    SelectedScenarioIndex =
        (SelectedScenarioIndex + 1)
        % Parapenting::Physics::TrainingScenarioCount();
    AirModel.SetMode(
        Parapenting::Physics::GetTrainingScenarioByIndex(
            SelectedScenarioIndex).weather);
    ResetFlight();
}

void AParagliderPawn::ApplyIncidentCue(
    Parapenting::Physics::IncidentCue Cue)
{
    switch (Cue)
    {
        case Parapenting::Physics::IncidentCue::LeftCollapse:
            InjectLeftCollapse(); break;
        case Parapenting::Physics::IncidentCue::RightCollapse:
            InjectRightCollapse(); break;
        case Parapenting::Physics::IncidentCue::FrontalCollapse:
            InjectFrontalCollapse(); break;
        case Parapenting::Physics::IncidentCue::RightSpiral:
            // Establish a repeatable developed-turn entry while leaving the
            // exit entirely to the player's normal brake/weight-shift inputs.
            Controls.rightBrake = FMath::Max(Controls.rightBrake, 0.62);
            Controls.weightShift = FMath::Max(Controls.weightShift, 0.4);
            State.angularVelocityBodyRadps.x =
                FMath::Max(State.angularVelocityBodyRadps.x, 0.62);
            State.angularVelocityBodyRadps.z =
                FMath::Max(State.angularVelocityBodyRadps.z, 0.92);
            State.angularVelocityBodyRadps.y =
                FMath::Min(State.angularVelocityBodyRadps.y, -0.12);
            State.velocityWorldMps.z =
                FMath::Min(State.velocityWorldMps.z, -4.5);
            break;
        case Parapenting::Physics::IncidentCue::None:
            break;
    }
}

void AParagliderPawn::ApplyManualWind()
{
    AirModel.SetBaseWind(
        Parapenting::Physics::WindVectorFromMeteorological(
            ManualWindFromDegrees, ManualWindSpeedMps));
    bLiveWeatherActive = false;
    LiveWeatherStatus = TEXT("MANUAL WEATHER");
}

void AParagliderPawn::WindSpeedDown()
{
    ManualWindSpeedMps = FMath::Max(0.0, ManualWindSpeedMps - 0.5);
    ApplyManualWind();
}

void AParagliderPawn::WindSpeedUp()
{
    ManualWindSpeedMps = FMath::Min(15.0, ManualWindSpeedMps + 0.5);
    ApplyManualWind();
}

void AParagliderPawn::WindRotateLeft()
{
    ManualWindFromDegrees = FMath::Fmod(
        ManualWindFromDegrees + 345.0, 360.0);
    ApplyManualWind();
}

void AParagliderPawn::WindRotateRight()
{
    ManualWindFromDegrees = FMath::Fmod(
        ManualWindFromDegrees + 15.0, 360.0);
    ApplyManualWind();
}

void AParagliderPawn::UpdateControllerHaptics()
{
    APlayerController* PlayerController =
        Cast<APlayerController>(GetController());
    if (!PlayerController) return;
    const auto& T = Dynamics.LastTelemetry();
    const auto Haptics = HapticModel.Evaluate(T, SimulationTimeSeconds);
    const double HapticScale =
        Parapenting::Physics::GetAccessibilityProfile(
            AccessibilityProfile).hapticScale;
    const float LeftIntensity = static_cast<float>(
        FMath::Clamp(Haptics.left * HapticScale, 0.0, 1.0));
    const float RightIntensity = static_cast<float>(
        FMath::Clamp(Haptics.right * HapticScale, 0.0, 1.0));
    const auto LeftAction = LeftForceFeedbackHandle == 0
        ? EDynamicForceFeedbackAction::Start
        : EDynamicForceFeedbackAction::Update;
    const auto RightAction = RightForceFeedbackHandle == 0
        ? EDynamicForceFeedbackAction::Start
        : EDynamicForceFeedbackAction::Update;
    LeftForceFeedbackHandle = PlayerController->PlayDynamicForceFeedback(
        LeftIntensity, 3600.0f, true, true, false, false,
        LeftAction, LeftForceFeedbackHandle);
    RightForceFeedbackHandle = PlayerController->PlayDynamicForceFeedback(
        RightIntensity, 3600.0f, false, false, true, true,
        RightAction, RightForceFeedbackHandle);
}

void AParagliderPawn::InjectLeftCollapse()
{
    if (bLanded) return;
    State.leftCollapse = FMath::Max(State.leftCollapse, 0.58);
    State.canopyPressure = FMath::Min(State.canopyPressure, 0.58);
}

void AParagliderPawn::InjectFrontalCollapse()
{
    if (bLanded) return;
    State.frontalCollapse = FMath::Max(State.frontalCollapse, 0.55);
    State.canopyPressure = FMath::Min(State.canopyPressure, 0.42);
}

void AParagliderPawn::InjectRightCollapse()
{
    if (bLanded) return;
    State.rightCollapse = FMath::Max(State.rightCollapse, 0.58);
    State.canopyPressure = FMath::Min(State.canopyPressure, 0.58);
}

double AParagliderPawn::GetGroundClearanceM() const
{
    return FMath::Max(0.0, State.positionWorldM.z
        - Parapenting::Physics::TerrainModel::HeightM(
            State.positionWorldM.x, State.positionWorldM.y));
}

double AParagliderPawn::GetDistanceToTargetM() const
{
    return FMath::Sqrt(
        FMath::Square(State.positionWorldM.x - LandingTargetXM)
        + FMath::Square(State.positionWorldM.y - LandingTargetYM));
}
