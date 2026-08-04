#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Physics/AtmosphereModel.h"
#include "Physics/ParagliderDynamics.h"
#include "Physics/CanopyGeometry.h"
#include "Physics/TensionCableSolver.h"
#include "Physics/ParagliderSolverClock.h"
#include "Physics/WingCatalogue.h"
#include "Physics/RouteCatalogue.h"
#include "Physics/TrainingScenario.h"
#include "Physics/ChallengeEvaluator.h"
#include "Physics/WeatherSnapshot.h"
#include "Physics/EquipmentSetup.h"
#include "Physics/PilotPose.h"
#include "Physics/GliderRigSnapshot.h"
#include "Physics/HapticFeedback.h"
#include "Physics/PilotProgression.h"
#include "Physics/AccessibilityProfile.h"
#include "Physics/GraphicsProfile.h"
#include "Physics/InputBindingProfile.h"
#include "Physics/LandingCircuitModel.h"
#include "Physics/GroundLaunchModel.h"
#include "Physics/FlightDebrief.h"
#include "Physics/PreflightBriefing.h"
#include "Physics/FlightNavigation.h"
#include "Physics/LandingRolloutModel.h"
#include <array>
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"
#include "ParagliderPawn.generated.h"

class UCameraComponent;
class UProceduralMeshComponent;
class UParaglidingAudioComponent;
class UPoseableMeshComponent;
class USceneComponent;
class UStaticMeshComponent;

UCLASS()
class PARAPENTING_API AParagliderPawn : public APawn
{
    GENERATED_BODY()

public:
    AParagliderPawn();
    virtual void Tick(float DeltaSeconds) override;
    virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
    const Parapenting::Physics::Telemetry& GetFlightTelemetry() const
        { return Dynamics.LastTelemetry(); }
    const Parapenting::Physics::FlightState& GetFlightState() const { return State; }
    const Parapenting::Physics::ControlInput& GetControlInput() const { return AppliedControls; }
    bool HasLanded() const { return bLanded; }
    bool WasHardLanding() const { return bHardLanding; }
    double GetLandingDistanceM() const { return LandingDistanceM; }
    double GetTouchdownVerticalSpeedMps() const { return TouchdownVerticalSpeedMps; }
    double GetTouchdownHorizontalSpeedMps() const { return TouchdownHorizontalSpeedMps; }
    double GetGroundClearanceM() const;
    double GetDistanceToTargetM() const;
    const char* GetWingDisplayName() const;
    const char* GetRouteDisplayName() const;
    const char* GetLandingDisplayName() const;
    double GetLandingElevationM() const;
    bool IsRecordingTelemetry() const { return bRecordingTelemetry; }
    bool IsRecordingReplay() const { return bRecordingReplay; }
    bool IsPlayingReplay() const { return bPlayingReplay; }
    bool IsGhostVisible() const { return bGhostVisible; }
    bool IsAirflowVisualizationEnabled() const
        { return bAirflowVisualization; }
    bool IsGeometryVisualizationEnabled() const
        { return bGeometryVisualization; }
    int32 GetReplayFrameCount() const { return ReplayFrames.Num(); }
    int32 GetReplayLibraryCount() const { return ReplayFiles.Num(); }
    int32 GetSelectedReplayNumber() const
        { return ReplayFiles.IsEmpty() ? 0 : SelectedReplayFileIndex + 1; }
    const FString& GetReplayDisplayLabel() const { return ReplayDisplayLabel; }
    const char* GetScenarioDisplayName() const;
    const char* GetScenarioObjective() const;
    double GetChallengeScore() const { return Challenge.Score(); }
    double GetChallengeProgress() const { return Challenge.Progress(); }
    double GetChallengeBestScore() const
        { return ChallengeBestScores[SelectedScenarioIndex]; }
    bool IsChallengeComplete() const { return Challenge.IsComplete(); }
    const char* GetChallengeFeedback() const
        { return Parapenting::Physics::ChallengeFeedbackText(Challenge.Feedback()); }
    Parapenting::Physics::WeatherMode GetWeatherMode() const
        { return AirModel.GetMode(); }
    Parapenting::Physics::Vec3 GetBaseWindMps() const
        { return AirModel.GetBaseWind(); }
    const Parapenting::Physics::WeatherSnapshot& GetWeatherSnapshot() const
        { return AirModel.GetSnapshot(); }
    const char* GetHarnessDisplayName() const;
    double GetPilotMassKg() const { return Equipment.pilotMassKg; }
    double GetBallastKg() const { return Equipment.ballastKg; }
    bool IsWithinRecommendedWingRange() const;
    const char* GetWingSizeDisplayName() const;
    const char* GetBrakeTravelDisplayName() const;
    const char* GetWeatherPresetDisplayName() const;
    const FString& GetLiveWeatherStatus() const { return LiveWeatherStatus; }
    bool IsLiveWeatherActive() const { return bLiveWeatherActive; }
    double GetLiveWeatherAgeMinutes() const;
    const char* GetCameraModeDisplayName() const;
    const char* GetPilotRankName() const;
    double GetPilotExperience() const;
    double GetPilotRankProgress() const;
    int32 GetPilotMedalCount() const;
    const char* GetAccessibilityProfileName() const;
    const char* GetKeyboardLayoutName() const;
    FString GetBindingCaptureText() const;
    bool IsCapturingBinding() const { return bCapturingBinding; }
    const char* GetGraphicsProfileName() const;
    int32 GetHudMode() const { return HudMode; }
    const char* GetHudModeName() const;
    bool IsFlightDeckVisible() const { return bFlightDeckVisible; }
    const char* GetSiteWindAssessmentName() const;
    const char* GetLaunchHazardText() const;
    const char* GetLandingCircuitText() const;
    Parapenting::Physics::SiteWindAssessment GetSiteWindAssessment() const;
    Parapenting::Physics::CloudFieldState GetCloudFieldState() const
        { return AirModel.SampleCloudField(SimulationTimeSeconds); }
    Parapenting::Physics::Atmosphere SampleAtmosphereAt(
        const Parapenting::Physics::Vec3& positionWorldM) const
        { return AirModel.Sample(positionWorldM, SimulationTimeSeconds); }
    double GetSimulationTimeSeconds() const { return SimulationTimeSeconds; }
    Parapenting::Physics::DiurnalState GetDiurnalState() const
        { return AirModel.SampleDiurnalState(SimulationTimeSeconds); }
    double GetLocalTimeHours() const
        { return GetDiurnalState().localHour; }
    // Deterministic visual-QA entry point. Kept separate from player input so
    // packaged capture jobs can select an exact sun state without cycling F11.
    void SetVisualQALocalHour(double LocalHour);
    void SetVisualQAWeatherPreset(
        Parapenting::Physics::WeatherPresetId Preset);
    FString GetLocalTimeDisplay() const;
    const char* GetLandingPhaseName() const
        { return Parapenting::Physics::LandingPhaseName(LandingGuidance.phase); }
    double GetApproachQuality() const { return LandingGuidance.approachQuality; }
    bool IsApproachStabilized() const { return LandingGuidance.stabilized; }
    bool IsGroundLaunching() const { return bGroundLaunching; }
    const char* GetLaunchPhaseName() const
        { return Parapenting::Physics::LaunchPhaseName(LaunchState.phase); }
    double GetLaunchInflation() const { return LaunchState.inflation; }
    double GetLaunchRunSpeedMps() const { return LaunchState.pilotRunSpeedMps; }
    bool AreLaunchBrakeSidesCrossed() const
        { return LaunchOutput.brakeSidesCrossed; }
    const Parapenting::Physics::FlightDebriefSummary& GetFlightDebrief() const
        { return Debrief.Summary(); }
    const char* GetFlightPhaseName() const
        { return Parapenting::Physics::FlightPhaseName(
            Debrief.Summary().currentPhase); }
    const char* GetDebriefFocusText() const
        { return Parapenting::Physics::FlightDebriefFocusText(
            Debrief.Summary()); }
    Parapenting::Physics::PreflightBriefing GetPreflightBriefing() const;
    bool IsBriefingVisible() const { return bBriefingVisible; }
    Parapenting::Physics::GlideNavigationSolution
        GetNavigationSolution() const;
    const char* GetActiveWaypointName() const;
    int32 GetActiveWaypointNumber() const
        { return static_cast<int32>(NavigationProgress.activeWaypoint) + 1; }
    bool IsNavigationComplete() const { return NavigationProgress.complete; }
    bool IsLandingFlareScenario() const
        { return Parapenting::Physics::GetTrainingScenarioByIndex(
            SelectedScenarioIndex).id
            == Parapenting::Physics::TrainingScenarioId::LandingFlare; }
    double GetFirstFlareClearanceM() const
        { return Challenge.FirstFlareClearanceM(); }
    double GetPeakFlareAuthority() const
        { return Challenge.PeakFlareAuthority(); }
    const char* GetLandingRolloutPhaseName() const
        { return bLanded
            ? Parapenting::Physics::LandingRolloutPhaseName(
                RolloutState.phase)
            : "AIRBORNE"; }
    double GetRunoutDistanceM() const
        { return RolloutState.runoutDistanceM; }
    bool IsLandingSettled() const
        { return !bLanded
            || RolloutState.phase
                == Parapenting::Physics::LandingRolloutPhase::Settled; }

protected:
    virtual void BeginPlay() override;

private:
    void StepWeightShiftLeft();
    void StepWeightShiftRight();
    void CenterWeightShift();
    void StepLeftBrake();
    void StepRightBrake();
    void StepBothBrakesMore();
    void StepBrakesRelease();
    void SetControllerLeftBrake(float Value);
    void SetControllerRightBrake(float Value);
    void SetControllerWeightShift(float Value);
    void SetControllerAccelerator(float Value);
    void AcceleratorMore();
    void AcceleratorLess();
    void ResetFlight();
    void PrepareGroundLaunch();
    void PrepareReverseGroundLaunch();
    void StartLaunchRun();
    void StopLaunchRun();
    void SetWeatherChill();
    void SetWeatherRidge();
    void SetWeatherLocalizedRotor();
    void SetWeatherRotorEverywhere();
    void SelectTrainingWing();
    void SelectEpicWing();
    void SelectSportWing();
    void SelectEpsilonWing();
    void CycleWing();
    void SelectWing(Parapenting::Physics::WingProfileId id);
    void ApplyEquipmentConfiguration();
    void CycleHarness();
    void PilotMassDown();
    void PilotMassUp();
    void CycleBallast();
    void CycleWingSize();
    void CycleBrakeTravel();
    void CycleWeatherPreset();
    void CycleTimeOfDay();
    void FetchLiveWeather();
    void HandleLiveWeatherResponse(
        FHttpRequestPtr Request, FHttpResponsePtr Response, bool bSucceeded);
    bool ApplyLiveWeatherJson(const FString& Json, bool bFromCache);
    void InjectLeftCollapse();
    void InjectFrontalCollapse();
    void InjectRightCollapse();
    void PreviousRoute();
    void NextRoute();
    void ToggleTelemetryRecording();
    void ToggleReplayRecording();
    void PlayReplay();
    void PreviousReplay();
    void NextReplay();
    void ToggleGhost();
    void ToggleAirflowVisualization();
    void ToggleGeometryVisualization();
    void DrawCanopyGeometryDebug();
    // Solves the suspension network for the controls and load in force now.
    // Called when the debug view is opened rather than every tick: this is a
    // full cold relaxation, and Level 7 is where the solver joins the step.
    void SolveSuspensionGraph();
    void CycleCameraMode();
    void CycleAccessibilityProfile();
    void CycleKeyboardLayout();
    void ApplyKeyboardLayout();
    void CycleBindingAction();
    void BeginBindingCapture();
    void UpdateBindingCapture();
    void CycleGraphicsProfile();
    void ApplyGraphicsProfile();
    void CycleHudMode();
    void TogglePreflightBriefing();
    void OpenFlightDeck();
    void CloseFlightDeck();
    void SaveReplayManifest();
    void RefreshReplayCatalogue();
    bool LoadReplayFile(const FString& Path);
    void LoadPilotProgress();
    void SavePilotProgress() const;
    void RecordTelemetrySample();
    void SaveFlightDebrief() const;
    void NextTrainingScenario();
    void ApplyIncidentCue(Parapenting::Physics::IncidentCue cue);
    void WindSpeedDown();
    void WindSpeedUp();
    void WindRotateLeft();
    void WindRotateRight();
    void ApplyManualWind();
    void UpdateControllerHaptics();
    bool IsKeyDown(const FKey& Key) const;
    void BuildCanopyMesh();
    void UpdateCanopyMesh();
    void UpdatePilotVisual(float DeltaSeconds);
    void UpdatePilotSkeleton(const Parapenting::Physics::PilotPose& Pose);
    void BuildHarnessMesh();
    void CaptureGliderRigSnapshot(double SimulationTimeSeconds);
    void BeginSuspensionMesh();
    void AddSuspensionSegment(
        const FVector& start, const FVector& end,
        const FColor& color, float radiusCm);
    void CommitSuspensionMesh();
    static void PosePilotSegment(
        UStaticMeshComponent* segment,
        const FVector& start, const FVector& end, float radiusScale);

    // The rig hangs off the pilot, not beside them.
    //
    // These three were independent before: the body was posed at
    // PilotPose::rigOffsetCm, the carabiners were rebuilt in actor space from
    // telemetry, and the brake "hands" were invented from the carabiner
    // position rather than being the hands the arms are drawn to. So a weight
    // shift moved the pilot while the lines stayed put, and the brake lines
    // ended in mid air near the hands rather than at them.
    //
    // Everything now derives from PilotRig's own transform, which already
    // carries the weight-shift offset and harness roll. Actor-space
    // centimetres, which is what the draw calls want.
    FTransform PilotRigToActor() const;
    FVector CarabinerLocalCm(bool bLeft) const;
    // Riser tops, where the four groups separate fore/aft. Group order is
    // A, A', B, C from front to back; the mains for a group start here rather
    // than at the carabiner, which is what makes them fan.
    FVector RiserTopLocalCm(int32 Group, bool bLeft) const;
    FVector BrakeHandLocalCm(bool bLeft) const;
    FVector CanopyAttachmentLocalCm(double SpanFraction,
        double ChordFraction) const;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<USceneComponent> Root;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UCameraComponent> Camera;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UProceduralMeshComponent> CanopyVisual;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UProceduralMeshComponent> SuspensionVisual;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UProceduralMeshComponent> GhostCanopyVisual;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> GhostPilotVisual;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UParaglidingAudioComponent> FlightAudio;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> PilotVisual;

    // UE5 Mannequin blockout. A licensed character swaps in through the same
    // Mannequin-compatible skeleton/retargeter without changing rig code.
    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UPoseableMeshComponent> PilotCharacter;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<USceneComponent> PilotRig;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> PilotTorso;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> PilotHead;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> HarnessVisual;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UProceduralMeshComponent> HarnessMesh;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> LeftUpperArm;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> RightUpperArm;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> LeftForearm;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> RightForearm;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> LeftThigh;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> RightThigh;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> LeftShin;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> RightShin;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> LeftBrakeHandle;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UStaticMeshComponent> RightBrakeHandle;

    Parapenting::Physics::ParagliderDynamics Dynamics;
    Parapenting::Physics::AtmosphereModel AirModel;
    Parapenting::Physics::FlightState State;
    Parapenting::Physics::ControlInput Controls;
    Parapenting::Physics::ControlInput ControllerControls;
    Parapenting::Physics::ControlInput AppliedControls;
    Parapenting::Physics::EquipmentSetup Equipment;
    Parapenting::Physics::ChallengeEvaluator Challenge;
    Parapenting::Physics::LandingGuidance LandingGuidance;
    Parapenting::Physics::GroundLaunchModel LaunchModel;
    Parapenting::Physics::GroundLaunchState LaunchState;
    Parapenting::Physics::GroundLaunchOutput LaunchOutput;
    Parapenting::Physics::LandingRolloutModel RolloutModel;
    Parapenting::Physics::LandingRolloutState RolloutState;
    Parapenting::Physics::LandingRolloutOutput RolloutOutput;
    Parapenting::Physics::FlightDebrief Debrief;
    Parapenting::Physics::NavigationRoute NavigationRoute;
    Parapenting::Physics::NavigationProgress NavigationProgress;
    Parapenting::Physics::HapticFeedbackModel HapticModel;
    Parapenting::Physics::WingProfileId SelectedWing =
        Parapenting::Physics::WingProfileId::Epic2MLResearch;
    Parapenting::Physics::WingSize SelectedWingSize =
        Parapenting::Physics::WingSize::Medium;
    Parapenting::Physics::BrakeTravel SelectedBrakeTravel =
        Parapenting::Physics::BrakeTravel::Standard;
    std::size_t SelectedRouteIndex = 0;
    std::size_t SelectedScenarioIndex = 0;
    std::array<double, Parapenting::Physics::ProgressionScenarioCount>
        ChallengeBestScores{};
    bool bChallengeResultRecorded = false;
    double ManualWindFromDegrees = 0.0;
    double ManualWindSpeedMps = 1.5;
    FString LiveWeatherStatus = TEXT("OFFLINE PRESET");
    double LiveWeatherObservedUnixSeconds = 0.0;
    bool bLiveWeatherActive = false;
    bool bLanded = false;
    bool bGroundLaunching = false;
    bool bLaunchHeld = false;
    // Opt-in only, via TogglePreflightBriefing. It used to re-show itself on
    // every wing, weather, preset and time change, each of which also resets
    // the flight, so it covered the viewport almost continuously.
    bool bBriefingVisible = false;
    // This is presentation-only.  It intentionally reads existing settings
    // rather than becoming a second owner for route, weather or accessibility.
    bool bFlightDeckVisible = true;
    float FlightDeckAutoCloseSeconds = 5.0f;
    bool bHardLanding = false;
    bool bRolloutFinalized = false;
    double LandingDistanceM = 0.0;
    double TouchdownVerticalSpeedMps = 0.0;
    double TouchdownHorizontalSpeedMps = 0.0;
    double LandingTargetXM = 2409.9;
    double LandingTargetYM = 0.0;
    // Authoritative canopy geometry. Physics and rendering both read this
    // rather than carrying separate span and chord constants.
    Parapenting::Physics::CanopyGeometry Canopy{};
    // Level 2 suspension: carabiners, risers, mains, cascades, upper
    // galleries, the brake fan and every canopy attachment, built from the
    // geometry above so the lines cannot describe a different wing.
    Parapenting::Physics::SuspensionGraph LineGraph;
    Parapenting::Physics::SuspensionSolution LineSolution;
    // Where the solver puts the cascade junctions, as a shape the render path
    // can apply to the deformed canopy: how far up the riser-to-attachment run
    // the junction sits, and how far off that straight line it hangs. Indexed
    // by LineRow. This replaces the invented sag curve the lines used to be
    // drawn with - the droop on screen is now the droop the solver found under
    // line weight and tension.
    struct SuspensionRenderShape
    {
        float splitAlongRun = 0.6f;
        // Offset of the junction from the straight run, metres, in payload
        // axes for the RIGHT side; the left side mirrors in Y. Kept as a
        // vector because a cascade spreads sideways as well as sagging, and a
        // scalar "sag" would have to invent a direction for it - which is the
        // habit this is replacing.
        FVector OffsetFromRunM = FVector::ZeroVector;
    };
    SuspensionRenderShape LineShape[Parapenting::Physics::LineRowCount];
    TArray<FVector> SuspensionVertices;
    TArray<int32> SuspensionTriangles;
    TArray<FVector> SuspensionNormals;
    TArray<FVector2D> SuspensionUVs;
    TArray<FColor> SuspensionColors;
    bool bSuspensionMeshInitialized = false;
    enum class PilotPoseFamily : uint8
    {
        Seated,
        LaunchRun,
        LandingRun,
        Flare,
        Fallen,
    };
    PilotPoseFamily ActivePilotPoseFamily = PilotPoseFamily::Seated;
    // One weight per additive family. A single shared blend restarted on every
    // family change, which popped the pose back to its base.
    float PilotRunPoseBlend = 0.0f;
    float PilotFlarePoseBlend = 0.0f;
    // The pose the pilot was last drawn in. Cached so the suspension draw can
    // hang the risers off the same body the arms and torso were built from,
    // rather than recomputing an anchor that then disagrees with it.
    Parapenting::Physics::PilotPose LastPilotPose{};
    // Previous/current fixed-step snapshots are the only control source for
    // presentation. RenderRigSnapshot is their bounded frame interpolation.
    Parapenting::Physics::GliderRigSnapshot PreviousRigSnapshot{};
    Parapenting::Physics::GliderRigSnapshot CurrentRigSnapshot{};
    Parapenting::Physics::GliderRigSnapshot RenderRigSnapshot{};
    Parapenting::Physics::ParagliderSolverClock SolverClock{
        PhysicsStepSeconds};
    double SimulationTimeSeconds = 0.0;
    double TelemetryAccumulatorSeconds = 0.0;
    bool bRecordingTelemetry = false;
    FString TelemetryFilePath;
    TArray<Parapenting::Physics::ControlInput> ReplayFrames;
    TArray<FTransform> GhostFrames;
    TArray<FString> ReplayFiles;
    FString ReplayDisplayLabel = TEXT("NO SAVED REPLAY");
    int32 SelectedReplayFileIndex = 0;
    int32 ReplayFrameIndex = 0;
    int32 GhostCaptureStep = 0;
    bool bGhostVisible = false;
    bool bAirflowVisualization = false;
    // Level 1 geometry debug view: rib stations, chord lines,
    // attachment nodes with their row labels, and the solved cell
    // section. Two sign errors in the arc were found by comparing
    // numbers; being able to look at the wing is cheaper.
    bool bGeometryVisualization = false;
    bool bRecordingReplay = false;
    bool bPlayingReplay = false;
    Parapenting::Physics::WingProfileId ReplayWing =
        Parapenting::Physics::WingProfileId::Epic2MLResearch;
    std::size_t ReplayRouteIndex = 0;
    std::size_t ReplayScenarioIndex = 0;
    Parapenting::Physics::WeatherMode ReplayWeatherMode =
        Parapenting::Physics::WeatherMode::LocalizedRotor;
    Parapenting::Physics::WeatherPresetId ReplayWeatherPreset =
        Parapenting::Physics::WeatherPresetId::Custom;
    double ReplayWindFromDegrees = 0.0;
    double ReplayWindSpeedMps = 1.5;
    double ReplayStartLocalHour = 13.0;
    Parapenting::Physics::EquipmentSetup ReplayEquipment;
    Parapenting::Physics::WingSize ReplayWingSize =
        Parapenting::Physics::WingSize::Medium;
    Parapenting::Physics::BrakeTravel ReplayBrakeTravel =
        Parapenting::Physics::BrakeTravel::Standard;
    uint64 LeftForceFeedbackHandle = 0;
    uint64 RightForceFeedbackHandle = 0;
    int32 CameraMode = 0;
    Parapenting::Physics::AccessibilityProfileId AccessibilityProfile =
        Parapenting::Physics::AccessibilityProfileId::FullMotion;
    int32 KeyboardLayoutIndex = 0;
    Parapenting::Physics::InputBindingProfile CustomBindings =
        Parapenting::Physics::InputBindingProfile::Standard();
    int32 SelectedBindingAction = 0;
    bool bCapturingBinding = false;
    FString BindingCaptureStatus = TEXT("F6 SELECT  F7 REBIND");
    int32 HudMode = 0;
    Parapenting::Physics::GraphicsProfileId GraphicsProfile =
        Parapenting::Physics::GraphicsProfileId::High;
    FVector LastCameraVelocityMps = FVector::ZeroVector;
    FVector SmoothedBodyAccelerationMps2 = FVector::ZeroVector;
    static constexpr double PhysicsStepSeconds = 1.0 / 120.0;
    static constexpr double ControlStep = 0.2;
};
