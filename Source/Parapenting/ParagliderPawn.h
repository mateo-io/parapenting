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
#include <vector>
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"
#include "ParagliderPawn.generated.h"

class UCameraComponent;
class UProceduralMeshComponent;
class UParaglidingAudioComponent;
class UPoseableMeshComponent;
class USkeletalMesh;
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
    bool IsPhotoMode() const { return bPhotoMode; }
    const char* GetSiteWindAssessmentName() const;
    FString GetScenicLandmarkText() const;
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
    void SetVisualQACameraMode(int32 Mode);
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
    void TogglePhotoMode();
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
    // How much of the brake travel this span station actually sees. A brake
    // fan pulls at its own attachment stations and the cloth between them
    // follows; applying the full travel to the whole half-span made the
    // trailing edge drop as one rigid flap.
    float BrakeStationInfluence(float SpanFraction) const;
    void RefreshBrakeStationCache();

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

    // The pilot body mesh, assigned in the editor. Leave it empty to use the
    // engine mannequin. A MetaHuman body mesh goes here: its core bone names
    // (pelvis, spine_01, spine_03, clavicle/upperarm/lowerarm/hand,
    // thigh/calf/foot, head) are the ones UpdatePilotSkeleton drives, so no
    // code changes when it is set. Assign the BODY mesh - the face is a
    // separate mesh and this rig has nothing to say to it.
    UPROPERTY(EditAnywhere, Category = "Pilot")
    TSoftObjectPtr<USkeletalMesh> PilotMeshOverride;

    // Whether the assigned mesh has been checked against the bones the rig
    // actually drives. Checked once, on the first pose, and reported.
    bool bPilotSkeletonValidated = false;

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
    // Render/HUD-only mode. It must not affect replay inputs, camera feedback,
    // graphics tiers or the fixed simulation clock.
    bool bPhotoMode = false;
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
    // Span fractions of the authoritative brake attachments, and the spanwise
    // reach of one station, derived from their spacing. Refreshed whenever the
    // graph is solved so a different wing changes the deformation.
    std::vector<double> BrakeStationSpans;
    double BrakeStationReach = 0.12;
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
    // Weight shift and accelerator. The brakes used to share this and no
    // longer do - see `BrakeLevels`.
    static constexpr double ControlStep = 0.2;

    // THE BRAKE LADDER, AND IT IS NOT A UNIFORM STEP BECAUSE THE WING IS NOT
    // UNIFORM. The brakes were on `ControlStep` too, which put the pilot on
    // {0, 0.2, 0.4, 0.6, 0.8, 1.0} - and measured on the flight model, exactly
    // ONE of those rungs is a stall:
    //
    //     rung        0.00  0.20  0.40  0.60  0.80  1.00
    //     sink m/s    1.14  1.11  1.62  4.02  7.52  5.43
    //     deepStall   0     0     0     0     0     0.957
    //
    // 0.80 is not a stall at all, it is a deep-braked mush. So every stall the
    // pilot could fly was the same input, which is why they all felt identical
    // - `PHYSICS_TODO` item 24, where the physics was fixed to carry stall
    // depth and the control could not reach it.
    //
    // The spacing follows where the wing actually changes. Nothing happens
    // between 0 and 0.30 (sink 1.14 to 1.13), so those rungs are cheap; the
    // entire stall lives between 0.84 and 1.00, so that is where the ladder
    // tightens. Measured at these rungs, `deepStall` runs 0, 0, 0, 0, 0,
    // 0.614, 0.822, 0.957 and the recovery surge 0.03, 1.12, 3.12, 4.5, 7.0,
    // 8.45, 10.85, 11.83, 11.98 m/s - three distinguishable stalls where there
    // was one, and monotone.
    //
    // Eight presses to full rather than five. That is deliberate and it is the
    // cost: there is no separate flare key, so a landing flare is repeated
    // presses of the same control. The bottom of the ladder is kept coarse for
    // exactly that reason - ordinary flying and the start of a flare are two
    // rungs, and only the stall band is expensive to reach, which is the right
    // way round for a control that ends in a stall.
    static constexpr double BrakeLevels[] = {
        0.00, 0.20, 0.40, 0.58, 0.72, 0.84, 0.90, 0.95, 1.00};
    static constexpr int BrakeLevelCount =
        static_cast<int>(sizeof(BrakeLevels) / sizeof(BrakeLevels[0]));
    // One rung up (+1) or down (-1) from wherever the control currently sits.
    // Takes the current value rather than an index because the controller axis
    // writes continuous values into the same control, so the keyboard cannot
    // assume the last thing that moved it was the keyboard.
    static double SteppedBrake(double current, int direction);
};
