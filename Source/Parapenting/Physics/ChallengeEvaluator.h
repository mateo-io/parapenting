#pragma once

#include "TrainingScenario.h"
#include "LandingCircuitModel.h"

namespace Parapenting::Physics
{
enum class ChallengeFeedback
{
    EstablishFlight,
    SearchForLift,
    CenterCore,
    AvoidSinkRing,
    ActivePiloting,
    ReduceBank,
    EaseBrake,
    StabilizeExit,
    ControlHeading,
    ReleaseBrakes,
    RebuildPressure,
    ContainSurge,
    SetUpLanding,
    JoinDownwind,
    TurnBase,
    EstablishFinal,
    StabilizeApproach,
    PreserveFlareEnergy,
    Flare,
    RunOut,
    Complete
};

struct ChallengeSample
{
    double verticalSpeedMps = 0.0;
    double thermalLiftMps = 0.0;
    double rotorStrength = 0.0;
    double airspeedMps = 0.0;
    double yawRad = 0.0;
    double rollRateRadps = 0.0;
    double pitchRateRadps = 0.0;
    double yawRateRadps = 0.0;
    double loadFactor = 1.0;
    double highLoadDeformation = 0.0;
    double leftCollapse = 0.0;
    double rightCollapse = 0.0;
    double frontalCollapse = 0.0;
    double canopyPressure = 1.0;
    double recoverySurge = 0.0;
    double deepStall = 0.0;
    double spin = 0.0;
    double leftCravat = 0.0;
    double rightCravat = 0.0;
    double leftBrake = 0.0;
    double rightBrake = 0.0;
    double distanceToLandingM = 0.0;
    double groundClearanceM = 1000.0;
    LandingPhase landingPhase = LandingPhase::Arrival;
    double approachQuality = 0.0;
    bool stabilizedApproach = false;
    double groundEffect = 0.0;
    double flareEnergy = 0.0;
    double flareAuthority = 0.0;
};

class ChallengeEvaluator
{
public:
    void Reset(TrainingScenarioId scenario);
    void Step(const ChallengeSample& sample, double dt);
    void FinalizeLanding(double distanceM, double verticalSpeedMps,
                         double horizontalSpeedMps);
    void FinalizeRollout(double runoutDistanceM, bool fell);

    double Score() const { return score_; }
    double Progress() const;
    double ElapsedSeconds() const { return elapsedS_; }
    bool IsComplete() const { return complete_; }
    ChallengeFeedback Feedback() const { return feedback_; }
    double MaximumCollapse() const { return maximumCollapse_; }
    double RecoverySeconds() const { return recoverySeconds_; }
    double FirstFlareClearanceM() const { return firstFlareClearanceM_; }
    double PeakFlareAuthority() const { return peakFlareAuthority_; }

private:
    void UpdateScore();
    double TargetDurationS() const;

    TrainingScenarioId scenario_ = TrainingScenarioId::FreeFlight;
    ChallengeFeedback feedback_ = ChallengeFeedback::EstablishFlight;
    double elapsedS_ = 0.0;
    double score_ = 0.0;
    double liftIntegral_ = 0.0;
    double thermalTimeS_ = 0.0;
    double sinkPenalty_ = 0.0;
    double roughnessPenalty_ = 0.0;
    double safetyPenalty_ = 0.0;
    double maximumCollapse_ = 0.0;
    double minimumPressure_ = 1.0;
    double maximumHeadingDeviationRad_ = 0.0;
    double incidentHeadingRad_ = 0.0;
    double recoverySeconds_ = 0.0;
    double stableRecoveryTimeS_ = 0.0;
    double maximumLoadFactor_ = 1.0;
    double highLoadTimeS_ = 0.0;
    double spiralRecoverySeconds_ = 0.0;
    double stableSpiralExitTimeS_ = 0.0;
    double exitRoughnessPenalty_ = 0.0;
    double approachQualityIntegral_ = 0.0;
    double approachObservedTimeS_ = 0.0;
    double stabilizedFinalTimeS_ = 0.0;
    double firstFlareClearanceM_ = -1.0;
    double peakFlareAuthority_ = 0.0;
    double minimumFlareEnergy_ = 1.0;
    double deepBrakeNearGroundTimeS_ = 0.0;
    bool incidentObserved_ = false;
    bool spiralObserved_ = false;
    bool complete_ = false;
    bool rolloutAdjustmentApplied_ = false;
};

const char* ChallengeFeedbackText(ChallengeFeedback feedback);
}
