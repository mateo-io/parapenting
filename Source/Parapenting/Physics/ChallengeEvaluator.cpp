#include "ChallengeEvaluator.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
double Clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

double AngleDifference(double a, double b)
{
    return std::atan2(std::sin(a - b), std::cos(a - b));
}
}

void ChallengeEvaluator::Reset(TrainingScenarioId scenario)
{
    *this = {};
    scenario_ = scenario;
    feedback_ = scenario == TrainingScenarioId::ThermalCentering
        ? ChallengeFeedback::SearchForLift
        : ChallengeFeedback::EstablishFlight;
}

void ChallengeEvaluator::Step(const ChallengeSample& s, double dt)
{
    if (complete_ || dt <= 0.0) return;
    elapsedS_ += dt;

    const double collapse = std::max(
        std::max(s.leftCollapse, s.rightCollapse), s.frontalCollapse);
    maximumCollapse_ = std::max(maximumCollapse_, collapse);
    minimumPressure_ = std::min(minimumPressure_, s.canopyPressure);
    liftIntegral_ += std::max(0.0, s.verticalSpeedMps) * dt;
    if (s.thermalLiftMps > 0.35) thermalTimeS_ += dt;
    sinkPenalty_ += std::max(0.0, -s.verticalSpeedMps - 2.2) * dt;
    roughnessPenalty_ += (std::abs(s.rollRateRadps) * 0.7
        + std::abs(s.pitchRateRadps) * 0.5) * s.rotorStrength * dt;
    safetyPenalty_ += (collapse * 2.0 + s.frontalCollapse
        + s.deepStall * 2.0 + std::abs(s.spin) * 1.7
        + (s.leftCravat + s.rightCravat) * 2.5) * dt;
    maximumLoadFactor_ = std::max(maximumLoadFactor_, s.loadFactor);
    if (s.loadFactor > 3.5)
        highLoadTimeS_ += dt;
    if (s.groundClearanceM < 190.0
        && s.landingPhase != LandingPhase::Arrival)
    {
        approachQualityIntegral_ += Clamp(s.approachQuality, 0.0, 1.0) * dt;
        approachObservedTimeS_ += dt;
        if (s.stabilizedApproach)
            stabilizedFinalTimeS_ += dt;
    }
    if (scenario_ == TrainingScenarioId::LandingFlare)
    {
        minimumFlareEnergy_ = std::min(
            minimumFlareEnergy_, s.flareEnergy);
        peakFlareAuthority_ = std::max(
            peakFlareAuthority_, s.flareAuthority);
        const double symmetricBrake =
            0.5 * (s.leftBrake + s.rightBrake);
        if (firstFlareClearanceM_ < 0.0
            && s.groundClearanceM < 30.0
            && (s.flareAuthority > 0.025 || symmetricBrake > 0.52))
        {
            firstFlareClearanceM_ = s.groundClearanceM;
        }
        if (s.groundClearanceM < 10.0 && symmetricBrake > 0.72)
            deepBrakeNearGroundTimeS_ += dt;
    }

    if (scenario_ == TrainingScenarioId::SpiralRecovery)
    {
        if (!spiralObserved_
            && (std::abs(s.yawRateRadps) > 0.45 || s.loadFactor > 2.0))
            spiralObserved_ = true;
        if (spiralObserved_)
        {
            exitRoughnessPenalty_ += (
                std::abs(s.pitchRateRadps) * 0.45
                + std::max(0.0, s.recoverySurge) * 2.8) * dt;
            const bool stableExit =
                std::abs(s.yawRateRadps) < 0.18
                && std::abs(s.rollRateRadps) < 0.22
                && s.loadFactor < 1.8
                && s.highLoadDeformation < 0.18;
            stableSpiralExitTimeS_ = stableExit
                ? stableSpiralExitTimeS_ + dt : 0.0;
            if (stableSpiralExitTimeS_ < 1.2)
                spiralRecoverySeconds_ += dt;
        }
    }

    if (!incidentObserved_ && collapse > 0.2)
    {
        incidentObserved_ = true;
        incidentHeadingRad_ = s.yawRad;
    }
    if (incidentObserved_)
    {
        recoverySeconds_ += dt;
        maximumHeadingDeviationRad_ = std::max(maximumHeadingDeviationRad_,
            std::abs(AngleDifference(s.yawRad, incidentHeadingRad_)));
        const bool recovered = collapse < 0.08 && s.canopyPressure > 0.78
            && std::abs(s.spin) < 0.15;
        stableRecoveryTimeS_ = recovered ? stableRecoveryTimeS_ + dt : 0.0;
    }

    if (scenario_ == TrainingScenarioId::LandingFlare)
    {
        const double symmetricBrake =
            0.5 * (s.leftBrake + s.rightBrake);
        if (s.groundClearanceM > 18.0)
            feedback_ = s.stabilizedApproach
                ? ChallengeFeedback::PreserveFlareEnergy
                : ChallengeFeedback::StabilizeApproach;
        else if (s.groundClearanceM > 5.5)
            feedback_ = symmetricBrake > 0.45
                ? ChallengeFeedback::PreserveFlareEnergy
                : ChallengeFeedback::SetUpLanding;
        else if (s.groundClearanceM > 0.9)
            feedback_ = ChallengeFeedback::Flare;
        else
            feedback_ = ChallengeFeedback::RunOut;
    }
    else if (s.groundClearanceM < 230.0)
    {
        if (s.groundClearanceM < 8.0)
            feedback_ = ChallengeFeedback::Flare;
        else if (s.landingPhase == LandingPhase::Downwind)
            feedback_ = ChallengeFeedback::TurnBase;
        else if (s.landingPhase == LandingPhase::Base)
            feedback_ = ChallengeFeedback::EstablishFinal;
        else if (s.landingPhase == LandingPhase::Final)
            feedback_ = s.stabilizedApproach
                ? ChallengeFeedback::SetUpLanding
                : ChallengeFeedback::StabilizeApproach;
        else
            feedback_ = ChallengeFeedback::JoinDownwind;
    }
    else if (collapse > 0.18)
        feedback_ = s.canopyPressure < 0.55
            ? ChallengeFeedback::RebuildPressure
            : ChallengeFeedback::ControlHeading;
    else if (s.recoverySurge > 0.035)
        feedback_ = ChallengeFeedback::ContainSurge;
    else if (s.deepStall > 0.25 || std::abs(s.spin) > 0.25)
        feedback_ = ChallengeFeedback::ReleaseBrakes;
    else if (scenario_ == TrainingScenarioId::SpiralRecovery)
    {
        if (s.loadFactor > 4.0 || s.highLoadDeformation > 0.72)
            feedback_ = ChallengeFeedback::EaseBrake;
        else if (std::abs(s.yawRateRadps) > 0.35
                 || std::abs(s.rollRateRadps) > 0.4)
            feedback_ = ChallengeFeedback::ReduceBank;
        else if (spiralObserved_ && stableSpiralExitTimeS_ < 1.2)
            feedback_ = ChallengeFeedback::StabilizeExit;
        else
            feedback_ = ChallengeFeedback::ActivePiloting;
    }
    else if (scenario_ == TrainingScenarioId::ThermalCentering)
        feedback_ = s.thermalLiftMps > 0.45
            ? ChallengeFeedback::CenterCore
            : (s.verticalSpeedMps < -2.5
                ? ChallengeFeedback::AvoidSinkRing
                : ChallengeFeedback::SearchForLift);
    else if (scenario_ == TrainingScenarioId::LeeRotor
             || scenario_ == TrainingScenarioId::Cascade)
        feedback_ = ChallengeFeedback::ActivePiloting;

    UpdateScore();
    if (scenario_ != TrainingScenarioId::FreeFlight
        && elapsedS_ + 1e-6 >= TargetDurationS())
    {
        complete_ = true;
        feedback_ = ChallengeFeedback::Complete;
    }
}

void ChallengeEvaluator::FinalizeLanding(
    double distanceM, double verticalSpeedMps, double horizontalSpeedMps)
{
    const double accuracy = 350.0 * (1.0 - Clamp(distanceM / 220.0, 0.0, 1.0));
    const double softness = 250.0
        * (1.0 - Clamp((std::abs(verticalSpeedMps) - 0.8) / 5.2, 0.0, 1.0));
    const double energy = 200.0
        * (1.0 - Clamp((horizontalSpeedMps - 3.0) / 11.0, 0.0, 1.0));
    const double averageApproach = approachObservedTimeS_ > 0.01
        ? approachQualityIntegral_ / approachObservedTimeS_ : 1.0;
    const double stabilizedBonus =
        Clamp(stabilizedFinalTimeS_ / 8.0, 0.0, 1.0);
    const double approach = 200.0
        * Clamp(0.8 * averageApproach + 0.2 * stabilizedBonus, 0.0, 1.0);
    const double landingScore = accuracy + softness + energy + approach;
    if (scenario_ == TrainingScenarioId::LandingFlare)
    {
        const double flareTiming = firstFlareClearanceM_ < 0.0
            ? 0.0
            : 1.0 - Clamp(
                std::abs(firstFlareClearanceM_ - 2.6) / 7.0, 0.0, 1.0);
        const double runout = 1.0 - Clamp(
            std::max(0.0, horizontalSpeedMps - 8.0) / 6.0, 0.0, 1.0);
        const double technique =
            Clamp(peakFlareAuthority_ / 0.16, 0.0, 1.0)
            * (1.0 - Clamp(
                std::max(0.0, deepBrakeNearGroundTimeS_ - 1.2) / 2.0,
                0.0, 1.0));
        score_ = Clamp(
            200.0 * (1.0 - Clamp(distanceM / 120.0, 0.0, 1.0))
            + 250.0 * (1.0 - Clamp(
                (std::abs(verticalSpeedMps) - 0.7) / 4.3, 0.0, 1.0))
            + 150.0 * runout
            + 250.0 * flareTiming
            + 100.0 * technique
            + 50.0 * Clamp(averageApproach, 0.0, 1.0),
            0.0, 1000.0);
    }
    else
    {
        score_ = scenario_ == TrainingScenarioId::FreeFlight
            ? landingScore
            : Clamp(score_ * 0.7 + landingScore * 0.3, 0.0, 1000.0);
    }
    complete_ = true;
    feedback_ = ChallengeFeedback::Complete;
}

void ChallengeEvaluator::FinalizeRollout(double runoutDistanceM, bool fell)
{
    if (scenario_ != TrainingScenarioId::LandingFlare
        || rolloutAdjustmentApplied_)
        return;
    rolloutAdjustmentApplied_ = true;
    const double runoutQuality = 1.0 - Clamp(
        std::abs(runoutDistanceM - 7.0) / 14.0, 0.0, 1.0);
    score_ = Clamp(
        score_ - 100.0 + runoutQuality * 100.0
            - (fell ? 180.0 : 0.0),
        0.0, 1000.0);
}

double ChallengeEvaluator::Progress() const
{
    if (complete_) return 1.0;
    if (scenario_ == TrainingScenarioId::FreeFlight) return 0.0;
    return Clamp(elapsedS_ / TargetDurationS(), 0.0, 1.0);
}

double ChallengeEvaluator::TargetDurationS() const
{
    switch (scenario_)
    {
        case TrainingScenarioId::ThermalCentering: return 60.0;
        case TrainingScenarioId::LeeRotor: return 45.0;
        case TrainingScenarioId::AsymmetricRecovery: return 25.0;
        case TrainingScenarioId::FrontalRecovery: return 25.0;
        case TrainingScenarioId::Cascade: return 40.0;
        case TrainingScenarioId::SpiralRecovery: return 35.0;
        case TrainingScenarioId::LandingFlare: return 60.0;
        default: return 0.0;
    }
}

void ChallengeEvaluator::UpdateScore()
{
    switch (scenario_)
    {
        case TrainingScenarioId::ThermalCentering:
            score_ = 250.0 + liftIntegral_ * 15.0 + thermalTimeS_ * 7.0
                - sinkPenalty_ * 8.0 - roughnessPenalty_ * 3.0
                - safetyPenalty_ * 10.0;
            break;
        case TrainingScenarioId::LeeRotor:
            score_ = 1000.0 - roughnessPenalty_ * 16.0
                - safetyPenalty_ * 28.0 - std::max(0.0, 0.55 - minimumPressure_) * 350.0;
            break;
        case TrainingScenarioId::AsymmetricRecovery:
        case TrainingScenarioId::FrontalRecovery:
        case TrainingScenarioId::Cascade:
            score_ = 1000.0 - safetyPenalty_ * 18.0
                - maximumHeadingDeviationRad_ * 180.0
                - (incidentObserved_ ? recoverySeconds_ * 4.0 : 0.0);
            if (stableRecoveryTimeS_ > 1.0) score_ += 80.0;
            break;
        case TrainingScenarioId::SpiralRecovery:
            score_ = 1000.0
                - highLoadTimeS_ * 28.0
                - std::max(0.0, maximumLoadFactor_ - 3.5) * 110.0
                - (spiralObserved_ ? spiralRecoverySeconds_ * 5.0 : 180.0)
                - exitRoughnessPenalty_ * 32.0
                - safetyPenalty_ * 20.0;
            if (stableSpiralExitTimeS_ > 1.2) score_ += 100.0;
            break;
        case TrainingScenarioId::LandingFlare:
            score_ = 250.0
                + approachQualityIntegral_ * 2.0
                + stabilizedFinalTimeS_ * 4.0
                - safetyPenalty_ * 25.0;
            break;
        case TrainingScenarioId::FreeFlight:
            score_ = 0.0;
            break;
    }
    score_ = Clamp(score_, 0.0, 1000.0);
}

const char* ChallengeFeedbackText(ChallengeFeedback feedback)
{
    switch (feedback)
    {
        case ChallengeFeedback::SearchForLift: return "SEARCH FOR LIFT";
        case ChallengeFeedback::CenterCore: return "CENTER THE CORE";
        case ChallengeFeedback::AvoidSinkRing: return "EXIT THE SINK RING";
        case ChallengeFeedback::ActivePiloting: return "ACTIVE PILOTING";
        case ChallengeFeedback::ReduceBank: return "PROGRESSIVELY REDUCE BANK";
        case ChallengeFeedback::EaseBrake: return "EASE INSIDE BRAKE - MANAGE LOAD";
        case ChallengeFeedback::StabilizeExit: return "STABILIZE PITCH AFTER EXIT";
        case ChallengeFeedback::ControlHeading: return "CONTROL HEADING";
        case ChallengeFeedback::ReleaseBrakes: return "HANDS UP - REGAIN AIRSPEED";
        case ChallengeFeedback::RebuildPressure: return "REBUILD CANOPY PRESSURE";
        case ChallengeFeedback::ContainSurge: return "CONTAIN THE SURGE";
        case ChallengeFeedback::SetUpLanding: return "SET UP THE APPROACH";
        case ChallengeFeedback::JoinDownwind: return "JOIN THE DOWNWIND LEG";
        case ChallengeFeedback::TurnBase: return "TURN BASE";
        case ChallengeFeedback::EstablishFinal: return "ESTABLISH FINAL INTO WIND";
        case ChallengeFeedback::StabilizeApproach: return "STABILIZE HEADING AND SINK";
        case ChallengeFeedback::PreserveFlareEnergy: return "HANDS UP - PRESERVE FLARE ENERGY";
        case ChallengeFeedback::Flare: return "FLARE PROGRESSIVELY";
        case ChallengeFeedback::RunOut: return "RUN OUT - KEEP THE WING OPEN";
        case ChallengeFeedback::Complete: return "CHALLENGE COMPLETE";
        default: return "ESTABLISH STABLE FLIGHT";
    }
}
}
