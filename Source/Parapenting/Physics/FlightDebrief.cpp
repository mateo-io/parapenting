#include "FlightDebrief.h"

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
}

void FlightDebrief::Reset()
{
    *this = {};
}

void FlightDebrief::Step(const FlightDebriefSample& s, double deltaSeconds)
{
    if (summary_.landed) return;
    const double dt = Clamp(deltaSeconds, 0.0, 0.1);
    if (dt <= 0.0) return;
    summary_.durationS += dt;

    if (hasPreviousPosition_)
    {
        const Vec3 delta = s.positionWorldM - previousPosition_;
        summary_.horizontalDistanceM += std::hypot(delta.x, delta.y);
        summary_.altitudeGainM += std::max(0.0, delta.z);
        summary_.altitudeLossM += std::max(0.0, -delta.z);
        if (s.thermalLiftMps > 0.35)
            summary_.thermalGainM += std::max(0.0, delta.z);
    }
    previousPosition_ = s.positionWorldM;
    hasPreviousPosition_ = true;

    if (s.thermalLiftMps > 0.35)
        summary_.thermalTimeS += dt;
    if (s.rotorStrength > 0.25)
        summary_.rotorExposureS += dt;
    if (s.rotorStrength > 0.65)
        summary_.severeRotorExposureS += dt;
    if (s.loadFactor > 3.5)
        summary_.highLoadTimeS += dt;
    if (s.stabilizedApproach)
        summary_.stabilizedFinalS += dt;
    if (s.groundClearanceM < 190.0 && !s.groundLaunching)
    {
        approachQualityIntegral_ += Clamp(s.approachQuality, 0.0, 1.0) * dt;
        approachTimeS_ += dt;
        summary_.averageApproachQuality =
            approachQualityIntegral_ / approachTimeS_;
    }

    summary_.maximumClimbMps =
        std::max(summary_.maximumClimbMps, s.verticalSpeedMps);
    summary_.maximumSinkMps =
        std::min(summary_.maximumSinkMps, s.verticalSpeedMps);
    summary_.maximumAirspeedMps =
        std::max(summary_.maximumAirspeedMps, s.airspeedMps);
    summary_.maximumLoadFactor =
        std::max(summary_.maximumLoadFactor, s.loadFactor);
    summary_.minimumCanopyPressure =
        std::min(summary_.minimumCanopyPressure, s.canopyPressure);
    summary_.controlActivity += (
        std::abs(s.leftBrake - previousLeftBrake_)
        + std::abs(s.rightBrake - previousRightBrake_)
        + 0.7 * std::abs(s.weightShift - previousWeightShift_));
    previousLeftBrake_ = s.leftBrake;
    previousRightBrake_ = s.rightBrake;
    previousWeightShift_ = s.weightShift;

    const bool collapse =
        std::max(s.leftCollapse, s.rightCollapse) > 0.2;
    const bool frontal = s.frontalCollapse > 0.2;
    const bool cravat = s.leftCravat + s.rightCravat > 0.08;
    const bool stallSpin = s.deepStall > 0.25 || std::abs(s.spin) > 0.25;
    if (collapse && !collapseActive_) ++summary_.asymmetricCollapseEvents;
    if (frontal && !frontalActive_) ++summary_.frontalCollapseEvents;
    if (cravat && !cravatActive_) ++summary_.cravatEvents;
    if (stallSpin && !stallSpinActive_) ++summary_.stallOrSpinEvents;
    collapseActive_ = collapse;
    frontalActive_ = frontal;
    cravatActive_ = cravat;
    stallSpinActive_ = stallSpin;

    if (s.groundLaunching)
        summary_.currentPhase = FlightPhase::Launch;
    else if (collapse || frontal || cravat || stallSpin)
        summary_.currentPhase = FlightPhase::Incident;
    else if (s.groundClearanceM < 230.0 || s.distanceToLandingM < 800.0)
        summary_.currentPhase = FlightPhase::Approach;
    else if (s.rotorStrength > 0.3)
        summary_.currentPhase = FlightPhase::Rotor;
    else if (s.thermalLiftMps > 0.35)
        summary_.currentPhase = FlightPhase::Thermal;
    else
        summary_.currentPhase = FlightPhase::Glide;

    UpdateRatings();
}

void FlightDebrief::FinalizeLanding(
    double distanceM, double verticalSpeedMps, double horizontalSpeedMps)
{
    summary_.landingDistanceM = std::max(0.0, distanceM);
    summary_.touchdownVerticalSpeedMps = verticalSpeedMps;
    summary_.touchdownHorizontalSpeedMps = std::max(0.0, horizontalSpeedMps);
    summary_.landed = true;
    summary_.currentPhase = FlightPhase::Landed;
    UpdateRatings();
}

void FlightDebrief::UpdateRatings()
{
    summary_.safetyRating = Clamp(
        100.0
        - summary_.asymmetricCollapseEvents * 7.0
        - summary_.frontalCollapseEvents * 9.0
        - summary_.cravatEvents * 14.0
        - summary_.stallOrSpinEvents * 16.0
        - summary_.highLoadTimeS * 1.7
        - std::max(0.0, summary_.maximumLoadFactor - 4.0) * 12.0
        - summary_.severeRotorExposureS * 0.15,
        0.0, 100.0);

    const double glideRatio = summary_.horizontalDistanceM
        / std::max(20.0, summary_.altitudeLossM);
    const double activityPerMinute = summary_.durationS > 1.0
        ? summary_.controlActivity * 60.0 / summary_.durationS : 0.0;
    summary_.efficiencyRating = Clamp(
        glideRatio / 9.0 * 88.0 + 12.0
        - std::max(0.0, activityPerMinute - 0.25) * 7.0,
        0.0, 100.0);

    if (summary_.thermalTimeS > 1.0)
    {
        const double meanThermalClimb =
            summary_.thermalGainM / summary_.thermalTimeS;
        summary_.thermalRating = Clamp(
            25.0 + meanThermalClimb / 2.5 * 60.0
            + std::min(15.0, summary_.thermalTimeS / 12.0),
            0.0, 100.0);
    }

    if (summary_.landed)
    {
        const double accuracy =
            1.0 - Clamp(summary_.landingDistanceM / 220.0, 0.0, 1.0);
        const double softness = 1.0 - Clamp(
            (std::abs(summary_.touchdownVerticalSpeedMps) - 0.8) / 5.2,
            0.0, 1.0);
        const double energy = 1.0 - Clamp(
            (summary_.touchdownHorizontalSpeedMps - 3.0) / 11.0,
            0.0, 1.0);
        const double stable = Clamp(
            summary_.stabilizedFinalS / 8.0, 0.0, 1.0);
        summary_.landingRating = 100.0 * Clamp(
            accuracy * 0.34 + softness * 0.24 + energy * 0.18
            + summary_.averageApproachQuality * 0.18 + stable * 0.06,
            0.0, 1.0);
    }
    summary_.overallRating =
        summary_.safetyRating * 0.34
        + summary_.efficiencyRating * 0.24
        + summary_.thermalRating * 0.20
        + summary_.landingRating * 0.22;
}

const char* FlightPhaseName(FlightPhase phase)
{
    switch (phase)
    {
        case FlightPhase::Launch: return "LAUNCH";
        case FlightPhase::Glide: return "GLIDE";
        case FlightPhase::Thermal: return "THERMAL";
        case FlightPhase::Rotor: return "ROTOR";
        case FlightPhase::Incident: return "INCIDENT";
        case FlightPhase::Approach: return "APPROACH";
        case FlightPhase::Landed: return "LANDED";
    }
    return "GLIDE";
}

const char* FlightDebriefFocusText(const FlightDebriefSummary& s)
{
    if (s.safetyRating <= s.efficiencyRating
        && s.safetyRating <= s.thermalRating
        && s.safetyRating <= s.landingRating)
        return "FOCUS: ACTIVE SAFETY AND LOAD MANAGEMENT";
    if (s.landingRating <= s.efficiencyRating
        && s.landingRating <= s.thermalRating)
        return "FOCUS: STABILIZED CIRCUIT AND ENERGY AT FLARE";
    if (s.thermalRating <= s.efficiencyRating)
        return "FOCUS: CENTER CORE AND REDUCE SINK-RING TIME";
    return "FOCUS: SMOOTHER INPUTS AND BETTER GLIDE LINE";
}
}
