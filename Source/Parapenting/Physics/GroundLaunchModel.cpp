#include "GroundLaunchModel.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double GravityMps2 = 9.80665;

double Clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}
}

void GroundLaunchModel::Reset(
    GroundLaunchState& state, bool reverseLaunch) const
{
    state = {};
    state.reverseLaunch = reverseLaunch;
    state.pilotFacingYawOffsetRad =
        reverseLaunch ? 3.14159265358979323846 : 0.0;
}

GroundLaunchOutput GroundLaunchModel::Step(
    GroundLaunchState& state, const GroundLaunchInput& input, double deltaSeconds) const
{
    GroundLaunchOutput output;
    output.pilotFacingYawOffsetRad = state.pilotFacingYawOffsetRad;
    output.brakeSidesCrossed =
        state.reverseLaunch
        && state.phase != LaunchPhase::Running
        && state.phase != LaunchPhase::Airborne;
    if (state.phase == LaunchPhase::Airborne)
    {
        output.liftOff = true;
        output.pilotFacingYawOffsetRad = 0.0;
        return output;
    }

    const double dt = Clamp(deltaSeconds, 0.0, 1.0 / 60.0);
    if (dt <= 0.0) return output;
    const Vec3 forward = Normalized(Vec3{
        input.launchDirection.x, input.launchDirection.y, 0.0});
    const Vec3 left{-forward.y, forward.x, 0.0};
    output.headwindMps = std::max(0.0, -Dot(input.surfaceWindMps, forward));
    output.crosswindMps = Dot(input.surfaceWindMps, left);

    if (state.phase == LaunchPhase::LaidOut && input.launchHeld)
        state.phase = LaunchPhase::Inflation;
    if (state.phase == LaunchPhase::Aborted)
    {
        state.inflation = std::max(0.0, state.inflation - dt * 1.6);
        state.pilotRunSpeedMps = std::max(0.0, state.pilotRunSpeedMps - dt * 3.0);
        state.canopyElevationRad += (-1.35 - state.canopyElevationRad)
            * std::min(1.0, dt * 4.0);
        output.abort = true;
        return output;
    }

    if (state.phase == LaunchPhase::LaidOut)
        return output;

    state.elapsedS += dt;
    const double symmetricBrake =
        0.5 * (input.leftBrake + input.rightBrake);
    const double differentialBrake = input.rightBrake - input.leftBrake;
    const double slopeAcceleration =
        GravityMps2 * std::sin(Clamp(input.slopeDownRadians, -0.1, 0.45));
    const double supportRelief = 1.0 - 0.42 * state.liftFraction;
    const bool reversePreparation =
        state.reverseLaunch
        && state.phase != LaunchPhase::Running
        && state.phase != LaunchPhase::Airborne;
    const double runAcceleration = reversePreparation
        ? (input.launchHeld ? 0.45 : -2.0)
        : (input.launchHeld
            ? (1.65 + slopeAcceleration) * supportRelief : -3.2);
    const double maximumRunSpeed = reversePreparation ? 1.4 : 8.2;
    state.pilotRunSpeedMps = Clamp(
        state.pilotRunSpeedMps + runAcceleration * dt, 0.0, maximumRunSpeed);
    output.apparentWindMps = state.pilotRunSpeedMps + output.headwindMps;

    const double inflationAuthority = Clamp(
        (output.apparentWindMps - 2.2) / 6.0, 0.0, 1.0);
    const double brakeInflationLoss = Clamp(
        (symmetricBrake - 0.28) / 0.5, 0.0, 1.0);
    const double inflationRate =
        0.72 * inflationAuthority * (1.0 - 0.85 * brakeInflationLoss);
    const double deflationRate =
        output.apparentWindMps < 2.4 || !input.launchHeld ? 0.42 : 0.0;
    state.inflation = Clamp(
        state.inflation + (inflationRate - deflationRate) * dt, 0.0, 1.0);
    state.canopyElevationRad =
        -1.35 + state.inflation * (1.35 + 0.16 * inflationAuthority);

    const double crosswindTorque =
        output.crosswindMps * (0.12 + 0.16 * state.inflation);
    const double pilotCorrection =
        (state.phase == LaunchPhase::Turning ? 0.0 : input.weightShift * 0.48)
        + differentialBrake * 0.82;
    state.canopyHeadingErrorRad +=
        (crosswindTorque - pilotCorrection
         - state.canopyHeadingErrorRad * 0.85) * dt;
    state.canopyHeadingErrorRad = Clamp(
        state.canopyHeadingErrorRad, -1.45, 1.45);

    const double dynamicPressure =
        0.5 * 1.12 * output.apparentWindMps * output.apparentWindMps;
    state.lineLoadN = dynamicPressure * input.wingAreaM2
        * state.inflation * 0.68;
    const double supportedWeight =
        std::max(1.0, input.allUpMassKg) * GravityMps2;
    state.liftFraction = Clamp(
        state.lineLoadN / supportedWeight, 0.0, 1.25);

    const bool overhead =
        state.inflation > 0.88
        && std::abs(state.canopyElevationRad) < 0.24;
    const bool centered = std::abs(state.canopyHeadingErrorRad) < 0.28;
    state.stableOverheadS = overhead && centered
        ? state.stableOverheadS + dt : 0.0;
    if (state.phase == LaunchPhase::Inflation && overhead)
        state.phase = LaunchPhase::OverheadCheck;
    if (state.phase == LaunchPhase::OverheadCheck
        && state.stableOverheadS > 0.65)
        state.phase = state.reverseLaunch
            ? LaunchPhase::Turning : LaunchPhase::Running;

    if (state.phase == LaunchPhase::Turning)
    {
        if (std::abs(input.weightShift) < 0.08)
            state.turnInputArmed = true;
        if (state.turnDirection == 0
            && state.turnInputArmed
            && std::abs(input.weightShift) >= 0.15
            && centered
            && symmetricBrake < 0.58)
            state.turnDirection = input.weightShift > 0.0 ? 1 : -1;
        if (state.turnDirection != 0)
        {
            const double turnRate = centered
                ? (1.0 / 0.95) : (1.0 / 1.7);
            state.turnProgress = Clamp(
                state.turnProgress + dt * turnRate, 0.0, 1.0);
            const double pi = 3.14159265358979323846;
            state.pilotFacingYawOffsetRad =
                pi + state.turnDirection * state.turnProgress * pi;
            if (state.turnProgress >= 1.0)
            {
                state.pilotFacingYawOffsetRad = 0.0;
                state.phase = LaunchPhase::Running;
            }
        }
    }

    const bool brakeAbort =
        symmetricBrake > 0.78 && state.phase != LaunchPhase::Running;
    const bool lostWing =
        std::abs(state.canopyHeadingErrorRad) > 1.12
        && state.inflation > 0.55;
    const bool releasedEarly =
        !input.launchHeld && state.phase != LaunchPhase::Running;
    if (brakeAbort || lostWing || releasedEarly)
    {
        state.phase = LaunchPhase::Aborted;
        output.abort = true;
    }

    const double takeoffSpeed = 8.0
        + 0.012 * std::max(0.0, input.allUpMassKg - 95.0)
        - 0.035 * std::max(0.0, input.wingAreaM2 - 24.0);
    if (state.phase == LaunchPhase::Running
        && state.stableOverheadS > 0.9
        && output.apparentWindMps >= takeoffSpeed
        && state.liftFraction > 0.72
        && symmetricBrake < 0.62)
    {
        state.phase = LaunchPhase::Airborne;
        output.liftOff = true;
    }

    output.pilotVelocityWorldMps =
        forward * state.pilotRunSpeedMps;
    output.canopyPressure = Clamp(
        state.inflation * (0.55 + 0.45 * inflationAuthority), 0.0, 1.0);
    output.pilotFacingYawOffsetRad = state.pilotFacingYawOffsetRad;
    output.brakeSidesCrossed =
        state.reverseLaunch && state.phase != LaunchPhase::Running
        && state.phase != LaunchPhase::Airborne;
    return output;
}

const char* LaunchPhaseName(LaunchPhase phase)
{
    switch (phase)
    {
        case LaunchPhase::LaidOut: return "WING LAID OUT";
        case LaunchPhase::Inflation: return "INFLATING";
        case LaunchPhase::OverheadCheck: return "CHECK WING";
        case LaunchPhase::Turning: return "A/D: TURN UNDER WING";
        case LaunchPhase::Running: return "COMMIT - KEEP RUNNING";
        case LaunchPhase::Airborne: return "AIRBORNE";
        case LaunchPhase::Aborted: return "LAUNCH ABORTED";
    }
    return "WING LAID OUT";
}
}
