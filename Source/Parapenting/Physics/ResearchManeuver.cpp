#include "ResearchManeuver.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
double Bank(const FlightState& state)
{
    return std::asin(std::clamp(
        state.attitude.Rotate({0.0, 1.0, 0.0}).z, -1.0, 1.0));
}

double Heading(const FlightState& state)
{
    return std::atan2(
        2.0 * (state.attitude.w * state.attitude.z
            + state.attitude.x * state.attitude.y),
        1.0 - 2.0 * (state.attitude.y * state.attitude.y
            + state.attitude.z * state.attitude.z));
}
}

ResearchManeuverResult RunResearchManeuver(
    ResearchManeuver maneuver, WingParameters parameters)
{
    constexpr double dt = 1.0 / 120.0;
    ParagliderDynamics dynamics(parameters);
    FlightState state;
    double duration = 6.0;
    if (maneuver == ResearchManeuver::BrakeZoom)
        state.velocityWorldMps = {16.0, 0.0, -1.0};
    if (maneuver == ResearchManeuver::SymmetricDeepStall)
        duration = 8.0;
    const double initialZ = state.positionWorldM.z;
    const double initialHeading = Heading(state);
    ResearchManeuverResult result;
    result.durationS = duration;
    for (int frame = 0; frame < static_cast<int>(duration / dt); ++frame)
    {
        ControlInput input;
        const double time = frame * dt;
        if (maneuver == ResearchManeuver::WeightShiftStep)
            input.weightShift = time < 4.0 ? 1.0 : 0.0;
        else if (maneuver == ResearchManeuver::BrakeZoom)
            input.leftBrake = input.rightBrake = time < 3.0 ? 0.78 : 0.0;
        else if (maneuver == ResearchManeuver::SymmetricDeepStall)
            input.leftBrake = input.rightBrake = time < 5.0 ? 1.0 : 0.0;
        else
        {
            input.leftBrake = time < 4.0 ? 1.0 : 0.0;
            input.rightBrake = time < 4.0 ? 0.18 : 0.0;
        }
        dynamics.Step(state, input, Atmosphere{}, dt);
        const auto& telemetry = dynamics.LastTelemetry();
        result.peakAbsBankRad = std::max(
            result.peakAbsBankRad, std::abs(Bank(state)));
        result.peakClimbMps = std::max(
            result.peakClimbMps, state.velocityWorldMps.z);
        result.altitudeGainM = std::max(
            result.altitudeGainM, state.positionWorldM.z - initialZ);
        result.minimumAirspeedMps = std::min(
            result.minimumAirspeedMps, telemetry.airspeedMps);
        result.peakLoadFactor = std::max(
            result.peakLoadFactor, telemetry.loadFactor);
        result.peakSeparatedSpan = std::max(
            result.peakSeparatedSpan,
            std::max(telemetry.leftSeparatedSpanState,
                telemetry.rightSeparatedSpanState));
        result.peakAbsEnergyResidualW = std::max(
            result.peakAbsEnergyResidualW,
            std::abs(telemetry.energyResidualW));
    }
    result.headingChangeRad = std::remainder(
        Heading(state) - initialHeading, 2.0 * 3.141592653589793);
    result.lateralDisplacementM = state.positionWorldM.y;
    return result;
}
}
