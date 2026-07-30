#include "LandingCircuitModel.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
double Clamp01(double value)
{
    return std::max(0.0, std::min(value, 1.0));
}
}

LandingCircuit BuildLandingCircuit(
    const Vec3& target, const Vec3& surfaceWindMps, bool rightHand)
{
    Vec3 wind{surfaceWindMps.x, surfaceWindMps.y, 0.0};
    Vec3 finalDirection = Length(wind) > 0.8
        ? Normalized(-wind) : Vec3{1.0, 0.0, 0.0};
    const Vec3 right{finalDirection.y, -finalDirection.x, 0.0};
    const Vec3 side = rightHand ? right : -right;

    LandingCircuit circuit;
    circuit.target = target;
    circuit.finalDirection = finalDirection;
    circuit.circuitSide = side;
    circuit.rightHand = rightHand;
    circuit.finalGate = target - finalDirection * 280.0 + Vec3{0.0, 0.0, 65.0};
    circuit.baseGate = target - finalDirection * 360.0
        + side * 300.0 + Vec3{0.0, 0.0, 115.0};
    circuit.downwindGate = target + finalDirection * 430.0
        + side * 300.0 + Vec3{0.0, 0.0, 185.0};
    return circuit;
}

LandingGuidance EvaluateLandingApproach(
    const LandingCircuit& circuit, const Vec3& position,
    const Vec3& velocity, double groundClearanceM)
{
    LandingGuidance result;
    result.circuit = circuit;
    const Vec3 relative = position - circuit.target;
    const double along = Dot(relative, circuit.finalDirection);
    result.lateralErrorM = Dot(relative, circuit.circuitSide);

    const Vec3 horizontalVelocity{velocity.x, velocity.y, 0.0};
    result.headingAlignment = Length(horizontalVelocity) > 0.5
        ? Dot(Normalized(horizontalVelocity), circuit.finalDirection) : 0.0;

    if (groundClearanceM < 8.0)
        result.phase = LandingPhase::Flare;
    else if (groundClearanceM < 105.0 && along < 70.0
             && along > -650.0 && std::abs(result.lateralErrorM) < 175.0)
        result.phase = LandingPhase::Final;
    else if (groundClearanceM < 155.0 && along < -80.0)
        result.phase = LandingPhase::Base;
    else if (groundClearanceM < 230.0)
        result.phase = LandingPhase::Downwind;
    else
        result.phase = LandingPhase::Arrival;

    const double lateralQuality =
        1.0 - Clamp01(std::abs(result.lateralErrorM) / 150.0);
    const double headingQuality =
        Clamp01((result.headingAlignment - 0.45) / 0.5);
    const double sinkQuality =
        1.0 - Clamp01((std::abs(std::min(velocity.z, 0.0)) - 1.0) / 4.0);
    result.approachQuality =
        0.42 * lateralQuality + 0.38 * headingQuality + 0.20 * sinkQuality;
    result.stabilized = result.phase == LandingPhase::Final
        && std::abs(result.lateralErrorM) < 90.0
        && result.headingAlignment > 0.82
        && velocity.z > -4.0;
    return result;
}

const char* LandingPhaseName(LandingPhase phase)
{
    switch (phase)
    {
        case LandingPhase::Arrival: return "ARRIVAL";
        case LandingPhase::Downwind: return "DOWNWIND";
        case LandingPhase::Base: return "BASE";
        case LandingPhase::Final: return "FINAL";
        case LandingPhase::Flare: return "FLARE";
    }
    return "ARRIVAL";
}
}
