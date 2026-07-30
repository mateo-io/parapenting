#pragma once

#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
enum class LandingPhase
{
    Arrival,
    Downwind,
    Base,
    Final,
    Flare
};

struct LandingCircuit
{
    Vec3 target{};
    Vec3 downwindGate{};
    Vec3 baseGate{};
    Vec3 finalGate{};
    Vec3 finalDirection{1.0, 0.0, 0.0};
    Vec3 circuitSide{0.0, -1.0, 0.0};
    bool rightHand = true;
};

struct LandingGuidance
{
    LandingCircuit circuit{};
    LandingPhase phase = LandingPhase::Arrival;
    double lateralErrorM = 0.0;
    double headingAlignment = 0.0;
    double approachQuality = 0.0;
    bool stabilized = false;
};

LandingCircuit BuildLandingCircuit(
    const Vec3& target, const Vec3& surfaceWindMps, bool rightHand);
LandingGuidance EvaluateLandingApproach(
    const LandingCircuit& circuit, const Vec3& position,
    const Vec3& velocity, double groundClearanceM);
const char* LandingPhaseName(LandingPhase phase);
}
