#pragma once

#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
enum class LandingRolloutPhase
{
    Running,
    Fallen,
    Settled
};

struct LandingRolloutState
{
    LandingRolloutPhase phase = LandingRolloutPhase::Settled;
    Vec3 velocityWorldMps{};
    double canopyPressure = 0.0;
    double elapsedSeconds = 0.0;
    double runoutDistanceM = 0.0;
    bool hardImpact = false;
};

struct LandingRolloutInput
{
    double leftBrake = 0.0;
    double rightBrake = 0.0;
    Vec3 surfaceWindWorldMps{};
};

struct LandingRolloutOutput
{
    Vec3 displacementWorldM{};
    double pilotFallRollDegrees = 0.0;
    bool moving = false;
};

class LandingRolloutModel
{
public:
    void Begin(
        LandingRolloutState& state,
        const Vec3& touchdownVelocityWorldMps,
        double canopyPressure);
    LandingRolloutOutput Step(
        LandingRolloutState& state,
        const LandingRolloutInput& input,
        double deltaSeconds) const;
};

const char* LandingRolloutPhaseName(LandingRolloutPhase phase);
}
