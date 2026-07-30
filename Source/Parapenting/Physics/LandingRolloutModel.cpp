#include "LandingRolloutModel.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
void LandingRolloutModel::Begin(
    LandingRolloutState& state,
    const Vec3& touchdownVelocityWorldMps,
    double canopyPressure)
{
    state = {};
    state.velocityWorldMps = {
        touchdownVelocityWorldMps.x,
        touchdownVelocityWorldMps.y,
        0.0
    };
    state.canopyPressure = std::clamp(canopyPressure, 0.0, 1.0);
    const double horizontalSpeed = Length(state.velocityWorldMps);
    state.hardImpact =
        touchdownVelocityWorldMps.z < -4.0 || horizontalSpeed > 12.0;
    if (state.hardImpact)
        state.phase = LandingRolloutPhase::Fallen;
    else if (horizontalSpeed < 0.65)
        state.phase = LandingRolloutPhase::Settled;
    else
        state.phase = LandingRolloutPhase::Running;
}

LandingRolloutOutput LandingRolloutModel::Step(
    LandingRolloutState& state,
    const LandingRolloutInput& raw,
    double deltaSeconds) const
{
    const double dt = std::clamp(deltaSeconds, 0.0, 1.0 / 30.0);
    if (dt <= 0.0) return {};
    state.elapsedSeconds += dt;

    const double leftBrake = std::clamp(raw.leftBrake, 0.0, 1.0);
    const double rightBrake = std::clamp(raw.rightBrake, 0.0, 1.0);
    const double symmetricBrake = 0.5 * (leftBrake + rightBrake);
    const double brakeAsymmetry = std::abs(rightBrake - leftBrake);
    const double oldSpeed = Length(state.velocityWorldMps);
    Vec3 direction = oldSpeed > 0.01
        ? state.velocityWorldMps / oldSpeed
        : Vec3{};

    if (state.phase == LandingRolloutPhase::Running)
    {
        const double tailwind =
            Dot(raw.surfaceWindWorldMps, direction);
        const double deceleration = std::clamp(
            1.35 + 1.65 * symmetricBrake
                + 0.45 * (1.0 - state.canopyPressure)
                - 0.10 * tailwind,
            0.65, 4.2);
        const double newSpeed = std::max(0.0, oldSpeed - deceleration * dt);
        state.velocityWorldMps = direction * newSpeed;
        state.runoutDistanceM += 0.5 * (oldSpeed + newSpeed) * dt;
        state.canopyPressure = std::max(
            0.0, state.canopyPressure
                - (0.045 + 0.20 * std::max(0.0, symmetricBrake - 0.55)
                    + 0.16 * brakeAsymmetry) * dt);

        // A fast pilot yanking one side after contact can be pulled off-balance.
        if (newSpeed > 7.5 && brakeAsymmetry > 0.58)
        {
            state.phase = LandingRolloutPhase::Fallen;
            state.hardImpact = true;
        }
        else if (newSpeed < 0.65)
        {
            state.phase = LandingRolloutPhase::Settled;
            state.velocityWorldMps = {};
        }
    }
    else if (state.phase == LandingRolloutPhase::Fallen)
    {
        const double newSpeed = std::max(0.0, oldSpeed - 5.8 * dt);
        state.velocityWorldMps = direction * newSpeed;
        state.runoutDistanceM += 0.5 * (oldSpeed + newSpeed) * dt;
        state.canopyPressure =
            std::max(0.0, state.canopyPressure - 0.82 * dt);
        if (state.elapsedSeconds > 1.35 && newSpeed < 0.4)
        {
            state.phase = LandingRolloutPhase::Settled;
            state.velocityWorldMps = {};
        }
    }
    else
    {
        state.velocityWorldMps = {};
        state.canopyPressure =
            std::max(0.0, state.canopyPressure - 0.32 * dt);
    }

    const double newSpeed = Length(state.velocityWorldMps);
    const Vec3 displacement =
        direction * (0.5 * (oldSpeed + newSpeed) * dt);
    return {
        displacement,
        state.phase == LandingRolloutPhase::Fallen
            ? std::min(76.0, state.elapsedSeconds * 95.0)
            : 0.0,
        state.phase != LandingRolloutPhase::Settled
    };
}

const char* LandingRolloutPhaseName(LandingRolloutPhase phase)
{
    switch (phase)
    {
        case LandingRolloutPhase::Running: return "RUNNING OUT";
        case LandingRolloutPhase::Fallen: return "FALLEN";
        case LandingRolloutPhase::Settled: return "SETTLED";
    }
    return "SETTLED";
}
}
