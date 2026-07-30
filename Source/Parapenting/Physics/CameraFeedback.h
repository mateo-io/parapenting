#pragma once

#include "AccessibilityProfile.h"
#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
struct CameraFeedback
{
    Vec3 positionOffsetCm{};
    double pitchDegrees = 0.0;
    double yawDegrees = 0.0;
    double rollDegrees = 0.0;
    double fieldOfViewDeltaDegrees = 0.0;
};

// Deterministic sensory camera response. Unreal supplies filtered body
// acceleration, but all flight-state coupling lives here for headless tests.
CameraFeedback EvaluateCameraFeedback(
    const Telemetry& telemetry,
    const Vec3& filteredBodyAccelerationMps2,
    double simulationTimeSeconds,
    const AccessibilityProfile& accessibility);
}
