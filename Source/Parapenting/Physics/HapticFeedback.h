#pragma once

#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
struct HapticOutput
{
    double left = 0.0;
    double right = 0.0;
};

// Engine-independent tactile model so controller feel can be regression-tested
// alongside the aerodynamics. Outputs are normalized motor amplitudes.
class HapticFeedbackModel
{
public:
    HapticOutput Evaluate(const Telemetry& telemetry, double timeSeconds);
    void Reset();

private:
    double previousLeftCollapse = 0.0;
    double previousRightCollapse = 0.0;
    double leftImpulse = 0.0;
    double rightImpulse = 0.0;
    double previousTimeSeconds = -1.0;
};
}
