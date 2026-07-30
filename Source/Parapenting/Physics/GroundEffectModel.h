#pragma once

namespace Parapenting::Physics
{
struct GroundEffectInput
{
    double pilotGroundClearanceM = 1000.0;
    double wingSpanM = 11.5;
    double airspeedMps = 10.0;
    double verticalSpeedMps = -1.2;
    double symmetricBrake = 0.0;
    double brakeApplicationRatePerS = 0.0;
    double canopyPressure = 1.0;
    double collapseFraction = 0.0;
    double previousFlareEnergy = 0.8;
    double previousFlareLift = 0.0;
    double deltaSeconds = 1.0 / 120.0;
};

struct GroundEffectOutput
{
    double proximity = 0.0;
    double inducedDragReduction = 0.0;
    double flareEnergy = 0.0;
    double flareLiftState = 0.0;
    double flareLiftCoefficient = 0.0;
    double flareAuthority = 0.0;
};

GroundEffectOutput EvaluateGroundEffect(const GroundEffectInput& input);
}
