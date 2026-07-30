#pragma once

#include <array>

namespace Parapenting::Physics
{
struct WingParameters;

struct PolarSample
{
    double liftDelta = 0.0;
    double dragDelta = 0.0;
};

struct SteadyPolarPoint
{
    double effectiveBrake = 0.0;
    double airspeedMps = 0.0;
    double sinkRateMps = 0.0;
    double glideRatio = 0.0;
    double liftCoefficient = 0.0;
    double dragCoefficient = 0.0;
};

// Five normalized control points correspond to 0, 25, 50, 75 and 100 percent
// effective brake after free play. Values are coefficient deltas, not forces.
PolarSample SampleBrakePolar(
    const std::array<double, 5>& liftCurve,
    const std::array<double, 5>& dragCurve,
    double effectiveBrake);

bool IsValidBrakePolar(
    const std::array<double, 5>& liftCurve,
    const std::array<double, 5>& dragCurve);

SteadyPolarPoint EstimateSteadyPolarPoint(
    const WingParameters& parameters, double effectiveBrake);

} // namespace Parapenting::Physics
