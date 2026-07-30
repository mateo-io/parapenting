#pragma once

#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
struct SpanwiseAeroResult
{
    double liftCoefficient = 0.0;
    double dragCoefficient = 0.0;
    double rollMomentNm = 0.0;
    double yawMomentNm = 0.0;
    double loadAsymmetry = 0.0;
    double leftStalledFraction = 0.0;
    double rightStalledFraction = 0.0;
};

// Quasi-steady strip model. It is deliberately engine-independent so the same
// canopy response is exercised by headless regression tests and Unreal.
SpanwiseAeroResult EvaluateSpanwiseCanopy(
    const WingParameters& parameters,
    const FlightState& state,
    const ControlInput& controls,
    double baseLiftCoefficient,
    double flareBoost,
    double dynamicPressure,
    double airspeedMps,
    double globalAngleOfAttackRad,
    bool stalled,
    double inducedDragReduction,
    const Atmosphere& atmosphere);
}
