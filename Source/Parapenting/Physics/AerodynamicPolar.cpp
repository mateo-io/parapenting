#include "AerodynamicPolar.h"
#include "ParagliderDynamics.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
PolarSample SampleBrakePolar(
    const std::array<double, 5>& liftCurve,
    const std::array<double, 5>& dragCurve,
    double effectiveBrake)
{
    const double scaled = std::clamp(effectiveBrake, 0.0, 1.0) * 4.0;
    const std::size_t lower = std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(scaled)), 3);
    const std::size_t upper = lower + 1;
    const double blend = scaled - static_cast<double>(lower);
    return {
        liftCurve[lower] * (1.0 - blend) + liftCurve[upper] * blend,
        dragCurve[lower] * (1.0 - blend) + dragCurve[upper] * blend
    };
}

bool IsValidBrakePolar(
    const std::array<double, 5>& liftCurve,
    const std::array<double, 5>& dragCurve)
{
    if (std::abs(liftCurve.front()) > 1e-9
        || std::abs(dragCurve.front()) > 1e-9)
        return false;
    for (std::size_t index = 0; index < liftCurve.size(); ++index)
    {
        if (!std::isfinite(liftCurve[index])
            || !std::isfinite(dragCurve[index])
            || dragCurve[index] < 0.0)
            return false;
        if (index > 0
            && (liftCurve[index] < liftCurve[index - 1]
                || dragCurve[index] < dragCurve[index - 1]))
            return false;
    }
    return dragCurve.back() > dragCurve[2];
}

SteadyPolarPoint EstimateSteadyPolarPoint(
    const WingParameters& p, double effectiveBrake)
{
    const PolarSample brake = SampleBrakePolar(
        p.brakeLiftCurve, p.brakeDragCurve, effectiveBrake);
    const double cl = std::clamp(
        p.trimCl + brake.liftDelta, 0.05, p.maxLiftCoefficient);
    const double cd = std::max(
        0.018, p.zeroLiftDrag + p.inducedDragFactor * cl * cl
            + brake.dragDelta);
    const double resultantCoefficient = std::hypot(cl, cd);
    const double weightN = p.allUpMassKg * 9.80665;
    const double airspeed = std::sqrt(
        2.0 * weightN
        / std::max(0.01,
            p.airDensityKgM3 * p.areaM2 * resultantCoefficient));
    return {
        std::clamp(effectiveBrake, 0.0, 1.0),
        airspeed,
        airspeed * cd / resultantCoefficient,
        cl / cd,
        cl,
        cd
    };
}

} // namespace Parapenting::Physics
