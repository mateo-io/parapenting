#include "SuspensionSystem.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
const SuspensionGeometry& Epic2MlSuspensionGeometry()
{
    static const SuspensionGeometry Geometry{
        7.3,
        254.0,
        0.00125,
        1.05,
        {0.5, 0.5, 0.5, 0.5},
        {0.38, 0.38, 0.42, 0.5},
        {{
            {SuspensionGroup::A, -0.86, 0.12},
            {SuspensionGroup::A, -0.58, 0.12},
            {SuspensionGroup::A, -0.28, 0.12},
            {SuspensionGroup::BabyA, -0.96, 0.16},
            {SuspensionGroup::B, -0.82, 0.36},
            {SuspensionGroup::B, -0.52, 0.36},
            {SuspensionGroup::B, -0.22, 0.36},
            {SuspensionGroup::C, -0.76, 0.62},
            {SuspensionGroup::C, -0.42, 0.62},
            {SuspensionGroup::C, -0.15, 0.78},
            {SuspensionGroup::A, 0.86, 0.12},
            {SuspensionGroup::A, 0.58, 0.12},
            {SuspensionGroup::A, 0.28, 0.12},
            {SuspensionGroup::BabyA, 0.96, 0.16},
            {SuspensionGroup::B, 0.82, 0.36},
            {SuspensionGroup::B, 0.52, 0.36},
            {SuspensionGroup::B, 0.22, 0.36},
            {SuspensionGroup::C, 0.76, 0.62},
            {SuspensionGroup::C, 0.42, 0.62},
            {SuspensionGroup::C, 0.15, 0.78}
        }}
    };
    return Geometry;
}

SuspensionLoads EvaluateSuspensionLoads(
    const SuspensionGeometry& geometry, const SuspensionInput& input)
{
    const double accelerator = std::clamp(input.accelerator, 0.0, 1.0);
    const double brake = std::clamp(input.symmetricBrake, 0.0, 1.0);
    const double collapse = std::clamp(
        0.25 * (input.leftCollapse + input.rightCollapse)
            + 0.5 * input.frontalCollapse,
        0.0, 0.9);

    // The forward risers shorten by 120/120/80/0 mm on full bar. Braking
    // transfers trailing-edge load into the brake fan and rear C/D cascade.
    double a = 0.36 + 0.105 * accelerator - 0.055 * brake;
    double b = 0.31 + 0.025 * accelerator - 0.025 * brake;
    double c = 0.33 - 0.130 * accelerator + 0.030 * brake;
    double brakeShare = 0.16 * brake * brake;
    c = std::max(0.08, c - brakeShare * 0.45);
    a *= 1.0 - 0.28 * collapse;
    b *= 1.0 - 0.16 * collapse;
    c *= 1.0 - 0.08 * collapse;
    const double sum = std::max(0.01, a + b + c + brakeShare);

    SuspensionLoads result;
    result.aFraction = a / sum;
    result.bFraction = b / sum;
    result.cFraction = c / sum;
    result.brakeFraction = brakeShare / sum;
    result.totalTensionN = std::max(0.0, input.totalAerodynamicLoadN);
    const double exposedLineArea =
        geometry.totalLineLengthM * geometry.lineDiameterM;
    // Total manufactured line length is not all normal to the airflow:
    // cascades overlap, upper galleries are inclined, and lower lines shield
    // one another. Use projected exposure rather than treating all 254 m as a
    // flat drag fence.
    constexpr double ProjectedExposureFraction = 0.35;
    result.lineDragN = std::max(0.0, input.dynamicPressurePa)
        * exposedLineArea * ProjectedExposureFraction
        * geometry.lineDragCoefficient;

    // Resultant attachment position relative to a neutral 38%-chord hang
    // point. Positive is a nose-up moment. The lever uses mean aerodynamic
    // chord derived from area and the EPIC 2 flat aspect ratio.
    const double meanChordM = std::sqrt(
        std::max(1.0, input.wingAreaM2) / 5.2);
    const double resultantChord =
        0.12 * result.aFraction
        + 0.36 * result.bFraction
        + 0.68 * result.cFraction
        + 0.92 * result.brakeFraction;
    result.pitchMomentNm = (resultantChord - 0.38)
        * meanChordM * result.totalTensionN * 0.085;

    const double pressure = std::clamp(input.canopyPressure, 0.0, 1.08);
    const double shift = std::clamp(input.weightShift, -1.0, 1.0);
    const double aeroAsymmetry =
        std::clamp(input.spanwiseLoadAsymmetry, -0.85, 0.85);
    // Moving the payload changes carabiner reaction before the canopy has
    // time to settle. Aerodynamic asymmetry then adds to or opposes it.
    const double lateralImbalance = std::clamp(
        0.38 * shift + 0.48 * aeroAsymmetry, -0.72, 0.72);
    result.lateralLoadImbalance = lateralImbalance;
    result.left.carabinerLoadFraction = 0.5 * (1.0 - lateralImbalance);
    result.right.carabinerLoadFraction = 0.5 * (1.0 + lateralImbalance);

    const auto fillSide = [&](SuspensionSideLoads& side, bool leftSide)
    {
        const double sideCollapse = std::clamp(
            leftSide ? input.leftCollapse : input.rightCollapse, 0.0, 0.95);
        const double sideBrake = std::clamp(
            leftSide ? input.leftBrake : input.rightBrake, 0.0, 1.0);
        const double sideLoad = result.totalTensionN
            * side.carabinerLoadFraction * pressure;
        std::array<double, 4> shares{
            result.aFraction,
            result.bFraction,
            result.cFraction,
            std::max(result.brakeFraction,
                0.18 * sideBrake * sideBrake)
        };
        shares[0] *= 1.0 - 0.72 * sideCollapse;
        shares[1] *= 1.0 - 0.48 * sideCollapse;
        shares[2] *= 1.0 - 0.24 * sideCollapse;
        const double sideSum = std::max(
            0.01, shares[0] + shares[1] + shares[2] + shares[3]);
        side.totalTensionN = 0.0;
        for (std::size_t group = 0; group < shares.size(); ++group)
        {
            side.loadFraction[group] = shares[group] / sideSum;
            side.tensionN[group] =
                sideLoad * side.loadFraction[group];
            // Lines are effectively slack below roughly 12 N, with a smooth
            // transition so visuals and force transmission do not chatter.
            side.slackFraction[group] = std::clamp(
                (14.0 - side.tensionN[group]) / 12.0, 0.0, 1.0);
            side.totalTensionN += side.tensionN[group]
                * (1.0 - side.slackFraction[group]);
        }
    };
    fillSide(result.left, true);
    fillSide(result.right, false);
    const double lateralTensionDifference =
        result.right.totalTensionN - result.left.totalTensionN;
    result.rollMomentNm = -lateralTensionDifference * 0.42;
    result.yawMomentNm = lateralTensionDifference * 0.055;
    return result;
}
}
