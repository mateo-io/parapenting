#pragma once

#include <array>

namespace Parapenting::Physics
{
enum class SuspensionGroup
{
    A,
    BabyA,
    B,
    C,
    Brake
};

struct SuspensionAttachment
{
    SuspensionGroup group;
    double spanFraction;
    double chordFraction;
};

struct SuspensionGeometry
{
    double canopyToRiserM = 7.3;
    double totalLineLengthM = 254.0;
    double lineDiameterM = 0.00125;
    double lineDragCoefficient = 1.05;
    // A, Baby-A, B, C. The upper D row cascades into the C riser.
    std::array<double, 4> trimRiserLengthM{0.5, 0.5, 0.5, 0.5};
    std::array<double, 4> acceleratedRiserLengthM{0.38, 0.38, 0.42, 0.5};
    std::array<SuspensionAttachment, 20> attachments{};
};

struct SuspensionInput
{
    double dynamicPressurePa = 0.0;
    double wingAreaM2 = 0.0;
    double totalAerodynamicLoadN = 0.0;
    double accelerator = 0.0;
    double symmetricBrake = 0.0;
    double leftCollapse = 0.0;
    double rightCollapse = 0.0;
    double frontalCollapse = 0.0;
    double leftBrake = 0.0;
    double rightBrake = 0.0;
    double weightShift = 0.0;
    double canopyPressure = 1.0;
    // Positive means the right half carries more aerodynamic load.
    double spanwiseLoadAsymmetry = 0.0;
};

struct SuspensionSideLoads
{
    // A/A', B, C (including the upper D cascade), brake.
    std::array<double, 4> tensionN{};
    std::array<double, 4> loadFraction{};
    std::array<double, 4> slackFraction{};
    double totalTensionN = 0.0;
    double carabinerLoadFraction = 0.5;
};

struct SuspensionLoads
{
    double aFraction = 0.0;
    double bFraction = 0.0;
    double cFraction = 0.0;
    double brakeFraction = 0.0;
    double totalTensionN = 0.0;
    double lineDragN = 0.0;
    double pitchMomentNm = 0.0;
    double rollMomentNm = 0.0;
    double yawMomentNm = 0.0;
    SuspensionSideLoads left;
    SuspensionSideLoads right;
    double lateralLoadImbalance = 0.0;
};

// Published EPIC 2 ML dimensions plus attachment positions digitized from the
// manufacturer's line plan. This is a research geometry, not manufacturer
// validation or an airworthiness representation.
const SuspensionGeometry& Epic2MlSuspensionGeometry();

SuspensionLoads EvaluateSuspensionLoads(
    const SuspensionGeometry& geometry, const SuspensionInput& input);
}
