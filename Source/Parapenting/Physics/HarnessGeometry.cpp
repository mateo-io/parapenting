#include "HarnessGeometry.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
const HarnessGeometry& DefaultHarnessGeometry()
{
    static const HarnessGeometry Harness{};
    return Harness;
}

double WeightShiftCgOffsetM(
    const HarnessGeometry& harness, double weightShift)
{
    const double input = std::clamp(weightShift, -1.0, 1.0);

    // A narrow chest strap pulls the risers together, which lets the harness
    // roll further for the same hip movement; a wide one spreads them and
    // holds the pilot square to the wing. Referenced to the carabiner
    // separation, so it is a shape ratio rather than an absolute.
    const double strapRatio = std::clamp(
        harness.chestStrapM / std::max(0.10, harness.carabinerSeparationM),
        0.6, 1.6);
    // 1.0 at a strap as wide as the carabiners, rising as it is narrowed.
    const double strapFactor = std::clamp(2.0 - strapRatio, 0.55, 1.6);

    // A seat plate carries the pilot's mass as one body with the seat, so hip
    // movement translates less of it. Plateless harnesses give the pilot
    // direct leverage, which is exactly why they feel more responsive.
    const double classFactor =
        harness.harnessClass == HarnessClass::NoPlate ? 1.35 : 1.0;

    return input * harness.hipTravelM * strapFactor * classFactor;
}

double PayloadRollFromCgOffsetRad(
    const HarnessGeometry& harness, double cgOffsetM)
{
    // The payload hangs from two carabiners. Moving its CG sideways by e means
    // it settles rolled by atan(e / h), where h is the CG's distance below the
    // carabiners - the same statics as any suspended mass, and the reason a
    // low-hung harness rolls further than a tightly strapped one.
    const double arm = std::max(0.05, harness.carabinerAboveCgM);
    return std::atan2(cgOffsetM, arm);
}
}
