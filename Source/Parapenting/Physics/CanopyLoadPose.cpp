#include "CanopyLoadPose.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
CanopyLoadPose EvaluateCanopyLoadPose(
    double highLoadDeformation, double loadFactor)
{
    const double deformation = std::clamp(
        highLoadDeformation, 0.0, 1.0);
    const double excessLoad = std::clamp(
        (loadFactor - 3.0) / 2.0, 0.0, 1.0);
    const double response = deformation * (0.72 + 0.28 * excessLoad);
    return {
        1.0 - 0.035 * response,
        1.0 + 0.018 * response,
        1.0 - 0.12 * response,
        30.0 * response,
        9.0 * response,
        5.5 * response
    };
}

CanopySwingOffset EvaluateCanopySwingOffset(
    double swingRad, double suspensionLengthM)
{
    // The canopy rides on an arc of radius `suspensionLengthM` centred on the
    // pilot. Swung aft by q it sits at (-L sin q, L cos q), so it moves
    // backward and DOWN - the drop is what makes a surge look like the wing
    // diving forward and rising rather than sliding along a shelf.
    const double length = std::max(0.0, suspensionLengthM);
    CanopySwingOffset offset;
    offset.forwardM = -length * std::sin(swingRad);
    offset.riseM = length * (std::cos(swingRad) - 1.0);
    return offset;
}
}