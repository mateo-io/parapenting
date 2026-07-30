#include "CanopyLoadPose.h"

#include <algorithm>

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
}
