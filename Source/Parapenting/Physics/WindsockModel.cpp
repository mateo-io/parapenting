#include "WindsockModel.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
WindsockPose EvaluateWindsockPose(
    const Vec3& wind, double gustEnergyMps, double timeSeconds)
{
    const double speed = std::hypot(wind.x, wind.y);
    const double inflation = std::clamp(speed / 4.5, 0.0, 1.0);
    Vec3 horizontal = speed > 0.05
        ? Vec3{wind.x / speed, wind.y / speed, 0.0}
        : Vec3{1.0, 0.0, 0.0};
    const double flutter = std::clamp(gustEnergyMps / 4.0, 0.0, 1.0)
        * (0.035 * std::sin(timeSeconds * 6.7)
            + 0.018 * std::sin(timeSeconds * 14.3 + 0.8));
    const double cosine = std::cos(flutter);
    const double sine = std::sin(flutter);
    horizontal = {
        horizontal.x * cosine - horizontal.y * sine,
        horizontal.x * sine + horizontal.y * cosine,
        0.0
    };
    const double droop = 0.92 * (1.0 - inflation)
        + 0.08 * (1.0 - std::clamp(speed / 9.0, 0.0, 1.0));
    const Vec3 direction = Normalized({
        horizontal.x * (0.18 + 0.82 * inflation),
        horizontal.y * (0.18 + 0.82 * inflation),
        -droop
    });
    return {
        direction,
        inflation,
        0.35 + 0.65 * inflation,
        0.18 + 0.17 * inflation
    };
}
}
