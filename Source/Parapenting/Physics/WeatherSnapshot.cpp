#include "WeatherSnapshot.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
Vec3 WindVectorFromMeteorological(double fromDegrees, double speedMps)
{
    constexpr double DegToRad = 3.14159265358979323846 / 180.0;
    // Surveyed primary-frame bearing: Amisbuehl launch to Lehn landing.
    constexpr double ForwardBearingDeg = 177.273;
const double ForwardEast = std::sin(ForwardBearingDeg * DegToRad);
const double ForwardNorth = std::cos(ForwardBearingDeg * DegToRad);
const double LeftEast = -ForwardNorth;
const double LeftNorth = ForwardEast;
    const double radians = fromDegrees * DegToRad;
    const double speed = std::max(0.0, speedMps);
    const double east = -std::sin(radians) * speed;
    const double north = -std::cos(radians) * speed;
    return {
        east * ForwardEast + north * ForwardNorth,
        east * LeftEast + north * LeftNorth,
        0.0
    };
}

double MeteorologicalDirectionFromWindVector(const Vec3& windWorldMps)
{
    constexpr double RadToDeg = 180.0 / 3.14159265358979323846;
    constexpr double DegToRad = 3.14159265358979323846 / 180.0;
    constexpr double ForwardBearingDeg = 177.273;
    const double forwardEast = std::sin(ForwardBearingDeg * DegToRad);
    const double forwardNorth = std::cos(ForwardBearingDeg * DegToRad);
    const double leftEast = -forwardNorth;
    const double leftNorth = forwardEast;
    const double east =
        windWorldMps.x * forwardEast + windWorldMps.y * leftEast;
    const double north =
        windWorldMps.x * forwardNorth + windWorldMps.y * leftNorth;
    double fromDegrees = std::atan2(-east, -north) * RadToDeg;
    if (fromDegrees < 0.0) fromDegrees += 360.0;
    return fromDegrees;
}
}
