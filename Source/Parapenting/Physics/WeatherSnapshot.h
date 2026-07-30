#pragma once

#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
struct WeatherSnapshot
{
    const char* source = "manual";
    const char* displayName = "Manual weather";
    double observedUnixSeconds = 0.0;
    double windFromDegrees = 0.0;
    double windSpeedMps = 0.0;
    double gustSpeedMps = 0.0;
    double thermalTopMslM = 0.0;
    unsigned int deterministicSeed = 1;
};

// Meteorological direction describes where the wind comes from. The vector is
// projected into the surveyed primary Amisbuehl-to-Lehn frame: world +X is
// route-forward (approximately south) and +Y is route-left.
Vec3 WindVectorFromMeteorological(double fromDegrees, double speedMps);
double MeteorologicalDirectionFromWindVector(const Vec3& windWorldMps);
}
