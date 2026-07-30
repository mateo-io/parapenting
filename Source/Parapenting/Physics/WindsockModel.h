#pragma once

#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
struct WindsockPose
{
    Vec3 directionWorld{1.0, 0.0, -1.0};
    double inflation = 0.0;
    double lengthScale = 0.35;
    double radiusScale = 0.18;
};

WindsockPose EvaluateWindsockPose(
    const Vec3& localWindWorldMps,
    double gustEnergyMps,
    double simulationTimeSeconds);
}
