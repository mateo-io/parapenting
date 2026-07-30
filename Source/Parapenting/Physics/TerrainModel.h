#pragma once

#include "ParagliderDynamics.h"
#include <string>

namespace Parapenting::Physics
{
// Lightweight analytic terrain used by the v0 renderer, collision and airflow.
// Coordinates are metres: +X follows the Amisbuehl-to-Lehn route, +Y is east,
// and Z is height above the Lehn landing field.
class TerrainModel
{
public:
    static bool LoadHeightfieldAscii(const std::string& filePath);
    static void ClearHeightfield();
    static bool HasHeightfield();
    static double HeightM(double x, double y);
    static Vec3 Normal(double x, double y);
    static double RidgeExposure(double x, double y);
    static double LeeRotorPotential(double x, double y, const Vec3& windWorldMps);
};
}
