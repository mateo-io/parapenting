#pragma once

#include "ParagliderDynamics.h"
#include <string>

namespace Parapenting::Physics
{
// Lightweight analytic terrain used by the v0 renderer, collision and airflow.
// Coordinates are metres: +X follows the Amisbuehl-to-Lehn route, +Y is east,
// and Z is height above the Lehn landing field.
// Where a given height sample came from. The surveyed grid covers only part
// of what the renderer draws, and the transition is a step rather than a
// blend, so anything that cares whether it is standing on real ground has to
// be able to ask.
enum class TerrainProvenance
{
    // Bilinear sample of the swissALTI3D-derived heightfield.
    Surveyed,
    // Analytic Interlaken proxy: outside the surveyed grid, below the
    // Grindelwald lane.
    AnalyticInterlaken,
    // Analytic Grindelwald lane. Geographically translated, not a real
    // position; never validate terrain-dependent behaviour here.
    AnalyticGrindelwald,
};

class TerrainModel
{
public:
    static bool LoadHeightfieldAscii(const std::string& filePath);
    static void ClearHeightfield();
    static bool HasHeightfield();
    static double HeightM(double x, double y);
    // Which source HeightM would use at this position.
    static TerrainProvenance ProvenanceAt(double x, double y);
    static bool IsSurveyed(double x, double y);
    static Vec3 Normal(double x, double y);
    static double RidgeExposure(double x, double y);
    static double LeeRotorPotential(double x, double y, const Vec3& windWorldMps);
};
}
