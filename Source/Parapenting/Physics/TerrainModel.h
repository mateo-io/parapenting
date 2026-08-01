#pragma once

#include "ParagliderDynamics.h"
#include <string>

namespace Parapenting::Physics
{
// Lightweight analytic terrain used by the v0 renderer, collision and airflow.
// Coordinates are metres: +X follows the Amisbuehl-to-Lehn route, +Y is
// route-right (west for the southbound primary route), and Z is height above
// the Lehn landing field. See ParagliderCoordinateSystem.h.

// Where a given height sample came from. The surveyed grids cover only part
// of what the renderer draws, and the transition is a step rather than a
// blend, so anything that cares whether it is standing on real ground has to
// be able to ask.
enum class TerrainProvenance
{
    // Bilinear sample of a swissALTI3D-derived heightfield.
    Surveyed,
    // The analytic Interlaken proxy, for anywhere no surveyed region covers.
    // It is a shape, not a place: it says nothing about real geography and
    // nothing terrain-dependent should be validated on it.
    Analytic,
};

class TerrainModel
{
public:
    // Additive. Regions are separate grids in one shared route frame -
    // Interlaken and Grindelwald are 20 km apart and the ground between them
    // is not flown, so stretching one grid across both would carry 250 km2 of
    // terrain nobody sees. Load order does not matter; the regions do not
    // overlap, and the first one covering a sample answers for it.
    static bool LoadHeightfieldAscii(const std::string& filePath);
    static void ClearHeightfield();
    static bool HasHeightfield();
    static int LoadedRegionCount();
    static double HeightM(double x, double y);
    // Which source HeightM would use at this position.
    static TerrainProvenance ProvenanceAt(double x, double y);
    static bool IsSurveyed(double x, double y);
    static Vec3 Normal(double x, double y);
    static double RidgeExposure(double x, double y);
    static double LeeRotorPotential(double x, double y, const Vec3& windWorldMps);
};
}
