#include "TerrainModel.h"
#include "HeightfieldGrid.h"

#include <algorithm>
#include <vector>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
std::vector<HeightfieldGrid> SurveyedRegions;

// True if any loaded region covers this position, and if so its elevation.
bool SampleSurveyed(double x, double y, double& elevationM)
{
    for (const HeightfieldGrid& region : SurveyedRegions)
        if (region.Sample(x, y, elevationM)) return true;
    return false;
}

double SmoothStep(double edge0, double edge1, double x)
{
    const double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double Gaussian(double x, double centre, double width)
{
    const double q = (x - centre) / width;
    return std::exp(-0.5 * q * q);
}

}

bool TerrainModel::LoadHeightfieldAscii(const std::string& filePath)
{
    HeightfieldGrid region;
    if (!region.LoadEsriAscii(filePath)) return false;
    SurveyedRegions.push_back(std::move(region));
    return true;
}

void TerrainModel::ClearHeightfield()
{
    SurveyedRegions.clear();
}

bool TerrainModel::HasHeightfield()
{
    return !SurveyedRegions.empty();
}

int TerrainModel::LoadedRegionCount()
{
    return static_cast<int>(SurveyedRegions.size());
}

TerrainProvenance TerrainModel::ProvenanceAt(double x, double y)
{
    double ignored = 0.0;
    return SampleSurveyed(x, y, ignored)
        ? TerrainProvenance::Surveyed
        : TerrainProvenance::Analytic;
}

bool TerrainModel::IsSurveyed(double x, double y)
{
    return ProvenanceAt(x, y) == TerrainProvenance::Surveyed;
}

double TerrainModel::HeightM(double x, double y)
{
    double surveyedElevation = 0.0;
    if (SampleSurveyed(x, y, surveyedElevation)) return surveyedElevation;
    // Grindelwald used to be a hand-shaped analytic lane here, sitting at an
    // invented y = -8500 because there was no surveyed ground 20 km out. It is
    // its own swissALTI3D region now, at the sites' true projected positions,
    // and the lane is gone with it. What remains is one analytic proxy for
    // anywhere off the surveyed regions - a shape, not a place.
    //
    // The launch shoulder drops into the Interlaken valley and broadens into
    // the Lehn landing basin. Side walls evoke the Harder/Kulm foothills while
    // leaving the route corridor readable and safely flyable.
    const double routeDescent = 690.0 * (1.0 - SmoothStep(-120.0, 2250.0, x));
    const double launchShoulder = 25.0 * Gaussian(x, 90.0, 300.0)
                               * Gaussian(y, 0.0, 520.0);
    const double valleyOpening = SmoothStep(450.0, 1900.0, x);
    const double sideDistance = std::abs(y);
    const double valleyWall = valleyOpening
        * std::pow(std::max(0.0, sideDistance - 520.0) / 900.0, 1.55) * 430.0;
    // Y centres negated with the frame flip to route-right +Y: west is now
    // positive, east negative, so the named ridges stay on their real sides.
    const double westRidge = 150.0 * Gaussian(x, 820.0, 390.0)
                           * Gaussian(y, 720.0, 330.0);
    const double eastRidge = 110.0 * Gaussian(x, 1180.0, 520.0)
                           * Gaussian(y, -880.0, 420.0);
    const double field = -8.0 * Gaussian(x, 2409.9, 380.0)
                        * Gaussian(y, -42.0, 480.0);
    return std::max(-8.0, routeDescent + launchShoulder + valleyWall
                          + westRidge + eastRidge + field);
}

Vec3 TerrainModel::Normal(double x, double y)
{
    constexpr double h = 5.0;
    const double dx = (HeightM(x + h, y) - HeightM(x - h, y)) / (2.0 * h);
    const double dy = (HeightM(x, y + h) - HeightM(x, y - h)) / (2.0 * h);
    const double length = std::sqrt(dx * dx + dy * dy + 1.0);
    return {-dx / length, -dy / length, 1.0 / length};
}

double TerrainModel::RidgeExposure(double x, double y)
{
    const Vec3 n = Normal(x, y);
    return std::clamp(std::sqrt(n.x * n.x + n.y * n.y) * 2.4, 0.0, 1.0);
}

double TerrainModel::LeeRotorPotential(double x, double y, const Vec3& wind)
{
    const Vec3 n = Normal(x, y);
    const double horizontalSpeed = std::max(0.1, std::sqrt(
        wind.x * wind.x + wind.y * wind.y));
    // Positive dot means wind is driving into the terrain. Rotor is displaced
    // downwind of the most exposed ridges instead of appearing everywhere.
    const double facing = std::max(0.0,
        -(wind.x * n.x + wind.y * n.y) / horizontalSpeed);
    const double ridge = RidgeExposure(x - wind.x * 55.0,
                                       y - wind.y * 55.0);
    return std::clamp(facing * ridge * 2.5, 0.0, 1.0);
}
}
