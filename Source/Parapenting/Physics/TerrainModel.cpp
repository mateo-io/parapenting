#include "TerrainModel.h"
#include "HeightfieldGrid.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
HeightfieldGrid SurveyedTerrain;
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
    return SurveyedTerrain.LoadEsriAscii(filePath);
}

void TerrainModel::ClearHeightfield()
{
    SurveyedTerrain.Clear();
}

bool TerrainModel::HasHeightfield()
{
    return SurveyedTerrain.IsLoaded();
}

double TerrainModel::HeightM(double x, double y)
{
    double surveyedElevation = 0.0;
    if (SurveyedTerrain.Sample(x, y, surveyedElevation))
        return surveyedElevation;
    // Grindelwald is held in a separate sparse regional lane so its verified
    // route geometry can coexist with the detailed Interlaken heightfield
    // without stretching that mesh over the 20 km geographic separation.
    if (y > 5000.0)
    {
        constexpr double RouteDxM = 4021.2845;
        constexpr double RouteDyM = -2199.4248;
        constexpr double FirstToGrundM = 4583.47;
        constexpr double ForwardX = RouteDxM / FirstToGrundM;
        constexpr double ForwardY = RouteDyM / FirstToGrundM;
        constexpr double LeftX = -ForwardY;
        constexpr double LeftY = ForwardX;
        const double dx = x;
        const double dy = y - 8500.0;
        const double along = dx * ForwardX + dy * ForwardY;
        const double cross = dx * LeftX + dy * LeftY;
        const double progress = SmoothStep(0.0, FirstToGrundM, along);
        const double valleyFloor = 385.0 + 1173.0 * (1.0 - progress);
        const double crossValley = std::abs(cross);
        const double wall = std::pow(
            std::max(0.0, crossValley - 520.0) / 850.0, 1.45) * 520.0;
        const double bodmiCorrection =
            -52.0 * Gaussian(along, 3264.0, 360.0)
            * Gaussian(cross, 573.0, 300.0);
        const double northRidge =
            180.0 * Gaussian(along, 1850.0, 720.0)
            * Gaussian(cross, 1000.0, 420.0);
        return std::max(360.0,
            valleyFloor + wall + bodmiCorrection + northRidge);
    }
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
    const double westRidge = 150.0 * Gaussian(x, 820.0, 390.0)
                           * Gaussian(y, -720.0, 330.0);
    const double eastRidge = 110.0 * Gaussian(x, 1180.0, 520.0)
                           * Gaussian(y, 880.0, 420.0);
    const double field = -8.0 * Gaussian(x, 2409.9, 380.0)
                        * Gaussian(y, 42.0, 480.0);
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
