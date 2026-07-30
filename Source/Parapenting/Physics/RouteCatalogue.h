#pragma once

#include "ParagliderDynamics.h"

#include <cstddef>

namespace Parapenting::Physics
{
enum class RouteProfileId
{
    AmisbuehlLehn,
    AmisbuehlHoehematte,
    BergboLehn,
    BergboHoehematte,
    HohwaldLehn,
    HohwaldHoehematte,
    NiederhornLehn,
    NiederhornHoehematte,
    GrindelwaldFirstGrund,
    GrindelwaldFirstBodmi
};

enum class SiteWindAssessment
{
    Suitable,
    MarginalDirection,
    TooStrong
};

struct GeoPoint
{
    const char* id;
    const char* displayName;
    double latitudeDeg;
    double longitudeDeg;
    double elevationM;
};

struct RouteProfile
{
    RouteProfileId id;
    const char* displayName;
    GeoPoint launch;
    GeoPoint landing;
    bool advancedLanding;
    double launchFacingDegrees;
    double preferredWindHalfWidthDegrees;
    double simulatorMaxLaunchWindMps;
    const char* launchHazard;
    const char* landingCircuit;
    const char* sourceLabel;
};

const RouteProfile& GetRouteProfile(RouteProfileId id);
const RouteProfile& GetRouteProfileByIndex(std::size_t index);
std::size_t RouteProfileCount();

// Local flight coordinates are ENU-like, but aligned to the route for stable
// simulation: +X points from launch to landing, +Y points to route-left and
// +Z is altitude above the landing elevation.
double RouteHorizontalDistanceM(const RouteProfile& route);
Vec3 GeoPointInPrimaryFrameM(const GeoPoint& point);
Vec3 RouteLaunchLocalM(const RouteProfile& route);
Vec3 RouteLandingLocalM(const RouteProfile& route);
double RouteLaunchHeightM(const RouteProfile& route);
SiteWindAssessment AssessRouteWind(
    const RouteProfile& route, double windFromDegrees, double windSpeedMps);
double RouteWindDirectionErrorDegrees(
    const RouteProfile& route, double windFromDegrees);
const char* SiteWindAssessmentName(SiteWindAssessment assessment);
}
