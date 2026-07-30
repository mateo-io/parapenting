#pragma once

#include "AerodynamicPolar.h"
#include "ParagliderDynamics.h"

#include <array>
#include <cstddef>

namespace Parapenting::Physics
{
enum class SpeedToFlyCue
{
    MinimumSink,
    BestGlide,
    Accelerate,
    Hold
};

struct NavigationWaypoint
{
    const char* displayName = "LANDING";
    Vec3 positionWorldM{};
    double captureRadiusM = 220.0;
    double desiredArrivalHeightM = 80.0;
    double maximumCaptureHeightM = 300.0;
};

struct NavigationRoute
{
    std::array<NavigationWaypoint, 3> waypoints{};
};

struct NavigationProgress
{
    std::size_t activeWaypoint = 0;
    bool complete = false;
};

struct GlideNavigationSolution
{
    double distanceM = 0.0;
    double bearingWorldDegrees = 0.0;
    double windAlongTrackMps = 0.0;
    double crosswindMps = 0.0;
    double crabAngleDegrees = 0.0;
    double predictedGroundSpeedMps = 0.0;
    double predictedSinkMps = 0.0;
    double timeToWaypointS = 0.0;
    double predictedArrivalHeightM = 0.0;
    double requiredGlideRatio = 0.0;
    double availableGroundGlideRatio = 0.0;
    bool crosswindFeasible = true;
    bool reachable = false;
    SpeedToFlyCue speedToFly = SpeedToFlyCue::Hold;
};

NavigationRoute BuildNavigationRoute(
    const Vec3& launchWorldM, const Vec3& landingWorldM);
bool UpdateNavigationProgress(
    NavigationProgress& progress, const NavigationRoute& route,
    const Vec3& positionWorldM);
GlideNavigationSolution EvaluateGlideNavigation(
    const Vec3& positionWorldM,
    const NavigationWaypoint& waypoint,
    const Vec3& windWorldMps,
    const SteadyPolarPoint& polar,
    double measuredAirspeedMps,
    double verticalAirMps);
const char* SpeedToFlyCueName(SpeedToFlyCue cue);
}
