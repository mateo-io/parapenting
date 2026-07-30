#include "FlightNavigation.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Parapenting::Physics
{
namespace
{
constexpr double RadToDeg = 180.0 / 3.14159265358979323846;
}

NavigationRoute BuildNavigationRoute(
    const Vec3& launch, const Vec3& landing)
{
    const Vec3 route = landing - launch;
    return {{{
        {"LAUNCH EXIT", launch + route * 0.18, 260.0, 80.0, 450.0},
        {"VALLEY TRANSIT", launch + route * 0.62, 330.0, 140.0, 500.0},
        {"LANDING", landing, 240.0, 65.0, 120.0}
    }}};
}

bool UpdateNavigationProgress(
    NavigationProgress& progress, const NavigationRoute& route,
    const Vec3& position)
{
    if (progress.complete) return false;
    const std::size_t index =
        std::min(progress.activeWaypoint, route.waypoints.size() - 1);
    const NavigationWaypoint& waypoint = route.waypoints[index];
    const double horizontalDistance = std::hypot(
        position.x - waypoint.positionWorldM.x,
        position.y - waypoint.positionWorldM.y);
    if (horizontalDistance > waypoint.captureRadiusM) return false;
    if (position.z - waypoint.positionWorldM.z
        > waypoint.maximumCaptureHeightM)
        return false;
    if (index + 1 < route.waypoints.size())
        ++progress.activeWaypoint;
    else
        progress.complete = true;
    return true;
}

GlideNavigationSolution EvaluateGlideNavigation(
    const Vec3& position,
    const NavigationWaypoint& waypoint,
    const Vec3& wind,
    const SteadyPolarPoint& polar,
    double measuredAirspeedMps,
    double verticalAirMps)
{
    GlideNavigationSolution result;
    const Vec3 offset = waypoint.positionWorldM - position;
    result.distanceM = std::hypot(offset.x, offset.y);
    if (result.distanceM < 1.0)
    {
        result.reachable =
            position.z - waypoint.positionWorldM.z
                >= waypoint.desiredArrivalHeightM;
        return result;
    }
    const Vec3 track{offset.x / result.distanceM,
                     offset.y / result.distanceM, 0.0};
    const Vec3 left{-track.y, track.x, 0.0};
    result.bearingWorldDegrees = std::atan2(track.y, track.x) * RadToDeg;
    if (result.bearingWorldDegrees < 0.0)
        result.bearingWorldDegrees += 360.0;
    result.windAlongTrackMps = Dot(wind, track);
    result.crosswindMps = Dot(wind, left);

    const double airspeed = std::max(
        3.0, measuredAirspeedMps > 2.0
            ? measuredAirspeedMps : polar.airspeedMps);
    const double crosswindRatio = result.crosswindMps / airspeed;
    result.crosswindFeasible = std::abs(crosswindRatio) < 0.98;
    if (result.crosswindFeasible)
    {
        result.crabAngleDegrees =
            -std::asin(std::clamp(crosswindRatio, -1.0, 1.0)) * RadToDeg;
        const double alongAir =
            std::sqrt(std::max(0.0,
                airspeed * airspeed
                    - result.crosswindMps * result.crosswindMps));
        result.predictedGroundSpeedMps =
            alongAir + result.windAlongTrackMps;
    }
    else
    {
        result.crabAngleDegrees =
            result.crosswindMps > 0.0 ? -90.0 : 90.0;
        result.predictedGroundSpeedMps = 0.0;
    }

    result.predictedSinkMps =
        std::max(0.12, polar.sinkRateMps - verticalAirMps);
    if (result.predictedGroundSpeedMps > 0.2)
    {
        result.timeToWaypointS =
            result.distanceM / result.predictedGroundSpeedMps;
        const double heightLoss =
            result.predictedSinkMps * result.timeToWaypointS;
        result.predictedArrivalHeightM =
            position.z - waypoint.positionWorldM.z - heightLoss;
        result.availableGroundGlideRatio =
            result.predictedGroundSpeedMps / result.predictedSinkMps;
    }
    else
    {
        result.timeToWaypointS = std::numeric_limits<double>::infinity();
        result.predictedArrivalHeightM =
            -std::numeric_limits<double>::infinity();
    }
    const double usableHeight =
        position.z - waypoint.positionWorldM.z
        - waypoint.desiredArrivalHeightM;
    result.requiredGlideRatio = usableHeight > 1.0
        ? result.distanceM / usableHeight
        : std::numeric_limits<double>::infinity();
    result.reachable =
        result.crosswindFeasible
        && result.predictedGroundSpeedMps > 0.2
        && result.predictedArrivalHeightM
            >= waypoint.desiredArrivalHeightM;

    if (verticalAirMps > 0.7)
        result.speedToFly = SpeedToFlyCue::MinimumSink;
    else if (verticalAirMps < -1.2 || result.windAlongTrackMps < -3.0)
        result.speedToFly = SpeedToFlyCue::Accelerate;
    else if (!result.reachable)
        result.speedToFly = SpeedToFlyCue::BestGlide;
    else
        result.speedToFly = SpeedToFlyCue::Hold;
    return result;
}

const char* SpeedToFlyCueName(SpeedToFlyCue cue)
{
    switch (cue)
    {
        case SpeedToFlyCue::MinimumSink: return "MIN SINK";
        case SpeedToFlyCue::BestGlide: return "BEST GLIDE";
        case SpeedToFlyCue::Accelerate: return "ACCELERATE";
        case SpeedToFlyCue::Hold: return "HOLD";
    }
    return "HOLD";
}
}
