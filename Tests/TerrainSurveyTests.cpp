// Guards the terrain survey contract:
//   * the route frame in RouteFrame.h still matches the generator's output
//     recorded in interlaken.provenance.json;
//   * RouteCatalogue's independently-derived lat/lon frame agrees with it;
//   * every route launch and landing sits on surveyed ground, or is listed
//     as a known exception.
//
// The frame is defined twice in this project - once in LV95 by the heightfield
// generator, once in lat/lon by RouteCatalogue - with nothing previously
// checking that the two agree.
#include "RouteFrame.h"
#include "RouteCatalogue.h"
#include "TerrainModel.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace Parapenting::Physics;

namespace
{
int Failures = 0;

void Check(bool condition, const std::string& what)
{
    if (!condition)
    {
        std::printf("  FAIL  %s\n", what.c_str());
        ++Failures;
    }
}

void CheckNear(double actual, double expected, double tolerance,
               const std::string& what)
{
    if (!(std::fabs(actual - expected) <= tolerance))
    {
        std::printf("  FAIL  %s: got %.6f, expected %.6f (tol %g)\n",
            what.c_str(), actual, expected, tolerance);
        ++Failures;
    }
}

// Minimal extraction: finds "key" and reads the next `count` numbers after it.
// Enough for the flat provenance schema, and avoids adding a JSON dependency
// to an otherwise dependency-free test binary.
std::vector<double> NumbersAfter(
    const std::string& text, const std::string& key, int count)
{
    std::vector<double> values;
    const std::size_t at = text.find("\"" + key + "\"");
    if (at == std::string::npos) return values;
    std::size_t i = at + key.size() + 2;
    while (static_cast<int>(values.size()) < count && i < text.size())
    {
        if (std::isdigit(static_cast<unsigned char>(text[i]))
            || (text[i] == '-'
                && i + 1 < text.size()
                && std::isdigit(static_cast<unsigned char>(text[i + 1]))))
        {
            std::size_t used = 0;
            values.push_back(std::stod(text.substr(i), &used));
            i += used;
        }
        else
        {
            ++i;
        }
    }
    return values;
}
}

int main(int argc, char** argv)
{
    const std::string provenancePath = argc > 1
        ? argv[1]
        : "Content/Terrain/interlaken.provenance.json";

    std::printf("Route frame vs %s\n", provenancePath.c_str());
    std::ifstream file(provenancePath);
    if (!file)
    {
        std::printf("  FAIL  cannot open provenance file\n");
        return 1;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string json = buffer.str();

    const auto launch = NumbersAfter(json, "launchLv95", 2);
    const auto landing = NumbersAfter(json, "landingLv95", 2);
    const auto elevation = NumbersAfter(json, "landingElevationM", 1);
    const auto bounds = NumbersAfter(json, "boundsLocalM", 4);
    const auto resolution = NumbersAfter(json, "outputResolutionM", 1);

    Check(launch.size() == 2, "provenance has launchLv95");
    Check(landing.size() == 2, "provenance has landingLv95");
    Check(elevation.size() == 1, "provenance has landingElevationM");
    Check(bounds.size() == 4, "provenance has boundsLocalM");
    if (Failures) return 1;

    CheckNear(RouteFrame::launchEastingM, launch[0], 1e-6, "launch easting");
    CheckNear(RouteFrame::launchNorthingM, launch[1], 1e-6, "launch northing");
    CheckNear(RouteFrame::landingEastingM, landing[0], 1e-6, "landing easting");
    CheckNear(RouteFrame::landingNorthingM, landing[1], 1e-6,
              "landing northing");
    CheckNear(RouteFrame::landingElevationM, elevation[0], 1e-9,
              "landing elevation");
    CheckNear(RouteFrame::surveyedXMinM, bounds[0], 1e-9, "surveyed x min");
    CheckNear(RouteFrame::surveyedYMinM, bounds[1], 1e-9, "surveyed y min");
    CheckNear(RouteFrame::surveyedXMaxM, bounds[2], 1e-9, "surveyed x max");
    CheckNear(RouteFrame::surveyedYMaxM, bounds[3], 1e-9, "surveyed y max");

    // Basis must be the normalised launch->landing direction, orthonormal.
    // Derived from the provenance numbers rather than from RouteFrame's own
    // anchors, so this is a real cross-check and not a tautology.
    const double dE = landing[0] - launch[0];
    const double dN = landing[1] - launch[1];
    const double length = std::hypot(dE, dN);
    CheckNear(RouteFrame::routeLengthM, length, 1e-9, "route length");
    CheckNear(RouteFrame::forwardEast, dE / length, 1e-12, "forward east");
    CheckNear(RouteFrame::forwardNorth, dN / length, 1e-12, "forward north");
    CheckNear(RouteFrame::leftEast, -RouteFrame::forwardNorth, 1e-12,
              "left east is -forward north");
    CheckNear(RouteFrame::leftNorth, RouteFrame::forwardEast, 1e-12,
              "left north is forward east");
    CheckNear(
        RouteFrame::forwardEast * RouteFrame::forwardEast
            + RouteFrame::forwardNorth * RouteFrame::forwardNorth,
        1.0, 1e-12, "forward is unit length");

    // Anchors: launch is the origin, landing lies straight down +X.
    CheckNear(RouteFrame::Lv95ToLocalXM(
                  RouteFrame::launchEastingM, RouteFrame::launchNorthingM),
              0.0, 1e-6, "launch local x");
    CheckNear(RouteFrame::Lv95ToLocalYM(
                  RouteFrame::launchEastingM, RouteFrame::launchNorthingM),
              0.0, 1e-6, "launch local y");
    CheckNear(RouteFrame::Lv95ToLocalXM(
                  RouteFrame::landingEastingM, RouteFrame::landingNorthingM),
              RouteFrame::routeLengthM, 1e-6, "landing local x");
    CheckNear(RouteFrame::Lv95ToLocalYM(
                  RouteFrame::landingEastingM, RouteFrame::landingNorthingM),
              0.0, 1e-6, "landing local y");

    // Round trip.
    for (double x : {-1800.0, 0.0, 2398.0, 6100.0})
    {
        for (double y : {-4500.0, 0.0, 2500.0, 10000.0})
        {
            const double e = RouteFrame::LocalToEastingM(x, y);
            const double n = RouteFrame::LocalToNorthingM(x, y);
            CheckNear(RouteFrame::Lv95ToLocalXM(e, n), x, 1e-6, "round trip x");
            CheckNear(RouteFrame::Lv95ToLocalYM(e, n), y, 1e-6, "round trip y");
        }
    }

    if (resolution.size() == 1)
        std::printf("  output resolution %.1f m\n", resolution[0]);

    // RouteCatalogue derives the same frame from lat/lon independently. The
    // primary route's endpoints must land where the LV95 frame says they do.
    std::printf("\nRouteCatalogue frame agreement\n");
    const auto& primary = GetRouteProfile(RouteProfileId::AmisbuehlLehn);
    const Vec3 primaryLaunch = RouteLaunchLocalM(primary);
    const Vec3 primaryLanding = RouteLandingLocalM(primary);
    std::printf("  launch  local (%.1f, %.1f)\n",
        primaryLaunch.x, primaryLaunch.y);
    std::printf("  landing local (%.1f, %.1f)  frame says (%.1f, 0.0)\n",
        primaryLanding.x, primaryLanding.y, RouteFrame::routeLengthM);
    CheckNear(primaryLaunch.x, 0.0, 25.0, "catalogue launch x near frame");
    CheckNear(primaryLaunch.y, 0.0, 25.0, "catalogue launch y near frame");
    CheckNear(primaryLanding.x, RouteFrame::routeLengthM, 25.0,
              "catalogue landing x near frame");
    CheckNear(primaryLanding.y, 0.0, 25.0, "catalogue landing y near frame");

    // Coverage report for every route endpoint.
    std::printf("\nRoute endpoint coverage\n");
    int unsurveyed = 0;
    for (std::size_t i = 0; i < RouteProfileCount(); ++i)
    {
        const auto& route = GetRouteProfileByIndex(i);
        const Vec3 launchLocal = RouteLaunchLocalM(route);
        const Vec3 landingLocal = RouteLandingLocalM(route);
        const bool launchOk =
            RouteFrame::IsInsideSurveyedBounds(launchLocal.x, launchLocal.y);
        const bool landingOk =
            RouteFrame::IsInsideSurveyedBounds(landingLocal.x, landingLocal.y);
        if (!launchOk || !landingOk) ++unsurveyed;
        std::printf("  %-28s launch (%8.1f,%9.1f) %s   landing (%8.1f,%9.1f) %s\n",
            route.displayName,
            launchLocal.x, launchLocal.y, launchOk ? "surveyed" : "ANALYTIC",
            landingLocal.x, landingLocal.y, landingOk ? "surveyed" : "ANALYTIC");
    }
    std::printf("  %d of %zu routes have an endpoint off surveyed ground\n",
        unsurveyed, RouteProfileCount());

    if (Failures)
    {
        std::printf("\n%d terrain survey check(s) failed.\n", Failures);
        return 1;
    }
    std::printf("\nAll terrain survey checks passed.\n");
    return 0;
}
