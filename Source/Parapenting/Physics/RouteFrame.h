#pragma once

#include <cmath>

namespace Parapenting::Physics
{
// The local simulation frame is route-aligned, not north-aligned. Every
// coordinate in the simulator is expressed in it, and until now its definition
// lived only inside Tools/Terrain/build_interlaken_heightfield.py: no engine
// code carried the transform, so a heightfield regenerated against a different
// origin would have loaded silently and moved the entire world.
//
// These constants mirror the generator exactly. RouteFrameTests asserts that
// they still agree with Content/Terrain/interlaken.provenance.json, so the two
// cannot drift apart unnoticed.
//
//   origin : Amisbuehl oben launch
//   +X     : along the Amisbuehl -> Lehn route (bearing ~176.7 deg, ~due south)
//   +Y     : route-left (bearing ~86.7 deg, ~due east)
//   Z      : metres relative to the Lehn landing field
//   CRS    : LV95 / LN02, EPSG:2056
struct RouteFrame
{
    static constexpr double launchEastingM = 2629258.04;
    static constexpr double launchNorthingM = 1172293.04;
    static constexpr double landingEastingM = 2629396.79;
    static constexpr double landingNorthingM = 1169899.00;
    static constexpr double landingElevationM = 565.0;

    // Unit basis, derived from the launch/landing pair exactly as the
    // generator does it. Derived rather than transcribed: hand-written
    // constants cannot be held to machine precision, and a basis that is only
    // approximately orthonormal quietly bends every coordinate in the world.
    // The anchors above are the single source of truth; RouteFrameTests
    // checks them against the provenance file.
    static inline const double routeLengthM = std::hypot(
        landingEastingM - launchEastingM, landingNorthingM - launchNorthingM);
    static inline const double forwardEast =
        (landingEastingM - launchEastingM) / routeLengthM;
    static inline const double forwardNorth =
        (landingNorthingM - launchNorthingM) / routeLengthM;
    static inline const double leftEast = -forwardNorth;
    static inline const double leftNorth = forwardEast;

    // Surveyed extent of Content/Terrain/interlaken.asc. Samples outside this
    // rectangle fall through to the analytic proxy.
    static constexpr double surveyedXMinM = -1800.0;
    static constexpr double surveyedXMaxM = 6100.0;
    static constexpr double surveyedYMinM = -4500.0;
    static constexpr double surveyedYMaxM = 3500.0;

    static constexpr bool IsInsideSurveyedBounds(double xM, double yM)
    {
        return xM >= surveyedXMinM && xM <= surveyedXMaxM
            && yM >= surveyedYMinM && yM <= surveyedYMaxM;
    }

    static double LocalToEastingM(double xM, double yM)
    {
        return launchEastingM + xM * forwardEast + yM * leftEast;
    }

    static double LocalToNorthingM(double xM, double yM)
    {
        return launchNorthingM + xM * forwardNorth + yM * leftNorth;
    }

    // The basis is orthonormal, so the inverse is the transpose.
    static double Lv95ToLocalXM(double eastingM, double northingM)
    {
        return (eastingM - launchEastingM) * forwardEast
            + (northingM - launchNorthingM) * forwardNorth;
    }

    static double Lv95ToLocalYM(double eastingM, double northingM)
    {
        return (eastingM - launchEastingM) * leftEast
            + (northingM - launchNorthingM) * leftNorth;
    }

    static constexpr double LocalToElevationM(double zM)
    {
        return zM + landingElevationM;
    }

    static constexpr double ElevationToLocalM(double elevationM)
    {
        return elevationM - landingElevationM;
    }
};
}
