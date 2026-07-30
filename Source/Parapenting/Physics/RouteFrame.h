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
    // WGS84 is authoritative: these must match the Amisbuehl oben and Lehn
    // GeoPoints in RouteCatalogue.cpp, and the generator's anchors.
    static constexpr double launchLatitudeDeg = 46.702500;
    static constexpr double launchLongitudeDeg = 7.822200;
    static constexpr double landingLatitudeDeg = 46.680956;
    static constexpr double landingLongitudeDeg = 7.823861;
    static constexpr double landingElevationM = 565.0;

    // swisstopo approximate WGS84 -> LV95 (EPSG:2056), ~1 m accurate;
    // reproduces the Bern reference point to 0.34 m.
    static double Wgs84ToEastingM(double latitudeDeg, double longitudeDeg)
    {
        const double phi = (latitudeDeg * 3600.0 - 169028.66) / 10000.0;
        const double lam = (longitudeDeg * 3600.0 - 26782.5) / 10000.0;
        return 2600072.37 + 211455.93 * lam - 10938.51 * lam * phi
            - 0.36 * lam * phi * phi - 44.54 * lam * lam * lam;
    }

    static double Wgs84ToNorthingM(double latitudeDeg, double longitudeDeg)
    {
        const double phi = (latitudeDeg * 3600.0 - 169028.66) / 10000.0;
        const double lam = (longitudeDeg * 3600.0 - 26782.5) / 10000.0;
        return 1200147.07 + 308807.95 * phi + 3745.25 * lam * lam
            + 76.63 * phi * phi - 194.56 * lam * lam * phi
            + 119.79 * phi * phi * phi;
    }

    // Projected, not transcribed. A hardcoded LV95 pair previously sat
    // (+76.1, +143.6) m from where these anchors actually project, shifting
    // the whole surveyed grid 163 m relative to the sites. On the four sites
    // then measured, mean elevation error was 14.5 m; deriving the projection
    // brings it to 1.1 m.
    static inline const double launchEastingM =
        Wgs84ToEastingM(launchLatitudeDeg, launchLongitudeDeg);
    static inline const double launchNorthingM =
        Wgs84ToNorthingM(launchLatitudeDeg, launchLongitudeDeg);
    static inline const double landingEastingM =
        Wgs84ToEastingM(landingLatitudeDeg, landingLongitudeDeg);
    static inline const double landingNorthingM =
        Wgs84ToNorthingM(landingLatitudeDeg, landingLongitudeDeg);

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
