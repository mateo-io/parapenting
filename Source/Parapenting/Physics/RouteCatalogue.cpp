#include "RouteCatalogue.h"

#include <array>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double EarthRadiusM = 6371008.8;
constexpr double DegToRad = 3.14159265358979323846 / 180.0;
constexpr double ReferenceLatitudeDeg = 46.7025;
constexpr double ReferenceLongitudeDeg = 7.8222;
constexpr double PrimaryLandingLatitudeDeg = 46.680956;
constexpr double PrimaryLandingLongitudeDeg = 7.823861;

const std::array<RouteProfile, 10> Routes = {{
    {RouteProfileId::AmisbuehlLehn, "Amisbuehl -> Lehn",
     {"amisbuehl-upper", "Amisbuehl oben", 46.7025, 7.8222, 1325.0},
     {"lehn", "Lehn", 46.680956, 7.823861, 565.0}, false,
     135.0, 55.0, 5.5,
     "Lee risk in mountain wind; respect separated launch lanes.",
     "Valley wind: right circuit. Mountain wind: left circuit.",
     "SHV/FSVL Interlaken site board"},
    {RouteProfileId::AmisbuehlHoehematte, "Amisbuehl -> Hoehematte",
     {"amisbuehl-upper", "Amisbuehl oben", 46.7025, 7.8222, 1325.0},
     {"hoehematte", "Hoehematte", 46.686117, 7.858719, 569.0}, true,
     135.0, 55.0, 5.5,
     "Lee risk in mountain wind; respect separated launch lanes.",
     "Valley wind: left circuit. Mountain wind: right circuit. "
     "Expect urban lee, tandem traffic and strong-valley-wind turbulence.",
     "DCI Amisbuehl site page and SHV/FSVL Interlaken site board"},
    {RouteProfileId::BergboLehn, "Bergbo -> Lehn",
     {"bergbo", "Bergbo", 46.70034, 7.82049, 1275.0},
     {"lehn", "Lehn", 46.680956, 7.823861, 565.0}, false,
     135.0, 52.0, 5.0,
     "Strong Bise places this southeast launch in lee.",
     "Valley wind: right circuit. Mountain wind: left circuit.",
     "SHV/FSVL Interlaken site board"},
    {RouteProfileId::HohwaldHoehematte, "Hohwald -> Hoehematte",
     {"hohwald", "Hohwald", 46.7138, 7.8238, 1585.0},
     {"hoehematte", "Hoehematte", 46.686117, 7.858719, 569.0}, true,
     157.5, 50.0, 4.8,
     "Northwest flow can be turbulent; closed during ski operations.",
     "Valley wind: left circuit. Mountain wind: right circuit. "
     "Expect urban lee, tandem traffic and strong-valley-wind turbulence.",
     "DCI Hohwald site page and SHV/FSVL Interlaken site board"},
    {RouteProfileId::BergboHoehematte, "Bergbo -> Hoehematte",
     {"bergbo", "Bergbo", 46.70034, 7.82049, 1275.0},
     {"hoehematte", "Hoehematte", 46.686117, 7.858719, 569.0}, true,
     135.0, 52.0, 5.0,
     "Strong Bise places this southeast launch in lee.",
     "Valley wind: left circuit. Mountain wind: right circuit.",
     "SHV/FSVL Interlaken site board"},
    {RouteProfileId::HohwaldLehn, "Hohwald -> Lehn",
     {"hohwald", "Hohwald", 46.7138, 7.8238, 1585.0},
     {"lehn", "Lehn", 46.680956, 7.823861, 565.0}, false,
     157.5, 50.0, 4.8,
     "Northwest flow can be turbulent; exposed in strong upper wind.",
     "Valley wind: right circuit. Mountain wind: left circuit.",
     "SHV/FSVL Interlaken site board"},
    {RouteProfileId::NiederhornLehn, "Niederhorn south -> Lehn",
     {"niederhorn-south", "Niederhorn south", 46.711111, 7.777894, 1926.0},
     {"lehn", "Lehn", 46.680956, 7.823861, 565.0}, false,
     180.0, 55.0, 4.5,
     "Flat launch; downdraft possible at thermal onset; upper-wind exposure.",
     "Valley wind: right circuit. Mountain wind: left circuit.",
     "SHV/FSVL Interlaken site board"},
    {RouteProfileId::NiederhornHoehematte, "Niederhorn south -> Hoehematte",
     {"niederhorn-south", "Niederhorn south", 46.711111, 7.777894, 1926.0},
     {"hoehematte", "Hoehematte", 46.686117, 7.858719, 569.0}, true,
     180.0, 55.0, 4.5,
     "Flat launch; downdraft possible at thermal onset; upper-wind exposure.",
     "Valley wind: left circuit. Mountain wind: right circuit.",
     "SHV/FSVL Interlaken site board"},
    {RouteProfileId::GrindelwaldFirstGrund, "Grindelwald First -> Grund",
     {"grindelwald-first", "Grindelwald First",
        46.657450, 8.055133, 2123.0},
     {"grindelwald-grund", "Grindelwald Grund",
        46.620292, 8.029056, 950.0}, false,
     180.0, 45.0, 5.5,
     "2024 club rule: launch only south of the start tower or west of the "
     "path. Long glide to Grund; landing frequencies can be limited.",
     "Simulator circuit only. Confirm current local procedure and traffic.",
     "Jungfrau-Taechi 2024 First notice; First rules; DHV First 1585"},
    {RouteProfileId::GrindelwaldFirstBodmi, "Grindelwald First -> Bodmi",
     {"grindelwald-first", "Grindelwald First",
        46.657450, 8.055133, 2123.0},
     {"grindelwald-bodmi", "Grindelwald Bodmi",
        46.628758, 8.043319, 1129.0}, true,
     180.0, 45.0, 5.0,
     "2024 club rule: launch only south of the start tower or west of the "
     "path. Starts are southeast through southwest.",
     "Advanced landing above the valley station; experienced pilots only. "
     "Simulator circuit only; confirm current local procedure.",
     "Jungfrau-Taechi 2024 First notice; Paragliding Jungfrau FAQ; DHV First 1585"}
}};
}

const RouteProfile& GetRouteProfile(RouteProfileId id)
{
    for (const RouteProfile& route : Routes)
        if (route.id == id) return route;
    return Routes.front();
}

const RouteProfile& GetRouteProfileByIndex(std::size_t index)
{
    return Routes[index % Routes.size()];
}

std::size_t RouteProfileCount()
{
    return Routes.size();
}

double RouteHorizontalDistanceM(const RouteProfile& route)
{
    const double lat1 = route.launch.latitudeDeg * DegToRad;
    const double lat2 = route.landing.latitudeDeg * DegToRad;
    const double dLat = lat2 - lat1;
    const double dLon =
        (route.landing.longitudeDeg - route.launch.longitudeDeg) * DegToRad;
    const double a = std::sin(dLat * 0.5) * std::sin(dLat * 0.5)
        + std::cos(lat1) * std::cos(lat2)
        * std::sin(dLon * 0.5) * std::sin(dLon * 0.5);
    return EarthRadiusM * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

Vec3 GeoPointInPrimaryFrameM(const GeoPoint& point)
{
    const double meanLatitude =
        0.5 * (point.latitudeDeg + ReferenceLatitudeDeg) * DegToRad;
    const double east = (point.longitudeDeg - ReferenceLongitudeDeg)
        * DegToRad * EarthRadiusM * std::cos(meanLatitude);
    const double north = (point.latitudeDeg - ReferenceLatitudeDeg)
        * DegToRad * EarthRadiusM;
    const double primaryLandingEast =
        (PrimaryLandingLongitudeDeg - ReferenceLongitudeDeg)
        * DegToRad * EarthRadiusM
        * std::cos(0.5 * (PrimaryLandingLatitudeDeg
                        + ReferenceLatitudeDeg) * DegToRad);
    const double primaryLandingNorth =
        (PrimaryLandingLatitudeDeg - ReferenceLatitudeDeg)
        * DegToRad * EarthRadiusM;
    const double primaryLength =
        std::hypot(primaryLandingEast, primaryLandingNorth);
    const double forwardEast = primaryLandingEast / primaryLength;
    const double forwardNorth = primaryLandingNorth / primaryLength;
    const double leftEast = -forwardNorth;
    const double leftNorth = forwardEast;
    return {
        east * forwardEast + north * forwardNorth,
        east * leftEast + north * leftNorth,
        point.elevationM - 565.0
    };
}

namespace
{
Vec3 RoutePointLocalM(const RouteProfile& route, const GeoPoint& point)
{
    if (route.id != RouteProfileId::GrindelwaldFirstGrund
        && route.id != RouteProfileId::GrindelwaldFirstBodmi)
        return GeoPointInPrimaryFrameM(point);
    const GeoPoint first{
        "first-frame", "First frame", 46.657450, 8.055133, 2123.0};
    const Vec3 origin = GeoPointInPrimaryFrameM(first);
    const Vec3 raw = GeoPointInPrimaryFrameM(point) - origin;
    return {
        raw.x, 8500.0 + raw.y,
        point.elevationM - 565.0};
}
}

Vec3 RouteLandingLocalM(const RouteProfile& route)
{
    return RoutePointLocalM(route, route.landing);
}

Vec3 RouteLaunchLocalM(const RouteProfile& route)
{
    return RoutePointLocalM(route, route.launch);
}

double RouteLaunchHeightM(const RouteProfile& route)
{
    return route.launch.elevationM - route.landing.elevationM;
}

double RouteWindDirectionErrorDegrees(
    const RouteProfile& route, double windFromDegrees)
{
    double difference = std::fmod(
        std::abs(windFromDegrees - route.launchFacingDegrees), 360.0);
    if (difference > 180.0) difference = 360.0 - difference;
    return difference;
}

SiteWindAssessment AssessRouteWind(
    const RouteProfile& route, double windFromDegrees, double windSpeedMps)
{
    if (windSpeedMps > route.simulatorMaxLaunchWindMps)
        return SiteWindAssessment::TooStrong;
    if (RouteWindDirectionErrorDegrees(route, windFromDegrees)
        > route.preferredWindHalfWidthDegrees)
        return SiteWindAssessment::MarginalDirection;
    return SiteWindAssessment::Suitable;
}

const char* SiteWindAssessmentName(SiteWindAssessment assessment)
{
    switch (assessment)
    {
        case SiteWindAssessment::Suitable: return "SIM ENVELOPE OK";
        case SiteWindAssessment::MarginalDirection: return "SIM WIND MARGINAL";
        case SiteWindAssessment::TooStrong: return "SIM WIND TOO STRONG";
    }
    return "SIM WIND UNKNOWN";
}
}
