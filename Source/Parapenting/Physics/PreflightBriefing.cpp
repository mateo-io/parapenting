#include "PreflightBriefing.h"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace Parapenting::Physics
{
namespace
{
double Clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}
}

PreflightBriefing EvaluatePreflightBriefing(
    const RouteProfile& route,
    const WeatherSnapshot& snapshot,
    const WeatherParameters& parameters,
    const std::array<WeatherVolume, 5>& volumes,
    const CloudFieldState& clouds,
    const Atmosphere& launchAir,
    const Atmosphere& landingAir,
    const Atmosphere& cruiseAir)
{
    PreflightBriefing result;
    const auto horizontalSpeed = [](const Vec3& wind)
    {
        return std::hypot(wind.x, wind.y);
    };
    result.windFromDegrees =
        MeteorologicalDirectionFromWindVector(launchAir.windWorldMps);
    result.launchWindMps = horizontalSpeed(launchAir.windWorldMps);
    result.landingWindMps = horizontalSpeed(landingAir.windWorldMps);
    result.cruiseWindMps = horizontalSpeed(cruiseAir.windWorldMps);
    result.gustSpreadMps = std::max(
        0.0, snapshot.gustSpeedMps - snapshot.windSpeedMps);
    result.launchDirectionErrorDegrees =
        RouteWindDirectionErrorDegrees(route, result.windFromDegrees);
    result.launchWindAssessment = AssessRouteWind(
        route, result.windFromDegrees, result.launchWindMps);
    result.thermalStrengthMps = parameters.thermalStrengthMps;
    result.thermalTopMslM = parameters.thermalTopMslM;
    result.cloudBaseAboveLaunchM =
        clouds.baseAltitudeM - route.launch.elevationM;
    result.cloudCoverage = clouds.coverage;
    result.advancedLanding = route.advancedLanding;
    const std::string_view source =
        snapshot.source ? std::string_view(snapshot.source) : std::string_view{};
    result.liveWeather =
        source.find("live") != std::string_view::npos
        || source.find("open-meteo")
            != std::string_view::npos;

    double strongestRotor = std::max({
        launchAir.rotorStrength,
        landingAir.rotorStrength,
        cruiseAir.rotorStrength});
    for (const WeatherVolume& volume : volumes)
    {
        if (volume.type == WeatherVolumeType::Rotor && volume.radiusM > 0.0)
        {
            ++result.authoredRotorVolumes;
            strongestRotor = std::max(strongestRotor, volume.strength);
        }
    }
    const double modeRotor =
        parameters.mode == WeatherMode::RotorEverywhere ? 1.0
        : (parameters.mode == WeatherMode::LocalizedRotor ? 0.58 : 0.0);
    result.rotorRisk = Clamp(
        std::max(strongestRotor, modeRotor)
            + parameters.turbulence * 0.18,
        0.0, 1.0);
    result.turbulenceRisk = Clamp(
        std::max({
            parameters.turbulence,
            launchAir.turbulence,
            landingAir.turbulence,
            cruiseAir.turbulence})
            + result.gustSpreadMps / 12.0,
        0.0, 1.0);

    double penalty = 0.0;
    if (result.launchWindAssessment == SiteWindAssessment::MarginalDirection)
        penalty += 24.0;
    else if (result.launchWindAssessment == SiteWindAssessment::TooStrong)
        penalty += 48.0;
    penalty += std::max(0.0,
        result.launchDirectionErrorDegrees
            - route.preferredWindHalfWidthDegrees) * 0.30;
    penalty += result.rotorRisk * 34.0;
    penalty += result.turbulenceRisk * 22.0;
    penalty += std::max(0.0,
        result.landingWindMps - (route.advancedLanding ? 3.5 : 4.5)) * 7.0;
    penalty += result.advancedLanding ? 8.0 : 0.0;
    if (result.cloudBaseAboveLaunchM < 250.0
        && result.thermalStrengthMps > 0.5)
        penalty += 14.0;
    result.suitabilityScore = Clamp(100.0 - penalty, 0.0, 100.0);
    result.overallRisk =
        result.suitabilityScore >= 78.0 ? PreflightRisk::Low
        : (result.suitabilityScore >= 55.0 ? PreflightRisk::Moderate
        : (result.suitabilityScore >= 30.0 ? PreflightRisk::High
                                           : PreflightRisk::Extreme));
    return result;
}

const char* PreflightRiskName(PreflightRisk risk)
{
    switch (risk)
    {
        case PreflightRisk::Low: return "LOW";
        case PreflightRisk::Moderate: return "MODERATE";
        case PreflightRisk::High: return "HIGH";
        case PreflightRisk::Extreme: return "EXTREME";
    }
    return "UNKNOWN";
}

const char* PreflightRecommendation(const PreflightBriefing& b)
{
    if (b.launchWindAssessment == SiteWindAssessment::TooStrong)
        return "DO NOT LAUNCH: ABOVE SIMULATOR SITE WIND ENVELOPE";
    if (b.rotorRisk > 0.72)
        return "SEVERE ROTOR EXPECTED: CHOOSE CALMER WEATHER";
    if (b.launchWindAssessment == SiteWindAssessment::MarginalDirection)
        return "MARGINAL LAUNCH DIRECTION: REVIEW LEE EXPOSURE";
    if (b.advancedLanding && b.landingWindMps > 3.5)
        return "ADVANCED LANDING: EXPECT TRAFFIC AND TURBULENCE";
    if (b.cloudBaseAboveLaunchM < 250.0 && b.thermalStrengthMps > 0.5)
        return "LOW CLOUD MARGIN: REMAIN CLEAR OF CLOUD";
    if (b.thermalStrengthMps > 1.5)
        return "ACTIVE THERMALS: EXPECT STRONG CORES AND SINK RINGS";
    return "SIMULATOR ENVELOPE ACCEPTABLE: REVIEW SITE HAZARDS";
}
}
