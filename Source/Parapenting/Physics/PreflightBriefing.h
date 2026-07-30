#pragma once

#include "AtmosphereModel.h"
#include "RouteCatalogue.h"

namespace Parapenting::Physics
{
enum class PreflightRisk
{
    Low,
    Moderate,
    High,
    Extreme
};

struct PreflightBriefing
{
    SiteWindAssessment launchWindAssessment = SiteWindAssessment::Suitable;
    PreflightRisk overallRisk = PreflightRisk::Low;
    double suitabilityScore = 100.0;
    double windFromDegrees = 0.0;
    double launchWindMps = 0.0;
    double landingWindMps = 0.0;
    double cruiseWindMps = 0.0;
    double gustSpreadMps = 0.0;
    double launchDirectionErrorDegrees = 0.0;
    double thermalStrengthMps = 0.0;
    double thermalTopMslM = 0.0;
    double cloudBaseAboveLaunchM = 0.0;
    double cloudCoverage = 0.0;
    double rotorRisk = 0.0;
    double turbulenceRisk = 0.0;
    int authoredRotorVolumes = 0;
    bool advancedLanding = false;
    bool liveWeather = false;
};

PreflightBriefing EvaluatePreflightBriefing(
    const RouteProfile& route,
    const WeatherSnapshot& snapshot,
    const WeatherParameters& parameters,
    const std::array<WeatherVolume, 5>& volumes,
    const CloudFieldState& clouds,
    const Atmosphere& launchAir,
    const Atmosphere& landingAir,
    const Atmosphere& cruiseAir);
const char* PreflightRiskName(PreflightRisk risk);
const char* PreflightRecommendation(const PreflightBriefing& briefing);
}
