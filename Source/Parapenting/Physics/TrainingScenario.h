#pragma once

#include "AtmosphereModel.h"

#include <cstddef>

namespace Parapenting::Physics
{
enum class TrainingScenarioId
{
    FreeFlight,
    ThermalCentering,
    LeeRotor,
    AsymmetricRecovery,
    FrontalRecovery,
    Cascade,
    SpiralRecovery,
    LandingFlare
};

enum class IncidentCue
{
    None,
    LeftCollapse,
    RightCollapse,
    FrontalCollapse,
    RightSpiral
};

struct TrainingScenario
{
    TrainingScenarioId id;
    const char* displayName;
    const char* objective;
    WeatherMode weather;
    double firstCueTimeS;
    IncidentCue firstCue;
    double secondCueTimeS;
    IncidentCue secondCue;
};

const TrainingScenario& GetTrainingScenarioByIndex(std::size_t index);
std::size_t TrainingScenarioCount();
IncidentCue ScenarioCueCrossed(
    const TrainingScenario& scenario, double previousTimeS, double currentTimeS);
}
