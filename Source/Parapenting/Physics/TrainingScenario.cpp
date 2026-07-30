#include "TrainingScenario.h"

#include <array>

namespace Parapenting::Physics
{
namespace
{
const std::array<TrainingScenario, 8> Scenarios{{
    {TrainingScenarioId::FreeFlight, "Free flight",
     "Explore the selected route and weather", WeatherMode::LocalizedRotor,
     -1.0, IncidentCue::None, -1.0, IncidentCue::None},
    {TrainingScenarioId::ThermalCentering, "Thermal centering",
     "Find the core, bank consistently and avoid the sink ring", WeatherMode::Ridge,
     -1.0, IncidentCue::None, -1.0, IncidentCue::None},
    {TrainingScenarioId::LeeRotor, "Lee rotor awareness",
     "Cross broken air with active pitch and heading control", WeatherMode::LocalizedRotor,
     -1.0, IncidentCue::None, -1.0, IncidentCue::None},
    {TrainingScenarioId::AsymmetricRecovery, "Asymmetric recovery",
     "Control heading, unload, then pump the collapsed side", WeatherMode::Chill,
     8.0, IncidentCue::LeftCollapse, -1.0, IncidentCue::None},
    {TrainingScenarioId::FrontalRecovery, "Frontal recovery",
     "Hands up, contain the surge, regain trim speed", WeatherMode::Chill,
     8.0, IncidentCue::FrontalCollapse, -1.0, IncidentCue::None},
    {TrainingScenarioId::Cascade, "Rotor cascade",
     "Manage pressure loss, heading and sequential incidents", WeatherMode::RotorEverywhere,
     6.0, IncidentCue::RightCollapse, 13.0, IncidentCue::FrontalCollapse},
    {TrainingScenarioId::SpiralRecovery, "Spiral energy management",
     "Release inside brake progressively, reduce bank and contain the exit",
     WeatherMode::Chill,
     8.0, IncidentCue::RightSpiral, -1.0, IncidentCue::None},
    {TrainingScenarioId::LandingFlare, "Flare and run-out",
     "Stabilize final, preserve speed, flare late and run out the landing",
     WeatherMode::Chill,
     -1.0, IncidentCue::None, -1.0, IncidentCue::None}
}};
}

const TrainingScenario& GetTrainingScenarioByIndex(std::size_t index)
{
    return Scenarios[index % Scenarios.size()];
}

std::size_t TrainingScenarioCount()
{
    return Scenarios.size();
}

IncidentCue ScenarioCueCrossed(
    const TrainingScenario& scenario, double previousTimeS, double currentTimeS)
{
    if (scenario.firstCue != IncidentCue::None
        && previousTimeS < scenario.firstCueTimeS
        && currentTimeS >= scenario.firstCueTimeS)
        return scenario.firstCue;
    if (scenario.secondCue != IncidentCue::None
        && previousTimeS < scenario.secondCueTimeS
        && currentTimeS >= scenario.secondCueTimeS)
        return scenario.secondCue;
    return IncidentCue::None;
}
}
