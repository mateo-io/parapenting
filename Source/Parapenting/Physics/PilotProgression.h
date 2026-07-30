#pragma once

#include <array>
#include <cstddef>

namespace Parapenting::Physics
{
constexpr std::size_t ProgressionScenarioCount = 8;

enum class MasteryMedal
{
    None,
    Bronze,
    Silver,
    Gold
};

struct PilotProgression
{
    const char* rankName = "STUDENT";
    double experience = 0.0;
    double rankProgress = 0.0;
    int bronzeMedals = 0;
    int silverMedals = 0;
    int goldMedals = 0;
    int masteredScenarios = 0;
};

MasteryMedal MedalForScore(double score);
PilotProgression EvaluatePilotProgression(
    const std::array<double, ProgressionScenarioCount>& scenarioBestScores);
}
