#include "PilotProgression.h"

#include <algorithm>

namespace Parapenting::Physics
{
namespace
{
struct RankThreshold
{
    double experience;
    int masteredScenarios;
    const char* name;
};

constexpr std::array<RankThreshold, 6> Ranks{{
    {0.0, 0, "STUDENT"},
    {600.0, 1, "NOVICE"},
    {1800.0, 2, "CLUB PILOT"},
    {3000.0, 3, "XC PILOT"},
    {4200.0, 4, "ADVANCED"},
    {6800.0, 8, "FLIGHT LAB MASTER"}
}};
}

MasteryMedal MedalForScore(double score)
{
    if (score >= 850.0) return MasteryMedal::Gold;
    if (score >= 700.0) return MasteryMedal::Silver;
    if (score >= 500.0) return MasteryMedal::Bronze;
    return MasteryMedal::None;
}

PilotProgression EvaluatePilotProgression(
    const std::array<double, ProgressionScenarioCount>& scores)
{
    PilotProgression result;
    for (const double rawScore : scores)
    {
        const double score = std::clamp(rawScore, 0.0, 1000.0);
        result.experience += score;
        const MasteryMedal medal = MedalForScore(score);
        if (medal == MasteryMedal::Bronze) ++result.bronzeMedals;
        if (medal == MasteryMedal::Silver) ++result.silverMedals;
        if (medal == MasteryMedal::Gold) ++result.goldMedals;
        if (score >= 700.0) ++result.masteredScenarios;
    }

    std::size_t currentRank = 0;
    for (std::size_t index = 1; index < Ranks.size(); ++index)
    {
        if (result.experience >= Ranks[index].experience
            && result.masteredScenarios >= Ranks[index].masteredScenarios)
            currentRank = index;
    }
    result.rankName = Ranks[currentRank].name;
    if (currentRank + 1 >= Ranks.size())
    {
        result.rankProgress = 1.0;
        return result;
    }

    const RankThreshold& current = Ranks[currentRank];
    const RankThreshold& next = Ranks[currentRank + 1];
    const double experienceProgress = std::clamp(
        (result.experience - current.experience)
            / (next.experience - current.experience),
        0.0, 1.0);
    const double masteryProgress = next.masteredScenarios == 0
        ? 1.0
        : std::clamp(
            static_cast<double>(result.masteredScenarios)
                / next.masteredScenarios,
            0.0, 1.0);
    result.rankProgress = std::min(experienceProgress, masteryProgress);
    return result;
}
}
