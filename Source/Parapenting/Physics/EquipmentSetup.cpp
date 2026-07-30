#include "EquipmentSetup.h"

#include <algorithm>

namespace Parapenting::Physics
{
namespace
{
constexpr std::array<HarnessProfile, 3> Profiles{{
    {HarnessType::SeatedSeatboard, "Seated seatboard", 5.5, 0.42,
        7.5, 3.2, 5.8, 2.8, 1.0},
    {HarnessType::Pod, "Pod harness", 7.0, 0.35,
        6.3, 2.6, 5.0, 2.3, 0.82},
    {HarnessType::Lightweight, "Lightweight split-leg", 2.8, 0.48,
        8.5, 3.8, 6.5, 3.3, 1.14}
}};
}

const std::array<HarnessProfile, 3>& GetHarnessProfiles()
{
    return Profiles;
}

const HarnessProfile& GetHarnessProfile(HarnessType type)
{
    for (const auto& profile : Profiles)
        if (profile.id == type) return profile;
    return Profiles[0];
}

double AllUpMassKg(const EquipmentSetup& setup, double wingMassKg)
{
    const auto& harness = GetHarnessProfile(setup.harness);
    return std::clamp(
        setup.pilotMassKg + harness.massKg + setup.reserveAndEquipmentKg
            + setup.ballastKg + wingMassKg,
        55.0, 160.0);
}

double WingLoadingKgM2(
    const EquipmentSetup& setup, double wingMassKg, double wingAreaM2)
{
    return AllUpMassKg(setup, wingMassKg) / std::max(10.0, wingAreaM2);
}

WingParameters ApplyEquipmentSetup(
    const WingParameters& base, const EquipmentSetup& setup, double wingMassKg)
{
    WingParameters result = base;
    result.allUpMassKg = AllUpMassKg(setup, wingMassKg);
    return result;
}

HarnessParameters HarnessParametersFor(const EquipmentSetup& setup)
{
    const auto& profile = GetHarnessProfile(setup.harness);
    return {
        profile.dragAreaM2,
        profile.pendulumRollStiffness,
        profile.pendulumRollDamping,
        profile.pendulumPitchStiffness,
        profile.pendulumPitchDamping,
        profile.weightShiftAuthority
    };
}
}
