#include "EquipmentSetup.h"

#include <algorithm>

namespace Parapenting::Physics
{
namespace
{
constexpr std::array<HarnessProfile, 3> Profiles{{
    // The last five columns are harness geometry, and they are what set
    // weight-shift authority now. A seatboard holds the pilot square to the
    // wing; a pod hangs them lower and wider, which is why it is the calmest
    // of the three; a split-leg lets the hips move furthest.
    {HarnessType::SeatedSeatboard, "Seated seatboard", 5.5, 0.42,
        7.5, 3.2, 5.8, 2.8,
        HarnessClass::SeatPlate, 0.44, 0.42, 0.28, 0.075},
    {HarnessType::Pod, "Pod harness", 7.0, 0.35,
        6.3, 2.6, 5.0, 2.3,
        HarnessClass::SeatPlate, 0.50, 0.46, 0.34, 0.068},
    {HarnessType::Lightweight, "Lightweight split-leg", 2.8, 0.48,
        8.5, 3.8, 6.5, 3.3,
        HarnessClass::NoPlate, 0.39, 0.40, 0.24, 0.082}
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
        profile.pendulumPitchDamping
    };
}

HarnessGeometry HarnessGeometryFor(const EquipmentSetup& setup)
{
    const auto& profile = GetHarnessProfile(setup.harness);
    HarnessGeometry geometry;
    geometry.harnessClass = profile.harnessClass;
    geometry.chestStrapM = profile.chestStrapM;
    geometry.carabinerSeparationM = profile.carabinerSeparationM;
    geometry.carabinerAboveCgM = profile.carabinerAboveCgM;
    geometry.hipTravelM = profile.hipTravelM;
    return geometry;
}

PayloadMassProperties PayloadMassFor(const EquipmentSetup& setup)
{
    const auto& profile = GetHarnessProfile(setup.harness);
    PayloadMassProperties mass;
    mass.pilotKg = setup.pilotMassKg;
    mass.harnessKg = profile.massKg;
    mass.ballastKg = setup.ballastKg;
    // The setup carries reserve and equipment as one figure; a reserve is
    // about a third of it.
    mass.reserveKg = 0.32 * setup.reserveAndEquipmentKg;
    mass.equipmentKg = 0.68 * setup.reserveAndEquipmentKg;
    return mass;
}
}
