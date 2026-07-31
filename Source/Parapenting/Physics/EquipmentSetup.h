#pragma once

#include "HarnessGeometry.h"
#include "ParagliderDynamics.h"

#include <array>

namespace Parapenting::Physics
{
enum class HarnessType
{
    SeatedSeatboard,
    Pod,
    Lightweight
};

struct HarnessProfile
{
    HarnessType id;
    const char* displayName;
    double massKg;
    double dragAreaM2;
    double pendulumRollStiffness;
    double pendulumRollDamping;
    double pendulumPitchStiffness;
    double pendulumPitchDamping;
    // Level 3: the geometry that decides weight-shift authority. There is no
    // authority multiplier any more - a harness earns its responsiveness by
    // how far it lets the pilot's mass move.
    HarnessClass harnessClass;
    double chestStrapM;
    double carabinerSeparationM;
    double carabinerAboveCgM;
    double hipTravelM;
};

struct EquipmentSetup
{
    double pilotMassKg = 85.0;
    double reserveAndEquipmentKg = 8.5;
    double ballastKg = 0.0;
    HarnessType harness = HarnessType::SeatedSeatboard;
};

const HarnessProfile& GetHarnessProfile(HarnessType type);
const std::array<HarnessProfile, 3>& GetHarnessProfiles();
double AllUpMassKg(const EquipmentSetup& setup, double wingMassKg);
double WingLoadingKgM2(
    const EquipmentSetup& setup, double wingMassKg, double wingAreaM2);
WingParameters ApplyEquipmentSetup(
    const WingParameters& base, const EquipmentSetup& setup, double wingMassKg);
HarnessParameters HarnessParametersFor(const EquipmentSetup& setup);
HarnessGeometry HarnessGeometryFor(const EquipmentSetup& setup);
PayloadMassProperties PayloadMassFor(const EquipmentSetup& setup);
}
