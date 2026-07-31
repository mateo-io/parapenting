#include "WingCatalogue.h"

namespace Parapenting::Physics
{
namespace
{
WingParameters TrainingParameters()
{
    WingParameters p;
    p.allUpMassKg = 95.0;
    p.areaM2 = 28.5;
    p.trimCl = 0.62;
    p.zeroLiftDrag = 0.035;
    p.inducedDragFactor = 0.105;
    p.brakeRollMoment = 190.0;
    p.brakeYawMoment = 120.0;
    p.maxLiftCoefficient = 1.42;
    p.loadSofteningOnsetG = 3.3;
    p.operationalLiftLimitG = 4.4;
    p.overspeedDragOnsetMps = 16.5;
    p.overspeedDragQuadratic = 0.0015;
    p.collapseResistance = 1.18;
    p.passiveReinflationRate = 0.24;
    p.brakeReinflationGain = 0.20;
    p.pumpReinflationGain = 1.15;
    p.frontalReinflationRate = 0.42;
    p.cravatSusceptibility = 0.65;
    p.recoverySurgeGain = 1.80;
    p.brakeLiftCurve = {0.0, 0.11, 0.25, 0.37, 0.43};
    p.brakeDragCurve = {0.0, 0.008, 0.082, 0.29, 0.68};
    return p;
}

WingParameters EpicParameters()
{
    return WingParameters{};
}

WingParameters AlpineParameters()
{
    WingParameters p;
    p.allUpMassKg = 100.0;
    p.areaM2 = 27.4;
    p.trimCl = 0.60;
    p.zeroLiftDrag = 0.032;
    p.inducedDragFactor = 0.096;
    p.brakeLiftGain = 0.44;
    p.brakeDragGain = 0.50;
    p.brakeRollMoment = 215.0;
    p.brakeYawMoment = 135.0;
    p.maxLiftCoefficient = 1.39;
    p.loadSofteningOnsetG = 3.45;
    p.operationalLiftLimitG = 4.55;
    p.overspeedDragOnsetMps = 17.2;
    p.overspeedDragQuadratic = 0.00135;
    p.rollDamping = 45.0;
    p.yawDamping = 51.0;
    p.collapseResistance = 1.10;
    p.passiveReinflationRate = 0.21;
    p.brakeReinflationGain = 0.18;
    p.pumpReinflationGain = 1.12;
    p.frontalReinflationRate = 0.38;
    p.cravatSusceptibility = 0.78;
    p.recoverySurgeGain = 2.0;
    p.brakeLiftCurve = {0.0, 0.105, 0.25, 0.38, 0.44};
    p.brakeDragCurve = {0.0, 0.007, 0.078, 0.27, 0.65};
    return p;
}

WingParameters CrossCountryParameters()
{
    WingParameters p;
    p.allUpMassKg = 105.0;
    p.areaM2 = 26.2;
    p.trimCl = 0.555;
    p.zeroLiftDrag = 0.0275;
    p.inducedDragFactor = 0.081;
    p.brakeLiftGain = 0.39;
    p.brakeDragGain = 0.43;
    p.brakeRollMoment = 255.0;
    p.brakeYawMoment = 162.0;
    p.maxLiftCoefficient = 1.33;
    p.loadSofteningOnsetG = 3.8;
    p.operationalLiftLimitG = 4.7;
    p.overspeedDragOnsetMps = 18.8;
    p.overspeedDragQuadratic = 0.0010;
    p.rollDamping = 39.0;
    p.pitchDamping = 88.0;
    p.yawDamping = 44.0;
    p.acceleratorLiftReduction = 0.19;
    p.acceleratorDragReduction = 0.008;
    p.acceleratorPitchMoment = 48.0;
    p.collapseResistance = 0.90;
    p.passiveReinflationRate = 0.14;
    p.brakeReinflationGain = 0.15;
    p.pumpReinflationGain = 0.96;
    p.frontalReinflationRate = 0.27;
    p.cravatSusceptibility = 1.18;
    p.recoverySurgeGain = 2.70;
    p.brakeLiftCurve = {0.0, 0.080, 0.205, 0.325, 0.39};
    p.brakeDragCurve = {0.0, 0.004, 0.060, 0.215, 0.54};
    return p;
}

WingParameters SportParameters()
{
    WingParameters p;
    p.allUpMassKg = 105.0;
    p.areaM2 = 25.5;
    p.trimCl = 0.54;
    p.zeroLiftDrag = 0.026;
    p.inducedDragFactor = 0.075;
    p.brakeRollMoment = 275.0;
    p.brakeYawMoment = 175.0;
    p.maxLiftCoefficient = 1.32;
    p.loadSofteningOnsetG = 3.95;
    p.operationalLiftLimitG = 4.8;
    p.overspeedDragOnsetMps = 19.5;
    p.overspeedDragQuadratic = 0.0009;
    p.rollDamping = 36.0;
    p.yawDamping = 41.0;
    p.collapseResistance = 0.82;
    p.passiveReinflationRate = 0.12;
    p.brakeReinflationGain = 0.14;
    p.pumpReinflationGain = 0.90;
    p.frontalReinflationRate = 0.24;
    p.cravatSusceptibility = 1.35;
    p.recoverySurgeGain = 3.0;
    p.brakeLiftCurve = {0.0, 0.072, 0.19, 0.31, 0.37};
    p.brakeDragCurve = {0.0, 0.003, 0.054, 0.195, 0.50};
    return p;
}

WingParameters EpsilonDls28Parameters()
{
    WingParameters p;
    p.allUpMassKg = 103.35;
    p.areaM2 = 27.6;
    p.trimCl = 0.595;
    p.zeroLiftDrag = 0.0305;
    p.inducedDragFactor = 0.091;
    p.brakeLiftGain = 0.43;
    p.brakeDragGain = 0.51;
    p.brakeRollMoment = 220.0;
    p.brakeYawMoment = 140.0;
    p.maxLiftCoefficient = 1.40;
    p.loadSofteningOnsetG = 3.55;
    p.operationalLiftLimitG = 4.6;
    p.overspeedDragOnsetMps = 17.8;
    p.overspeedDragQuadratic = 0.0012;
    p.rollDamping = 46.0;
    p.pitchDamping = 96.0;
    p.yawDamping = 52.0;
    p.acceleratorLiftReduction = 0.17;
    p.acceleratorDragReduction = 0.006;
    p.acceleratorPitchMoment = 43.0;
    p.collapseResistance = 1.12;
    p.passiveReinflationRate = 0.22;
    p.brakeReinflationGain = 0.18;
    p.pumpReinflationGain = 1.12;
    p.frontalReinflationRate = 0.39;
    p.cravatSusceptibility = 0.74;
    p.recoverySurgeGain = 1.95;
    p.brakeLiftCurve = {0.0, 0.10, 0.24, 0.36, 0.43};
    p.brakeDragCurve = {0.0, 0.006, 0.074, 0.255, 0.62};
    return p;
}

const std::array<WingProfile, WingProfileCount> Profiles{{
    {WingProfileId::TrainingA, "Training A 28", "EN A", TrainingParameters(),
        5.2, 75.0, 105.0, 0.08, 1.55, 42.0,
        37.0, 45.0, 1.15, 8.0, "generic class envelope"},
    {WingProfileId::AlpineAPlus, "Alpine A+ 27 (research)", "EN A+ inspired",
        AlpineParameters(), 4.9, 80.0, 108.0, 0.09, 1.65, 47.0,
        38.0, 48.0, 1.08, 8.6, "generic class envelope"},
    {WingProfileId::Epic2MLResearch, "BGD EPIC 2 ML (research)", "low EN B",
        EpicParameters(), 5.0, 90.0, 110.0, 0.10, 1.75, 52.0,
        39.0, 53.0, 1.0, 9.5, "published envelope; handling unvalidated",
        "BGD product page and owner's manual",
        "Data/Wings/bgd-epic-2-ml-research.json"},
    {WingProfileId::CrossCountryB, "XC B 26 (research)", "mid EN B inspired",
        CrossCountryParameters(), 4.8, 88.0, 113.0, 0.11, 1.85, 58.0,
        40.0, 55.0, 0.95, 10.2, "generic class envelope"},
    {WingProfileId::SportB, "Sport B 26", "high EN B", SportParameters(),
        4.7, 90.0, 115.0, 0.12, 1.95, 64.0,
        41.0, 57.0, 0.92, 10.8, "generic class envelope"},
    {WingProfileId::AdvanceEpsilonDls28Research,
        "ADVANCE EPSILON DLS 28 (research)", "basic intermediate EN/LTF B",
        EpsilonDls28Parameters(), 4.35, 91.0, 118.0, 0.11, 1.78, 54.0,
        38.5, 52.0, 1.02, 9.4,
        "published geometry/loading; handling and polar unvalidated",
        "ADVANCE official EPSILON DLS product data",
        "Data/Wings/advance-epsilon-dls-28-research.json"}
}};

constexpr std::array<WingSizeVariant, 3> Sizes{{
    {WingSize::Small, "S", 0.90, 0.94, 0.90},
    {WingSize::Medium, "M", 1.00, 1.00, 1.00},
    {WingSize::Large, "L", 1.11, 1.07, 1.11}
}};

constexpr std::array<BrakeTravelVariant, 3> BrakeTravels{{
    {BrakeTravel::Short, "SHORT", 520.0, 1.12, 0.07},
    {BrakeTravel::Standard, "STANDARD", 620.0, 1.00, 0.10},
    {BrakeTravel::Long, "LONG", 720.0, 0.88, 0.14}
}};
}

const std::array<WingProfile, WingProfileCount>& GetWingProfiles()
{
    return Profiles;
}

const WingProfile& GetWingProfile(WingProfileId id)
{
    for (const WingProfile& profile : Profiles)
        if (profile.id == id) return profile;
    return Profiles[1];
}

const WingSizeVariant& GetWingSizeVariant(WingSize size)
{
    return Sizes[static_cast<std::size_t>(size)];
}

const BrakeTravelVariant& GetBrakeTravelVariant(BrakeTravel travel)
{
    return BrakeTravels[static_cast<std::size_t>(travel)];
}

WingParameters ConfigureWing(
    const WingProfile& profile, WingSize size, BrakeTravel travel)
{
    const auto& variant = GetWingSizeVariant(size);
    const auto& brake = GetBrakeTravelVariant(travel);
    WingParameters result = profile.parameters;
    result.areaM2 *= variant.areaScale;
    const double inertiaScale = variant.areaScale * variant.areaScale;
    result.rollInertiaKgM2 *= inertiaScale;
    result.pitchInertiaKgM2 *= inertiaScale;
    result.yawInertiaKgM2 *= inertiaScale;
    result.brakeLiftGain *= brake.controlGain;
    result.brakeDragGain *= brake.controlGain;
    for (double& value : result.brakeLiftCurve) value *= brake.controlGain;
    for (double& value : result.brakeDragCurve) value *= brake.controlGain;
    result.brakeRollMoment *= brake.controlGain * variant.areaScale;
    result.brakeYawMoment *= brake.controlGain * variant.areaScale;
    result.brakeTravelMm = brake.travelMm;
    result.brakeFreePlayFraction = brake.freePlayFraction;
    result.brakePressureExponent = profile.brakePressureExponent;
    result.brakeForceAtFullTravelN =
        profile.brakeForceAtFullTravelN * brake.controlGain;
    return result;
}

double ConfiguredWingMassKg(const WingProfile& profile, WingSize size)
{
    return profile.wingMassKg * GetWingSizeVariant(size).massScale;
}

double ConfiguredRangeMinKg(const WingProfile& profile, WingSize size)
{
    return profile.recommendedAllUpMinKg
        * GetWingSizeVariant(size).loadingRangeScale;
}

double ConfiguredRangeMaxKg(const WingProfile& profile, WingSize size)
{
    return profile.recommendedAllUpMaxKg
        * GetWingSizeVariant(size).loadingRangeScale;
}
}
