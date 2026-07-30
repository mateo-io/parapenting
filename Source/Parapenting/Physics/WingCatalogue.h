#pragma once

#include "ParagliderDynamics.h"

#include <array>

namespace Parapenting::Physics
{
enum class WingProfileId
{
    TrainingA,
    AlpineAPlus,
    Epic2MLResearch,
    CrossCountryB,
    SportB,
    AdvanceEpsilonDls28Research
};

enum class WingSize
{
    Small,
    Medium,
    Large
};

enum class BrakeTravel
{
    Short,
    Standard,
    Long
};

struct WingSizeVariant
{
    WingSize id;
    const char* displayName;
    double areaScale;
    double massScale;
    double loadingRangeScale;
};

struct BrakeTravelVariant
{
    BrakeTravel id;
    const char* displayName;
    double travelMm;
    double controlGain;
    double freePlayFraction;
};

struct WingProfile
{
    WingProfileId id;
    const char* displayName;
    const char* className;
    WingParameters parameters;
    double wingMassKg = 5.0;
    double recommendedAllUpMinKg = 80.0;
    double recommendedAllUpMaxKg = 110.0;
    double brakeFreePlay = 0.10;
    double brakePressureExponent = 1.7;
    double brakeForceAtFullTravelN = 52.0;
    double targetTrimSpeedKmh = 0.0;
    double targetTopSpeedKmh = 0.0;
    double targetMinimumSinkMps = 0.0;
    double targetBestGlide = 0.0;
    const char* validationLevel = "generic research envelope";
    const char* sourceLabel = "simulator-authored";
    const char* dataPackagePath = "";
};

const WingProfile& GetWingProfile(WingProfileId id);
constexpr std::size_t WingProfileCount = 6;
const std::array<WingProfile, WingProfileCount>& GetWingProfiles();
const WingSizeVariant& GetWingSizeVariant(WingSize size);
const BrakeTravelVariant& GetBrakeTravelVariant(BrakeTravel travel);
WingParameters ConfigureWing(
    const WingProfile& profile, WingSize size, BrakeTravel travel);
double ConfiguredWingMassKg(const WingProfile& profile, WingSize size);
double ConfiguredRangeMinKg(const WingProfile& profile, WingSize size);
double ConfiguredRangeMaxKg(const WingProfile& profile, WingSize size);
}
