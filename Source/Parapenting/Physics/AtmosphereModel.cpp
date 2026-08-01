#include "AtmosphereModel.h"
#include "RouteFrame.h"
#include "TerrainModel.h"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace Parapenting::Physics
{
namespace
{
constexpr WeatherVolume EmptyVolume{
    WeatherVolumeType::Thermal, {}, 0.0, 0.0, 0.0, 0.0};

// A convection trigger: a place on the ground that reliably sets off a
// thermal, in local route-frame metres.
struct ThermalTrigger
{
    double xM;
    double yM;
    double radiusM;
    double strengthScale;
};

struct ThermalTriggerSet
{
    std::array<ThermalTrigger, 3> triggers;
    // Authored weather volumes in every preset are placed in Interlaken
    // coordinates, relative to the Amisbuehl launch. A region away from that
    // corridor shifts them onto its own, by the offset from Amisbuehl to its
    // own launch. Both components matter: offsetting only Y left every
    // authored volume at x = 760..2520 while Grindelwald starts at x = 4500,
    // so a foehn day out there was smooth air.
    double authoredVolumeOffsetXM;
    double authoredVolumeOffsetYM;
    // Z as well: authored centres are heights in the Lehn datum chosen for
    // the Interlaken valley floor, and Grindelwald's floor is 800 m higher, so
    // an unshifted volume sits underground there.
    double authoredVolumeOffsetZM;
};

// Interlaken: unchanged from the three cells this model has always had, over
// the sunny valley shoulder between the launches and the landing fields.
constexpr ThermalTriggerSet InterlakenTriggers{
    {{{1280.0, 330.0, 170.0, 1.0},
      {1880.0, -470.0, 230.0, 0.78},
      {2260.0, 560.0, 145.0, 1.12}}},
    0.0, 0.0, 0.0};

// Grindelwald: along the First -> Grund descent, on the south-facing flank
// that gets the sun first. The valley runs from the First launch at
// (5941, -17481) down to the Grund field at (9962, -15281).
constexpr ThermalTriggerSet GrindelwaldTriggers{
    {{{6900.0, -16900.0, 190.0, 1.0},
      {7900.0, -16300.0, 240.0, 0.82},
      {8900.0, -15800.0, 165.0, 1.08}}},
    5940.8, -17480.6, 798.0};

const ThermalTriggerSet& TriggersFor(double xM, double yM)
{
    const RouteFrame::SurveyedRegion* region = RouteFrame::RegionAt(xM, yM);
    if (region != nullptr && std::string_view(region->name) == "grindelwald")
        return GrindelwaldTriggers;
    return InterlakenTriggers;
}

const std::array<WeatherPreset, 5> Presets{{
    {
        WeatherPresetId::MorningCalm,
        "STILL AIR LAB",
        {WeatherMode::Chill, {}, 0.0, 0.0, 0.0, 1500.0, 0.0},
        {EmptyVolume, EmptyVolume, EmptyVolume, EmptyVolume, EmptyVolume},
        8.5
    },
    {
        WeatherPresetId::ValleyBreeze,
        "VALLEY BREEZE",
        {WeatherMode::Ridge, {-2.4, 0.3, 0.0}, 0.12, 0.75, 1.25, 2050.0, 0.48},
        {{
            {WeatherVolumeType::Thermal, {1180.0, 280.0, 470.0},
                210.0, 800.0, 0.8, 0.12},
            {WeatherVolumeType::Thermal, {2050.0, -510.0, 390.0},
                260.0, 720.0, 0.65, 0.10},
            EmptyVolume, EmptyVolume, EmptyVolume
        }},
        11.5
    },
    {
        WeatherPresetId::ThermalDay,
        "ACTIVE THERMAL DAY",
        {WeatherMode::Ridge, {-1.2, -0.2, 0.0}, 0.24, 1.85, 1.55, 2450.0, 0.92},
        {{
            {WeatherVolumeType::Thermal, {920.0, -260.0, 520.0},
                190.0, 1050.0, 2.2, 0.28},
            {WeatherVolumeType::Thermal, {1660.0, 430.0, 560.0},
                240.0, 1100.0, 2.7, 0.32},
            {WeatherVolumeType::Thermal, {2460.0, -180.0, 430.0},
                170.0, 900.0, 1.9, 0.24},
            {WeatherVolumeType::Sink, {1660.0, 430.0, 560.0},
                520.0, 1100.0, 0.65, 0.16},
            EmptyVolume
        }},
        14.0
    },
    {
        WeatherPresetId::FoehnRotor,
        "FOEHN / STRONG ROTOR",
        {WeatherMode::LocalizedRotor, {1.0, -7.5, 0.0}, 0.52, 0.45, 2.4, 1850.0, 0.12},
        {{
            {WeatherVolumeType::Rotor, {760.0, 760.0, 260.0},
                520.0, 520.0, 0.82, 0.75},
            {WeatherVolumeType::Rotor, {1580.0, -690.0, 230.0},
                610.0, 470.0, 0.92, 0.85},
            {WeatherVolumeType::Rotor, {2520.0, 820.0, 180.0},
                540.0, 420.0, 0.72, 0.68},
            {WeatherVolumeType::Sink, {1900.0, 0.0, 400.0},
                680.0, 600.0, 1.2, 0.45},
            EmptyVolume
        }},
        14.0
    },
    {
        WeatherPresetId::EveningDrainage,
        "EVENING DRAINAGE",
        {WeatherMode::Ridge, {0.35, 0.1, 0.0},
            0.07, 0.05, 0.18, 1450.0, 0.72},
        {EmptyVolume, EmptyVolume, EmptyVolume, EmptyVolume, EmptyVolume},
        21.0
    }
}};

double SmoothPulse(double value, double centre, double halfWidth)
{
    const double distance = std::abs(value - centre) / std::max(halfWidth, 1e-6);
    if (distance >= 1.0) return 0.0;
    const double t = 1.0 - distance;
    return t * t * (3.0 - 2.0 * t);
}

double SmoothStep01(double value)
{
    const double t = std::clamp(value, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double ThermalLifecycle(
    double timeSeconds, double originX, double originY)
{
    const double cyclePeriod = 205.0 + std::fmod(originX * 0.071, 47.0);
    const double phaseOffset = std::fmod(
        originX * 0.113 + std::abs(originY) * 0.067, cyclePeriod);
    const double cycleTime = std::fmod(
        std::max(0.0, timeSeconds) + phaseOffset, cyclePeriod);
    const double phase = cycleTime / cyclePeriod;
    return SmoothStep01(phase / 0.18)
        * (1.0 - SmoothStep01((phase - 0.72) / 0.28));
}

struct GustSpectrum
{
    Vec3 velocityMps{};
    double lowBandMps = 0.0;
    double highBandMps = 0.0;
};

GustSpectrum SampleGustSpectrum(
    const Vec3& p, double t, const Vec3& advectionWind,
    double baseAmplitudeMps, double rotorAmplitudeMps)
{
    // Three frozen-field spatial bands are advected by the model wind and
    // evolve slowly on their own. The shortest wavelength/frequency remains
    // far below the 60 Hz Nyquist limit of the maximum accepted physics step.
    const Vec3 q = p - advectionWind * t;
    const double largeAmplitude =
        baseAmplitudeMps * 0.50 + rotorAmplitudeMps * 0.30;
    const double mediumAmplitude =
        baseAmplitudeMps * 0.33 + rotorAmplitudeMps * 0.43;
    const double smallAmplitude =
        baseAmplitudeMps * 0.17 + rotorAmplitudeMps * 0.27;
    const Vec3 large{
        largeAmplitude * std::sin(
            0.010 * q.x + 0.006 * q.y + 0.19 * t),
        largeAmplitude * std::sin(
            -0.008 * q.x + 0.011 * q.y + 0.004 * q.z + 0.27 * t + 1.1),
        largeAmplitude * 0.72 * std::sin(
            0.007 * q.x - 0.009 * q.y + 0.006 * q.z + 0.23 * t + 2.0)
    };
    const Vec3 medium{
        mediumAmplitude * std::sin(
            0.039 * q.x - 0.028 * q.y + 0.73 * t + 0.4),
        mediumAmplitude * std::sin(
            0.031 * q.y + 0.022 * q.z + 0.91 * t + 2.4),
        mediumAmplitude * 0.88 * std::sin(
            -0.035 * q.x + 0.027 * q.y - 0.019 * q.z + 0.81 * t)
    };
    const Vec3 small{
        smallAmplitude * std::sin(
            0.121 * q.x + 0.087 * q.y + 1.71 * t + 0.8),
        smallAmplitude * std::sin(
            -0.096 * q.x + 0.133 * q.y - 0.061 * q.z + 2.13 * t),
        smallAmplitude * std::sin(
            0.108 * q.x - 0.079 * q.y + 0.094 * q.z + 1.93 * t + 1.7)
    };
    return {large + medium + small, Length(large), Length(small)};
}

double ThermalCell(const Vec3& p, double t, const Vec3& baseWind,
                   double originX, double originY, double radius,
                   double strength, double thermalTopMslM,
                   Atmosphere& result)
{
    // A ground source repeatedly releases finite-lived plumes. Re-using the
    // same deterministic cycle makes training/replay repeatable while avoiding
    // the permanently-on Gaussian columns used by the early prototype.
    const double cyclePeriod = 205.0 + std::fmod(originX * 0.071, 47.0);
    const double phaseOffset = std::fmod(
        originX * 0.113 + std::abs(originY) * 0.067, cyclePeriod);
    const double cycleTime = std::fmod(
        std::max(0.0, t) + phaseOffset, cyclePeriod);
    const double phase = cycleTime / cyclePeriod;
    const double lifecycle = ThermalLifecycle(t, originX, originY);

    const double ground = TerrainModel::HeightM(p.x, p.y);
    const double agl = std::max(0.0, p.z - ground);
    // Older air higher in the plume has been advected longer. A small
    // deterministic meander prevents a perfectly straight, game-like column.
    const double plumeAge = cycleTime * 0.28 + agl / 2.7;
    const double meander = std::sin(
        t * 0.031 + agl * 0.0047 + originX * 0.013);
    const double driftX = baseWind.x * plumeAge * 0.34
        - baseWind.y * meander * 9.0;
    const double driftY = baseWind.y * plumeAge * 0.34
        + baseWind.x * meander * 9.0;
    const double dx = p.x - originX - driftX;
    const double dy = p.y - originY - driftY;
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double expandedRadius = radius * (0.72 + 0.00042 * agl);
    const double breathing = 0.84 + 0.16
        * std::sin(t * 0.19 + originX * 0.01 + agl * 0.006);
    const double core = std::exp(
        -0.5 * std::pow(distance / expandedRadius, 2.0));
    const double ring = std::exp(-0.5 * std::pow(
        (distance - expandedRadius * 1.72) / (expandedRadius * 0.40), 2.0));
    const double cloudBaseClearance = thermalTopMslM - p.z;
    const double baseFade = SmoothStep01(agl / 75.0);
    // The inversion caps buoyant air over roughly 240 m. Immediately below
    // cloud base, latent heating/entrainment broadens and slightly strengthens
    // an active core without creating unbounded "cloud suck".
    const double topFade = 1.0 - SmoothStep01(
        (p.z - (thermalTopMslM - 240.0)) / 300.0);
    const double cloudBaseBoost = 1.0 + 0.16 * lifecycle
        * SmoothPulse(cloudBaseClearance, 170.0, 170.0);
    const double verticalShape = baseFade * topFade;
    const double lift =
        strength * core * breathing * verticalShape * lifecycle
        * cloudBaseBoost;
    const double sink =
        strength * 0.30 * ring * verticalShape * (0.35 + 0.65 * lifecycle);

    // A toroidal circulation feeds the column: convergence is strongest low
    // down and becomes weak outflow near cloud base. Edge vorticity alternates
    // direction over a plume cycle and provides bank/yaw texture without
    // turning every thermal into a permanent vertical-axis vortex.
    if (distance > 1.0)
    {
        const double radialDirection = cloudBaseClearance < 260.0
            ? 0.35 : -1.0;
        const double radialFlow = radialDirection * strength * 0.20
            * core * lifecycle
            * (cloudBaseClearance < 260.0
                ? SmoothStep01((260.0 - cloudBaseClearance) / 220.0)
                : std::exp(-agl / 650.0));
        const double edge = std::exp(-0.5 * std::pow(
            (distance - expandedRadius) / (expandedRadius * 0.42), 2.0));
        const double circulationSign =
            std::sin(originX * 0.019 + originY * 0.011) >= 0.0 ? 1.0 : -1.0;
        const double tangentialFlow = circulationSign * strength * 0.075
            * edge * lifecycle
            * std::sin(phase * 6.283185307179586 + agl * 0.003);
        result.windWorldMps.x += dx / distance * radialFlow
            - dy / distance * tangentialFlow;
        result.windWorldMps.y += dy / distance * radialFlow
            + dx / distance * tangentialFlow;
    }
    result.thermalLiftMps += lift;
    result.sinkRingMps += sink;
    if (lift > result.thermalCoreStrength)
    {
        result.thermalCoreStrength = lift;
        result.thermalLifecycle = lifecycle;
    }
    result.cloudBaseClearanceM = std::min(
        result.cloudBaseClearanceM, cloudBaseClearance);
    result.turbulence = std::clamp(
        result.turbulence + (0.08 * core + 0.24 * ring)
            * verticalShape * (0.4 + 0.6 * lifecycle),
        0.0, 1.0);
    return lift - sink;
}
}

TerrainCirculation EvaluateTerrainCirculation(
    const Vec3& terrainNormal, double aboveGroundM, double surfaceHeating)
{
    const double normalZ = std::max(0.15, std::abs(terrainNormal.z));
    const double grade = std::hypot(
        terrainNormal.x, terrainNormal.y) / normalZ;
    if (grade < 0.015 || std::abs(surfaceHeating) < 0.001)
        return {};

    // A terrain normal points away from the surface; its negative horizontal
    // components are the local uphill direction. Signed heating reverses this
    // into downslope drainage without requiring a separate authored vector.
    const double horizontalNormal =
        std::hypot(terrainNormal.x, terrainNormal.y);
    const double directionSign = surfaceHeating >= 0.0 ? 1.0 : -1.0;
    const Vec3 uphill{
        -terrainNormal.x / horizontalNormal,
        -terrainNormal.y / horizontalNormal,
        0.0
    };
    const double slopeResponse = SmoothStep01(grade / 0.55);
    const double surfaceFade =
        std::exp(-std::max(0.0, aboveGroundM) / 145.0);
    const double alongSlope = directionSign
        * 1.65 * std::sqrt(std::abs(surfaceHeating))
        * slopeResponse * surfaceFade;
    // Anabatic air has a restrained vertical component following the terrain;
    // katabatic drainage remains slightly terrain-following/downward.
    const double vertical = alongSlope
        * std::clamp(grade * 0.28, 0.0, 0.32);
    return {
        uphill * alongSlope + Vec3{0.0, 0.0, vertical},
        alongSlope
    };
}

AtmosphereModel::AtmosphereModel(WeatherParameters parameters)
    : Params(parameters)
{
}

const std::array<WeatherPreset, 5>& GetWeatherPresets()
{
    return Presets;
}

const WeatherPreset& GetWeatherPreset(WeatherPresetId id)
{
    for (const auto& preset : Presets)
        if (preset.id == id) return preset;
    return Presets[1];
}

void AtmosphereModel::SetPreset(WeatherPresetId id)
{
    const auto& preset = GetWeatherPreset(id);
    Params = preset.parameters;
    Volumes = preset.volumes;
    PresetId = preset.id;
    StartLocalHour = preset.startLocalHour;
    Snapshot = {
        "authored-preset", preset.displayName, 0.0,
        MeteorologicalDirectionFromWindVector(Params.baseWindMps),
        std::hypot(Params.baseWindMps.x, Params.baseWindMps.y),
        std::hypot(Params.baseWindMps.x, Params.baseWindMps.y)
            + Params.turbulence * 4.0,
        Params.thermalTopMslM,
        static_cast<unsigned int>(id) + 1
    };
}

void AtmosphereModel::ApplySnapshot(const WeatherSnapshot& snapshot)
{
    Snapshot = snapshot;
    Params.baseWindMps = WindVectorFromMeteorological(
        snapshot.windFromDegrees, snapshot.windSpeedMps);
    const double gustDelta = std::max(
        0.0, snapshot.gustSpeedMps - snapshot.windSpeedMps);
    Params.turbulence = std::clamp(0.04 + gustDelta / 8.0, 0.02, 0.8);
    Params.thermalStrengthMps = snapshot.thermalTopMslM > 1400.0
        ? std::clamp((snapshot.thermalTopMslM - 1100.0) / 500.0, 0.3, 2.4)
        : 0.25;
    Params.thermalTopMslM = std::clamp(
        snapshot.thermalTopMslM, 1200.0, 4200.0);
    Params.ridgeLiftStrengthMps = 1.2;
    Params.surfaceHeating = std::clamp(
        Params.thermalStrengthMps / 2.4, 0.08, 0.85);
    Params.mode = gustDelta > 4.5
        ? WeatherMode::LocalizedRotor
        : (snapshot.windSpeedMps < 0.8
            ? WeatherMode::Chill : WeatherMode::Ridge);
    Volumes = {};
    PresetId = WeatherPresetId::Custom;
}

Atmosphere AtmosphereModel::Sample(const Vec3& p, double timeSeconds) const
{
    Atmosphere result;
    result.windWorldMps = Params.baseWindMps;
    const double ground = TerrainModel::HeightM(p.x, p.y);
    const double aboveGround = std::max(0.0, p.z - ground);
    result.groundClearanceM = aboveGround;

    if (Params.mode == WeatherMode::Chill)
    {
        result.turbulence = 0.02;
        return result;
    }

    const Vec3 normal = TerrainModel::Normal(p.x, p.y);
    const DiurnalState diurnal =
        EvaluateDiurnalCycle(StartLocalHour, timeSeconds);

    // Terrain boundary layer: wind slows near the surface while directional
    // shear and exposed slopes create mechanical turbulence.
    const double boundaryLayer = 0.34 + 0.66
        * (1.0 - std::exp(-aboveGround / 95.0));
    result.windWorldMps.x *= boundaryLayer;
    result.windWorldMps.y *= boundaryLayer;
    const double shear = (1.0 - boundaryLayer)
        * std::hypot(Params.baseWindMps.x, Params.baseWindMps.y);
    result.turbulence = std::clamp(
        Params.turbulence
            + TerrainModel::RidgeExposure(p.x, p.y) * shear * 0.055,
        0.0, 1.0);

    const TerrainCirculation circulation = EvaluateTerrainCirculation(
        normal, aboveGround,
        Params.surfaceHeating * diurnal.surfaceHeating);
    result.windWorldMps += circulation.velocityWorldMps;
    result.slopeFlowMps = circulation.alongSlopeMps;

    const double windIntoSlope = std::max(0.0,
        -(result.windWorldMps.x * normal.x + result.windWorldMps.y * normal.y));
    const double terrainLiftFade = std::exp(-aboveGround / 260.0);
    result.windWorldMps.z += Params.ridgeLiftStrengthMps
                          * windIntoSlope * terrainLiftFade;

    // Thermal triggers sit in the valley being flown. This used to be a single
    // Interlaken set plus a `p.y > 5000 ? 7500 : 0` translation, which was a
    // lane offset rather than a place; anywhere it did not reach - Grindelwald
    // above all - was dead air with no thermals at any time of day. Each
    // surveyed region now carries its own triggers, anchored on its own
    // corridor, and the offsets below keep Interlaken's three exactly where
    // they were.
    const ThermalTriggerSet& triggers = TriggersFor(p.x, p.y);
    const double effectiveThermalStrength =
        Params.thermalStrengthMps
        * (0.08 + 0.92 * diurnal.convectiveActivity);
    for (const ThermalTrigger& trigger : triggers.triggers)
    {
        if (trigger.radiusM <= 0.0) continue;
        result.windWorldMps.z += ThermalCell(
            p, timeSeconds, Params.baseWindMps, trigger.xM, trigger.yM,
            trigger.radiusM, effectiveThermalStrength * trigger.strengthScale,
            Params.thermalTopMslM, result);
    }
    const double regionalX = triggers.authoredVolumeOffsetXM;
    const double regionalY = triggers.authoredVolumeOffsetYM;
    const double regionalZ = triggers.authoredVolumeOffsetZM;

    double authoredRotor = 0.0;
    for (const WeatherVolume& volume : Volumes)
    {
        if (volume.radiusM <= 0.0 || volume.heightM <= 0.0) continue;
        const double dx = p.x - (volume.centreWorldM.x + regionalX);
        const double dy = p.y
            - (volume.centreWorldM.y + regionalY);
        const double horizontal =
            std::sqrt(dx * dx + dy * dy) / volume.radiusM;
        const double vertical =
            std::abs(p.z - (volume.centreWorldM.z + regionalZ))
            / volume.heightM;
        if (horizontal >= 1.0 || vertical >= 1.0) continue;
        const double radial = 1.0 - horizontal;
        const double verticalFade = 1.0 - vertical;
        const double influence =
            radial * radial * (3.0 - 2.0 * radial) * verticalFade;
        result.turbulence = std::clamp(
            result.turbulence + volume.turbulence * influence, 0.0, 1.0);
        if (volume.type == WeatherVolumeType::Thermal)
        {
            const double lift = volume.strength * influence
                * diurnal.convectiveActivity;
            result.windWorldMps.z += lift;
            result.thermalLiftMps += lift;
        }
        else if (volume.type == WeatherVolumeType::Sink)
        {
            const double sink = volume.strength * influence
                * (0.15 + 0.85 * diurnal.convectiveActivity);
            result.windWorldMps.z -= sink;
            result.sinkRingMps += sink;
        }
        else
        {
            authoredRotor = std::max(
                authoredRotor, volume.strength * influence);
        }
    }

    double rotor = 0.0;
    if (Params.mode == WeatherMode::LocalizedRotor)
    {
        // Keep the official departure corridor flyable in normal conditions;
        // the strongest broken air lives in the lee pockets beside the route.
        const double leePocket = 0.12 + 0.88
            * SmoothPulse(std::abs(p.y), 720.0, 660.0);
        rotor = TerrainModel::LeeRotorPotential(
            p.x, p.y, result.windWorldMps)
              * std::exp(-aboveGround / 190.0) * leePocket;
    }
    else if (Params.mode == WeatherMode::RotorEverywhere)
    {
        rotor = 0.72;
    }

    rotor = std::clamp(std::max(rotor, authoredRotor), 0.0, 1.0);
    result.turbulence = std::clamp(
        std::max(result.turbulence, Params.turbulence) + rotor * 0.82,
        0.0, 1.0);
    const double boundaryEnergy =
        0.45 + 0.55 * std::exp(-aboveGround / 520.0);
    const GustSpectrum gust = SampleGustSpectrum(
        p, timeSeconds, Params.baseWindMps,
        result.turbulence * boundaryEnergy * 1.45,
        rotor * 3.0);
    result.windWorldMps += gust.velocityMps;
    result.rotorStrength = rotor;
    result.lateralGust = gust.velocityMps.y;
    result.lowFrequencyGustMps = gust.lowBandMps;
    result.highFrequencyGustMps = gust.highBandMps;
    result.gustEnergyMps = Length(gust.velocityMps);
    return result;
}

Atmosphere AtmosphereModel::SampleCanopy(
    const Vec3& centreWorldM, const Vec3& rawWingRightWorld,
    double halfSpanM, double timeSeconds) const
{
    Atmosphere centre = Sample(centreWorldM, timeSeconds);
    const Vec3 wingRight = Normalized(rawWingRightWorld);
    const double span = std::clamp(halfSpanM, 2.0, 8.0);
    const Atmosphere left =
        Sample(centreWorldM - wingRight * span, timeSeconds);
    const Atmosphere right =
        Sample(centreWorldM + wingRight * span, timeSeconds);
    centre.leftWingWindDeltaMps =
        left.windWorldMps - centre.windWorldMps;
    centre.rightWingWindDeltaMps =
        right.windWorldMps - centre.windWorldMps;
    centre.spanwiseAirflowShearMps =
        Length(right.windWorldMps - left.windWorldMps);
    // A half-wing entering a sharp boundary also exposes the whole flexible
    // structure to the stronger local turbulence/rotor value.
    centre.turbulence = std::max(
        centre.turbulence, std::max(left.turbulence, right.turbulence));
    centre.rotorStrength = std::max(
        centre.rotorStrength, std::max(left.rotorStrength, right.rotorStrength));
    centre.lowFrequencyGustMps = std::max(
        centre.lowFrequencyGustMps,
        std::max(left.lowFrequencyGustMps, right.lowFrequencyGustMps));
    centre.highFrequencyGustMps = std::max(
        centre.highFrequencyGustMps,
        std::max(left.highFrequencyGustMps, right.highFrequencyGustMps));
    centre.gustEnergyMps = std::max(
        centre.gustEnergyMps,
        std::max(left.gustEnergyMps, right.gustEnergyMps));
    return centre;
}

CloudFieldState AtmosphereModel::SampleCloudField(double timeSeconds) const
{
    if (Params.mode == WeatherMode::Chill
        || Params.thermalStrengthMps <= 0.05)
    {
        return {
            Params.thermalTopMslM, 180.0, 0.025, 0.0, 0.04,
            Params.baseWindMps * timeSeconds
        };
    }

    const std::array<Vec3, 3> sources{{
        {1280.0, 330.0, 0.0},
        {1880.0, -470.0, 0.0},
        {2260.0, 560.0, 0.0}
    }};
    double lifecycleSum = 0.0;
    double strongestLifecycle = 0.0;
    for (const Vec3& source : sources)
    {
        const double lifecycle = ThermalLifecycle(
            timeSeconds, source.x, source.y);
        lifecycleSum += lifecycle;
        strongestLifecycle = std::max(strongestLifecycle, lifecycle);
    }
    const double meanLifecycle = lifecycleSum / sources.size();
    const DiurnalState diurnal =
        EvaluateDiurnalCycle(StartLocalHour, timeSeconds);
    const double activity = std::clamp(
        Params.thermalStrengthMps / 2.7, 0.0, 1.0)
        * diurnal.convectiveActivity;
    const double coverage = std::clamp(
        0.06 + activity * (0.16 + 0.34 * meanLifecycle)
            + Params.turbulence * 0.10,
        0.03, 0.72);
    const double thickness = 260.0
        + 760.0 * strongestLifecycle * activity
        + 170.0 * Params.turbulence;
    const double shadow = std::clamp(
        0.05 + coverage * (0.28 + 0.58 * strongestLifecycle),
        0.04, 0.68);
    // Wrap visual material advection to keep parameter magnitudes bounded
    // while remaining deterministic over long sessions.
    const double wrappedTime = std::fmod(std::max(0.0, timeSeconds), 1800.0);
    return {
        Params.thermalTopMslM,
        thickness,
        coverage,
        strongestLifecycle,
        shadow,
        Params.baseWindMps * wrappedTime
    };
}
}
