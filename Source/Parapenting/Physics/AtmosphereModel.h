#pragma once

#include "ParagliderDynamics.h"
#include "WeatherSnapshot.h"
#include "DiurnalCycle.h"

#include <array>

namespace Parapenting::Physics
{
enum class WeatherMode
{
    Chill,
    Ridge,
    LocalizedRotor,
    RotorEverywhere
};

struct WeatherParameters
{
    WeatherMode mode = WeatherMode::LocalizedRotor;
    // Light northerly/headwind, flowing upslope toward the launch shoulder.
    Vec3 baseWindMps{-1.5, 0.0, 0.0};
    double turbulence = 0.18;
    double thermalStrengthMps = 1.4;
    double ridgeLiftStrengthMps = 1.8;
    double thermalTopMslM = 2100.0;
    // Peak terrain-circulation strength. Direction comes from the deterministic
    // diurnal surface heat flux.
    double surfaceHeating = 0.45;
};

struct TerrainCirculation
{
    Vec3 velocityWorldMps{};
    double alongSlopeMps = 0.0;
};

TerrainCirculation EvaluateTerrainCirculation(
    const Vec3& terrainNormal, double aboveGroundM, double surfaceHeating);

enum class WeatherVolumeType
{
    Thermal,
    Sink,
    Rotor
};

struct WeatherVolume
{
    WeatherVolumeType type = WeatherVolumeType::Thermal;
    Vec3 centreWorldM{};
    double radiusM = 0.0;
    double heightM = 0.0;
    double strength = 0.0;
    double turbulence = 0.0;
};

enum class WeatherPresetId
{
    MorningCalm,
    ValleyBreeze,
    ThermalDay,
    FoehnRotor,
    Custom,
    EveningDrainage
};

struct WeatherPreset
{
    WeatherPresetId id;
    const char* displayName;
    WeatherParameters parameters;
    std::array<WeatherVolume, 5> volumes;
    double startLocalHour = 13.0;
};

struct CloudFieldState
{
    double baseAltitudeM = 2100.0;
    double layerThicknessM = 500.0;
    double coverage = 0.1;
    double development = 0.0;
    double shadowStrength = 0.1;
    Vec3 driftM{};
};

const WeatherPreset& GetWeatherPreset(WeatherPresetId id);
const std::array<WeatherPreset, 5>& GetWeatherPresets();

class AtmosphereModel
{
public:
    explicit AtmosphereModel(WeatherParameters parameters = {});

    Atmosphere Sample(const Vec3& positionWorldM, double timeSeconds) const;
    Atmosphere SampleCanopy(
        const Vec3& centreWorldM, const Vec3& wingRightWorld,
        double halfSpanM, double timeSeconds) const;
    CloudFieldState SampleCloudField(double timeSeconds) const;
    DiurnalState SampleDiurnalState(double timeSeconds) const
        { return EvaluateDiurnalCycle(StartLocalHour, timeSeconds); }
    void SetStartLocalHour(double hour)
        { StartLocalHour = WrapLocalHour(hour); }
    double GetStartLocalHour() const { return StartLocalHour; }
    void SetMode(WeatherMode mode)
    {
        Params.mode = mode;
        PresetId = WeatherPresetId::Custom;
        Snapshot.source = "manual";
        Snapshot.displayName = "Manual weather";
    }
    WeatherMode GetMode() const { return Params.mode; }
    void SetBaseWind(const Vec3& windWorldMps)
    {
        Params.baseWindMps = windWorldMps;
        PresetId = WeatherPresetId::Custom;
        Snapshot.source = "manual";
        Snapshot.displayName = "Manual weather";
        Snapshot.windFromDegrees =
            MeteorologicalDirectionFromWindVector(windWorldMps);
        Snapshot.windSpeedMps =
            std::hypot(windWorldMps.x, windWorldMps.y);
        Snapshot.gustSpeedMps =
            Snapshot.windSpeedMps + Params.turbulence * 4.0;
        Snapshot.thermalTopMslM = Params.thermalTopMslM;
    }
    const Vec3& GetBaseWind() const { return Params.baseWindMps; }
    void SetPreset(WeatherPresetId id);
    WeatherPresetId GetPresetId() const { return PresetId; }
    void ApplySnapshot(const WeatherSnapshot& snapshot);
    const WeatherSnapshot& GetSnapshot() const { return Snapshot; }
    const WeatherParameters& GetParameters() const { return Params; }
    const std::array<WeatherVolume, 5>& GetVolumes() const { return Volumes; }

private:
    WeatherParameters Params;
    std::array<WeatherVolume, 5> Volumes{};
    WeatherPresetId PresetId = WeatherPresetId::Custom;
    WeatherSnapshot Snapshot;
    double StartLocalHour = 13.0;
};
}
