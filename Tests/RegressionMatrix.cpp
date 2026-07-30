#include "AtmosphereModel.h"
#include "ParagliderDynamics.h"
#include "TerrainModel.h"
#include "WingCatalogue.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <iterator>

using namespace Parapenting::Physics;

struct RunResult
{
    FlightState state;
    double maxAirspeed = 0.0;
    double maxLoad = 0.0;
    double secondsAboveFiveG = 0.0;
    double longestAboveFiveG = 0.0;
};

struct WeatherCase
{
    const char* name;
    WeatherMode mode;
    bool useEveningPreset = false;
};

static RunResult Run(WingProfileId wing, const WeatherCase& weather)
{
    ParagliderDynamics dynamics(GetWingProfile(wing).parameters);
    AtmosphereModel atmosphere;
    if (weather.useEveningPreset)
        atmosphere.SetPreset(WeatherPresetId::EveningDrainage);
    else
        atmosphere.SetMode(weather.mode);
    FlightState state;
    state.positionWorldM = {0.0, 0.0, 737.0};
    constexpr double dt = 1.0 / 120.0;
    RunResult result;
    double currentAboveFiveG = 0.0;

    for (int frame = 0; frame < 600 * 120; ++frame)
    {
        const double time = frame * dt;
        ControlInput controls;
        const int phase = static_cast<int>(time / 20.0) % 4;
        if (phase == 1)
        {
            controls.leftBrake = 0.28;
            controls.weightShift = -0.22;
        }
        else if (phase == 2)
        {
            controls.rightBrake = 0.28;
            controls.weightShift = 0.22;
        }
        else if (phase == 3)
        {
            controls.leftBrake = 0.18;
            controls.rightBrake = 0.18;
        }
        const double halfSpan =
            0.5 * std::sqrt(dynamics.Parameters().areaM2 * 5.2);
        Atmosphere sample = atmosphere.SampleCanopy(
            state.positionWorldM,
            state.attitude.Rotate({0.0, 1.0, 0.0}),
            halfSpan, time);
        dynamics.Step(state, controls, sample, dt);
        const Telemetry& telemetry = dynamics.LastTelemetry();
        assert(std::isfinite(state.positionWorldM.x));
        assert(std::isfinite(state.positionWorldM.y));
        assert(std::isfinite(state.positionWorldM.z));
        assert(std::isfinite(telemetry.airspeedMps));
        assert(telemetry.airspeedMps >= 0.0 && telemetry.airspeedMps < 80.0);
        assert(telemetry.loadFactor >= 0.0 && telemetry.loadFactor < 15.0);
        assert(telemetry.leftCollapse >= 0.0 && telemetry.leftCollapse <= 1.0);
        assert(telemetry.rightCollapse >= 0.0 && telemetry.rightCollapse <= 1.0);
        assert(telemetry.canopyPressure >= 0.0 && telemetry.canopyPressure <= 1.1);
        result.maxAirspeed = std::max(result.maxAirspeed, telemetry.airspeedMps);
        result.maxLoad = std::max(result.maxLoad, telemetry.loadFactor);
        if (telemetry.loadFactor > 5.0)
        {
            result.secondsAboveFiveG += dt;
            currentAboveFiveG += dt;
            result.longestAboveFiveG = std::max(
                result.longestAboveFiveG, currentAboveFiveG);
        }
        else
        {
            currentAboveFiveG = 0.0;
        }
    }
    result.state = state;
    return result;
}

int main()
{
    const WeatherCase weatherCases[] = {
        {"chill", WeatherMode::Chill},
        {"ridge", WeatherMode::Ridge},
        {"localized rotor", WeatherMode::LocalizedRotor},
        {"rotor everywhere", WeatherMode::RotorEverywhere},
        {"evening drainage", WeatherMode::Ridge, true}
    };
    for (const WingProfile& wing : GetWingProfiles())
    {
        for (const WeatherCase& weather : weatherCases)
        {
            const RunResult first = Run(wing.id, weather);
            const RunResult second = Run(wing.id, weather);
            assert(std::abs(first.state.positionWorldM.x
                          - second.state.positionWorldM.x) < 1e-9);
            assert(std::abs(first.state.positionWorldM.y
                          - second.state.positionWorldM.y) < 1e-9);
            assert(std::abs(first.state.positionWorldM.z
                          - second.state.positionWorldM.z) < 1e-9);
            std::cout << wing.displayName << " weather "
                      << weather.name
                      << " max speed " << first.maxAirspeed
                      << " m/s max load " << first.maxLoad
                      << " g >5g " << first.secondsAboveFiveG
                      << " s longest " << first.longestAboveFiveG
                      << " s" << std::endl;
            // EN 926-1's 8 g sustained test is a structural qualification
            // threshold, not an ordinary flight target. Severe scripted
            // weather may create brief numerical peaks, but must not pin the
            // simulated pilot at structural-test loads.
            assert(first.longestAboveFiveG < 0.35);
            assert(first.secondsAboveFiveG < 1.5);
        }
    }
    std::cout << "All " << GetWingProfiles().size()
                            * std::size(weatherCases) * 2
              << " ten-minute deterministic matrix runs passed.\n";
}
