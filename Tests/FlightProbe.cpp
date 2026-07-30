#include "ParagliderDynamics.h"
#include "AtmosphereModel.h"
#include "TerrainModel.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace Parapenting::Physics;

int main()
{
    ParagliderDynamics dynamics;
    AtmosphereModel atmosphere;
    FlightState state;
    state.positionWorldM.z = 737.0;

    constexpr double dt = 1.0 / 120.0;
    std::cout << "time,x,y,z,agl,vx,vz,airspeed,rotor,leftCollapse,rightCollapse\n";
    for (int frame = 0; frame <= 240 * 120; ++frame)
    {
        const double halfSpan =
            0.5 * std::sqrt(dynamics.Parameters().areaM2 * 5.2);
        const Atmosphere air = atmosphere.SampleCanopy(
            state.positionWorldM,
            state.attitude.Rotate({0.0, 1.0, 0.0}),
            halfSpan, frame * dt);
        dynamics.Step(state, {}, air, dt);
        if (frame % 120 == 0)
        {
            const auto& telemetry = dynamics.LastTelemetry();
            const double ground = TerrainModel::HeightM(
                state.positionWorldM.x, state.positionWorldM.y);
            std::cout << std::fixed << std::setprecision(4)
                      << frame * dt << ','
                      << state.positionWorldM.x << ','
                      << state.positionWorldM.y << ','
                      << state.positionWorldM.z << ','
                      << state.positionWorldM.z - ground << ','
                      << state.velocityWorldMps.x << ','
                      << state.velocityWorldMps.z << ','
                      << telemetry.airspeedMps << ','
                      << telemetry.rotorStrength << ','
                      << telemetry.leftCollapse << ','
                      << telemetry.rightCollapse << '\n';
        }
        if (!std::isfinite(state.positionWorldM.x) || !std::isfinite(state.positionWorldM.z))
            return 2;
        if (state.positionWorldM.z <= TerrainModel::HeightM(
                state.positionWorldM.x, state.positionWorldM.y) + 0.8)
        {
            std::cerr << "Terrain contact at " << frame * dt << " seconds\n";
            break;
        }
    }
}
