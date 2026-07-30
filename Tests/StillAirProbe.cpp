#include "ParagliderDynamics.h"
#include "WingCatalogue.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

using namespace Parapenting::Physics;

int main(int argc, char** argv)
{
    constexpr double Dt = 1.0 / 120.0;
    const auto& epic = GetWingProfile(WingProfileId::Epic2MLResearch);
    ParagliderDynamics dynamics(epic.parameters);
    FlightState state;
    const bool leftWeightShift =
        argc > 1 && std::string(argv[1]) == "left";
    std::cout << "time,airspeed,vx,vy,vz,glide,pitch,bank,payload_pitch,aoa,cl,cd\n";
    for (int frame = 0; frame <= 600 * 120; ++frame)
    {
        if (frame > 0)
        {
            ControlInput input;
            input.weightShift = leftWeightShift ? -1.0 : 0.0;
            dynamics.Step(state, input, Atmosphere{}, Dt);
        }
        if (frame % 120 != 0) continue;
        const auto& t = dynamics.LastTelemetry();
        const double horizontal = std::hypot(
            state.velocityWorldMps.x, state.velocityWorldMps.y);
        const double glide = horizontal
            / std::max(0.01, -state.velocityWorldMps.z);
        const double pitch = std::asin(std::clamp(
            state.attitude.Rotate({1.0, 0.0, 0.0}).z, -1.0, 1.0));
        const double bank = std::asin(std::clamp(
            state.attitude.Rotate({0.0, 1.0, 0.0}).z, -1.0, 1.0));
        std::cout << std::fixed << std::setprecision(4)
            << frame * Dt << ',' << t.airspeedMps << ','
            << state.velocityWorldMps.x << ',' << state.velocityWorldMps.y << ','
            << state.velocityWorldMps.z << ',' << glide << ','
            << pitch << ',' << bank << ',' << state.harnessPitchRad << ','
            << t.angleOfAttackRad << ',' << t.liftCoefficient << ','
            << t.dragCoefficient << '\n';
    }
}
