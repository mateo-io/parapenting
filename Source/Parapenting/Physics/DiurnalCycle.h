#pragma once

namespace Parapenting::Physics
{
struct DiurnalState
{
    double localHour = 12.0;
    double sunElevationDegrees = 45.0;
    double sunAzimuthDegrees = 180.0;
    // Signed normalized surface heat flux. Positive drives anabatic flow,
    // negative drives katabatic drainage.
    double surfaceHeating = 0.0;
    double convectiveActivity = 0.0;
    double ambientLight = 1.0;
    double warmLight = 0.0;
};

double WrapLocalHour(double hour);

// One simulated local hour currently passes in ten minutes of flight. This is
// slow enough to fly in a stable air mass but fast enough to observe a full
// transition in a deliberate weather-lab session.
DiurnalState EvaluateDiurnalCycle(
    double startLocalHour, double simulationTimeSeconds,
    double simulatedSecondsPerLocalHour = 600.0);
}
