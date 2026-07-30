#pragma once

#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
enum class ResearchManeuver
{
    WeightShiftStep,
    BrakeZoom,
    SymmetricDeepStall,
    AsymmetricStall
};

struct ResearchManeuverResult
{
    double durationS = 0.0;
    double peakAbsBankRad = 0.0;
    double headingChangeRad = 0.0;
    double lateralDisplacementM = 0.0;
    double peakClimbMps = 0.0;
    double altitudeGainM = 0.0;
    double minimumAirspeedMps = 1000.0;
    double peakLoadFactor = 0.0;
    double peakSeparatedSpan = 0.0;
    double peakAbsEnergyResidualW = 0.0;
};

ResearchManeuverResult RunResearchManeuver(
    ResearchManeuver maneuver, WingParameters parameters = {});
}
