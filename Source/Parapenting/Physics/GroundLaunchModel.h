#pragma once

#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
enum class LaunchPhase
{
    LaidOut,
    Inflation,
    OverheadCheck,
    Turning,
    Running,
    Airborne,
    Aborted
};

struct GroundLaunchInput
{
    bool launchHeld = false;
    Vec3 surfaceWindMps{};
    Vec3 launchDirection{1.0, 0.0, 0.0};
    double slopeDownRadians = 0.12;
    double leftBrake = 0.0;
    double rightBrake = 0.0;
    double weightShift = 0.0;
    double allUpMassKg = 105.0;
    double wingAreaM2 = 27.0;
};

struct GroundLaunchState
{
    LaunchPhase phase = LaunchPhase::LaidOut;
    double inflation = 0.0;
    double pilotRunSpeedMps = 0.0;
    double canopyHeadingErrorRad = 0.0;
    double canopyElevationRad = -1.35;
    double lineLoadN = 0.0;
    double liftFraction = 0.0;
    double elapsedS = 0.0;
    double stableOverheadS = 0.0;
    bool reverseLaunch = false;
    double pilotFacingYawOffsetRad = 0.0;
    double turnProgress = 0.0;
    int turnDirection = 0;
    bool turnInputArmed = false;
};

struct GroundLaunchOutput
{
    Vec3 pilotVelocityWorldMps{};
    double canopyPressure = 0.0;
    double apparentWindMps = 0.0;
    double headwindMps = 0.0;
    double crosswindMps = 0.0;
    double pilotFacingYawOffsetRad = 0.0;
    bool brakeSidesCrossed = false;
    bool liftOff = false;
    bool abort = false;
};

class GroundLaunchModel
{
public:
    void Reset(GroundLaunchState& state, bool reverseLaunch = false) const;
    GroundLaunchOutput Step(
        GroundLaunchState& state, const GroundLaunchInput& input, double dt) const;
};

const char* LaunchPhaseName(LaunchPhase phase);
}
