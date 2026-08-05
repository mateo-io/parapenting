#pragma once

#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
struct PilotPoseInput
{
    double harnessRollRad = 0.0;
    double harnessPitchRad = 0.0;
    double weightShift = 0.0;
    double leftBrake = 0.0;
    double rightBrake = 0.0;
    double leftBrakeForceN = 0.0;
    double rightBrakeForceN = 0.0;
    double incidentSeverity = 0.0;
    double recoverySurge = 0.0;
    // Chest and head lean, already filtered. The torso is a mass on a spine,
    // not welded to the seat, so it lags the harness it is strapped to. The
    // filtering happens at the fixed simulation step in BuildGliderRigSnapshot
    // rather than here: this function stays a pure map from state to pose, and
    // the lag stays independent of frame rate.
    double torsoSurge = 0.0;
    // The brake pulley each handle hangs from, in rig centimetres. A brake
    // line runs from the trailing edge, through a ring or pulley on the
    // rearmost riser, and down to the handle, so this point is what sets the
    // direction the hand travels in.
    //
    // Defaults match the rearmost riser of the built rig. The snapshot passes
    // the real ones, which is what lets a two-liner route its brakes through
    // the B riser without this file knowing how many risers there are.
    Vec3 leftBrakePulleyCm{-13.0, -21.0, 77.6};
    Vec3 rightBrakePulleyCm{-13.0, 21.0, 77.6};
};

struct PilotPose
{
    Vec3 rigOffsetCm{};
    Vec3 rigRotationDegrees{};
    Vec3 pelvisCm{};
    Vec3 chestCm{};
    Vec3 headCm{};
    Vec3 leftShoulderCm{};
    Vec3 rightShoulderCm{};
    Vec3 leftElbowCm{};
    Vec3 rightElbowCm{};
    Vec3 leftHandCm{};
    Vec3 rightHandCm{};
    Vec3 leftHipCm{};
    Vec3 rightHipCm{};
    Vec3 leftKneeCm{};
    Vec3 rightKneeCm{};
    Vec3 leftAnkleCm{};
    Vec3 rightAnkleCm{};
};

constexpr double PilotUpperArmLengthCm = 39.0;
constexpr double PilotForearmLengthCm = 37.0;

// Handle travel from hands-up to full brake, measured along the brake line.
// The arm cannot always deliver it: past full extension ConstrainHandReach
// stops the hand, which is the honest answer rather than a hand that detaches
// from the shoulder to reach a number.
constexpr double PilotBrakeTravelCm = 78.0;

PilotPose EvaluatePilotPose(const PilotPoseInput& input);
}
