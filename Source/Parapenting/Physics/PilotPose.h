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

PilotPose EvaluatePilotPose(const PilotPoseInput& input);
}
