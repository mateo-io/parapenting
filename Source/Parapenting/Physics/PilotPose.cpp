#include "PilotPose.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double RadToDeg = 180.0 / 3.14159265358979323846;
}

PilotPose EvaluatePilotPose(const PilotPoseInput& raw)
{
    const double roll = std::clamp(raw.harnessRollRad, -0.65, 0.65);
    const double pitch = std::clamp(raw.harnessPitchRad, -0.65, 0.65);
    const double shift = std::clamp(raw.weightShift, -1.0, 1.0);
    const double leftBrake = std::clamp(raw.leftBrake, 0.0, 1.0);
    const double rightBrake = std::clamp(raw.rightBrake, 0.0, 1.0);
    const double leftForce =
        std::clamp(raw.leftBrakeForceN / 65.0, 0.0, 1.0);
    const double rightForce =
        std::clamp(raw.rightBrakeForceN / 65.0, 0.0, 1.0);
    const double incident = std::clamp(raw.incidentSeverity, 0.0, 1.0);
    const double surge = std::clamp(raw.recoverySurge, -0.2, 0.45);

    PilotPose pose;
    pose.rigOffsetCm = {
        -95.0 * std::sin(pitch) - surge * 24.0,
        95.0 * std::sin(roll) + shift * 8.0,
        -22.0 * (std::abs(std::sin(roll)) + std::abs(std::sin(pitch)))
            - incident * 5.0
    };
    pose.rigRotationDegrees = {
        pitch * RadToDeg + surge * 8.0,
        0.0,
        roll * RadToDeg
    };
    pose.leftShoulderCm = {-10.0, -19.0, 36.0};
    pose.rightShoulderCm = {-10.0, 19.0, 36.0};
    pose.leftHandCm = {
        -40.0 - leftForce * 8.0,
        -32.0 - leftForce * 3.0,
        48.0 - leftBrake * 78.0
    };
    pose.rightHandCm = {
        -40.0 - rightForce * 8.0,
        32.0 + rightForce * 3.0,
        48.0 - rightBrake * 78.0
    };
    pose.leftElbowCm =
        pose.leftShoulderCm
        + (pose.leftHandCm - pose.leftShoulderCm) * 0.48
        + Vec3{9.0 + leftForce * 5.0, -8.0, 5.0};
    pose.rightElbowCm =
        pose.rightShoulderCm
        + (pose.rightHandCm - pose.rightShoulderCm) * 0.48
        + Vec3{9.0 + rightForce * 5.0, 8.0, 5.0};
    return pose;
}
}
