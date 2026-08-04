#include "PilotPose.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double RadToDeg = 180.0 / 3.14159265358979323846;

Vec3 NormalizedOr(const Vec3& value, const Vec3& fallback)
{
    const double length = std::sqrt(value.x * value.x + value.y * value.y
        + value.z * value.z);
    return length > 1e-6
        ? Vec3{value.x / length, value.y / length, value.z / length}
        : fallback;
}

Vec3 SolveElbow(const Vec3& shoulder, const Vec3& requestedHand,
    const Vec3& pole, double upperArmCm, double forearmCm)
{
    const Vec3 run = requestedHand - shoulder;
    const double requestedDistance = std::sqrt(
        run.x * run.x + run.y * run.y + run.z * run.z);
    const double minimumReach = std::abs(upperArmCm - forearmCm) + 0.01;
    const double maximumReach = upperArmCm + forearmCm - 0.01;
    const Vec3 axis = NormalizedOr(run, {0.0, 0.0, 1.0});
    const double reach = std::clamp(requestedDistance, minimumReach, maximumReach);
    const double along = (upperArmCm * upperArmCm - forearmCm * forearmCm
        + reach * reach) / (2.0 * reach);
    const double height = std::sqrt(std::max(0.0,
        upperArmCm * upperArmCm - along * along));
    Vec3 perpendicular = pole - axis * (pole.x * axis.x + pole.y * axis.y
        + pole.z * axis.z);
    perpendicular = NormalizedOr(perpendicular,
        NormalizedOr(Vec3{-axis.z, 0.0, axis.x}, {0.0, 1.0, 0.0}));
    return shoulder + axis * along + perpendicular * height;
}

Vec3 ConstrainHandReach(const Vec3& shoulder, const Vec3& requestedHand,
    double upperArmCm, double forearmCm)
{
    const Vec3 run = requestedHand - shoulder;
    const double distance = std::sqrt(run.x * run.x + run.y * run.y
        + run.z * run.z);
    const double minimumReach = std::abs(upperArmCm - forearmCm) + 0.01;
    const double maximumReach = upperArmCm + forearmCm - 0.01;
    const double reach = std::clamp(distance, minimumReach, maximumReach);
    return shoulder + NormalizedOr(run, {0.0, 0.0, 1.0}) * reach;
}
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
    pose.pelvisCm = {1.0, shift * 5.0, -13.0};
    pose.chestCm = {-9.0 - surge * 9.0, shift * 3.0, 17.0};
    pose.headCm = {-12.0 - surge * 5.0, shift * 2.0, 53.0};
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
    pose.leftHandCm = ConstrainHandReach(pose.leftShoulderCm,
        pose.leftHandCm, PilotUpperArmLengthCm, PilotForearmLengthCm);
    pose.rightHandCm = ConstrainHandReach(pose.rightShoulderCm,
        pose.rightHandCm, PilotUpperArmLengthCm, PilotForearmLengthCm);
    pose.leftElbowCm = SolveElbow(pose.leftShoulderCm, pose.leftHandCm,
        {9.0 + leftForce * 5.0, -8.0, 5.0}, PilotUpperArmLengthCm,
        PilotForearmLengthCm);
    pose.rightElbowCm = SolveElbow(pose.rightShoulderCm, pose.rightHandCm,
        {9.0 + rightForce * 5.0, 8.0, 5.0}, PilotUpperArmLengthCm,
        PilotForearmLengthCm);
    // The seated leg is one fixed two-segment chain hung off the hip. Surge
    // rotates that chain about the hip and weight shift translates it
    // laterally, both of which are rigid.
    //
    // Offsetting each joint by its own coefficient instead, as this did first,
    // made thigh and shin length functions of surge and weight shift, and the
    // differing lateral coefficients made the left and right legs different
    // lengths under shift. On the primitive blockout that was a scaled
    // cylinder; on a skinned mesh it is a stretching limb.
    const double legSwing = -0.18 * surge;
    const double legCos = std::cos(legSwing);
    const double legSin = std::sin(legSwing);
    const double hipX = 1.0;
    const double hipZ = -13.0;
    const double lateral = shift * 3.0;
    const auto legJoint = [&](double side, double dx, double dy, double dz)
    {
        return Vec3{
            hipX + dx * legCos - dz * legSin,
            side * dy + lateral,
            hipZ + dx * legSin + dz * legCos};
    };
    pose.leftHipCm = legJoint(-1.0, 0.0, 14.0, 0.0);
    pose.rightHipCm = legJoint(1.0, 0.0, 14.0, 0.0);
    pose.leftKneeCm = legJoint(-1.0, 30.0, 17.0, -18.0);
    pose.rightKneeCm = legJoint(1.0, 30.0, 17.0, -18.0);
    pose.leftAnkleCm = legJoint(-1.0, 59.0, 18.0, -34.0);
    pose.rightAnkleCm = legJoint(1.0, 59.0, 18.0, -34.0);
    return pose;
}
}
