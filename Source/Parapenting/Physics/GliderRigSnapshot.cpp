#include "GliderRigSnapshot.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
double Lerp(double a, double b, double t) { return a + (b - a) * t; }
Vec3 Lerp(const Vec3& a, const Vec3& b, double t)
{
    return {Lerp(a.x, b.x, t), Lerp(a.y, b.y, t), Lerp(a.z, b.z, t)};
}

PilotPose Lerp(const PilotPose& a, const PilotPose& b, double t)
{
    PilotPose out;
    out.rigOffsetCm = Lerp(a.rigOffsetCm, b.rigOffsetCm, t);
    out.rigRotationDegrees = Lerp(a.rigRotationDegrees, b.rigRotationDegrees, t);
    out.leftShoulderCm = Lerp(a.leftShoulderCm, b.leftShoulderCm, t);
    out.rightShoulderCm = Lerp(a.rightShoulderCm, b.rightShoulderCm, t);
    out.leftElbowCm = Lerp(a.leftElbowCm, b.leftElbowCm, t);
    out.rightElbowCm = Lerp(a.rightElbowCm, b.rightElbowCm, t);
    out.leftHandCm = Lerp(a.leftHandCm, b.leftHandCm, t);
    out.rightHandCm = Lerp(a.rightHandCm, b.rightHandCm, t);
    return out;
}

void PopulateHardwareAnchors(GliderRigSnapshot& snapshot,
    const GliderRigSnapshotInput& input)
{
    constexpr std::array<double, GliderRigRiserCount> ForeAftCm{
        6.0, 1.0, -5.0, -11.0};
    const double halfSeparation = std::max(0.0, input.carabinerHalfSeparationCm);
    const double riserLength = std::max(0.0, input.riserLengthCm);
    for (int side = 0; side < GliderRigSideCount; ++side)
    {
        const double lateral = side == 0 ? -halfSeparation : halfSeparation;
        snapshot.carabinerRigCm[side] = {-2.0, lateral, 34.0};
        for (int riser = 0; riser < GliderRigRiserCount; ++riser)
        {
            const double foreAft = ForeAftCm[riser];
            const double vertical = std::sqrt(std::max(0.0,
                riserLength * riserLength - foreAft * foreAft));
            snapshot.riserTopRigCm[side][riser] = {
                -2.0 + foreAft, lateral, 34.0 + vertical};
        }
    }
}
}

GliderRigSnapshot BuildGliderRigSnapshot(const GliderRigSnapshotInput& input,
    const GliderRigSnapshot* previous)
{
    GliderRigSnapshot snapshot;
    snapshot.simulationTimeSeconds = input.simulationTimeSeconds;
    snapshot.weightShift = std::clamp(input.weightShift, -1.0, 1.0);
    snapshot.brakeTravel = {std::clamp(input.leftBrakeTravel, 0.0, 1.0),
                            std::clamp(input.rightBrakeTravel, 0.0, 1.0)};
    snapshot.brakeForceN = {std::max(0.0, input.leftBrakeForceN),
                            std::max(0.0, input.rightBrakeForceN)};
    snapshot.telemetry = input.telemetry;
    snapshot.pilot = EvaluatePilotPose({input.harnessRollRad,
        input.harnessPitchRad, snapshot.weightShift, snapshot.brakeTravel[0],
        snapshot.brakeTravel[1], snapshot.brakeForceN[0],
        snapshot.brakeForceN[1], input.incidentSeverity, input.recoverySurge});
    PopulateHardwareAnchors(snapshot, input);
    if (previous)
    {
        const double dt = snapshot.simulationTimeSeconds
            - previous->simulationTimeSeconds;
        if (dt > 0.0)
            for (int side = 0; side < 2; ++side)
                snapshot.brakeTravelVelocityPerS[side] =
                    (snapshot.brakeTravel[side] - previous->brakeTravel[side]) / dt;
    }
    return snapshot;
}

GliderRigSnapshot InterpolateGliderRigSnapshot(
    const GliderRigSnapshot& previous, const GliderRigSnapshot& current,
    double alpha)
{
    const double t = std::clamp(alpha, 0.0, 1.0);
    GliderRigSnapshot out;
    out.simulationTimeSeconds = Lerp(previous.simulationTimeSeconds,
        current.simulationTimeSeconds, t);
    out.pilot = Lerp(previous.pilot, current.pilot, t);
    out.weightShift = Lerp(previous.weightShift, current.weightShift, t);
    out.telemetry = current.telemetry;
    // Only render-consumed continuous measurements are blended. Discrete
    // solver events retain the current boundary state below, which prevents
    // a half-collapse or half-stall invented between two fixed steps.
    const auto blendTelemetry = [&previous, &current, &out, t](
        double Telemetry::*field)
    {
        out.telemetry.*field = Lerp(
            previous.telemetry.*field, current.telemetry.*field, t);
    };
    for (double Telemetry::*field : {
        &Telemetry::airspeedMps, &Telemetry::loadFactor,
        &Telemetry::highLoadDeformation, &Telemetry::turbulence,
        &Telemetry::rotorStrength, &Telemetry::canopyPressure,
        &Telemetry::harnessRollRad, &Telemetry::harnessPitchRad,
        &Telemetry::leftBrakeForceN, &Telemetry::rightBrakeForceN,
        &Telemetry::recoverySurge, &Telemetry::deepStall,
        &Telemetry::leftCravat, &Telemetry::rightCravat,
        &Telemetry::canopyRelativePitchRad,
        &Telemetry::canopyRelativeRollRad,
        &Telemetry::canopyRelativePitchRateRadps,
        &Telemetry::canopyRelativeRollRateRadps})
        blendTelemetry(field);
    for (int line = 0; line < 4; ++line)
    {
        out.telemetry.leftLineTensionN[line] = Lerp(
            previous.telemetry.leftLineTensionN[line],
            current.telemetry.leftLineTensionN[line], t);
        out.telemetry.rightLineTensionN[line] = Lerp(
            previous.telemetry.rightLineTensionN[line],
            current.telemetry.rightLineTensionN[line], t);
        out.telemetry.leftLineSlack[line] = Lerp(
            previous.telemetry.leftLineSlack[line],
            current.telemetry.leftLineSlack[line], t);
        out.telemetry.rightLineSlack[line] = Lerp(
            previous.telemetry.rightLineSlack[line],
            current.telemetry.rightLineSlack[line], t);
    }
    out.telemetry.leftCollapse = Lerp(previous.telemetry.leftCollapse,
        current.telemetry.leftCollapse, t);
    out.telemetry.rightCollapse = Lerp(previous.telemetry.rightCollapse,
        current.telemetry.rightCollapse, t);
    out.telemetry.frontalCollapse = Lerp(previous.telemetry.frontalCollapse,
        current.telemetry.frontalCollapse, t);
    for (int side = 0; side < 2; ++side)
    {
        out.brakeTravel[side] = Lerp(previous.brakeTravel[side],
            current.brakeTravel[side], t);
        out.brakeForceN[side] = Lerp(previous.brakeForceN[side],
            current.brakeForceN[side], t);
        out.brakeTravelVelocityPerS[side] = Lerp(
            previous.brakeTravelVelocityPerS[side],
            current.brakeTravelVelocityPerS[side], t);
        out.carabinerRigCm[side] = Lerp(previous.carabinerRigCm[side],
            current.carabinerRigCm[side], t);
        for (int riser = 0; riser < GliderRigRiserCount; ++riser)
            out.riserTopRigCm[side][riser] = Lerp(
                previous.riserTopRigCm[side][riser],
                current.riserTopRigCm[side][riser], t);
    }
    return out;
}
}
