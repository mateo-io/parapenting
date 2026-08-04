#pragma once

#include "PilotPose.h"

#include <array>

namespace Parapenting::Physics
{
constexpr int GliderRigSideCount = 2;
constexpr int GliderRigRiserCount = 4;

// Immutable presentation state published at a fixed simulation boundary.
// It deliberately contains achieved controls only: command input remains an
// implementation detail of the solver and must never animate the rig.
struct GliderRigSnapshot
{
    double simulationTimeSeconds = 0.0;
    PilotPose pilot{};
    double weightShift = 0.0;
    // Filtered torso lean. Published so the lag is part of the immutable
    // snapshot rather than render-side state that a pause or a frame-rate
    // change could desynchronise.
    double torsoSurge = 0.0;
    std::array<double, 2> brakeTravel{};
    std::array<double, 2> brakeForceN{};
    std::array<double, 2> brakeTravelVelocityPerS{};
    // Hardware anchors are relative to the posed harness. The renderer applies
    // the snapshot's pilot transform once; it never rebuilds these anchors
    // from control input or independent harness geometry.
    std::array<Vec3, GliderRigSideCount> carabinerRigCm{};
    std::array<std::array<Vec3, GliderRigRiserCount>, GliderRigSideCount>
        riserTopRigCm{};
    Telemetry telemetry{};
};

struct GliderRigSnapshotInput
{
    double simulationTimeSeconds = 0.0;
    double harnessRollRad = 0.0;
    double harnessPitchRad = 0.0;
    double weightShift = 0.0;
    double leftBrakeTravel = 0.0;
    double rightBrakeTravel = 0.0;
    double leftBrakeForceN = 0.0;
    double rightBrakeForceN = 0.0;
    double incidentSeverity = 0.0;
    double recoverySurge = 0.0;
    double carabinerHalfSeparationCm = 21.0;
    double riserLengthCm = 45.0;
    Telemetry telemetry{};
};

GliderRigSnapshot BuildGliderRigSnapshot(const GliderRigSnapshotInput& input,
    const GliderRigSnapshot* previous = nullptr);

// Bounded render interpolation. Alpha is clamped so a hitch or caller error
// cannot extrapolate a hand, line root or attachment past the solver state.
GliderRigSnapshot InterpolateGliderRigSnapshot(
    const GliderRigSnapshot& previous, const GliderRigSnapshot& current,
    double alpha);
}
