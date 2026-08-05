#pragma once

#include "PilotPose.h"

#include <array>

namespace Parapenting::Physics
{
constexpr int GliderRigSideCount = 2;
constexpr int GliderRigRiserCount = 4;

// The riser the brake pulley is mounted on is always the rearmost one: the C
// on this three-liner, the B on a two-liner. It follows the back of the riser
// set rather than being a fixed index, which is the whole point of the count
// being data.
constexpr int RearRiserIndex(int riserCount)
{
    return riserCount > 0 ? riserCount - 1 : 0;
}

// Immutable presentation state published at a fixed simulation boundary.
// It deliberately contains achieved controls only: command input remains an
// implementation detail of the solver and must never animate the rig.
struct GliderRigSnapshot
{
    double simulationTimeSeconds = 0.0;
    PilotPose pilot{};
    double weightShift = 0.0;
    // Risers actually present on this wing. Anything reading riserTopRigCm
    // must stop here rather than at the array's width.
    int riserCount = GliderRigRiserCount;
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
    // How many risers this wing has, and where each sits fore/aft on the
    // plate. Defaults are the three-liner's A, A', B, C. A two-liner passes 2
    // and its own offsets; nothing downstream counts risers for itself.
    int riserCount = GliderRigRiserCount;
    std::array<double, GliderRigRiserCount> riserForeAftCm{
        6.0, 1.0, -5.0, -11.0};
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
