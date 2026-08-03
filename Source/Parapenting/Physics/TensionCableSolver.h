#pragma once

#include "SuspensionGraph.h"

#include <vector>

namespace Parapenting::Physics
{
// Level 2: solve the suspension network for static equilibrium.
//
// The unknowns are the cascade junction positions and the canopy's pose
// relative to the payload. Everything else is anchored: carabiners and brake
// handles to the payload, attachments to the canopy body. Cables pull and
// never push, so a slack line contributes exactly nothing - not a small force,
// not a clamped one.
//
// The method is damped dynamic relaxation with a fixed iteration count. It is
// a fictitious dynamics used only to reach a static answer: the solver masses
// below are numerical, they do not appear in any force the flight model sees,
// and the residual is reported so a failure to converge is visible rather than
// silently shaping handling (guiding rule 12).

struct SuspensionSolveInput
{
    // Resultant aerodynamic force on the canopy, newtons, in the payload
    // frame rather than canopy body axes.
    //
    // The airflow decides this vector, not the canopy's own attitude. Holding
    // it in body axes at a level with no aerodynamic solver would let a
    // nose-up rotation carry the lift round with it, and the pitch equilibrium
    // would run away instead of standing still. Level 4 supplies it from VSM;
    // until then it is an input.
    Vec3 aeroForceN{0.0, 0.0, 1030.0};
    // Chord station of the centre of pressure, and how far outboard each half
    // wing's share acts.
    double aeroCentreChordFraction = 0.28;
    double aeroCentreSpanOffset = 0.40;
    // Positive means the right half carries more load.
    double spanwiseLoadAsymmetry = 0.0;
    double canopyWeightN = 50.0;
    double accelerator = 0.0;
    double leftBrake = 0.0;
    double rightBrake = 0.0;
    // How much of that brake pull the trailing edge itself absorbs by bending,
    // metres, per side. A brake line ends at the trailing edge at 98% chord,
    // so the pull has two things to do and only one length to do them with:
    // bend the fabric aft of the brake attachment, and rotate the canopy on
    // its lines. This network models the canopy as rigid, so with this at zero
    // the whole pull becomes rotation and the section's camber change is a
    // second, free copy of the same input.
    //
    // Zero is the right default HERE: standalone, this solver is a statement
    // about a cable network and a rigid body, and the take-up is a property of
    // the section the caller has and it does not. The coupled solver supplies
    // it from the section geometry.
    double leftBrakeFlapTakeUpM = 0.0;
    double rightBrakeFlapTakeUpM = 0.0;
    // -1 fully left, +1 fully right. Moves the carabiner pair; it commands no
    // moment of its own.
    double weightShift = 0.0;
    double gravityMps2 = 9.80665;

    // Hold the canopy at this attitude instead of letting the solve find it,
    // and report the moment the lines exert there.
    //
    // Solved free, the canopy rotates until the lines exert no moment on it,
    // which answers "what incidence does this wing hang at". Held, the same
    // network answers a different and equally real question: "how hard do
    // these lines resist being rotated away from that". That is the wing's
    // pitch stiffness, and it is a property of where the A, B and C rows
    // attach along the chord - not something to write down as a number.
    bool holdCanopyAttitude = false;
    Quaternion imposedCanopyAttitude{};
};

struct SuspensionSolverSettings
{
    // 12000 steps at 0.999 retention leaves a canopy force residual under
    // 0.01 N on the trim case - four orders below the flight load. Fewer steps
    // or heavier damping both converge more slowly, and the residual is
    // reported so a case that does not converge says so.
    int iterations = 12000;
    double timeStepS = 5.0e-4;
    // Velocity kept per step. Sets how fast the fictitious dynamics bleeds off
    // energy on its way to equilibrium.
    double velocityRetention = 0.999;
    double nodeSolverMassKg = 0.25;
    double canopySolverMassKg = 25.0;
    double canopySolverInertiaKgM2 = 45.0;
    double cableDampingRatio = 0.06;
};

struct CableState
{
    int nodeA = -1;
    int nodeB = -1;
    LineRow row = LineRow::A;
    int cascadeLevel = 0;
    double lengthM = 0.0;
    double restLengthM = 0.0;
    double strain = 0.0;
    double tensionN = 0.0;
    bool slack = true;
};

struct SuspensionSolution
{
    std::vector<Vec3> nodePositionM;
    std::vector<CableState> cables;

    Vec3 canopyOriginM{};
    Quaternion canopyAttitude{};
    double canopyPitchRad = 0.0;
    double canopyRollRad = 0.0;
    // Change in root-chord incidence from the unloaded design pose. This is
    // the only path bar and brake have to the canopy.
    double incidenceChangeRad = 0.0;

    Vec3 leftCarabinerForceN{};
    Vec3 rightCarabinerForceN{};
    double leftCarabinerLoadN = 0.0;
    double rightCarabinerLoadN = 0.0;
    // Positive means the right carabiner carries more.
    double lateralLoadImbalance = 0.0;

    // Indexed by LineRow.
    double rowTensionN[LineRowCount]{};
    double rowLoadFraction[LineRowCount]{};
    double leftRowTensionN[LineRowCount]{};
    double rightRowTensionN[LineRowCount]{};

    double totalTensionN = 0.0;
    double maxTensionN = 0.0;
    double maxStrain = 0.0;
    // Mean stretch of the loaded main lines, and the vertical trim shift it
    // costs. Not negligible on a modern wing, so it is reported rather than
    // assumed away.
    double meanMainStrain = 0.0;
    double lineStretchM = 0.0;

    int slackCableCount = 0;
    // Largest compressive force any slack cable transmitted. Must be exactly
    // zero; the exit gate is not "small".
    double slackCompressiveForceN = 0.0;

    // The moment the lines exert on the canopy at the pose the solve ended
    // at, world axes. Solved free this goes to zero by construction, which is
    // what equilibrium means. Solved with the attitude held it is the real
    // answer, and the derivative of it is the wing's pitch stiffness.
    Vec3 canopyMomentBodyNm{};

    // Equilibrium residuals from the final iteration.
    double canopyForceResidualN = 0.0;
    double canopyMomentResidualNm = 0.0;
    double maxNodeResidualN = 0.0;
    // Net force summed over every cable endpoint. Equal and opposite reactions
    // make this zero by construction; it is measured to prove it.
    Vec3 endpointForceSumN{};
};

// Warm-start state, so the network can be carried between steps instead of
// relaxed from its design pose every time. A cold solve takes 12000
// iterations; continued from where it was last step it takes a few dozen,
// which is what makes running it inside a 120 Hz loop possible at all.
struct SuspensionWarmStart
{
    std::vector<Vec3> junctionPositionM;
    std::vector<Vec3> junctionVelocityMps;
    Vec3 canopyOriginM{};
    Quaternion canopyAttitude{};
    Vec3 canopyVelocityMps{};
    Vec3 canopyAngularVelocityRadps{};
    bool initialised = false;
};

SuspensionSolution SolveSuspension(
    const SuspensionGraph& graph, const SuspensionSolveInput& input,
    const SuspensionSolverSettings& settings = {},
    SuspensionWarmStart* warmStart = nullptr);
}
