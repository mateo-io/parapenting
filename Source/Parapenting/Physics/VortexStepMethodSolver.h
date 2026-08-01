#pragma once

#include "CanopyGeometry.h"
#include "ParagliderDynamics.h"
#include "SectionPolarTable.h"

#include <vector>

namespace Parapenting::Physics
{
// Level 4: the Vortex Step Method.
//
// The current model computes one lift and one drag for the whole wing from a
// fitted polar, then adds a spanwise load-split approximation on top. This
// replaces both with a spanwise circulation solve: each section gets its own
// local inflow, its own angle of attack, its own 2D coefficients, and its own
// force applied at its own position. Roll, yaw and pitch moments come out as
// the integral of that rather than as separate terms.
//
// The method is the nonlinear lifting line as reformulated at TU Delft for
// ram-air and leading-edge-inflatable wings. Its one structural difference
// from classical lifting-line theory is where the flow is sampled: the control
// point sits at the three-quarter chord rather than on the bound vortex. That
// is what lets it hold up at the low aspect ratio, strong anhedral and sweep a
// paraglider actually has, where classical lifting line does not.
//
// What is NOT here yet, and is not pretended to be:
//   * a wake that deforms - the trailing filaments are straight and aligned
//     with the freestream, which is the steady assumption (Level 11);
//   * attached/separated hysteresis - the polar table has no memory;
//   * apparent mass - Lissaman and Brown's tensor is a separate piece;
//   * validation against the reference Python/Julia implementations, which
//     the plan requires before any flight number from this is trusted. What
//     it is checked against here is classical lifting-line theory on an
//     elliptical wing, which is exact and needs no external data.

struct VsmSection
{
    // Bound vortex endpoints, at the section quarter chord, ordered from the
    // RIGHT end to the left. That ordering is not cosmetic: with the
    // freestream running toward -X in body axes, Kutta-Joukowski
    // F = rho (V x Gamma) only gives upward lift for positive circulation if
    // the bound vortex points along -Y. Ordered the other way, a positive
    // circulation makes downforce and the nonlinear solve runs away.
    Vec3 boundStartM{};
    Vec3 boundEndM{};
    // Where the inflow is sampled: three-quarter chord, mid-section.
    Vec3 controlPointM{};
    // Unit chordwise direction, leading edge to trailing edge is -X, so this
    // points forward along +X.
    Vec3 chordDirection{};
    // Unit normal to the section, positive toward the upper surface.
    Vec3 normal{};
    // Unit vector along the bound vortex.
    Vec3 spanDirection{};
    double chordM = 0.0;
    double widthM = 0.0;
    double areaM2 = 0.0;
    // -1 at the left tip to +1 at the right, at the section's midpoint.
    double spanFraction = 0.0;
    // How much of this section lies right of the centreline, 0 to 1. A panel
    // that straddles the centre is braked partly by each hand. Deciding by
    // the midpoint's sign instead makes the answer depend on whether the
    // panel count is odd, and breaks left-right mirror symmetry when it is.
    double rightSideFraction = 1.0;
};

struct VsmSolveInput
{
    // The wing's velocity through the air, body axes, metres per second.
    // Forward flight is +X. The freestream the solver works in is the
    // negative of this - the air's velocity relative to the wing - and the
    // conversion happens in one place inside the solver rather than at every
    // call site.
    Vec3 airspeedBodyMps{11.0, 0.0, 0.0};
    // Body angular velocity, so a rolling wing sees the spanwise variation in
    // local incidence that damps it. Roll damping is then an integral, not a
    // coefficient.
    Vec3 angularVelocityBodyRadps{};
    double airDensityKgM3 = 1.12;
    double leftBrake = 0.0;
    double rightBrake = 0.0;
    // Internal pressure coefficient per section, from the Level 5 cell
    // solver. Empty means fully pressurised, which is what a caller with no
    // pressure model gets. This is the coupling the plan asks for: a
    // depressurised cell group stops making lift where it is, rather than the
    // wing losing performance uniformly.
    std::vector<double> internalPressureCoefficient;
};

// Per-section separation, carried between solves. This is the attached and
// separated state the plan asks for, and it does two jobs at once: it gives
// stall the memory it physically has, and it makes the solve well posed at
// high brake, because the lift curve the solver sees during one solve is
// single valued instead of switching branches under it.
struct VsmSeparationState
{
    std::vector<double> sectionSeparation;
    // Circulation carried between solves. A cold nonlinear solve takes about
    // ninety iterations; continued from the last one it takes a handful,
    // because the wing's loading changes far more slowly than the solver
    // converges. This is what makes running the VSM inside a flight loop
    // affordable at all.
    std::vector<double> circulation;
    bool initialised = false;
};

struct VsmSectionResult
{
    double circulation = 0.0;
    double angleOfAttackRad = 0.0;
    double liftCoefficient = 0.0;
    double dragCoefficient = 0.0;
    // Downwash angle induced at this section by the whole wing.
    double inducedAngleRad = 0.0;
    // 0 attached, 1 fully separated.
    double separation = 0.0;
    Vec3 forceBodyN{};
};

struct VsmSolution
{
    std::vector<VsmSectionResult> sections;
    Vec3 forceBodyN{};
    // About the wing's quarter-chord centre, body axes.
    Vec3 momentBodyNm{};
    double liftCoefficient = 0.0;
    double inducedDragCoefficient = 0.0;
    double profileDragCoefficient = 0.0;
    double totalDragCoefficient = 0.0;
    int iterations = 0;
    // Largest change in circulation on the last iteration, as a fraction of
    // the largest circulation. Reported so a solve that did not converge says
    // so rather than returning a plausible-looking answer.
    double residual = 0.0;
    // What the adaptive under-relaxation settled on. Far below the starting
    // value means the solve was fighting the stall knee.
    double finalRelaxation = 0.0;
    bool converged = false;
};

struct VsmSettings
{
    // Well-posed cases converge in about 90 iterations, so this is generous.
    // A case that wants thousands is not converging, and letting it run does
    // not change that - it only makes finding out expensive.
    int maxIterations = 600;
    double convergenceTolerance = 1.0e-6;
    // Floor for the adaptive under-relaxation. A case that needs less than
    // this is not converging for a reason worth looking at rather than
    // damping harder.
    double minimumRelaxation = 0.002;
    // How fast separation spreads and how fast it clears, per second. Stall
    // arrives quickly and leaves slowly, which is what makes a paraglider
    // stall easier to enter than to exit.
    double separationOnsetRatePerS = 12.0;
    double reattachmentRatePerS = 4.0;
    // Under-relaxation. The circulation solve is nonlinear through the polar,
    // and taking the full step oscillates near stall.
    double relaxation = 0.60;
};

// Everything hanging below the wing. On a paraglider this is not a correction
// term: lines, risers, harness and pilot are a large fraction of total drag,
// and a canopy polar alone flies far better than the real thing.
//
// The line figures come from the suspension graph, so this is the same 254 m
// of line the Level 2 solver strings up, not a second description of it.
struct InstalledDragSpec
{
    double lineTotalLengthM = 254.0;
    double lineMeanDiameterM = 0.00105;
    // A cylinder in crossflow, in the subcritical range these Reynolds
    // numbers sit in.
    double lineDragCoefficient = 1.05;
    // Manufactured line length is not all normal to the flow: cascades
    // overlap, upper galleries are inclined, lower lines shield one another.
    double lineProjectedFraction = 0.35;
    // Seated pilot plus harness, frontal area.
    double harnessAreaM2 = 0.32;
    double harnessDragCoefficient = 1.05;
    // How far below the canopy the harness drag acts. It is a long lever, so
    // this drag is also a pitching moment.
    double harnessBelowCanopyM = 7.8;
};

struct InstalledDragResult
{
    double lineDragN = 0.0;
    double harnessDragN = 0.0;
    double totalDragN = 0.0;
    // About the canopy quarter-chord centre, body axes.
    Vec3 momentBodyNm{};
};

InstalledDragResult EvaluateInstalledDrag(
    const InstalledDragSpec& spec, const Vec3& airspeedBodyMps,
    double airDensityKgM3);

class VortexStepMethodSolver
{
public:
    // Builds sections from the Level 1 canopy: same geometry as the physics
    // panels, the render mesh and the line attachments.
    // `coreFraction` is the trailing-filament core radius as a fraction of
    // the panel width. Numerical, not physical: see the note on the kernel.
    VortexStepMethodSolver(
        const CanopyGeometry& geometry, SectionPolarTable polars,
        int sectionCount = 40, double coreFraction = 0.5);

    // A flat, untwisted planform for validation against analytic results.
    // `taper` of 0 gives an ellipse, 1 gives a rectangle.
    static VortexStepMethodSolver FlatWing(
        double spanM, double rootChordM, bool elliptical,
        SectionPolarTable polars, int sectionCount = 60,
        double coreFraction = 0.5);

    VsmSolution Solve(
        const VsmSolveInput& input, const VsmSettings& settings = {}) const;

    // Steps the separation state forward by `deltaSeconds` and solves with it
    // held. Passing an uninitialised state starts it at the equilibrium for
    // the incidence found, so the first call is not a transient.
    VsmSolution SolveUnsteady(
        const VsmSolveInput& input, VsmSeparationState& state,
        double deltaSeconds, const VsmSettings& settings = {}) const;

    // Asks a what-if question of the wing the last unsteady solve found: the
    // same separation state, held, and a circulation the caller keeps warm,
    // with neither advanced. A caller measuring a derivative needs exactly
    // this - the same wing at a different rate - and calling Solve instead
    // silently substitutes a different wing, at the equilibrium separation for
    // whatever incidence it lands on.
    VsmSolution SolveFrozen(
        const VsmSolveInput& input, const VsmSeparationState& state,
        std::vector<double>& warmCirculation,
        const VsmSettings& settings = {}) const;

    const std::vector<VsmSection>& Sections() const { return SectionList; }
    double ReferenceAreaM2() const { return ReferenceArea; }
    double ReferenceSpanM() const { return ReferenceSpan; }
    double AspectRatio() const
    {
        return ReferenceSpan * ReferenceSpan / ReferenceArea;
    }

private:
    VortexStepMethodSolver() = default;
    void BuildInfluenceMatrix(double coreFraction);
    VsmSolution SolveHeld(
        const VsmSolveInput& input, const VsmSettings& settings,
        const std::vector<double>* heldSeparation,
        std::vector<double>* warmCirculation) const;

    std::vector<VsmSection> SectionList;
    SectionPolarTable Polars;
    // Induced velocity at control point i per unit circulation on section j.
    std::vector<Vec3> Influence;
    double ReferenceArea = 0.0;
    double ReferenceSpan = 0.0;
};
}
