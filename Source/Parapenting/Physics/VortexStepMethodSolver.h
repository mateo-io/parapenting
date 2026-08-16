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
    // Air velocity at each section, over and above the freestream, body axes.
    // Empty means still air, which is what every caller before Level 8 had.
    //
    // A wing cannot be disturbed by a uniform wind: a wind that covers the
    // whole canopy changes the frame it flies in and nothing else. What
    // actually folds a paraglider is air arriving at part of it - a rotor
    // edge, a thermal wall, a gust front - and this is where that enters. It
    // is the same term the rotation already contributes, so it costs nothing
    // but the lookup.
    std::vector<Vec3> sectionGustBodyMps;
    // Per-section incidence offset from the design pose, radians, nose-up
    // positive. Empty means the design pose, which is what every caller
    // before the geometric channel had.
    //
    // This is the channel the suspension is meant to reach the canopy
    // through. Today the lines publish ONE scalar - a root-chord incidence
    // change - and the aerodynamic solve is handed brake as a control scalar
    // instead, which is the control-to-aero shortcut guiding rule 4 forbids.
    // A per-section pose is what retires that: an asymmetric line load twists
    // the wing, the twisted wing rolls, and the roll is an outcome rather
    // than a command.
    //
    // The offset rotates the section's INCIDENCE and not its force axes. The
    // section's own lift and drag directions still come from the design-pose
    // chord and normal, so this is a first-order statement, exact in the
    // limit of small offsets and wrong by O(offset) in the force direction.
    // At the magnitudes a line network can twist a canopy through - under a
    // degree - that is far below the polar's own uncertainty.
    std::vector<double> sectionIncidenceOffsetRad;
};

// LEVEL 11, STRAND 1: THE WAGNER INDICIAL RESPONSE.
//
// A wing that changes incidence does not change its circulation at the same
// instant. The vorticity it sheds into the wake induces a downwash on itself
// that decays as the wake convects away, so lift arrives gradually: half of it
// immediately, the rest over the next twenty-odd semichords of travel. Every
// solve in this project so far reads the polar instantaneously, which is a
// quasi-steady assumption, and Level 11's work list names removing it.
//
// PUBLISHED METHOD, per guiding rule 13. This is R. T. Jones' two-exponential
// approximation to Wagner's function for a step change in incidence:
//
//     Phi(s) = 1 - 0.165 e^(-0.0455 s) - 0.335 e^(-0.30 s)
//
// with `s` the reduced time in SEMICHORDS TRAVELLED, s = 2 V t / c - not
// seconds. A wing at 11 m/s with a 2.5 m chord covers a semichord in 114 ms, so
// the fast lag is worth tens of milliseconds and the slow one seconds. The
// deviation from Wagner's exact function is under 1% and it is the standard
// engineering form.
//
// Written as two first-order states rather than as a convolution, which is what
// makes it affordable in a flight loop: the exponential form integrates
// exactly, so advancing by any `ds` is a closed-form step and there is no
// history to store. Phi(0) = 0.5 and Phi(inf) = 1 fall out of the states
// starting at zero and ending at the target.
//
// WIRED INTO THE FORCE ASSEMBLY BY STRAND 2, behind `VsmSettings::lagCirculation`.
// The consistency requirement that kept it unwired through strand 1 is real and
// is what the wiring has to satisfy: the VSM builds section forces from a lift
// coefficient and lets circulation enter the induced velocity, so lagging one
// without the other reports a wing whose lift and downwash come from different
// instants. `SolveHeld` therefore takes the lagged circulation as the state and
// derives the lift coefficient back out of it, Cl = 2*Gamma/(c V), rather than
// reading the polar a second time. Lift and downwash are then the same instant
// by construction instead of by agreement.
struct WagnerLag
{
    // The two lag states, in whatever unit the lagged quantity is.
    double slow = 0.0;
    double fast = 0.0;
    bool initialised = false;

    // A wing that has been holding this value forever carries no transient.
    // Starting a simulation mid-flight has to start here, for the same reason
    // the canopy starts trimmed rather than at its hang pose.
    void Settle(double value)
    {
        slow = fast = value;
        initialised = true;
    }

    // Advance by `ds` semichords toward `target` and return the lagged value.
    double Advance(double target, double ds)
    {
        if (!initialised) { Settle(target); return target; }
        if (!(ds > 0.0)) return Value(target);
        slow += (target - slow) * (1.0 - std::exp(-0.0455 * ds));
        fast += (target - fast) * (1.0 - std::exp(-0.30 * ds));
        return Value(target);
    }

    double Value(double target) const
    {
        return target - 0.165 * (target - slow) - 0.335 * (target - fast);
    }
};

// Semichords travelled in `seconds` at `speedMps` on a chord of `chordM`. The
// reduced time Wagner is written in, and the conversion every caller would
// otherwise get wrong in its own way.
inline double ReducedTimeSemichords(double speedMps, double chordM,
                                    double seconds)
{
    if (!(chordM > 1.0e-9)) return 0.0;
    return 2.0 * speedMps * seconds / chordM;
}

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
    // Strand 2's per-section indicial state, used only when
    // `VsmSettings::lagCirculation` is set. Empty otherwise, so a quasi-steady
    // caller carries no cost and no state it does not read.
    std::vector<WagnerLag> circulationLag;
    bool initialised = false;
};

struct VsmSectionResult
{
    double circulation = 0.0;
    // ITEM 30, DIAGNOSTIC ONLY. What the lagged pass aimed AT this tick,
    // before Wagner's response was applied - the target, not the state. Set
    // only under `VsmSettings::lagCirculation`; equal to `circulation` on
    // every quasi-steady path, where the solve converges onto its own target.
    //
    // Here to answer one question and it is worth its own field: item 30
    // measured the wired lag closing 12% of a circulation step where Jones'
    // Phi(0) is 0.5, and there are exactly two places that can come from -
    // the response applied to the target, or the target itself. Nothing else
    // reads this.
    double quasiSteadyCirculation = 0.0;
    double angleOfAttackRad = 0.0;
    double liftCoefficient = 0.0;
    double dragCoefficient = 0.0;
    // The section's own quarter-chord pitching moment coefficient, positive
    // nose-up. Camber makes it negative, and it is a couple rather than the
    // moment of the section's force about anything.
    double momentCoefficient = 0.0;
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

    // ITEM 30. HOW FAR THE TARGET ITSELF CONVERGED, under `lagCirculation`
    // only. -1 where it was not measured, which is every quasi-steady solve
    // and every lagged solve running a single pass - one pass has no
    // pass-to-pass change to report, and reporting 0 there would say
    // "converged" about a quantity nobody looked at.
    //
    // This is NOT `residual` and the distinction is the point. Under lag
    // `residual` is the distance the STATE still has to travel, which is a
    // transient and not an error - a healthy lagged solve reports a large one.
    // Nothing therefore reported whether the target the Wagner step aims at
    // is a converged fixed point or an iterate wandering, and item 30 measured
    // that past the stall it is the second. This is that signal, so the
    // separated regime can be DECLARED at fixed cost rather than discovered
    // downstream in a collapse gate.
    double targetResidual = -1.0;
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
    // Added to every section's drag coefficient. THIS IS AN INSTRUMENT, NOT
    // PHYSICS, and it defaults to zero so that nothing flies on it.
    //
    // `PHYSICS_TODO` item 12 says the solved section runs 0.0157 at trim where
    // paraglider sections are quoted at 0.018-0.025, and names the missing term
    // - the shear layer off the cell mouth - which has been tried twice and
    // left out both times because its size is a dial. This offset is that
    // deficit made adjustable ON PURPOSE, so that a test can ask what the wing
    // would do if the drag were right without anybody having to decide what the
    // missing term is. A coefficient chosen to land the glide is exactly what
    // item 12 refuses to ship; the difference is that this one is set by a test
    // at the moment of asking and is zero everywhere else.
    double sectionDragOffset = 0.0;

    // LEVEL 11, STRAND 2. Make the bound circulation a STATE that integrates
    // forward under Wagner's indicial response, instead of a fixed point solved
    // afresh every tick.
    //
    // What this changes is the OUTER loop only. The inner secant stays: a
    // section's own trailing legs pass half a panel width from its control
    // point, so the self-induced downwash has a gain of chord over panel width
    // and iterating it explicitly would make the answer depend on the mesh.
    // That term is solved implicitly here exactly as it is quasi-steadily, and
    // it is well posed on its own. What is dropped is the global fixed point
    // ACROSS sections, which becomes explicit in time, which is what a state is
    // allowed to be.
    //
    // THIS NOTE USED TO CALL THAT COUPLING "the weak one" AND ITEM 30 MEASURED
    // THAT IT IS NOT. One Jacobi pass across sections closes 0.233 of a
    // circulation step where the iterated solve closes 1.000, so what is
    // dropped here carries three quarters of the answer. The consequence is
    // that the wing is lagged TWICE - once by Wagner and once by this pass -
    // and the unpublished one is the larger: the composite closes 12% of a step
    // where Jones' Phi(0) is 0.5, and Phi(0.076) x 0.233 reproduces it to three
    // decimals. `aerodynamics_tests` measures it, bounded rather than fixed.
    // The self term above really is the strong one and really is implicit; what
    // was wrong is the inference about the remainder. Weak PER ITERATION is not
    // the same as weak IN TOTAL.
    //
    // Item 6 is why this is worth doing rather than a refinement: a wing in deep
    // stall has no stable steady state to find, because the separated branch's
    // negative lift slope inverts the downwash feedback between sections. That
    // is a property of the STEADY solve. A circulation that integrates forward
    // is not asked to find it.
    //
    // DEFAULTS OFF, and the shipped aircraft does not fly on it until the gate
    // in `coupled_tests` says it should - the symmetric frontal holding mirror
    // symmetry through the tick that currently breaks it.
    bool lagCirculation = false;

    // ITEM 30. How many Jacobi passes build the target the Wagner step aims
    // at, per solve. Only read when `lagCirculation` is set.
    //
    // 1 is what strand 2 shipped and is bit-identical to it. It is also
    // measurably wrong: one pass closes 0.233 of a circulation step where the
    // iterated solve closes 1.000, so the wing carries that shortfall on top
    // of Wagner's own lag and the composite bears no resemblance to the
    // published function. See `aerodynamics_tests`.
    //
    // This is NOT the same knob as `maxIterations`, and the difference is the
    // whole point: `maxIterations` is a cap on a loop that stops when it
    // converges, and a solve that wants more of them is a solve in trouble.
    // This is a fixed, declared amount of work done every solve whether or not
    // anything converges - which is what keeps the separated regime, where
    // item 6 says there is no fixed point to find, costing the same as any
    // other and reporting no convergence it does not have.
    int lagTargetPasses = 1;

    // HOW MANY TIMES SLOWER THAN WAGNER'S THE LAG RUNS. THIS IS AN INSTRUMENT,
    // NOT PHYSICS, and it defaults to 1.0 - Wagner's own depth - so nothing
    // flies on it and the default is bit-identical to having no hook.
    //
    // It exists to answer one question, and the question is the one item 30's
    // four-row gate created rather than settled. Every step from strand 2's
    // shipped configuration toward the published function made the symmetric
    // frontal break EARLIER, monotonically: never, then 1.050 s, then 0.250 s,
    // then 0.050 s. Those four rows differ in lag depth among other things, so
    // the obvious reading - that the symmetry was bought by depth alone and
    // not by circulation being a state - is a hypothesis with three points and
    // two confounds.
    //
    // Dividing the reduced time by this scale makes depth the ONLY thing that
    // moves, on an otherwise correct wing: correct elapsed time, converged
    // target, Wagner's own response. If break time is a continuous function of
    // it, "the lagged solve is single-valued" was never a mechanism - it was a
    // threshold, and the price of buying symmetry that way is the number this
    // reports.
    double lagDepthScale = 1.0;

    // THE SAME INSTRUMENT FOR THE OTHER STATE, and it exists because the first
    // one refuted its own hypothesis. `lagDepthScale` above says the frontal's
    // break time barely moves with circulation-lag depth and never holds
    // symmetry at any depth to 64x - so the depth that mattered was not the
    // circulation's.
    //
    // The elapsed-time defect slowed TWO states, not one, and the second is
    // the separation state - the stall memory, which is where item 6's branch
    // ambiguity actually lives. Correcting the elapsed time corrected both at
    // once, so no measurement so far separates them. This scale divides the
    // time handed to the separation update ALONE, which makes the attribution
    // a 2x2 rather than an argument.
    //
    // AN INSTRUMENT, NOT PHYSICS. 1.0 is the real rate and is bit-identical to
    // having no hook; `coupled_tests` is its only caller.
    double separationDepthScale = 1.0;
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
    //
    // THOSE THREE EFFECTS ARE NOT COMPARABLE, and this number spent eleven
    // levels implying they were. Summed off the built suspension graph -
    // `L d sin(theta)` over every cable, which is geometry the solver has had
    // since Level 2 - INCLINATION IS WORTH 0.993. The lines hang canopy to
    // pilot and fan out spanwise, both perpendicular to a horizontal wind, and
    // only the fore-and-aft spread between the A and C rows tilts any of them.
    // So this fraction is not a blend: it is a self-shielding allowance with a
    // rounding error of geometry on top. `PHYSICS_LEARNINGS` §62.
    //
    // Kept as the legacy path, used when `lineProjectedAreaM2` is zero. Prefer
    // the measured area below, which leaves exactly one stated number.
    double lineProjectedFraction = 0.35;
    // The projected line area measured off the graph, m2. Zero means "not
    // measured, use the three numbers above". A solver that has built its own
    // suspension network fills this in, because at that point the length, the
    // diameter and the inclination are all consequences of the wing rather
    // than statements about it.
    double lineProjectedAreaM2 = 0.0;
    // How much of that measured area the cascade shields from itself. THIS IS
    // THE ONE NUMBER IN LINE DRAG THAT IS STILL STATED, which is the whole
    // point of measuring the rest: it is a single flow-physics question with a
    // literature behind it instead of a lumped fudge carrying two negligible
    // effects and one unexamined one.
    //
    // 0.394 is what the old lumped 0.35 amounts to once the geometry is
    // measured - 254.0 x 0.00105 x 0.35 divided by the graph's own projected
    // area - and it is carried forward DELIBERATELY UNCHANGED so that this
    // refactor alters no flight behaviour. It is not evidence. A factor of
    // two and a half from shielding alone is aggressive against published
    // practice on line drag, and justifying or replacing it is item 12's.
    double lineShieldingFactor = 0.394;
    // Seated pilot plus harness, frontal area.
    double harnessAreaM2 = 0.32;
    double harnessDragCoefficient = 1.05;
    // How far below the canopy the harness drag acts. It is a long lever, so
    // this drag is also a pitching moment.
    double harnessBelowCanopyM = 7.8;
    // Extra drag AREA (a Cd times A, in m2) carried on the harness. Like
    // `VsmSettings::sectionDragOffset` this is an instrument rather than
    // physics, defaults to zero, and exists so that one question can be asked:
    // section 55 found that drag added at the CANOPY destabilises the wing,
    // against a classical relation derived for drag at the centre of gravity.
    // The suspect is the 6.6 m moment arm rather than the drag. Adding the same
    // drag down HERE, on the pilot, is the difference between those two, and it
    // is a difference no amount of measuring the canopy case can produce.
    double extraDragAreaM2 = 0.0;
    // How much of that extra drag acts on the PILOT rather than at the canopy.
    // 1 is all of it on the pilot, which is what section 56 measured; 0 puts
    // the same force at the canopy, where its arm about the canopy is zero and
    // it pushes the pendulum bob not at all.
    //
    // This is the sweepable form of "the height at which the drag acts", and it
    // exists because the obvious form is not available: the geometric arm is
    // not a free parameter of a pendulum whose bob is at the end of the link.
    // What IS free is how much of the force pushes the bob, and the resultant's
    // height moves with it. The total force on the system does not change with
    // this fraction, so a sweep holds the drag AND the glide fixed and varies
    // only where it is applied - which is exactly the isolation section 57
    // asked for.
    double extraDragAtPilotFraction = 1.0;
};

struct InstalledDragResult
{
    double lineDragN = 0.0;
    double harnessDragN = 0.0;
    double totalDragN = 0.0;
    // About the canopy quarter-chord centre, body axes.
    Vec3 momentBodyNm{};
    // The two contributions separately, because they do not act on the same
    // body. The lines hang off the canopy and their drag is a moment on it.
    // The harness drag acts on the PILOT, and reaches the canopy only through
    // the lines - so a model with a real pendulum must not apply it to the
    // canopy as well, or the same force pitches the wing twice.
    Vec3 lineMomentBodyNm{};
    Vec3 harnessMomentBodyNm{};
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
    // Induced velocity at control point `i` per unit circulation on section
    // `j`. Read-only, and exposed for one reason: a mirror-symmetric wing must
    // have a mirror-symmetric influence matrix, and until that was checked the
    // claim that this solver treats its two half-spans identically was an
    // assumption. `aerodynamics_tests` checks it BITWISE - see the note there
    // on why non-convergence cannot, on its own, produce a turn direction.
    Vec3 InfluenceAt(std::size_t i, std::size_t j) const
    {
        return Influence[i * SectionList.size() + j];
    }
    double ReferenceAreaM2() const { return ReferenceArea; }
    double ReferenceSpanM() const { return ReferenceSpan; }
    double AspectRatio() const
    {
        return ReferenceSpan * ReferenceSpan / ReferenceArea;
    }

private:
    VortexStepMethodSolver() = default;
    void BuildInfluenceMatrix(double coreFraction);
    // `lag` non-null selects strand 2: one Jacobi pass producing each section's
    // quasi-steady circulation, then a Wagner step of `lagDeltaSeconds` toward
    // it, instead of iterating to a fixed point. `warmCirculation` carries the
    // state in and out in that mode rather than merely seeding it.
    VsmSolution SolveHeld(
        const VsmSolveInput& input, const VsmSettings& settings,
        const std::vector<double>* heldSeparation,
        std::vector<double>* warmCirculation,
        std::vector<WagnerLag>* lag = nullptr,
        double lagDeltaSeconds = 0.0) const;

    std::vector<VsmSection> SectionList;
    SectionPolarTable Polars;
    // Induced velocity at control point i per unit circulation on section j.
    std::vector<Vec3> Influence;
    double ReferenceArea = 0.0;
    double ReferenceSpan = 0.0;
};
}
