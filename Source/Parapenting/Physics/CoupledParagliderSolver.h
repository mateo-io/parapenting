#pragma once

#include "ApparentMassTensor.h"
#include "CanopyCollapseSolver.h"
#include "CanopyGeometry.h"
#include "CanopyMembraneSolver.h"
#include "CanopyPressureSolver.h"
#include "PayloadRigidBody.h"
#include "SectionPolarTable.h"
#include "SuspensionGraph.h"
#include "TensionCableSolver.h"
#include "VortexStepMethodSolver.h"

#include <algorithm>
#include <vector>

namespace Parapenting::Physics
{
// Level 7 of the master plan: run the whole thing, in a defined order, and
// account for it.
//
// Trim works and matches the published wing. Hands off it settles at 10.70 m/s
// (38.5 km/h against a published 39), 1.12 m/s of sink and a glide of 9.5
// against a published 9.5, with net force and moment going to zero. Ten
// minutes holds between 10.55 and 10.70 m/s. Internal force closure is exact
// and energy residual stays under 4 W on a 925 N aircraft.
//
// Asymmetric brake turns, and the suite runs in `Tools/check-build.sh` with
// the rest. Right brake held for ten seconds settles at 0.094 rad of bank and
// 0.030 rad/s, its mirror image agrees to 2e-8, and heavy brake walks the wing
// into a fully separated 46-degree stall at 2.8 m/s of sink without the
// numerical safety envelope having to engage at all.
//
// Six bugs were found and fixed getting here, all recorded below: rotational
// damping held across the aerodynamic interval, the pendulum restoring moment
// missing entirely, a pressure state that started packed while the flight
// state started flying - that one caused the stall - and then the three that
// made asymmetric brake diverge to a NaN turn rate: damping integrated
// explicitly at a coefficient eleven times its stability limit, a damping
// derivative measured by dividing by whatever rate the wing happened to have,
// and an aerodynamic gate that bounded the force it accepted but not the
// moment.
//
// Levels 1 to 6 built six solvers that each answer their own question well and
// have never been asked to agree. This is the order they run in and the
// bookkeeping that proves nothing is invented between them:
//
//   1. pilot controls and rest-length changes
//   2. atmospheric sampling
//   3. aerodynamics    - VSM, at a reduced rate with interpolation between
//   4. pressure        - cells fed through the inlets the VSM's incidence moves
//   5. membrane        - the skin under that pressure
//   6. collapse        - Level 8's pressure balance across the nose, which
//                        reads the four levels above it and hands back a
//                        per-section fold that takes the pressure out of the
//                        cell and the load off the lines
//   7. lines           - the suspension network, warm-started
//   8. rigid motion    - canopy and payload
//
// Three of those are expensive and none of them needs to run at 120 Hz to be
// right. The VSM changes only as fast as the wing's attitude does, so it runs
// at a reduced rate and its loads are held between solves; the line network is
// continued from the previous step rather than relaxed from its design pose;
// the membrane runs at a few chord stations rather than all of them. Each rate
// is a declared number that the convergence tests vary, not something tuned
// until it looked right.
//
// The aerodynamic and structural sides are coupled in a staggered scheme, the
// way the ram-air kite FSI literature does it: solve the aero, solve the
// structure, feed the structure's new shape back to the aero, and repeat with
// relaxation on the exchanged load. The relaxation is not optional. Weakly
// coupled FSI on a light structure in dense flow is unstable through the
// added-mass mechanism, and a paraglider - four kilos per square metre,
// dragging a third of its own mass in air - is exactly the case that breaks.

struct CoupledSchedule
{
    // Physics rate. Fixed, and decoupled from the engine tick since Level 0.
    double timeStepS = 1.0 / 120.0;
    // Steps between full aerodynamic solves. At 120 Hz, 12 gives 10 Hz.
    int aerodynamicsInterval = 12;
    // Chord stations the membrane is solved at.
    int membraneStations = 3;
    // Staggered coupling iterations per step. The convergence gate is that
    // taking one away changes nothing qualitative.
    int couplingIterations = 3;
    // Relaxation on the load handed from aero to structure. One would be no
    // relaxation at all, which is the unstable case.
    double couplingRelaxation = 0.5;
    // Iterations the warm-started line network gets per step.
    int suspensionIterations = 120;
    // Iterations the two FROZEN VSM solves get - the rotation-free solve for
    // the moment, and the rate probes for the damping derivative. Together
    // they are 30% of a step against the unsteady solve's 6%, entirely
    // because of this number.
    //
    // It is 600 for a reason and the reason is recorded: run cold at the
    // unsteady solve's 40, the probes were not converged and not even the same
    // wing, the derivative they returned moved 10% between intervals, and two
    // mirror-image flights measured different damping. See PHYSICS_LEARNINGS
    // section 28. They are warm-started now, which is a different question -
    // but it is a question to MEASURE before lowering this, not to assume.
    int frozenSolveIterations = 600;
    // Aerodynamic ticks between damping-derivative probes. 1 probes one axis
    // every tick, so each axis refreshes every 3 ticks - 0.3 s at the default
    // schedule.
    //
    // This, and NOT the iteration cap above, is the lever on the probes' 23.8%
    // of a step. Measured: dropping the cap from 600 to 40 saves 0-4%, inside
    // the noise, because the warm-started probes converge and exit long before
    // the cap. What costs is running two extra frozen solves at all.
    int dampingProbeInterval = 1;
};

// The two schedules that have been measured against each other. Anything else
// is a set of loose knobs, and a tier nobody has swept is a guess.
//
// FULL is the default and the reference: every published number and every
// calibration gate is measured on it, and it is what a disagreement means.
//
// REDUCED is for the cases Level 10's profile identified - weaker machines,
// more than one aircraft in the air, faster-than-real-time research runs - and
// it moves exactly two knobs, both chosen off `solver_lod`'s sweep at the knee
// where the saving flattens and the error starts to climb:
//
//   * suspensionIterations 120 -> 40. The flight signature is unchanged to
//     three decimals; what moves is the network's own residual, 0.140 N to
//     0.198 N against a flight load near 1030 N. Below 40 the saving flattens
//     (40% at 40 iterations, 49% at 20) while the residual does not (0.198 to
//     0.384), which is what makes 40 the knee rather than a preference.
//   * dampingProbeInterval 1 -> 3. Each axis then refreshes every 0.9 s
//     instead of every 0.3 s.
//
// Two knobs are deliberately NOT moved. `couplingIterations` stays at 3
// because 2 changes the peak of a 4 m/s asymmetric collapse from 0.648 to
// 0.691 - 6.5%, on the number a pilot is judged on - for 21%.
// `frozenSolveIterations` stays at 600 because lowering it buys nothing: the
// warm-started probes converge and exit long before the cap, so 600 to 40 is
// worth 0-6%, inside the run-to-run noise.
CoupledSchedule FullFidelitySchedule();
CoupledSchedule ReducedFidelitySchedule();

struct CoupledControls
{
    double leftBrake = 0.0;
    double rightBrake = 0.0;
    double weightShift = 0.0;
    double accelerator = 0.0;
};

struct CoupledAtmosphere
{
    Vec3 windWorldMps{};
    double densityKgM3 = 1.12;

    // An incident gust, world axes, acting over part of the span. Level 8's
    // asymmetric benchmark needs this and there is no substitute for it: the
    // wind above changes the frame the whole wing flies in and cannot fold
    // anything, and a roll rate is the wing's own state rather than something
    // happening to it. Air arriving at one half and not the other is what a
    // rotor edge, a thermal wall and a gust front all are.
    Vec3 gustWorldMps{};
    // Span fractions the gust covers, -1 at the left tip to +1 at the right.
    // The default covers the wing, which is the symmetric case.
    double gustSpanFrom = -1.0;
    double gustSpanTo = 1.0;
};

struct CoupledState
{
    // Canopy rigid body, in world axes.
    Vec3 positionWorldM{0.0, 0.0, 1000.0};
    Vec3 velocityWorldMps{10.8, 0.0, -1.15};
    Quaternion attitude{};
    Vec3 angularVelocityBodyRadps{};

    // Subsystem states, carried between steps.
    PayloadState payload;
    CellPressureState pressure;
    VsmSeparationState separation;
    SuspensionWarmStart suspension;
    // The line link, canopy to payload, as a unit vector in WORLD axes, with
    // the link's own angular velocity about the canopy.
    //
    // This used to be a single angle in CANOPY BODY axes, and that was the
    // whole of PHYSICS_TODO item 10. A body-frame angle is not a degree of
    // freedom of the pilot: rotating the canopy carried the pilot round with
    // it, so gravity's restoring torque appeared once in the swing equation
    // and once again as the lumped body's weight moment, and the wing had
    // roughly 14000 Nm/rad of pitch stiffness where the lines provide 6300.
    //
    // Held in world axes the link knows nothing about the canopy except
    // through the lines. It hangs along apparent gravity, the canopy hangs off
    // it through the measured line springs, and each restoring torque is
    // written exactly once. The body-relative angles the lines actually see
    // are read back out of it every step.
    Vec3 payloadDirWorld{0.0, 0.0, -1.0};
    Vec3 linkRateWorldRadps{};

    // Level 8. Stepped every physics step, because a fold takes about a tenth
    // of a second and the aerodynamic interval is a tenth of a second.
    CollapseState collapse;
    // What the collapse solver was last told about each section. Refreshed
    // whenever the aerodynamics run, so a fold between aerodynamic solves
    // continues from the state that produced it rather than from nothing.
    std::vector<SectionCollapseInput> collapseInput;

    // Aerodynamic load held between full solves, split into the part that
    // depends on the wing's attitude and circulation - which changes slowly
    // and can be held - and the part that depends on its rotation rate, which
    // cannot.
    //
    // Roll damping has a time constant of about 20 ms on this wing, so
    // holding it across a 100 ms aerodynamic interval applies a damping
    // moment computed at a rate five time constants stale. That is not a
    // small error: it turns damping into excitation and the wing diverges in
    // under five seconds. The rate-dependent part is therefore re-evaluated
    // every step from a derivative measured at each full solve, which is the
    // cheap per-step interpolation the plan asks for.
    //
    // That derivative also has to be integrated implicitly. Roll damping runs
    // to 3.7e3 Nm per rad/s against an inertia of 95 - a time constant of
    // 26 ms - and yaw damping is stiffer still relative to its inertia. An
    // explicit c*omega at 120 Hz overshoots, alternates sign and doubles every
    // step, which is where the asymmetric-brake NaN came from.
    Vec3 heldAeroForceBodyN{};
    Vec3 heldAeroMomentBodyNm{};
    Vec3 rotationalDampingNmPerRadps{};
    // Which axis the next full solve probes for its damping derivative. One
    // per aerodynamic interval, round-robin, so each is refreshed every 0.3 s.
    int dampingProbeAxis = 0;
    // Aerodynamic ticks since the last probe, against
    // `CoupledSchedule::dampingProbeInterval`.
    int aeroTicksSinceDampingProbe = 0;
    // Circulation carried for the rotation-free solve and the two damping
    // probes, so each continues from its own last answer instead of being
    // solved cold every interval.
    std::vector<double> stationaryCirculation;
    std::vector<double> forwardProbeCirculation;
    std::vector<double> backwardProbeCirculation;
    double heldPressureCoefficientMean = 1.0;
    std::vector<double> heldPressureCoefficient;
    int stepsSinceAerodynamics = 1 << 20;
    bool initialised = false;
};

struct CoupledDiagnostics
{
    // What each subsystem contributed, so an unaccounted force has somewhere
    // to be found.
    Vec3 aeroForceBodyN{};
    Vec3 lineForceBodyN{};
    Vec3 weightForceWorldN{};
    // Everything applied to the system, summed. In steady flight this is the
    // net accelerating force and nothing else - a subsystem inventing force
    // shows up here first.
    Vec3 netForceWorldN{};
    Vec3 netMomentBodyNm{};

    // Closure: every internal reaction must appear twice with opposite sign.
    // This is what "equal and opposite" means when six solvers are involved.
    double internalForceClosureN = 0.0;
    double internalMomentClosureNm = 0.0;

    // Energy. Work done by the aerodynamic force against the change in
    // kinetic plus potential energy, per second. A solver creating energy
    // shows here.
    double energyResidualW = 0.0;
    double kineticEnergyJ = 0.0;
    double potentialEnergyJ = 0.0;

    // How far the last staggered iteration moved the exchanged load, relative
    // to the load itself. Small means the coupling has converged within the
    // step rather than merely run out of iterations.
    double couplingResidual = 0.0;
    int couplingIterationsUsed = 0;

    // Passed through from the subsystems that report their own.
    double suspensionResidualN = 0.0;
    double membraneStrain = 0.0;
    double meanPressureCoefficient = 0.0;
    double vsmResidual = 0.0;
    bool vsmConverged = false;
    bool aerodynamicsSolvedThisStep = false;
    // Set when a solve was rejected and the previous load held instead. This
    // is a numerical safety envelope, not flight behaviour (guiding rule 12):
    // it is reported, it is separate, and if it engages during a manoeuvre
    // that manoeuvre's numbers mean nothing.
    bool aerodynamicsRejected = false;

    // Level 8. What the canopy is doing to itself, and what that is doing to
    // the flight. None of these is a mode: they are span-weighted sums over
    // per-section states that came out of a pressure balance.
    CollapseResult collapseState;
    // The load imbalance the collapse handed to the line network this step,
    // positive when the right half is carrying more. This is the path from a
    // folded half wing to slack lines on that side.
    double collapseLoadAsymmetry = 0.0;

    // The pendulum between the wing and the pilot. Positive swing is the pilot
    // ahead of the wing; positive surge rate is the wing coming forward past
    // the pilot, which is what a pilot means by the word.
    double payloadSwingRad = 0.0;
    double payloadSwingRateRadps = 0.0;
    // The same angle in the roll plane: positive when the pilot hangs out to
    // the RIGHT of the canopy's own down. In a coordinated turn this is what
    // goes to zero - the link lies along apparent gravity - which is why a
    // banked wing has no gravity spring trying to level it.
    double payloadSwingLateralRad = 0.0;
    // How far ahead of the pilot the canopy is, metres along track. The same
    // angle in the units the rendering needs, and the one a pilot can see.
    double canopyLeadM = 0.0;
    // The moment the lines' fore-aft spread puts on the canopy. Zero when the
    // pilot hangs where the lines are unstressed, and the wing's entire pitch
    // stiffness otherwise.
    double linePitchMomentNm = 0.0;
    // Its roll-plane counterpart, off the same graph by the same probe.
    double lineRollMomentNm = 0.0;
    // The stiffnesses those two moments were computed at, this step. Not
    // constants: the line spring is a geometric one, so it scales with the
    // load the wing is carrying, and reporting it is how that stays visible.
    double linePitchStiffnessNmPerRad = 0.0;
    double lineRollStiffnessNmPerRad = 0.0;
    // The payload's weight acting on its arm about the canopy. Reported and
    // NOT applied: the link carries the payload's weight along its own axis,
    // so the only thing the canopy feels from the pilot is line tension
    // through the attachments, which is the spring above. Kept as a
    // diagnostic because "the term we removed is small at equilibrium" is a
    // claim worth measuring rather than asserting.
    double pendulumWeightMomentNm = 0.0;
    double aeroPitchMomentNm = 0.0;
    // The nose-up rotation the shortened brake line commands geometrically,
    // before the section's nose-down couple has argued with it. Item 11 is the
    // gap between this and `payloadSwingRad`.
    double brakeCommandedSwingRad = 0.0;
    // Power leaving through the pendulum's damper. A real sink rather than a
    // solver artefact, so the energy accounting subtracts it instead of
    // reporting it as energy that went missing.
    double swingDampingPowerW = 0.0;

    // Flight numbers, for the tests to read.
    double airspeedMps = 0.0;
    double angleOfAttackRad = 0.0;
    double bankRad = 0.0;
    double turnRateRadps = 0.0;
    double leftCarabinerLoadN = 0.0;
    double rightCarabinerLoadN = 0.0;
};

// Where a step's wall clock goes, in nanoseconds, accumulated over however
// many steps were run. Level 10 needs this before it can pick solver levels of
// detail: a cheaper tier is a guess until the expensive stage is known.
//
// The three VSM entries are separate because they are separate solves with
// different iteration caps, not three parts of one. An aerodynamic tick runs
// FOUR: the unsteady solve the wing actually flies on, a rotation-free solve
// for the moment, and two rate probes for the damping derivative - and the
// last three are capped at 600 iterations where the first is capped at 40.
struct CoupledStepProfile
{
    long long vsmUnsteadyNs = 0;
    long long vsmStationaryNs = 0;
    long long vsmDampingProbeNs = 0;
    long long pressureNs = 0;
    long long membraneNs = 0;
    long long collapseNs = 0;
    long long suspensionNs = 0;
    long long rigidMotionNs = 0;
    long long totalNs = 0;
    long long steps = 0;
    long long aeroTicks = 0;

    long long VsmTotalNs() const
        { return vsmUnsteadyNs + vsmStationaryNs + vsmDampingProbeNs; }
    // What the stages add up to, against `totalNs`. The gap is the step's own
    // bookkeeping - the vectors it fills, the diagnostics it writes - and it is
    // reported rather than distributed, because a large gap means the timers
    // are in the wrong places and that should be visible.
    long long AccountedNs() const
    {
        return VsmTotalNs() + pressureNs + membraneNs + collapseNs
            + suspensionNs + rigidMotionNs;
    }
};

// How hard the CONSTRUCTION-time probes relax the suspension network.
// Not a flight setting: these are the once-per-wing measurements of the
// line stiffness curve, the hang pose, the accelerated pose and the brake
// swing curve, and they are the whole of what makes building a solver
// cost more than a millisecond.
//
// Exposed because the shipped values had to be justified against a
// reference rather than chosen, and a number that cannot be varied cannot
// be justified. `suspension_tests` sweeps it; nothing in flight sets it.
// See the comment on `MeasureLineStiffness` for why 4000 at 0.995 beats
// 12000 at 0.999 on accuracy as well as on time.
struct ConstructionProbe
{
    // TWO settings, because the construction sequence asks the network two
    // different questions and they converge at different speeds.
    //
    // SHIPPED VALUES ARE THE OLD ONES, DELIBERATELY, and the reason is the
    // whole of what this hook was added to find out.
    //
    // HELD solves impose the canopy's attitude and read the moment the lines
    // exert - the stiffness probes, 24 of the 35 solves. Only the network's
    // SHAPE has to settle, and it is not settling: it is RINGING. Held at
    // 0.02 rad the pitch probe passes +177%, -176%, +27% and -13% of its
    // converged value at 500, 1000, 2000 and 4000 iterations. Damping that
    // ring - fewer iterations at lower velocity retention - lands every static
    // output closer to a 48000-iteration reference than the shipped settings
    // do, and costs a quarter to a third less time:
    //
    //                       pitch k / roll k at 1 g      worst over 0.5-4 g
    //   reference 48000      5749.8 / 8253.6                    -
    //   shipped 12000/.999   5739.3 / 8116.4              1.66% (roll, 1 g)
    //   held 6000/.995       5750.4 / 8242.1              1.40% (pitch, 0.5 g)
    //   held 8000/.997       5750.4 / 8261.8              0.24%
    //
    // AND IT IS NOT SHIPPED, because two flight gates say the converged model
    // is not the one the gates were written against - including the reference
    // itself, which fails them too. See `PHYSICS_LEARNINGS` §52 and
    // `PHYSICS_TODO` item 14. Changing these defaults is a modelling decision
    // about two known-limitation events, not a performance tuning, and it is
    // left to whoever takes that decision.
    //
    // FREE solves let the canopy rotate to where the lines balance - the hang
    // pose, the accelerated pose, the brake swing curve. They carry a slow
    // ROTATIONAL mode that the same change makes converge more SLOWLY: at
    // 4000/0.995 the trim incidence lands 0.198 degrees high, and on this
    // aircraft a hundredth of a degree moved a held collapse from 0.83 to 0.30
    // of fold.
    int heldIterations = 12000;
    double heldRetention = 0.999;
    int freeIterations = 12000;
    double freeRetention = 0.999;
};

class CoupledParagliderSolver
{
public:
    // The payload is a parameter, not a constant, and Level 9 is why. A wing's
    // published envelope is quoted at ONE all-up weight - the EPIC 2 ML's
    // 39 km/h is a 105 kg figure against a 90-110 kg certified range - and
    // this solver's default payload comes to 94.3 kg. Comparing the two
    // without saying so is a 5.5% speed error built into the comparison
    // rather than into the model, because trim speed goes as the square root
    // of wing loading. Flying the configuration the numbers were published at
    // is part of the measurement.
    CoupledParagliderSolver(
        const CanopyGeometry& geometry, const LinePlanSpec& linePlan,
        const CoupledSchedule& schedule = {},
        const PayloadMassProperties& payload = {},
        const ConstructionProbe& probe = {});

    void Step(CoupledState& state, const CoupledControls& controls,
              const CoupledAtmosphere& atmosphere);

    const CoupledDiagnostics& Diagnostics() const { return LastDiagnostics; }
    const CoupledSchedule& Schedule() const { return ScheduleValue; }

    // Wall-clock accounting for one step, by stage. Off by default and costing
    // one predictable branch per stage when off, because the thing being
    // measured here is tens of microseconds and a clock read is tens of
    // nanoseconds - always-on timers would be a tenth of a percent of the
    // answer, which is fine, but they would also be a permanent claim on a hot
    // loop for a number nobody reads in flight.
    // Item 11's only free number, 0.35, registered Tuned/Unvalidated. Exposed
    // for `pitch_axis_trace` to sweep; nothing in flight changes it.
    void SetSwingDampingRatio(double ratio) { SwingDampingRatioValue = ratio; }
    double SwingDampingRatio() const { return SwingDampingRatioValue; }

    // WHAT THE LINK'S DAMPER IS MEASURED AGAINST. `World` is what flies and is
    // the default; `Canopy` is the alternative the solver's own comment
    // describes as where the friction physically sits - lines, maillons and a
    // harness resisting the wing and the pilot moving with respect to each
    // other - and which was tried once, flown for twenty seconds, and rejected
    // on the departure rather than on a measurement.
    //
    // This exists so that rejection can be DIFFERENCED instead of repeated.
    // `PHYSICS_TODO` item 11 carried it for four levels as "still unclaimed
    // either way: differencing its matrix would test whether the two failure
    // modes are separable, but it needs a solver hook that does not exist".
    // Nothing in flight sets this; `pitch_eigenmodes --damper` is its only
    // caller, and the default is bit-identical to having no hook at all.
    enum class LinkDampingReference { World, Canopy };
    void SetLinkDampingReference(LinkDampingReference reference)
        { LinkDampingReferenceValue = reference; }
    LinkDampingReference DampingReference() const
        { return LinkDampingReferenceValue; }

    // THE DRAG THE SECTION IS MISSING, made adjustable so that item 12 can be
    // pointed at item 11. Added to every section's drag coefficient; zero by
    // default, so nothing in flight is touched and the default is bit-identical
    // to having no hook.
    //
    // Item 12 is the largest disagreement in the model - the solved section
    // runs 0.0157 at trim against a published 0.018-0.025, and glide 10.96
    // against 9.5 - and item 11 needs a stabilising mechanism a wing term could
    // supply. A phugoid's damping is classically CD/CL over root two, so a wing
    // whose drag is a sixth low has a phugoid whose damping is a sixth low, and
    // `swingDampingRatio` may be paying for it. That is a testable sentence
    // only if the drag can be moved, which is what this is for.
    // `pitch_eigenmodes --drag` is its only caller.
    void SetSectionDragOffset(double offset) { SectionDragOffsetValue = offset; }
    double SectionDragOffset() const { return SectionDragOffsetValue; }

    // How many iterations the FLIGHT solve may take once it is warm. THIS IS
    // AN INSTRUMENT, NOT PHYSICS, and it defaults to the shipped 40 so that
    // nothing flies on a different number by accident.
    //
    // It exists to make one sentence testable. §67 measured the symmetric
    // frontal losing its mirror symmetry on a single aerodynamic tick, and
    // §68 traced that tick to the VSM residual jumping four orders - from
    // 1.1e-06 to 2.2e-02 - at exactly the step the incidence field stops being
    // symmetric. That is either a solve that ran out of iterations, or a solve
    // with nothing to converge TO. The two need different fixes and the
    // difference is worth more than an argument: raising this cap separates
    // them, because iterations cure the first and cannot touch the second.
    // `coupled_tests` is its only caller.
    void SetFlightSolveIterationCap(int iterations)
        { FlightSolveIterationCapValue = std::max(1, iterations); }
    int FlightSolveIterationCap() const { return FlightSolveIterationCapValue; }

    // The same extra drag, moved to the pilot. Units are Cd times A, m2, added
    // to the harness. Zero by default; `pitch_eigenmodes --height` is its only
    // caller, and it is the other half of `SetSectionDragOffset` above.
    void SetHarnessExtraDragAreaM2(double area)
        { InstalledDrag.extraDragAreaM2 = area; }
    double HarnessExtraDragAreaM2() const
        { return InstalledDrag.extraDragAreaM2; }

    // How much of that extra drag pushes the pendulum bob rather than acting at
    // the canopy. 1 by default, which is the harness wing of §56; sweeping it to
    // 0 holds the force and the glide fixed and moves only where it is applied.
    void SetHarnessExtraDragAtPilotFraction(double fraction)
        { InstalledDrag.extraDragAtPilotFraction = fraction; }

    // WHAT AIRFLOW THE PILOT'S DRAG IS EVALUATED AGAINST. `Aircraft` is what
    // flies and is the default; `Pilot` adds the bob's own rotation about the
    // hinge, w x r, so the drag opposes the SWING as well as the flight path.
    //
    // This is the first of the two terms item 11's opening estimate names -
    // "the pilot's own drag on an 8 m arm, plus the lines sweeping" - and §59
    // found that neither was ever in this solver, which is why a coefficient
    // has been standing in for them since Level 3. It is not a dial: the arm
    // and the rate are geometry the solver already has. It defaults off only
    // because turning it on changes what the aircraft does, and that is a
    // deliberate step to take on measurement rather than on a first run.
    enum class HarnessDragReference { Aircraft, Pilot };
    void SetHarnessDragReference(HarnessDragReference reference)
        { HarnessDragReferenceValue = reference; }
    HarnessDragReference DragReference() const
        { return HarnessDragReferenceValue; }

    // THE LINES SWEEPING - the second of the two terms item 11's opening
    // estimate names, and the other one §59 found missing. A line at distance s
    // from the hinge sweeps at s*qdot when the pilot swings, and its drag
    // opposes that, so the cascade is a rotational damper about the hinge. The
    // shipped model applies line drag as a canopy moment from the flight-path
    // airspeed only, so this is absent exactly as the harness term was.
    //
    // Off by default for the same reason as the harness reference: it changes
    // what the aircraft does. §60's arithmetic predicts it is worth about 7% of
    // a term itself worth 0.01 of coefficient, so this is a CLOSURE
    // measurement - it cannot change item 11's conclusion, it can only confirm
    // or refute the estimate that dismissed it.
    void SetLineSweepDamping(bool on) { LineSweepDamping = on; }
    bool LineSweepDampingEnabled() const { return LineSweepDamping; }

    void SetProfiling(bool on) { Profiling = on; }
    const CoupledStepProfile& Profile() const { return ProfileValue; }
    void ResetProfile() { ProfileValue = CoupledStepProfile{}; }

    void SetSchedule(const CoupledSchedule& schedule)
        { ScheduleValue = schedule; }

    double AllUpMassKg() const { return SystemMassKg; }

    // What each section carries in clean hands-up trim, newtons. The reference
    // the collapse criterion's local unloading is measured against.
    const std::vector<double>& TrimLoadDistributionN() const
        { return SectionTrimLoadN; }

    // What the lines do to the canopy at a given load, measured off the built
    // graph by rotating the canopy either side of its free equilibrium at that
    // load and reading the moment back.
    //
    // The load is the whole point. This spring is not elastic - the lines
    // stretch by 0.2% under a 0.02 rad rotation and the canopy's origin moves
    // 0.13 m, so what is happening is the wing pivoting about a virtual hinge
    // roughly six metres below itself, and the restoring moment is a tension
    // times an arm. Measured across half a g to three g the stiffness runs
    // 3306, 6317, 11512 and 15393 Nm/rad: proportional to load to within the
    // slow droop of the arm.
    //
    // Freezing it at its 1 g value, which is what this solver did, is what
    // made the wing's pitch diverge once the double-counted gravity spring of
    // item 10 was removed. The aerodynamic moment scales with dynamic pressure;
    // a spring that does not scale with anything loses to it at speed, and the
    // incidence runs away. Load-proportional, the two scale together and the
    // trim incidence depends on lift coefficient alone, which is what makes a
    // fixed-incidence glider speed-stable.
    struct LineStiffness
    {
        double pitchNmPerRad = 0.0;
        double rollNmPerRad = 0.0;
        double hangIncidenceRad = 0.0;
        // How far the canopy's origin travels per radian it rotates through.
        // The wing does not spin about itself: the network holds it on a
        // virtual hinge 6.6 m below, so rotating it also SWINGS it, and both
        // the inertia that resists the rotation and the relative wind the
        // sections see follow from that arm.
        double pitchHingeArmM = 0.0;
        double rollHingeArmM = 0.0;
    };
    LineStiffness LineStiffnessAt(double loadN) const;

    // At one g, which is the number to compare against anything published and
    // the one the tests read.
    double PitchStiffnessNmPerRad() const
        { return LineStiffnessAt(SystemMassKg * 9.80665).pitchNmPerRad; }
    // The same measurement in the roll plane. A wing hanging from a single
    // point would have none; this one has the two carabiners' lateral spread
    // and the whole span of attachments above them.
    double RollStiffnessNmPerRad() const
        { return LineStiffnessAt(SystemMassKg * 9.80665).rollNmPerRad; }
    // The incidence the wing hangs at with no aerodynamic moment. The zero of
    // that spring.
    double TrimIncidenceRad() const { return TrimLineIncidenceRad; }

private:
    // Measured stiffness against line load, in ascending load order.
    struct StiffnessSample
    {
        double loadN = 0.0;
        LineStiffness stiffness;
    };
    std::vector<StiffnessSample> StiffnessCurve;
    // The virtual hinge arms, which are geometry rather than load, so they are
    // taken once off the built graph and used everywhere.
    double PitchHingeArmM = 0.0;
    double RollHingeArmM = 0.0;
    double TrimLineIncidenceRad = 0.0;
    double TrimSwingRad = 0.0;
    double AcceleratedSwingRad = 0.0;
    // How far brake rotates the wing on its lines, against handle travel.
    // Sampled at explicit stations rather than evenly, because the sewn-in
    // slack puts a corner in this curve at 19% of travel and an even spacing
    // interpolates straight across it.
    struct BrakeSwingSample
    {
        double travel = 0.0;
        double offsetRad = 0.0;
    };
    ConstructionProbe ConstructionProbeSettings;
    double SwingDampingRatioValue = 0.35;
    double SectionDragOffsetValue = 0.0;
    int FlightSolveIterationCapValue = 40;
    HarnessDragReference HarnessDragReferenceValue =
        HarnessDragReference::Aircraft;
    bool LineSweepDamping = false;
    // Sum of d * L * s^2 over the cascade, m^4, where s is a cable's distance
    // from the hinge. Times 0.5 rho Cd V it is a damping torque per unit swing
    // rate. Measured off the graph at construction, like the drag area.
    double LineSweepAreaMomentM4 = 0.0;
    LinkDampingReference LinkDampingReferenceValue =
        LinkDampingReference::World;
    bool Profiling = false;
    CoupledStepProfile ProfileValue{};
    std::vector<BrakeSwingSample> BrakeSwingCurve;
    double BrakeSwingOffsetRad(double travel) const;
    // The section the ribs are cut to, and the mean chord at the span
    // stations the brake fan actually lands on - both taken off the geometry
    // at construction. Together they say how much line the trailing edge
    // bends with, which is line the canopy does not rotate with.
    SectionProfileSpec BrakeSection{};
    double BrakeStationChordM = 0.0;
    // Handle travel to trailing-edge deflection, with the sewn-in slack
    // removed. The aerodynamics and the line network must be driven by the
    // same one of these or brake is counted twice.
    double EngagedBrake(double handTravel) const;
    double BrakeFlapTakeUpM(double handTravel) const;
    void SolveTrimLoadDistribution();
    void MeasureLineStiffness();

    CoupledSchedule ScheduleValue;
    VortexStepMethodSolver Aerodynamics;
    CanopyPressureSolver Pressure;
    CanopyMembraneSolver Membrane;
    SuspensionGraph Lines;
    // Declared after the graph it is measured against: the collapse solver
    // solves the aerodynamic sections, and the room a fold has at each of them
    // comes off the suspension graph.
    CanopyCollapseSolver Collapse;
    SectionPolarTable Polars;
    // Per-section room a fold has before it is past the line beside it,
    // measured off the built graph at construction. Geometry, so it is fixed.
    std::vector<double> SectionLineGapM;
    // What each section carries in clean hands-up trim, newtons normal to the
    // section. This is the reference the collapse criterion's "has this
    // section stopped carrying load" is measured against, and it has to be the
    // wing's own span distribution rather than its weight spread evenly: the
    // loading on an arced, tapered wing falls away toward the tips by a factor
    // of three, so an even share reads every healthy tip as half unloaded and
    // folds it. Solved once at construction, from the same solver that flies.
    std::vector<double> SectionTrimLoadN;
    ApparentMassTensor ApparentMass;
    PayloadMassProperties PayloadMass;
    HarnessGeometry Harness;
    InstalledDragSpec InstalledDrag;

    double CanopyMassKg = 5.1;
    double SystemMassKg = 105.0;
    double ReferenceAreaM2 = 27.0;
    double ReferenceSpanM = 11.8;
    double PendulumLengthM = 8.08;
    CoupledDiagnostics LastDiagnostics;
};
}
