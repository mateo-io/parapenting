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
};

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
    // Where the pilot is hanging, relative to straight below the canopy.
    // Radians, positive when the pilot is AHEAD of the wing - which is what
    // pulling brake does, and the surge is the wing coming back past it.
    //
    // This was not a degree of freedom at all until now: the payload was
    // pinned straight below the canopy in body axes, so the wing could pitch
    // but it could never swing. A paraglider is two masses on a 7 m line and
    // almost everything a pilot feels in pitch is the angle between them -
    // the dive on release, the surge out of a collapse, the pendulum that
    // makes a brake input arrive late. None of that could happen.
    double payloadSwingRad = 0.0;
    double payloadSwingRateRadps = 0.0;

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
    // How far ahead of the pilot the canopy is, metres along track. The same
    // angle in the units the rendering needs, and the one a pilot can see.
    double canopyLeadM = 0.0;
    // The moment the lines' fore-aft spread puts on the canopy. Zero when the
    // pilot hangs where the lines are unstressed, and the wing's entire pitch
    // stiffness otherwise.
    double linePitchMomentNm = 0.0;
    // The payload's weight acting on its arm. Near zero at equilibrium by
    // construction - the line hangs along apparent gravity, so the weight acts
    // through the attachment - and reported so that "near zero" is checked
    // rather than assumed.
    double pendulumWeightMomentNm = 0.0;
    double aeroPitchMomentNm = 0.0;
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

class CoupledParagliderSolver
{
public:
    CoupledParagliderSolver(
        const CanopyGeometry& geometry, const LinePlanSpec& linePlan,
        const CoupledSchedule& schedule = {});

    void Step(CoupledState& state, const CoupledControls& controls,
              const CoupledAtmosphere& atmosphere);

    const CoupledDiagnostics& Diagnostics() const { return LastDiagnostics; }
    const CoupledSchedule& Schedule() const { return ScheduleValue; }
    void SetSchedule(const CoupledSchedule& schedule)
        { ScheduleValue = schedule; }

    double AllUpMassKg() const { return SystemMassKg; }

    // What each section carries in clean hands-up trim, newtons. The reference
    // the collapse criterion's local unloading is measured against.
    const std::vector<double>& TrimLoadDistributionN() const
        { return SectionTrimLoadN; }

    // The wing's pitch stiffness, measured off the built suspension graph by
    // rotating the canopy either side of its free equilibrium and reading the
    // moment the lines exert. Newton-metres per radian.
    double PitchStiffnessNmPerRad() const { return LinePitchStiffnessNmPerRad; }
    // The incidence the wing hangs at with no aerodynamic moment, from the
    // same solve. The zero of that spring.
    double TrimIncidenceRad() const { return TrimLineIncidenceRad; }

private:
    double LinePitchStiffnessNmPerRad = 0.0;
    double TrimLineIncidenceRad = 0.0;
    double TrimSwingRad = 0.0;
    double AcceleratedSwingRad = 0.0;
    void SolveTrimLoadDistribution();
    void MeasureLinePitchStiffness();

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
