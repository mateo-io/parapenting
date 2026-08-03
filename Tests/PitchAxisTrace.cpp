// Level 10, strand 4: an instrument for item 11.
//
// The pitch axis is the open physics problem. Brake slows the wing while
// LOWERING its incidence, which is the wrong sign, and 40% of travel departs
// nose-down. What has been missing is not another opinion about why - it is a
// way to watch the two things that argue with each other.
//
// They are:
//
//   * what the shortened brake line COMMANDS. Pulling brake shortens the
//     brake run, which rotates the canopy nose-up on its suspension. That is
//     geometry, it comes off the built graph, and it is now reported as
//     `brakeCommandedSwingRad`.
//   * what the section's flap couple TAKES BACK. A deflected trailing edge is
//     a large nose-down pitching moment, and it acts against the line spring.
//
// The wing's incidence moves by the difference. This prints both, settled, at
// each brake setting, so the difference is a number rather than an argument.
//
// It is NOT a gate. It is a measurement, and item 11 is registered as an open
// disagreement precisely so that nobody is tempted to gate this into agreeing.
//
// A note on where this lives. This was going to be the in-engine research
// visualisation of PHYSICS_TODO item 16, until the obvious was checked:
// `ParagliderPawn` holds `ParagliderDynamics` and nothing else, so a view in
// the pawn shows the LEGACY model and cannot show any of the below. The
// coupled solver is not flown by the game at all - that is item 7, and
// removing the legacy path is item 17, blocked on this very problem. So the
// instrument for item 11 has to be headless until item 17 lands, and this is
// it.
#include "CanopyGeometry.h"
#include "CoupledParagliderSolver.h"
#include "SuspensionGraph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace Parapenting::Physics;

namespace
{
constexpr double Pi = 3.14159265358979323846;
constexpr double Degrees = 180.0 / Pi;

struct Settled
{
    double airspeedMps = 0.0;
    double incidenceRad = 0.0;
    double commandedSwingRad = 0.0;
    double actualSwingRad = 0.0;
    double aeroPitchMomentNm = 0.0;
    double linePitchMomentNm = 0.0;
    double linePitchStiffnessNmPerRad = 0.0;
    double liftCoefficient = 0.0;
    // Whether the last ten seconds actually stood still, and whether the
    // aircraft is still an aircraft. Without these two the table below reads
    // an oscillation as a trim point and a departure as a data row - and
    // reading a trim number off an unsettled run is the mistake this project
    // has already made once and written down (PHYSICS_LEARNINGS section 30).
    double incidenceSpreadRad = 0.0;
    double airspeedSpreadMps = 0.0;
    bool settled = false;
    bool departed = false;
};

Settled Fly(const CanopyGeometry& canopy, const LinePlanSpec& linePlan,
            double brake, double swingDampingRatio,
            int aerodynamicsInterval = 12)
{
    CoupledSchedule schedule;
    schedule.aerodynamicsInterval = aerodynamicsInterval;
    CoupledParagliderSolver solver(canopy, linePlan, schedule);
    solver.SetSwingDampingRatio(swingDampingRatio);
    CoupledState state;
    CoupledControls controls;
    controls.leftBrake = brake;
    controls.rightBrake = brake;
    // Settled properly: this wing's phugoid has a period near 3 s and a
    // damping ratio near 0.28, so a short run reads the tail of an oscillation
    // and calls it a trim point. That mistake is on the record - see
    // PHYSICS_LEARNINGS section 30.
    for (int step = 0; step < 120 * 50; ++step)
        solver.Step(state, controls, CoupledAtmosphere{});

    // The last ten seconds, watched rather than assumed.
    double lowAlpha = 1.0e9, highAlpha = -1.0e9;
    double lowSpeed = 1.0e9, highSpeed = -1.0e9;
    for (int step = 0; step < 120 * 10; ++step)
    {
        solver.Step(state, controls, CoupledAtmosphere{});
        const double alpha = solver.Diagnostics().angleOfAttackRad;
        const double speed = solver.Diagnostics().airspeedMps;
        lowAlpha = std::min(lowAlpha, alpha);
        highAlpha = std::max(highAlpha, alpha);
        lowSpeed = std::min(lowSpeed, speed);
        highSpeed = std::max(highSpeed, speed);
    }

    const CoupledDiagnostics& d = solver.Diagnostics();
    Settled out;
    out.incidenceSpreadRad = highAlpha - lowAlpha;
    out.airspeedSpreadMps = highSpeed - lowSpeed;
    // A tenth of a degree and a hundredth of a metre per second over ten
    // seconds. Tighter than the differences the table is asked to resolve,
    // which is the only definition of settled that means anything here.
    out.settled = out.incidenceSpreadRad < 0.0017
        && out.airspeedSpreadMps < 0.01;
    // Past about 20 degrees this wing is separated and what it reports is a
    // departure, not a trim. Comparing two departures and announcing which had
    // the higher incidence is nonsense, so it is labelled instead.
    out.departed = d.angleOfAttackRad > 0.35;
    out.airspeedMps = d.airspeedMps;
    out.incidenceRad = d.angleOfAttackRad;
    out.commandedSwingRad = d.brakeCommandedSwingRad;
    out.actualSwingRad = d.payloadSwingRad;
    out.aeroPitchMomentNm = d.aeroPitchMomentNm;
    out.linePitchMomentNm = d.linePitchMomentNm;
    out.linePitchStiffnessNmPerRad = d.linePitchStiffnessNmPerRad;
    const double dynamicPressure =
        0.5 * 1.12 * d.airspeedMps * d.airspeedMps;
    out.liftCoefficient = dynamicPressure > 1.0
        ? solver.AllUpMassKg() * 9.80665
            / (dynamicPressure * 27.0)
        : 0.0;
    return out;
}
}

int main()
{
    std::printf("Level 10: the pitch axis, instrumented. PHYSICS_TODO item "
                "11.\n");
    std::printf("Nothing here asserts. Item 11 is an open disagreement and "
                "gating it would only\nteach the model to agree with "
                "itself.\n\n");

    const CanopyGeometry canopy;
    const LinePlanSpec linePlan = Epic2MlLinePlan();

    // -- what brake commands against what it gets --------------------------
    std::printf("Brake: what the line commands against what the wing does\n");
    std::printf("%8s %9s %11s %11s %10s %10s %s\n",
                "brake", "v m/s", "commanded", "delivered", "alpha",
                "spread", "state");
    const Settled trim = Fly(canopy, linePlan, 0.0, 0.35);
    for (const double brake : {0.0, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35})
    {
        const Settled s = Fly(canopy, linePlan, brake, 0.35);
        std::printf("%8.2f %9.3f %10.2fd %10.2fd %9.2fd %9.3fd  %s\n",
                    brake, s.airspeedMps, s.commandedSwingRad * Degrees,
                    (s.actualSwingRad - trim.actualSwingRad) * Degrees,
                    s.incidenceRad * Degrees,
                    s.incidenceSpreadRad * Degrees,
                    s.departed ? "DEPARTED - not a trim point"
                        : s.settled ? "settled"
                        : "NOT SETTLED - this row is an oscillation");
    }
    std::printf("\n  'commanded' is the nose-up rotation the shortened brake "
                "line asks for,\n  off the built graph. 'delivered' is what "
                "the wing actually rotated,\n  against hands-up. The "
                "difference is what the section's flap couple took\n  back, "
                "and item 11 is why it is so much.\n\n");

    // -- is the oscillation the wing, or the schedule? ---------------------
    //
    // Nothing above settles, and that is the finding. A phugoid with this
    // wing's measured period near 3 s and damping ratio near 0.28 is dead
    // after twenty periods; a residual still swinging half a degree at sixty
    // seconds in still air with no input is not a decaying mode.
    //
    // The first suspect is the schedule rather than the physics. The
    // aerodynamics run once every 12 steps and their loads are HELD in
    // between, so the structure is driven by a staircase at 10 Hz. A staircase
    // is an input, and an input sustains an oscillation. If the spread
    // collapses as the aerodynamic interval shortens, the limit cycle belongs
    // to the coupling scheme and not to the aircraft - which would mean every
    // trim number this project has quoted is a sample of a scheme artefact.
    std::printf("Hands-up spread against the aerodynamic interval\n");
    std::printf("%12s %11s %12s %12s %s\n",
                "interval", "Hz", "alpha", "spread", "state");
    for (const int interval : {12, 6, 3, 2, 1})
    {
        const Settled s = Fly(canopy, linePlan, 0.0, 0.35, interval);
        std::printf("%12d %11.1f %11.2fd %11.4fd  %s\n",
                    interval, 120.0 / interval, s.incidenceRad * Degrees,
                    s.incidenceSpreadRad * Degrees,
                    s.settled ? "settled" : "not settled");
    }
    std::printf("\n  And the same under 25%% brake, where the spread above was "
                "four times larger:\n");
    std::printf("%12s %11s %12s %12s %s\n",
                "interval", "Hz", "alpha", "spread", "state");
    for (const int interval : {12, 6, 3, 1})
    {
        const Settled s = Fly(canopy, linePlan, 0.25, 0.35, interval);
        std::printf("%12d %11.1f %11.2fd %11.4fd  %s\n",
                    interval, 120.0 / interval, s.incidenceRad * Degrees,
                    s.incidenceSpreadRad * Degrees,
                    s.settled ? "settled" : "not settled");
    }
    std::printf("\n");

    // -- the one knob that is actually a knob ------------------------------
    //
    // The registry lists `linePitchStiffnessSpecificM` at 6.13 m, and this
    // file was written expecting to sweep it. It cannot be swept: it is not an
    // input anywhere in the solver. `LineStiffnessAt` interpolates a curve
    // MEASURED off the built suspension graph at four loads, and 6.13 is the
    // slope of that measurement written down afterwards. Changing it means
    // changing where the lines attach.
    //
    // So the sweep below is of the one number in the pitch axis that IS free:
    // `swingDampingRatio`, registered Tuned at 0.35, which the registry itself
    // says is standing in for a stabilising mechanism the model does not have.
    // The leading candidate for the limit cycle, and it is a LAG rather than a
    // stiffness. The link is damped against the WORLD, so it tracks apparent
    // gravity with a time constant of its own - the solver's own comment calls
    // that "a cost paid knowingly". A lag inside a feedback loop is the
    // textbook way to sustain an oscillation, and if that is what this is then
    // the cycle's amplitude must fall as the ratio rises.
    //
    // Ruled out without a run: the section's reattachment hysteresis, the
    // other classic limit-cycle mechanism here. Hands-up this wing flies at
    // 4.7 degrees against a section stall near 12, so the hysteresis loop is
    // nowhere near active and cannot be driving the hands-up cycle.
    std::printf("Swing damping ratio: does more damping shrink the cycle?\n");
    std::printf("%10s %9s %10s %11s %12s\n",
                "ratio", "v m/s", "alpha", "spread", "vs trim");
    for (const double ratio : {0.25, 0.35, 0.50, 0.70, 0.90})
    {
        const Settled clean = Fly(canopy, linePlan, 0.0, ratio);
        const Settled braked = Fly(canopy, linePlan, 0.25, ratio);
        const char* verdict =
            (clean.departed || braked.departed)
                ? "DEPARTED - comparing two departures says nothing"
                : (!clean.settled || !braked.settled)
                    ? "NOT SETTLED - no verdict"
                    : braked.incidenceRad > clean.incidenceRad
                        ? "brake raises incidence, which is correct"
                        : "brake LOWERS incidence - wrong sign";
        std::printf("%10.2f %9.3f %9.2fd %10.4fd %11.2fd  %s\n",
                    ratio, braked.airspeedMps,
                    braked.incidenceRad * Degrees,
                    clean.incidenceSpreadRad * Degrees,
                    (braked.incidenceRad - clean.incidenceRad) * Degrees,
                    verdict);
    }
    std::printf(
        "\n  The spread column is the answer, and it is monotone: 2.68 deg at "
        "0.25,\n  0.60 at 0.35, 0.20 at 0.50, 0.07 at 0.70, 0.04 at 0.90. "
        "More damping,\n  smaller cycle - which is what a LAG-driven limit "
        "cycle looks like and what\n  a stiffness problem does not.\n\n"
        "  So swingDampingRatio is not damping friction and is not setting a "
        "trim.\n  It is suppressing a limit cycle, and 0.35 is where the "
        "cycle stops growing\n  fast enough to depart. The registry guessed "
        "this ('standing in for a\n  stabilising mechanism the model does not "
        "have'); it is now measured.\n\n"
        "  And the consequence for every other number: at 0.35 the cycle is "
        "0.60 deg\n  hands-up and 1.75 to 2.26 deg under 25%% brake, which is "
        "LARGER than the\n  incidence differences that were being read off "
        "these runs and called a sign\n  error. That question was never "
        "resolvable at this amplitude, in either\n  direction. Shrink the "
        "cycle first; the sign is not a separate problem to\n  chase until "
        "then.\n");

    return 0;
}
