// Level 10, second strand: what a cheaper solver costs in answers.
//
// The profile said where the time goes. This asks the only question that
// matters next: how much of each expensive stage can be taken away before the
// aircraft stops being the same aircraft.
//
// Every row here is one knob moved away from the reference schedule, flown
// through the same two signatures, and reported as BOTH a cost and a
// disagreement. A cheaper tier chosen on cost alone is a guess; a tolerance
// stated without a sweep behind it is a number somebody made up.
//
// The two signatures:
//
//   * QUIET - settled hands-up trim. Airspeed, sink and incidence. This is
//     where the game spends its time and where a small bias would go unnoticed
//     and then turn up in Level 9's calibration as a fake disagreement.
//   * VIOLENT - a 4 m/s downdraught over the left half, which is the gust that
//     folds this wing in `coupled_tests`. Peak collapse and whether it clears.
//     A tier that flies cruise perfectly and gets a collapse wrong is worse
//     than no tier, because the collapse is what a pilot is judged on.
//
// Nothing here asserts. It is a measurement that produces the tolerances the
// gates in `coupled_tests` are then written against.
#include "CanopyGeometry.h"
#include "CoupledParagliderSolver.h"
#include "SuspensionGraph.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace Parapenting::Physics;

namespace
{
constexpr double Pi = 3.14159265358979323846;
constexpr double Degrees = 180.0 / Pi;

struct Signature
{
    // Quiet.
    double trimAirspeedMps = 0.0;
    double trimSinkMps = 0.0;
    double trimIncidenceRad = 0.0;
    // Violent.
    double peakCollapse = 0.0;
    double residualCollapse = 0.0;
    // The line network's own convergence, newtons of unbalanced force left on
    // the canopy. This is the sharp instrument: a flight signature can sit on
    // top of a network that is not converged, because the residual is a force
    // the rigid body then integrates away over many steps. Reported as the
    // WORST over the measured leg, not the mean.
    double worstSuspensionResidualN = 0.0;
    // Cost, over the quiet leg only, so the two are comparable across rows.
    double microsecondsPerStep = 0.0;
};

Signature Measure(const CanopyGeometry& canopy, const LinePlanSpec& linePlan,
                  const CoupledSchedule& schedule)
{
    Signature out;

    // -- quiet -------------------------------------------------------------
    {
        CoupledParagliderSolver solver(canopy, linePlan, schedule);
        CoupledState state;
        for (int step = 0; step < 120 * 40; ++step)
            solver.Step(state, CoupledControls{}, CoupledAtmosphere{});

        solver.ResetProfile();
        solver.SetProfiling(true);
        const double startZ = state.positionWorldM.z;
        constexpr int MeasureSteps = 120 * 20;
        for (int step = 0; step < MeasureSteps; ++step)
        {
            solver.Step(state, CoupledControls{}, CoupledAtmosphere{});
            out.worstSuspensionResidualN = std::max(
                out.worstSuspensionResidualN,
                std::fabs(solver.Diagnostics().suspensionResidualN));
        }
        solver.SetProfiling(false);

        out.trimAirspeedMps = solver.Diagnostics().airspeedMps;
        out.trimSinkMps = (startZ - state.positionWorldM.z) / (MeasureSteps / 120.0);
        out.trimIncidenceRad = solver.Diagnostics().angleOfAttackRad;

        const CoupledStepProfile& p = solver.Profile();
        out.microsecondsPerStep = p.steps > 0
            ? static_cast<double>(p.totalNs) * 1.0e-3
                / static_cast<double>(p.steps)
            : 0.0;
    }

    // -- violent -----------------------------------------------------------
    {
        CoupledParagliderSolver solver(canopy, linePlan, schedule);
        CoupledState state;
        for (int step = 0; step < 120 * 20; ++step)
            solver.Step(state, CoupledControls{}, CoupledAtmosphere{});

        CoupledAtmosphere gust;
        gust.gustWorldMps = Vec3{0.0, 0.0, -4.0};
        gust.gustSpanFrom = -1.0;
        gust.gustSpanTo = 0.0;
        for (int step = 0; step < 120 * 2; ++step)
        {
            solver.Step(state, CoupledControls{}, gust);
            out.peakCollapse = std::max(
                out.peakCollapse,
                solver.Diagnostics().collapseState.leftCollapse);
        }
        // Then let it go and see whether the wing comes back.
        for (int step = 0; step < 120 * 30; ++step)
        {
            solver.Step(state, CoupledControls{}, CoupledAtmosphere{});
            out.peakCollapse = std::max(
                out.peakCollapse,
                solver.Diagnostics().collapseState.leftCollapse);
        }
        out.residualCollapse = solver.Diagnostics().collapseState.leftCollapse;
    }

    return out;
}

void PrintHeader()
{
    std::printf("%-34s %8s %8s %7s %8s %10s %9s\n",
                "schedule", "v m/s", "sink", "alpha", "fold", "residual N",
                "us/step");
}

void PrintRow(const std::string& label, const Signature& s,
              const Signature& reference, bool isReference)
{
    std::printf("%-34s %8.3f %8.3f %6.2f%s %8.3f %10.2e %9.1f",
                label.c_str(), s.trimAirspeedMps, s.trimSinkMps,
                s.trimIncidenceRad * Degrees, "d", s.peakCollapse,
                s.worstSuspensionResidualN, s.microsecondsPerStep);
    if (isReference)
    {
        std::printf("   reference\n");
        return;
    }
    const double speedError = std::fabs(
        s.trimAirspeedMps - reference.trimAirspeedMps);
    const double sinkError = std::fabs(s.trimSinkMps - reference.trimSinkMps);
    const double alphaErrorDeg = std::fabs(
        s.trimIncidenceRad - reference.trimIncidenceRad) * Degrees;
    const double foldError = std::fabs(
        s.peakCollapse - reference.peakCollapse);
    const double saving = reference.microsecondsPerStep > 0.0
        ? 100.0 * (1.0 - s.microsecondsPerStep
                          / reference.microsecondsPerStep)
        : 0.0;
    (void)sinkError;
    std::printf("   dv %.3f  dalpha %.2fd  dfold %.3f  saves %.0f%%\n",
                speedError, alphaErrorDeg, foldError, saving);
}
}

int main()
{
    std::printf("Level 10: what a cheaper solver costs in answers.\n");
    std::printf("Each row moves ONE knob off the reference schedule. "
                "Nothing here asserts.\n\n");

    const CanopyGeometry canopy;
    const LinePlanSpec linePlan = Epic2MlLinePlan();

    const CoupledSchedule reference;
    const Signature base = Measure(canopy, linePlan, reference);

    PrintHeader();
    PrintRow("reference (120 / 3 / 600)", base, base, true);
    std::printf("\n");

    // -- the 60.7% stage ---------------------------------------------------
    std::printf("suspension iterations, 120 is the reference:\n");
    for (const int iterations : {80, 60, 40, 30, 20, 10})
    {
        CoupledSchedule schedule = reference;
        schedule.suspensionIterations = iterations;
        char label[64];
        std::snprintf(label, sizeof(label), "  suspensionIterations %d",
                      iterations);
        PrintRow(label, Measure(canopy, linePlan, schedule), base, false);
    }
    std::printf("\n");

    std::printf("coupling iterations, 3 is the reference:\n");
    for (const int iterations : {2, 1})
    {
        CoupledSchedule schedule = reference;
        schedule.couplingIterations = iterations;
        char label[64];
        std::snprintf(label, sizeof(label), "  couplingIterations %d",
                      iterations);
        PrintRow(label, Measure(canopy, linePlan, schedule), base, false);
    }
    std::printf("\n");

    // -- the 23.8% stage ---------------------------------------------------
    // Read PHYSICS_LEARNINGS section 28 before believing a cheap row here.
    // These probes were already wrong once from being under-converged, and the
    // symptom was not a bad trim speed - it was a damping derivative that
    // moved 10% between intervals and differed between mirror-image flights.
    // Neither signature below would catch that on its own, which is why the
    // mirror check is run separately at the end.
    std::printf("frozen VSM iterations, 600 is the reference:\n");
    for (const int iterations : {300, 150, 80, 40})
    {
        CoupledSchedule schedule = reference;
        schedule.frozenSolveIterations = iterations;
        char label[64];
        std::snprintf(label, sizeof(label), "  frozenSolveIterations %d",
                      iterations);
        PrintRow(label, Measure(canopy, linePlan, schedule), base, false);
    }
    std::printf("\n");

    // The lever the iteration cap turned out not to be. The probes converge
    // and exit long before 600, so what costs is running two extra frozen
    // solves at all - which makes their FREQUENCY the knob. Each axis already
    // refreshes only every third tick; this asks how much slower it can go.
    std::printf("damping probe interval in aerodynamic ticks, 1 is the "
                "reference:\n");
    for (const int interval : {2, 3, 6, 12})
    {
        CoupledSchedule schedule = reference;
        schedule.dampingProbeInterval = interval;
        char label[64];
        std::snprintf(label, sizeof(label), "  dampingProbeInterval %d",
                      interval);
        PrintRow(label, Measure(canopy, linePlan, schedule), base, false);
    }
    std::printf("\n");

    // -- the two knobs together --------------------------------------------
    // Measured, not assumed additive. Savings on two stages of one loop are
    // not independent in general, and a tier is what ships, so the tier is
    // what gets flown.
    std::printf("the reduced tier, both knobs at once:\n");
    PrintRow("  ReducedFidelitySchedule",
             Measure(canopy, linePlan, ReducedFidelitySchedule()), base,
             false);
    std::printf("\n");

    // -- what the two signatures above cannot see --------------------------
    //
    // The damping derivative itself, and whether it is still the same on both
    // wings. A left roll and its mirror image must return derivatives that
    // agree; when the probes were under-converged they did not, and no trim
    // number showed it.
    std::printf("damping derivative against frozen iteration count "
                "(the failure mode section 28 records):\n");
    std::printf("%-34s %14s %14s %12s\n", "schedule", "roll damping",
                "mirrored", "disagreement");
    for (const int iterations : {600, 300, 150, 80, 40})
    {
        CoupledSchedule schedule;
        schedule.frozenSolveIterations = iterations;

        const auto rollDamping = [&](double sign)
        {
            CoupledParagliderSolver solver(canopy, linePlan, schedule);
            CoupledState state;
            CoupledControls rolling;
            rolling.rightBrake = sign > 0.0 ? 0.35 : 0.0;
            rolling.leftBrake = sign > 0.0 ? 0.0 : 0.35;
            for (int step = 0; step < 120 * 30; ++step)
                solver.Step(state, rolling, CoupledAtmosphere{});
            return state.rotationalDampingNmPerRadps.x;
        };
        const double right = rollDamping(+1.0);
        const double left = rollDamping(-1.0);
        // A DIFFERENCE, not a sum. Roll damping is a diagonal derivative and
        // it is negative whichever way the wing is rolling, so mirroring the
        // input must return the same number, not its negative. Written as a
        // sum first, which reported a flat 200% disagreement on two identical
        // values and would have condemned every row here.
        const double scale = std::max(
            1.0, 0.5 * (std::fabs(right) + std::fabs(left)));
        char label[64];
        std::snprintf(label, sizeof(label), "  frozenSolveIterations %d",
                      iterations);
        std::printf("%-34s %14.1f %14.1f %11.3f%%\n", label, right, left,
                    100.0 * std::fabs(right - left) / scale);
    }

    std::printf("\nRead this with docs/SOLVER_PROFILE.md.\n");
    return 0;
}
