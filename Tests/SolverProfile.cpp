// Level 10, first strand: where the coupled solver's time actually goes.
//
// Solver levels of detail come next, and a cheaper tier is a guess until the
// expensive stage is known. Nothing here asserts - a profile is a measurement
// of this machine on this day, and gating on wall clock would turn a busy
// laptop into a red suite. It prints, and the numbers are recorded in
// docs/SOLVER_PROFILE.md with the machine they came off.
//
// What is measured is a step of the flight loop, broken down by stage, over
// flight states that load the stages differently:
//
//   * hands-up cruise - the case the game spends its time in;
//   * 25% brake      - an aerodynamic solve away from the warm start;
//   * asymmetric     - two half wings doing different things;
//   * collapsed      - a fold, which is the collapse solver's own case and
//                      the one where the VSM is furthest from converged.
//
// Construction is timed separately. It builds a section polar table over 21
// brake stations, each a panel factorisation and an incidence sweep, and that
// is a load-time cost rather than a per-step one - but it is the number that
// decides whether a wing can be swapped in flight.
#include "CanopyGeometry.h"
#include "CoupledParagliderSolver.h"
#include "SuspensionGraph.h"

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

using namespace Parapenting::Physics;

namespace
{
struct Case
{
    std::string name;
    CoupledControls controls;
    CoupledAtmosphere atmosphere;
};

double Milliseconds(long long ns) { return static_cast<double>(ns) * 1.0e-6; }

// Microseconds per step, which is the unit that matters: the schedule runs at
// 120 Hz, so the real-time budget for one step is 8333 us and a step costing
// more than that cannot be flown.
double MicrosecondsPerStep(long long ns, long long steps)
{
    return steps > 0
        ? static_cast<double>(ns) * 1.0e-3 / static_cast<double>(steps) : 0.0;
}

void PrintRow(const char* label, long long ns, long long steps,
              long long totalNs)
{
    const double share = totalNs > 0
        ? 100.0 * static_cast<double>(ns) / static_cast<double>(totalNs) : 0.0;
    std::printf("    %-22s %9.2f us/step  %5.1f%%\n",
                label, MicrosecondsPerStep(ns, steps), share);
}
}

int main()
{
    std::printf("Level 10: coupled solver profile.\n");
    std::printf("Nothing here is a gate. Wall clock is a property of the "
                "machine, not the model.\n\n");

    const CanopyGeometry canopy;
    const LinePlanSpec linePlan = Epic2MlLinePlan();

    // -- construction ------------------------------------------------------
    {
        const auto start = std::chrono::steady_clock::now();
        CoupledParagliderSolver solver(canopy, linePlan);
        const auto built = std::chrono::steady_clock::now();
        const double buildMs =
            std::chrono::duration<double, std::milli>(built - start).count();
        std::printf("Construction: %.1f ms\n", buildMs);
        std::printf("  section polar table, suspension graph, trim load "
                    "distribution, line stiffness curve and brake swing "
                    "curve, all solved rather than loaded\n");
        std::printf("  this is a load-time cost, and it is what decides "
                    "whether a wing can be swapped in flight\n\n");
    }

    // -- per-stage, per flight state ---------------------------------------
    std::vector<Case> cases;
    {
        Case cruise{"hands-up cruise", CoupledControls{}, CoupledAtmosphere{}};
        cases.push_back(cruise);

        Case braked{"25% brake", CoupledControls{}, CoupledAtmosphere{}};
        braked.controls.leftBrake = 0.25;
        braked.controls.rightBrake = 0.25;
        cases.push_back(braked);

        Case asymmetric{"asymmetric brake", CoupledControls{},
                        CoupledAtmosphere{}};
        asymmetric.controls.rightBrake = 0.35;
        asymmetric.controls.weightShift = 0.6;
        cases.push_back(asymmetric);

        // A 4 m/s downdraught over the left half is the gust that folds this
        // wing in `coupled_tests`. The collapse solver and the VSM are both
        // furthest from a quiet steady state here.
        Case gust{"4 m/s gust, left half", CoupledControls{},
                  CoupledAtmosphere{}};
        gust.atmosphere.windWorldMps = Vec3{0.0, 0.0, -4.0};
        cases.push_back(gust);
    }

    constexpr int SettleSteps = 120 * 20;
    constexpr int MeasureSteps = 120 * 30;

    for (const Case& scenario : cases)
    {
        CoupledParagliderSolver solver(canopy, linePlan);
        CoupledState state;

        // Settle first, and settle UNPROFILED. The first steps of a fresh
        // solver run the cold 12000-iteration suspension relaxation and a
        // 600-iteration cold VSM, which are real costs but they are not the
        // steady flight loop and averaging them in would flatter nothing and
        // mislead everything.
        for (int step = 0; step < SettleSteps; ++step)
            solver.Step(state, scenario.controls, CoupledAtmosphere{});

        solver.ResetProfile();
        solver.SetProfiling(true);
        for (int step = 0; step < MeasureSteps; ++step)
            solver.Step(state, scenario.controls, scenario.atmosphere);
        solver.SetProfiling(false);

        const CoupledStepProfile& p = solver.Profile();
        const long long total = p.totalNs;

        std::printf("%s - %lld steps, %lld aerodynamic ticks "
                    "(1 in %.1f)\n",
                    scenario.name.c_str(), p.steps, p.aeroTicks,
                    p.aeroTicks > 0
                        ? static_cast<double>(p.steps)
                            / static_cast<double>(p.aeroTicks)
                        : 0.0);

        PrintRow("VSM unsteady", p.vsmUnsteadyNs, p.steps, total);
        PrintRow("VSM stationary", p.vsmStationaryNs, p.steps, total);
        PrintRow("VSM damping probes", p.vsmDampingProbeNs, p.steps, total);
        PrintRow("cell pressure", p.pressureNs, p.steps, total);
        PrintRow("membrane", p.membraneNs, p.steps, total);
        PrintRow("collapse", p.collapseNs, p.steps, total);
        PrintRow("suspension", p.suspensionNs, p.steps, total);
        PrintRow("rigid motion", p.rigidMotionNs, p.steps, total);
        PrintRow("unaccounted", total - p.AccountedNs(), p.steps, total);
        std::printf("    %-22s %9.2f us/step\n", "TOTAL",
                    MicrosecondsPerStep(total, p.steps));

        // The two numbers a level-of-detail decision is made against.
        const double usPerStep = MicrosecondsPerStep(total, p.steps);
        constexpr double BudgetUsPerStep = 1.0e6 / 120.0;
        std::printf("    %.1fx real time at 120 Hz "
                    "(budget %.0f us/step, using %.1f%%)\n",
                    usPerStep > 0.0 ? BudgetUsPerStep / usPerStep : 0.0,
                    BudgetUsPerStep, 100.0 * usPerStep / BudgetUsPerStep);
        std::printf("    aerodynamic tick costs %.2f ms of which the VSM is "
                    "%.2f ms\n",
                    p.aeroTicks > 0
                        ? Milliseconds(p.VsmTotalNs() + p.pressureNs
                                       + p.membraneNs) / p.aeroTicks
                        : 0.0,
                    p.aeroTicks > 0
                        ? Milliseconds(p.VsmTotalNs()) / p.aeroTicks : 0.0);
        std::printf("\n");
    }

    std::printf("Read this with docs/SOLVER_PROFILE.md, which records what "
                "was decided from it.\n");
    return 0;
}
