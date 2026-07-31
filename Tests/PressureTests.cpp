// Level 5: cell openings and internal pressure.
//
// These are the master plan's Level 5 exit gates, written as checks. The one
// they all rest on is the last: the stagnation point moves with incidence, and
// what the inlets recover follows it. Everything else - inflation, local
// pressure loss, an unloaded inlet emptying its cell, recovery that follows
// airflow rather than a timer - is a consequence of that and of the cell being
// a plenum, not a separate rule.
#include "CanopyGeometry.h"
#include "CanopyPressureSolver.h"
#include "SectionPolarTable.h"
#include "VortexStepMethodSolver.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace Parapenting::Physics;

namespace
{
int Failures = 0;
constexpr double Pi = 3.14159265358979323846;

void Check(bool condition, const std::string& what)
{
    if (!condition)
    {
        std::printf("  FAIL  %s\n", what.c_str());
        ++Failures;
    }
}

constexpr int Cells = 45;
constexpr double TrimDynamicPressurePa = 68.0;
constexpr double TrimAlphaRad = 0.10;

CellPressureInput UniformFlow(
    double dynamicPressurePa, double alphaRad)
{
    CellPressureInput input;
    input.dynamicPressurePa.assign(Cells, dynamicPressurePa);
    input.angleOfAttackRad.assign(Cells, alphaRad);
    return input;
}

// Runs to a steady state and returns the last result.
CellPressureResult Settle(
    const CanopyPressureSolver& solver, CellPressureState& state,
    const CellPressureInput& input, double seconds)
{
    CellPressureResult result;
    const int steps = static_cast<int>(seconds * 120.0);
    for (int step = 0; step < steps; ++step)
        result = solver.Step(state, input, 1.0 / 120.0);
    return result;
}
}

int main()
{
    const CanopyPressureSolver solver(Cells);

    // -- the stagnation point ---------------------------------------------
    {
        // The gate everything else follows from.
        const double zeroLift = -0.07;
        const double atTrim = solver.StagnationAngleRad(TrimAlphaRad, zeroLift);
        const double onBar = solver.StagnationAngleRad(0.02, zeroLift);
        const double braked = solver.StagnationAngleRad(0.18, zeroLift);
        std::printf("Stagnation point below the chord line: "
                    "%.1f deg on bar, %.1f at trim, %.1f braked\n",
                    onBar * 180.0 / Pi, atTrim * 180.0 / Pi,
                    braked * 180.0 / Pi);
        Check(onBar < atTrim && atTrim < braked,
              "the stagnation point migrates with incidence");
        Check(onBar > 0.0,
              "and stays below the chord line through the normal range");

        // The inlet is cut where trim puts the stagnation point, so trim is
        // where it recovers most.
        const CellPressureSpec spec = solver.Spec();
        const double atInlet = CanopyPressureSolver::NosePressureCoefficient(
            spec.inletAngularPositionRad - atTrim);
        const double onBarAtInlet =
            CanopyPressureSolver::NosePressureCoefficient(
                spec.inletAngularPositionRad - onBar);
        std::printf("  inlet recovers Cp %.2f at trim, %.2f on bar\n",
                    atInlet, onBarAtInlet);
        Check(atInlet > 0.9,
              "at trim the inlet sits within a few degrees of stagnation and "
              "recovers nearly all of it");
        Check(onBarAtInlet < atInlet,
              "on bar the stagnation point has moved away from the inlet and "
              "it recovers less - which is why accelerated flight is the "
              "collapse-prone one, with no rule that says so");
        Check(CanopyPressureSolver::NosePressureCoefficient(0.0) == 1.0,
              "the stagnation point itself recovers all of the dynamic "
              "pressure");
        Check(CanopyPressureSolver::NosePressureCoefficient(Pi / 6.0)
                  < 1.0e-9,
              "and thirty degrees away it recovers none - the cylinder "
              "distribution");
    }

    // -- inflation --------------------------------------------------------
    CellPressureState state;
    {
        const CellPressureInput trim =
            UniformFlow(TrimDynamicPressurePa, TrimAlphaRad);
        const CellPressureResult first = solver.Step(state, trim, 1.0 / 120.0);
        Check(first.filledFraction[0] > 0.0 && first.filledFraction[0] < 1.0,
              "a packed wing starts empty and begins filling immediately");
        Check(first.gaugePressurePa[0] == 0.0,
              "and holds no pressure while it is still filling - the air is "
              "going into volume, not into compression");

        const CellPressureResult early = Settle(solver, state, trim, 0.25);
        const CellPressureResult settled = Settle(solver, state, trim, 5.0);
        std::printf("Inflation: %.1f Pa after 0.25 s, %.1f Pa settled "
                    "(Cp %.2f)\n",
                    early.gaugePressurePa[0], settled.gaugePressurePa[0],
                    settled.meanPressureCoefficient);
        Check(early.gaugePressurePa[0] < settled.gaugePressurePa[0],
              "pressure rises during inflation");
        Check(settled.meanPressureCoefficient > 0.85,
              "and stabilises near the stagnation pressure the inlets are "
              "recovering");

        // Settled means settled: another five seconds must not move it.
        const double before = settled.gaugePressurePa[0];
        const CellPressureResult later = Settle(solver, state, trim, 5.0);
        Check(std::fabs(later.gaugePressurePa[0] - before) < 0.05,
              "and then holds steady in trim");
        Check(later.cellsBelowCollapseRisk == 0,
              "a wing in trim has no cell at collapse risk");
    }

    // -- local adverse incidence ------------------------------------------
    {
        // Drive the outer third of the left wing to strongly negative
        // incidence, as a gust or a tip stall would. Only those cells may
        // lose pressure.
        CellPressureInput gust =
            UniformFlow(TrimDynamicPressurePa, TrimAlphaRad);
        // Far enough negative that the inlet passes the thirty degrees at
        // which the nose distribution crosses zero, and sits in suction.
        for (int cell = 0; cell < Cells / 3; ++cell)
            gust.angleOfAttackRad[static_cast<std::size_t>(cell)] = -0.50;

        const CellPressureResult hit = Settle(solver, state, gust, 3.0);
        const double affected = hit.pressureCoefficient[2];
        const double untouched =
            hit.pressureCoefficient[static_cast<std::size_t>(Cells - 3)];
        std::printf("Local adverse incidence: Cp %.2f on the affected tip, "
                    "%.2f on the far one\n", affected, untouched);
        Check(affected < untouched,
              "local adverse incidence causes local pressure loss");
        Check(hit.cellsWithReversedInflow > 0,
              "the inlet has moved into suction, so air leaves through it - "
              "the front collapse, arrived at from the geometry");
        Check(hit.cellsBelowCollapseRisk > 0,
              "and those cells report themselves at collapse risk");
        Check(untouched > 0.85,
              "while the far wing is unaffected");
    }

    // -- recovery follows airflow, not a timer ----------------------------
    {
        // Restore the flow and watch it come back. The rate must depend on
        // what the air is doing: at half the dynamic pressure it must take
        // longer, because the inlet has less to push with.
        const auto recoveryTime = [&](double dynamicPressure)
        {
            CellPressureState collapsed;
            collapsed.gaugePressurePa.assign(Cells, 0.0);
            collapsed.initialised = true;
            const CellPressureInput flow =
                UniformFlow(dynamicPressure, TrimAlphaRad);
            for (int step = 0; step < 120 * 20; ++step)
            {
                const CellPressureResult now =
                    solver.Step(collapsed, flow, 1.0 / 120.0);
                if (now.meanPressureCoefficient > 0.8)
                    return static_cast<double>(step) / 120.0;
            }
            return -1.0;
        };
        const double fast = recoveryTime(TrimDynamicPressurePa);
        const double slow = recoveryTime(0.25 * TrimDynamicPressurePa);
        std::printf("Recovery to Cp 0.8: %.2f s at trim pressure, %.2f s at a "
                    "quarter of it\n", fast, slow);
        Check(fast > 0.0 && slow > 0.0, "the wing refills in both cases");
        Check(slow > fast * 1.5,
              "recovery follows the airflow rather than a scripted timer - "
              "less dynamic pressure, slower refill");
    }

    // -- an unloaded inlet ------------------------------------------------
    {
        // Close the inlets on one group by taking their dynamic pressure
        // away, and only that group may empty.
        CellPressureState working;
        const CellPressureInput trim =
            UniformFlow(TrimDynamicPressurePa, TrimAlphaRad);
        Settle(solver, working, trim, 6.0);

        CellPressureInput unloaded = trim;
        for (int cell = 0; cell < 6; ++cell)
            unloaded.dynamicPressurePa[static_cast<std::size_t>(cell)] = 0.0;
        const CellPressureResult after = Settle(solver, working, unloaded, 4.0);
        std::printf("Unloaded inlet group: %.1f Pa at the tip, %.1f Pa just "
                    "inboard of it, %.1f Pa on the far wing\n",
                    after.gaugePressurePa[0], after.gaugePressurePa[7],
                    after.gaugePressurePa[static_cast<std::size_t>(Cells - 1)]);
        Check(after.gaugePressurePa[0]
                  < after.gaugePressurePa[static_cast<std::size_t>(Cells - 1)],
              "unloading an inlet reduces the pressure of its own cell group");
        // The inlet is the biggest hole in the cell, so an unloaded one vents
        // it faster than the crossports can feed it. That is the right way
        // round: it is why a cell group whose inlets stop working deflates
        // rather than being held up by its neighbours.
        Check(after.gaugePressurePa[0] < 1.0,
              "and it empties through that same inlet, which is a bigger hole "
              "than the crossports are");
        // The crossports only do anything where there is a pressure
        // difference to move air along, so the place to see them is an
        // inflation in which one group's inlets are not feeding. With
        // crossports that group fills from its neighbours; without them it
        // cannot fill at all.
        const auto fillWithDeadInlets = [&](double crossportArea)
        {
            CellPressureSpec variant = solver.Spec();
            variant.crossportAreaM2 = crossportArea;
            const CanopyPressureSolver variantSolver(Cells, variant);
            CellPressureState fresh;
            CellPressureInput partial =
                UniformFlow(TrimDynamicPressurePa, TrimAlphaRad);
            // One cell, so it has a fed neighbour. A crossport passes far
            // less air than an inlet, so a chain of dead cells fills inward
            // one at a time over tens of seconds - which is itself right, and
            // is why blocked inlets across a whole group are serious.
            partial.dynamicPressurePa[0] = 0.0;
            return Settle(variantSolver, fresh, partial, 8.0)
                .filledFraction[0];
        };
        const double ported = fillWithDeadInlets(solver.Spec().crossportAreaM2);
        const double sealed = fillWithDeadInlets(0.0);
        std::printf("  a cell whose inlet never feeds reaches %.0f%% full "
                    "with crossports, %.0f%% without\n",
                    100.0 * ported, 100.0 * sealed);
        Check(sealed < 0.01,
              "with the ribs sealed, a cell whose inlet does not feed cannot "
              "fill at all");
        Check(ported > sealed,
              "the rib crossports fill it from the cells beside it - which is "
              "how a cell group with blocked inlets is kept flying");
    }

    // -- driven by the aerodynamic solve ----------------------------------
    {
        // The point of taking per-cell incidence rather than one number: the
        // pressure model can be driven by the VSM's own spanwise solution, so
        // a wing whose tip is at a different angle of attack pressurises
        // differently there without anyone saying it should.
        const CanopyGeometry canopy;
        const VortexStepMethodSolver wing(
            canopy, SectionPolarTable::Analytic(), Cells);
        VsmSolveInput flight;
        flight.airspeedBodyMps = {11.0 * std::cos(0.06), 0.0,
                                  -11.0 * std::sin(0.06)};
        flight.rightBrake = 0.5;
        const VsmSolution aero = wing.Solve(flight);

        CellPressureInput fromAero;
        fromAero.dynamicPressurePa.resize(Cells);
        fromAero.angleOfAttackRad.resize(Cells);
        for (std::size_t cell = 0; cell < static_cast<std::size_t>(Cells);
             ++cell)
        {
            fromAero.dynamicPressurePa[cell] = TrimDynamicPressurePa;
            fromAero.angleOfAttackRad[cell] =
                aero.sections[cell].angleOfAttackRad;
        }

        CellPressureState coupled;
        const CellPressureResult solved =
            Settle(solver, coupled, fromAero, 6.0);
        std::printf("Driven by the VSM with right brake 0.5: Cp %.2f left "
                    "tip, %.2f right tip, %d cells at risk\n",
                    solved.pressureCoefficient[0],
                    solved.pressureCoefficient[
                        static_cast<std::size_t>(Cells - 1)],
                    solved.cellsBelowCollapseRisk);
        Check(solved.pressureCoefficient[0] > 0.0
              && solved.pressureCoefficient[
                     static_cast<std::size_t>(Cells - 1)] > 0.0,
              "the whole wing pressurises under a real spanwise solution");
        Check(std::fabs(solved.pressureCoefficient[0]
                        - solved.pressureCoefficient[
                            static_cast<std::size_t>(Cells - 1)]) > 1.0e-6,
              "and the braked half, flying at a different incidence, "
              "pressurises differently - the two solvers are coupled through "
              "the angle of attack, not through a shared constant");
    }

    if (Failures == 0) std::printf("All pressure checks passed.\n");
    else std::printf("%d pressure check(s) failed.\n", Failures);
    return Failures == 0 ? 0 : 1;
}
