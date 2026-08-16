// Level 4: the Vortex Step Method and the section polar table.
//
// The plan's exit gate is agreement with the reference Python/Julia VSM
// implementations. That comparison needs those implementations to hand, so it
// is not what runs here. What runs here is agreement with classical
// lifting-line theory on an elliptical wing, which is exact, analytic and
// needs no external data:
//
//   CL_alpha = 2 pi AR / (AR + 2)      elliptical loading, thin sections
//   CDi      = CL^2 / (pi AR)          minimum induced drag
//
// A solver that reproduces those two on an ellipse has its influence matrix,
// its circulation solve and its force integration right. What it does not yet
// have is any claim about the EPIC 2, because the polars are analytic.
#include "CanopyGeometry.h"
#include "SectionPolarCache.h"
#include "SectionPolarTable.h"
#include "SectionProfile.h"
#include "SectionViscousSolver.h"
#include "ApparentMassTensor.h"
#include "VortexStepMethodSolver.h"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>

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

void CheckWithin(double actual, double expected, double percent,
                 const std::string& what)
{
    const double error = expected == 0.0
        ? actual : 100.0 * (actual - expected) / expected;
    if (std::fabs(error) > percent)
    {
        std::printf("  FAIL  %s: %.5f vs %.5f (%+.2f%%, allowed %g%%)\n",
                    what.c_str(), actual, expected, error, percent);
        ++Failures;
    }
}

// A section that is exactly thin-airfoil: no camber, no thickness correction,
// no profile drag. Anything the solver reports beyond lifting-line theory is
// then the solver's, not the polar's.
SectionPolarTable ThinFlatPlatePolar()
{
    AnalyticPolarSpec spec;
    spec.thicknessFraction = 0.0;
    spec.camberFraction = 0.0;
    spec.minimumDragCoefficient = 0.0;
    spec.dragRiseFactor = 0.0;
    // Push the stall far out so the linear range is clean to test in.
    spec.stallMarginRad = 1.2;
    return SectionPolarTable::Analytic(spec);
}

VsmSolveInput Inflow(double alphaRad, double speedMps = 20.0)
{
    VsmSolveInput input;
    // Flying forward and, at positive incidence, sinking: the wing's velocity
    // through the air has a downward component.
    input.airspeedBodyMps = {
        speedMps * std::cos(alphaRad), 0.0, -speedMps * std::sin(alphaRad)};
    input.airDensityKgM3 = 1.225;
    return input;
}
}

int main()
{
    // -- Level 11 strand 1: the Wagner indicial response -------------------
    //
    // Checked against PUBLISHED values of Jones' approximation rather than
    // against the implementation's own output, which is the only kind of check
    // worth having on a function whose whole purpose is to reproduce a known
    // curve. Phi(0) = 0.5 exactly - a wing gets half its steady lift the
    // instant it changes incidence and the rest as the shed wake convects
    // away - and the tail approaches 1.
    {
        std::printf("\nLevel 11 strand 1: Wagner indicial response "
                    "(Jones' two-lag form)\n");
        struct Row { double s; double phi; };
        // Phi(s) = 1 - 0.165 e^(-0.0455 s) - 0.335 e^(-0.30 s), evaluated
        // independently of the code under test.
        const Row published[] = {
            {0.0, 0.5000}, {1.0, 0.5942}, {2.0, 0.6655},
            {5.0, 0.7938}, {10.0, 0.8786}, {20.0, 0.9328},
        };
        // One long march, sampled at the published points, so this exercises
        // repeated stepping rather than a single closed-form evaluation - the
        // way a flight loop uses it.
        WagnerLag lag;
        lag.Settle(0.0);
        double s = 0.0;
        double worst = 0.0;
        const double stepS = 0.001;
        for (const Row& row : published)
        {
            while (s < row.s - 1.0e-12)
            {
                const double ds = std::min(stepS, row.s - s);
                lag.Advance(1.0, ds);
                s += ds;
            }
            const double got = lag.Value(1.0);
            worst = std::max(worst, std::fabs(got - row.phi));
            std::printf("  s = %5.1f semichords: %.4f against published "
                        "%.4f\n", row.s, got, row.phi);
        }
        Check(worst < 1.0e-3,
              "the indicial response reproduces published Wagner/Jones "
              "values across four decades of reduced time, marched rather "
              "than evaluated");

        // A step response starts at half and rises monotonically. Both halves
        // matter: the 0.5 is the physics that makes this worth having, and
        // monotonicity is what says the two states are not fighting.
        WagnerLag step;
        step.Settle(0.0);
        Check(std::fabs(step.Value(1.0) - 0.5) < 1.0e-12,
              "and a step begins at exactly half the steady value, which is "
              "the whole reason an unsteady formulation changes transient "
              "phase");
        // Marched to s = 100 rather than 20. The slow lag's time constant is
        // 1/0.0455 = 22 semichords, so Phi(20) is 0.93 and still climbing -
        // a wing takes a surprisingly long way to finish arriving at its own
        // steady lift, which is the point of modelling it at all.
        double previous = step.Value(1.0);
        bool monotone = true;
        for (int i = 0; i < 10000; ++i)
        {
            const double now = step.Advance(1.0, 0.01);
            if (now < previous - 1.0e-15) monotone = false;
            previous = now;
        }
        Check(monotone && std::fabs(previous - 1.0) < 0.01,
              "and it rises monotonically to the steady value, so the two "
              "lag states are not fighting each other");

        // A settled wing carries no transient. This is the same rule the
        // canopy's own start-up follows - a simulation that begins mid-flight
        // begins trimmed - and getting it wrong would make every solve start
        // with a lift step it never took.
        WagnerLag settled;
        settled.Settle(1.0);
        Check(std::fabs(settled.Advance(1.0, 10.0) - 1.0) < 1.0e-12,
              "a wing that has held its incidence carries no transient, so "
              "starting mid-flight does not inject a lift step");

        // Reduced time is semichords, not seconds, and mixing them up is the
        // obvious way to use this wrongly. At 11 m/s on a 2.5 m chord one
        // semichord takes 114 ms.
        const double oneSemichordS =
            2.5 / (2.0 * 11.0);
        Check(std::fabs(ReducedTimeSemichords(11.0, 2.5, oneSemichordS) - 1.0)
                  < 1.0e-12,
              "and reduced time counts semichords travelled rather than "
              "seconds elapsed");
    }

    // -- Level 11 strand 2, item 30: does the WIRED lag reproduce Wagner? --
    //
    // Everything above tests the COMPONENT, and it passes. Item 30 measured
    // the composite through the coupled solver and found it closing 12% of a
    // circulation step where Phi(0) is 0.5 - so either the wiring adds lag of
    // its own, or the coupled measurement was reading something else. This
    // block removes the coupled solver entirely: no pressure, no membrane, no
    // collapse, no suspension, no settle. One wing, one airspeed step, and the
    // circulation read straight out of the state it is carried in.
    //
    // THE CONTROL IS THE POINT OF THE BLOCK. The quasi-steady wing takes the
    // same step, and a converged quasi-steady solve reaches its new answer
    // within the solve, so its 'closed' must be ~1 immediately. That is what
    // establishes the denominator every lagged number is quoted against - the
    // one thing the coupled measurement never checked, and the first thing to
    // suspect when arithmetic and measurement disagree.
    {
        std::printf("\nLevel 11 strand 2 (item 30): the WIRED lag against "
                    "Wagner's own function\n");
        const CanopyGeometry canopy;
        const VortexStepMethodSolver wing(
            canopy, SectionPolarTable::Analytic(), 45);

        constexpr double TrimSpeedMps = 11.0;
        constexpr double AlphaRad = 0.06;
        constexpr double StepFactor = 1.20;
        constexpr double StepSeconds = 1.0 / 120.0;

        const auto totalCirculation = [](const VsmSeparationState& state)
        {
            double total = 0.0;
            for (const double gamma : state.circulation) total += gamma;
            return total;
        };
        // Jones' form, written out here rather than reached for from the
        // solver, for the same reason strand 1 does it: a check against the
        // code's own copy of a published curve is not a check.
        const auto phi = [](double s)
        {
            return 1.0 - 0.165 * std::exp(-0.0455 * s)
                       - 0.335 * std::exp(-0.30 * s);
        };

        // Mean chord, for reduced time. The solver spans the projected wing,
        // so its own reference area over its own span is the chord the
        // sections actually have rather than the root chord.
        const double meanChordM = wing.ReferenceAreaM2() / wing.ReferenceSpanM();

        struct Trace
        {
            double before = 0.0;
            std::vector<double> circulation;
            // What the lagged pass aimed at, span-summed. The two places the
            // shortfall can live are the response and the target, and this is
            // what tells them apart.
            std::vector<double> target;
            // How far the target's own iteration settled on the last solve.
            // -1 where the solve did not measure it.
            double targetResidual = -1.0;
        };
        const auto march = [&](bool lagCirculation, int steps,
                               int targetPasses = 1, double alphaRad = AlphaRad)
        {
            VsmSettings settings;
            settings.lagCirculation = lagCirculation;
            settings.lagTargetPasses = targetPasses;
            VsmSeparationState state;
            Trace out;
            // Hold trim long enough that both the separation state and the lag
            // state are settled on it. A transient here would be indexed as a
            // response to the step, which is exactly the seed error strand 2
            // measured and fixed on its own gate.
            for (int step = 0; step < 600; ++step)
                wing.SolveUnsteady(Inflow(alphaRad, TrimSpeedMps), state,
                                   StepSeconds, settings);
            out.before = totalCirculation(state);
            for (int step = 0; step < steps; ++step)
            {
                const VsmSolution solved = wing.SolveUnsteady(
                    Inflow(alphaRad, TrimSpeedMps * StepFactor), state,
                    StepSeconds, settings);
                out.circulation.push_back(totalCirculation(state));
                double aimed = 0.0;
                for (const VsmSectionResult& section : solved.sections)
                    aimed += section.quasiSteadyCirculation;
                out.target.push_back(aimed);
                out.targetResidual = solved.targetResidual;
            }
            return out;
        };

        // 240 steps is 2 s, which at the stepped speed is about twenty
        // semichords - the range over which Phi does most of its arriving.
        const Trace quasi = march(false, 240);
        const Trace lagged = march(true, 240);

        // Incidence is fixed and speed is up by `StepFactor`, and
        // G = 1/2 V c Cl, so the quasi-steady target is the same factor above
        // trim. Asserted through the control below rather than assumed.
        const double gap = (StepFactor - 1.0) * quasi.before;
        std::printf("  trim circulation %.4f quasi-steady, %.4f lagged "
                    "(mean chord %.3f m)\n",
                    quasi.before, lagged.before, meanChordM);
        std::printf("%12s %12s %12s %12s %12s %12s\n", "t", "semichords",
                    "quasi", "lagged", "aimed at", "Wagner Phi");
        double worstLagged = 0.0;
        for (const int step : {1, 2, 6, 12, 30, 60, 120, 240})
        {
            const std::size_t i = static_cast<std::size_t>(step) - 1;
            if (i >= lagged.circulation.size()) continue;
            const double seconds = step * StepSeconds;
            const double s = ReducedTimeSemichords(
                TrimSpeedMps * StepFactor, meanChordM, seconds);
            const double closedQuasi =
                (quasi.circulation[i] - quasi.before) / gap;
            const double closedLagged =
                (lagged.circulation[i] - lagged.before) / gap;
            worstLagged = std::max(worstLagged, std::fabs(closedLagged
                                                          - phi(s)));
            const double aimed = (lagged.target[i] - lagged.before) / gap;
            std::printf("%11.3fs %12.2f %12.3f %12.3f %12.3f %12.3f\n",
                        seconds, s, closedQuasi, closedLagged, aimed, phi(s));
        }

        // THE CONTROL, and if this fails nothing else in the block is
        // readable: the quasi-steady wing has to arrive at the target inside
        // its own solve, which is what makes `gap` the right denominator.
        Check(std::fabs((quasi.circulation.front() - quasi.before) / gap - 1.0)
                  < 0.05,
              "CONTROL: the quasi-steady wing closes its circulation step "
              "within one solve, which is what makes the target 1.2x trim and "
              "the denominator below meaningful");

        // The measurement. NOT a tolerance the code passes - it is the size of
        // a disagreement, bounded so it cannot move unnoticed while item 30 is
        // open. Wagner's defining feature is that half the lift arrives at
        // once; strand 1 checks that on the component and it holds there.
        const double firstStep =
            (lagged.circulation.front() - lagged.before) / gap;
        std::printf("  first step closes %.3f where Wagner's Phi(0) is 0.500, "
                    "worst gap %.3f\n", firstStep, worstLagged);
        Check(firstStep < 0.45,
              "KNOWN DEFECT (item 30): the wired lag delivers far less than "
              "Wagner's instantaneous half. Strand 1's component check passes "
              "and this composite one does not, so the wiring carries lag the "
              "published function does not describe. Bounded, not fixed");

        // AND THE DEFECT IS LOCATED, WHICH IS WHAT THE 'aimed at' COLUMN IS
        // FOR. The shortfall is not in the response - it is in the target.
        // Wagner is doing exactly its job, onto a target that has itself
        // travelled only a fraction of the way, so the two multiply:
        //
        //     closed = Phi(s) x (how far the one-pass target has moved)
        //
        // Checked as a product rather than asserted as a story, because a
        // mechanism that reproduces the number to three decimals is identified
        // and one that merely points the right way is a hypothesis.
        const double aimedFirst = (lagged.target.front() - lagged.before) / gap;
        const double sFirst = ReducedTimeSemichords(
            TrimSpeedMps * StepFactor, meanChordM, StepSeconds);
        std::printf("  and it decomposes: Phi(%.3f) %.3f x target %.3f = "
                    "%.3f against a measured %.3f\n",
                    sFirst, phi(sFirst), aimedFirst, phi(sFirst) * aimedFirst,
                    firstStep);
        Check(std::fabs(phi(sFirst) * aimedFirst - firstStep) < 0.01,
              "ITEM 30 LOCATED: the response is Wagner's and the TARGET is "
              "not. One Jacobi pass across sections closes under a quarter of "
              "a circulation step, and the lag is applied to that - the two "
              "multiply, and their product is the measured shortfall");

        // WHICH CONTRADICTS THE ASSUMPTION STRAND 2 WAS BUILT ON. Its design
        // note drops the global fixed point across sections on the grounds
        // that it is "the coupling the quasi-steady path already documents as
        // the weak one". The quasi-steady column above closes 1.000 in one
        // solve because its outer loop ITERATES; one pass of the same coupling
        // leaves three quarters of the step on the table.
        Check(aimedFirst < 0.5,
              "and the cross-section coupling strand 2 dropped as 'the weak "
              "one' carries most of a circulation step: one Jacobi pass closes "
              "under half of it, where the iterated solve closes all of it");

        // -- THE DESIGN MEASUREMENT: what does the target cost? ------------
        //
        // Item 30's remaining half is a decision, and this is the number it
        // turns on. If the target needs a handful of passes, iterating it is
        // affordable and the composite can be made to follow Wagner. If it
        // needs hundreds, or if the count explodes where item 6 says there is
        // no fixed point, then a converged target is not available and the
        // scheme has to be something else.
        //
        // Reported at trim AND deep in the separated regime, because those are
        // the two regimes with different answers and the second is the one
        // strand 2 exists for.
        // 25 degrees is well past the section stall - deep in the separated
        // branch, which is where item 6 says the negative lift slope inverts
        // the downwash feedback and the steady solve has nothing single-valued
        // to converge to. Each case computes its own gap from its own trim,
        // because the separated wing does not carry the attached one's
        // circulation.
        constexpr double SeparatedAlphaRad = 0.436;
        const auto closureAt = [&](int passes, double alphaRad)
        {
            const Trace t = march(true, 1, passes, alphaRad);
            const double ownGap = (StepFactor - 1.0) * t.before;
            return std::fabs(ownGap) > 1.0e-9
                ? (t.target.front() - t.before) / ownGap : 0.0;
        };
        // The quasi-steady control at each incidence, which is what "closed"
        // is measured against. If the separated one does NOT reach 1.000, that
        // is item 6 showing up rather than a problem with this table - and it
        // is the answer to the design question either way, so it prints.
        const Trace quasiSeparated = march(false, 1, 1, SeparatedAlphaRad);
        const double quasiSeparatedGap =
            (StepFactor - 1.0) * quasiSeparated.before;
        const double quasiSeparatedClosed =
            std::fabs(quasiSeparatedGap) > 1.0e-9
                ? (quasiSeparated.circulation.front() - quasiSeparated.before)
                      / quasiSeparatedGap
                : 0.0;

        // THE DENOMINATOR, PRINTED BEFORE ANY RATIO BUILT ON IT. A ratio whose
        // divisor has not been shown is the mistake this item already made
        // once, and the separated wing is exactly where a trim circulation
        // could be near zero, oscillating, or signed - any of which would make
        // the column beside it meaningless rather than interesting.
        const Trace separatedTrim = march(true, 1, 1, SeparatedAlphaRad);
        std::printf("\n  Trim circulation the separated column is measured "
                    "against: %.4f, against\n  %.4f attached. Gap %.4f. A "
                    "denominator that is sane is what makes the\n  numbers "
                    "below a measurement rather than a division.\n",
                    separatedTrim.before, quasi.before,
                    (StepFactor - 1.0) * separatedTrim.before);

        std::printf("\n  What the target costs, in Jacobi passes per solve:\n");
        std::printf("%14s %16s %18s\n", "passes", "closed at trim",
                    "closed at 25 deg");
        for (const int passes : {1, 2, 4, 8, 16, 32, 64})
            std::printf("%14d %16.3f %18.3f\n", passes,
                        closureAt(passes, AlphaRad),
                        closureAt(passes, SeparatedAlphaRad));
        std::printf("%14s %16.3f %18.3f\n", "quasi-steady", 1.000,
                    quasiSeparatedClosed);
        std::printf("\n  'closed' is the TARGET, not the state - what one "
                    "solve's passes reach before\n  Wagner is applied. The "
                    "bottom row is the iterated quasi-steady solve, which is\n"
                    "  what a converged target would be worth.\n\n");

        // THE DESIGN ANSWER, AND IT HAS TWO HALVES THAT POINT OPPOSITE WAYS.
        //
        // ATTACHED: the target converges monotonically, and ~32 passes is
        // PARITY with the 40-iteration cap the shipped flight solve already
        // pays for and which item 19 measured converged. Affordable.
        Check(closureAt(32, AlphaRad) > 0.95,
              "ITEM 30: with the flow attached the target converges, in about "
              "the same number of passes the shipped quasi-steady solve "
              "already pays for - so a target Wagner can honestly be applied "
              "to costs no more than today's aerodynamics");

        // SEPARATED: it does not converge at ANY fixed budget up to 64, and it
        // does not merely fail to arrive - it lands 1 to 5 times the step away,
        // on the wrong side, and WHERE it lands depends on the budget. That is
        // not an unconverged solve, it is an iteration with nothing attracting
        // it, which is item 6 measured as a function of pass count for the
        // first time.
        //
        // So "iterate the target" is not the fix: it works where the flow is
        // attached and fails where it is separated, which is the regime strand
        // 2 exists for. The full 600-iteration adaptive solve does land (the
        // quasi-steady row), which is why the quasi-steady path works at all -
        // but it gets there by an amount of work that has no bound, and buying
        // a fixed-cost state with an unbounded solve inside it is not a state.
        bool monotoneSeparated = true;
        double previousSeparated = closureAt(1, SeparatedAlphaRad);
        for (const int passes : {2, 4, 8, 16, 32, 64})
        {
            const double now = closureAt(passes, SeparatedAlphaRad);
            if (now < previousSeparated) monotoneSeparated = false;
            previousSeparated = now;
        }
        Check(!monotoneSeparated
                  && std::fabs(closureAt(64, SeparatedAlphaRad)) > 1.0,
              "AND THE OTHER HALF: past the stall the target is non-monotone "
              "in the pass count and lands whole multiples of the step away, "
              "so a fixed budget does not buy a converged target where item 6 "
              "says there is no fixed point. Iterating is not the fix");

        // -- IS THE COMPOSITE WAGNER'S ONCE THE TARGET IS CONVERGED? -------
        //
        // Everything above measured the TARGET, which is one factor of the
        // product. Nobody has yet marched the wing on a converged target and
        // asked the original question again: does `closed(s)` become Phi(s)?
        //
        // It is not a formality. The state is advanced from its own registers
        // once per solve, and the passes that build the target start FROM the
        // lagged state rather than from the previous target - so the fixed
        // point they converge to is the quasi-steady answer for a wing whose
        // downwash is the lagged one. Whether that reproduces the indicial
        // response of the published function is a measurement, and if it does
        // not, "iterate the target" would have been wrong on the attached side
        // too and not only on the separated one.
        const Trace converged = march(true, 240, 64);
        std::printf("\n  The same sweep on a CONVERGED target (64 passes), "
                    "which is the arrangement\n  in which Jones' function is "
                    "supposed to mean what it says:\n");
        std::printf("%12s %12s %14s %12s %12s\n", "t", "semichords",
                    "closed (64)", "closed (1)", "Wagner Phi");
        double worstConverged = 0.0;
        for (const int step : {1, 2, 6, 12, 30, 60, 120, 240})
        {
            const std::size_t i = static_cast<std::size_t>(step) - 1;
            if (i >= converged.circulation.size()) continue;
            const double seconds = step * StepSeconds;
            const double s = ReducedTimeSemichords(
                TrimSpeedMps * StepFactor, meanChordM, seconds);
            const double closed =
                (converged.circulation[i] - converged.before) / gap;
            worstConverged = std::max(worstConverged,
                                      std::fabs(closed - phi(s)));
            std::printf("%11.3fs %12.2f %14.3f %12.3f %12.3f\n", seconds, s,
                        closed,
                        (lagged.circulation[i] - lagged.before) / gap, phi(s));
        }
        const double convergedFirst =
            (converged.circulation.front() - converged.before) / gap;
        std::printf("  first step closes %.3f against Phi(0) = 0.500, worst "
                    "gap over the sweep %.3f\n  (one pass: %.3f and %.3f)\n",
                    convergedFirst, worstConverged, firstStep, worstLagged);

        // THE HALF THAT WORKS, ASSERTED AS THE PUBLISHED FUNCTION AND NOT AS
        // AN IMPROVEMENT. "Closer than one pass" would pass for a wing still
        // several times slow; the check is against Jones' curve itself, at
        // every row of the sweep.
        Check(worstConverged < 0.05,
              "ITEM 30, THE ATTACHED HALF CLOSED: with the target converged "
              "the composite response IS Wagner's, across the whole sweep and "
              "not merely at the first step. The lag the wing carried twice is "
              "carried once, and the once is the published one");
        Check(worstConverged < 0.5 * worstLagged,
              "and it is the same solver and the same harness that measured "
              "the defect, so the improvement is a change of scheme rather "
              "than a change of instrument");

        // -- CAN THE SEPARATED REGIME BE DECLARED RATHER THAN DISCOVERED? --
        //
        // The remaining route in item 30's list is the third: iterate where
        // that converges and degrade deliberately where it does not, "honest
        // only if the degradation is declared and gated rather than discovered
        // later". Declaring it needs a signal available INSIDE the solve, at
        // the fixed cost the state is supposed to have. `residual` is not that
        // signal - under lag it reports the distance the state still has to
        // travel, which is large on a healthy solve. `targetResidual` is.
        //
        // This does not choose the route. It measures whether the route is
        // available at all, which is what "not chosen without the collapse
        // gates in front of it" leaves open and what a gate would need.
        std::printf("\n  Does the solve KNOW which regime it is in? The "
                    "target's own residual,\n  at fixed cost, on the same two "
                    "incidences:\n");
        std::printf("%14s %20s %20s\n", "passes", "target resid trim",
                    "target resid 25 deg");
        const auto residualAt = [&](int passes, double alphaRad)
        {
            return march(true, 1, passes, alphaRad).targetResidual;
        };
        for (const int passes : {2, 4, 8, 16, 32, 64})
            std::printf("%14d %20.3e %20.3e\n", passes,
                        residualAt(passes, AlphaRad),
                        residualAt(passes, SeparatedAlphaRad));
        std::printf("  A single pass reports %.1f at both, which is the "
                    "sentinel: one pass has no\n  pass-to-pass change, and "
                    "reporting zero there would claim convergence about\n  a "
                    "quantity nobody measured.\n\n",
                    residualAt(1, AlphaRad));

        Check(residualAt(1, AlphaRad) < 0.0
                  && residualAt(1, SeparatedAlphaRad) < 0.0,
              "one pass reports NOT MEASURED rather than converged, which is "
              "the shipped setting and the one where a false green would be "
              "worst");
        Check(residualAt(32, AlphaRad) < 1.0e-3,
              "attached, the target's own residual falls to where a gate could "
              "read it - so 'converged' is a claim the solve can make about "
              "itself rather than one the test makes on its behalf");
        Check(residualAt(32, SeparatedAlphaRad)
                  > 100.0 * residualAt(32, AlphaRad),
              "AND SEPARATED IT DOES NOT, BY ORDERS OF MAGNITUDE. The regime "
              "where iterating the target fails is visible from inside the "
              "solve at fixed cost, so route 3's degradation can be DECLARED "
              "on the tick it happens instead of surfacing later as a collapse "
              "gate nobody can attribute");

        // -- ITEM 6's MECHANISM, MEASURED RATHER THAN ASSERTED -------------
        //
        // Three iterations have now ended at the same place: item 6 is
        // upstream of everything Level 11 was going to build, and nothing in
        // strand 2 supplies what it takes away. So it is worth asking whether
        // item 6's own sentence is right, because it has never been measured.
        //
        // What it says: "the separated branch has a negative lift slope, which
        // inverts the downwash feedback between sections; a wing in deep stall
        // has no stable steady state to find." That is a MECHANISM, and it
        // makes a sharp prediction. The cross-section iteration is a fixed
        // point whose gain runs through dCl/dalpha - each section's circulation
        // sets a downwash on its neighbours, which moves their incidence, which
        // moves their lift by the slope. If the slope's SIGN is what breaks it,
        // then the iteration's contraction factor must pass 1 where the slope
        // passes zero, and nowhere else.
        //
        // THE FIRST INSTRUMENT FOR THIS WAS WRONG AND IT IS WORTH RECORDING
        // WHY, because it is the obvious one. A contraction factor - the ratio
        // of target residuals at k and k+1 passes - is the textbook statistic
        // for "does this iteration converge", and it does not distinguish
        // these regimes at all: at 25 degrees, where the table above measures
        // the target landing one to five times the step away and wandering
        // with the budget, the 8-to-9 ratio is 0.429. A single pair of passes
        // cannot tell contraction from an iterate part-way round a cycle.
        //
        // What does distinguish them is where the residual ENDS UP over a real
        // budget. A convergent iteration drives it toward zero; a wandering
        // one has a floor it never gets under, and that floor is the thing
        // item 6 is about. So the statistic here is the residual after 64
        // passes, with the 8-pass value printed beside it: the pair says
        // whether the budget bought anything.
        std::printf("\n  Item 6's mechanism: is the non-convergence the SIGN "
                    "of the lift slope?\n");
        std::printf("%10s %14s %14s %14s %10s %6s\n", "alpha", "dCl/dalpha",
                    "resid @8", "resid @64", "8x bought", "conv");
        const SectionPolarTable polars = SectionPolarTable::Analytic();
        const double stallAngleRad = polars.StallAngleRad(0.0);
        const auto residualAfter = [&](int passes, double alphaRad)
        {
            return march(true, 1, passes, alphaRad).targetResidual;
        };
        // Local slope from the polar the SOLVER samples, by central
        // difference at the separation that incidence settles to - not the
        // linear-range slope, which is a different number past the knee and
        // would make the sign change appear in the wrong place.
        const auto localSlope = [&](double alphaRad)
        {
            constexpr double Delta = 0.5 * Pi / 180.0;
            const auto clAt = [&](double a)
            {
                const double separation =
                    polars.SeparationEquilibrium(a, 0.0, 0.0);
                return polars.SampleAtSeparation(a, 0.0, separation, 1.0)
                    .liftCoefficient;
            };
            return (clAt(alphaRad + Delta) - clAt(alphaRad - Delta))
                / (2.0 * Delta);
        };

        // AND THE CRITERION IS A RATIO, NOT A MAGNITUDE, WHICH IS THE SECOND
        // INSTRUMENT THIS BLOCK HAD TO FIX. The obvious test is "is the
        // residual small", and any threshold for that is a number somebody
        // chose: the attached floor is 2.6e-06 and the solver's own tolerance
        // is 1e-06, so reading the solver's constant literally calls a solve
        // that has converged to five decimal places non-convergent, and any
        // looser number is picked to make the answer come out.
        //
        // What needs no chosen constant is whether the BUDGET BOUGHT ANYTHING.
        // Eight times the passes either drives the residual down by orders or
        // it does not, and that is a statement about the iteration rather than
        // about a scale. Attached, the eightfold budget buys three and a half
        // orders. Separated, it buys nothing - and in one row the residual is
        // LARGER at 64 passes than at 8.
        constexpr double BudgetMustBuy = 100.0;
        double slopeSignChange = -1.0;
        double convergenceLost = -1.0;
        double previousSlope = 0.0;
        bool first = true;
        for (const double alphaDeg :
             {2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 18.0, 25.0})
        {
            const double alphaRad = alphaDeg * Pi / 180.0;
            const double slope = localSlope(alphaRad);
            const double atEight = residualAfter(8, alphaRad);
            const double floorAt64 = residualAfter(64, alphaRad);
            const double bought = floorAt64 > 0.0 ? atEight / floorAt64 : 0.0;
            const bool converges = bought > BudgetMustBuy;
            std::printf("%9.1f%s %14.3f %14.2e %14.2e %9.0fx %6s\n", alphaDeg,
                        " deg", slope, atEight, floorAt64, bought,
                        converges ? "yes" : "NO");
            if (!first && slopeSignChange < 0.0 && previousSlope > 0.0
                && slope <= 0.0)
                slopeSignChange = alphaDeg;
            if (convergenceLost < 0.0 && !converges) convergenceLost = alphaDeg;
            previousSlope = slope;
            first = false;
        }
        std::printf("  lift slope changes sign by %.1f deg; the iteration "
                    "stops converging by %.1f deg;\n  the polar's own stall "
                    "angle is %.1f deg.\n  'conv' is whether eight times the "
                    "budget bought at least %.0fx off the residual.\n\n",
                    slopeSignChange, convergenceLost,
                    stallAngleRad * 180.0 / Pi, BudgetMustBuy);

        // THE MECHANISM, CONFIRMED OR NOT. Both crossings are located
        // independently - one from the polar alone, with no solve in it, and
        // one from the iteration alone, with no polar in it - so agreeing is
        // a measurement rather than a restatement.
        Check(slopeSignChange > 0.0 && convergenceLost > 0.0,
              "both crossings are located inside the swept range, so the "
              "comparison below is between two measured numbers rather than "
              "between a number and a missing one");
        Check(std::fabs(convergenceLost - slopeSignChange) <= 2.0,
              "ITEM 6's MECHANISM, MEASURED FOR THE FIRST TIME: the "
              "cross-section iteration stops contracting at the incidence "
              "where the section's lift slope changes sign. The sentence item "
              "6 has carried as an assertion since it was written is the one "
              "the solver obeys");

        // AND THE CONSEQUENCE THAT MAKES IT USEFUL. If the criterion is the
        // slope, it is available from the POLAR - a table lookup at a section's
        // own incidence, before any iterating - rather than only from watching
        // an iteration fail. That is what route 3 needs to declare its
        // degradation on entry instead of detecting it after the fact, and it
        // costs a sample.
        Check(localSlope(stallAngleRad + 6.0 * Pi / 180.0) < 0.0,
              "and the criterion is available from the polar alone, at a "
              "section's own incidence, without running the iteration that "
              "would fail - which is what lets a scheme declare the separated "
              "regime on entry rather than diagnose it afterwards");

        // -- AND CAN THE SOLVER ALREADY SEE IT, WITH NOTHING ADDED? --------
        //
        // The criterion is a polar lookup, which is cheap but not free: on
        // the tick it would cost two samples a section. Before adding a field
        // for it, the question worth asking is whether the wing ALREADY
        // carries something that locates the same crossing - because the
        // separation state is exactly "how separated is this section", it is
        // already updated every tick, and if it moves at 12 degrees too then
        // the answer to route 3's signal is a comparison rather than a new
        // quantity.
        //
        // Measuring that before building the field is the point. A new
        // diagnostic added on the assumption that nothing else reports this
        // would be a second description of a state the solver already has.
        std::printf("  Does a state the solver ALREADY carries locate the "
                    "same crossing?\n");
        std::printf("%10s %14s %18s\n", "alpha", "dCl/dalpha",
                    "separation eqm");
        double separationAtSignChange = -1.0;
        double separationBelowIt = -1.0;
        for (const double alphaDeg : {8.0, 10.0, 11.0, 12.0, 14.0, 25.0})
        {
            const double alphaRad = alphaDeg * Pi / 180.0;
            const double separation =
                polars.SeparationEquilibrium(alphaRad, 0.0, 0.0);
            std::printf("%9.1f%s %14.3f %18.3f\n", alphaDeg, " deg",
                        localSlope(alphaRad), separation);
            if (alphaDeg == 12.0) separationAtSignChange = separation;
            if (alphaDeg == 10.0) separationBelowIt = separation;
        }
        std::printf("\n");

        // WHETHER IT DOES IS THE MEASUREMENT, AND THE ANSWER DECIDES WHETHER
        // ANY FIELD GETS ADDED AT ALL. If separation moves sharply across the
        // same two degrees the slope's sign does, then the wing already
        // reports the criterion and route 3's signal costs nothing new.
        // -- WHERE DOES THE ASYMMETRY COME FROM AT ALL? --------------------
        //
        // The coupled suite now measures that a mirror-symmetric gust turns
        // the shipped wing at 0.737 rad/s and the elapsed-time-corrected wing
        // at 2.831. Both are the solver choosing a direction it has no right
        // to choose, and the whole ladder attributes that to item 6: "the
        // separated solve has nothing single-valued to converge to".
        //
        // THAT EXPLANATION DOES NOT ACTUALLY EXPLAIN IT, and it took ten
        // iterations to notice. Non-convergence is not a symmetry-breaking
        // operation. A symmetric problem, solved by a deterministic algorithm
        // that treats the two half-spans identically, stays symmetric to the
        // last bit whether or not it converges - an iterate with no fixed
        // point still wanders SYMMETRICALLY. To get a direction out, something
        // in the chain has to actually be asymmetric: the geometry, the
        // influence matrix, or an order-dependent step in the solve.
        //
        // So this checks the three places it can live, bitwise, before any
        // physics is involved. Exact equality is the right test and not a
        // strict one: these are mirrored copies of the same arithmetic, so
        // they agree exactly or something is not mirrored.
        {
            std::printf("\n  Where can a direction come from? Bitwise mirror "
                        "symmetry, before physics:\n");
            const CanopyGeometry canopy;
            const VortexStepMethodSolver wing(
                canopy, SectionPolarTable::Analytic(), 45);
            const std::vector<VsmSection>& sections = wing.Sections();
            const std::size_t count = sections.size();

            // Span position first: if the mesh itself is not mirrored, nothing
            // downstream can be.
            double worstSpan = 0.0;
            double worstChord = 0.0;
            for (std::size_t i = 0; i < count / 2; ++i)
            {
                const std::size_t m = count - 1 - i;
                worstSpan = std::max(worstSpan,
                    std::fabs(sections[i].controlPointM.y
                              + sections[m].controlPointM.y));
                worstChord = std::max(worstChord,
                    std::fabs(sections[i].chordM - sections[m].chordM));
            }
            std::printf("    geometry: worst mirrored span %.3e, worst "
                        "chord difference %.3e\n", worstSpan, worstChord);

            // The influence matrix. Section i's downwash from section j must
            // equal its mirror's from j's mirror, or the two half-spans are
            // solving different problems.
            double worstInfluence = 0.0;
            for (std::size_t i = 0; i < count; ++i)
                for (std::size_t j = 0; j < count; ++j)
                {
                    const std::size_t mi = count - 1 - i;
                    const std::size_t mj = count - 1 - j;
                    const Vec3 a = wing.InfluenceAt(i, j);
                    const Vec3 b = wing.InfluenceAt(mi, mj);
                    // The y component mirrors with a sign flip; x and z do
                    // not. Comparing the wrong parity here would manufacture
                    // an asymmetry that is not there.
                    worstInfluence = std::max(worstInfluence,
                        std::max(std::fabs(a.x - b.x),
                                 std::max(std::fabs(a.y + b.y),
                                          std::fabs(a.z - b.z))));
                }
            std::printf("    influence matrix: worst mirrored difference "
                        "%.3e\n", worstInfluence);

            // And one solve, at an incidence where the flow is attached and
            // item 6 does not apply at all. If the answer is asymmetric HERE,
            // the asymmetry has nothing to do with separation.
            VsmSeparationState state;
            const VsmSolution solved =
                wing.SolveUnsteady(Inflow(0.06, 11.0), state, 1.0 / 120.0, {});
            double worstCirculation = 0.0;
            for (std::size_t i = 0; i < count / 2; ++i)
                worstCirculation = std::max(worstCirculation,
                    std::fabs(solved.sections[i].circulation
                              - solved.sections[count - 1 - i].circulation));
            std::printf("    one attached solve: worst mirrored circulation "
                        "%.3e\n\n", worstCirculation);

            // AND THE ANSWER IS THAT THE SEED IS THERE FROM THE START. The
            // wing is mirror-symmetric to ROUND-OFF and not to the bit, at
            // every stage, before any separation exists: the mesh by 5.6e-15,
            // the influence matrix by 1.5e-14, and an attached solve at trim
            // by 9.9e-14. Nothing here is separated and nothing here is
            // non-convergent - this is the solve the whole aircraft flies on.
            //
            // Which corrects §68. Its gate says the wing "enters the collapse
            // mirror-symmetric - the asymmetry it leaves with is acquired
            // during the event, not carried into it", and that reads as
            // symmetric-to-zero. It is symmetric to 1e-15, which is a
            // different statement: the direction IS carried in, and what the
            // event does is amplify it about thirteen orders to O(1).
            //
            // AND THE SEED CANNOT BE REMOVED BY BUILDING A TIDIER MESH. Each
            // section's downwash is accumulated over j = 0..n, so a section
            // and its mirror sum the SAME terms in OPPOSITE ORDER, and
            // floating-point addition is not associative. Even given a
            // perfectly mirrored mesh the two half-spans would disagree in the
            // last bits. A seed at round-off is unavoidable in this algorithm.
            //
            // So the fix is not to remove the seed - it is to remove the
            // AMPLIFICATION, which is item 6. That is what the ladder has been
            // calling the blocker all along, and this is the first measurement
            // that says why it is the blocker rather than merely upstream.
            Check(worstSpan > 0.0 && worstSpan < 1.0e-12,
                  "THE SEED IS IN THE MESH: the wing is mirror-symmetric to "
                  "round-off and NOT to the bit, before any physics. A "
                  "deterministic symmetric algorithm cannot invent a "
                  "direction, so it did not have to - one was there to amplify");
            Check(worstInfluence > 0.0 && worstInfluence < 1.0e-12,
                  "and the influence matrix carries it too, at 1e-14, so the "
                  "two half-spans are solving problems that differ in their "
                  "last bits");
            Check(worstCirculation > 0.0 && worstCirculation < 1.0e-12,
                  "and an ATTACHED solve at trim already returns a mirrored "
                  "circulation differing at 1e-13 - no separation, no "
                  "non-convergence, and the direction is already present. What "
                  "the separated regime adds is thirteen orders of "
                  "amplification, not the asymmetry itself");
        }

        // -- AND WHAT DOES THE AMPLIFYING? THE SPECTRUM OF ONE PASS --------
        //
        // The block above ends by naming the thing to remove: not the seed,
        // which is unavoidable, but the AMPLIFICATION that turns 1e-14 into
        // O(1). "Amplification" was a word there. Here it is a number.
        //
        // The outer loop is a fixed-point iteration, Gamma <- G(Gamma), one
        // Jacobi pass across sections with each section's own circulation
        // taken implicitly. Whether a perturbation grows is therefore not a
        // question about stall, or about physics at all: it is the spectral
        // radius of dG/dGamma. Under-relaxation replaces that operator with
        // (1-w) I + w J, which is the only knob the shipped solver has.
        //
        // AND THE PERTURBATION THAT MATTERS HAS A PARITY. The seed §68's
        // successor found is MIRROR-ANTISYMMETRIC - the two half-spans
        // disagreeing in their last bits - and a wing whose symmetric modes
        // are perfectly damped will still turn if its antisymmetric ones are
        // not. Nothing in the record has ever separated the two. On a wing
        // that is mirror-symmetric to round-off, J commutes with the mirror to
        // the same precision, so the two subspaces can be iterated separately
        // and each reports its own growth per pass.
        //
        // Measured by linearising the shipped pass about the wing's own
        // operating point, at the incidences item 6's own table sweeps.
        {
            std::printf("\n  What amplifies it? Growth per Jacobi pass, "
                        "linearised, split by mirror parity:\n");
            const CanopyGeometry canopy;
            const SectionPolarTable polars = SectionPolarTable::Analytic();
            const VortexStepMethodSolver wing(canopy, polars, 45);
            const std::size_t count = wing.Sections().size();

            const auto mirrored = [count](const std::vector<double>& v)
            {
                std::vector<double> m(count);
                for (std::size_t i = 0; i < count; ++i) m[i] = v[count - 1 - i];
                return m;
            };
            // parity +1 keeps the mirror-symmetric half of a vector, -1 the
            // antisymmetric half. Applied every step, so an iterate cannot
            // drift into the other subspace through round-off and report the
            // dominant mode twice.
            const auto project = [&](std::vector<double> v, double parity)
            {
                const std::vector<double> m = mirrored(v);
                for (std::size_t i = 0; i < count; ++i)
                    v[i] = 0.5 * (v[i] + parity * m[i]);
                return v;
            };
            const auto norm = [](const std::vector<double>& v)
            {
                double sum = 0.0;
                for (const double x : v) sum += x * x;
                return std::sqrt(sum);
            };

            // The separation the incidence settles to, held. Item 6 is about
            // the branch, so the branch has to be the one the wing is on.
            const auto separationAt = [&](double alphaRad)
            {
                double s = 0.0;
                for (int k = 0; k < 200; ++k)
                    s = polars.SeparationEquilibrium(alphaRad, 0.0, s);
                return s;
            };

            // dG/dGamma by central differences on ONE pass of the shipped
            // solve, undamped. Central rather than forward because the polar
            // has a knee in it and a one-sided difference straddling the knee
            // reports the knee rather than the slope.
            const auto jacobianAt = [&](double alphaRad)
            {
                VsmSeparationState state;
                state.sectionSeparation.assign(count, separationAt(alphaRad));
                state.initialised = true;
                const VsmSolveInput input = Inflow(alphaRad, 11.0);

                std::vector<double> base(count, 0.0);
                wing.SolveFrozen(input, state, base, {});
                // Linearise about an EXACTLY symmetric point, so the parity
                // split below is a property of the operator rather than of
                // where it happened to be measured.
                base = project(base, 1.0);

                VsmSettings onePass;
                onePass.maxIterations = 1;
                onePass.relaxation = 1.0;
                const auto pass = [&](std::vector<double> gamma)
                {
                    wing.SolveFrozen(input, state, gamma, onePass);
                    return gamma;
                };

                const double scale = std::max(
                    1.0e-6, norm(base) / std::sqrt(static_cast<double>(count)));
                const double epsilon = 1.0e-6 * scale;
                std::vector<double> jacobian(count * count, 0.0);
                for (std::size_t j = 0; j < count; ++j)
                {
                    std::vector<double> up = base;
                    std::vector<double> down = base;
                    up[j] += epsilon;
                    down[j] -= epsilon;
                    up = pass(up);
                    down = pass(down);
                    for (std::size_t i = 0; i < count; ++i)
                        jacobian[i * count + j] =
                            (up[i] - down[i]) / (2.0 * epsilon);
                }
                return jacobian;
            };

            // Growth per pass of the damped operator (1-w) I + w J, inside one
            // parity. Taken as the geometric mean of the last hundred steps
            // rather than the last one: a complex pair does not settle to a
            // single ratio, and reading one step of it would report a number
            // that depends on where in the cycle the loop stopped. This is the
            // instrument the earlier contraction factor should have been.
            const auto growthOf = [&](const std::vector<double>& jacobian,
                                      double parity, double omega)
            {
                std::vector<double> v(count);
                for (std::size_t i = 0; i < count; ++i)
                    v[i] = std::sin(2.399963 * static_cast<double>(i + 1));
                v = project(v, parity);
                double scale = norm(v);
                if (!(scale > 0.0)) return 0.0;
                for (double& x : v) x /= scale;

                double logSum = 0.0;
                int counted = 0;
                for (int k = 0; k < 200; ++k)
                {
                    std::vector<double> w(count, 0.0);
                    for (std::size_t i = 0; i < count; ++i)
                        for (std::size_t j = 0; j < count; ++j)
                            w[i] += jacobian[i * count + j] * v[j];
                    for (std::size_t i = 0; i < count; ++i)
                        w[i] = (1.0 - omega) * v[i] + omega * w[i];
                    w = project(w, parity);
                    const double grew = norm(w);
                    if (!(grew > 0.0)) return 0.0;
                    if (k >= 100) { logSum += std::log(grew); ++counted; }
                    for (double& x : w) x /= grew;
                    v = w;
                }
                return std::exp(logSum / std::max(1, counted));
            };

            // The SIGN of the dominant eigenvalue, which is the whole question
            // for damping. Under-relaxation moves lambda to 1 + w (lambda - 1):
            // a real NEGATIVE eigenvalue outside the unit disc can always be
            // pulled inside it by a small enough w, and a real POSITIVE one
            // greater than 1 can never be, because every w > 0 leaves the
            // result greater than 1. Read as the Rayleigh quotient of the
            // converged iterate, which carries the sign the norm above throws
            // away.
            const auto dominantSign = [&](const std::vector<double>& jacobian,
                                          double parity)
            {
                std::vector<double> v(count);
                for (std::size_t i = 0; i < count; ++i)
                    v[i] = std::sin(2.399963 * static_cast<double>(i + 1));
                v = project(v, parity);
                double scale = norm(v);
                if (!(scale > 0.0)) return 0.0;
                for (double& x : v) x /= scale;
                double quotient = 0.0;
                for (int k = 0; k < 200; ++k)
                {
                    std::vector<double> w(count, 0.0);
                    for (std::size_t i = 0; i < count; ++i)
                        for (std::size_t j = 0; j < count; ++j)
                            w[i] += jacobian[i * count + j] * v[j];
                    w = project(w, parity);
                    quotient = 0.0;
                    for (std::size_t i = 0; i < count; ++i)
                        quotient += v[i] * w[i];
                    const double grew = norm(w);
                    if (!(grew > 0.0)) return 0.0;
                    for (double& x : w) x /= grew;
                    v = w;
                }
                return quotient;
            };

            // AND WHERE IN THE OPERATOR THE GAIN LIVES. Item 6's sentence puts
            // it in the coupling BETWEEN sections - "inverts the downwash
            // feedback between sections" - and that is a claim about the
            // OFF-DIAGONAL of this matrix. The diagonal is a different
            // mechanism: each section's own circulation is solved implicitly
            // against its own trailing legs, and that implicit solve has a
            // gain of 1/(1 - dSelf) which runs away on its own when the
            // section's own feedback approaches unity. The two are told apart
            // by looking, which nothing has done.
            // Attributed at the level of the OPERATOR rather than of a norm:
            // the same growth measurement run on the matrix with its diagonal
            // deleted, and on the matrix with everything BUT its diagonal
            // deleted. Comparing a single diagonal entry against a row sum of
            // forty-four off-diagonal ones would answer a question about
            // arithmetic; this answers the one item 6 asks, which is which
            // term drives the iterate.
            const auto keepOnly = [&](const std::vector<double>& jacobian,
                                      bool diagonal)
            {
                std::vector<double> out(count * count, 0.0);
                for (std::size_t i = 0; i < count; ++i)
                    for (std::size_t j = 0; j < count; ++j)
                        if ((i == j) == diagonal)
                            out[i * count + j] = jacobian[i * count + j];
                return out;
            };

            std::printf("%10s %12s %12s %14s %12s %11s %11s %10s\n", "alpha",
                        "sym/pass", "anti/pass", "best w", "anti @ best",
                        "self only", "coupling", "sign");
            double attachedAnti = -1.0;
            double separatedAnti = -1.0;
            double separatedSym = -1.0;
            double separatedBest = -1.0;
            double separatedSign = 0.0;
            double separatedDiagonal = 0.0;
            double separatedRow = 0.0;
            double separatedBestOmega = -1.0;
            for (const double alphaDeg : {2.0, 10.0, 12.0, 18.0, 25.0})
            {
                const double alphaRad = alphaDeg * Pi / 180.0;
                const std::vector<double> jacobian = jacobianAt(alphaRad);
                const double sym = growthOf(jacobian, 1.0, 1.0);
                const double anti = growthOf(jacobian, -1.0, 1.0);

                // THE QUESTION THE SHIPPED SOLVER CAN ACT ON. Its adaptive
                // under-relaxation already halves the step when the residual
                // grows, down to 0.002, so if damping could remove this
                // amplification the solver would already have removed it. The
                // sweep says whether ANY w does, which is a statement about
                // the spectrum: a real negative eigenvalue can always be
                // damped into the unit disc, a real positive one greater than
                // one never can, whatever w is chosen.
                double best = 1.0e30;
                double bestOmega = 0.0;
                for (int step = 1; step <= 40; ++step)
                {
                    const double omega = 0.025 * static_cast<double>(step);
                    const double growth = growthOf(jacobian, -1.0, omega);
                    if (growth < best) { best = growth; bestOmega = omega; }
                }
                const double selfOnly =
                    growthOf(keepOnly(jacobian, true), -1.0, 1.0);
                const double couplingOnly =
                    growthOf(keepOnly(jacobian, false), -1.0, 1.0);
                const double sign = dominantSign(jacobian, -1.0);
                std::printf("%9.1f%s %12.4f %12.4f %14.3f %12.4f %11.2e "
                            "%11.2e %10s\n", alphaDeg, " deg", sym, anti,
                            bestOmega, best, selfOnly, couplingOnly,
                            sign > 0.0 ? "+" : "-");
                if (alphaDeg == 2.0) attachedAnti = anti;
                if (alphaDeg == 25.0)
                {
                    separatedAnti = anti;
                    separatedSym = sym;
                    separatedBest = best;
                    separatedBestOmega = bestOmega;
                    separatedSign = sign;
                    separatedDiagonal = selfOnly;
                    separatedRow = couplingOnly;
                }
            }
            if (separatedAnti > 1.0)
                std::printf("  At 25 deg an antisymmetric perturbation is "
                            "multiplied by %.3f every pass, so the\n  1e-14 "
                            "seed reaches O(1) in %.0f passes. Best damping "
                            "leaves %.3f at w = %.3f.\n\n",
                            separatedAnti,
                            std::log(1.0e14) / std::log(separatedAnti),
                            separatedBest, separatedBestOmega);
            std::printf("  Dominant antisymmetric eigenvalue is REAL %s at 25 "
                        "deg (Rayleigh %+.3e), so no w\n  damps it. The gain "
                        "is the SELF term: the diagonal alone grows %.2e per "
                        "pass, the\n  coupling between sections alone %.2e - "
                        "and attached it is the other way round.\n\n",
                        separatedSign > 0.0 ? "POSITIVE" : "negative",
                        separatedSign, separatedDiagonal, separatedRow);

            // WHAT THE ATTACHED ROWS SAY, and they are the control: growth per
            // pass under 1 means the round-off seed the block above found stays
            // at round-off forever, which is exactly what an attached solve was
            // measured doing. Nothing has to be done about a seed in a
            // contracting iteration.
            Check(attachedAnti > 0.0 && attachedAnti < 1.0,
                  "ATTACHED, THE ITERATION CONTRACTS AN ANTISYMMETRIC "
                  "PERTURBATION: 0.68 per pass, so the 1e-13 seed an attached "
                  "solve carries is not going anywhere. This is the control "
                  "that makes the separated number below mean something");

            // AND THE SEPARATED ROWS PUT A NUMBER ON "AMPLIFICATION". Thirteen
            // orders was inferred from the outcome; this is the per-pass rate
            // that produces it, and it produces it in three passes out of a
            // flight solve's forty.
            Check(separatedAnti > 1.0e3,
                  "SEPARATED, IT AMPLIFIES ONE BY FOUR ORDERS PER PASS. The "
                  "round-off seed reaches O(1) in three passes, and the flight "
                  "solve takes forty every tick - so the direction is fully "
                  "grown long before the tick that reports it, which is why no "
                  "gate has ever caught it developing");

            // THE PARITY QUESTION, ANSWERED IN THE NEGATIVE, WHICH MATTERS
            // BECAUSE IT CLOSES OFF THE ATTRACTIVE FIX. If the antisymmetric
            // modes grew and the symmetric ones did not, the amplification
            // would be a symmetry defect and something in the meshing or the
            // accumulation order could be made to answer for it. They grow
            // together to under a percent.
            Check(std::fabs(separatedSym - separatedAnti)
                      < 0.05 * separatedAnti,
                  "AND THE AMPLIFICATION IS PARITY-BLIND: the mirror-symmetric "
                  "and mirror-antisymmetric subspaces grow at the same rate to "
                  "under 3%. It is not a symmetry defect that a tidier mesh or "
                  "a symmetrised accumulation could answer for - it is the "
                  "iteration itself, and the antisymmetric half is merely the "
                  "one whose growth shows up as a turn");

            // THE ONE THAT DECIDES WHAT CAN BE BUILT. The shipped solver's
            // only lever on this is its adaptive under-relaxation, which
            // already halves the step whenever the residual grows and floors
            // at 0.002.
            Check(separatedBest > 1.0,
                  "AND NO UNDER-RELAXATION REMOVES IT. Swept over forty values "
                  "of w, the best any of them achieves is still growth - "
                  "because the dominant eigenvalue is real POSITIVE, where "
                  "damping gives 1 + w (lambda - 1) and every w > 0 leaves "
                  "that above 1. The solver's adaptive relaxation is not "
                  "under-tuned; it is the wrong instrument, and item 6 needs "
                  "the operator changed rather than the step size");
            Check(separatedSign > 0.0,
                  "which is a measured sign and not an inference from the "
                  "sweep: the Rayleigh quotient of the converged iterate is "
                  "positive, so the fixed point is a repellor and not an "
                  "oscillation, and a Newton or globally implicit step is the "
                  "class of fix that applies");

            // AND IT MOVES ITEM 6's OWN SENTENCE. It attributes the failure to
            // the coupling BETWEEN sections; the matrix says otherwise.
            Check(separatedDiagonal > separatedRow,
                  "AND THE GAIN IS NOT IN THE COUPLING BETWEEN SECTIONS, WHICH "
                  "IS WHERE ITEM 6's SENTENCE PUTS IT. Run on the diagonal "
                  "alone the operator grows 3.9e4 per pass; run on the "
                  "coupling alone, 3.6e3. It is each section's OWN implicit "
                  "self-solve - gain 1/(1 - dSelf), which runs away as that "
                  "feedback approaches unity - and attached the ranking is "
                  "reversed, with the self term at 1e-6 and the coupling "
                  "carrying the whole contracting iteration at 0.68");
        }

        // -- IS THERE A FIXED POINT AT ALL? ASK NEWTON ---------------------
        //
        // Item 6's sentence, carried since it was written, is "a wing in deep
        // stall has NO STABLE STEADY STATE TO FIND". The block above measured
        // that the fixed point is a REPELLOR - dominant eigenvalue real
        // positive, +3.97e+04 - and a repellor is a fixed point. Those two
        // statements are not the same, and the record has never told them
        // apart, because every instrument pointed at this has been a variant
        // of "does the shipped Picard iteration converge", which a repellor
        // makes fail whether or not a root exists.
        //
        // Newton does not care about the sign of the eigenvalue. It converges
        // to repellors as readily as to attractors, because it inverts the
        // operator instead of iterating it. So running it here is the direct
        // test of item 6's sentence, and it has three possible answers, all
        // of them worth having:
        //
        //   * it converges, and to the SAME root from every start - the
        //     steady state exists and is unique. Item 6's sentence is wrong,
        //     and what is wrong with the solver is the algorithm.
        //   * it converges to DIFFERENT roots from different starts - the
        //     steady state exists but is multi-valued, which is the honest
        //     version of item 6 and means no solver of this class fixes it.
        //   * it does not converge at all - there is no root, and item 6's
        //     sentence stands as written.
        //
        // Solved on the HELD separation branch, which is what makes the
        // question well posed: the polar the solver samples is single valued
        // along a branch, so anything multi-valued found here is the
        // circulation system's, not the polar switching under it.
        {
            std::printf("\n  Is there a fixed point at all? Newton on the "
                        "same operator:\n");
            const CanopyGeometry canopy;
            const SectionPolarTable polars = SectionPolarTable::Analytic();
            const VortexStepMethodSolver wing(canopy, polars, 45);
            const std::size_t count = wing.Sections().size();

            const auto separationAt = [&](double alphaRad)
            {
                double s = 0.0;
                for (int k = 0; k < 200; ++k)
                    s = polars.SeparationEquilibrium(alphaRad, 0.0, s);
                return s;
            };
            const auto worstOf = [](const std::vector<double>& v)
            {
                double worst = 0.0;
                for (const double x : v) worst = std::max(worst, std::fabs(x));
                return worst;
            };

            // Dense LU with partial pivoting. Forty-five sections, solved once
            // per Newton step - this is a test asking whether the root exists,
            // not a proposal for what runs in the flight loop, and the cost of
            // the real thing is measured further down.
            const auto solveLinear = [&](std::vector<double> a,
                                         std::vector<double> b)
            {
                for (std::size_t k = 0; k < count; ++k)
                {
                    std::size_t pivot = k;
                    for (std::size_t i = k + 1; i < count; ++i)
                        if (std::fabs(a[i * count + k])
                            > std::fabs(a[pivot * count + k]))
                            pivot = i;
                    if (pivot != k)
                    {
                        for (std::size_t j = 0; j < count; ++j)
                            std::swap(a[k * count + j], a[pivot * count + j]);
                        std::swap(b[k], b[pivot]);
                    }
                    const double diagonal = a[k * count + k];
                    if (std::fabs(diagonal) < 1.0e-300) return std::vector<double>();
                    for (std::size_t i = k + 1; i < count; ++i)
                    {
                        const double factor = a[i * count + k] / diagonal;
                        if (factor == 0.0) continue;
                        for (std::size_t j = k; j < count; ++j)
                            a[i * count + j] -= factor * a[k * count + j];
                        b[i] -= factor * b[k];
                    }
                }
                std::vector<double> x(count, 0.0);
                for (std::size_t k = count; k-- > 0;)
                {
                    double sum = b[k];
                    for (std::size_t j = k + 1; j < count; ++j)
                        sum -= a[k * count + j] * x[j];
                    x[k] = sum / a[k * count + k];
                }
                return x;
            };

            // One Newton solve of F(Gamma) = G(Gamma) - Gamma = 0, from a
            // given start. Damped: the polar has a knee, and an undamped step
            // that overshoots it lands somewhere the linearisation knew
            // nothing about. Halving until the residual falls is the standard
            // globalisation and adds no tuned constant.
            const auto newtonFrom = [&](double alphaRad,
                                        std::vector<double> gamma,
                                        int& stepsOut, int& passesOut,
                                        double& residualOut)
            {
                VsmSeparationState state;
                state.sectionSeparation.assign(count, separationAt(alphaRad));
                state.initialised = true;
                const VsmSolveInput input = Inflow(alphaRad, 11.0);
                VsmSettings onePass;
                onePass.maxIterations = 1;
                onePass.relaxation = 1.0;

                int passes = 0;
                const auto residualAt = [&](std::vector<double> g)
                {
                    std::vector<double> next = g;
                    wing.SolveFrozen(input, state, next, onePass);
                    ++passes;
                    for (std::size_t i = 0; i < count; ++i)
                        next[i] -= g[i];
                    return next;
                };

                std::vector<double> residual = residualAt(gamma);
                stepsOut = 0;
                for (int step = 0; step < 60; ++step)
                {
                    // A FIXED reference, not the current iterate's own size.
                    // Scaling the difference step by |Gamma| looks natural and
                    // is a trap: a start at zero then differences the operator
                    // over 1e-15, which is round-off, and Newton gets a
                    // Jacobian of noise and reports that no root was found.
                    // Circulations on this wing are O(1-10).
                    const double scale = std::max(1.0, worstOf(gamma));
                    if (worstOf(residual) / scale < 1.0e-10) break;

                    // dF/dGamma = J - I, by central differences on the same
                    // pass the Picard loop uses. The identity is subtracted
                    // inside `residualAt`, so this differences F directly.
                    const double epsilon = 1.0e-6 * scale;
                    std::vector<double> jacobian(count * count, 0.0);
                    for (std::size_t j = 0; j < count; ++j)
                    {
                        std::vector<double> up = gamma;
                        std::vector<double> down = gamma;
                        up[j] += epsilon;
                        down[j] -= epsilon;
                        const std::vector<double> a = residualAt(up);
                        const std::vector<double> b = residualAt(down);
                        for (std::size_t i = 0; i < count; ++i)
                            jacobian[i * count + j] =
                                (a[i] - b[i]) / (2.0 * epsilon);
                    }
                    std::vector<double> negative(count, 0.0);
                    for (std::size_t i = 0; i < count; ++i)
                        negative[i] = -residual[i];
                    const std::vector<double> delta =
                        solveLinear(jacobian, negative);
                    if (delta.empty()) break;

                    double damping = 1.0;
                    bool improved = false;
                    for (int halving = 0; halving < 30; ++halving)
                    {
                        std::vector<double> trial = gamma;
                        for (std::size_t i = 0; i < count; ++i)
                            trial[i] += damping * delta[i];
                        const std::vector<double> trialResidual =
                            residualAt(trial);
                        if (worstOf(trialResidual) < worstOf(residual))
                        {
                            gamma = trial;
                            residual = trialResidual;
                            improved = true;
                            break;
                        }
                        damping *= 0.5;
                    }
                    ++stepsOut;
                    if (!improved) break;
                }
                passesOut = passes;
                residualOut = worstOf(residual) / std::max(1.0, worstOf(gamma));
                return gamma;
            };

            // FOUR STARTS, AND THE POINT IS THAT THEY DISAGREE WITH EACH
            // OTHER. A single Newton run landing somewhere proves only that it
            // landed; whether the root is unique is the question item 6 turns
            // on, and it needs starts that are far apart - including one that
            // is deliberately NOT mirror-symmetric, since a root reached from
            // an asymmetric start is the whole frontal problem in miniature.
            std::printf("%10s %10s %14s %14s %12s %12s %12s\n", "alpha",
                        "start", "Picard resid", "Newton resid", "worst start",
                        "root spread", "root asym");
            double separatedNewton = -1.0;
            double separatedPicard = -1.0;
            double separatedSpread = -1.0;
            double separatedAsymmetry = -1.0;
            double attachedSpread = -1.0;
            int separatedPasses = 0;
            for (const double alphaDeg : {2.0, 12.0, 18.0, 25.0})
            {
                const double alphaRad = alphaDeg * Pi / 180.0;
                VsmSeparationState state;
                state.sectionSeparation.assign(count, separationAt(alphaRad));
                state.initialised = true;
                const VsmSolveInput input = Inflow(alphaRad, 11.0);

                // What the shipped iteration reaches, for the comparison.
                std::vector<double> picard(count, 0.0);
                const VsmSolution shipped =
                    wing.SolveFrozen(input, state, picard, {});

                std::vector<std::vector<double>> roots;
                double worstAsymmetry = 0.0;
                double worstStartResidual = 0.0;
                int steps = 0;
                int passes = 0;
                const char* names[] = {"picard", "zero", "1.3x", "asym"};
                for (int which = 0; which < 4; ++which)
                {
                    std::vector<double> start = picard;
                    if (which == 1) start.assign(count, 0.0);
                    if (which == 2)
                        for (double& x : start) x *= 1.3;
                    if (which == 3)
                        for (std::size_t i = 0; i < count; ++i)
                            start[i] *= 1.0 + (i < count / 2 ? 0.05 : -0.05);

                    double reached = 0.0;
                    const std::vector<double> root =
                        newtonFrom(alphaRad, start, steps, passes, reached);
                    roots.push_back(root);
                    worstStartResidual = std::max(worstStartResidual, reached);

                    double asymmetry = 0.0;
                    const double scale = std::max(1.0e-9, worstOf(root));
                    for (std::size_t i = 0; i < count / 2; ++i)
                        asymmetry = std::max(asymmetry,
                            std::fabs(root[i] - root[count - 1 - i]) / scale);
                    worstAsymmetry = std::max(worstAsymmetry, asymmetry);
                }
                (void)names;

                // How far apart the four roots are, relative to the root's own
                // size. This is the uniqueness measurement.
                double spread = 0.0;
                const double scale = std::max(1.0e-9, worstOf(roots[0]));
                for (std::size_t a = 0; a < roots.size(); ++a)
                    for (std::size_t b = a + 1; b < roots.size(); ++b)
                        for (std::size_t i = 0; i < count; ++i)
                            spread = std::max(spread,
                                std::fabs(roots[a][i] - roots[b][i]) / scale);

                std::vector<double> check = roots[0];
                VsmSettings onePass;
                onePass.maxIterations = 1;
                onePass.relaxation = 1.0;
                wing.SolveFrozen(input, state, check, onePass);
                double newtonResidual = 0.0;
                for (std::size_t i = 0; i < count; ++i)
                    newtonResidual = std::max(newtonResidual,
                        std::fabs(check[i] - roots[0][i]));
                newtonResidual /= std::max(1.0e-9, worstOf(roots[0]));

                std::printf("%9.1f%s %10s %14.2e %14.2e %12.2e %12.2e "
                            "%12.2e\n", alphaDeg, " deg", "all four",
                            shipped.residual, newtonResidual,
                            worstStartResidual, spread, worstAsymmetry);
                (void)steps;
                if (alphaDeg == 2.0) attachedSpread = spread;
                if (alphaDeg == 25.0)
                {
                    separatedNewton = newtonResidual;
                    separatedPicard = shipped.residual;
                    separatedSpread = spread;
                    separatedAsymmetry = worstAsymmetry;
                    separatedPasses = passes;
                }
            }
            std::printf("  'Newton resid' is the SHIPPED residual evaluated at "
                        "Newton's root - the same\n  number the solver reports "
                        "about itself, so the two columns are comparable.\n"
                        "  At 25 deg Newton reaches %.1e where six hundred "
                        "damped Picard passes reach %.1e,\n  and it costs %d "
                        "passes to get there.\n\n",
                        separatedNewton, separatedPicard, separatedPasses);

            // ATTACHED, NEWTON IS EXACT AND THE INSTRUMENT IS SOUND. This is
            // the control the separated rows are read against: same code, same
            // four starts, one root to eleven digits.
            Check(attachedSpread < 1.0e-6,
                  "attached, four Newton runs from starts a third of the "
                  "solution apart land on ONE root to 5e-10, at a residual of "
                  "8e-12 - so the solve, the linear algebra and the line "
                  "search all work, and the separated rows below are a "
                  "measurement rather than a broken instrument");

            // AND SEPARATED IT STALLS - WHICH IS NOT YET AN ANSWER, AND
            // READING IT AS ONE WOULD BE THE MISTAKE THIS FILE KEEPS
            // RETRACTING. Newton converging to a repellor is guaranteed only
            // where F is DIFFERENTIABLE. A line search that runs out of
            // halvings has exactly two causes and they lead opposite ways:
            // there is no root, or the thing being differentiated is not a
            // smooth function. The next block separates them.
            Check(separatedNewton > 1.0e-6,
                  "AND SEPARATED NEWTON STALLS TOO, at 8.5e-01 after 2238 "
                  "passes with a line search - so the fix named last iteration "
                  "does NOT simply work, and 'invert the operator instead of "
                  "iterating it' is not on its own the answer");
            Check(separatedSpread > 1.0e-2,
                  "and the four starts end far apart rather than on one root, "
                  "which is either multi-valuedness or an operator Newton "
                  "cannot differentiate - two very different things that this "
                  "measurement alone cannot tell apart");
            (void)separatedAsymmetry;
            (void)separatedPicard;
        }

        // -- IS G EVEN A FUNCTION? THE SECTION'S OWN IMPLICIT SOLVE --------
        //
        // Newton stalling has two causes and the record is at risk of writing
        // down the wrong one. So: is F differentiable at all in the separated
        // regime?
        //
        // The suspicion has a specific address, and it comes from the previous
        // iteration rather than from nowhere. The gain that amplifies sits on
        // the DIAGONAL - each section's own implicit self-solve - and that
        // solve is a 12-step secant on
        //
        //     r(gamma) = 0.5 c V Cl(alpha(gamma)) - gamma = 0
        //
        // where alpha depends on gamma through the section's own trailing
        // legs. Attached, Cl rises with alpha and this is monotonic: one root,
        // found from anywhere. PAST THE STALL Cl FALLS WITH ALPHA, and a
        // falling Cl against a rising gamma can cross zero more than once.
        //
        // If it does, then G is not a function of the other sections'
        // circulations at all: it returns whichever root the secant happened
        // to walk to from wherever it started, which moves discontinuously
        // when the input moves infinitesimally. No outer algorithm - Picard,
        // Newton, Broyden, continuation - is defined on an operator like that,
        // and the fix would not be an outer-loop fix at all.
        //
        // This reconstructs one section's own residual curve from the parts
        // the solver uses - its own influence coefficient, its own chord, the
        // same polar branch - and counts the crossings.
        {
            std::printf("\n  Is G even a function? One section's own implicit "
                        "residual, root count:\n");
            const CanopyGeometry canopy;
            const SectionPolarTable polars = SectionPolarTable::Analytic();
            const VortexStepMethodSolver wing(canopy, polars, 45);
            const std::vector<VsmSection>& sections = wing.Sections();
            const std::size_t count = sections.size();
            const std::size_t probe = count / 2;

            const auto separationAt = [&](double alphaRad)
            {
                double s = 0.0;
                for (int k = 0; k < 200; ++k)
                    s = polars.SeparationEquilibrium(alphaRad, 0.0, s);
                return s;
            };

            std::printf("%10s %12s %10s %14s %14s\n", "alpha", "separation",
                        "roots", "lowest root", "highest root");
            int attachedRoots = 0;
            int separatedRoots = 0;
            double separatedLow = 0.0;
            double separatedHigh = 0.0;
            for (const double alphaDeg : {2.0, 10.0, 12.0, 18.0, 25.0})
            {
                const double alphaRad = alphaDeg * Pi / 180.0;
                const double separation = separationAt(alphaRad);
                const VsmSolveInput input = Inflow(alphaRad, 11.0);
                VsmSeparationState state;
                state.sectionSeparation.assign(count, separation);
                state.initialised = true;

                // Every other section held at what the wing is actually
                // carrying, so this is the equation the solver really poses
                // rather than an isolated aerofoil.
                std::vector<double> circulation(count, 0.0);
                wing.SolveFrozen(input, state, circulation, {});

                Vec3 external = -input.airspeedBodyMps;
                for (std::size_t j = 0; j < count; ++j)
                    if (j != probe)
                        external += wing.InfluenceAt(probe, j) * circulation[j];

                const VsmSection& section = sections[probe];
                const Vec3 self = wing.InfluenceAt(probe, probe);
                const auto residualAt = [&](double gamma)
                {
                    const Vec3 localFlow = external + self * gamma;
                    const Vec3 inPlane = localFlow - section.spanDirection
                        * Dot(localFlow, section.spanDirection);
                    const double speed = Length(inPlane);
                    if (speed < 1.0e-6) return -gamma;
                    const Vec3 sectionVelocity = -inPlane;
                    const double alpha = std::atan2(
                        -Dot(sectionVelocity, section.normal),
                        Dot(sectionVelocity, section.chordDirection));
                    return 0.5 * section.chordM * speed
                        * polars.SampleAtSeparation(
                              alpha, 0.0, separation, 1.0).liftCoefficient
                        - gamma;
                };

                // Swept over a range that comfortably contains the wing's own
                // loading in both signs, finely enough that two roots a
                // percent apart are still two.
                constexpr double Span = 60.0;
                constexpr int Samples = 240001;
                int roots = 0;
                double lowest = 0.0;
                double highest = 0.0;
                double previous = residualAt(-Span);
                for (int k = 1; k < Samples; ++k)
                {
                    const double gamma = -Span + 2.0 * Span
                        * static_cast<double>(k)
                        / static_cast<double>(Samples - 1);
                    const double value = residualAt(gamma);
                    if ((previous < 0.0) != (value < 0.0))
                    {
                        ++roots;
                        if (roots == 1) lowest = gamma;
                        highest = gamma;
                    }
                    previous = value;
                }
                std::printf("%9.1f%s %12.3f %10d %14.3f %14.3f\n", alphaDeg,
                            " deg", separation, roots, lowest, highest);
                if (alphaDeg == 2.0) attachedRoots = roots;
                if (alphaDeg == 25.0)
                {
                    separatedRoots = roots;
                    separatedLow = lowest;
                    separatedHigh = highest;
                }
            }
            std::printf("\n");

            Check(attachedRoots == 1,
                  "ATTACHED, THE SECTION'S OWN EQUATION HAS EXACTLY ONE ROOT, "
                  "which is why the secant inside the solve is reliable there "
                  "and why nobody has ever had to look at it");
            if (separatedRoots > 1)
                std::printf("  AND SEPARATED IT HAS %d, between %.3f and "
                            "%.3f.\n\n", separatedRoots, separatedLow,
                            separatedHigh);
            Check(separatedRoots > 1,
                  "AND SEPARATED IT HAS THREE, SPREAD 14.2 TO 21.1 - a half of "
                  "the section's own loading apart. The 12-step secant inside "
                  "the solve returns whichever of them it walks to from "
                  "wherever it started, so what the outer loop iterates is not "
                  "a function of its input: it is a branch selection");

            // AND THE CONSEQUENCE IS MEASURABLE ON THE OUTER OPERATOR ITSELF,
            // which is what turns this from an observation about a curve into
            // the reason Newton stalled. A differentiable G has a directional
            // derivative that PLATEAUS as the difference step shrinks - that
            // is what differentiable means numerically. One that jumps between
            // branches has a difference quotient that grows like 1/eps,
            // because the numerator stops shrinking while the denominator
            // does.
            {
                std::printf("  And what that does to the OUTER operator - "
                            "directional derivative vs step size:\n");
                std::printf("%14s %18s %18s\n", "eps", "attached 2 deg",
                            "separated 25 deg");
                const auto slopeAt = [&](double alphaDeg, double epsilon)
                {
                    const double alphaRad = alphaDeg * Pi / 180.0;
                    const VsmSolveInput input = Inflow(alphaRad, 11.0);
                    VsmSeparationState state;
                    state.sectionSeparation.assign(
                        count, separationAt(alphaRad));
                    state.initialised = true;
                    std::vector<double> base(count, 0.0);
                    wing.SolveFrozen(input, state, base, {});

                    VsmSettings onePass;
                    onePass.maxIterations = 1;
                    onePass.relaxation = 1.0;
                    std::vector<double> up = base;
                    std::vector<double> down = base;
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        const double direction =
                            std::sin(2.399963 * static_cast<double>(i + 1));
                        up[i] += epsilon * direction;
                        down[i] -= epsilon * direction;
                    }
                    wing.SolveFrozen(input, state, up, onePass);
                    wing.SolveFrozen(input, state, down, onePass);
                    double worst = 0.0;
                    for (std::size_t i = 0; i < count; ++i)
                        worst = std::max(worst, std::fabs(up[i] - down[i]));
                    return worst / (2.0 * epsilon);
                };
                double attachedSpreadOverDecades = 0.0;
                double attachedSmallest = 0.0;
                double separatedSmallest = 0.0;
                double separatedLargest = 0.0;
                bool firstDecade = true;
                double attachedFirst = 0.0;
                for (const double epsilon :
                     {1.0e-2, 1.0e-3, 1.0e-4, 1.0e-5, 1.0e-6, 1.0e-7})
                {
                    const double attached = slopeAt(2.0, epsilon);
                    const double separated = slopeAt(25.0, epsilon);
                    std::printf("%14.0e %18.4f %18.4e\n", epsilon, attached,
                                separated);
                    if (firstDecade) { attachedFirst = attached; firstDecade = false; }
                    attachedSpreadOverDecades = std::max(
                        attachedSpreadOverDecades,
                        std::fabs(attached - attachedFirst));
                    attachedSmallest = attached;
                    separatedSmallest = separated;
                    separatedLargest = std::max(separatedLargest, separated);
                }
                std::printf("\n");

                Check(attachedSpreadOverDecades < 0.01 * attachedSmallest,
                      "ATTACHED THE DERIVATIVE PLATEAUS across five decades of "
                      "step size to under a percent, which is what a "
                      "differentiable operator looks like and is the control "
                      "for the column beside it");
                // AND THE ANSWER IS NOT THE ONE THIS BLOCK WAS BUILT
                // EXPECTING, WHICH IS WHY IT IS WORTH THE SPACE. A jump would
                // have shown a quotient growing without limit as the step
                // shrank. It does grow - 1.6e3 at 1e-2, 1.3e5 at 1e-5 - and
                // then it PLATEAUS at 1.215e5 for the last two decades. So G
                // is differentiable at the wing's own operating point after
                // all, and its derivative there is 1.2e5 against the attached
                // 0.21: six orders, on the same operator, same wing.
                //
                // Which is the real finding. The operator is not
                // discontinuous AT the operating point - it is differentiable
                // with a fold sitting immediately beside it, and the
                // three-root curve above is where that fold comes from. A
                // Newton step sized by a derivative of 1.2e5 is valid over a
                // neighbourhood far smaller than the step it proposes, so the
                // line search halves back to nearly nothing and the iterate
                // crawls. That is what the 8.5e-01 stall is.
                Check(separatedSmallest > 1.0e4 * attachedSmallest,
                      "SEPARATED, THE SAME DERIVATIVE IS SIX ORDERS LARGER - "
                      "1.2e5 against 0.21 - and it PLATEAUS, so G is "
                      "differentiable there and the operator is not the "
                      "discontinuity this block went looking for. What it is "
                      "is a fold sitting immediately beside the wing's own "
                      "operating point: the derivative is finite, enormous, "
                      "and valid over a neighbourhood far smaller than any "
                      "Newton step sized by it, which is exactly how a damped "
                      "Newton crawls to a stall instead of diverging");
                Check(separatedLargest > 1.0e3,
                      "and it is stiff at every step size swept, so this is "
                      "not an artefact of one difference step being too fine "
                      "or too coarse");
            }
        }

        Check(separationAtSignChange > separationBelowIt + 0.1,
              "THE SOLVER ALREADY CARRIES IT: the separation state moves "
              "sharply across the same two degrees the lift slope changes "
              "sign over, so the criterion item 6 establishes is readable "
              "from a state that is already updated every tick - no new "
              "per-section field is needed to declare the separated regime");
    }

    // -- section polars ---------------------------------------------------
    {
        const SectionPolarTable polar = SectionPolarTable::Analytic();
        std::printf("Section polar (analytic)\n");
        std::printf("  slope %.3f /rad, zero-lift %+.2f deg, "
                    "stall %+.2f deg\n",
                    polar.LiftCurveSlopePerRad(0.0),
                    polar.ZeroLiftAngleRad(0.0) * 180.0 / Pi,
                    polar.StallAngleRad(0.0) * 180.0 / Pi);

        // Thin-airfoil theory with the usual thickness correction.
        const AnalyticPolarSpec spec = polar.Spec();
        CheckWithin(polar.LiftCurveSlopePerRad(0.0),
                    2.0 * Pi * (1.0 + 0.77 * spec.thicknessFraction), 1e-9,
                    "lift slope is thin-airfoil with thickness correction");
        // Circular-arc camber line: alpha_L0 = -2 h/c exactly.
        CheckWithin(polar.ZeroLiftAngleRad(0.0),
                    -2.0 * spec.camberFraction, 1e-9,
                    "zero-lift angle is that of a circular-arc camber line");
        Check(polar.ZeroLiftAngleRad(0.0) < 0.0,
              "a cambered section lifts at zero incidence");

        // Flap effectiveness is derived, so it can be checked against the
        // closed form rather than against a previous run.
        const double tau = ThinAirfoilFlapEffectiveness(0.78);
        // x/c = (1 - cos t)/2 on the camber line, so the hinge is at
        // t = acos(1 - 2 x/c).
        const double thetaF = std::acos(1.0 - 2.0 * 0.78);
        CheckWithin(tau, 1.0 - (thetaF - std::sin(thetaF)) / Pi, 1e-12,
                    "flap effectiveness matches thin-airfoil theory");
        Check(tau > 0.45 && tau < 0.70,
              "a 22% flap is a little over half as effective as pitching the "
              "whole section - the textbook figure, and a check that the "
              "camber-line mapping is the right way round");

        // Brake is camber, so it moves the zero-lift angle and not the slope.
        Check(polar.ZeroLiftAngleRad(1.0) < polar.ZeroLiftAngleRad(0.0),
              "brake shifts the zero-lift angle negative - it is a camber "
              "change, not an incidence change");
        CheckWithin(polar.LiftCurveSlopePerRad(1.0),
                    polar.LiftCurveSlopePerRad(0.0), 1e-9,
                    "and leaves the lift slope alone");
        Check(polar.StallAngleRad(1.0) - polar.ZeroLiftAngleRad(1.0)
                  < polar.StallAngleRad(0.0) - polar.ZeroLiftAngleRad(0.0),
              "a braked section stalls at a smaller margin above zero lift");

        // Sampled behaviour: lift rises to a peak and falls away past stall.
        const double clAtStall =
            polar.Sample(polar.StallAngleRad(0.0), 0.0).liftCoefficient;
        const double clPastStall =
            polar.Sample(polar.StallAngleRad(0.0) + 0.35, 0.0)
                .liftCoefficient;
        const double clDeep = polar.Sample(0.5 * Pi, 0.0).liftCoefficient;
        std::printf("  Cl at stall %.3f, +20 deg past %.3f, at 90 deg %.3f\n",
                    clAtStall, clPastStall, clDeep);
        Check(clAtStall > 1.0 && clAtStall < 1.9,
              "peak section Cl is in the range a thick cambered section makes");
        Check(clPastStall < clAtStall, "lift falls away past the stall");
        Check(std::fabs(clDeep) < 0.15,
              "a section broadside to the flow makes no lift");
        Check(polar.Sample(0.5 * Pi, 0.0).dragCoefficient > 1.0,
              "and a great deal of drag - Viterna's flat plate");

        // Drag is least near the zero-lift angle and rises both ways.
        const double cdAtZeroLift =
            polar.Sample(polar.ZeroLiftAngleRad(0.0), 0.0).dragCoefficient;
        Check(cdAtZeroLift <= polar.Sample(0.10, 0.0).dragCoefficient,
              "profile drag is least where the section makes no lift");
        Check(polar.Sample(0.05, 1.0).dragCoefficient
                  > polar.Sample(0.05, 0.0).dragCoefficient,
              "brake adds drag");
        Check(polar.Sample(0.05, 1.0).momentCoefficient
                  < polar.Sample(0.05, 0.0).momentCoefficient,
              "and pitches the section nose-down");
    }

    // -- the computed section polars, against sections with published data -
    {
        // The panel-plus-boundary-layer solve is only worth what it can
        // reproduce on a section somebody has measured. NACA 2412 is the
        // check: it is thin enough that thin-airfoil theory nearly works, so
        // agreement on the linear part proves little - but its zero-lift
        // angle, its quarter-chord moment, its minimum drag and its maximum
        // lift are all published, and nothing in this solver was built while
        // looking at them.
        SectionProfileSpec naca2412;
        naca2412.maxThicknessFraction = 0.12;
        naca2412.maxThicknessPosition = 0.30;
        naca2412.maxCamberFraction = 0.02;
        naca2412.maxCamberPosition = 0.40;
        // A wind-tunnel model has no cell opening in its nose.
        naca2412.inletChordFraction = 0.0;
        const SectionViscousSolver tunnel(
            BuildSectionProfile(naca2412, 0.0), 3.0e6);
        Check(tunnel.Valid(), "the panel system factorises");

        double zeroLiftDeg = 0.0;
        double slopePerDeg = 0.0;
        {
            const double low = tunnel.Solve(-2.0 / 57.29577951308232)
                .liftCoefficient;
            const double high = tunnel.Solve(2.0 / 57.29577951308232)
                .liftCoefficient;
            slopePerDeg = (high - low) / 4.0;
            zeroLiftDeg = -(0.5 * (low + high)) / slopePerDeg;
        }
        const SectionAerodynamics atZero = tunnel.Solve(0.0);
        double maximumLift = 0.0;
        double maximumLiftDeg = 0.0;
        {
            double lower = 1.0;
            double upper = 1.0;
            for (int degrees = 0; degrees <= 20; ++degrees)
            {
                const SectionAerodynamics flow = tunnel.Solve(
                    degrees / 57.29577951308232, lower, upper);
                lower = flow.lowerAttachedFraction;
                upper = flow.upperAttachedFraction;
                // The branch ends where the solve falls into the fully
                // separated state; the peak is the largest lift before that.
                if (flow.separatedChordFraction > 0.6) break;
                if (flow.liftCoefficient < maximumLift) continue;
                maximumLift = flow.liftCoefficient;
                maximumLiftDeg = degrees;
            }
        }
        std::printf("NACA 2412 solved: zero lift %+.2f deg (published -2.1), "
                    "slope %.4f /deg (0.11), Cm %+.3f (-0.05)\n",
                    zeroLiftDeg, slopePerDeg, atZero.momentCoefficient);
        std::printf("  Cd at zero incidence %.4f (0.006), CLmax %.2f at "
                    "%.0f deg (1.6-1.7 at 16)\n",
                    atZero.dragCoefficient, maximumLift, maximumLiftDeg);
        CheckWithin(zeroLiftDeg, -2.1, 5.0,
                    "NACA 2412's zero-lift angle, from its own coordinates");
        CheckWithin(slopePerDeg, 0.11, 12.0,
                    "and its lift-curve slope");
        CheckWithin(atZero.momentCoefficient, -0.05, 15.0,
                    "and its quarter-chord pitching moment - which the "
                    "analytic table could only produce by stating it");
        CheckWithin(atZero.dragCoefficient, 0.006, 12.0,
                    "and its minimum profile drag, out of Thwaites, Michel, "
                    "Head and Squire-Young rather than out of a constant");
        // 1.96 measured against a published 1.6-1.7, at 16 degrees against a
        // published 16. The angle lands and the value is 18% high, and the
        // direction of that error is the method's: with no inverse-mode
        // boundary layer the branch runs until it ends rather than being
        // solved through the separation, so the last degree before the end
        // carries more lift than it should. It is bounded here rather than
        // corrected, because correcting it would mean a coefficient chosen to
        // land on a published number.
        Check(maximumLift > 1.4 && maximumLift < 2.1,
              "and a maximum lift coefficient near the published range, high "
              "by about a fifth - the one number thin-airfoil theory cannot "
              "produce at all, because it has no nose radius to run out of");
        Check(maximumLiftDeg > 11.0 && maximumLiftDeg < 19.0,
              "at an incidence in the published range");

        // The wing's own section, and the thing this was all for.
        const SectionPolarTable& computed = SectionPolarTable::Default();
        Check(computed.Provenance() == PolarProvenance::Computed,
              "the wing flies on polars solved from its own section");
        std::printf("EPIC 2 section: CLmax %.2f hands up, %.2f at 25%% brake, "
                    "%.2f at 40%%, %.2f at full\n",
                    computed.MaximumLiftCoefficient(0.0),
                    computed.MaximumLiftCoefficient(0.25),
                    computed.MaximumLiftCoefficient(0.40),
                    computed.MaximumLiftCoefficient(1.0));
        Check(computed.MaximumLiftCoefficient(0.40)
                  > computed.MaximumLiftCoefficient(0.0) + 0.3,
              "BRAKE RAISES MAXIMUM LIFT. This is the whole point: the "
              "analytic table's maximum lift was the slope times a stated "
              "stall margin, so it could not change with brake at all, and "
              "40% brake walked the wing off the top of a curve that never "
              "rose");
        // KNOWN LIMITATION, and it is worth printing rather than asserting
        // around. The incidence at which the solved lift peaks is not smooth
        // across the brake axis: 10, 11, 7, 12, 3 and 13 degrees at 0, 10,
        // 25, 40, 60 and 100% brake. Maximum lift itself is far better
        // behaved and rises with brake, which is the claim the wing depends
        // on - but the angle it happens at is decided by whether the
        // Kirchhoff fixed point survives one more degree before falling into
        // the fully separated state, and near the peak that is a close-run
        // thing. Every brake setting's peak is a real branch end at this
        // Reynolds number; which one a neighbouring setting reaches is not
        // something this method resolves.
        //
        // What it costs: the stall angle a pilot would feel is not
        // repeatable degree by degree across brake travel. What would fix it
        // is the boundary layer solved in inverse mode past separation rather
        // than the branch simply ending, which is the difference between this
        // and XFOIL. PHYSICS_TODO item 12.
        std::printf("  KNOWN LIMITATION: stall angle %.0f deg hands up, "
                    "%.0f at 25%%, %.0f at 40%%, %.0f at full - not smooth "
                    "across the brake axis\n",
                    computed.StallAngleRad(0.0) * 180.0 / Pi,
                    computed.StallAngleRad(0.25) * 180.0 / Pi,
                    computed.StallAngleRad(0.40) * 180.0 / Pi,
                    computed.StallAngleRad(1.0) * 180.0 / Pi);
        Check(computed.ZeroLiftAngleRad(0.0) < 0.0
                  && computed.ZeroLiftAngleRad(0.0) > -0.12,
              "the wing's own section lifts at zero incidence, by about the "
              "amount its camber says");

        // The moment is no longer a constant, which is what gives the wing an
        // aerodynamic centre that moves and the pitch loop something to work
        // with.
        const double momentLow = computed.Sample(0.0, 0.0).momentCoefficient;
        const double momentHigh =
            computed.Sample(computed.StallAngleRad(0.0), 0.0)
                .momentCoefficient;
        std::printf("  section Cm %.3f at zero incidence, %.3f at the stall\n",
                    momentLow, momentHigh);
        Check(std::fabs(momentHigh - momentLow) > 0.005,
              "the section's pitching moment varies with incidence - it was a "
              "constant, so the section had no aerodynamic-centre movement "
              "and the pitch loop gain had nothing in it");

        // Brake's own pitching moment, against thin-airfoil flap theory. The
        // analytic table had this at roughly half, because it multiplied the
        // moment by the flap effectiveness a second time - the effectiveness
        // belongs to the LIFT increment, not to the moment.
        const double momentPerRadian =
            (computed.Sample(0.0, 1.0).momentCoefficient - momentLow)
                / computed.ComputedSpec().section.fullBrakeDeflectionRad;
        std::printf("  brake pitching moment %.2f per radian of deflection "
                    "(thin-airfoil flap theory: about -0.55)\n",
                    momentPerRadian);
        Check(momentPerRadian < -0.4 && momentPerRadian > -0.8,
              "brake's pitching moment matches thin-airfoil flap theory for a "
              "22% flap");
    }

    // -- Biot-Savart and the influence matrix -----------------------------
    {
        // An elliptical wing with thin flat-plate sections must reproduce
        // lifting-line theory. This is the load-bearing test in this file.
        // -- does the section loop itself break mirror symmetry? -----------
        //
        // §65 eliminated the suspension graph as a seed for the collapse
        // asymmetry that blocks §64's line-drag correction, and named the
        // untested downstream candidates: the VSM's section ordering, the
        // collapse solver, the pressure model. This is the first of them, and
        // it is the most likely - the circulation solve is iterative over
        // sections, and a loop that reads partially updated neighbours breaks
        // symmetry SYSTEMATICALLY rather than by round-off.
        //
        // The two signatures are far apart and that is what makes this cheap
        // and decisive. Round-off gives a mirror residual near 1e-16 relative;
        // a Gauss-Seidel-style sweep over a symmetric wing at symmetric
        // incidence gives something many orders larger, and biased toward the
        // end the sweep finishes at.
        {
            const VortexStepMethodSolver mirror =
                VortexStepMethodSolver::FlatWing(
                    12.0, 1.5, true, ThinFlatPlatePolar(), 80);
            const VsmSolution symmetric =
                mirror.Solve(Inflow(4.0 * Pi / 180.0));
            const std::size_t count = symmetric.sections.size();
            double worstRelative = 0.0;
            double largest = 0.0;
            for (const VsmSectionResult& section : symmetric.sections)
                largest = std::max(largest, std::fabs(section.circulation));
            for (std::size_t i = 0; i < count / 2; ++i)
            {
                const double a = symmetric.sections[i].circulation;
                const double b = symmetric.sections[count - 1 - i].circulation;
                if (largest > 0.0)
                    worstRelative = std::max(worstRelative,
                                             std::fabs(a - b) / largest);
            }
            std::printf("VSM mirror symmetry: worst relative left-right "
                        "circulation difference %.3e\n", worstRelative);
            // A bound, not a fit. Anything above 1e-9 on a symmetric wing at
            // symmetric incidence is a systematic asymmetry in the solve, not
            // arithmetic - and would be the seed §65 went looking for.
            Check(worstRelative < 1.0e-9,
                  "the VSM's section loop is mirror-symmetric, so it does not "
                  "seed the collapse asymmetry");
        }

        const double span = 12.0;
        const double rootChord = 1.5;
        const VortexStepMethodSolver ellipse =
            VortexStepMethodSolver::FlatWing(
                span, rootChord, true, ThinFlatPlatePolar(), 80);

        // An ellipse of semi-axes b/2 and c0/2 has area pi b c0 / 4.
        const double analyticArea = Pi * span * rootChord / 4.0;
        CheckWithin(ellipse.ReferenceAreaM2(), analyticArea, 0.5,
                    "elliptical planform area");
        const double aspectRatio = ellipse.AspectRatio();
        std::printf("Elliptical wing: span %.1f m, area %.3f m2, AR %.3f\n",
                    span, ellipse.ReferenceAreaM2(), aspectRatio);

        // Lifting-line theory, exactly.
        const double expectedSlope =
            2.0 * Pi * aspectRatio / (aspectRatio + 2.0);

        const double alphaA = 2.0 * Pi / 180.0;
        const double alphaB = 6.0 * Pi / 180.0;
        const VsmSolution solutionA = ellipse.Solve(Inflow(alphaA));
        const VsmSolution solutionB = ellipse.Solve(Inflow(alphaB));
        Check(solutionA.converged && solutionB.converged,
              "the circulation solve converges");

        const double measuredSlope =
            (solutionB.liftCoefficient - solutionA.liftCoefficient)
                / (alphaB - alphaA);
        std::printf("  CL_alpha  %.4f /rad measured, %.4f expected "
                    "(2 pi AR / (AR + 2))\n",
                    measuredSlope, expectedSlope);
        CheckWithin(measuredSlope, expectedSlope, 4.0,
                    "lift-curve slope matches lifting-line theory");

        // Minimum induced drag: CDi = CL^2 / (pi AR) for elliptical loading.
        const double expectedInduced = solutionB.liftCoefficient
            * solutionB.liftCoefficient / (Pi * aspectRatio);
        std::printf("  CL %.4f, CDi %.5f measured, %.5f expected "
                    "(CL^2 / pi AR)\n",
                    solutionB.liftCoefficient,
                    solutionB.inducedDragCoefficient, expectedInduced);
        CheckWithin(solutionB.inducedDragCoefficient, expectedInduced, 8.0,
                    "induced drag matches elliptical-loading theory");

        // Elliptical loading means constant downwash across the span. That is
        // the property that produces the minimum, so it is worth checking
        // directly rather than only through the drag.
        double minDownwash = 1.0e9;
        double maxDownwash = -1.0e9;
        for (std::size_t i = 0; i < solutionB.sections.size(); ++i)
        {
            // Ignore the last panel each side: the tip of an ellipse has zero
            // chord and the discretisation cannot resolve it.
            // The outer panels of an ellipse have vanishing chord and sit
            // inside the filament core, so neither the physics nor the
            // discretisation resolves them.
            if (i < 8 || i + 8 >= solutionB.sections.size()) continue;
            const double downwash = solutionB.sections[i].inducedAngleRad;
            minDownwash = std::min(minDownwash, downwash);
            maxDownwash = std::max(maxDownwash, downwash);
        }
        std::printf("  downwash across the span: %.4f to %.4f rad\n",
                    minDownwash, maxDownwash);
        Check(std::fabs(maxDownwash - minDownwash) < 0.12 *
                  std::fabs(0.5 * (maxDownwash + minDownwash)),
              "elliptical loading gives near-constant downwash");

        // Zero incidence on a flat, uncambered wing makes no lift at all.
        const VsmSolution zero = ellipse.Solve(Inflow(0.0));
        Check(std::fabs(zero.liftCoefficient) < 1e-6,
              "a flat plate at zero incidence makes no lift");
        Check(std::fabs(zero.inducedDragCoefficient) < 1e-6,
              "and no induced drag");

        // Mirror symmetry, which no coefficient in the solver enforces.
        const std::size_t sectionCount = solutionB.sections.size();
        for (std::size_t i = 0; i < sectionCount / 2; ++i)
        {
            const double left = solutionB.sections[i].circulation;
            const double right =
                solutionB.sections[sectionCount - 1 - i].circulation;
            Check(std::fabs(left - right) < 1e-9 * std::max(1.0, std::fabs(left)),
                  "spanwise circulation is symmetric");
        }
    }

    // -- rectangular wing -------------------------------------------------
    {
        // A rectangular wing must come out below the ellipse: its loading is
        // not optimal, so it makes less lift for the same incidence and more
        // induced drag for the same lift. The span efficiency of a
        // rectangular wing of this aspect ratio is about 0.9 in the
        // literature, which is the range to land in.
        const VortexStepMethodSolver rectangle =
            VortexStepMethodSolver::FlatWing(
                12.0, 1.5, false, ThinFlatPlatePolar(), 80);
        const double aspectRatio = rectangle.AspectRatio();
        const VsmSolution solution =
            rectangle.Solve(Inflow(6.0 * Pi / 180.0));
        const double idealInduced = solution.liftCoefficient
            * solution.liftCoefficient / (Pi * aspectRatio);
        const double spanEfficiency =
            idealInduced / std::max(1e-9, solution.inducedDragCoefficient);
        std::printf("Rectangular wing: AR %.2f, CL %.4f, span efficiency "
                    "%.3f\n", aspectRatio, solution.liftCoefficient,
                    spanEfficiency);
        Check(spanEfficiency > 0.80 && spanEfficiency < 1.0,
              "a rectangular wing is less efficient than an ellipse, by about "
              "the margin the literature reports");
    }

    // -- panel count ------------------------------------------------------
    {
        // The answer must be a property of the wing, not of the mesh.
        const double alpha = 5.0 * Pi / 180.0;
        const VsmSolution coarse = VortexStepMethodSolver::FlatWing(
            12.0, 1.5, true, ThinFlatPlatePolar(), 20).Solve(Inflow(alpha));
        const VsmSolution fine = VortexStepMethodSolver::FlatWing(
            12.0, 1.5, true, ThinFlatPlatePolar(), 120).Solve(Inflow(alpha));
        std::printf("Panel convergence: CL %.5f at 20 panels, %.5f at 120\n",
                    coarse.liftCoefficient, fine.liftCoefficient);
        CheckWithin(coarse.liftCoefficient, fine.liftCoefficient, 2.0,
                    "CL is converged in panel count");
    }

    // -- the EPIC 2 canopy ------------------------------------------------
    {
        // The real geometry, with the arc. No claim is made about the numbers
        // being right for this wing - the polars are analytic - only that the
        // solver runs on the manufactured geometry and behaves sensibly.
        const CanopyGeometry canopy;
        const VortexStepMethodSolver wing(
            canopy, SectionPolarTable::Analytic(), 45);
        std::printf("EPIC 2 ML: area %.2f m2, span %.2f m, AR %.2f\n",
                    wing.ReferenceAreaM2(), wing.ReferenceSpanM(),
                    wing.AspectRatio());
        // The solver sees the projected wing, which is what flies.
        CheckWithin(wing.ReferenceSpanM(), canopy.ProjectedSpanM(), 1.0,
                    "the solver spans the projected wing");

        const VsmSolution trim = wing.Solve(Inflow(0.06, 11.0));
        std::printf("  at 6 deg / 11 m/s: CL %.3f, CDi %.4f, CD %.4f, "
                    "L/D %.2f, %d iterations\n",
                    trim.liftCoefficient, trim.inducedDragCoefficient,
                    trim.totalDragCoefficient,
                    trim.liftCoefficient / trim.totalDragCoefficient,
                    trim.iterations);
        Check(trim.converged, "the canopy solve converges");
        Check(trim.liftCoefficient > 0.4 && trim.liftCoefficient < 1.2,
              "CL is in the range this wing works in");
        Check(trim.inducedDragCoefficient > 0.0,
              "a lifting wing has induced drag");

        // Roll damping must emerge. A wing rolling right sees more incidence
        // on the falling tip and less on the rising one, and the resulting
        // moment must oppose the roll. There is no damping coefficient in the
        // solver - this is the integral of the section forces.
        VsmSolveInput rolling = Inflow(0.06, 11.0);
        rolling.angularVelocityBodyRadps = {0.5, 0.0, 0.0};
        const VsmSolution rolled = wing.Solve(rolling);
        std::printf("  roll rate +0.5 rad/s -> roll moment %+.1f Nm\n",
                    rolled.momentBodyNm.x);
        Check(rolled.converged, "the rolling case converges");
        Check(rolled.momentBodyNm.x < 0.0,
              "roll damping emerges from the spanwise incidence change");

        // Symmetric input must be exactly symmetric. Not nearly - the wing is
        // mirror-symmetric and so is the input, so any roll or yaw at all is
        // the solver inventing one.
        VsmSolveInput symmetric = Inflow(0.06, 11.0);
        symmetric.leftBrake = 0.5;
        symmetric.rightBrake = 0.5;
        const VsmSolution both = wing.Solve(symmetric);
        std::printf("  symmetric brake 0.5: roll %+.3f Nm, yaw %+.3f Nm, "
                    "%d iterations\n",
                    both.momentBodyNm.x, both.momentBodyNm.z, both.iterations);
        Check(both.converged, "the symmetric braked case converges");
        Check(std::fabs(both.momentBodyNm.x) < 1.0e-6,
              "symmetric brake rolls nothing");
        Check(std::fabs(both.momentBodyNm.z) < 1.0e-6,
              "and yaws nothing");
        Check(both.liftCoefficient > trim.liftCoefficient,
              "symmetric brake adds camber, so it adds lift before it stalls");

        // Asymmetric brake. The braked half carries more camber and more drag,
        // so it makes more lift and more drag than the other - both come out
        // of the section forces, neither is a term.
        VsmSolveInput braked = Inflow(0.06, 11.0);
        braked.rightBrake = 0.6;
        const VsmSolution asymmetric = wing.Solve(braked);
        std::printf("  right brake 0.6: roll %+.1f Nm, yaw %+.1f Nm\n",
                    asymmetric.momentBodyNm.x, asymmetric.momentBodyNm.z);
        Check(asymmetric.converged, "the asymmetric braked case converges");
        Check(asymmetric.momentBodyNm.z > 1.0,
              "brake yaws the nose toward the braked side, from that side's "
              "drag");
        Check(asymmetric.momentBodyNm.x > 1.0,
              "and rolls the braked tip up, because below stall more camber "
              "is more lift. The turn a pilot gets is that yaw plus the "
              "payload reacting through the lines, which is Level 7's to "
              "couple");

        // Mirrored input must mirror exactly.
        VsmSolveInput mirrored = Inflow(0.06, 11.0);
        mirrored.leftBrake = 0.6;
        const VsmSolution mirroredSolution = wing.Solve(mirrored);
        Check(std::fabs(mirroredSolution.momentBodyNm.x
                        + asymmetric.momentBodyNm.x) < 1.0e-6,
              "left and right brake mirror exactly in roll");
        Check(std::fabs(mirroredSolution.momentBodyNm.z
                        + asymmetric.momentBodyNm.z) < 1.0e-6,
              "and in yaw");

        // THE GEOMETRIC CHANNEL. `sectionIncidenceOffsetRad` is the path the
        // suspension is meant to reach the canopy through: an asymmetric line
        // load twists the wing, the twisted wing rolls, and the roll is an
        // outcome rather than a command. This gates the aerodynamic half of
        // it - what a given twist is worth - which is the half that can be
        // measured before anything is able to supply the twist.
        //
        // Empty means the design pose, so every caller above is unchanged.
        VsmSolveInput untwisted = Inflow(0.06, 11.0);
        untwisted.sectionIncidenceOffsetRad.assign(wing.Sections().size(), 0.0);
        const VsmSolution zeroTwist = wing.Solve(untwisted);
        Check(std::fabs(zeroTwist.liftCoefficient - trim.liftCoefficient)
                  < 1.0e-12,
              "a zero offset is the design pose, bit for bit - the channel is "
              "additive and costs nothing to callers that do not use it");

        // Antisymmetric twist, linear in span. Positive twists the RIGHT tip
        // nose-up, which must roll the right tip up.
        const auto twisted = [&](double twistRad)
        {
            VsmSolveInput in = Inflow(0.06, 11.0);
            in.sectionIncidenceOffsetRad.resize(wing.Sections().size());
            for (std::size_t i = 0; i < wing.Sections().size(); ++i)
                in.sectionIncidenceOffsetRad[i] =
                    twistRad * wing.Sections()[i].spanFraction;
            return wing.Solve(in);
        };
        constexpr double Degree = 3.14159265358979 / 180.0;
        const VsmSolution smallTwist = twisted(0.25 * Degree);
        const VsmSolution largeTwist = twisted(4.0 * Degree);
        const double smallGain =
            smallTwist.momentBodyNm.x / (0.25 * Degree);
        const double largeGain = largeTwist.momentBodyNm.x / (4.0 * Degree);
        std::printf("  antisymmetric twist: 0.25 deg -> %+.1f Nm, "
                    "4 deg -> %+.1f Nm, gain %.0f vs %.0f Nm/rad\n",
                    smallTwist.momentBodyNm.x, largeTwist.momentBodyNm.x,
                    smallGain, largeGain);
        Check(smallTwist.converged && largeTwist.converged,
              "the twisted cases converge");
        Check(smallTwist.momentBodyNm.x > 0.0,
              "twisting the right tip nose-up rolls the right tip up - the "
              "channel carries the right sign");
        // Sixteen times the input over sixteen times the range, and the gain
        // moves by well under a percent. The channel is a clean linear
        // aerodynamic gain, which is what makes the twist a REQUIREMENT that
        // can be stated rather than a number that has to be swept.
        Check(std::fabs(largeGain - smallGain) < 0.01 * smallGain,
              "and it is linear across sixteen times the range, so the twist "
              "a given roll moment needs is one division");
        Check(std::fabs(largeTwist.forceBodyN.z - trim.forceBodyN.z)
                  < 0.01 * std::fabs(trim.forceBodyN.z),
              "twist is a couple: four degrees of it changes total lift by "
              "under a percent, so it rolls the wing without re-trimming it");

        // Mirror, to round-off. Same gate the brake cases above get, because
        // a new spanwise input is a new way to break the wing's symmetry.
        const VsmSolution mirroredTwist = twisted(-1.0 * Degree);
        const VsmSolution rightTwist = twisted(1.0 * Degree);
        Check(std::fabs(mirroredTwist.momentBodyNm.x
                        + rightTwist.momentBodyNm.x) < 1.0e-9,
              "left and right twist mirror exactly in roll");
        Check(std::fabs(mirroredTwist.forceBodyN.z
                        - rightTwist.forceBodyN.z) < 1.0e-9,
              "and carry identical lift");

        // What the twist is worth against the control that already works.
        // Full one-side brake makes about 1500 N.m of roll on this wing, so
        // matching it takes something like ten degrees of antisymmetric
        // twist - and brake itself is several times slower than a real wing
        // (PHYSICS_TODO item 0b). Recorded rather than gated tightly: it is
        // the number that says whether a canopy can twist far enough to be
        // the mechanism, and that question is open.
        VsmSolveInput fullBrake = Inflow(0.06, 11.0);
        fullBrake.rightBrake = 1.0;
        const VsmSolution braked1 = wing.Solve(fullBrake);
        std::printf("  full right brake rolls %+.1f Nm = %.1f deg of "
                    "antisymmetric twist\n",
                    braked1.momentBodyNm.x,
                    braked1.momentBodyNm.x / smallGain / Degree);
        Check(braked1.momentBodyNm.x / smallGain / Degree > 3.0,
              "matching today's full-brake roll takes SEVERAL DEGREES of "
              "twist, not a fraction of one. The channel is linear and its "
              "gain is known, so what is unresolved is whether a canopy on "
              "its lines twists that far - and that is a structural question "
              "about the canopy, not an aerodynamic one");

        // Installed drag. A canopy polar on its own glides far better than
        // the wing it belongs to, because on a paraglider the lines, risers,
        // harness and pilot are a large fraction of the total. The published
        // best glide for this wing is 9.5, and that is a number this can be
        // held against - the only published performance figure Level 4 can
        // check itself on before real polars arrive.
        const InstalledDragSpec installedSpec;
        const InstalledDragResult installed = EvaluateInstalledDrag(
            installedSpec, Inflow(0.06, 11.0).airspeedBodyMps, 1.225);
        const double referenceForce = 0.5 * 1.225 * 11.0 * 11.0
            * wing.ReferenceAreaM2();
        const double installedCd = installed.totalDragN / referenceForce;
        const double totalCd = trim.totalDragCoefficient + installedCd;
        std::printf("  installed drag: lines %.1f N, harness %.1f N, "
                    "CD +%.4f\n",
                    installed.lineDragN, installed.harnessDragN, installedCd);
        std::printf("  canopy alone L/D %.2f, whole aircraft L/D %.2f "
                    "(published best glide 9.5)\n",
                    trim.liftCoefficient / trim.totalDragCoefficient,
                    trim.liftCoefficient / totalCd);
        Check(installed.harnessDragN > installed.lineDragN,
              "the pilot is a bigger hole in the air than the lines");
        Check(installedCd > 0.4 * trim.totalDragCoefficient,
              "installed drag is the same order as the whole canopy's - 47% "
              "of it here, so not a correction term");
        Check(installed.momentBodyNm.y != 0.0,
              "harness drag on a 7.8 m arm is a pitching moment too");
        const double wholeAircraftGlide = trim.liftCoefficient / totalCd;
        Check(wholeAircraftGlide > 8.0 && wholeAircraftGlide < 11.0,
              "and with it the glide lands near the published 9.5, from "
              "geometry and published drag figures rather than from a fitted "
              "polar");

        // Separation with memory. The plan asks for attached and separated
        // states, and they do two jobs: stall gets the hysteresis it
        // physically has, and the solve gets a lift curve that stays put
        // under it instead of switching branches mid-iteration.
        {
            const SectionPolarTable polar = SectionPolarTable::Analytic();
            // The same incidence, approached from attached and from stalled,
            // must not give the same answer. That is the whole point.
            const double justBelowStall = polar.StallAngleRad(0.0) - 0.04;
            const double fromAttached =
                polar.SeparationEquilibrium(justBelowStall, 0.0, 0.0);
            const double fromStalled =
                polar.SeparationEquilibrium(justBelowStall, 0.0, 1.0);
            std::printf("  hysteresis at %.1f deg: separation %.2f coming up, "
                        "%.2f coming back down\n",
                        justBelowStall * 180.0 / Pi, fromAttached, fromStalled);
            Check(fromAttached < 0.05,
                  "below the stall angle, a section that was flying is still "
                  "flying");
            Check(fromStalled > fromAttached + 0.4,
                  "but one that has stalled is still largely stalled at the "
                  "same incidence - it has to be brought further down before "
                  "it reattaches");

            // Ramping brake up and back down must trace a loop, not a line.
            // A ramp is many solves, so cap the work per step: what is being
            // checked is where the separation state goes, not the last digit
            // of each circulation solve.
            VsmSettings ramping;
            ramping.maxIterations = 150;
            VsmSeparationState state;
            const auto rampTo = [&](double brake)
            {
                VsmSolveInput input = Inflow(0.06, 11.0);
                input.leftBrake = brake;
                input.rightBrake = brake;
                double lift = 0.0;
                for (int step = 0; step < 20; ++step)
                    lift = wing.SolveUnsteady(input, state, 1.0 / 60.0,
                                              ramping).liftCoefficient;
                return lift;
            };
            double climbing = 0.0;
            for (int step = 0; step <= 6; ++step)
                climbing = rampTo(0.1 * step);
            // Past the stall break. Where it falls depends on the assumed
            // full-brake deflection, so what is checked is that a break
            // exists and that it is not recovered on the way back.
            rampTo(0.75);
            const double atPeakBrake = rampTo(0.95);
            double descending = 0.0;
            for (int step = 8; step >= 6; --step)
                descending = rampTo(0.1 * step);
            std::printf("  brake 0.6: CL %.3f on the way up, %.3f on the way "
                        "back down\n", climbing, descending);
            Check(atPeakBrake < climbing,
                  "the wing stalls: past a point more brake is less lift");
            Check(descending < climbing,
                  "and coming back down it does not recover the lift it had "
                  "at the same brake going up");
        }

        // Deep stall is where a steady solve runs out of meaning.
        VsmSolveInput deep = Inflow(0.06, 11.0);
        deep.leftBrake = 0.9;
        deep.rightBrake = 0.9;
        const VsmSolution stalled = wing.Solve(deep);
        std::printf("  full brake 0.9: converged %d, residual %.1e, "
                    "relaxation fell to %.4f\n",
                    static_cast<int>(stalled.converged), stalled.residual,
                    stalled.finalRelaxation);
        std::printf("  KNOWN LIMITATION: deeply stalled cases still do not "
                    "settle, and now for a\n"
                    "  better reason than branch-swapping - holding the "
                    "separation state fixed removed\n"
                    "  that. What is left is physical: the separated branch "
                    "has a NEGATIVE lift slope,\n"
                    "  which inverts the sign of the downwash feedback "
                    "between sections, and a wing\n"
                    "  in deep stall genuinely has no stable steady state. "
                    "The unsteady wake of\n"
                    "  Level 11 is the honest treatment. Until then nothing "
                    "past the stall break\n"
                    "  should be read as a flight number.\n");
        Check(!stalled.converged,
              "KNOWN FAILURE: deep stall does not converge. Delete this check "
              "when it does");
    }

    // -- apparent mass ----------------------------------------------------
    {
        const CanopyGeometry canopy;
        const ApparentMassTensor apparent = CanopyApparentMass(canopy);
        std::printf("Apparent mass (Lissaman & Brown): "
                    "%.1f / %.1f / %.1f kg,  %.1f / %.1f / %.1f kg m2\n",
                    apparent.massKg.x, apparent.massKg.y, apparent.massKg.z,
                    apparent.inertiaKgM2.x, apparent.inertiaKgM2.y,
                    apparent.inertiaKgM2.z);

        // The ordering is the physics: a wing accelerating broadside carries
        // far more air than one accelerating edgewise or along its chord.
        Check(apparent.massKg.z > apparent.massKg.y,
              "normal apparent mass is the largest - a wing pushed broadside "
              "drags a cylinder of air with it");
        Check(apparent.massKg.y > apparent.massKg.x,
              "and chordwise is the smallest, a thin plate edge-on");
        Check(apparent.massKg.x > 0.0 && apparent.inertiaKgM2.x > 0.0,
              "every term is positive");

        // Scale. This is why the plan calls apparent mass not optional: on a
        // wing loaded at 4 kg per square metre the air it accelerates is a
        // large fraction of the aircraft, not a correction to it.
        constexpr double AllUpMassKg = 100.0;
        const double normalFraction = apparent.massKg.z / AllUpMassKg;
        std::printf("  normal apparent mass is %.0f%% of a 100 kg all-up "
                    "aircraft\n", 100.0 * normalFraction);
        Check(normalFraction > 0.15 && normalFraction < 1.2,
              "normal apparent mass is a large fraction of the aircraft, "
              "which is the reason it cannot be left out");

        // It comes from geometry, so it must move with geometry.
        const ApparentMassTensor denser =
            CanopyApparentMass(canopy, 0.155, 2.0 * 1.225);
        CheckWithin(denser.massKg.z, 2.0 * apparent.massKg.z, 1e-9,
                    "apparent mass is proportional to air density");
        const ApparentMassTensor bigger =
            LissamanBrownApparentMass(2.0 * canopy.ProjectedSpanM(),
                canopy.ProjectedAreaM2() / canopy.ProjectedSpanM(), 0.155);
        Check(bigger.massKg.z > apparent.massKg.z,
              "and a wing of twice the span carries more air");
        Check(bigger.inertiaKgM2.x > 4.0 * apparent.inertiaKgM2.x,
              "roll apparent inertia grows fast with span - it is the term "
              "that slows a big wing into a turn");

        // Cross-check against the estimate the flight model already carried,
        // arrived at independently. The linear terms agree; the rotational
        // ones do not, and that disagreement is recorded rather than split
        // down the middle.
        const WingParameters wing;
        std::printf("  normal apparent mass %.1f kg against the model's "
                    "existing estimate of %.1f kg\n",
                    apparent.massKg.z, wing.apparentMassKg.z);
        CheckWithin(apparent.massKg.z, wing.apparentMassKg.z, 15.0,
                    "normal apparent mass agrees with the independent "
                    "estimate already in the model");
        std::printf("  KNOWN DISAGREEMENT: roll apparent inertia is %.0f "
                    "kg m2 here against %.0f in\n"
                    "  WingParameters. The rotational coefficients could not "
                    "be checked against the\n"
                    "  source paper, so they are registered Disputed and "
                    "nothing uses their magnitude.\n",
                    apparent.inertiaKgM2.x, wing.apparentRotationalInertiaKgM2.x);
        Check(apparent.inertiaKgM2.x > wing.apparentRotationalInertiaKgM2.x,
              "KNOWN DISAGREEMENT: the roll term is the one to check against "
              "the paper. Delete this when it has been");
    }

    // -- the section's pitching moment ------------------------------------
    //
    // Checked against the closed-form thin-airfoil result rather than against
    // itself, because it was wrong by a factor of four and nothing noticed.
    // For a circular-arc camber line the Fourier coefficients of the camber
    // slope are A1 = 4 h/c and A2 = 0, so
    //
    //     Cm_c/4 = (pi/4)(A2 - A1) = -pi h/c
    //
    // and the SAME A1 gives the zero-lift angle -2 h/c. Deriving one from
    // A1 = 4 h/c and the other from A1 = h/c describes two different sections.
    {
        const SectionPolarTable table = SectionPolarTable::Analytic();
        AnalyticPolarSpec spec;
        const double camber = spec.camberFraction;
        const SectionPolarSample sample = table.Sample(0.05, 0.0);
        const double expected = -3.14159265358979323846 * camber;
        std::printf("Section Cm at %.1f%% camber: %.4f, thin-airfoil "
                    "-pi h/c = %.4f\n",
                    camber * 100.0, sample.momentCoefficient, expected);
        Check(std::fabs(sample.momentCoefficient - expected) < 1.0e-9,
              "the quarter-chord moment is -pi (h/c), the closed-form result "
              "for the camber line this section already uses for its "
              "zero-lift angle");

        // The two must come from one camber line, which is a relationship
        // rather than two numbers: Cm/alpha0 = (-pi h/c)/(-2 h/c) = pi/2,
        // independent of the camber itself.
        const double ratio =
            sample.momentCoefficient / table.ZeroLiftAngleRad(0.0);
        Check(std::fabs(ratio - 3.14159265358979323846 / 2.0) < 1.0e-9,
              "and it is the same camber line the zero-lift angle came from - "
              "their ratio is pi/2 whatever the camber is");

        // It is a couple, so it does not vanish where the section makes no
        // lift. That is the property that sets a wing's trim incidence.
        const SectionPolarSample atZeroLift =
            table.Sample(table.ZeroLiftAngleRad(0.0), 0.0);
        Check(std::fabs(atZeroLift.liftCoefficient) < 1.0e-6,
              "at the zero-lift angle the section makes no lift");
        Check(atZeroLift.momentCoefficient < -0.01,
              "and still carries its nose-down couple, which is what a "
              "pitching moment about the aerodynamic centre means");
    }

    // Level 10, item 14: the polar cache, and the four ways it must refuse.
    //
    // Caching the solved table turns a 1021 ms construction into a read. The
    // risk it introduces is the one this whole level was built away from: a
    // file that disagrees with the geometry that produced it would put a
    // stated section polar back, with none of the honesty of having declared
    // it stated. So what is gated here is not that the cache is fast. It is
    // that the cache REFUSES - on a changed spec, on a changed solver, on a
    // corrupted file and on a truncated one - and that a refusal costs
    // accuracy nothing because the solve is still there behind it.
    {
        std::printf("Polar cache:\n");
        const std::filesystem::path scratch =
            std::filesystem::temp_directory_path() / "parapenting-polar-test";
        std::filesystem::remove_all(scratch);
        std::filesystem::create_directories(scratch);
        setenv("PARAPENTING_POLAR_CACHE", scratch.c_str(), 1);

        ComputedPolarSpec spec;
        const SectionPolarTable solved = SectionPolarTable::Computed(spec);
        Check(SaveSectionPolarTable(spec, solved), "a solved table saves");

        // Round trip. Compared on what callers actually read, at incidences
        // either side of the stall and at both ends of the brake axis, rather
        // than on a checksum of the bytes.
        {
            SectionPolarTable loaded;
            const SectionPolarCacheResult result =
                LoadSectionPolarTable(spec, loaded);
            Check(result.hit, "and loads again");
            bool identical = result.hit;
            for (const double brake : {0.0, 0.5, 1.0})
            {
                for (double alpha = -0.20; alpha <= 0.40; alpha += 0.01)
                {
                    const SectionPolarSample a = solved.Sample(alpha, brake);
                    const SectionPolarSample b = loaded.Sample(alpha, brake);
                    identical = identical
                        && a.liftCoefficient == b.liftCoefficient
                        && a.dragCoefficient == b.dragCoefficient
                        && a.momentCoefficient == b.momentCoefficient;
                }
                identical = identical
                    && solved.StallAngleRad(brake)
                           == loaded.StallAngleRad(brake)
                    && solved.MaximumLiftCoefficient(brake)
                           == loaded.MaximumLiftCoefficient(brake);
            }
            Check(identical,
                  "and is BIT-identical to the solved table across the stall "
                  "and the brake axis - a cache that is merely close is a "
                  "second model");
        }

        // A changed section must not load a table solved for the old one.
        {
            ComputedPolarSpec other = spec;
            other.section.maxCamberFraction += 0.001;
            SectionPolarTable loaded;
            const SectionPolarCacheResult result =
                LoadSectionPolarTable(other, loaded);
            Check(!result.hit,
                  "a section a thousandth of a chord different does not load "
                  "the old table");
        }

        // The check the spec cannot make. Editing the stored witness stands in
        // for the real hazard - the viscous solver changing while every input
        // stays identical - because on load the witness is re-solved and
        // compared, and that is the comparison that fails.
        {
            const std::string path = SectionPolarCachePath(spec);
            std::fstream file(path, std::ios::binary | std::ios::in
                                        | std::ios::out);
            Check(static_cast<bool>(file), "the cache file is there to edit");
            // Magic, version, then the spec; the witness follows.
            file.seekp(sizeof(char) * 8 + sizeof(std::uint32_t)
                       + 15 * sizeof(double) + 2 * sizeof(std::uint64_t),
                       std::ios::beg);
            const double wrong = 1234.5;
            file.write(reinterpret_cast<const char*>(&wrong), sizeof(wrong));
            file.close();

            SectionPolarTable loaded;
            const SectionPolarCacheResult result =
                LoadSectionPolarTable(spec, loaded);
            std::printf("  witness edited: %s\n",
                        SectionPolarCacheMissName(result.miss));
            Check(!result.hit,
                  "a table whose witness no longer reproduces is REFUSED - "
                  "this is what catches a changed solver with an unchanged "
                  "spec, and it needs no version constant to be remembered");
        }

        // Truncation. A short file that happens to parse must not become a
        // table with the wrong shape.
        {
            Check(SaveSectionPolarTable(spec, solved), "resaved");
            const std::string path = SectionPolarCachePath(spec);
            const auto size = std::filesystem::file_size(path);
            std::filesystem::resize_file(path, size / 2);
            SectionPolarTable loaded;
            const SectionPolarCacheResult result =
                LoadSectionPolarTable(spec, loaded);
            std::printf("  truncated: %s\n",
                        SectionPolarCacheMissName(result.miss));
            Check(!result.hit, "a truncated cache is refused");
        }

        // And the fallback is real: with the cache directory disabled
        // entirely, the table still comes out and still agrees.
        {
            setenv("PARAPENTING_POLAR_CACHE", "", 1);
            SectionPolarTable loaded;
            Check(!LoadSectionPolarTable(spec, loaded).hit,
                  "an empty cache directory disables the cache");
            const SectionPolarTable cold = SectionPolarTable::Computed(spec);
            Check(cold.MaximumLiftCoefficient(0.0)
                      == solved.MaximumLiftCoefficient(0.0),
                  "and solving from cold gives the same wing it always did");
        }

        std::filesystem::remove_all(scratch);
        unsetenv("PARAPENTING_POLAR_CACHE");
    }

    if (Failures == 0) std::printf("All aerodynamics checks passed.\n");
    else std::printf("%d aerodynamics check(s) failed.\n", Failures);
    return Failures == 0 ? 0 : 1;
}
