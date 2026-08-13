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
        };
        const auto march = [&](bool lagCirculation, int steps)
        {
            VsmSettings settings;
            settings.lagCirculation = lagCirculation;
            VsmSeparationState state;
            Trace out;
            // Hold trim long enough that both the separation state and the lag
            // state are settled on it. A transient here would be indexed as a
            // response to the step, which is exactly the seed error strand 2
            // measured and fixed on its own gate.
            for (int step = 0; step < 600; ++step)
                wing.SolveUnsteady(Inflow(AlphaRad, TrimSpeedMps), state,
                                   StepSeconds, settings);
            out.before = totalCirculation(state);
            for (int step = 0; step < steps; ++step)
            {
                const VsmSolution solved = wing.SolveUnsteady(
                    Inflow(AlphaRad, TrimSpeedMps * StepFactor), state,
                    StepSeconds, settings);
                out.circulation.push_back(totalCirculation(state));
                double aimed = 0.0;
                for (const VsmSectionResult& section : solved.sections)
                    aimed += section.quasiSteadyCirculation;
                out.target.push_back(aimed);
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
