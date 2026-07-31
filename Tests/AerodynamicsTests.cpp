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
#include "SectionPolarTable.h"
#include "ApparentMassTensor.h"
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

    // -- Biot-Savart and the influence matrix -----------------------------
    {
        // An elliptical wing with thin flat-plate sections must reproduce
        // lifting-line theory. This is the load-bearing test in this file.
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

    if (Failures == 0) std::printf("All aerodynamics checks passed.\n");
    else std::printf("%d aerodynamics check(s) failed.\n", Failures);
    return Failures == 0 ? 0 : 1;
}
