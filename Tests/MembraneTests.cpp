// Level 6: the skin is fabric.
//
// The plan's Level 6 exit gates: pressurised cells hold their shape without
// explosive energy growth, attachment loads deform the canopy smoothly,
// released deformation oscillates and damps, and the Level 4 aero gates still
// pass with a deformable canopy.
#include "CanopyMembraneSolver.h"

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

// EPIC 2 ML rib spacing, from Level 1.
constexpr double CellWidthM = 0.2622;

CanopyMembraneSolver MakeSection(const MembraneSpec& spec = {})
{
    return CanopyMembraneSolver(CellWidthM, spec);
}

MembraneLoad Pressurised(double pascals)
{
    MembraneLoad load;
    load.internalPressurePa = pascals;
    return load;
}

// Do two non-adjacent segments of the strip cross?
//
// The invariant a piece of fabric obeys, stated geometrically rather than as a
// number to match: a sheet does not pass through itself. Counted with an
// orientation test on the settled polyline, which needs no tolerance and no
// reference value - either the segments cross or they do not.
int SegmentCrossings(const std::vector<Vec3>& points)
{
    const auto cross2D = [](const Vec3& o, const Vec3& a, const Vec3& b)
    {
        return (a.y - o.y) * (b.z - o.z) - (a.z - o.z) * (b.y - o.y);
    };
    const auto straddles = [&](const Vec3& p1, const Vec3& p2,
                               const Vec3& q1, const Vec3& q2)
    {
        const double d1 = cross2D(p1, p2, q1);
        const double d2 = cross2D(p1, p2, q2);
        const double d3 = cross2D(q1, q2, p1);
        const double d4 = cross2D(q1, q2, p2);
        return ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0));
    };
    int crossings = 0;
    const std::size_t count = points.size();
    for (std::size_t i = 0; i + 1 < count; ++i)
        for (std::size_t j = i + 2; j + 1 < count; ++j)
            if (straddles(points[i], points[i + 1],
                          points[j], points[j + 1]))
                ++crossings;
    return crossings;
}
}

// The analytic answer this is checked against.
//
// A chain of cut length L pinned across a gap c, pushed out by uniform
// pressure, settles as a circular arc. If its half-angle is t then
// c = 2R sin t and L = 2R t, so t/sin t = L/c pins the shape completely and
// the sagitta is R(1 - cos t). The hoop tension is p R, and the fabric
// stretches by that over its membrane stiffness. Nothing about the solver
// enters any of it.
struct AnalyticArc
{
    double sagittaM = 0.0;
    double sagittaFraction = 0.0;
    double hoopTensionNPerM = 0.0;
    double strain = 0.0;
};

AnalyticArc SolveArc(double cellWidthM, double seamAllowance,
                     double pressurePa, double membraneStiffnessNPerM)
{
    const double cutLength = cellWidthM * (1.0 + seamAllowance);
    const double ratio = cutLength / cellWidthM;
    double low = 1.0e-6;
    double high = 3.0;
    for (int iteration = 0; iteration < 200; ++iteration)
    {
        const double mid = 0.5 * (low + high);
        if (mid / std::sin(mid) < ratio) low = mid;
        else high = mid;
    }
    const double halfAngle = 0.5 * (low + high);
    AnalyticArc arc;
    const double radius = cellWidthM / (2.0 * std::sin(halfAngle));
    arc.sagittaM = radius * (1.0 - std::cos(halfAngle));
    arc.sagittaFraction = arc.sagittaM / cellWidthM;
    arc.hoopTensionNPerM = pressurePa * radius;
    arc.strain = arc.hoopTensionNPerM / membraneStiffnessNPerM;
    return arc;
}

int main()
{
    // -- against the analytic arc -----------------------------------------
    {
        const MembraneSpec spec;
        // The panel is cut warp-spanwise, so a spanwise strip runs along the
        // warp and it is the warp stiffness that resists it.
        const AnalyticArc arc = SolveArc(
            CellWidthM, spec.seamAllowanceFraction, 65.0,
            spec.fabric.warpStiffnessNPerM);
        std::printf("Analytic arc: sagitta %.2f mm (%.4f of width), hoop "
                    "tension %.2f N/m, strain %.5f\n",
                    1000.0 * arc.sagittaM, arc.sagittaFraction,
                    arc.hoopTensionNPerM, arc.strain);

        CanopyMembraneSolver section = MakeSection();
        const MembraneResult settled = section.Settle(Pressurised(65.0), 8.0);
        std::printf("Solved:       sagitta %.2f mm (%.4f of width), "
                    "strain %.5f, slack %.0f%%\n",
                    1000.0 * settled.sagittaM, settled.sagittaFraction,
                    settled.maximumStrain, 100.0 * settled.slackFraction);

        // The solved arc sits a little above the inextensible one because the
        // fabric does stretch, and a 24-segment polyline cuts a corner off the
        // continuous curve. Both are small and both have the right sign.
        Check(settled.sagittaM > arc.sagittaM,
              "the solved bulge is a little deeper than the inextensible arc, "
              "because the fabric stretches");
        Check(settled.sagittaM < 1.05 * arc.sagittaM,
              "and within 5% of it - the shape is the pattern's, not the "
              "solver's");
        Check(settled.maximumStrain > 0.3 * arc.strain
              && settled.maximumStrain < 3.0 * arc.strain,
              "and the fabric carries the strain the hoop tension implies");
        Check(settled.slackFraction < 1.0e-9,
              "with every segment in tension - a pressurised cell has no "
              "slack skin anywhere");
    }

    // -- it is converged, not merely stable -------------------------------
    {
        // The failure this replaces looked like an equilibrium: kinetic
        // energy fell to zero and the shape stopped moving, at 91 mm against
        // an analytic 26. It was a stalled Gauss-Seidel state, and the tell
        // was that it moved with the iteration count. So that is what gets
        // checked, rather than whether it settles.
        const auto solveWith = [](int substeps, int iterations)
        {
            MembraneSpec spec;
            spec.substeps = substeps;
            spec.constraintIterations = iterations;
            return MakeSection(spec).Settle(Pressurised(65.0), 8.0);
        };
        const MembraneResult coarse = solveWith(4, 8);
        const MembraneResult fine = solveWith(8, 64);
        std::printf("Budget independence: sagitta %.3f mm at 4x8, %.3f mm at "
                    "8x64\n", 1000.0 * coarse.sagittaM, 1000.0 * fine.sagittaM);
        Check(std::fabs(coarse.sagittaM - fine.sagittaM)
                  < 0.005 * fine.sagittaM,
              "the shape is the same to within half a percent at 4 substeps "
              "and 8 iterations as at 8 and 64 - a solve that moves with its "
              "own budget has not converged, whatever its energy is doing");

        // And the same for the solver mass, which is a device and must not
        // decide the answer.
        // A heavier solver mass settles more slowly, so it is given longer
        // to get there - the claim is that it reaches the same place, not
        // that it reaches it as fast.
        MembraneSpec heavier;
        heavier.solverMassScale = 1.0e5;
        const MembraneResult heavy =
            MakeSection(heavier).Settle(Pressurised(65.0), 40.0);
        std::printf("  sagitta %.3f mm at ten times the solver mass\n",
                    1000.0 * heavy.sagittaM);
        Check(std::fabs(heavy.sagittaM - fine.sagittaM)
                  < 0.005 * fine.sagittaM,
              "and the same again at ten times the solver mass, which is what "
              "makes that mass a numerical device rather than a tuning knob");
    }

    // -- energy ------------------------------------------------------------
    {
        CanopyMembraneSolver section = MakeSection();
        const MembraneResult settled = section.Settle(Pressurised(65.0), 6.0);
        const MembraneResult later = section.Settle(Pressurised(65.0), 6.0);
        std::printf("Energy: %.3e J after 6 s, %.3e J after 12 s\n",
                    settled.kineticEnergyJ, later.kineticEnergyJ);
        Check(later.kineticEnergyJ <= settled.kineticEnergyJ + 1.0e-9,
              "a pressurised cell does not gain energy by sitting there");
        Check(std::isfinite(later.kineticEnergyJ)
              && std::isfinite(later.maximumStrain),
              "everything stays finite");
        Check(std::fabs(later.sagittaM - settled.sagittaM) < 1.0e-7,
              "and holds its shape");
    }

    // -- fabric anisotropy -------------------------------------------------
    {
        // The bias is what governs how a canopy wrinkles and folds, so it has
        // to reach the solve. A panel cut warp-spanwise puts the strip along
        // the warp; cut at 45 degrees the same strip is on the bias.
        const auto cutAt = [](double degrees)
        {
            MembraneSpec spec;
            spec.fabric.warpAngleRad = degrees * Pi / 180.0;
            return MakeSection(spec).Settle(Pressurised(65.0), 12.0);
        };
        const MembraneResult alongWarp = cutAt(90.0);
        const MembraneResult offAxis = cutAt(70.0);
        const MembraneResult onBias = cutAt(45.0);
        std::printf("Cut angle: strain %.5f along the warp, %.5f at 70 deg, "
                    "%.5f on the bias\n",
                    alongWarp.maximumStrain, offAxis.maximumStrain,
                    onBias.maximumStrain);
        Check(offAxis.maximumStrain > alongWarp.maximumStrain,
              "taking the panel off the thread line softens it");
        Check(onBias.maximumStrain > 4.0 * alongWarp.maximumStrain,
              "and on the bias it is several times softer - the weave can "
              "shear where the threads cannot stretch");
        Check(onBias.sagittaM > alongWarp.sagittaM,
              "so a bias-cut panel bulges further under the same pressure");
    }

    // -- pressure decides how firm the section is -------------------------
    {
        const MembraneResult soft =
            MakeSection().Settle(Pressurised(8.0), 4.0);
        const MembraneResult firm =
            MakeSection().Settle(Pressurised(120.0), 4.0);
        std::printf("Section firmness: slack %.0f%% at 8 Pa, %.0f%% at "
                    "120 Pa\n",
                    100.0 * soft.slackFraction, 100.0 * firm.slackFraction);
        Check(firm.slackFraction <= soft.slackFraction,
              "more pressure pulls more of the skin into tension");
        Check(firm.maximumStrain > soft.maximumStrain,
              "and stretches it further");

        // A cell with no pressure at all cannot hold a section. This is what
        // Level 8 will grow a collapse out of.
        const MembraneResult dead =
            MakeSection().Settle(Pressurised(0.0), 3.0);
        std::printf("  with no pressure: slack %.0f%%, sagitta %.1f mm\n",
                    100.0 * dead.slackFraction, 1000.0 * dead.sagittaM);
        Check(dead.slackFraction > 0.5,
              "an unpressurised cell has most of its skin carrying nothing");
        Check(dead.sagittaFraction < firm.sagittaFraction,
              "and loses the section the pressure was holding");
    }

    // -- an attachment load deforms it smoothly ---------------------------
    {
        CanopyMembraneSolver section = MakeSection();
        section.Settle(Pressurised(65.0), 4.0);
        const MembraneResult before = section.Settle(Pressurised(65.0), 0.5);

        MembraneLoad braked = Pressurised(65.0);
        braked.brakeLineForceN = 90.0;
        const MembraneResult pulled = section.Settle(braked, 2.0);
        std::printf("Attachment load at 90 N: sagitta %.1f mm against "
                    "%.1f mm unloaded\n",
                    1000.0 * pulled.sagittaM, 1000.0 * before.sagittaM);
        Check(std::fabs(pulled.sagittaM - before.sagittaM) > 1.0e-5,
              "a line pulling on the skin deforms it");
        Check(std::isfinite(pulled.maximumStrain)
              && pulled.maximumStrain < 0.10,
              "and the skin deforms rather than tearing through its own "
              "constraints");

        // Smoothly: no node may be far from its neighbours.
        double largestGap = 0.0;
        for (std::size_t index = 1; index < pulled.positionM.size(); ++index)
            largestGap = std::max(largestGap,
                Length(pulled.positionM[index] - pulled.positionM[index - 1]));
        Check(largestGap < 0.5 * CellWidthM,
              "the deformed section stays a section - no node has been thrown "
              "clear of the skin");
    }

    // -- released deformation oscillates and damps ------------------------
    {
        CanopyMembraneSolver section = MakeSection();
        section.Settle(Pressurised(65.0), 4.0);
        MembraneLoad braked = Pressurised(65.0);
        braked.brakeLineForceN = 120.0;
        section.Settle(braked, 1.5);

        // Release it and watch the energy.
        double peak = 0.0;
        double late = 0.0;
        for (int step = 0; step < 240; ++step)
        {
            const MembraneResult now = section.Step(Pressurised(65.0), 1.0 / 120.0);
            if (step < 60) peak = std::max(peak, now.kineticEnergyJ);
            if (step >= 200) late = std::max(late, now.kineticEnergyJ);
        }
        std::printf("Release: peak energy %.3e J, %.3e J a second and a half "
                    "later\n", peak, late);
        Check(peak > 0.0, "releasing the brake sets the skin moving");
        Check(late < 0.5 * peak,
              "and it damps rather than ringing indefinitely");
    }

    // -- determinism ------------------------------------------------------
    {
        const auto run = []()
        {
            CanopyMembraneSolver section = MakeSection();
            MembraneLoad load = Pressurised(65.0);
            load.brakeLineForceN = 40.0;
            return section.Settle(load, 2.0);
        };
        const MembraneResult first = run();
        const MembraneResult second = run();
        bool identical =
            first.positionM.size() == second.positionM.size();
        for (std::size_t index = 0;
             identical && index < first.positionM.size(); ++index)
        {
            identical = first.positionM[index].x == second.positionM[index].x
                && first.positionM[index].z == second.positionM[index].z;
        }
        Check(identical,
              "the same input gives a bit-identical section - fixed substeps "
              "and a fixed iteration count, no adaptive anything");
    }

    // -- the ribs converge, and the skin folds rather than shrinking -------
    {
        // A collapsing section is not the same ribs with less pressure. The
        // cell loses its air, the shell loses what held it open, and the ribs
        // are drawn together - which is when the cut fabric has length to
        // spare and hangs into a fold. That fold depth is what Level 8 reads
        // to decide whether a folded tip reaches the lines.
        const auto collapseAt = [](double ribSeparationFraction)
        {
            CanopyMembraneSolver section = MakeSection();
            // Pressurised first, so this starts as a flying section rather
            // than as a flat line folding from nothing.
            section.Settle(Pressurised(65.0), 4.0);
            MembraneLoad collapsing = Pressurised(0.0);
            collapsing.externalDynamicPressurePa = 240.0;
            collapsing.brakeLineForceN = 26.0;
            collapsing.ribSeparationFraction = ribSeparationFraction;
            return section.Settle(collapsing, 3.0);
        };

        std::printf("Rib convergence:\n");
        double previousFold = 0.0;
        bool deepens = true;
        int worstCrossings = 0;
        for (const double separation : {1.0, 0.8, 0.55, 0.35, 0.2, 0.1})
        {
            const MembraneResult folded = collapseAt(separation);
            const int crossings = SegmentCrossings(folded.positionM);
            worstCrossings = std::max(worstCrossings, crossings);
            std::printf("  ribs at %.2f of spacing: fold %6.1f mm, "
                        "crossings %d\n",
                        separation, 1000.0 * folded.foldDepthM, crossings);
            if (folded.foldDepthM < previousFold) deepens = false;
            previousFold = folded.foldDepthM;
        }
        Check(deepens,
              "drawing the ribs together deepens the fold - the skin has "
              "nowhere to go but out of plane, because its cut length does "
              "not change");

        // The measurement that removed a feature. The plan puts fabric
        // self-collision in this solver for cravats; it was built here and
        // taken out again, because a one-dimensional strip cannot
        // self-intersect. A chain under a transverse load with both ends
        // pinned settles as a simple curve, and no amount of collapsing
        // changes that - the fold merely deepens. Collision code on this
        // object could never have fired, and this is the check that says so.
        //
        // Crumpling needs loads that reverse along the skin, which needs the
        // two-dimensional mesh Level 6 does not have. A cravat is fabric
        // caught on a LINE, which is contact between two things that both
        // exist, and that is where Level 8 builds it.
        Check(worstCrossings == 0,
              "a spanwise strip never folds through itself at any rib "
              "convergence, which is why self-collision here would be dead "
              "code rather than physics");

        // And a flying section is unaffected by any of this: rib separation
        // of one is the wing, and the analytic-arc result above still stands.
        const MembraneResult flying =
            MakeSection().Settle(Pressurised(65.0), 8.0);
        Check(flying.foldDepthM < 1.0e-9,
              "a pressurised section has no fold at all - it bulges, which is "
              "the sagitta the analytic arc is checked against");
    }

    if (Failures == 0) std::printf("All membrane checks passed.\n");
    else std::printf("%d membrane check(s) failed.\n", Failures);
    return Failures == 0 ? 0 : 1;
}
