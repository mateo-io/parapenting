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
}

int main()
{
    // -- a pressurised cell holds its shape -------------------------------
    {
        CanopyMembraneSolver section = MakeSection();
        const MembraneResult settled = section.Settle(Pressurised(65.0), 4.0);
        std::printf("Pressurised strip: sagitta %.1f mm (%.3f of cell width), "
                    "max strain %.4f, slack %.0f%%\n",
                    1000.0 * settled.sagittaM, settled.sagittaFraction,
                    settled.maximumStrain, 100.0 * settled.slackFraction);

        // Level 1 solves this same bulge statically from the seam allowance.
        // Arriving at the same place dynamically, from pressure and
        // tension-only fabric, is the check that both are describing one wing.
        Check(settled.sagittaFraction > 0.05
              && settled.sagittaFraction < 0.30,
              "a pressurised strip bulges into the slack its seam allowance "
              "gave it");
        Check(settled.sagittaM > 0.0, "and holds that bulge");
        Check(settled.maximumStrain >= 0.0 && settled.maximumStrain < 0.05,
              "the fabric stretches by a few percent at most - it is a "
              "membrane, not a balloon");

        // The stability gate. Energy must not grow.
        const double energyAtRest = settled.kineticEnergyJ;
        const MembraneResult later = section.Settle(Pressurised(65.0), 6.0);
        std::printf("  kinetic energy %.3e J after 4 s, %.3e J after 10 s\n",
                    energyAtRest, later.kineticEnergyJ);
        Check(later.kineticEnergyJ <= energyAtRest + 1.0e-6,
              "and the section does not gain energy by sitting there - the "
              "explosive growth gate");
        Check(std::isfinite(later.kineticEnergyJ)
              && std::isfinite(later.maximumStrain),
              "everything stays finite");

        // XPBD's compliance must mean the same thing at any iteration count.
        // That is the whole reason for using it over PBD: the stiffness of
        // the fabric cannot be allowed to depend on the solver budget.
        MembraneSpec fewer;
        fewer.constraintIterations = 8;
        MembraneSpec more;
        more.constraintIterations = 64;
        const MembraneResult coarse =
            MakeSection(fewer).Settle(Pressurised(65.0), 4.0);
        const MembraneResult fine =
            MakeSection(more).Settle(Pressurised(65.0), 4.0);
        std::printf("  sagitta %.4f at 8 iterations, %.4f at 64\n",
                    coarse.sagittaFraction, fine.sagittaFraction);
        Check(std::fabs(coarse.sagittaFraction - fine.sagittaFraction)
                  < 0.02,
              "and its shape is the same at 8 iterations as at 64, because "
              "XPBD's compliance is metres per newton rather than a stiffness "
              "the iteration count sets");
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

    // -- fabric anisotropy is real ----------------------------------------
    {
        // Bias softness is what governs how a canopy wrinkles, so it must
        // actually reach the solve rather than sitting in a struct.
        MembraneSpec stiffBias;
        stiffBias.fabric.biasStiffnessNPerM =
            stiffBias.fabric.warpStiffnessNPerM;
        const MembraneResult isotropic =
            MakeSection(stiffBias).Settle(Pressurised(65.0), 4.0);
        const MembraneResult anisotropic =
            MakeSection().Settle(Pressurised(65.0), 4.0);
        std::printf("Anisotropy: max strain %.4f on bias-soft fabric, %.4f "
                    "with the bias stiffened to the warp\n",
                    anisotropic.maximumStrain, isotropic.maximumStrain);
        Check(anisotropic.maximumStrain > isotropic.maximumStrain,
              "fabric that is soft on the bias stretches further than fabric "
              "that is stiff everywhere");
    }

    if (Failures == 0) std::printf("All membrane checks passed.\n");
    else std::printf("%d membrane check(s) failed.\n", Failures);
    return Failures == 0 ? 0 : 1;
}
