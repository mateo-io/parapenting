#pragma once

#include "ParagliderDynamics.h"

#include <vector>

namespace Parapenting::Physics
{
// Level 6 of the master plan: the skin is fabric, not a surface.
//
// ============================ NOT WORKING =============================
// This does not converge and is not in the test suite. It is committed
// because the diagnosis is worth more than the code, and the next person
// should not have to rediscover it.
//
// Measured: a strip cut with a 2.6% seam allowance across a 262 mm cell
// should bulge to a sagitta of about 26 mm - c*sqrt(3*slack/8) for a shallow
// arc - and stretch by about 0.03%, since the hoop tension p*R is around
// 9 N/m against a membrane stiffness of 30000 N/m. What it actually does is
// bulge to 91 mm and report 28% strain, and the answer changes with the
// iteration count: sagitta 0.62 of cell width at 8 iterations, 0.24 at 64.
// So the constraints are not holding and the solve has not converged.
//
// The compliance arithmetic checks out on paper - alpha = compliance/h^2 is
// around 0.08 against an inverse-mass sum of 4370, so the constraints should
// behave as nearly rigid - which points at the iteration scheme rather than
// the material model: 24 Gauss-Seidel sweeps over a 24-segment chain only
// just lets information cross it, and the pressure load injects a large
// displacement each substep for the solve to then remove. Worth trying:
// far more iterations, a coarser chain, or solving the chain directly rather
// than by relaxation.
//
// The first attempt was worse and is recorded in the note below: a chordwise
// loop is the wrong object entirely.
// ======================================================================
//
// Everything above this point has treated the canopy as rigid. Level 1 solves
// the inflated section from the seam allowance, but it solves it once, as a
// circular arc in equilibrium - it cannot show a section going soft as its
// cell loses pressure, or the trailing edge being pulled down by a brake line,
// or the skin letting go and folding.
//
// This is a SPANWISE strip: nodes running from one rib to the next across a
// single cell, at one chord station, pinned where the ribs hold them and free
// to bulge between. That is the direction the fabric actually works in - the
// ribs hold the profile, and the skin between them takes the pressure. A
// chordwise loop was the first thing tried here and it is simply the wrong
// object: a closed loop of fabric with pressure inside and only its ends
// pinned is a balloon, and it inflates to a circle no matter what profile it
// was cut to.
//
// It is XPBD, so the constraints have a real compliance in metres per newton
// rather than a stiffness that depends on how many iterations were run - which
// matters because the iteration count is fixed for determinism, and a
// stiffness that moved with it would make the wing's own fabric depend on the
// frame rate.
//
// Fabric is not isotropic and the difference matters: ripstop is stiff along
// the warp and weft threads and much softer on the bias, and it is the bias
// that governs how a canopy wrinkles and where it folds. So the compliance of
// each constraint depends on the angle its segment makes to the weave.
//
// Scope, stated because the plan's Level 6 is larger than this: strips at
// chosen chord stations rather than a full two-dimensional mesh; ribs as fixed
// endpoints rather than as their own membranes; no self-collision, which the
// plan puts at Level 8 with cravats. What it does give is a section whose
// bulge, slack and stretch are solved from pressure, seam length and applied
// load rather than drawn - and which goes slack when the pressure leaves.

struct FabricMaterial
{
    // Membrane stiffness along the warp and weft threads, newtons per metre
    // of width per unit strain. Coated ripstop is stiff both ways.
    double warpStiffnessNPerM = 35000.0;
    double weftStiffnessNPerM = 30000.0;
    // On the bias the weave can shear, and the fabric is far softer. This is
    // the number that decides how a canopy folds.
    double biasStiffnessNPerM = 4000.0;
    // Angle of the warp threads relative to the chord, radians. Panels are
    // usually cut with the warp running spanwise, which puts the chordwise
    // direction on the weft.
    double warpAngleRad = 1.5708;
    // Resistance to bending. Fabric has almost none, which is why a canopy
    // creases rather than curving smoothly when it goes slack.
    double bendingComplianceRadPerNm = 0.02;
    // Damping applied to node velocities, per second.
    double dampingPerSecond = 6.0;
    double arealDensityKgM2 = 0.042;
};

struct MembraneSpec
{
    // Nodes across the cell, rib to rib.
    int nodeCount = 25;
    FabricMaterial fabric{};
    // Seam allowance, as Level 1 defines it: how much longer the cut panel is
    // than the straight rib-to-rib distance. Here it is what gives the
    // section slack to bulge into.
    double seamAllowanceFraction = 0.026;
    // Fixed iteration count. Determinism is guiding rule 10, and XPBD is what
    // makes a fixed count give a stiffness that means something.
    int constraintIterations = 24;
    int substeps = 4;
};

struct MembraneNode
{
    // Position in the cell's cross-plane: y across the cell from rib to rib,
    // z normal to the skin.
    Vec3 positionM{};
    Vec3 velocityMps{};
    double inverseMassKgInv = 0.0;
    // Fraction across the cell, 0 at one rib and 1 at the other.
    double acrossCell = 0.0;
    // Pinned nodes are the ribs themselves, which hold the profile.
    bool pinned = false;
};

struct MembraneLoad
{
    // Internal gauge pressure inside the cell, pascals. From Level 5.
    double internalPressurePa = 65.0;
    // External suction distribution is not resolved at this level; what the
    // section feels is the difference across the skin, and the aerodynamic
    // side of that is the dynamic pressure times the local pressure
    // coefficient. Supplying zero leaves the cell inflating against ambient.
    double externalDynamicPressurePa = 0.0;
    // Downward pull applied at the trailing edge by the brake line, newtons.
    double brakeLineForceN = 0.0;
    // Gravity, mostly negligible against the pressure but present.
    double gravityMps2 = 9.80665;
};

struct MembraneResult
{
    // Section shape after the solve.
    std::vector<Vec3> positionM;
    // Maximum tensile strain anywhere in the skin, and where.
    double maximumStrain = 0.0;
    int maximumStrainNode = 0;
    // Fraction of the skin carrying no tension at all. A pressurised section
    // has none; a collapsing one has a great deal, and this is what a fold
    // looks like before there is a fold.
    double slackFraction = 0.0;
    // How far the skin bulges above the straight rib-to-rib line, metres, and
    // as a fraction of the cell width. This is the ovalization Level 1 solves
    // statically, now arrived at dynamically and able to go away.
    double sagittaM = 0.0;
    double sagittaFraction = 0.0;
    // Total kinetic energy left in the skin. The stability gate is that this
    // does not grow.
    double kineticEnergyJ = 0.0;
    // Largest constraint violation left after the fixed iteration budget.
    double constraintResidualM = 0.0;
};

class CanopyMembraneSolver
{
public:
    // Builds a strip across one cell: the rib-to-rib spacing, and the seam
    // allowance the panel was cut with.
    explicit CanopyMembraneSolver(
        double cellWidthM, const MembraneSpec& spec = {});

    // Advances the membrane. Deterministic for a given input sequence: fixed
    // substeps, fixed iterations, no adaptive anything.
    MembraneResult Step(const MembraneLoad& load, double deltaSeconds);

    // Settles the section under a load, for callers that want the equilibrium
    // shape rather than the transient.
    MembraneResult Settle(const MembraneLoad& load, double seconds);

    const std::vector<MembraneNode>& Nodes() const { return NodeList; }
    const MembraneSpec& Spec() const { return SpecValue; }
    double CellWidthM() const { return CellWidth; }

private:
    struct Constraint
    {
        int nodeA = 0;
        int nodeB = 0;
        double restLengthM = 0.0;
        // Metres per newton. This is what makes the stiffness independent of
        // the iteration count.
        double complianceMPerN = 0.0;
        // Accumulated Lagrange multiplier, reset each substep.
        double multiplier = 0.0;
    };

    double ComplianceForDirection(const Vec3& direction, double lengthM) const;

    std::vector<MembraneNode> NodeList;
    std::vector<Constraint> Constraints;
    MembraneSpec SpecValue;
    double CellWidth = 0.0;
};
}
