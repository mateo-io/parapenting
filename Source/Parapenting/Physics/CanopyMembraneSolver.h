#pragma once

#include "ParagliderDynamics.h"

#include <vector>

namespace Parapenting::Physics
{
// Level 6 of the master plan: the skin is fabric, not a surface.
//
// It is checked against the analytic answer rather than against itself. A
// chain of cut length L pinned across a gap c under uniform pressure settles
// as a circular arc: t/sin t = L/c fixes the shape, the sagitta is
// R(1 - cos t), the hoop tension is p R and the fabric stretches by that over
// its membrane stiffness. For this wing that is 25.99 mm and 0.064% strain,
// and the solver gives 26.32 mm and 0.060% - a little deeper because the
// fabric really does stretch.
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
// chosen chord stations rather than a full two-dimensional mesh; ribs as their
// own endpoints rather than their own membranes. What it does give is a section
// whose bulge, slack, fold depth and stretch are solved from pressure, seam
// length, rib spacing and applied load rather than drawn - and which goes slack
// when the pressure leaves.
//
// SELF-COLLISION IS NOT HERE, AND CANNOT BE USEFULLY. The plan puts fabric
// self-collision in this object at Level 8, for cravats. It was built, and then
// removed, because a one-dimensional strip cannot self-intersect: a chain under
// a transverse load with both ends pinned settles as a simple curve. That was
// measured rather than argued - the strip was collapsed with the ribs drawn
// from full spacing down to a tenth of it, and the segment-crossing count was
// zero at every step of the way, with the fold merely deepening from 31 mm to
// 121 mm. Collision code on this object can never fire.
//
// Crumpling needs loads that reverse along the skin, which needs the
// two-dimensional mesh this level does not have. A cravat, meanwhile, is
// fabric caught on a LINE rather than on itself, and Level 8 builds it from
// this strip's fold depth against the suspension geometry - which is real
// contact between two things that both exist.

struct FabricMaterial
{
    // Membrane stiffness along the warp and weft threads, newtons per metre
    // of width per unit strain. Coated ripstop is stiff both ways.
    double warpStiffnessNPerM = 35000.0;
    double weftStiffnessNPerM = 30000.0;
    // On the bias the weave can shear, and the fabric is far softer. This is
    // the number that decides how a canopy folds.
    double biasStiffnessNPerM = 4000.0;
    // Angle of the warp threads from the chord axis, radians. A panel cut
    // with the warp running spanwise is at 90 degrees, which is the usual
    // cut and puts a spanwise strip along the warp. Cut at 45 degrees the
    // same strip is on the bias and far softer - which is the whole reason
    // the angle is a parameter rather than an assumption.
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
    // Fictitious extra node mass, as a multiple of the fabric's own.
    //
    // Ripstop is very stiff against very little mass: the constraint
    // stiffness is 2.7e6 N/m against 0.46 g per node, which puts the fabric's
    // own elastic waves at 12 kHz. Those are of no interest at flight
    // timescales, but resolving them is what the solve is spending itself on.
    // Each substep pushes a node by F h^2 / 2m before the constraints pull it
    // back, so that push falls with mass exactly as it falls with substep
    // count - and mass is free where substeps are not.
    //
    // This is a solver device, like the Level 2 relaxation masses. It changes
    // how fast the skin settles, not where it settles, and the shape is
    // checked against the analytic arc rather than against itself.
    double solverMassScale = 1.0e4;

};

struct MembraneNode
{
    // Position in the cell's cross-plane: y across the cell from rib to rib,
    // z normal to the skin.
    Vec3 positionM{};
    Vec3 velocityMps{};
    double inverseMassKgInv = 0.0;
    // The fabric's own mass, before the solver's scaling. Physical forces are
    // computed from this, never from the scaled mass - a numerical device
    // that changed how hard gravity pulled would not be a device.
    double physicalMassKg = 0.0;
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
    // How far apart the two ribs holding this strip are, as a fraction of the
    // design spacing. One is the flying wing.
    //
    // This is what makes a fold possible at all, and it took a failing control
    // to notice. The strip is pinned to its ribs, and a panel cut 2.6% longer
    // than the gap it spans is very nearly taut - and a taut chain between two
    // fixed points cannot cross itself, however hard it is pushed. So with the
    // ribs held at design spacing, self-collision is inert: there is no fold
    // for it to resolve.
    //
    // A collapsing section is not a section with the same ribs and less
    // pressure. The cell loses its air, the shell loses the pressure that was
    // holding it open, and the ribs are drawn together by the fabric between
    // them - which is exactly when the skin has length to spare and crumples
    // rather than bulging. That convergence is an input here rather than
    // something this strip could solve, because the ribs belong to the cell
    // and this is one station across it.
    double ribSeparationFraction = 1.0;
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
    // How deep the deepest fold is, metres below the rib-to-rib line. A bulge
    // is positive sagitta; a fold is this. It is what a collapsed section
    // hangs into, and Level 8 reads it to decide whether a folded tip reaches
    // the lines.
    double foldDepthM = 0.0;
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

    // Compliance of a spanwise segment, from where the span sits relative to
    // the weave. The strip runs along the span, so the angle that matters is
    // between the span and the warp - taking it from the segment's own
    // direction in the x-z plane, as this once did, gives atan2(0, 0) for
    // every segment and the weave never enters at all.
    double ComplianceForSpanwiseSegment(double lengthM) const;

    std::vector<MembraneNode> NodeList;
    std::vector<Constraint> Constraints;
    MembraneSpec SpecValue;
    double CellWidth = 0.0;
};
}
