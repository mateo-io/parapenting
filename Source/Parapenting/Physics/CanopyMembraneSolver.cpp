#include "CanopyMembraneSolver.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
CanopyMembraneSolver::CanopyMembraneSolver(
    double cellWidthM, const MembraneSpec& spec)
    : SpecValue(spec), CellWidth(std::max(0.01, cellWidthM))
{
    const int count = std::max(5, SpecValue.nodeCount);
    SpecValue.nodeCount = count;
    NodeList.resize(static_cast<std::size_t>(count));

    // Flat between the ribs to start with. The bulge is what the solve is for,
    // so starting flat means nothing about the answer has been drawn.
    for (int index = 0; index < count; ++index)
    {
        const double across = static_cast<double>(index)
            / static_cast<double>(count - 1);
        MembraneNode node;
        node.acrossCell = across;
        node.positionM = {0.0, across * CellWidth, 0.0};
        node.pinned = index == 0 || index == count - 1;
        NodeList[static_cast<std::size_t>(index)] = node;
    }

    const double segment = CellWidth / static_cast<double>(count - 1);
    const double physicalMass = SpecValue.fabric.arealDensityKgM2 * segment;
    const double solverMass =
        physicalMass * std::max(1.0, SpecValue.solverMassScale);
    for (MembraneNode& node : NodeList)
    {
        node.physicalMassKg = physicalMass;
        node.inverseMassKgInv =
            node.pinned ? 0.0 : 1.0 / std::max(1e-12, solverMass);
    }

    // The panel is cut longer than the gap it spans. That extra length is the
    // seam allowance, and it is the only reason the skin has anywhere to bulge
    // to - a panel cut exactly to the rib spacing would be a flat drum.
    for (int index = 0; index + 1 < count; ++index)
    {
        Constraint constraint;
        constraint.nodeA = index;
        constraint.nodeB = index + 1;
        constraint.restLengthM =
            segment * (1.0 + SpecValue.seamAllowanceFraction);
        constraint.complianceMPerN = ComplianceForSpanwiseSegment(segment);
        Constraints.push_back(constraint);
    }
}

double CanopyMembraneSolver::ComplianceForSpanwiseSegment(
    double lengthM) const
{
    // The strip runs spanwise, which is 90 degrees from the chord axis the
    // warp angle is measured against.
    constexpr double SpanFromChordRad = 1.5707963267948966;
    const double toWarp = SpanFromChordRad - SpecValue.fabric.warpAngleRad;
    // cos^2 picks out the warp, sin^2 the weft, and sin^2(2a) is largest on
    // the bias exactly halfway between them.
    const double alongWarp = std::cos(toWarp) * std::cos(toWarp);
    const double alongWeft = std::sin(toWarp) * std::sin(toWarp);
    const double onBias = std::sin(2.0 * toWarp) * std::sin(2.0 * toWarp);

    const double stiffness =
        alongWarp * SpecValue.fabric.warpStiffnessNPerM
        + alongWeft * SpecValue.fabric.weftStiffnessNPerM;
    // The bias is a shear mode, so it softens whatever the threads would have
    // given rather than adding to it.
    const double effective = stiffness
        * (1.0 - onBias)
        + onBias * SpecValue.fabric.biasStiffnessNPerM;

    // XPBD compliance is the inverse of stiffness, scaled by the rest length:
    // a longer segment stretches more for the same load.
    return std::max(1.0e-9, lengthM / std::max(1.0, effective));
}

MembraneResult CanopyMembraneSolver::Step(
    const MembraneLoad& load, double deltaSeconds)
{
    MembraneResult result;
    const int substeps = std::max(1, SpecValue.substeps);
    const double h = std::max(0.0, deltaSeconds) / static_cast<double>(substeps);
    const auto nodes = NodeList.size();
    const double segment = CellWidth / static_cast<double>(nodes - 1);

    // The ribs, wherever the cell is holding them this step. Moving them is
    // what gives the skin length to spare; the cut length of the fabric never
    // changes, which is why the strip crumples rather than shrinking.
    {
        const double separation =
            CellWidth * std::clamp(load.ribSeparationFraction, 0.05, 1.5);
        const double slackOffset = 0.5 * (CellWidth - separation);
        for (MembraneNode& node : NodeList)
        {
            if (node.inverseMassKgInv > 0.0) continue;
            node.positionM.y = slackOffset + node.acrossCell * separation;
        }
    }

    for (int substep = 0; substep < substeps; ++substep)
    {
        for (Constraint& constraint : Constraints) constraint.multiplier = 0.0;

        // Pressure pushes the skin outward, normal to itself, over the width
        // each node is responsible for. The strip is a metre deep in the
        // chordwise direction for the purpose of this per-metre load.
        std::vector<Vec3> forces(nodes, Vec3{});
        const double net =
            load.internalPressurePa - load.externalDynamicPressurePa;
        for (std::size_t index = 0; index + 1 < nodes; ++index)
        {
            const Vec3 edge =
                NodeList[index + 1].positionM - NodeList[index].positionM;
            const double edgeLength = Length(edge);
            if (edgeLength < 1.0e-9) continue;
            // Outward normal of the strip in the y-z plane.
            const Vec3 normal = Normalized(Vec3{0.0, -edge.z, edge.y});
            const Vec3 force = normal * (net * edgeLength * 0.5);
            forces[index] += force;
            forces[index + 1] += force;
        }

        // A brake line pulling on this station drags the skin down with it.
        if (load.brakeLineForceN != 0.0)
        {
            const std::size_t middle = nodes / 2;
            forces[middle].z -= load.brakeLineForceN;
        }

        std::vector<Vec3> previous(nodes);
        for (std::size_t index = 0; index < nodes; ++index)
        {
            MembraneNode& node = NodeList[index];
            previous[index] = node.positionM;
            if (node.inverseMassKgInv <= 0.0) continue;
            // Gravity as a force on the real fabric, then accelerated
            // against the solver mass like every other force. Subtracting g
            // from the acceleration directly would have applied it to the
            // scaled mass instead, and a 10^4 solver mass then sags the strip
            // under 10^4 times its own weight - which is exactly what it did.
            Vec3 total = forces[index];
            total.z -= node.physicalMassKg * load.gravityMps2;
            node.velocityMps += total * node.inverseMassKgInv * h;
            const double damping = std::clamp(
                1.0 - SpecValue.fabric.dampingPerSecond * h, 0.0, 1.0);
            node.velocityMps = node.velocityMps * damping;
            node.positionM += node.velocityMps * h;
        }

        // Fabric takes tension only. A segment shorter than its cut length is
        // slack and does nothing at all, which is what lets the skin wrinkle
        // instead of pushing itself straight.
        const double h2 = h * h;
        for (int iteration = 0; iteration < SpecValue.constraintIterations;
             ++iteration)
        {
            // Alternate the sweep direction. A Gauss-Seidel pass over a chain
            // carries information one segment per sweep in whichever
            // direction it runs, so sweeping one way only makes the far end
            // of the strip lag the near end by as many sweeps as there are
            // segments. Alternating propagates from both ribs at once.
            const bool forward = (iteration % 2) == 0;
            const int count = static_cast<int>(Constraints.size());
            for (int step = 0; step < count; ++step)
            {
                Constraint& constraint = Constraints[static_cast<std::size_t>(
                    forward ? step : count - 1 - step)];
                MembraneNode& a =
                    NodeList[static_cast<std::size_t>(constraint.nodeA)];
                MembraneNode& b =
                    NodeList[static_cast<std::size_t>(constraint.nodeB)];
                const double inverseMassSum =
                    a.inverseMassKgInv + b.inverseMassKgInv;
                if (inverseMassSum <= 0.0) continue;

                const Vec3 delta = b.positionM - a.positionM;
                const double length = Length(delta);
                if (length < 1.0e-9) continue;
                const double violation = length - constraint.restLengthM;
                if (violation <= 0.0) continue;

                const double alpha = constraint.complianceMPerN / h2;
                const double denominator = inverseMassSum + alpha;
                const double change =
                    -(violation + alpha * constraint.multiplier) / denominator;
                constraint.multiplier += change;

                const Vec3 correction = delta / length * change;
                a.positionM += correction * -a.inverseMassKgInv;
                b.positionM += correction * b.inverseMassKgInv;
            }

        }

        for (std::size_t index = 0; index < nodes; ++index)
        {
            if (NodeList[index].inverseMassKgInv <= 0.0) continue;
            NodeList[index].velocityMps =
                (NodeList[index].positionM - previous[index])
                    / std::max(1e-9, h);
        }
    }

    result.positionM.resize(nodes);
    double energy = 0.0;
    double peak = 0.0;
    double deepest = 0.0;
    for (std::size_t index = 0; index < nodes; ++index)
    {
        result.positionM[index] = NodeList[index].positionM;
        const double mass = NodeList[index].inverseMassKgInv > 0.0
            ? 1.0 / NodeList[index].inverseMassKgInv : 0.0;
        energy += 0.5 * mass * Dot(NodeList[index].velocityMps,
                                   NodeList[index].velocityMps);
        peak = std::max(peak, std::fabs(NodeList[index].positionM.z));
        deepest = std::min(deepest, NodeList[index].positionM.z);
    }
    result.kineticEnergyJ = energy;
    result.sagittaM = peak;
    result.sagittaFraction = peak / CellWidth;
    result.foldDepthM = -deepest;

    double slack = 0.0;
    for (const Constraint& constraint : Constraints)
    {
        const Vec3 delta =
            NodeList[static_cast<std::size_t>(constraint.nodeB)].positionM
            - NodeList[static_cast<std::size_t>(constraint.nodeA)].positionM;
        const double length = Length(delta);
        const double strain =
            (length - constraint.restLengthM) / constraint.restLengthM;
        if (strain <= 0.0) slack += 1.0;
        if (strain > result.maximumStrain)
        {
            result.maximumStrain = strain;
            result.maximumStrainNode = constraint.nodeA;
        }
        result.constraintResidualM = std::max(result.constraintResidualM,
            std::max(0.0, length - constraint.restLengthM));
    }
    result.slackFraction = Constraints.empty()
        ? 0.0 : slack / static_cast<double>(Constraints.size());
    (void)segment;
    return result;
}

MembraneResult CanopyMembraneSolver::Settle(
    const MembraneLoad& load, double seconds)
{
    MembraneResult result;
    const int steps = std::max(1, static_cast<int>(seconds * 120.0));
    for (int step = 0; step < steps; ++step)
        result = Step(load, 1.0 / 120.0);
    return result;
}
}
