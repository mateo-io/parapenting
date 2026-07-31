#include "TensionCableSolver.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
int RiserIndexFor(LineRow row)
{
    switch (row)
    {
    case LineRow::A: return 0;
    case LineRow::ABaby: return 1;
    case LineRow::B: return 2;
    case LineRow::C: return 3;
    case LineRow::Brake: return 3;
    }
    return 0;
}

// Angle of the root chord above the horizontal, nose-up positive.
double IncidenceOf(const Quaternion& attitude)
{
    const Vec3 chord = attitude.Rotate({1.0, 0.0, 0.0});
    return std::asin(std::clamp(chord.z, -1.0, 1.0));
}

// Bank angle, right wing down positive.
double BankOf(const Quaternion& attitude)
{
    const Vec3 span = attitude.Rotate({0.0, 1.0, 0.0});
    return std::asin(std::clamp(-span.z, -1.0, 1.0));
}

Quaternion Integrate(const Quaternion& q, const Vec3& angularVelocity, double dt)
{
    const Quaternion omega{0.0, angularVelocity.x, angularVelocity.y,
                           angularVelocity.z};
    const Quaternion derivative = omega * q;
    const Quaternion stepped{
        q.w + 0.5 * dt * derivative.w,
        q.x + 0.5 * dt * derivative.x,
        q.y + 0.5 * dt * derivative.y,
        q.z + 0.5 * dt * derivative.z};
    return stepped.Normalized();
}

bool AnchoredToPayload(SuspensionNodeKind kind)
{
    return kind == SuspensionNodeKind::Carabiner
        || kind == SuspensionNodeKind::RiserTop
        || kind == SuspensionNodeKind::BrakeHandle;
}
}

SuspensionSolution SolveSuspension(
    const SuspensionGraph& graph, const SuspensionSolveInput& input,
    const SuspensionSolverSettings& settings)
{
    const std::size_t nodeCount = graph.nodes.size();
    const std::size_t cableCount = graph.elements.size();
    const LinePlanSpec& plan = graph.plan;

    SuspensionSolution solution;
    solution.nodePositionM.assign(nodeCount, Vec3{});
    solution.cables.resize(cableCount);
    if (nodeCount == 0 || cableCount == 0) return solution;

    // ---------------------------------------------------------------------
    // Anchors. Bar shortens the forward risers and weight shift swings the
    // carabiner pair; neither writes a moment anywhere.
    // ---------------------------------------------------------------------
    const double accelerator = std::clamp(input.accelerator, 0.0, 1.0);
    const double shift = std::clamp(input.weightShift, -1.0, 1.0);
    const double shiftY = shift * plan.weightShiftTravelM;

    std::vector<Vec3> position(nodeCount);
    std::vector<Vec3> velocity(nodeCount, Vec3{});
    std::vector<Vec3> force(nodeCount, Vec3{});
    std::vector<char> isFree(nodeCount, 0);

    for (std::size_t i = 0; i < nodeCount; ++i)
    {
        const SuspensionNode& node = graph.nodes[i];
        if (AnchoredToPayload(node.kind))
        {
            Vec3 anchor = node.payloadLocalM;
            anchor.y += shiftY;
            if (node.kind == SuspensionNodeKind::RiserTop)
            {
                const int riser = RiserIndexFor(node.row);
                anchor.z = plan.trimRiserLengthM[riser]
                    + accelerator * (plan.acceleratedRiserLengthM[riser]
                                     - plan.trimRiserLengthM[riser]);
            }
            // The harness rolls with the shift: the loaded side drops.
            anchor.z -= shift * node.side * plan.weightShiftTiltM;
            position[i] = anchor;
        }
        else if (node.kind == SuspensionNodeKind::CascadeJunction)
        {
            position[i] = node.designM;
            isFree[i] = 1;
        }
        else
        {
            position[i] = node.designM;  // rewritten from the canopy pose
        }
    }

    // ---------------------------------------------------------------------
    // Rest lengths. Brake input shortens the brake main run; it does not move
    // the handle, so hands-up slack stays slack and a released brake stops
    // transmitting immediately.
    // ---------------------------------------------------------------------
    std::vector<double> restLength(cableCount);
    for (std::size_t c = 0; c < cableCount; ++c)
    {
        const CableElement& cable = graph.elements[c];
        double rest = cable.restLengthM;
        if (cable.row == LineRow::Brake && cable.cascadeLevel == 0)
        {
            const double side =
                graph.nodes[static_cast<std::size_t>(cable.nodeA)].side;
            const double brake = std::clamp(
                side < 0.0 ? input.leftBrake : input.rightBrake, 0.0, 1.0);
            rest -= brake * plan.brakeTravelM;
        }
        restLength[c] = std::max(0.05, rest);
    }

    // ---------------------------------------------------------------------
    // Applied load. The aerodynamic resultant is split between the two half
    // wings and applied at their centres of pressure, so a spanwise asymmetry
    // reaches the lines as geometry rather than as a roll moment.
    // ---------------------------------------------------------------------
    const double asymmetry = std::clamp(input.spanwiseLoadAsymmetry, -0.95, 0.95);
    const double leftShare = 0.5 * (1.0 - asymmetry);
    const double rightShare = 0.5 * (1.0 + asymmetry);
    const Vec3 leftCentreLocal = CanopyPointLocalM(
        graph, -input.aeroCentreSpanOffset, input.aeroCentreChordFraction);
    const Vec3 rightCentreLocal = CanopyPointLocalM(
        graph, input.aeroCentreSpanOffset, input.aeroCentreChordFraction);
    const Vec3 canopyWeightN{0.0, 0.0, -std::max(0.0, input.canopyWeightN)};
    const double gravity = input.gravityMps2;

    Vec3 canopyOrigin = graph.canopyDesignOriginM;
    Quaternion canopyAttitude = NoseUpAttitude(graph.designIncidenceRad);
    Vec3 canopyVelocity{};
    Vec3 canopyAngularVelocity{};

    const double dt = settings.timeStepS;
    const double retention = std::clamp(settings.velocityRetention, 0.0, 1.0);
    const double nodeMass = std::max(1e-4, settings.nodeSolverMassKg);
    const double canopyMass = std::max(1e-3, settings.canopySolverMassKg);
    const double canopyInertia = std::max(1e-3, settings.canopySolverInertiaKgM2);

    for (int iteration = 0; iteration < settings.iterations; ++iteration)
    {
        // Canopy attachments follow the canopy pose exactly. There is one
        // wing, so this is a lookup and not a second description of it.
        for (std::size_t i = 0; i < nodeCount; ++i)
        {
            const SuspensionNode& node = graph.nodes[i];
            if (node.kind != SuspensionNodeKind::CanopyAttachment) continue;
            position[i] =
                canopyOrigin + canopyAttitude.Rotate(node.canopyLocalM);
            velocity[i] = canopyVelocity + Cross(
                canopyAngularVelocity, position[i] - canopyOrigin);
        }

        std::fill(force.begin(), force.end(), Vec3{});
        Vec3 canopyForce = canopyWeightN;
        Vec3 canopyMoment{};

        const Vec3 aeroWorld = input.aeroForceN;
        const Vec3 leftArm = canopyAttitude.Rotate(leftCentreLocal);
        const Vec3 rightArm = canopyAttitude.Rotate(rightCentreLocal);
        canopyForce += aeroWorld * leftShare;
        canopyForce += aeroWorld * rightShare;
        canopyMoment += Cross(leftArm, aeroWorld * leftShare);
        canopyMoment += Cross(rightArm, aeroWorld * rightShare);

        for (std::size_t c = 0; c < cableCount; ++c)
        {
            const CableElement& cable = graph.elements[c];
            const auto a = static_cast<std::size_t>(cable.nodeA);
            const auto b = static_cast<std::size_t>(cable.nodeB);
            const Vec3 delta = position[b] - position[a];
            const double length = Length(delta);

            // Line weight, half at each end. This is what gives the cascade
            // junctions their droop, so the drawn sag is solved rather than
            // invented.
            const Vec3 halfWeight{0.0, 0.0, -0.5 * cable.massKg * gravity};
            force[a] += halfWeight;
            force[b] += halfWeight;

            double tension = 0.0;
            if (length > restLength[c] && length > 1e-9)
            {
                const double strain =
                    (length - restLength[c]) / restLength[c];
                tension = cable.axialStiffnessN * strain;
                const Vec3 axis = delta / length;
                const double damping = 2.0 * settings.cableDampingRatio
                    * std::sqrt(cable.axialStiffnessN / restLength[c] * nodeMass);
                tension += damping
                    * Dot(velocity[b] - velocity[a], axis);
                // Damping may only remove energy from a taut line; it can
                // never push one.
                tension = std::max(0.0, tension);
                const Vec3 pull = axis * tension;
                force[a] += pull;
                force[b] += -pull;
            }

        }

        // Attachment loads become canopy loads. Equal and opposite: the same
        // vector the line applied to the node is what the canopy feels.
        for (std::size_t i = 0; i < nodeCount; ++i)
        {
            if (graph.nodes[i].kind != SuspensionNodeKind::CanopyAttachment)
                continue;
            canopyForce += force[i];
            canopyMoment += Cross(position[i] - canopyOrigin, force[i]);
        }

        for (std::size_t i = 0; i < nodeCount; ++i)
        {
            if (!isFree[i]) continue;
            velocity[i] = (velocity[i] + force[i] / nodeMass * dt) * retention;
            position[i] += velocity[i] * dt;
        }

        canopyVelocity =
            (canopyVelocity + canopyForce / canopyMass * dt) * retention;
        canopyAngularVelocity =
            (canopyAngularVelocity + canopyMoment / canopyInertia * dt)
                * retention;
        canopyOrigin += canopyVelocity * dt;
        canopyAttitude = Integrate(canopyAttitude, canopyAngularVelocity, dt);

    }

    // ---------------------------------------------------------------------
    // Report. Everything below reads the converged state; nothing is fitted.
    // ---------------------------------------------------------------------
    for (std::size_t i = 0; i < nodeCount; ++i)
    {
        const SuspensionNode& node = graph.nodes[i];
        if (node.kind == SuspensionNodeKind::CanopyAttachment)
            position[i] =
                canopyOrigin + canopyAttitude.Rotate(node.canopyLocalM);
    }
    solution.nodePositionM = position;
    solution.canopyOriginM = canopyOrigin;
    solution.canopyAttitude = canopyAttitude;
    solution.canopyPitchRad = IncidenceOf(canopyAttitude);
    solution.canopyRollRad = BankOf(canopyAttitude);
    solution.incidenceChangeRad =
        solution.canopyPitchRad - graph.designIncidenceRad;

    std::vector<Vec3> residual(nodeCount, Vec3{});
    double mainStrainSum = 0.0;
    int mainStrainCount = 0;
    for (std::size_t c = 0; c < cableCount; ++c)
    {
        const CableElement& cable = graph.elements[c];
        const auto a = static_cast<std::size_t>(cable.nodeA);
        const auto b = static_cast<std::size_t>(cable.nodeB);
        const Vec3 delta = position[b] - position[a];
        const double length = Length(delta);

        CableState& state = solution.cables[c];
        state.nodeA = cable.nodeA;
        state.nodeB = cable.nodeB;
        state.row = cable.row;
        state.cascadeLevel = cable.cascadeLevel;
        state.lengthM = length;
        state.restLengthM = restLength[c];
        state.slack = length <= restLength[c];
        state.strain = state.slack
            ? 0.0 : (length - restLength[c]) / restLength[c];
        // Static tension only: the damping term is a solver device and has no
        // business in reported line load.
        state.tensionN = cable.axialStiffnessN * state.strain;
        if (state.slack) ++solution.slackCableCount;

        const Vec3 halfWeight{0.0, 0.0, -0.5 * cable.massKg * gravity};
        residual[a] += halfWeight;
        residual[b] += halfWeight;
        if (!state.slack && length > 1e-9)
        {
            const Vec3 pull = delta / length * state.tensionN;
            residual[a] += pull;
            residual[b] += -pull;
            solution.endpointForceSumN += pull;
            solution.endpointForceSumN += -pull;

            const double side = graph.nodes[a].side;
            const auto row = static_cast<std::size_t>(cable.row);
            if (cable.cascadeLevel == 0)
            {
                solution.rowTensionN[row] += state.tensionN;
                if (side < 0.0) solution.leftRowTensionN[row] += state.tensionN;
                else solution.rightRowTensionN[row] += state.tensionN;
                solution.totalTensionN += state.tensionN;
                if (cable.row != LineRow::Brake)
                {
                    mainStrainSum += state.strain;
                    ++mainStrainCount;
                }
            }
            solution.maxTensionN =
                std::max(solution.maxTensionN, state.tensionN);
            solution.maxStrain = std::max(solution.maxStrain, state.strain);

            // Anchors: a riser top's reaction is carried by its carabiner,
            // and a brake handle's by the pilot's hand through the same side.
            if (AnchoredToPayload(graph.nodes[a].kind))
            {
                if (graph.nodes[a].side < 0.0)
                    solution.leftCarabinerForceN += pull;
                else
                    solution.rightCarabinerForceN += pull;
            }
        }
    }

    solution.meanMainStrain = mainStrainCount > 0
        ? mainStrainSum / static_cast<double>(mainStrainCount) : 0.0;
    solution.lineStretchM = solution.meanMainStrain * plan.canopyToRiserM;
    solution.leftCarabinerLoadN = Length(solution.leftCarabinerForceN);
    solution.rightCarabinerLoadN = Length(solution.rightCarabinerForceN);
    const double carabinerTotal =
        solution.leftCarabinerLoadN + solution.rightCarabinerLoadN;
    solution.lateralLoadImbalance = carabinerTotal > 1e-9
        ? (solution.rightCarabinerLoadN - solution.leftCarabinerLoadN)
            / carabinerTotal
        : 0.0;
    if (solution.totalTensionN > 1e-9)
    {
        for (int row = 0; row < LineRowCount; ++row)
            solution.rowLoadFraction[row] =
                solution.rowTensionN[row] / solution.totalTensionN;
    }

    for (std::size_t i = 0; i < nodeCount; ++i)
    {
        if (!isFree[i]) continue;
        solution.maxNodeResidualN =
            std::max(solution.maxNodeResidualN, Length(residual[i]));
    }

    // Canopy residual from the converged state, not from the last iteration:
    // the solver's damping term would otherwise flatter the number it is being
    // judged by.
    Vec3 canopyForce = canopyWeightN + input.aeroForceN;
    Vec3 canopyMoment =
        Cross(canopyAttitude.Rotate(leftCentreLocal), input.aeroForceN * leftShare)
        + Cross(canopyAttitude.Rotate(rightCentreLocal),
                input.aeroForceN * rightShare);
    for (std::size_t i = 0; i < nodeCount; ++i)
    {
        if (graph.nodes[i].kind != SuspensionNodeKind::CanopyAttachment) continue;
        canopyForce += residual[i];
        canopyMoment += Cross(position[i] - canopyOrigin, residual[i]);
    }
    solution.canopyForceResidualN = Length(canopyForce);
    solution.canopyMomentResidualNm = Length(canopyMoment);
    return solution;
}
}
