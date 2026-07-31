#include "VortexStepMethodSolver.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double Pi = 3.14159265358979323846;
constexpr double FourPi = 4.0 * Pi;
// Below this the Biot-Savart kernel is singular. A filament has no thickness,
// so a control point on one is a division by zero rather than a large number.
constexpr double CoreRadiusSq = 1.0e-10;

// Velocity at P from a straight vortex segment A->B of unit circulation.
Vec3 SegmentInducedVelocity(const Vec3& p, const Vec3& a, const Vec3& b)
{
    const Vec3 r0 = b - a;
    const Vec3 r1 = p - a;
    const Vec3 r2 = p - b;
    const Vec3 crossed = Cross(r1, r2);
    const double crossedLengthSq = Dot(crossed, crossed);
    if (crossedLengthSq < CoreRadiusSq) return {};
    const double length1 = Length(r1);
    const double length2 = Length(r2);
    if (length1 < 1.0e-9 || length2 < 1.0e-9) return {};
    const double scalar = Dot(r0, r1 / length1 - r2 / length2);
    return crossed * (scalar / (FourPi * crossedLengthSq));
}

// Velocity at P from a filament running from `start` to infinity along
// `direction`, unit circulation. Half of an infinite filament's field, which
// is the check to make on this expression.
//
// `coreRadius` desingularises it. A filament of zero thickness induces an
// unbounded velocity on itself, and a control point sits only half a panel
// width from its own trailing legs, so the tip panels see an induced velocity
// that grows without limit as the mesh is refined - which is a property of the
// discretisation, not of the wing. Real trailing vorticity rolls up into a
// core of finite size, and the standard treatment is to give the kernel one.
// This is a numerical parameter and is declared as such: it is visible in
// VsmSettings, it is reported, and it must not be used to shape handling.
Vec3 SemiInfiniteInducedVelocity(
    const Vec3& p, const Vec3& start, const Vec3& direction,
    double coreRadius)
{
    const Vec3 r = p - start;
    const Vec3 crossed = Cross(direction, r);
    const double crossedLengthSq = Dot(crossed, crossed);
    if (crossedLengthSq < CoreRadiusSq) return {};
    const double length = Length(r);
    const double denominator = length * (length - Dot(direction, r));
    if (std::fabs(denominator) < 1.0e-12) return {};
    // Perpendicular distance to the filament.
    const double perpendicularSq = crossedLengthSq;
    const double coreSq = coreRadius * coreRadius;
    const double desingularised =
        perpendicularSq / (perpendicularSq + coreSq);
    return crossed * (desingularised / (FourPi * denominator));
}

// A horseshoe: circulation arrives from downstream infinity into the bound
// vortex start, crosses the bound segment, and leaves to downstream infinity.
Vec3 HorseshoeInducedVelocity(
    const Vec3& p, const Vec3& boundStart, const Vec3& boundEnd,
    const Vec3& wakeDirection, double coreRadius)
{
    return SemiInfiniteInducedVelocity(p, boundEnd, wakeDirection, coreRadius)
        - SemiInfiniteInducedVelocity(p, boundStart, wakeDirection, coreRadius)
        + SegmentInducedVelocity(p, boundStart, boundEnd);
}
}

VortexStepMethodSolver::VortexStepMethodSolver(
    const CanopyGeometry& geometry, SectionPolarTable polars, int sectionCount,
    double coreFraction)
    : Polars(std::move(polars))
{
    const int sections = std::max(4, sectionCount);
    SectionList.resize(static_cast<std::size_t>(sections));

    for (int index = 0; index < sections; ++index)
    {
        const double leftFraction = -1.0 + 2.0 * static_cast<double>(index)
            / static_cast<double>(sections);
        const double rightFraction = -1.0 + 2.0
            * static_cast<double>(index + 1) / static_cast<double>(sections);
        const double midFraction = 0.5 * (leftFraction + rightFraction);

        const RibStation left = geometry.StationAt(leftFraction);
        const RibStation right = geometry.StationAt(rightFraction);
        const RibStation mid = geometry.StationAt(midFraction);

        VsmSection section;
        // The bound vortex lies on the quarter-chord line, which is where the
        // rib station positions already are. Right end first: see VsmSection.
        section.boundStartM = right.positionM;
        section.boundEndM = left.positionM;
        section.chordM = mid.chordM;
        section.spanFraction = midFraction;

        // Chordwise is +X forward. The canopy arc rolls the section normal
        // outboard, and that anhedral is precisely what classical lifting
        // line cannot represent - it is the reason this solver exists.
        section.chordDirection = {1.0, 0.0, 0.0};
        const double arcAngle = mid.arcAngleRad;
        section.normal = Normalized(
            Vec3{0.0, std::sin(arcAngle), std::cos(arcAngle)});
        const Vec3 spanVector = section.boundEndM - section.boundStartM;
        section.widthM = Length(spanVector);
        section.spanDirection = Normalized(spanVector);
        section.areaM2 = section.chordM * section.widthM;

        section.controlPointM =
            (section.boundStartM + section.boundEndM) * 0.5;

        SectionList[static_cast<std::size_t>(index)] = section;
        ReferenceArea += section.areaM2;
    }
    ReferenceSpan = std::fabs(
        SectionList.back().boundStartM.y - SectionList.front().boundEndM.y);
    BuildInfluenceMatrix(coreFraction);
}

VortexStepMethodSolver VortexStepMethodSolver::FlatWing(
    double spanM, double rootChordM, bool elliptical,
    SectionPolarTable polars, int sectionCount, double coreFraction)
{
    VortexStepMethodSolver solver;
    solver.Polars = std::move(polars);
    const int sections = std::max(4, sectionCount);
    solver.SectionList.resize(static_cast<std::size_t>(sections));

    const auto chordAt = [&](double fraction)
    {
        if (!elliptical) return rootChordM;
        const double t = std::clamp(std::fabs(fraction), 0.0, 1.0);
        return rootChordM * std::sqrt(std::max(0.0, 1.0 - t * t));
    };

    for (int index = 0; index < sections; ++index)
    {
        // Cosine spacing: an elliptical wing's circulation changes fastest at
        // the tips, and uniform panels resolve that badly.
        const auto cosineFraction = [sections](int step)
        {
            const double theta = Pi * static_cast<double>(step)
                / static_cast<double>(sections);
            return -std::cos(theta);
        };
        const double leftFraction = cosineFraction(index);
        const double rightFraction = cosineFraction(index + 1);
        const double midFraction = 0.5 * (leftFraction + rightFraction);

        VsmSection section;
        section.boundStartM = {0.0, 0.5 * spanM * rightFraction, 0.0};
        section.boundEndM = {0.0, 0.5 * spanM * leftFraction, 0.0};
        section.chordM = chordAt(midFraction);
        section.spanFraction = midFraction;
        section.chordDirection = {1.0, 0.0, 0.0};
        section.normal = {0.0, 0.0, 1.0};
        const Vec3 spanVector = section.boundEndM - section.boundStartM;
        section.widthM = Length(spanVector);
        section.spanDirection = Normalized(spanVector);
        section.areaM2 = section.chordM * section.widthM;
        section.controlPointM =
            (section.boundStartM + section.boundEndM) * 0.5;

        solver.SectionList[static_cast<std::size_t>(index)] = section;
        solver.ReferenceArea += section.areaM2;
    }
    solver.ReferenceSpan = spanM;
    solver.BuildInfluenceMatrix(coreFraction);
    return solver;
}

void VortexStepMethodSolver::BuildInfluenceMatrix(double coreFraction)
{
    const std::size_t count = SectionList.size();
    Influence.assign(count * count, Vec3{});
    // The wake leaves downstream. In body axes the freestream arrives along
    // +X, so the wake trails toward -X. Holding it fixed is the steady
    // assumption; Level 11 lets it deform.
    const Vec3 wakeDirection{-1.0, 0.0, 0.0};
    for (std::size_t i = 0; i < count; ++i)
    {
        for (std::size_t j = 0; j < count; ++j)
        {
            // The self term stays. A panel's trailing legs must be present to
            // cancel against its neighbours' - the wake only carries the
            // spanwise CHANGE in circulation, and dropping a panel's own legs
            // leaves that cancellation one-sided and the induced velocity
            // enormous. Its bound segment contributes nothing at its own
            // midpoint, being collinear with it, so there is no double count
            // of the section's 2D lift to remove.
            // Core scaled to the panel it belongs to, so refining the mesh
            // refines the core with it and the answer stays a property of the
            // wing rather than of the panel count.
            const double core = coreFraction * SectionList[j].widthM;
            Influence[i * count + j] = HorseshoeInducedVelocity(
                SectionList[i].controlPointM,
                SectionList[j].boundStartM,
                SectionList[j].boundEndM,
                wakeDirection, core);
        }
    }
}

VsmSolution VortexStepMethodSolver::Solve(
    const VsmSolveInput& input, const VsmSettings& settings) const
{
    const std::size_t count = SectionList.size();
    VsmSolution solution;
    solution.sections.resize(count);
    if (count == 0) return solution;

    std::vector<double> circulation(count, 0.0);
    std::vector<double> nextCirculation(count, 0.0);

    const double dynamicPressureScale = 0.5 * input.airDensityKgM3;
    // The air's velocity relative to the wing. Converted here, once.
    const Vec3 freestream = -input.airspeedBodyMps;

    for (int iteration = 0; iteration < settings.maxIterations; ++iteration)
    {
        double largestChange = 0.0;
        double largestCirculation = 1.0e-9;

        for (std::size_t i = 0; i < count; ++i)
        {
            const VsmSection& section = SectionList[i];

            // Local inflow: freestream, the rotation of the wing about its own
            // axes, and the downwash every bound vortex induces here.
            Vec3 induced{};
            for (std::size_t j = 0; j < count; ++j)
                induced += Influence[i * count + j] * circulation[j];
            const Vec3 rotational = Cross(
                input.angularVelocityBodyRadps, section.controlPointM);
            const Vec3 localFlow = freestream - rotational + induced;

            // Work in the section plane: the component along the bound vortex
            // does not turn the section's flow, it sweeps along it.
            const Vec3 inPlane = localFlow
                - section.spanDirection * Dot(localFlow, section.spanDirection);
            const double inPlaneSpeed = Length(inPlane);
            if (inPlaneSpeed < 1.0e-6)
            {
                nextCirculation[i] = 0.0;
                continue;
            }

            // Incidence is measured on the section's own motion through the
            // air, which is the negative of the flow it sees.
            const Vec3 sectionVelocity = -inPlane;
            const double alongChord =
                Dot(sectionVelocity, section.chordDirection);
            const double alongNormal = Dot(sectionVelocity, section.normal);
            const double alpha = std::atan2(-alongNormal, alongChord);

            const double brake = section.spanFraction < 0.0
                ? input.leftBrake : input.rightBrake;
            const SectionPolarSample polar = Polars.Sample(alpha, brake);

            // Kutta-Joukowski against the section polar: the circulation that
            // reproduces the lift the 2D data says this section makes at the
            // incidence it is actually seeing.
            nextCirculation[i] =
                0.5 * section.chordM * inPlaneSpeed * polar.liftCoefficient;

            solution.sections[i].angleOfAttackRad = alpha;
            solution.sections[i].liftCoefficient = polar.liftCoefficient;
            solution.sections[i].dragCoefficient = polar.dragCoefficient;
            // Downwash relative to the undisturbed freestream direction.
            const Vec3 freeInPlane = -(freestream
                - section.spanDirection
                    * Dot(freestream, section.spanDirection));
            solution.sections[i].inducedAngleRad = alpha - std::atan2(
                -Dot(freeInPlane, section.normal),
                Dot(freeInPlane, section.chordDirection));
        }

        for (std::size_t i = 0; i < count; ++i)
        {
            const double blended = circulation[i]
                + settings.relaxation * (nextCirculation[i] - circulation[i]);
            largestChange = std::max(
                largestChange, std::fabs(blended - circulation[i]));
            circulation[i] = blended;
            largestCirculation =
                std::max(largestCirculation, std::fabs(blended));
        }

        solution.iterations = iteration + 1;
        solution.residual = largestChange / largestCirculation;
        if (solution.residual < settings.convergenceTolerance)
        {
            solution.converged = true;
            break;
        }
    }

    // Forces. Each section's lift acts perpendicular to its own local flow and
    // its drag along it, so induced drag appears as the tilt of the local lift
    // vector rather than as a formula.
    double inducedDragN = 0.0;
    double profileDragN = 0.0;
    for (std::size_t i = 0; i < count; ++i)
    {
        const VsmSection& section = SectionList[i];
        Vec3 induced{};
        for (std::size_t j = 0; j < count; ++j)
            induced += Influence[i * count + j] * circulation[j];
        const Vec3 rotational = Cross(
            input.angularVelocityBodyRadps, section.controlPointM);
        const Vec3 localFlow = freestream - rotational + induced;
        const Vec3 inPlane = localFlow
            - section.spanDirection * Dot(localFlow, section.spanDirection);
        const double inPlaneSpeed = Length(inPlane);
        if (inPlaneSpeed < 1.0e-6) continue;

        const Vec3 flowDirection = inPlane / inPlaneSpeed;
        // Lift is across the local flow. With the bound vortex running right
        // to left, this cross product comes out toward the upper surface.
        const Vec3 liftDirection =
            Normalized(Cross(flowDirection, section.spanDirection));
        const double dynamicPressure =
            dynamicPressureScale * inPlaneSpeed * inPlaneSpeed;

        const Vec3 lift = liftDirection
            * (dynamicPressure * section.areaM2
               * solution.sections[i].liftCoefficient);
        const Vec3 drag = flowDirection
            * (dynamicPressure * section.areaM2
               * solution.sections[i].dragCoefficient);

        // Induced drag is the streamwise component of the local lift, which
        // exists only because the local flow is tilted by the rest of the
        // wing. Drag is positive downstream, so it is a positive projection
        // onto the freestream direction.
        const Vec3 freestreamDirection = Normalized(freestream);
        inducedDragN += Dot(lift, freestreamDirection);
        profileDragN += Dot(drag, freestreamDirection);

        solution.sections[i].circulation = circulation[i];
        solution.sections[i].forceBodyN = lift + drag;
        solution.forceBodyN += solution.sections[i].forceBodyN;
        solution.momentBodyNm += Cross(
            (section.boundStartM + section.boundEndM) * 0.5,
            solution.sections[i].forceBodyN);
    }

    const double freestreamSpeed = Length(freestream);
    const double referenceForce = dynamicPressureScale * freestreamSpeed
        * freestreamSpeed * std::max(1.0e-6, ReferenceArea);
    if (referenceForce > 1.0e-9)
    {
        const Vec3 freestreamDirection = Normalized(freestream);
        // Lift is what acts across the freestream, drag along it.
        const Vec3 liftComponent = solution.forceBodyN
            - freestreamDirection * Dot(solution.forceBodyN,
                                        freestreamDirection);
        solution.liftCoefficient = Length(liftComponent) / referenceForce
            * (Dot(liftComponent, Vec3{0.0, 0.0, 1.0}) < 0.0 ? -1.0 : 1.0);
        solution.inducedDragCoefficient = inducedDragN / referenceForce;
        solution.profileDragCoefficient = profileDragN / referenceForce;
        solution.totalDragCoefficient =
            solution.inducedDragCoefficient + solution.profileDragCoefficient;
    }
    return solution;
}
}
