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

// The two trailing legs of a horseshoe. Circulation arrives from downstream
// infinity into the bound vortex start and leaves from its end.
Vec3 TrailingInducedVelocity(
    const Vec3& p, const Vec3& boundStart, const Vec3& boundEnd,
    const Vec3& wakeDirection, double coreRadius)
{
    return SemiInfiniteInducedVelocity(p, boundEnd, wakeDirection, coreRadius)
        - SemiInfiniteInducedVelocity(p, boundStart, wakeDirection, coreRadius);
}
}

InstalledDragResult EvaluateInstalledDrag(
    const InstalledDragSpec& spec, const Vec3& airspeedBodyMps,
    double airDensityKgM3)
{
    InstalledDragResult result;
    const double speed = Length(airspeedBodyMps);
    if (speed < 1.0e-6) return result;
    const double dynamicPressure = 0.5 * airDensityKgM3 * speed * speed;

    const double lineArea = spec.lineTotalLengthM * spec.lineMeanDiameterM
        * spec.lineProjectedFraction;
    result.lineDragN =
        dynamicPressure * lineArea * spec.lineDragCoefficient;
    result.harnessDragN = dynamicPressure * spec.harnessAreaM2
        * spec.harnessDragCoefficient;
    result.totalDragN = result.lineDragN + result.harnessDragN;

    // The harness drag acts a long way below the canopy, so it pitches the
    // wing nose-down as well as slowing it. Lines are spread over that span,
    // so half the arm is the reasonable place to put their resultant.
    const Vec3 dragDirection = -Normalized(airspeedBodyMps);
    const Vec3 harnessArm{0.0, 0.0, -spec.harnessBelowCanopyM};
    const Vec3 lineArm{0.0, 0.0, -0.5 * spec.harnessBelowCanopyM};
    result.momentBodyNm =
        Cross(harnessArm, dragDirection * result.harnessDragN)
        + Cross(lineArm, dragDirection * result.lineDragN);
    return result;
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
        section.rightSideFraction = std::clamp(
            (rightFraction - 0.0) / std::max(1.0e-12,
                rightFraction - leftFraction), 0.0, 1.0);

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

        // On the lifting line. The three-quarter-chord placement a vortex
        // lattice uses buys nothing here: a trailing filament runs parallel to
        // the freestream, so moving the control point downstream does not
        // increase its distance from one at all - it only exposes more of the
        // filament's length. What it costs is real, because the section's own
        // legs then act on it more strongly.
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
        section.rightSideFraction = std::clamp(
            (rightFraction - 0.0) / std::max(1.0e-12,
                rightFraction - leftFraction), 0.0, 1.0);
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
            // Which parts of horseshoe j act on control point i, and why.
            //
            // TRAILING legs: always, including the section's own. The wake
            // carries only the spanwise CHANGE in circulation, so a panel's
            // legs must be there to cancel against its neighbours'. Drop them
            // and that cancellation goes one-sided.
            //
            // BOUND segment: every section except this one. A section's own
            // bound vortex is what its 2D polar already describes - the polar
            // IS the section's own circulation. Counting it again halves the
            // lift-curve slope, exactly and quietly.
            //
            const double core = coreFraction * SectionList[j].widthM;
            Vec3 velocity = TrailingInducedVelocity(
                SectionList[i].controlPointM,
                SectionList[j].boundStartM,
                SectionList[j].boundEndM,
                wakeDirection, core);
            if (i != j)
            {
                velocity += SegmentInducedVelocity(
                    SectionList[i].controlPointM,
                    SectionList[j].boundStartM,
                    SectionList[j].boundEndM);
            }
            Influence[i * count + j] = velocity;
        }
    }
}

VsmSolution VortexStepMethodSolver::SolveUnsteady(
    const VsmSolveInput& input, VsmSeparationState& state,
    double deltaSeconds, const VsmSettings& settings) const
{
    if (!state.initialised
        || state.sectionSeparation.size() != SectionList.size())
    {
        // Start from equilibrium so the first call is a steady answer rather
        // than a section-by-section transient from nowhere.
        const VsmSolution seed = Solve(input, settings);
        state.sectionSeparation.resize(SectionList.size());
        for (std::size_t i = 0; i < SectionList.size(); ++i)
            state.sectionSeparation[i] = seed.sections[i].separation;
        state.initialised = true;
    }

    const VsmSolution solution = SolveHeld(input, settings,
                                           &state.sectionSeparation);

    // Advance the state with the incidence this solve found. Separation
    // spreads faster than it clears.
    for (std::size_t i = 0; i < SectionList.size(); ++i)
    {
        const double brake =
            input.leftBrake * (1.0 - SectionList[i].rightSideFraction)
            + input.rightBrake * SectionList[i].rightSideFraction;
        const double target = Polars.SeparationEquilibrium(
            solution.sections[i].angleOfAttackRad, brake,
            state.sectionSeparation[i]);
        const double rate = target > state.sectionSeparation[i]
            ? settings.separationOnsetRatePerS
            : settings.reattachmentRatePerS;
        const double step = std::clamp(rate * deltaSeconds, 0.0, 1.0);
        state.sectionSeparation[i] +=
            (target - state.sectionSeparation[i]) * step;
    }
    return solution;
}

VsmSolution VortexStepMethodSolver::Solve(
    const VsmSolveInput& input, const VsmSettings& settings) const
{
    return SolveHeld(input, settings, nullptr);
}

VsmSolution VortexStepMethodSolver::SolveHeld(
    const VsmSolveInput& input, const VsmSettings& settings,
    const std::vector<double>* heldSeparation) const
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

    // Angle of attack and speed a section sees for a given circulation of its
    // own, with every other section's contribution held fixed.
    const auto sectionFlow = [&](std::size_t i, const Vec3& external,
                                 double gamma, double& speedOut)
    {
        const VsmSection& section = SectionList[i];
        const Vec3 localFlow =
            external + Influence[i * count + i] * gamma;
        const Vec3 inPlane = localFlow
            - section.spanDirection * Dot(localFlow, section.spanDirection);
        speedOut = Length(inPlane);
        if (speedOut < 1.0e-6) return 0.0;
        const Vec3 sectionVelocity = -inPlane;
        return std::atan2(-Dot(sectionVelocity, section.normal),
                          Dot(sectionVelocity, section.chordDirection));
    };

    // Under-relaxation, adapted as it goes. Away from stall the polar is
    // smooth and the coupling between sections is weak, so a large step is
    // safe. On the stall knee the lift curve turns over within a fraction of a
    // degree and a step that was fine one iteration diverges the next. Backing
    // off when the residual grows, and easing back up when it falls, converges
    // both cases without either being tuned for.
    double relaxation = std::clamp(settings.relaxation, 0.01, 1.0);
    double previousResidual = 1.0e30;

    for (int iteration = 0; iteration < settings.maxIterations; ++iteration)
    {
        double largestChange = 0.0;
        double largestCirculation = 1.0e-9;

        for (std::size_t i = 0; i < count; ++i)
        {
            const VsmSection& section = SectionList[i];

            // Everything except this section's own circulation.
            Vec3 external{};
            for (std::size_t j = 0; j < count; ++j)
            {
                if (j == i) continue;
                external += Influence[i * count + j] * circulation[j];
            }
            const Vec3 rotational = Cross(
                input.angularVelocityBodyRadps, section.controlPointM);
            external += freestream - rotational;

            const double brake =
                input.leftBrake * (1.0 - section.rightSideFraction)
                + input.rightBrake * section.rightSideFraction;
            const double cellPressure =
                input.internalPressureCoefficient.empty() ? 1.0
                : input.internalPressureCoefficient[
                      std::min(i, input.internalPressureCoefficient.size() - 1)];

            // Solve this section's own circulation implicitly.
            //
            // A section's trailing legs pass half a panel width from its own
            // control point, so the downwash they induce on it grows as the
            // mesh is refined: the gain of the obvious fixed-point iteration
            // is chord over panel width, which passes one as soon as panels
            // are narrower than the chord. Under-relaxing cannot fix that,
            // because the gain keeps rising with panel count - the answer
            // would depend on the mesh, which is the one thing it must not.
            //
            // Taking the self term implicitly removes it from the iteration
            // entirely. What is left for the outer loop is the coupling
            // BETWEEN sections, which is weak and converges quickly.
            const auto residualAt = [&](double gamma)
            {
                double speed = 0.0;
                const double alpha = sectionFlow(i, external, gamma, speed);
                const SectionPolarSample polar = heldSeparation
                    ? Polars.SampleAtSeparation(
                          alpha, brake, (*heldSeparation)[i], cellPressure)
                    : Polars.SampleAtEquilibrium(alpha, brake, cellPressure);
                return 0.5 * section.chordM * speed * polar.liftCoefficient
                    - gamma;
            };

            double gamma = circulation[i];
            double residual = residualAt(gamma);
            // Secant iteration. The relation is smooth away from stall and
            // the starting point is last step's answer, so this converges in
            // a handful of steps; the cap is there for the stalled case where
            // the polar has a knee in it.
            double previousGamma = gamma + (std::fabs(gamma) > 1.0e-6
                ? 0.01 * gamma : 0.01);
            double lastResidual = residualAt(previousGamma);
            for (int inner = 0; inner < 12; ++inner)
            {
                const double denominator = residual - lastResidual;
                if (std::fabs(denominator) < 1.0e-14) break;
                const double step =
                    residual * (gamma - previousGamma) / denominator;
                previousGamma = gamma;
                lastResidual = residual;
                gamma -= step;
                residual = residualAt(gamma);
                if (std::fabs(residual) < 1.0e-10) break;
            }
            if (!std::isfinite(gamma)) gamma = circulation[i];
            nextCirculation[i] = gamma;

            double speed = 0.0;
            const double alpha = sectionFlow(i, external, gamma, speed);
            const SectionPolarSample polar = heldSeparation
                ? Polars.SampleAtSeparation(
                      alpha, brake, (*heldSeparation)[i], cellPressure)
                : Polars.SampleAtEquilibrium(alpha, brake, cellPressure);
            solution.sections[i].separation = heldSeparation
                ? (*heldSeparation)[i]
                : Polars.SeparationEquilibrium(alpha, brake, 0.0);
            solution.sections[i].angleOfAttackRad = alpha;
            solution.sections[i].liftCoefficient = polar.liftCoefficient;
            solution.sections[i].dragCoefficient = polar.dragCoefficient;
            const Vec3 freeInPlane = -(freestream
                - section.spanDirection
                    * Dot(freestream, section.spanDirection));
            solution.sections[i].inducedAngleRad = alpha - std::atan2(
                -Dot(freeInPlane, section.normal),
                Dot(freeInPlane, section.chordDirection));
        }

        for (std::size_t i = 0; i < count; ++i)
        {
            // The residual is how far the circulation is from satisfying its
            // own equation, measured BEFORE relaxation. Measuring the relaxed
            // step instead makes the number shrink whenever the damping is
            // increased, so a solve that has merely been damped to a crawl
            // reports itself converged.
            largestChange = std::max(largestChange,
                std::fabs(nextCirculation[i] - circulation[i]));
            circulation[i] += relaxation * (nextCirculation[i] - circulation[i]);
            largestCirculation =
                std::max(largestCirculation, std::fabs(circulation[i]));
        }

        solution.iterations = iteration + 1;
        solution.residual = largestChange / largestCirculation;
        if (solution.residual < settings.convergenceTolerance)
        {
            solution.converged = true;
            break;
        }

        if (solution.residual > previousResidual)
            relaxation = std::max(settings.minimumRelaxation,
                                  0.5 * relaxation);
        else
            relaxation = std::min(settings.relaxation, 1.08 * relaxation);
        previousResidual = solution.residual;
    }
    solution.finalRelaxation = relaxation;

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
