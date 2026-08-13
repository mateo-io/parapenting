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

    // Measured area times the one stated allowance, when the caller has built
    // a graph to measure; the three-number legacy path when it has not.
    const double lineArea = spec.lineProjectedAreaM2 > 0.0
        ? spec.lineProjectedAreaM2 * spec.lineShieldingFactor
        : spec.lineTotalLengthM * spec.lineMeanDiameterM
            * spec.lineProjectedFraction;
    result.lineDragN =
        dynamicPressure * lineArea * spec.lineDragCoefficient;
    // NOTE THE SHAPE OF THIS. The extra area is added in a SEPARATE statement,
    // guarded, rather than folded into the product - because `q * A * Cd` and
    // `q * (A * Cd + 0)` are not the same double. Written the folded way, this
    // line changed nothing physically and still failed a coupled check: the
    // deep symmetric frontal is a partly separated solve with no steady state,
    // so a last-bit difference in the harness drag walks into a different fold
    // path. A hook that defaults to off has to be bit-identical when it is off,
    // and reassociating a product is not bit-identical.
    //
    // The split below keeps that promise a second time, and it is why the two
    // `if`s are shaped the way they are rather than as one expression: at the
    // default fraction of 1 the pilot's share IS the whole extra, the second
    // `if` is skipped, and both statements reduce to exactly the arithmetic
    // that produced sections 56 and 57. Anything else would silently re-derive
    // those numbers.
    const double extraDragN = spec.extraDragAreaM2 != 0.0
        ? dynamicPressure * spec.extraDragAreaM2 : 0.0;
    const double extraAtPilotN = extraDragN * spec.extraDragAtPilotFraction;
    result.harnessDragN = dynamicPressure * spec.harnessAreaM2
        * spec.harnessDragCoefficient;
    if (extraAtPilotN != 0.0) result.harnessDragN += extraAtPilotN;
    result.totalDragN = result.lineDragN + result.harnessDragN;
    // The share that acts at the canopy still slows the aircraft; it just does
    // not push the bob, and its arm about the canopy is zero so it makes no
    // moment either.
    if (extraDragN != extraAtPilotN)
        result.totalDragN += extraDragN - extraAtPilotN;

    // The harness drag acts a long way below the canopy, so it pitches the
    // wing nose-down as well as slowing it. Lines are spread over that span,
    // so half the arm is the reasonable place to put their resultant.
    const Vec3 dragDirection = -Normalized(airspeedBodyMps);
    const Vec3 harnessArm{0.0, 0.0, -spec.harnessBelowCanopyM};
    const Vec3 lineArm{0.0, 0.0, -0.5 * spec.harnessBelowCanopyM};
    result.harnessMomentBodyNm =
        Cross(harnessArm, dragDirection * result.harnessDragN);
    result.lineMomentBodyNm =
        Cross(lineArm, dragDirection * result.lineDragN);
    result.momentBodyNm =
        result.harnessMomentBodyNm + result.lineMomentBodyNm;
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

        // Strand 2 needs the CIRCULATION seeded here too, and only strand 2:
        // seeding it unconditionally would change the quasi-steady path's
        // first solve from a cold start to a warm one, which converges to the
        // same wing but not to the same bits.
        //
        // A one-pass lagged solve cannot be started from zero. The
        // quasi-steady path tolerates a zero start because its outer loop
        // iterates to the fixed point regardless of where it begins; the
        // lagged path takes ONE pass, so a zero start means every section
        // computes its target in the absence of any other section's downwash,
        // which is a wing carrying far too much lift - and `Settle` then
        // adopts that as the state rather than passing through it. Measured
        // before this was added: the frontal lost mirror symmetry at t=0.000 s
        // with the canopy already fully collapsed, which is the seed transient
        // and not the aerodynamics under test.
        if (settings.lagCirculation)
        {
            state.circulation.resize(SectionList.size());
            for (std::size_t i = 0; i < SectionList.size(); ++i)
                state.circulation[i] = seed.sections[i].circulation;
        }
        state.initialised = true;
    }

    // Strand 2's lag states start settled on the circulation the wing is
    // already carrying, for the same reason the separation state starts at
    // equilibrium: a wing that has been holding this loading forever carries no
    // transient, and starting them at zero would inject one at t=0.
    if (settings.lagCirculation
        && state.circulationLag.size() != SectionList.size())
    {
        state.circulationLag.assign(SectionList.size(), WagnerLag{});
        if (state.circulation.size() == SectionList.size())
            for (std::size_t i = 0; i < SectionList.size(); ++i)
                state.circulationLag[i].Settle(state.circulation[i]);
    }

    const VsmSolution solution = SolveHeld(
        input, settings, &state.sectionSeparation, &state.circulation,
        settings.lagCirculation ? &state.circulationLag : nullptr,
        deltaSeconds);

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
    return SolveHeld(input, settings, nullptr, nullptr);
}

VsmSolution VortexStepMethodSolver::SolveFrozen(
    const VsmSolveInput& input, const VsmSeparationState& state,
    std::vector<double>& warmCirculation, const VsmSettings& settings) const
{
    const bool usable = state.initialised
        && state.sectionSeparation.size() == SectionList.size();
    return SolveHeld(input, settings,
                     usable ? &state.sectionSeparation : nullptr,
                     &warmCirculation);
}

VsmSolution VortexStepMethodSolver::SolveHeld(
    const VsmSolveInput& input, const VsmSettings& settings,
    const std::vector<double>* heldSeparation,
    std::vector<double>* warmCirculation,
    std::vector<WagnerLag>* lag, double lagDeltaSeconds) const
{
    const bool lagging = lag != nullptr;
    const std::size_t count = SectionList.size();
    VsmSolution solution;
    solution.sections.resize(count);
    if (count == 0) return solution;

    std::vector<double> circulation(count, 0.0);
    std::vector<double> nextCirculation(count, 0.0);
    // The in-plane speed each section saw at its own quasi-steady circulation.
    // Kept because the lagged pass converts circulation back to a lift
    // coefficient, Cl = 2*Gamma/(c V), and needs the same V the target was
    // built from rather than a second, slightly different, recomputation.
    std::vector<double> sectionSpeed(count, 0.0);
    // Every section's inflow with its OWN circulation excluded, kept so the
    // lagged pass can re-evaluate incidence at the lagged circulation instead
    // of reporting the incidence the target happened to sit at.
    std::vector<Vec3> sectionExternal(count);
    if (warmCirculation && warmCirculation->size() == count)
        circulation = *warmCirculation;
    if (lagging && lag->size() != count) lag->assign(count, WagnerLag{});

    const double dynamicPressureScale = 0.5 * input.airDensityKgM3;
    // The air's velocity relative to the wing. Converted here, once.
    const Vec3 freestream = -input.airspeedBodyMps;
    // Air arriving at one section and not another. Added to the freestream the
    // same way the rotation is, because it is the same kind of term: what the
    // air is doing where this section happens to be.
    const auto sectionGust = [&input](std::size_t i)
    {
        return i < input.sectionGustBodyMps.size()
            ? input.sectionGustBodyMps[i] : Vec3{};
    };

    // How far this section's chord has been twisted away from the design
    // pose by the lines. Empty means the design pose.
    const auto sectionIncidenceOffset = [&input](std::size_t i)
    {
        return i < input.sectionIncidenceOffsetRad.size()
            ? input.sectionIncidenceOffsetRad[i] : 0.0;
    };

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
        // Twisting the chord nose-up raises the incidence the section sees by
        // exactly that angle: the flow has not moved, the wing has.
        return std::atan2(-Dot(sectionVelocity, section.normal),
                          Dot(sectionVelocity, section.chordDirection))
            + sectionIncidenceOffset(i);
    };

    // Under-relaxation, adapted as it goes. Away from stall the polar is
    // smooth and the coupling between sections is weak, so a large step is
    // safe. On the stall knee the lift curve turns over within a fraction of a
    // degree and a step that was fine one iteration diverges the next. Backing
    // off when the residual grows, and easing back up when it falls, converges
    // both cases without either being tuned for.
    double relaxation = std::clamp(settings.relaxation, 0.01, 1.0);
    double previousResidual = 1.0e30;

    // Last pass's target, kept only under lag, so that `targetResidual` can
    // say whether the iterated target is settling on something. Nothing else
    // reads it and it does not enter any circulation, so the numbers this
    // solve produces are unchanged by its presence.
    std::vector<double> previousTarget;

    // HOW MANY PASSES BUILD THE TARGET THE WAGNER STEP THEN AIMS AT.
    //
    // This comment used to read: "Lagging makes the circulation a state, so
    // there is no fixed point to iterate toward: one Jacobi pass builds the
    // quasi-steady target and the Wagner step below does the advancing.
    // Iterating here instead would converge the state onto its own target
    // every tick, which is the quasi-steady answer with extra steps."
    //
    // THE LAST SENTENCE IS FALSE AND ITEM 30 MEASURED THE COST OF BELIEVING
    // IT. Wagner's function is the indicial response of circulation toward its
    // STEADY value, so iterating the target and then lagging ONCE per tick is
    // not "the quasi-steady answer with extra steps" - it is the only
    // arrangement in which the published function means what it says. What one
    // pass produces is a target that has itself travelled 0.233 of a step, and
    // Wagner applied to that gives 0.508 x 0.233 = 0.118 where Phi(0) is 0.5.
    //
    // The state is still a state: it is advanced exactly once per solve, from
    // its own lag registers, and it is never asked to converge to anything.
    // What iterates is the TARGET, which is allowed to.
    //
    // Defaults to 1, which is bit-identical to the behaviour above.
    const int outerPasses =
        lagging ? std::max(1, settings.lagTargetPasses)
                : settings.maxIterations;
    for (int iteration = 0; iteration < outerPasses; ++iteration)
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
            external += freestream - rotational + sectionGust(i);
            sectionExternal[i] = external;

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
            sectionSpeed[i] = speed;
            const SectionPolarSample polar = heldSeparation
                ? Polars.SampleAtSeparation(
                      alpha, brake, (*heldSeparation)[i], cellPressure)
                : Polars.SampleAtEquilibrium(alpha, brake, cellPressure);
            solution.sections[i].separation = heldSeparation
                ? (*heldSeparation)[i]
                : Polars.SeparationEquilibrium(alpha, brake, 0.0);
            solution.sections[i].angleOfAttackRad = alpha;
            solution.sections[i].liftCoefficient = polar.liftCoefficient;
            solution.sections[i].dragCoefficient =
                polar.dragCoefficient + settings.sectionDragOffset;
            solution.sections[i].momentCoefficient = polar.momentCoefficient;
            const Vec3 freeInPlane = -(freestream
                - section.spanDirection
                    * Dot(freestream, section.spanDirection));
            // Both terms carry the same twist, so it cancels: the induced
            // angle is what the wake did to this section, not where the
            // section is pointing.
            solution.sections[i].inducedAngleRad = alpha - std::atan2(
                -Dot(freeInPlane, section.normal),
                Dot(freeInPlane, section.chordDirection))
                - sectionIncidenceOffset(i);
        }

        // Does the target the Wagner step will aim at have a fixed point, and
        // did these passes find it? Measured as the pass-to-pass change in the
        // target itself, which is a different question from `residual` below
        // and the one nothing was asking. Read-only: it changes no circulation.
        if (lagging)
        {
            if (!previousTarget.empty())
            {
                double change = 0.0;
                double largest = 1.0e-9;
                for (std::size_t i = 0; i < count; ++i)
                {
                    change = std::max(change,
                        std::fabs(nextCirculation[i] - previousTarget[i]));
                    largest = std::max(largest, std::fabs(nextCirculation[i]));
                }
                solution.targetResidual = change / largest;
            }
            previousTarget = nextCirculation;
        }

        // The Wagner step happens ONCE, on the last pass. Everything before it
        // is the ordinary quasi-steady relaxation building the target.
        if (lagging && iteration == outerPasses - 1)
        {
            // Advanced in its own pass, not in the loop above, so that every
            // section's target is built from the SAME circulation distribution.
            // Updating in place would make this Gauss-Seidel, and a sweep that
            // runs left to right would give the left half of the wing one more
            // update than the right - an order-dependent seed, in the one solve
            // whose mirror symmetry is the gate.
            for (std::size_t i = 0; i < count; ++i)
            {
                const double ds = ReducedTimeSemichords(
                    sectionSpeed[i], SectionList[i].chordM, lagDeltaSeconds);
                circulation[i] = (*lag)[i].Advance(nextCirculation[i], ds);
                largestChange = std::max(largestChange,
                    std::fabs(nextCirculation[i] - circulation[i]));
                largestCirculation = std::max(largestCirculation,
                                              std::fabs(circulation[i]));

                solution.sections[i].circulation = circulation[i];
                solution.sections[i].quasiSteadyCirculation =
                    nextCirculation[i];

                // Re-evaluate the section AT THE LAGGED CIRCULATION. The pass
                // above reported whatever incidence the quasi-steady target
                // sat at, which is not where this wing is: the lagged
                // circulation induces a different downwash on the section, so
                // it flies at a different angle. That distinction is not
                // cosmetic here - the collapse model's `externalNoseCp` is a
                // function of section incidence alone, so reporting the
                // target's incidence would hand the pressure margin a field
                // belonging to a wing that does not exist.
                double speed = 0.0;
                const double alpha = sectionFlow(
                    i, sectionExternal[i], circulation[i], speed);
                const double brake =
                    input.leftBrake * (1.0 - SectionList[i].rightSideFraction)
                    + input.rightBrake * SectionList[i].rightSideFraction;
                const double cellPressure =
                    input.internalPressureCoefficient.empty() ? 1.0
                    : input.internalPressureCoefficient[
                          std::min(i,
                                   input.internalPressureCoefficient.size() - 1)];
                const SectionPolarSample polar = heldSeparation
                    ? Polars.SampleAtSeparation(
                          alpha, brake, (*heldSeparation)[i], cellPressure)
                    : Polars.SampleAtEquilibrium(alpha, brake, cellPressure);

                solution.sections[i].angleOfAttackRad = alpha;
                // Drag and moment are read off the polar at that incidence,
                // NOT lagged. Wagner's function is the indicial response of
                // circulatory lift; there is no published indicial response
                // for profile drag or for the quarter-chord couple to check
                // against, and inventing one would be a dial (guiding rule
                // 13). They follow the incidence, which is itself lagged
                // through the circulation, so they are not frozen either.
                solution.sections[i].dragCoefficient =
                    polar.dragCoefficient + settings.sectionDragOffset;
                solution.sections[i].momentCoefficient =
                    polar.momentCoefficient;

                // Lift read back OUT of the lagged circulation rather than off
                // the polar. This is the consistency item 27 names: the
                // induced velocity below uses `circulation`, so taking the
                // lift coefficient from the polar instead would be a wing
                // whose downwash and lift come from different instants. At a
                // settled state the two agree by construction, because the
                // target the lag converges on is the polar's own answer.
                if (speed > 1.0e-6)
                    solution.sections[i].liftCoefficient =
                        2.0 * circulation[i] / (SectionList[i].chordM * speed);

                const Vec3 freeInPlane = -(freestream
                    - SectionList[i].spanDirection
                        * Dot(freestream, SectionList[i].spanDirection));
                solution.sections[i].inducedAngleRad = alpha - std::atan2(
                    -Dot(freeInPlane, SectionList[i].normal),
                    Dot(freeInPlane, SectionList[i].chordDirection))
                    - sectionIncidenceOffset(i);
            }
            solution.iterations = 1;
            solution.residual = largestChange / largestCirculation;
            // A state does not converge to anything this tick, so saying it
            // did would be a false diagnostic. The residual above is the
            // distance still to travel, which is a transient and not an error.
            solution.converged = false;
            break;
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
        // Under lag the loop always runs its full count, because breaking out
        // early would skip the Wagner pass that only the LAST iteration
        // performs. It also keeps the cost per solve a fixed, declared number
        // rather than one that depends on how convergeable the current flow
        // happens to be - which in the separated regime is exactly the
        // property item 6 says is missing.
        if (!lagging && solution.residual < settings.convergenceTolerance)
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
    if (warmCirculation) *warmCirculation = circulation;

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
        const Vec3 localFlow =
            freestream - rotational + induced + sectionGust(i);
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

        // The section's OWN pitching moment about its quarter chord, which is
        // not the moment of its force about anything. A cambered section
        // carries a nose-down couple that survives even where it makes no
        // lift, and the whole of it was being discarded: the polar table has
        // computed a moment coefficient since Level 4 and nothing had ever
        // read it, so the wing had no aerodynamic pitching moment of its own.
        //
        // Nothing caught that either, because with the canopy pinned straight
        // below the payload the wing's incidence was set by the pin rather
        // than by any moment balance. A wing whose pitching moment is missing
        // entirely flies exactly as well as one whose pitching moment is
        // right, as long as nothing is free to pitch.
        //
        // Sign: the aerodynamic convention has Cm positive nose-up, and in
        // this frame a positive right-hand rotation about +Y tips the nose
        // DOWN (SuspensionGraph.h). spanDirection runs right to left, so
        // adding it scaled by q S c Cm puts a negative Cm on the +Y axis as a
        // nose-down moment, which is what camber does.
        const double sectionMoment = dynamicPressure * section.areaM2
            * section.chordM * solution.sections[i].momentCoefficient;
        solution.momentBodyNm += section.spanDirection * sectionMoment;
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
