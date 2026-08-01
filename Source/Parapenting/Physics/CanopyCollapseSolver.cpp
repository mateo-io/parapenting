#include "CanopyCollapseSolver.h"

#include "CanopyPressureSolver.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
CanopyCollapseSolver::CanopyCollapseSolver(
    std::vector<double> spanFractions, const CollapseSpec& spec)
    : SpanFractions(std::move(spanFractions)), SpecValue(spec)
{
}

double CanopyCollapseSolver::ExternalNoseCp(
    const SectionCollapseInput& section) const
{
    // Where the stagnation point sits, from Level 5's own relation - not a
    // second copy of it. The fold station is a fixed place on the nose skin;
    // what moves is the stagnation point, and therefore how far the fold
    // station sits from it.
    //
    // Raising incidence moves the stagnation point down toward the lower
    // surface, which puts the upper shoulder further from it and deeper into
    // suction - holding the skin out. Dropping incidence brings the stagnation
    // point up over the nose toward the shoulder, and Cp there rises through
    // zero into positive pressure that pushes the skin in. Same nose, same
    // distribution Level 5 feeds the inlets with, read at a different station.
    const double stagnation = section.angleOfAttackRad - section.zeroLiftAngleRad;
    // Toward the upper surface is the opposite sign to the inlet's position,
    // which Level 5 measures toward the lower surface.
    const double offsetFromStagnation =
        -SpecValue.noseFoldAngleRad - stagnation;
    return CanopyPressureSolver::NosePressureCoefficient(offsetFromStagnation);
}

double CanopyCollapseSolver::PressureMargin(
    const SectionCollapseInput& section) const
{
    // The balance across the nose skin, non-dimensionalised by the local
    // dynamic pressure so it is a pure number and the same criterion holds at
    // any airspeed:
    //
    //     margin = Cp_internal - Cp_external(fold station)
    //
    // Positive means the air inside is pushing the shell out harder than the
    // flow outside is pulling it in, which is what holding a shape means.
    const double external = ExternalNoseCp(section);
    double margin = section.internalPressureCoefficient - external;

    // A section that has stopped carrying load has slack A lines and nothing
    // holding its nose forward into the airflow, so it folds at a margin that
    // would hold a loaded one. This is the mechanism behind a turbulence
    // collapse: the wing is unloaded first and folds second.
    const double unloaded = std::clamp(1.0 - section.loadFraction, 0.0, 1.0);
    margin -= SpecValue.unloadedMarginPenalty * unloaded;

    // Skin already carrying no tension is skin that has begun to crease. Level
    // 6 reports it directly, and it erodes the same margin.
    margin -= 0.35 * std::clamp(section.skinSlackFraction, 0.0, 1.0);
    return margin;
}

CollapseResult CanopyCollapseSolver::Step(
    CollapseState& state, const std::vector<SectionCollapseInput>& input,
    double deltaSeconds) const
{
    const std::size_t count = std::min(SpanFractions.size(), input.size());
    CollapseResult result;
    result.sections.resize(count);
    if (count == 0) return result;

    if (!state.initialised || state.sections.size() != count)
    {
        state.sections.assign(count, SectionCollapseState{});
        state.initialised = true;
    }

    const double dt = std::max(0.0, deltaSeconds);

    // Pass one: each section's own pressure balance.
    std::vector<double> target(count, 0.0);
    for (std::size_t i = 0; i < count; ++i)
    {
        const double margin = PressureMargin(input[i]);
        result.sections[i].pressureMargin = margin;
        result.sections[i].externalNoseCp = ExternalNoseCp(input[i]);
        // A margin at or above zero holds. Below zero the section folds, and
        // how far it folds is how far past the balance it has gone - a nose
        // barely under is a flutter, a nose well under is gone.
        target[i] = std::clamp(-margin / 0.6, 0.0, 1.0);
    }

    // Pass two: neighbours. Cells share ribs and crossports, so a fold pulls
    // on the sections either side of it. Read from the state at the start of
    // the step rather than from the values being written, or the spread
    // depends on which end the loop started at.
    std::vector<double> neighbourPull(count, 0.0);
    for (std::size_t i = 0; i < count; ++i)
    {
        double pull = 0.0;
        if (i > 0) pull = std::max(pull, state.sections[i - 1].collapse);
        if (i + 1 < count)
            pull = std::max(pull, state.sections[i + 1].collapse);
        neighbourPull[i] = pull;
    }

    for (std::size_t i = 0; i < count; ++i)
    {
        SectionCollapseState& section = state.sections[i];
        const double own = target[i];
        // What the neighbours can drag this section to, which is less than
        // they are themselves - a fold spreads by degrees rather than copying.
        const double spread = std::clamp(
            neighbourPull[i] * SpecValue.spanwisePropagationPerSecond * dt,
            0.0, neighbourPull[i]);
        const double combined = std::max(own, spread);
        result.sections[i].propagated = spread > own && spread > 1.0e-6;

        if (combined > section.collapse)
        {
            // Folding. Fast, and not gated on anything: a nose that has lost
            // its pressure balance is already going.
            const double rate = SpecValue.foldRatePerSecond * dt;
            section.collapse += (combined - section.collapse)
                * std::clamp(rate, 0.0, 1.0);
        }
        else
        {
            // Reopening. The section has to be flying again before its inlet
            // is fed at all, so a separated section stays folded however good
            // its pressure balance looks - and brake on that side holds the
            // trailing edge down and keeps the nose from catching air, which
            // is why the recovery is to release first and pump second.
            const double flying = input[i].separation
                    >= SpecValue.reopenSeparationLimit
                ? 0.0
                : 1.0 - input[i].separation / SpecValue.reopenSeparationLimit;
            const double braked = 1.0
                - SpecValue.brakeHoldsCollapseScale
                    * std::clamp(input[i].brake, 0.0, 1.0);
            const double rate = SpecValue.reopenRatePerSecond * dt
                * flying * std::max(0.0, braked);
            section.collapse += (combined - section.collapse)
                * std::clamp(rate, 0.0, 1.0);
        }
        section.collapse = std::clamp(section.collapse, 0.0, 1.0);
        result.sections[i].collapse = section.collapse;

        // -- cravats -------------------------------------------------------
        // Fabric caught on a line. Geometry first: how far the fold reaches
        // past the line running under this station. Level 6 solves the depth,
        // Level 2 owns the gap, and neither of them is about cravats - which
        // is what makes this a contact test rather than a risk model.
        //
        // The fold only reaches where the section has actually folded, so the
        // depth is scaled by how collapsed it is. A flying section has a
        // bulge, not a fold, and reaches nothing.
        const double reach =
            input[i].foldDepthM * section.collapse - input[i].lineGapM;
        result.sections[i].foldReachPastLineM = reach;

        const bool trapped =
            input[i].loadFraction >= SpecValue.cravatTrappingLoadFraction;

        // Catching. A cravat LATCHES: once the fabric is round the line and
        // the line is loaded, it stays there even as the rest of the section
        // reopens and the fold shallows. Getting this wrong makes the cravat a
        // function of the current fold rather than a state, and it then melts
        // away as the wing recovers - which is precisely the behaviour that
        // makes a cravat dangerous by NOT happening.
        if (reach > 0.0 && trapped)
        {
            const double depth = std::clamp(reach / std::max(0.02,
                input[i].lineGapM), 0.0, 1.0);
            if (depth > section.cravat)
            {
                const double rate = std::clamp(
                    SpecValue.cravatCatchRatePerSecond * dt, 0.0, 1.0);
                section.cravat += (depth - section.cravat) * rate;
            }
        }

        // Clearing, which is a separate question from catching and can happen
        // in the same step. A cravat does not fall out on its own while the
        // line through it is loaded - the harder the wing flies, the tighter
        // that line pulls. What clears it is deep brake on that side walking
        // the fabric back off, or the line going slack enough to let go.
        if (section.cravat > 0.0)
        {
            const double brakeHelp =
                input[i].brake >= SpecValue.cravatClearingBrake
                    ? (input[i].brake - SpecValue.cravatClearingBrake)
                        / std::max(1.0e-6, 1.0 - SpecValue.cravatClearingBrake)
                    : 0.0;
            const double slackHelp = trapped ? 0.0 : 1.0;
            const double rate = std::clamp(
                SpecValue.cravatClearRatePerSecond * dt
                    * (slackHelp + brakeHelp),
                0.0, 1.0);
            section.cravat -= section.cravat * rate;
        }
        section.cravat = std::clamp(section.cravat, 0.0, 1.0);
        result.sections[i].cravat = section.cravat;
    }

    // Span-weighted halves, and the worst single section.
    double leftWeight = 0.0;
    double rightWeight = 0.0;
    for (std::size_t i = 0; i < count; ++i)
    {
        const double collapse = state.sections[i].collapse;
        const double span = SpanFractions[i];
        const double cravat = state.sections[i].cravat;
        if (span < 0.0)
        {
            result.leftCollapse += collapse;
            result.leftCravat += cravat;
            leftWeight += 1.0;
        }
        else
        {
            result.rightCollapse += collapse;
            result.rightCravat += cravat;
            rightWeight += 1.0;
        }
        if (cravat > 0.5) ++result.cravattedSectionCount;
        if (collapse > result.worstCollapse)
        {
            result.worstCollapse = collapse;
            result.worstSpanFraction = span;
        }
        if (collapse > 0.5) ++result.collapsedSectionCount;
    }
    if (leftWeight > 0.0)
    {
        result.leftCollapse /= leftWeight;
        result.leftCravat /= leftWeight;
    }
    if (rightWeight > 0.0)
    {
        result.rightCollapse /= rightWeight;
        result.rightCravat /= rightWeight;
    }
    result.symmetricCollapse =
        std::min(result.leftCollapse, result.rightCollapse);
    return result;
}
}
