// Level 8: the wing folds because of what is happening to it.
//
// The gates here are the plan's, reduced to what this stage can answer: a
// collapse must come from a local pressure balance rather than a threshold; it
// must produce genuinely unloaded sections; asymmetric collapse must be
// asymmetric; and nominal flight must never fold anything.
//
// Cravats and the reopening surge are not here. They need fabric-to-line
// contact and the collapsed section's shape, which is the self-collision the
// plan puts in this level and which is not built yet.
#include "CanopyCollapseSolver.h"
#include "CanopyGeometry.h"
#include "CanopyPressureSolver.h"
#include "SuspensionGraph.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

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

std::vector<double> SpanStations(int count)
{
    std::vector<double> spans(count);
    for (int i = 0; i < count; ++i)
        spans[i] = -1.0 + 2.0 * (i + 0.5) / count;
    return spans;
}

// A section flying normally: well pressurised, positive incidence, loaded
// lines, taut skin.
SectionCollapseInput Trim()
{
    SectionCollapseInput section;
    section.internalPressureCoefficient = 0.90;
    section.angleOfAttackRad = 0.08;
    section.separation = 0.0;
    section.loadFraction = 1.0;
    section.skinSlackFraction = 0.0;
    section.brake = 0.0;
    return section;
}

double Run(const CanopyCollapseSolver& solver, CollapseState& state,
           std::vector<SectionCollapseInput> sections, double seconds)
{
    constexpr double Dt = 1.0 / 120.0;
    CollapseResult result;
    const int steps = static_cast<int>(seconds / Dt);
    for (int step = 0; step < steps; ++step)
        result = solver.Step(state, sections, Dt);
    return result.worstCollapse;
}
}

int main()
{
    const auto spans = SpanStations(16);
    const CanopyCollapseSolver solver(spans);

    // -- the criterion is a pressure balance ------------------------------
    {
        // Checked directly, because the criterion is the whole level. A
        // threshold on brake would pass every behavioural test below while
        // being exactly what this replaces.
        const SectionCollapseInput trim = Trim();
        const double trimMargin = solver.PressureMargin(trim);
        std::printf("Trim: internal Cp %.2f, external nose Cp %+.2f, "
                    "margin %+.2f\n",
                    trim.internalPressureCoefficient,
                    solver.ExternalNoseCp(trim), trimMargin);
        Check(trimMargin > 0.0,
              "a pressurised section at trim holds its nose - internal "
              "pressure exceeds the suction at the fold station");

        // Bar. Incidence drops, and BOTH sides of the balance move the wrong
        // way at once: Level 5 has the inlet feeding less well, and the nose
        // shoulder loses the suction that was holding the skin taut.
        SectionCollapseInput accelerated = trim;
        accelerated.angleOfAttackRad = 0.01;
        accelerated.internalPressureCoefficient = 0.78;
        const double barMargin = solver.PressureMargin(accelerated);
        std::printf("On bar: internal Cp %.2f, external nose Cp %+.2f, "
                    "margin %+.2f\n",
                    accelerated.internalPressureCoefficient,
                    solver.ExternalNoseCp(accelerated), barMargin);
        Check(barMargin < trimMargin,
              "accelerated flight has less margin than trim, which is why it "
              "is the collapse-prone regime");

        // The physical claim underneath all of it, checked as a relationship
        // rather than at two points: the nose shoulder must lose its outward
        // suction MONOTONICALLY as incidence drops, and pass through zero into
        // positive pressure pushing the skin in. A criterion that only happened
        // to be ordered at the two incidences a test picked would pass every
        // behavioural gate below while being the wrong shape.
        std::printf("External nose Cp against incidence:\n");
        double previous = -1.0e9;
        bool monotone = true;
        bool reachesPressure = false;
        for (const double alpha :
                 {-0.30, -0.20, -0.10, 0.0, 0.08, 0.18, 0.28})
        {
            SectionCollapseInput section = trim;
            section.angleOfAttackRad = alpha;
            const double cp = solver.ExternalNoseCp(section);
            std::printf("  alpha %+.2f rad -> Cp %+.3f\n", alpha, cp);
            // Lower incidence must mean a LESS negative shoulder, so walking
            // upward in alpha must walk downward in Cp.
            if (cp > previous && previous > -1.0e8) monotone = false;
            if (alpha < 0.0 && cp > 0.0) reachesPressure = true;
            previous = cp;
        }
        Check(monotone,
              "the nose shoulder loses suction monotonically as incidence "
              "drops - the criterion is a relationship, not two points");
        Check(reachesPressure,
              "and at negative incidence the shoulder is under positive "
              "pressure pushing the skin in, which is the frontal collapse");

        // Unloading alone, at unchanged pressure and incidence. This is the
        // turbulence mechanism: the wing is unloaded first and folds second.
        SectionCollapseInput unloaded = trim;
        unloaded.loadFraction = 0.0;
        std::printf("Unloaded: margin %+.2f against %+.2f loaded\n",
                    solver.PressureMargin(unloaded), trimMargin);
        Check(solver.PressureMargin(unloaded) < trimMargin,
              "a section carrying no line load folds at a margin that would "
              "hold a loaded one");

        // And the external side is Level 5's own nose distribution, not a
        // second copy of it. Evaluated at the stagnation point it must be 1.
        SectionCollapseInput atStagnation = trim;
        atStagnation.angleOfAttackRad =
            atStagnation.zeroLiftAngleRad - solver.Spec().noseFoldAngleRad;
        Check(std::fabs(solver.ExternalNoseCp(atStagnation) - 1.0) < 1.0e-9,
              "the fold station sees Cp = 1 when the stagnation point is on "
              "it - the same cylinder distribution Level 5 feeds inlets with");
    }

    // -- nominal flight never folds ---------------------------------------
    {
        // The plan's exit gate: the numerical safety envelope must not engage
        // during any nominal manoeuvre. The behavioural half of that is that
        // ordinary flight must not collapse the wing either.
        CollapseState state;
        std::vector<SectionCollapseInput> sections(spans.size(), Trim());
        const double clean = Run(solver, state, sections, 20.0);
        std::printf("Twenty seconds at trim: worst collapse %.4f\n", clean);
        Check(clean < 1.0e-6, "a pressurised wing at trim does not collapse");

        // Nor does a braked one - that is a stall, and it belongs to Level 4's
        // separation rather than here.
        for (auto& section : sections)
        {
            section.brake = 0.8;
            section.angleOfAttackRad = 0.28;
            section.separation = 0.9;
            section.internalPressureCoefficient = 0.72;
        }
        CollapseState braked;
        const double stalled = Run(solver, braked, sections, 20.0);
        std::printf("Twenty seconds at 0.8 brake: worst collapse %.4f\n",
                    stalled);
        Check(stalled < 1.0e-6,
              "deep brake stalls the wing rather than collapsing it - a "
              "braked section has MORE nose pressure, not less");
    }

    // -- an asymmetric collapse is asymmetric ------------------------------
    {
        CollapseState state;
        std::vector<SectionCollapseInput> sections(spans.size(), Trim());
        // The left tip loses its pressure and its load, which is what a tip
        // flying into a rotor edge actually experiences.
        for (std::size_t i = 0; i < spans.size(); ++i)
        {
            if (spans[i] > -0.55) continue;
            sections[i].internalPressureCoefficient = 0.05;
            sections[i].loadFraction = 0.0;
            sections[i].angleOfAttackRad = -0.06;
        }
        constexpr double Dt = 1.0 / 120.0;
        CollapseResult result;
        for (int step = 0; step < 120; ++step)
            result = solver.Step(state, sections, Dt);
        std::printf("Left tip unloaded, 1 s: left %.3f, right %.3f, worst "
                    "%.3f at span %+.2f\n",
                    result.leftCollapse, result.rightCollapse,
                    result.worstCollapse, result.worstSpanFraction);
        Check(result.leftCollapse > 0.2,
              "the tip that lost its pressure folds");
        Check(result.rightCollapse < 0.02,
              "and the other side does not - a collapse is a place, not a "
              "state of the whole wing");
        Check(result.worstSpanFraction < 0.0,
              "the worst section is on the side that lost pressure");
    }

    // -- a fold spreads, and stops ----------------------------------------
    {
        // Cells share ribs and crossports, so a fold pulls on its neighbours.
        // It must spread - a collapse that stops dead at a cell boundary is
        // not what fabric does - and it must not run away across a wing whose
        // other sections are perfectly pressurised.
        CollapseState state;
        std::vector<SectionCollapseInput> sections(spans.size(), Trim());
        sections[0].internalPressureCoefficient = 0.0;
        sections[0].loadFraction = 0.0;
        sections[0].angleOfAttackRad = -0.10;
        constexpr double Dt = 1.0 / 120.0;
        CollapseResult result;
        for (int step = 0; step < 120 * 3; ++step)
            result = solver.Step(state, sections, Dt);
        std::printf("One dead cell, 3 s: section 0 %.3f, 1 %.3f, 2 %.3f, "
                    "far side %.3f\n",
                    result.sections[0].collapse, result.sections[1].collapse,
                    result.sections[2].collapse,
                    result.sections[spans.size() - 1].collapse);
        Check(result.sections[0].collapse > 0.5, "the dead cell folds");
        Check(result.sections[1].collapse > 1.0e-3,
              "and pulls on its neighbour through the shared rib");
        Check(result.sections[1].propagated,
              "which is reported as propagation rather than as that section "
              "having its own pressure problem");
        Check(result.sections[1].collapse < result.sections[0].collapse,
              "a fold spreads by degrees rather than copying itself");
        Check(result.sections[spans.size() - 1].collapse < 0.05,
              "and a pressurised far tip stays flying");
    }

    // -- folding is fast, reopening is not ---------------------------------
    {
        CollapseState state;
        std::vector<SectionCollapseInput> sections(spans.size(), Trim());
        for (auto& section : sections)
        {
            section.internalPressureCoefficient = 0.0;
            section.loadFraction = 0.0;
            section.angleOfAttackRad = -0.10;
        }
        constexpr double Dt = 1.0 / 120.0;
        CollapseResult folding;
        for (int step = 0; step < 60; ++step)
            folding = solver.Step(state, sections, Dt);
        const double afterHalfSecond = folding.worstCollapse;

        // Pressure back, section flying, hands up.
        for (auto& section : sections) section = Trim();
        CollapseResult reopening;
        for (int step = 0; step < 60; ++step)
            reopening = solver.Step(state, sections, Dt);
        std::printf("Fold in 0.5 s: %.3f. Half a second of recovery: %.3f\n",
                    afterHalfSecond, reopening.worstCollapse);
        Check(afterHalfSecond > 0.9, "half a second is enough to fold a wing");
        Check(reopening.worstCollapse > 0.3,
              "and half a second is nowhere near enough to reopen it - the "
              "air has to go back in through the inlet that just emptied");

        // Brake on the folded side holds it in, which is why the recovery is
        // to release first and pump second.
        CollapseState held = state;
        CollapseState released = state;
        std::vector<SectionCollapseInput> braked(spans.size(), Trim());
        for (auto& section : braked) section.brake = 1.0;
        CollapseResult heldResult;
        CollapseResult releasedResult;
        for (int step = 0; step < 120 * 2; ++step)
        {
            heldResult = solver.Step(held, braked, Dt);
            releasedResult = solver.Step(released, sections, Dt);
        }
        std::printf("Two seconds: brake held %.3f, brake released %.3f\n",
                    heldResult.worstCollapse, releasedResult.worstCollapse);
        Check(heldResult.worstCollapse > releasedResult.worstCollapse,
              "brake on the collapsed side holds the fold in");
    }

    // -- a cravat is fabric caught on a line -------------------------------
    {
        // Measured off the built EPIC 2 suspension graph rather than invented:
        // how far a folded tip has to reach to get past the line running under
        // it. The plan asks for cravats from "tip geometry and line/fabric
        // contact", and both halves are real here - Level 6 solves the fold
        // depth, Level 2 owns where the lines are.
        const SuspensionGraph graph =
            BuildSuspensionGraph(CanopyGeometry{}, Epic2MlLinePlan());
        double tipGapM = 1.0e9;
        double tipSpan = 0.0;
        for (const SuspensionNode& node : graph.nodes)
        {
            if (node.kind != SuspensionNodeKind::CanopyAttachment) continue;
            if (std::fabs(node.spanFraction) < 0.75) continue;
            // The lateral distance from this attachment to the centreline
            // side of it is the room a fold has to swing through before it is
            // outboard of the line hanging from the attachment.
            const double gap = std::fabs(node.canopyLocalM.y)
                - std::fabs(CanopyPointLocalM(graph, 0.75, 0.25).y);
            if (gap > 0.0 && gap < tipGapM)
            {
                tipGapM = gap;
                tipSpan = node.spanFraction;
            }
        }
        std::printf("Tip line gap from the built graph: %.3f m at span "
                    "%+.2f\n", tipGapM, tipSpan);
        Check(tipGapM > 0.02 && tipGapM < 2.0,
              "the tip line gap is a real distance off the suspension graph");

        // A folded tip whose fold does not reach the line cannot cravat,
        // however hard it is collapsed. This is the check that keeps the
        // criterion geometric rather than probabilistic.
        const auto flyCravat = [&](double foldDepthM, double loadFraction,
                                   double brake, double seconds)
        {
            CollapseState state;
            std::vector<SectionCollapseInput> sections(spans.size(), Trim());
            for (std::size_t i = 0; i < spans.size(); ++i)
            {
                if (spans[i] > -0.75) continue;
                sections[i].internalPressureCoefficient = 0.0;
                sections[i].loadFraction = 0.0;
                sections[i].angleOfAttackRad = -0.12;
                sections[i].foldDepthM = foldDepthM;
                sections[i].lineGapM = tipGapM;
            }
            constexpr double Dt = 1.0 / 120.0;
            CollapseResult result;
            // Fold it first, with the lines slack.
            for (int step = 0; step < 120; ++step)
                result = solver.Step(state, sections, Dt);
            // Then the line comes back under load, which is the moment the
            // fabric is either trapped or not.
            for (std::size_t i = 0; i < spans.size(); ++i)
            {
                if (spans[i] > -0.75) continue;
                sections[i].loadFraction = loadFraction;
                sections[i].brake = brake;
            }
            const int steps = static_cast<int>(seconds * 120.0);
            for (int step = 0; step < steps; ++step)
                result = solver.Step(state, sections, Dt);
            return result;
        };

        const CollapseResult shallow = flyCravat(0.5 * tipGapM, 1.0, 0.0, 3.0);
        const CollapseResult deep = flyCravat(2.2 * tipGapM, 1.0, 0.0, 3.0);
        std::printf("Fold shallower than the gap: cravat %.3f. Deeper: %.3f\n",
                    shallow.leftCravat, deep.leftCravat);
        Check(shallow.leftCravat < 1.0e-6,
              "a fold that does not reach the line cannot catch on it - the "
              "criterion is contact, not a chance of one");
        Check(deep.leftCravat > 0.05,
              "and one that reaches past a loaded line does catch");
        Check(deep.rightCravat < 1.0e-6,
              "on the side it happened, not across the wing");

        // The order of events is the whole reason a cravat is rarer than a
        // collapse: the fabric has to be past the line at the moment the line
        // reloads. Reaching while everything stays slack drops back out.
        const CollapseResult neverLoaded =
            flyCravat(2.2 * tipGapM, 0.0, 0.0, 3.0);
        std::printf("Deep fold, line never reloads: cravat %.3f\n",
                    neverLoaded.leftCravat);
        Check(neverLoaded.leftCravat < 1.0e-6,
              "a fold that reaches past a slack line is not caught by it - "
              "nothing is holding it there");

        // It LATCHES. This is the property that matters and the one that was
        // wrong first time: a cravat that is a function of the current fold
        // melts away as the section reopens, which is exactly the behaviour
        // that makes a cravat dangerous by not happening. Checked as
        // persistence rather than against a magnitude - how deep the catch
        // gets depends on the race between the line reloading and the section
        // reopening, and that race is physics rather than a number to pin.
        const CollapseResult persists =
            flyCravat(2.2 * tipGapM, 1.0, 0.0, 20.0);
        const CollapseResult braked =
            flyCravat(2.2 * tipGapM, 1.0, 0.9, 20.0);
        std::printf("Twenty seconds: hands up %.3f, deep brake %.3f\n",
                    persists.leftCravat, braked.leftCravat);
        Check(persists.leftCravat >= deep.leftCravat * 0.98,
              "a cravat held by a loaded line does not fall out on its own - "
              "seventeen more seconds of flying does not shift it");
        Check(braked.leftCravat < persists.leftCravat * 0.75,
              "and deep brake on that side walks it back off, which is the "
              "clearance a pilot actually flies");

        // The race itself, stated: reloading the line is what traps the fabric
        // AND what helps the section reopen. That is why a cravat is rarer
        // than a collapse, and why the catch is partial rather than total in a
        // section that recovers quickly.
        std::printf("  fold reach past the line at catch: %.3f m of a "
                    "%.3f m gap\n",
                    deep.sections[0].foldReachPastLineM, tipGapM);
    }

    // -- determinism -------------------------------------------------------
    {
        const auto run = [&]()
        {
            CollapseState state;
            std::vector<SectionCollapseInput> sections(spans.size(), Trim());
            sections[3].internalPressureCoefficient = 0.1;
            sections[3].loadFraction = 0.2;
            return Run(solver, state, sections, 4.0);
        };
        Check(run() == run(),
              "the same inputs give a bit-identical collapse state");
    }

    if (Failures == 0) std::printf("All collapse checks passed.\n");
    else std::printf("%d collapse check(s) failed.\n", Failures);
    return Failures == 0 ? 0 : 1;
}
