#pragma once

#include <cstddef>
#include <vector>

namespace Parapenting::Physics
{
// Level 8 of the master plan: the wing collapses because of what is happening
// to it, not because a threshold said so.
//
// The legacy model has collapse as a scripted state machine: a global brake
// number crosses a limit, a timer starts, a shape is played back. That is
// guiding rule 6's explicit target - "collapses arise from local unloading,
// pressure loss, and membrane deformation rather than random scripted
// folding" - and it is what this replaces.
//
// Nothing here invents a new criterion. Every quantity it needs was already
// solved by a level below:
//
//   Level 4  per-section incidence and separation, from the circulation solve
//   Level 5  per-cell internal pressure coefficient, and what the inlet sees
//   Level 2  per-section line tension, and whether a row has gone slack
//   Level 6  how much of the skin between two ribs is carrying no tension
//
// The criterion is a pressure balance across the nose, which is what a leading
// edge physically is: a curved fabric shell held in shape by the air inside it.
// It keeps its shape while
//
//     internal gauge pressure  >  external suction at the nose
//
// and folds when that reverses. Both sides of it are already computed. The
// internal side is Level 5's cell pressure. The external side is the same
// rounded-nose cylinder distribution Level 5 uses to decide what the inlet
// recovers, Cp = 1 - 4 sin^2 t, evaluated where the nose skin is rather than
// where the opening is.
//
// The external term needs its sign said out loud, because it is the opposite
// of the intuition that suction is dangerous. Suction on the outside of the
// nose shoulder pulls the skin OUTWARD and holds it taut; positive pressure
// there pushes it IN. So the external contribution to the margin is negative
// Cp, and what folds a nose is the shoulder losing its suction, not gaining
// it. Across the incidence range the fold station sees:
//
//     braked  0.28 rad   Cp -1.06     strongly held out
//     trim    0.08 rad   Cp -0.28     held out
//     bar     0.01 rad   Cp -0.02     nothing holding it
//     low    -0.10 rad   Cp +0.34     being pushed in
//     lower  -0.30 rad   Cp +0.81     near stagnation, folding
//
// which is the frontal collapse exactly as it happens: incidence drops, the
// stagnation point climbs over the nose onto the upper surface, and the
// shoulder that was under suction is now under pressure pushing it in.
//
// This is why the real behaviours fall out rather than being written in:
//
//   * ACCELERATED FLIGHT IS COLLAPSE-PRONE. Bar drops the incidence, so the
//     stagnation point moves up and away from the inlets and the cell recovers
//     less total pressure - Level 5's mechanism - while the same incidence
//     change takes the suction off the nose shoulder that was holding the skin
//     taut. Both sides of the balance move the wrong way at once, for the same
//     reason.
//   * BRAKE DOES NOT COLLAPSE A WING. It raises incidence, which deepens the
//     suction on the shoulder and feeds the inlets better. Pulling too much
//     brake stalls a wing; it does not fold it. That fell out of the balance
//     rather than being asserted.
//   * A TIP GOES FIRST. The tip cells are smallest, so they hold the least
//     air, and they are furthest from the crossports feeding them.
//   * TURBULENCE COLLAPSES A WING BY UNLOADING IT. A section that stops
//     carrying load has slack A lines and nothing holding its nose forward
//     against the airflow, so the same suction folds it at a much lower
//     margin.
//
// What is NOT here yet, and is named rather than hidden: cravats, which need
// fabric-to-line contact and therefore the self-collision the plan puts in
// this level; and the reopening surge, which needs the collapsed section's
// shape rather than only its state.

struct CollapseSpec
{
    // Where on the nose the fold starts, radians from the chord line toward
    // the UPPER surface. This is the shoulder just aft of the leading edge,
    // where the suction peak sits and where a real canopy creases.
    double noseFoldAngleRad = 0.45;
    // How fast a section folds once the balance has reversed, and how fast it
    // recovers once it is back. A collapse is violent and a reopening is not:
    // the fabric has to be re-inflated through the same inlet that just
    // emptied it, and the section has to be flying again before that inlet is
    // fed at all. Level 4's separation state has the same asymmetry for the
    // same reason.
    double foldRatePerSecond = 9.0;
    double reopenRatePerSecond = 1.6;
    // Neighbouring cells share ribs and crossports, so a fold does not stop at
    // a cell boundary. This is the fraction of a section's collapse that
    // spreads to each neighbour per second - the mechanism is the shared
    // membrane and the crossport flow, both of which exist a level below.
    double spanwisePropagationPerSecond = 2.4;
    // A section whose A row has gone slack has nothing holding its nose
    // forward. This scales how much that unloading erodes the pressure margin:
    // at full slack the section folds at a margin that would hold a loaded
    // one.
    double unloadedMarginPenalty = 0.75;
    // Reopening needs the section flying again, not merely re-pressurised. A
    // fully separated section is not flying, so this gates recovery on the
    // Level 4 separation state.
    double reopenSeparationLimit = 0.55;
    // Brake holds a collapse in. Pulling the trailing edge down on a folded
    // section keeps the nose from catching air, which is why the standard
    // recovery is to release the brake on that side first and only then pump.
    double brakeHoldsCollapseScale = 0.6;

    // -- cravats ----------------------------------------------------------
    // A cravat is fabric caught on a LINE, not on itself. Three things have to
    // be true at once, and all three are solved elsewhere: the section is
    // folded (Level 8's own collapse), the fold is deep enough to reach past
    // the line running under it (Level 6's fold depth against Level 2's
    // geometry), and the line then comes back under tension and traps it
    // (Level 2 again).
    //
    // The order matters and is the whole reason a cravat is rarer than a
    // collapse: the fabric has to be somewhere it should not be at the moment
    // the line reloads. A fold that reaches while everything stays slack drops
    // straight back out.
    double cravatCatchRatePerSecond = 6.0;
    // Tension above which a reloading line is holding the fold rather than
    // hanging beside it, as a fraction of the section's normal load.
    double cravatTrappingLoadFraction = 0.35;
    // How fast a cravat clears when the line lets go of it. Slow, and slower
    // than a collapse reopens - the fabric has to come back out the way it
    // went in, past a line that is still there.
    double cravatClearRatePerSecond = 0.35;
    // Deep brake on that side pulls the trailing edge and can walk the fabric
    // back off the line, which is the standard cravat clearance before a
    // stabilo pull. Below this the brake does nothing for it.
    double cravatClearingBrake = 0.55;
};

// What one section is doing, per span station. All of it is solved, none of it
// is a mode flag.
struct SectionCollapseInput
{
    // Level 5: internal gauge pressure over local dynamic pressure.
    double internalPressureCoefficient = 0.9;
    // Level 4: local incidence and how separated the section is.
    double angleOfAttackRad = 0.08;
    double separation = 0.0;
    // The section's zero-lift angle, which is where the stagnation point sits
    // on the chord line. Brake moves it, so brake moves the fold angle.
    double zeroLiftAngleRad = -0.07;
    // Level 2: how much of this section's A-row tension is left, 1 fully
    // loaded and 0 completely slack.
    double loadFraction = 1.0;
    // Level 6: fraction of the skin here carrying no tension at all.
    double skinSlackFraction = 0.0;
    // Brake applied to this section, 0 to 1.
    double brake = 0.0;

    // -- cravat geometry, from Level 6 and Level 2 ------------------------
    // How far the folded skin hangs below the canopy surface here, metres.
    // Level 6's fold depth, which only exists once the ribs have converged.
    double foldDepthM = 0.0;
    // How far a fold has to reach to get past the nearest line running under
    // this station, metres. Measured off the built suspension graph, so it is
    // the real gap between this attachment and the line beside it rather than
    // a number about tips in general.
    double lineGapM = 1.0;
};

struct SectionCollapseState
{
    // 0 flying, 1 fully folded. Continuous, because a canopy folds
    // progressively and a half-collapsed wing flies differently from both a
    // clean one and a fully folded one.
    double collapse = 0.0;
    // Fabric caught on a line, 0 to 1.
    //
    // This is the state that does not clear itself. A collapse reopens when
    // the pressure comes back; a cravat is held by the line that is through
    // it, and the harder the wing flies the tighter that line pulls. It is
    // why a cravat turns into a spiral and a collapse usually does not.
    double cravat = 0.0;
};

struct CollapseState
{
    std::vector<SectionCollapseState> sections;
    bool initialised = false;
};

struct SectionCollapseDiagnostics
{
    // The pressure balance itself, in pascals of margin per unit dynamic
    // pressure: positive holds the nose, negative folds it. Reported because
    // it is the criterion, and a criterion nobody can read is a threshold.
    double pressureMargin = 0.0;
    // External pressure coefficient at the fold station.
    double externalNoseCp = 0.0;
    double collapse = 0.0;
    // True when this section's fold came from its neighbours rather than from
    // its own pressure balance - which is what makes a collapse spread across
    // a wing rather than appearing everywhere with the same cause.
    bool propagated = false;
    double cravat = 0.0;
    // How far the fold reaches past the line beside it, metres. Positive means
    // the fabric is where a line can trap it. Reported because it is the
    // geometric half of the cravat criterion and it is measurable.
    double foldReachPastLineM = 0.0;
};

struct CollapseResult
{
    std::vector<SectionCollapseDiagnostics> sections;
    // Span-weighted collapse on each half. These are what a flight model and a
    // HUD want, and they are sums over the sections rather than the state.
    double leftCollapse = 0.0;
    double rightCollapse = 0.0;
    double symmetricCollapse = 0.0;
    // Worst single section, and where it is. A wing with one folded tip and a
    // clean rest reads very differently from a uniformly soft one.
    double worstCollapse = 0.0;
    double worstSpanFraction = 0.0;
    int collapsedSectionCount = 0;
    double leftCravat = 0.0;
    double rightCravat = 0.0;
    int cravattedSectionCount = 0;
};

class CanopyCollapseSolver
{
public:
    // Span fractions, -1 at the left tip to +1 at the right, one per section.
    // Taken rather than derived so this solves the same stations the
    // aerodynamics does.
    explicit CanopyCollapseSolver(
        std::vector<double> spanFractions, const CollapseSpec& spec = {});

    // Advances every section. Deterministic: fixed order, no adaptive step.
    CollapseResult Step(
        CollapseState& state, const std::vector<SectionCollapseInput>& input,
        double deltaSeconds) const;

    // The pressure balance for one section, exposed because it is the whole
    // criterion and the tests check it directly rather than checking a
    // collapse that resulted from it.
    double PressureMargin(const SectionCollapseInput& section) const;

    // External pressure coefficient at the fold station, from the rounded-nose
    // cylinder distribution Level 5 already uses.
    double ExternalNoseCp(const SectionCollapseInput& section) const;

    const CollapseSpec& Spec() const { return SpecValue; }
    std::size_t SectionCount() const { return SpanFractions.size(); }

private:
    std::vector<double> SpanFractions;
    CollapseSpec SpecValue;
};
}
