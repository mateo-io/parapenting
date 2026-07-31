#pragma once

#include <cstddef>
#include <vector>

namespace Parapenting::Physics
{
// Level 5 of the master plan: the canopy is a set of pressurised cells fed
// through holes at the leading edge.
//
// Until now internal pressure has been a constant - 65 Pa, written into the
// geometry spec as an assumption - and canopy "pressure" in the flight model
// has been a single scalar between 0 and 1 that other systems multiply things
// by. Neither knows where the air comes in, or that it stops coming in when
// the wing is unloaded.
//
// This models it as it works. Each cell is a plenum. Air enters through an
// opening near the leading edge, and how much total pressure that opening
// recovers depends on where it sits relative to the stagnation point - which
// is not fixed, but moves along the leading edge as the section's angle of
// attack changes. Push the bar and the stagnation point moves down and aft of
// the inlets; the wing pressurises less well, which is exactly why accelerated
// flight is collapse-prone. At strongly negative incidence the inlet sits in
// suction instead of stagnation and the cell empties, which is the front
// collapse.
//
// The two pieces of physics it rests on are standard:
//
//   * the stagnation point on a rounded leading edge sits at an angular
//     position of about (alpha - alpha_L0) from the chord line, on the lower
//     surface for positive incidence;
//   * pressure around that rounded nose follows the cylinder distribution,
//     Cp = 1 - 4 sin^2 of the angle from the stagnation point. It reaches
//     zero 30 degrees away and is strongly negative beyond.
//
// What is NOT here: cell volume changing with pressure, which needs the
// membrane of Level 6, and the reduced-pressure polar family that Level 4 owes
// - so pressure does not yet feed back into section performance. Both are
// named in the exit gates as belonging elsewhere.

struct CellPressureSpec
{
    // Where the inlet sits on the leading-edge circle, radians from the chord
    // line, measured toward the lower surface. Paraglider inlets are cut
    // below the nose for exactly this reason: it puts them near the
    // stagnation point in normal flight rather than at trim only.
    double inletAngularPositionRad = 0.26;
    // Angular width of the opening.
    double inletAngularWidthRad = 0.20;
    // Open area of one cell's inlet, and the discharge coefficient of a
    // sharp-edged opening in a fabric wall.
    double inletAreaM2 = 0.010;
    double dischargeCoefficient = 0.62;
    // Fabric leakage. Coated ripstop is close to airtight, but not quite, and
    // this is what empties a cell whose inlet has stopped feeding it.
    double porosityM3PerM2sPerPa = 2.0e-6;
    double cellSurfaceAreaM2 = 1.2;
    // Rib crossports, which let neighbouring cells equalise. They are why a
    // single blocked inlet does not deflate one cell on its own.
    double crossportAreaM2 = 0.004;
    // Cell volume. Sets how long it takes to fill and to empty.
    double cellVolumeM3 = 0.22;
    // Below this fraction of the dynamic pressure a cell is treated as at
    // risk of collapse. It is a reporting threshold, not a behaviour.
    double collapseRiskPressureCoefficient = 0.35;
};

struct CellPressureState
{
    // Gauge pressure inside each cell, pascals above ambient.
    std::vector<double> gaugePressurePa;
    // How much of each cell's volume has actually been filled, 0 to 1.
    //
    // This is what makes inflation take the time it does. At a fixed volume a
    // cell reaches flying pressure almost instantly - 66 Pa is six parts in
    // ten thousand of atmospheric, so a fifth of a cubic metre of cell needs
    // about a seventh of a litre of air to get there, which an inlet delivers
    // in two milliseconds. What takes seconds is moving the volume itself:
    // filling a packed cell means passing its whole volume through the
    // opening. Pressure only builds once it is full.
    std::vector<double> filledFraction;
    bool initialised = false;
};

struct CellPressureInput
{
    // Per cell, from the aerodynamic solve: the local dynamic pressure and
    // the local angle of attack. Driving this from the VSM's own per-section
    // incidence is what makes the stagnation point move for the right reason.
    std::vector<double> dynamicPressurePa;
    std::vector<double> angleOfAttackRad;
    // The section's zero-lift angle, which is where the stagnation point sits
    // on the chord line. Brake moves it, so brake moves the inlets' feed.
    double zeroLiftAngleRad = -0.07;
};

struct CellPressureResult
{
    // Gauge pressure per cell, pascals.
    std::vector<double> gaugePressurePa;
    // Internal pressure coefficient per cell: gauge over local dynamic
    // pressure. The single most important scalar for collapse risk, so it is
    // first-class output rather than something a caller has to derive.
    std::vector<double> pressureCoefficient;
    // How full each cell is, 0 to 1. A cell that is not yet full holds no
    // pressure however hard its inlet is feeding it.
    std::vector<double> filledFraction;
    // What the inlet is seeing: the pressure coefficient at its position, and
    // how far it sits from the stagnation point.
    std::vector<double> inletPressureCoefficient;
    std::vector<double> inletOffsetFromStagnationRad;
    // Cells whose inlet is in suction rather than stagnation - air leaving
    // rather than entering.
    int cellsWithReversedInflow = 0;
    int cellsBelowCollapseRisk = 0;
    double minimumPressureCoefficient = 0.0;
    double meanPressureCoefficient = 0.0;
};

class CanopyPressureSolver
{
public:
    explicit CanopyPressureSolver(
        int cellCount, const CellPressureSpec& spec = {});

    // Advances the cell pressures by deltaSeconds and reports them. An
    // uninitialised state starts empty, so the first seconds are an
    // inflation.
    CellPressureResult Step(
        CellPressureState& state, const CellPressureInput& input,
        double deltaSeconds) const;

    // Where the stagnation point sits for a given incidence, radians from the
    // chord line toward the lower surface.
    double StagnationAngleRad(
        double angleOfAttackRad, double zeroLiftAngleRad) const;

    // Pressure coefficient at an angular distance from the stagnation point,
    // on the rounded nose.
    static double NosePressureCoefficient(double angleFromStagnationRad);

    int CellCount() const { return Cells; }
    const CellPressureSpec& Spec() const { return SpecValue; }

private:
    int Cells = 0;
    CellPressureSpec SpecValue{};
};
}
