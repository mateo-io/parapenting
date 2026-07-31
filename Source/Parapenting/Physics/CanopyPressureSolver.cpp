#include "CanopyPressureSolver.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
// Air density used to turn a pressure difference into a flow speed through an
// opening. The solver works in gauge pressure against a local dynamic
// pressure, so this is the only place absolute density enters.
constexpr double AirDensityKgM3 = 1.12;

// Flow speed through an orifice under a pressure difference, signed: negative
// means the flow is running the other way, out of the cell.
double OrificeFlowSpeed(double pressureDifferencePa)
{
    const double magnitude = std::sqrt(
        2.0 * std::fabs(pressureDifferencePa) / AirDensityKgM3);
    return pressureDifferencePa >= 0.0 ? magnitude : -magnitude;
}
}

CanopyPressureSolver::CanopyPressureSolver(
    int cellCount, const CellPressureSpec& spec)
    : Cells(std::max(1, cellCount)), SpecValue(spec)
{
}

double CanopyPressureSolver::StagnationAngleRad(
    double angleOfAttackRad, double zeroLiftAngleRad) const
{
    // On a rounded leading edge the stagnation point sits below the chord
    // line by about the angle the section is flying at above its own zero-lift
    // angle. At trim that puts it close to where the inlets are cut; on bar it
    // moves up and away from them, which is the geometric reason accelerated
    // flight pressurises less well.
    return angleOfAttackRad - zeroLiftAngleRad;
}

double CanopyPressureSolver::NosePressureCoefficient(
    double angleFromStagnationRad)
{
    // The cylinder distribution: one at the stagnation point, zero thirty
    // degrees away, strongly negative beyond. A rounded nose is close enough
    // to a cylinder over the small angles an inlet lives in.
    const double s = std::sin(angleFromStagnationRad);
    return 1.0 - 4.0 * s * s;
}

CellPressureResult CanopyPressureSolver::Step(
    CellPressureState& state, const CellPressureInput& input,
    double deltaSeconds) const
{
    const auto cells = static_cast<std::size_t>(Cells);
    CellPressureResult result;
    result.gaugePressurePa.assign(cells, 0.0);
    result.pressureCoefficient.assign(cells, 0.0);
    result.inletPressureCoefficient.assign(cells, 0.0);
    result.inletOffsetFromStagnationRad.assign(cells, 0.0);
    result.filledFraction.assign(cells, 0.0);

    if (!state.initialised || state.gaugePressurePa.size() != cells
        || state.filledFraction.size() != cells)
    {
        // A packed wing starts empty. Inflation is then something that
        // happens, rather than a state the simulation begins in.
        state.gaugePressurePa.assign(cells, 0.0);
        state.filledFraction.assign(cells, 0.0);
        state.initialised = true;
    }

    const double dt = std::max(0.0, deltaSeconds);
    std::vector<double> updated = state.gaugePressurePa;
    std::vector<double> filled = state.filledFraction;

    const auto sampleAt = [&](const std::vector<double>& values,
                              std::size_t index, double fallback)
    {
        if (values.empty()) return fallback;
        return values[std::min(index, values.size() - 1)];
    };

    for (std::size_t cell = 0; cell < cells; ++cell)
    {
        const double dynamicPressure = std::max(0.0,
            sampleAt(input.dynamicPressurePa, cell, 0.0));
        const double alpha = sampleAt(input.angleOfAttackRad, cell, 0.0);

        // Where the stagnation point is, and therefore what the inlet sees.
        const double stagnation =
            StagnationAngleRad(alpha, input.zeroLiftAngleRad);
        const double offset =
            SpecValue.inletAngularPositionRad - stagnation;
        const double inletCp = NosePressureCoefficient(offset);
        result.inletOffsetFromStagnationRad[cell] = offset;
        result.inletPressureCoefficient[cell] = inletCp;

        // The pressure the inlet is trying to drive the cell to.
        const double inletPressure = inletCp * dynamicPressure;
        const double internal = updated[cell];
        const double difference = inletPressure - internal;

        // Volume flow through the inlet, signed. When the inlet has moved
        // into suction this is negative and the cell empties through its own
        // opening - the front collapse, with no rule that says so.
        const double inletSpeed = OrificeFlowSpeed(difference);
        const double inletFlow = SpecValue.dischargeCoefficient
            * SpecValue.inletAreaM2 * inletSpeed;
        if (inletFlow < 0.0) ++result.cellsWithReversedInflow;

        // Leakage through the fabric, always outward while the cell is above
        // ambient.
        const double leakFlow = SpecValue.porosityM3PerM2sPerPa
            * SpecValue.cellSurfaceAreaM2 * internal;

        // Crossports to the neighbours on each side. This is what stops one
        // blocked inlet emptying a single cell, and what lets a collapsed
        // region be re-fed from the cells beside it.
        double crossFlow = 0.0;
        for (int side = -1; side <= 1; side += 2)
        {
            const auto neighbourIndex = static_cast<std::ptrdiff_t>(cell) + side;
            if (neighbourIndex < 0
                || neighbourIndex >= static_cast<std::ptrdiff_t>(cells))
                continue;
            const double neighbour =
                updated[static_cast<std::size_t>(neighbourIndex)];
            crossFlow += SpecValue.dischargeCoefficient
                * SpecValue.crossportAreaM2
                * OrificeFlowSpeed(neighbour - internal);
        }

        const double netFlow = inletFlow - leakFlow + crossFlow;

        // A cell that is not yet full puts everything it takes in into
        // filling itself, and holds no pressure while it does. This is the
        // part of inflation that takes seconds.
        if (filled[cell] < 1.0)
        {
            filled[cell] = std::clamp(
                filled[cell] + netFlow * dt
                    / std::max(1.0e-4, SpecValue.cellVolumeM3),
                0.0, 1.0);
            updated[cell] = 0.0;
            continue;
        }

        // Full, so further net inflow is compression. The bulk modulus of air
        // converts a volume imbalance into a pressure rate:
        // dp/dt = (gamma p_atm / V) * Q. That is very stiff - a full cell
        // reaches flying pressure in milliseconds - so the step is not
        // allowed to carry the pressure past the equilibrium it is heading
        // for. Without that guard a 120 Hz step overshoots by a factor of
        // five and the wing reports twelve times its own dynamic pressure.
        constexpr double AtmosphericPressurePa = 101325.0;
        constexpr double HeatCapacityRatio = 1.4;
        const double stiffness = HeatCapacityRatio * AtmosphericPressurePa
            / std::max(1.0e-4, SpecValue.cellVolumeM3);
        double next = internal + stiffness * netFlow * dt;

        // The inlet cannot drive the cell past what it is recovering, and
        // cannot suck it below that either.
        if (netFlow > 0.0) next = std::min(next, std::max(internal,
                                                         inletPressure));
        else next = std::max(next, std::min(internal,
                                            std::max(0.0, inletPressure)));

        // Emptying below zero means the cell has started to deflate, which is
        // volume leaving rather than pressure falling further.
        if (next <= 0.0 && netFlow < 0.0)
        {
            filled[cell] = std::clamp(
                filled[cell] + netFlow * dt
                    / std::max(1.0e-4, SpecValue.cellVolumeM3),
                0.0, 1.0);
        }
        updated[cell] = std::max(0.0, next);
    }

    state.gaugePressurePa = updated;
    state.filledFraction = filled;
    result.gaugePressurePa = updated;
    result.filledFraction = filled;

    // Telemetry. The internal pressure coefficient is what collapse risk is
    // read from, so it is computed here rather than left to a caller.
    double sum = 0.0;
    result.minimumPressureCoefficient = 1.0e9;
    for (std::size_t cell = 0; cell < cells; ++cell)
    {
        const double dynamicPressure =
            sampleAt(input.dynamicPressurePa, cell, 0.0);
        const double coefficient = dynamicPressure > 1.0e-6
            ? updated[cell] / dynamicPressure : 0.0;
        result.pressureCoefficient[cell] = coefficient;
        sum += coefficient;
        result.minimumPressureCoefficient =
            std::min(result.minimumPressureCoefficient, coefficient);
        if (coefficient < SpecValue.collapseRiskPressureCoefficient)
            ++result.cellsBelowCollapseRisk;
    }
    result.meanPressureCoefficient = sum / static_cast<double>(cells);
    if (result.minimumPressureCoefficient > 1.0e8)
        result.minimumPressureCoefficient = 0.0;
    return result;
}
}
