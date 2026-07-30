#include "GroundEffectModel.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
GroundEffectOutput EvaluateGroundEffect(const GroundEffectInput& raw)
{
    const double dt = std::clamp(raw.deltaSeconds, 0.0, 1.0 / 30.0);
    const double span = std::max(6.0, raw.wingSpanM);
    const double clearance = std::max(0.0, raw.pilotGroundClearanceM);
    const double pressure = std::clamp(raw.canopyPressure, 0.0, 1.1);
    const double collapse =
        std::clamp(raw.collapseFraction, 0.0, 1.0);
    const double brake = std::clamp(raw.symmetricBrake, 0.0, 1.0);

    // The canopy sits above the pilot, but its downwash footprint starts to
    // interact with the surface before the projected wing centre reaches it.
    // A quarter-span offset avoids the old unphysical hard cutoff at pilot
    // height while keeping the effect modest for a high suspended canopy.
    const double effectiveWingHeight = clearance + span * 0.24;
    const double heightRatio = effectiveWingHeight / span;
    const double proximity = std::clamp(
        1.0 / (1.0 + 16.0 * heightRatio * heightRatio),
        0.0, 1.0);
    const double speedAuthority = std::clamp(
        (raw.airspeedMps - 5.5) / 7.0, 0.0, 1.0);
    const double pressureAuthority =
        pressure * (1.0 - 0.82 * collapse);
    const double availableEnergy =
        speedAuthority * pressureAuthority;

    double energy = std::clamp(raw.previousFlareEnergy, 0.0, 1.0);
    const double recharge = brake < 0.32
        ? (availableEnergy - energy) * 0.72
        : 0.0;
    const double brakeIncrement = std::clamp(
        raw.brakeApplicationRatePerS * dt * 1.25, 0.0, 1.0)
        * std::clamp((brake - 0.22) / 0.58, 0.0, 1.0);
    const double heldDeep = std::clamp(
        (brake - 0.48) / 0.42, 0.0, 1.0);

    double flareLift = std::clamp(raw.previousFlareLift, 0.0, 1.0);
    flareLift += brakeIncrement * energy;
    flareLift -= flareLift * (1.15 + 1.35 * heldDeep) * dt;
    flareLift = std::clamp(flareLift, 0.0, 1.0);
    energy += recharge * dt;
    energy -= brakeIncrement * 0.55
        + heldDeep * flareLift * 0.42 * dt;
    energy = std::clamp(energy, 0.0, 1.0);

    // A flare still changes incidence away from the ground, but the useful
    // landing cushion is strongest in proximity. Descending airframes retain
    // slightly more authority than a zooming/climbing one.
    const double descentAuthority = std::clamp(
        (-raw.verticalSpeedMps + 0.2) / 2.5, 0.35, 1.0);
    const double flareAuthority = flareLift * availableEnergy
        * descentAuthority * (0.28 + 0.72 * proximity);
    const double flareDeltaCl = 0.34 * flareAuthority;
    const double inducedDragReduction =
        0.24 * proximity * pressureAuthority;

    return {
        proximity,
        inducedDragReduction,
        energy,
        flareLift,
        flareDeltaCl,
        flareAuthority
    };
}
}
