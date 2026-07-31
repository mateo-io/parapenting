#pragma once

namespace Parapenting::Physics
{
// Level 3 of the master plan: the harness is geometry, not a gain.
//
// Weight shift used to be a number multiplied by weightShiftAuthority. That
// hid the actual mechanism, and it hid a sign error for as long as it existed:
// the pilot's mass never moved, no force was ever applied at a carabiner, and
// the roll it produced shared a variable with the aerodynamic load asymmetry -
// which pushes the opposite way. A wing whose lift is biased right rises on
// the right; a wing whose carabiner is loaded right is pulled down on the
// right. One number could not mean both, so one of brake and weight shift was
// always inverted.
//
// This file describes the harness the pilot is actually sitting in, and
// PayloadRigidBody turns that into carabiner loads.

enum class HarnessClass
{
    // Rigid seat plate. The pilot and the seat roll together, so hip movement
    // moves a lot of mass but the harness resists rolling.
    SeatPlate,
    // No plate: string or pod. The pilot's mass moves more freely and the
    // harness rolls with far less input, which is why these are the higher
    // weight-shift-authority harnesses and also the twitchier ones.
    NoPlate,
};

struct HarnessGeometry
{
    HarnessClass harnessClass = HarnessClass::SeatPlate;
    // Carabiner separation. The lever the payload's weight acts on, so it sets
    // how much load asymmetry a given CG offset produces.
    double carabinerSeparationM = 0.42;
    // Chest strap setting, measured between the two riser attachment points.
    // Narrower couples the pilot to the wing more tightly and lets the whole
    // payload roll further for the same hip movement; wider damps it. Typical
    // range on a B wing is 0.38 to 0.50 m.
    double chestStrapM = 0.44;
    // Carabiners above the seated pilot's CG. This is the payload's own
    // pendulum arm, distinct from the 7.3 m line length.
    double carabinerAboveCgM = 0.28;
    // How far the pilot's CG can move laterally at full hip travel, before
    // the harness class and chest strap modify it.
    double hipTravelM = 0.075;
    // Longitudinal CG travel from body pose: leaning forward and back.
    double bodyPitchTravelM = 0.11;
};

// Lateral movement of the pilot CG at full weight shift, metres, for this
// harness. Positive is toward the right carabiner.
//
// The chest strap and the harness class are the whole story: a narrow strap on
// a plateless harness moves nearly the full hip travel, a wide strap on a seat
// plate moves appreciably less. Nothing here is a tuning gain - change the
// strap setting and the authority changes because the geometry changed.
double WeightShiftCgOffsetM(const HarnessGeometry& harness, double weightShift);

// Roll angle the payload settles at under a given CG offset, radians, right
// side down positive. Small-angle statics: the payload hangs with its CG under
// the midpoint of the carabiners.
double PayloadRollFromCgOffsetRad(
    const HarnessGeometry& harness, double cgOffsetM);

// EPIC 2 ML research harness.
const HarnessGeometry& DefaultHarnessGeometry();
}
