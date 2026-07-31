#pragma once

#include "HarnessGeometry.h"

namespace Parapenting::Physics
{
// Level 3: the payload is a body with mass, not a rotation offset.
//
// The important consequence is that weight shift stops being a control gain.
// The pilot moves their CG, the two carabiners take unequal shares of a real
// weight, and the wing responds to that asymmetry. Because the load appears at
// the carabiners, it is separable from the aerodynamic load asymmetry, which
// acts in the opposite sense - and that separation is what makes brake and
// weight shift both roll the correct way.

struct PayloadMassProperties
{
    double pilotKg = 78.0;
    double harnessKg = 5.6;
    double reserveKg = 2.4;
    double ballastKg = 0.0;
    double equipmentKg = 3.2;

    double TotalKg() const
    {
        return pilotKg + harnessKg + reserveKg + ballastKg + equipmentKg;
    }
};

// Roll inertia of the seated payload about its own CG. A seated human plus
// harness is close to a 0.24 m radius of gyration in roll; ballast in the
// reserve pocket sits near the CG and adds little.
double PayloadRollInertiaKgM2(const PayloadMassProperties& mass);
double PayloadPitchInertiaKgM2(const PayloadMassProperties& mass);
// Same radii, for a caller that already knows the payload mass.
double PayloadRollInertiaForMassKgM2(double payloadMassKg);
double PayloadPitchInertiaForMassKgM2(double payloadMassKg);

struct PayloadInput
{
    double weightShift = 0.0;
    // Pilot leaning fore and aft, -1 back to +1 forward.
    double bodyPitch = 0.0;
    // Total load the lines are carrying, newtons. In steady flight this is the
    // system weight; in a spiral it is several g.
    double suspendedLoadN = 1030.0;
    // Load factor at the payload, g. Weight shift loses authority as this
    // rises, because a pilot pressed into the harness at 3 g cannot move
    // their hips as far - at that load the same shift is most of a bodyweight
    // lift. It is a mobility limit, not a swing: in a coordinated turn the
    // harness already hangs along apparent gravity, so there is no apparent
    // vertical for the pilot to be catching up with.
    double loadFactor = 1.0;
    double longitudinalAccelerationMps2 = 0.0;
    double gravityMps2 = 9.80665;
};

struct PayloadState
{
    // Roll of the payload relative to the carabiner line, right side down
    // positive.
    double rollRad = 0.0;
    double rollRateRadps = 0.0;
    double pitchRad = 0.0;
    double pitchRateRadps = 0.0;
};

struct PayloadLoads
{
    // What each carabiner is actually holding, newtons. These are forces at a
    // point on the harness, not shares of an abstraction.
    double leftCarabinerN = 0.0;
    double rightCarabinerN = 0.0;
    // (right - left) / total. Positive means the right carabiner carries more.
    double loadAsymmetry = 0.0;
    // Lateral offset of the pilot CG actually achieved, metres, and the offset
    // measured against the apparent vertical, which is what produces load.
    double cgOffsetM = 0.0;
    double effectiveCgOffsetM = 0.0;
    // Roll moment this asymmetry applies to the canopy, N m, about body +X.
    // Negative is right-tip-down, matching the right-handed algebra used by
    // the attitude quaternion.
    double rollMomentNm = 0.0;
    // Longitudinal CG offset from body pose and the pitch moment it makes.
    double cgOffsetLongitudinalM = 0.0;
    double pitchMomentNm = 0.0;
};

// Steps the payload's own pendulum and returns the loads it puts into the
// carabiners. The payload hangs below the carabiners, so it has a pendulum of
// its own that is much faster than the 7.3 m line pendulum, and it is that
// fast one the pilot feels as the harness settling under them.
PayloadLoads StepPayload(
    PayloadState& state, const PayloadMassProperties& mass,
    const HarnessGeometry& harness, const PayloadInput& input, double dt);

// Static solution, for tests and for the suspension solver: no pendulum
// transient, just where the loads settle.
PayloadLoads SettledPayloadLoads(
    const PayloadMassProperties& mass, const HarnessGeometry& harness,
    const PayloadInput& input);
}
