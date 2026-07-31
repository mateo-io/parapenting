#include "PayloadRigidBody.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
// Radius of gyration of a seated pilot plus harness, metres. Roll is the
// tighter axis: the mass is stacked vertically about the spine.
constexpr double RollRadiusOfGyrationM = 0.24;
constexpr double PitchRadiusOfGyrationM = 0.31;
}

double PayloadRollInertiaKgM2(const PayloadMassProperties& mass)
{
    return mass.TotalKg() * RollRadiusOfGyrationM * RollRadiusOfGyrationM;
}

double PayloadPitchInertiaKgM2(const PayloadMassProperties& mass)
{
    return mass.TotalKg() * PitchRadiusOfGyrationM * PitchRadiusOfGyrationM;
}

PayloadLoads SettledPayloadLoads(
    const PayloadMassProperties& mass, const HarnessGeometry& harness,
    const PayloadInput& input)
{
    PayloadLoads loads;
    const double separation = std::max(0.05, harness.carabinerSeparationM);
    const double load = std::max(0.0, input.suspendedLoadN);

    loads.cgOffsetM = WeightShiftCgOffsetM(harness, input.weightShift);

    // In a turn the payload hangs along the apparent vertical, not the true
    // one, so part of the hip movement is spent catching up with the swing.
    // This is why weight shift feels progressively less effective the deeper
    // a spiral gets, and it needs no separate rule to say so.
    const double apparentTilt = std::atan2(
        input.lateralAccelerationMps2, std::max(1.0, input.gravityMps2));
    loads.effectiveCgOffsetM =
        loads.cgOffsetM - harness.carabinerAboveCgM * std::tan(apparentTilt);

    // Statics of a mass hung from two points: take moments about the left
    // carabiner. Nothing is fitted here - the split is the lever arm.
    const double offsetRatio = std::clamp(
        loads.effectiveCgOffsetM / separation, -0.5, 0.5);
    loads.rightCarabinerN = load * (0.5 + offsetRatio);
    loads.leftCarabinerN = load * (0.5 - offsetRatio);
    loads.loadAsymmetry = load > 1e-6
        ? (loads.rightCarabinerN - loads.leftCarabinerN) / load : 0.0;

    // The moment the payload puts into the wing is its weight acting at the
    // offset. Right-side-down is a negative rotation about body +X under the
    // right-handed quaternion algebra, so the sign is explicit here rather
    // than left for a caller to guess.
    loads.rollMomentNm = -load * loads.effectiveCgOffsetM;

    loads.cgOffsetLongitudinalM =
        std::clamp(input.bodyPitch, -1.0, 1.0) * harness.bodyPitchTravelM;
    loads.pitchMomentNm = load * loads.cgOffsetLongitudinalM;
    return loads;
}

PayloadLoads StepPayload(
    PayloadState& state, const PayloadMassProperties& mass,
    const HarnessGeometry& harness, const PayloadInput& input, double dt)
{
    const PayloadLoads settled =
        SettledPayloadLoads(mass, harness, input);

    // The payload swings under the carabiners as a pendulum with its own
    // period. Its restoring stiffness is the suspended load times the arm, so
    // it stiffens under g - a payload in a spiral settles far faster than one
    // in level flight, which is what a pilot feels.
    const double arm = std::max(0.05, harness.carabinerAboveCgM);
    const double rollInertia = std::max(0.5, PayloadRollInertiaKgM2(mass));
    const double pitchInertia = std::max(0.5, PayloadPitchInertiaKgM2(mass));
    const double load = std::max(1.0, input.suspendedLoadN);

    const double rollTarget =
        PayloadRollFromCgOffsetRad(harness, settled.effectiveCgOffsetM);
    const double rollStiffness = load * arm / rollInertia;
    const double rollFrequency = std::sqrt(std::max(1e-6, rollStiffness));
    // A body in a harness is well damped: webbing, the pilot's own muscles and
    // the seat all take energy out. Slightly under critical.
    constexpr double DampingRatio = 0.72;
    const double rollAcceleration =
        rollStiffness * (rollTarget - state.rollRad)
        - 2.0 * DampingRatio * rollFrequency * state.rollRateRadps;
    state.rollRateRadps += rollAcceleration * dt;
    state.rollRad += state.rollRateRadps * dt;

    const double pitchTarget = std::atan2(
        settled.cgOffsetLongitudinalM
            - arm * input.longitudinalAccelerationMps2
                / std::max(1.0, input.gravityMps2),
        arm);
    const double pitchStiffness = load * arm / pitchInertia;
    const double pitchFrequency = std::sqrt(std::max(1e-6, pitchStiffness));
    const double pitchAcceleration =
        pitchStiffness * (pitchTarget - state.pitchRad)
        - 2.0 * DampingRatio * pitchFrequency * state.pitchRateRadps;
    state.pitchRateRadps += pitchAcceleration * dt;
    state.pitchRad += state.pitchRateRadps * dt;

    // Report the loads for where the payload actually is, not where it is
    // heading: a pilot who has just moved has not yet loaded the carabiner.
    PayloadLoads loads = settled;
    const double achievedOffset = arm * std::tan(
        std::clamp(state.rollRad, -0.7, 0.7));
    const double separation = std::max(0.05, harness.carabinerSeparationM);
    const double offsetRatio =
        std::clamp(achievedOffset / separation, -0.5, 0.5);
    loads.effectiveCgOffsetM = achievedOffset;
    loads.rightCarabinerN = load * (0.5 + offsetRatio);
    loads.leftCarabinerN = load * (0.5 - offsetRatio);
    loads.loadAsymmetry =
        (loads.rightCarabinerN - loads.leftCarabinerN) / load;
    loads.rollMomentNm = -load * achievedOffset;
    const double achievedLongitudinal =
        arm * std::tan(std::clamp(state.pitchRad, -0.7, 0.7));
    loads.cgOffsetLongitudinalM = achievedLongitudinal;
    loads.pitchMomentNm = load * achievedLongitudinal;
    return loads;
}
}
