#include "ParagliderDynamics.h"
#include "SpanwiseCanopyModel.h"
#include "GroundEffectModel.h"
#include "SuspensionSystem.h"

namespace Parapenting::Physics
{
Quaternion Quaternion::operator*(const Quaternion& rhs) const
{
    return {
        w*rhs.w - x*rhs.x - y*rhs.y - z*rhs.z,
        w*rhs.x + x*rhs.w + y*rhs.z - z*rhs.y,
        w*rhs.y - x*rhs.z + y*rhs.w + z*rhs.x,
        w*rhs.z + x*rhs.y - y*rhs.x + z*rhs.w
    };
}

Quaternion Quaternion::Normalized() const
{
    const double length = std::sqrt(w*w + x*x + y*y + z*z);
    return length > 1e-12 ? Quaternion{w/length, x/length, y/length, z/length}
                          : Quaternion{};
}

Vec3 Quaternion::Rotate(const Vec3& value) const
{
    const Quaternion p{0.0, value.x, value.y, value.z};
    const Quaternion inverse{w, -x, -y, -z};
    const Quaternion result = (*this) * p * inverse;
    return {result.x, result.y, result.z};
}

Vec3 Quaternion::InverseRotate(const Vec3& value) const
{
    return Quaternion{w, -x, -y, -z}.Rotate(value);
}

ParagliderDynamics::ParagliderDynamics(WingParameters parameters)
    : Params(parameters)
{
}

void ParagliderDynamics::Step(FlightState& state, const ControlInput& rawControls,
                              const Atmosphere& atmosphere, double deltaSeconds)
{
    const double dt = std::clamp(deltaSeconds, 0.0, 1.0 / 60.0);
    if (dt <= 0.0) return;

    const ControlInput commanded{
        std::clamp(rawControls.leftBrake, 0.0, 1.0),
        std::clamp(rawControls.rightBrake, 0.0, 1.0),
        std::clamp(rawControls.weightShift, -1.0, 1.0),
        std::clamp(rawControls.accelerator, 0.0, 1.0)
    };
    const auto effectiveBrake = [this](double command)
    {
        return std::clamp(
            (command - Params.brakeFreePlayFraction)
                / std::max(0.1, 1.0 - Params.brakeFreePlayFraction),
            0.0, 1.0);
    };
    const ControlInput brakeTargets{
        effectiveBrake(commanded.leftBrake),
        effectiveBrake(commanded.rightBrake),
        commanded.weightShift,
        commanded.accelerator
    };
    const auto moveBrakeTravel = [dt](double current, double target)
    {
        const double maximumRate = target > current ? 3.2 : 4.6;
        return current + std::clamp(
            target - current, -maximumRate * dt, maximumRate * dt);
    };
    state.leftBrakeTravel = std::clamp(
        moveBrakeTravel(state.leftBrakeTravel, brakeTargets.leftBrake),
        0.0, 1.0);
    state.rightBrakeTravel = std::clamp(
        moveBrakeTravel(state.rightBrakeTravel, brakeTargets.rightBrake),
        0.0, 1.0);
    const ControlInput controls{
        state.leftBrakeTravel,
        state.rightBrakeTravel,
        brakeTargets.weightShift,
        brakeTargets.accelerator
    };
    const Vec3 relativeWindWorld =
        atmosphere.windWorldMps - state.velocityWorldMps;
    const Vec3 airflowBody = state.attitude.InverseRotate(relativeWindWorld);
    const double speed = std::max(Length(airflowBody), 0.01);
    const double bodyAlpha = std::atan2(-airflowBody.z, -airflowBody.x);
    // THE INCIDENCE THE CANOPY SEES, WHICH IS NOT THE ONE THE BODY SEES.
    //
    // The wing hangs on lines and swings relative to the pilot - that angle is
    // `canopyRelativePitchRad`, integrated below as a real pendulum. A canopy
    // swung 30 degrees behind the pilot has its chord rotated 30 degrees
    // nose-up against the same airflow, so its incidence is 30 degrees higher.
    // Until this line existed the swing was carried in the pose and nowhere
    // else: through a firm brake pulse the wing went 32 degrees back while the
    // incidence the model flew on went DOWN, 6 degrees below trim to 17. The
    // wing swung and the aircraft did not notice, which is why a pendulum
    // could be seen and never felt.
    //
    // Closing it is what makes the two ends of the swing different things
    // rather than one signal with two signs:
    //
    //   * BACK, incidence UP. The wing decelerates, the pilot keeps going, the
    //     canopy pitches aft and its incidence rises toward the stall. This is
    //     how a fast deep brake, a thermal entry or the recovery half of a
    //     surge puts a wing into a stall, and it is why the recovery is to let
    //     the brakes up rather than to hold them.
    //   * FRONT, incidence DOWN. The wing accelerates ahead of the pilot on a
     //    release or out of a thermal, its incidence falls, and at the bottom
    //     of that swing it is closest to a frontal collapse - which is why a
    //     surge is checked with brake and not with nothing.
    //
    // The previous step's swing is used deliberately: the pendulum is
    // integrated after the aerodynamics, so reading this step's value would be
    // an algebraic loop. At 120 Hz the lag is one step against a pendulum
    // period of seconds.
    //
    // The gain is 1.0 because it is geometry, not a handling number - a
    // rotation of the chord IS a change of incidence. It exists as a parameter
    // so the claim can be swept rather than asserted.
    const double canopySwingRad = state.canopyRelativePitchRad;
    const double alpha = bodyAlpha
        + Params.swingIncidenceGain * canopySwingRad;
    const double dynamicPressure =
        0.5 * Params.airDensityKgM3 * speed * speed;
    const double pressureDropRate = state.previousDynamicPressurePa > 1.0
        ? std::max(0.0,
            (state.previousDynamicPressurePa - dynamicPressure) / dt)
        : 0.0;
    const double incidenceUnloading = std::clamp(
        (Params.trimAngleOfAttackRad - alpha - 0.045) / 0.22,
        0.0, 1.0);
    const double pressureShock = std::clamp(
        pressureDropRate / 520.0, 0.0, 1.0);
    const double lowDynamicPressure = std::clamp(
        (38.0 - dynamicPressure) / 28.0, 0.0, 1.0);
    const double disturbance = std::clamp(
        std::max(atmosphere.turbulence,
            std::abs(atmosphere.lateralGust) / 3.0), 0.0, 1.0);
    const double aerodynamicUnloading = std::clamp(
        incidenceUnloading * (0.02 + 0.98 * disturbance)
            + pressureShock * (0.10 + 0.90 * disturbance)
            + lowDynamicPressure * disturbance * 0.32,
        0.0, 1.0);
    state.previousDynamicPressurePa = dynamicPressure;
    state.previousAngleOfAttackRad = alpha;
    const double symmetricBrake = 0.5 * (controls.leftBrake + controls.rightBrake);
    const double asymmetricBrake = controls.rightBrake - controls.leftBrake;
    const double leftBrakeRate =
        (controls.leftBrake - state.previousLeftBrake) / dt;
    const double rightBrakeRate =
        (controls.rightBrake - state.previousRightBrake) / dt;
    const double leftPumpImpulse = std::clamp(leftBrakeRate, 0.0, 50.0)
        * std::clamp(controls.leftBrake - 0.22, 0.0, 0.55);
    const double rightPumpImpulse = std::clamp(rightBrakeRate, 0.0, 50.0)
        * std::clamp(controls.rightBrake - 0.22, 0.0, 0.55);
    state.leftPumpEnergy = std::clamp(
        state.leftPumpEnergy + leftPumpImpulse * dt
            - state.leftPumpEnergy * 1.65 * dt, 0.0, 1.0);
    state.rightPumpEnergy = std::clamp(
        state.rightPumpEnergy + rightPumpImpulse * dt
            - state.rightPumpEnergy * 1.65 * dt, 0.0, 1.0);
    state.previousLeftBrake = controls.leftBrake;
    state.previousRightBrake = controls.rightBrake;
    // HOW FAR PAST THE DEPARTURE THE BRAKE ACTUALLY IS, CONTINUOUS. This was a
    // step at 0.86, and the step is item 24's defect: it made the whole top
    // eighth of the brake range one single stall. Measured on this model, a
    // three-second hold at 0.88 / 0.92 / 0.96 / 1.00 brake gave stalled sinks
    // of 5.58 / 5.52 / 5.50 / 5.43 m/s and first surges of 11.80 / 11.86 /
    // 12.00 / 12.02 m/s - a 1.9% spread across a control range the pilot moves
    // by hand, which is why every stall felt like the same stall.
    //
    // Depth now sets how fast the wing goes there as well as how far, so a
    // gentler stall also takes longer to arrive. Full brake is unchanged at
    // both ends: depth 1.0, target 1.0, accrual 1.0/s.
    const double brakeStallDepth = std::clamp(
        (symmetricBrake - 0.78) / 0.22, 0.0, 1.0);
    if (brakeStallDepth > 0.0)
        state.deepBrakeTime = std::min(4.0,
            state.deepBrakeTime + dt * brakeStallDepth);
    else
        state.deepBrakeTime = std::max(0.0, state.deepBrakeTime - dt * 1.8);
    // `deepStall` stays bounded at 1.0 - a dozen consumers multiply by it and
    // several would change sign above one - but its TARGET is no longer
    // binary, so a deep stall and a marginal one are no longer the same state.
    // Full brake still targets 1.0, which is what keeps every shipped
    // full-brake number where it was.
    const double deepStallTarget = state.deepBrakeTime > 0.9
        ? std::clamp(0.55 + 0.45 * brakeStallDepth, 0.0, 1.0)
        : 0.0;
    const double deepStallRate = deepStallTarget > state.deepStall ? 1.7 : 0.72;
    state.deepStall += (deepStallTarget - state.deepStall)
                     * std::min(1.0, dt * deepStallRate);
    if (symmetricBrake < 0.22)
        state.deepStall = std::max(0.0, state.deepStall - dt * 0.55);
    const double spinTarget =
        std::abs(asymmetricBrake) > 0.64 && symmetricBrake > 0.42 ? asymmetricBrake : 0.0;
    state.spin += (spinTarget - state.spin)
                * std::min(1.0, dt * (spinTarget == 0.0 ? 1.1 : 2.1));
    const double brakeApplicationRate = std::clamp(
        (symmetricBrake - state.previousSymmetricBrake) / dt, 0.0, 50.0);
    state.previousSymmetricBrake = symmetricBrake;
    const double speedEnergy = std::clamp(
        (speed - 9.0) / 7.0, 0.0, 1.0);
    const double zoomInjection = std::clamp(
        brakeApplicationRate / 3.2, 0.0, 1.0)
        * std::clamp((symmetricBrake - 0.18) / 0.58, 0.0, 1.0)
        * speedEnergy;
    constexpr double EpicTrimSpeedMps = 39.0 / 3.6;
    const double availableKineticEnergyJ = 0.5 * Params.allUpMassKg
        * std::max(0.0,
            speed * speed - EpicTrimSpeedMps * EpicTrimSpeedMps);
    const double capturedEnergyJ = zoomInjection
        * availableKineticEnergyJ * dt * 3.2;
    state.brakeZoomEnergyJ = std::clamp(
        state.brakeZoomEnergyJ + capturedEnergyJ,
        0.0, availableKineticEnergyJ * 0.72);
    state.brakeZoomEnergy = std::clamp(
        state.brakeZoomEnergyJ
            / std::max(900.0, availableKineticEnergyJ * 0.72),
        0.0, 1.0);
    const double brakeZoomLiftCoefficient =
        1.15 * state.brakeZoomEnergy * (1.0 - 0.78 * state.deepStall);
    // Canopy pitch relative to the suspended payload. Suspension length sets
    // the natural pendulum frequency; brake/zoom displacement shifts its
    // equilibrium aft. This state can overshoot on release instead of
    // teleporting with the rigid system attitude.
    // Measured, not sqrt(g/L) - see `canopyPitchPeriodS`. A wing's pitch
    // oscillation is aerodynamic; only the pilot's swing under the carabiners
    // is a gravity pendulum, and that one is stepped separately below and
    // still uses the line length.
    const double pitchNaturalFrequency = 2.0 * 3.141592653589793
        / std::max(0.2, Params.canopyPitchPeriodS);
    // Where the wing hangs relative to the pilot, and it is not a function of
    // the brake handle. The pilot is 95 kg of a 105 kg aircraft, so the pilot
    // is very nearly the centre of mass and the WING is what moves: the pair
    // hangs along apparent gravity, and apparent gravity tilts by the
    // aircraft's own longitudinal acceleration. Positive pitches the canopy
    // aft, which is what happens when the wing decelerates and the pilot
    // keeps going.
    //
    // This used to be `symmetricBrake * 0.14 + zoom * 0.16 - 0.55 *
    // harnessPitch` - a scripted target in the control input, capped at 8
    // degrees, with no dependence on what the aircraft was actually doing. It
    // is why the pilot visibly swung and the wing did not, and why there was
    // no surge on release: the two ends of one angle were two different
    // scripts, and only one of them was driven by the flight.
    //
    // It is the same expression as the harness pendulum's target above, and
    // deliberately so. There is one relative angle between wing and pilot;
    // this is that angle, seen from the wing.
    // Bounded at 34 degrees, which is about 4 m of arc on this wing. A real
    // canopy does go a long way back behind the pilot under a fast, deep
    // brake application - that is exactly what loads the surge that follows -
    // but this legacy model's brake deceleration is fierce enough to hold the
    // pendulum near its stop for a full second, which a wing does not. The
    // bound is a stated limit of the legacy path, not a handling number: the
    // coupled solver has the same angle as a real degree of freedom and needs
    // no limit at all.
    // A CANDIDATE SECOND TERM WAS TRIED HERE AND MEASURED OUT, and the reason
    // is worth more than the term. `previousHangTiltRad` is an ACCELERATION,
    // so it decays to zero once the stalled descent steadies and the canopy
    // drifts forward while the wing is still fully stalled: at full brake the
    // release angle falls from 0.843 rad after a 3 s hold to 0.695 after 5 s.
    // That looks like the defect, and a severity-proportional aft bias holds
    // the canopy back through the stall exactly as intended.
    //
    // It buys nothing. The duration axis was already monotone once the first
    // surge is measured as an excursion from the release speed rather than as
    // a peak-to-peak between extrema - 12.02 m/s at 3 s against 12.38 at 5 s -
    // and the inversion that motivated the term was the metric's artefact, not
    // the model's. What the bias did do is hold the wing aft into the
    // recovery, turning a dive-and-ring into one long dive: ringing cycles
    // fell from 6 to 2. It is removed. Item 24's other criterion is that a
    // recovery shows more than one cycle, and a term that halves them to fix
    // a measurement error is a bad trade.
    const double canopyPitchTarget =
        std::clamp(state.previousHangTiltRad, -0.60, 0.60);
    const double relativePitchAcceleration =
        pitchNaturalFrequency * pitchNaturalFrequency
            * (canopyPitchTarget - state.canopyRelativePitchRad)
        - Params.canopyPitchDampingRatio * 2.0 * pitchNaturalFrequency
            * state.canopyRelativePitchRateRadps;
    state.canopyRelativePitchRateRadps +=
        relativePitchAcceleration * dt;
    // Wide enough that a real surge is not shaved off by the limit. The old
    // range stopped at 0.38 rad, which a hard brake release reaches on its
    // own and a collapse recovery goes well past - so the most visible pitch
    // event a pilot ever sees was being clipped at the top. What bounds it
    // now is the physics: the lines cannot push, so the wing cannot swing
    // above the pilot's own level.
    state.canopyRelativePitchRad = std::clamp(
        state.canopyRelativePitchRad
            + state.canopyRelativePitchRateRadps * dt,
        -0.85, 0.85);
    const double wingSpanM = std::sqrt(Params.areaM2 * 5.2);
    const GroundEffectOutput landingAero = EvaluateGroundEffect({
        atmosphere.groundClearanceM,
        wingSpanM,
        speed,
        state.velocityWorldMps.z,
        symmetricBrake,
        brakeApplicationRate,
        state.canopyPressure,
        std::max(state.frontalCollapse,
            0.5 * (state.leftCollapse + state.rightCollapse)),
        state.flareEnergy,
        state.flareLift,
        dt
    });
    state.flareEnergy = landingAero.flareEnergy;
    state.flareLift = landingAero.flareLiftState;
    const double groundEffect = landingAero.proximity;
    const double flareBoost = landingAero.flareLiftCoefficient;

    // Level 3: the pilot is a mass on a harness, not a roll animation. Hip
    // movement offsets the payload CG, the two carabiners take unequal shares
    // of a real weight, and everything downstream reads those forces.
    PayloadInput payloadInput;
    payloadInput.weightShift = controls.weightShift;
    // The lines carry the payload; the canopy is above them. Mass comes from
    // the wing parameters, which the equipment setup drives, and its
    // distribution from PayloadMassProperties.
    const double payloadMassKg = std::max(
        25.0, Params.allUpMassKg - Params.canopyMassKg);
    payloadInput.suspendedLoadN =
        std::max(1.0, payloadMassKg * 9.80665
            * std::max(0.2, state.previousLoadFactor));
    payloadInput.loadFactor = state.previousLoadFactor;
    payloadInput.longitudinalAccelerationMps2 =
        state.previousLongitudinalAccelerationMps2;
    PayloadMassProperties payloadMass = PayloadMass;
    // Keep the payload body's total consistent with the configured all-up
    // mass: the distribution is PayloadMassProperties', the total is the
    // equipment setup's, and they must not be two different answers.
    payloadMass.pilotKg += payloadMassKg - payloadMass.TotalKg();
    const PayloadLoads payload = StepPayload(
        state.payload, payloadMass, HarnessShape, payloadInput, dt);
    // The payload is a suspended mass, not a pitch animation. Its
    // longitudinal equilibrium follows the previous-step canopy/system
    // acceleration: it swings forward under deceleration and aft under
    // acceleration. The suspension length sets the pendulum period.
    const double longitudinalPendulumFrequency =
        std::sqrt(9.80665 / SuspensionLengthM);
    const double harnessPitchTarget = std::clamp(
        std::atan2(
            -state.previousLongitudinalAccelerationMps2, 9.80665),
        -0.48, 0.48);
    // The visible harness roll is the payload body's own roll; there is no
    // second model of it.
    const double harnessRollAcceleration = 0.0;
    const double harnessPitchAcceleration =
        longitudinalPendulumFrequency * longitudinalPendulumFrequency
            * (harnessPitchTarget - state.harnessPitchRad)
        - 0.62 * 2.0 * longitudinalPendulumFrequency
            * state.harnessPitchRateRadps;
    (void)harnessRollAcceleration;
    state.harnessRollRateRadps = state.payload.rollRateRadps;
    state.harnessPitchRateRadps += harnessPitchAcceleration * dt;
    state.harnessRollRad = state.payload.rollRad;
    state.harnessPitchRad += state.harnessPitchRateRadps * dt;

    // The pilot pushes a speed system through legs, harness and A/B risers.
    // Travel is deliberately compliant rather than an instantaneous polar
    // switch, which lets release energy feed back into canopy pitch.
    const double acceleratorAcceleration =
        12.0 * (controls.accelerator - state.acceleratorTravel)
        - 5.0 * state.acceleratorRate;
    state.acceleratorRate += acceleratorAcceleration * dt;
    state.acceleratorTravel = std::clamp(
        state.acceleratorTravel + state.acceleratorRate * dt, 0.0, 1.0);

    const double wingLoading = Params.allUpMassKg / std::max(10.0, Params.areaM2);
    const double pressureFromLoading = std::clamp(
        (wingLoading - 3.0) * 0.16, -0.12, 0.20);
    const double collapseDrive = (std::max(
        0.0, atmosphere.rotorStrength - 0.24)
        * (0.45 + 0.55 * atmosphere.turbulence)
        * (1.0 - pressureFromLoading)
        + aerodynamicUnloading * (0.34 + 0.34 * disturbance)
            * (1.0 - pressureFromLoading))
        / std::max(0.35, Params.collapseResistance);
    const double leftSpatialDisturbance = std::clamp(
        Length(atmosphere.leftWingWindDeltaMps) / 2.2
            + std::abs(atmosphere.leftWingWindDeltaMps.z) / 1.4,
        0.0, 1.0);
    const double rightSpatialDisturbance = std::clamp(
        Length(atmosphere.rightWingWindDeltaMps) / 2.2
            + std::abs(atmosphere.rightWingWindDeltaMps.z) / 1.4,
        0.0, 1.0);
    // A signed centre-point gust remains a useful fallback for authored
    // incidents, but zero must be neutral. Treating zero as "left" used to
    // make perfectly symmetric rotor preferentially fold that side.
    double leftLegacyBias = 0.64;
    double rightLegacyBias = 0.64;
    if (atmosphere.lateralGust > 0.05)
    {
        leftLegacyBias = 1.0;
        rightLegacyBias = 0.28;
    }
    else if (atmosphere.lateralGust < -0.05)
    {
        leftLegacyBias = 0.28;
        rightLegacyBias = 1.0;
    }
    const double leftDrive = collapseDrive * std::max(
        leftLegacyBias, 0.22 + 1.18 * leftSpatialDisturbance)
        + std::max(0.0, leftSpatialDisturbance - 0.18)
            * disturbance * 0.16;
    const double rightDrive = collapseDrive * std::max(
        rightLegacyBias, 0.22 + 1.18 * rightSpatialDisturbance)
        + std::max(0.0, rightSpatialDisturbance - 0.18)
            * disturbance * 0.16;
    // Reinflation responds most strongly to a positive brake pulse followed
    // by release. Holding deep brake starves the folded cells of dynamic
    // pressure and is deliberately less effective than pumping.
    const double leftOverBrake =
        std::clamp((controls.leftBrake - 0.68) / 0.25, 0.0, 1.0);
    const double rightOverBrake =
        std::clamp((controls.rightBrake - 0.68) / 0.25, 0.0, 1.0);
    const double halfWeightN = 0.5 * Params.allUpMassKg * 9.80665;
    const double leftSupport = std::clamp(
        (state.filteredLeftLineTensionN[0]
            + state.filteredLeftLineTensionN[1]
            + state.filteredLeftLineTensionN[2])
            / std::max(1.0, halfWeightN),
        0.12, 1.15);
    const double rightSupport = std::clamp(
        (state.filteredRightLineTensionN[0]
            + state.filteredRightLineTensionN[1]
            + state.filteredRightLineTensionN[2])
            / std::max(1.0, halfWeightN),
        0.12, 1.15);
    const double leftBrakeTransmission = std::clamp(
        state.filteredLeftLineTensionN[3] / 18.0, 0.0, 1.0);
    const double rightBrakeTransmission = std::clamp(
        state.filteredRightLineTensionN[3] / 18.0, 0.0, 1.0);
    const double leftRecovery = (Params.passiveReinflationRate
        + Params.brakeReinflationGain * controls.leftBrake
            * leftBrakeTransmission
        + Params.pumpReinflationGain * state.leftPumpEnergy
            * leftBrakeTransmission)
        * (0.45 + 0.55 * leftSupport)
        * (1.0 - 0.72 * leftOverBrake);
    const double rightRecovery = (Params.passiveReinflationRate
        + Params.brakeReinflationGain * controls.rightBrake
            * rightBrakeTransmission
        + Params.pumpReinflationGain * state.rightPumpEnergy
            * rightBrakeTransmission)
        * (0.45 + 0.55 * rightSupport)
        * (1.0 - 0.72 * rightOverBrake);
    const double previousLeftCollapse = state.leftCollapse;
    const double previousRightCollapse = state.rightCollapse;
    state.leftCollapse = std::clamp(
        state.leftCollapse + (leftDrive - leftRecovery * state.leftCollapse) * dt,
        0.0, 0.82);
    state.rightCollapse = std::clamp(
        state.rightCollapse + (rightDrive - rightRecovery * state.rightCollapse) * dt,
        0.0, 0.82);
    const double frontalDrive = std::max(0.0, atmosphere.rotorStrength - 0.52)
        * atmosphere.turbulence
        * (0.65 + 0.35 * std::abs(atmosphere.lateralGust))
        + aerodynamicUnloading
            * (0.28 + 0.46 * pressureShock + 0.24 * incidenceUnloading);
    const double frontalRecovery = Params.frontalReinflationRate
        * (1.0 - 0.78 * std::clamp(
            (symmetricBrake - 0.25) / 0.65, 0.0, 1.0));
    const double previousFrontalCollapse = state.frontalCollapse;
    state.frontalCollapse = std::clamp(
        state.frontalCollapse
            + (frontalDrive - frontalRecovery * state.frontalCollapse) * dt,
        0.0, 0.72);
    const double leftCravatDrive = state.leftCollapse > 0.66
        ? (state.leftCollapse - 0.66) * (0.35 + atmosphere.turbulence)
            * Params.cravatSusceptibility
        : 0.0;
    const double rightCravatDrive = state.rightCollapse > 0.66
        ? (state.rightCollapse - 0.66) * (0.35 + atmosphere.turbulence)
            * Params.cravatSusceptibility
        : 0.0;
    const double leftCravatRecovery =
        controls.leftBrake > 0.42 && controls.weightShift > -0.2 ? 0.16 : 0.018;
    const double rightCravatRecovery =
        controls.rightBrake > 0.42 && controls.weightShift < 0.2 ? 0.16 : 0.018;
    state.leftCravat = std::clamp(
        state.leftCravat + (leftCravatDrive
            - leftCravatRecovery * state.leftCravat) * dt, 0.0, 0.58);
    state.rightCravat = std::clamp(
        state.rightCravat + (rightCravatDrive
            - rightCravatRecovery * state.rightCravat) * dt, 0.0, 0.58);
    const double meanCollapse =
        0.5 * (state.leftCollapse + state.rightCollapse);
    const double collapseRecoveryRate = std::max(
        0.0, (state.previousMeanCollapse - meanCollapse) / dt);
    state.previousMeanCollapse = meanCollapse;
    const double surgeContainment = state.recoverySurge > 0.012
        ? std::clamp((symmetricBrake - 0.12) / 0.48, 0.0, 1.0)
            * std::clamp(state.canopyPressure / 0.65, 0.0, 1.0)
        : 0.0;
    const double surgeAcceleration =
        collapseRecoveryRate * Params.recoverySurgeGain
        - (5.0 + 4.8 * surgeContainment) * state.recoverySurge
        - (2.7 + 1.8 * surgeContainment) * state.recoverySurgeRate;
    state.recoverySurgeRate += surgeAcceleration * dt;
    state.recoverySurge += state.recoverySurgeRate * dt;
    state.recoverySurge = std::clamp(state.recoverySurge, -0.2, 0.45);

    const double alphaFromTrim = alpha - Params.trimAngleOfAttackRad;
    const bool incidenceStall =
        alphaFromTrim > Params.stallAngleRad;
    const bool stalled = symmetricBrake > 0.94
        || incidenceStall || state.deepStall > 0.5;
    double baseCl = Params.trimCl
        - Params.acceleratorLiftReduction * state.acceleratorTravel;
    if (stalled)
    {
        const double excess = std::max(
            0.0, alphaFromTrim - Params.stallAngleRad);
        baseCl *= std::max(0.22, 1.0 - 2.8 * excess);
    }
    const SpanwiseAeroResult canopy = EvaluateSpanwiseCanopy(
        Params, state, controls, baseCl,
        flareBoost + brakeZoomLiftCoefficient,
        dynamicPressure, speed, alpha, stalled,
        landingAero.inducedDragReduction, atmosphere);
    const auto updateSeparatedSpan = [dt, speed](
        double current, double target, double brake)
    {
        const bool entering = target > current;
        const double recoverySpeed = std::clamp(
            (speed - 6.0) / 5.0, 0.15, 1.0);
        const double rate = entering
            ? 5.2
            : 1.15 * recoverySpeed
                * (1.0 - 0.72 * std::clamp(brake, 0.0, 1.0));
        return std::clamp(
            current + (target - current) * std::min(1.0, rate * dt),
            0.0, 1.0);
    };
    state.leftSeparatedSpan = updateSeparatedSpan(
        state.leftSeparatedSpan, canopy.leftStalledFraction,
        controls.leftBrake);
    state.rightSeparatedSpan = updateSeparatedSpan(
        state.rightSeparatedSpan, canopy.rightStalledFraction,
        controls.rightBrake);
    const double cl = canopy.liftCoefficient;
    double cd = std::max(
        0.018, canopy.dragCoefficient
            - Params.acceleratorDragReduction * state.acceleratorTravel);
    // Porous fabric, line/harness drag, arc deformation and section
    // separation create a steep drag rise outside the operational speed
    // envelope. The basic polar alone otherwise lets prolonged turns settle
    // at rigid-wing-like speeds above 30 m/s.
    const double overspeedMps = std::max(
        0.0, speed - Params.overspeedDragOnsetMps);
    cd += Params.overspeedDragQuadratic * overspeedMps * overspeedMps;
    const Vec3 dragDirectionWorld = Normalized(relativeWindWorld);
    const Vec3 wingRightWorld = state.attitude.Rotate({0.0, 1.0, 0.0});
    Vec3 liftDirectionWorld = Normalized(Cross(wingRightWorld, dragDirectionWorld));
    if (liftDirectionWorld.z < 0.0) liftDirectionWorld = -liftDirectionWorld;

    Vec3 liftForce =
        liftDirectionWorld * (dynamicPressure * Params.areaM2 * cl);
    const Vec3 dragForce =
        dragDirectionWorld * (dynamicPressure * Params.areaM2 * cd);
    const double harnessDragMagnitude =
        dynamicPressure * Harness.dragAreaM2 * 1.05;
    const SuspensionLoads suspension = EvaluateSuspensionLoads(
        Epic2MlSuspensionGeometry(), {
            dynamicPressure,
            Params.areaM2,
            std::sqrt(
                std::pow(dynamicPressure * Params.areaM2 * cl, 2.0)
                + std::pow(dynamicPressure * Params.areaM2 * cd, 2.0)),
            state.acceleratorTravel,
            symmetricBrake,
            state.leftCollapse,
            state.rightCollapse,
            state.frontalCollapse,
            controls.leftBrake,
            controls.rightBrake,
            payload.loadAsymmetry,
            state.canopyPressure,
            canopy.loadAsymmetry
        });
    const auto filterLineGroup = [dt](
        double& tensionState, double& slackState,
        double tensionTarget, double slackTarget)
    {
        const double tensionRate =
            tensionTarget > tensionState ? 11.0 : 6.5;
        tensionState += (tensionTarget - tensionState)
            * std::min(1.0, tensionRate * dt);
        const double slackRate =
            slackTarget > slackState ? 8.5 : 13.0;
        slackState += (slackTarget - slackState)
            * std::min(1.0, slackRate * dt);
    };
    for (std::size_t group = 0; group < 4; ++group)
    {
        filterLineGroup(
            state.filteredLeftLineTensionN[group],
            state.filteredLeftLineSlack[group],
            suspension.left.tensionN[group],
            suspension.left.slackFraction[group]);
        filterLineGroup(
            state.filteredRightLineTensionN[group],
            state.filteredRightLineSlack[group],
            suspension.right.tensionN[group],
            suspension.right.slackFraction[group]);
    }
    const double suspendedWeightN = Params.allUpMassKg * 9.80665;
    const double suspensionControlTransmission = std::clamp(
        (suspension.left.totalTensionN
            + suspension.right.totalTensionN)
            / std::max(1.0, suspendedWeightN),
        0.0, 1.0);
    const double rollNaturalFrequency =
        std::sqrt(9.80665 / SuspensionLengthM);
    const double attachedSpanTransmission = std::clamp(
        1.0 - 0.92 * std::max(
            canopy.leftStalledFraction,
            canopy.rightStalledFraction)
            - 0.82 * state.deepStall,
        0.02, 1.0);
    // Two channels, opposite senses, and they must stay apart.
    //
    //   payload: the carabiner carrying more load pulls that side of the wing
    //            DOWN. Weight shift right rolls right.
    //   aero:    the half generating more lift pushes that tip UP. A braked
    //            half loses lift, so brake right also rolls right - by the
    //            other route.
    //
    // Signs are stated against the bank convention used below, where positive
    // is right-tip-high. Summed into one number, as they were, whichever
    // control was not the one the coefficient had been fitted to came out
    // inverted, and no downstream sign could have rescued both.
    // Where the canopy hangs relative to the payload is statics: the roll
    // moment divided by the pendulum stiffness of the suspended load, W times
    // L. Nothing is fitted, and the answer is properly small - full weight
    // shift is under a degree of hang angle. The large sustained bank of a
    // real turn is not this; it is the coordinated turn below amplifying it.
    const double lateralPendulumStiffnessNmPerRad = std::max(
        200.0, suspendedWeightN * SuspensionLengthM);
    // Aerodynamic roll moment from the spanwise load split. The half carrying
    // more lift rises, so this is positive - the opposite sense to the
    // payload term, which is the whole point of keeping them apart.
    const double halfWingLiftArmM = 0.42 * 0.5
        * std::sqrt(Params.areaM2 * 5.2);
    const double aeroRollMomentNm = canopy.loadAsymmetry
        * Length(liftForce) * halfWingLiftArmM * attachedSpanTransmission;
    const double canopyRollTarget = std::clamp(
        (payload.rollMomentNm * suspensionControlTransmission
             + aeroRollMomentNm)
            / lateralPendulumStiffnessNmPerRad,
        -0.72, 0.72);
    const double relativeRollAcceleration =
        rollNaturalFrequency * rollNaturalFrequency
            * (canopyRollTarget - state.canopyRelativeRollRad)
        - 0.66 * 2.0 * rollNaturalFrequency
            * state.canopyRelativeRollRateRadps;
    state.canopyRelativeRollRateRadps +=
        relativeRollAcceleration * dt;
    state.canopyRelativeRollRad = std::clamp(
        state.canopyRelativeRollRad
            + state.canopyRelativeRollRateRadps * dt,
        -0.78, 0.78);
    // One mass-property calculator, in PayloadRigidBody. The payload's
    // inertia was separately approximated here as a fraction of its mass; two
    // descriptions of the same body is the duplication guiding rule 1 forbids.
    const double payloadRollInertia =
        std::max(1.0, PayloadRollInertiaForMassKgM2(payloadMassKg));
    const double payloadPitchInertia =
        std::max(1.0, PayloadPitchInertiaForMassKgM2(payloadMassKg));
    // No moment is injected here. The roll moments act on the system in the
    // moment sum below; adding them again to the hang angle counted them
    // twice and pinned the relative roll against its limit, which is what
    // made the wing tip over a few seconds into a weight-shift turn.
    state.canopyRelativePitchRateRadps +=
        (suspension.pitchMomentNm / payloadPitchInertia) * dt;
    const Vec3 harnessDragForce = dragDirectionWorld
        * (harnessDragMagnitude + suspension.lineDragN);
    // A ram-air canopy is not a rigid wing. Above its maneuver envelope,
    // billow, arc change and spanwise deformation progressively reduce the
    // effective lift increase. EN 926-1's 8 g sustained-load threshold is
    // retained as a final structural boundary, not used as normal handling.
    const double weightN = Params.allUpMassKg * 9.80665;
    const double rawLiftMagnitude = Length(liftForce);
    const double rawLiftG = rawLiftMagnitude / std::max(1.0, weightN);
    const double softeningOnset = std::clamp(
        Params.loadSofteningOnsetG, 2.5, 5.0);
    const double operationalLimit = std::clamp(
        Params.operationalLiftLimitG, softeningOnset + 0.2, 6.0);
    double highLoadDeformation = 0.0;
    if (rawLiftG > softeningOnset)
    {
        const double range = operationalLimit - softeningOnset;
        const double softenedLiftG = softeningOnset + range
            * (1.0 - std::exp(-(rawLiftG - softeningOnset) / range));
        highLoadDeformation = std::clamp(
            (rawLiftG - softeningOnset) / 2.0, 0.0, 1.0);
        liftForce = liftForce
            * (softenedLiftG / std::max(0.01, rawLiftG));
    }
    const double structuralLimitN = weightN * 8.0;
    const double softenedLiftMagnitude = Length(liftForce);
    if (softenedLiftMagnitude > structuralLimitN)
        liftForce = liftForce
            * (structuralLimitN / softenedLiftMagnitude);
    const Vec3 aerodynamicForce = liftForce + dragForce + harnessDragForce;
    const Vec3 gravityForce{0.0, 0.0, -9.80665 * Params.allUpMassKg};
    const Vec3 totalForceBody = state.attitude.InverseRotate(
        aerodynamicForce + gravityForce);
    const Vec3 effectiveMass{
        Params.allUpMassKg + Params.apparentMassKg.x,
        Params.allUpMassKg + Params.apparentMassKg.y,
        Params.allUpMassKg + Params.apparentMassKg.z
    };
    const Vec3 acceleration = state.attitude.Rotate({
        totalForceBody.x / effectiveMass.x,
        totalForceBody.y / effectiveMass.y,
        totalForceBody.z / effectiveMass.z
    });
    state.previousLongitudinalAccelerationMps2 =
        state.attitude.InverseRotate(acceleration).x;
    // How far a bob hanging under this aircraft is displaced from vertical.
    // A pendulum hangs along apparent gravity, and what tilts apparent gravity
    // away from true vertical is the HORIZONTAL acceleration - so that is what
    // this measures, in the world frame, along the direction of travel.
    //
    // Measuring it in body axes instead does not work, and both of the obvious
    // ways were tried. Body-x of the total acceleration puts a large part of
    // gravity itself into the answer whenever the wing is pitched, which under
    // brake is most of it. Apparent gravity expressed in body axes is worse
    // again: it carries the body's whole pitch attitude, which the actor's own
    // rotation already shows, so the canopy swings once for the aircraft
    // pitching and once more for the same thing measured again. Horizontal and
    // world-framed, gravity cannot leak in and neither can attitude.
    //
    // Referenced to where the aircraft POINTS, not to where it is going, and
    // that distinction is a defect fix rather than a preference. The direction
    // of travel is undefined at zero ground speed, so the version that used it
    // had to guard the division - and the guard zeroed the hang tilt whenever
    // the wing had less than 0.5 m/s of ground speed. A stall descends at 0.39.
    // The pendulum therefore switched off for the whole early recovery, which
    // is the one regime it exists for: measured, the descent went on rising
    // from 5.4 to 11.8 m/s AFTER the stall had cleared, and the glide took 34
    // seconds to come back. A pilot flying it described the wing as stabilising
    // "as if it was on the moon's gravity", and as working normally the moment
    // any brake or weight shift was touched - which is the same observation,
    // because an input restores ground speed and un-gates this.
    //
    // The heading is defined at zero airspeed, so there is nothing to guard and
    // no regime where the pendulum silently leaves. `physics_tests` gates the
    // recovery it was missing.
    {
        Vec3 alongHeading = state.attitude.Rotate({1.0, 0.0, 0.0});
        alongHeading.z = 0.0;
        double headingLength = Length(alongHeading);
        if (headingLength <= 1.0e-3)
        {
            // Pointing straight up or down, where a horizontal heading has no
            // meaning. Fall back to the track, and to zero only if that is
            // degenerate too.
            alongHeading = state.velocityWorldMps;
            alongHeading.z = 0.0;
            headingLength = Length(alongHeading);
        }
        const double alongTrackAcceleration = headingLength > 1.0e-3
            ? Dot(acceleration, alongHeading / headingLength) : 0.0;
        state.previousHangTiltRad =
            std::atan2(-alongTrackAcceleration, 9.80665);
    }
    // Load factor is the specific force - what an accelerometer on the
    // harness reads, the total acceleration less gravity - in g. The payload
    // body was reading this and it was never written, so the carabiners sat
    // at 1 g through a spiral and weight shift never lost any reach.
    state.previousLoadFactor =
        Length(acceleration + Vec3{0.0, 0.0, 9.80665}) / 9.80665;
    state.velocityWorldMps += acceleration * dt;
    state.positionWorldM += state.velocityWorldMps * dt;
    const Vec3 velocityBody =
        state.attitude.InverseRotate(state.velocityWorldMps);
    const double mechanicalEnergyJ =
        0.5 * (effectiveMass.x * velocityBody.x * velocityBody.x
            + effectiveMass.y * velocityBody.y * velocityBody.y
            + effectiveMass.z * velocityBody.z * velocityBody.z)
        + Params.allUpMassKg * 9.80665 * state.positionWorldM.z;
    const double aerodynamicPowerW =
        Dot(aerodynamicForce, state.velocityWorldMps);
    const double energyResidualW =
        state.previousMechanicalEnergyJ != 0.0
        ? (mechanicalEnergyJ - state.previousMechanicalEnergyJ) / dt
            - aerodynamicPowerW
        : 0.0;
    state.previousMechanicalEnergyJ = mechanicalEnergyJ;
    const double potentialEnergySpentJ = Params.allUpMassKg * 9.80665
        * std::max(0.0, state.velocityWorldMps.z) * dt;
    const double brakeDissipationJ = state.brakeZoomEnergyJ
        * (0.22 + 0.85 * symmetricBrake) * dt;
    state.brakeZoomEnergyJ = std::max(
        0.0, state.brakeZoomEnergyJ
            - potentialEnergySpentJ - brakeDissipationJ);
    state.brakeZoomEnergy = std::clamp(
        state.brakeZoomEnergyJ
            / std::max(900.0, availableKineticEnergyJ * 0.72),
        0.0, 1.0);

    const double bankAngle = std::asin(std::clamp(
        state.attitude.Rotate({0.0, 1.0, 0.0}).z, -1.0, 1.0));
    const double speedAuthority = std::clamp((speed - 5.5) / 5.0, 0.0, 1.0);
    const double stalledSpan = std::max(
        state.leftSeparatedSpan, state.rightSeparatedSpan);
    const double brakeRollAuthority = std::clamp(
        1.0 - 0.94 * stalledSpan - 0.88 * state.deepStall,
        0.03, 1.0);
    const double activeTurnCommand = std::clamp(
        std::abs(asymmetricBrake) + 0.85 * std::abs(controls.weightShift),
        0.0, 1.0);
    const double passiveRollExcitation = std::clamp(
        atmosphere.turbulence
            + atmosphere.spanwiseAirflowShearMps / 3.0
            + 0.8 * std::max(state.leftCollapse, state.rightCollapse),
        0.0, 1.0);
    const double relativeRollTransmission = std::max(
        std::clamp(activeTurnCommand / 0.20, 0.0, 1.0),
        passiveRollExcitation);
    // Heading and bank use the same signed turn convention in the simulator:
    // negative is left, positive is right.
    const double coordinatedBankTarget = std::clamp(
        // The same relation read the other way: a nose-right yaw rate
        // (positive) belongs with a right-tip-down bank (negative).
        -Params.yawToBankGain * state.angularVelocityBodyRadps.z
                * activeTurnCommand
                * brakeRollAuthority
            + 0.78 * state.canopyRelativeRollRad
                * relativeRollTransmission
                * std::sqrt(brakeRollAuthority),
        -Params.maximumSustainedBankRad,
        Params.maximumSustainedBankRad);
    const double bankError = coordinatedBankTarget - bankAngle;
    const double spiralTarget = std::clamp(
        (std::abs(bankAngle) - 0.58) / 0.30, 0.0, 1.0)
        * speedAuthority * (1.0 - 0.72 * state.deepStall);
    state.spiralDevelopment += (spiralTarget - state.spiralDevelopment)
        * std::min(1.0, dt * (spiralTarget > state.spiralDevelopment
            ? 0.75 : 1.8));
    const double stalledBankLimit =
        0.38 + 0.54 * brakeRollAuthority;
    const double bankBarrierMoment =
        std::abs(bankAngle) > stalledBankLimit
        ? -std::copysign(
            1800.0 * (std::abs(bankAngle) - stalledBankLimit),
            bankAngle)
        : 0.0;

    // A banked wing turns toward its low tip. Measured, not assumed: a
    // positive rotation about body +X puts the RIGHT tip UP, and a positive
    // body +Z rate takes the nose RIGHT. So right-tip-up (bankAngle positive)
    // is a left turn and wants a negative yaw rate. See the note in
    // ParagliderCoordinateSystem.h, which described both the other way round.
    const double coordinatedYawTarget = std::clamp(
        -9.80665 * std::tan(bankAngle)
            / std::max(7.0, speed),
        -1.25, 1.25);
    Vec3 moments{
        Params.coordinatedRollStiffness * bankError * speedAuthority
            - Params.coordinatedRollDamping
                * state.angularVelocityBodyRadps.x
            + 310.0 * (state.rightCollapse - state.leftCollapse)
            + 390.0 * (state.rightCravat - state.leftCravat)
            - 95.0 * state.spin * brakeRollAuthority
            + canopy.rollMomentNm * brakeRollAuthority
            + payload.rollMomentNm
            + bankBarrierMoment
            - Params.rollDamping * state.angularVelocityBodyRadps.x,
        -Params.pitchDamping * state.angularVelocityBodyRadps.y
            - Params.pitchStiffness * alphaFromTrim
            - Params.acceleratorPitchMoment * state.acceleratorTravel
            + suspension.pitchMomentNm
            + payload.pitchMomentNm
            - 22.0 * state.acceleratorRate
            - 210.0 * state.recoverySurge
            + 85.0 * state.frontalCollapse,
        Params.brakeYawMoment * asymmetricBrake
            + 190.0 * (state.rightCollapse - state.leftCollapse)
            + 420.0 * (state.rightCravat - state.leftCravat)
            + 360.0 * state.spin
            + canopy.yawMomentNm
            + suspension.yawMomentNm
            + Params.coordinatedYawStiffness
                * (coordinatedYawTarget
                    - state.angularVelocityBodyRadps.z)
            - Params.yawDamping * state.angularVelocityBodyRadps.z
    };
    const Vec3 angularAcceleration{
        moments.x / (Params.rollInertiaKgM2
            + Params.apparentRotationalInertiaKgM2.x),
        moments.y / (Params.pitchInertiaKgM2
            + Params.apparentRotationalInertiaKgM2.y),
        moments.z / (Params.yawInertiaKgM2
            + Params.apparentRotationalInertiaKgM2.z)
    };
    state.angularVelocityBodyRadps += angularAcceleration * dt;
    // An EN-B full-size wing can enter a steep spiral or spin, but ordinary
    // sustained brake cannot inject unlimited angular energy and roll it like
    // an aerobatic mini-wing.
    const double rollRateLimit =
        (0.06 + 1.00 * brakeRollAuthority)
        + 0.30 * state.spiralDevelopment * brakeRollAuthority;
    const double pitchRateLimit = 0.90 + 0.25 * state.recoverySurge;
    const double yawRateLimit = 1.35 + 0.55 * std::abs(state.spin);
    state.angularVelocityBodyRadps.x = std::clamp(
        state.angularVelocityBodyRadps.x, -rollRateLimit, rollRateLimit);
    state.angularVelocityBodyRadps.y = std::clamp(
        state.angularVelocityBodyRadps.y, -pitchRateLimit, pitchRateLimit);
    state.angularVelocityBodyRadps.z = std::clamp(
        state.angularVelocityBodyRadps.z, -yawRateLimit, yawRateLimit);

    const Quaternion omega{
        0.0,
        state.angularVelocityBodyRadps.x,
        state.angularVelocityBodyRadps.y,
        state.angularVelocityBodyRadps.z
    };
    const Quaternion derivative = state.attitude * omega;
    state.attitude = Quaternion{
        state.attitude.w + 0.5 * derivative.w * dt,
        state.attitude.x + 0.5 * derivative.x * dt,
        state.attitude.y + 0.5 * derivative.y * dt,
        state.attitude.z + 0.5 * derivative.z * dt
    }.Normalized();

    const double pressureTarget = std::clamp(
        speed / 9.5 - 0.35 * meanCollapse - 0.52 * state.frontalCollapse
            - 0.08 * state.acceleratorTravel
            - 0.12 * symmetricBrake, 0.22, 1.08);
    const double statePressureLoss = 0.32 * state.deepStall
        + 0.22 * (state.leftCravat + state.rightCravat);
    state.canopyPressure += (pressureTarget - state.canopyPressure) * std::min(1.0, dt * 3.2);
    state.canopyPressure = std::clamp(
        state.canopyPressure - statePressureLoss * dt, 0.12, 1.08);

    TelemetryState.airspeedMps = speed;
    // The canopy's incidence, which is what stalls. The body's own is
    // reported beside it so the swing's contribution is visible rather than
    // buried in one number.
    TelemetryState.angleOfAttackRad = alpha;
    TelemetryState.bodyAngleOfAttackRad = bodyAlpha;
    TelemetryState.liftCoefficient = cl;
    TelemetryState.dragCoefficient = cd;
    TelemetryState.loadFactor = Length(aerodynamicForce)
                              / (Params.allUpMassKg * 9.80665);
    TelemetryState.highLoadDeformation = highLoadDeformation;
    TelemetryState.turbulence = atmosphere.turbulence;
    TelemetryState.rotorStrength = atmosphere.rotorStrength;
    TelemetryState.lowFrequencyGustMps =
        atmosphere.lowFrequencyGustMps;
    TelemetryState.highFrequencyGustMps =
        atmosphere.highFrequencyGustMps;
    TelemetryState.gustEnergyMps = atmosphere.gustEnergyMps;
    TelemetryState.leftCollapse = state.leftCollapse;
    TelemetryState.rightCollapse = state.rightCollapse;
    TelemetryState.frontalCollapse = state.frontalCollapse;
    TelemetryState.canopyPressure = state.canopyPressure;
    TelemetryState.harnessRollRad = state.harnessRollRad;
    TelemetryState.harnessPitchRad = state.harnessPitchRad;
    TelemetryState.flareBoost = flareBoost;
    TelemetryState.groundEffect = groundEffect;
    TelemetryState.flareEnergy = state.flareEnergy;
    TelemetryState.flareAuthority = landingAero.flareAuthority;
    const double brakeDynamicLoad = std::clamp(
        dynamicPressure / 66.0, 0.10, 1.85);
    const double loadingForceScale = std::clamp(
        std::sqrt(wingLoading / 3.85), 0.78, 1.28);
    const double pressureForceScale = std::clamp(
        0.28 + 0.72 * state.canopyPressure, 0.22, 1.08);
    const double leftUnloading = std::clamp(
        1.0 - 0.78 * state.leftCollapse - 0.86 * state.leftCravat
            - 0.38 * state.frontalCollapse, 0.08, 1.0);
    const double rightUnloading = std::clamp(
        1.0 - 0.78 * state.rightCollapse - 0.86 * state.rightCravat
            - 0.38 * state.frontalCollapse, 0.08, 1.0);
    TelemetryState.leftBrakeForceN = Params.brakeForceAtFullTravelN
        * brakeDynamicLoad * loadingForceScale * pressureForceScale
        * std::pow(controls.leftBrake, Params.brakePressureExponent)
        * leftUnloading;
    TelemetryState.rightBrakeForceN = Params.brakeForceAtFullTravelN
        * brakeDynamicLoad * loadingForceScale * pressureForceScale
        * std::pow(controls.rightBrake, Params.brakePressureExponent)
        * rightUnloading;
    TelemetryState.leftBrakePressure = std::clamp(
        TelemetryState.leftBrakeForceN
            / std::max(1.0, Params.brakeForceAtFullTravelN),
        0.0, 1.8);
    TelemetryState.rightBrakePressure = std::clamp(
        TelemetryState.rightBrakeForceN
            / std::max(1.0, Params.brakeForceAtFullTravelN),
        0.0, 1.8);
    TelemetryState.thermalLiftMps = atmosphere.thermalLiftMps;
    TelemetryState.thermalCoreStrength = atmosphere.thermalCoreStrength;
    TelemetryState.thermalLifecycle = atmosphere.thermalLifecycle;
    TelemetryState.cloudBaseClearanceM = atmosphere.cloudBaseClearanceM;
    TelemetryState.recoverySurge = state.recoverySurge;
    TelemetryState.deepStall = state.deepStall;
    TelemetryState.spin = state.spin;
    TelemetryState.leftCravat = state.leftCravat;
    TelemetryState.rightCravat = state.rightCravat;
    TelemetryState.spanwiseLoadAsymmetry = canopy.loadAsymmetry;
    TelemetryState.accelerator = state.acceleratorTravel;
    const double lineLoad = Length(aerodynamicForce);
    TelemetryState.aRiserLoad = suspension.aFraction;
    TelemetryState.bRiserLoad = suspension.bFraction;
    TelemetryState.cRiserLoad = suspension.cFraction;
    // Kept for replay/CSV compatibility. EPIC 2's upper D row cascades into C.
    TelemetryState.dRiserLoad = 0.0;
    TelemetryState.brakeLineLoad = suspension.brakeFraction;
    TelemetryState.lineLoadTotalN = lineLoad;
    TelemetryState.lineDragN = suspension.lineDragN;
    TelemetryState.suspensionPitchMomentNm = suspension.pitchMomentNm;
    TelemetryState.allUpMassKg = Params.allUpMassKg;
    TelemetryState.wingLoadingKgM2 = wingLoading;
    TelemetryState.harnessDragN = harnessDragMagnitude;
    TelemetryState.brakeTravelLeftMm =
        commanded.leftBrake * Params.brakeTravelMm;
    TelemetryState.brakeTravelRightMm =
        commanded.rightBrake * Params.brakeTravelMm;
    TelemetryState.leftReinflationAuthority = leftRecovery;
    TelemetryState.rightReinflationAuthority = rightRecovery;
    TelemetryState.leftReinflationRatePerS = std::max(
        0.0, (previousLeftCollapse - state.leftCollapse) / dt);
    TelemetryState.rightReinflationRatePerS = std::max(
        0.0, (previousRightCollapse - state.rightCollapse) / dt);
    TelemetryState.frontalReinflationRatePerS = std::max(
        0.0, (previousFrontalCollapse - state.frontalCollapse) / dt);
    TelemetryState.surgeContainment = surgeContainment;
    TelemetryState.aerodynamicUnloading = aerodynamicUnloading;
    TelemetryState.dynamicPressureDropPaPerS = pressureDropRate;
    TelemetryState.spanwiseAirflowShearMps =
        atmosphere.spanwiseAirflowShearMps;
    TelemetryState.leftAirflowDisturbance = leftSpatialDisturbance;
    TelemetryState.rightAirflowDisturbance = rightSpatialDisturbance;
    TelemetryState.collapseResistance = Params.collapseResistance;
    TelemetryState.passiveReinflationRate = Params.passiveReinflationRate;
    TelemetryState.frontalReinflationRate = Params.frontalReinflationRate;
    TelemetryState.cravatSusceptibility = Params.cravatSusceptibility;
    TelemetryState.stalled = stalled;
    TelemetryState.bankAngleRad = bankAngle;
    TelemetryState.coordinatedBankTargetRad = coordinatedBankTarget;
    TelemetryState.spiralDevelopment = state.spiralDevelopment;
    TelemetryState.leftLineTensionN =
        state.filteredLeftLineTensionN;
    TelemetryState.rightLineTensionN =
        state.filteredRightLineTensionN;
    TelemetryState.leftLineSlack = state.filteredLeftLineSlack;
    TelemetryState.rightLineSlack = state.filteredRightLineSlack;
    TelemetryState.lateralLineLoadImbalance =
        suspension.lateralLoadImbalance;
    TelemetryState.leftStalledSpan = canopy.leftStalledFraction;
    TelemetryState.rightStalledSpan = canopy.rightStalledFraction;
    TelemetryState.brakeRollAuthority = brakeRollAuthority;
    TelemetryState.effectiveLeftBrake = controls.leftBrake;
    TelemetryState.effectiveRightBrake = controls.rightBrake;
    TelemetryState.brakeZoomEnergy = state.brakeZoomEnergy;
    TelemetryState.brakeZoomLiftCoefficient =
        brakeZoomLiftCoefficient;
    TelemetryState.coordinatedYawTargetRadps =
        coordinatedYawTarget;
    TelemetryState.brakeZoomEnergyJ = state.brakeZoomEnergyJ;
    TelemetryState.brakeZoomAvailableKineticEnergyJ =
        availableKineticEnergyJ;
    TelemetryState.canopyRelativePitchRad =
        state.canopyRelativePitchRad;
    TelemetryState.canopyRelativePitchRateRadps =
        state.canopyRelativePitchRateRadps;
    TelemetryState.canopyRelativeRollRad =
        state.canopyRelativeRollRad;
    TelemetryState.leftCarabinerLoadN = payload.leftCarabinerN;
    TelemetryState.rightCarabinerLoadN = payload.rightCarabinerN;
    TelemetryState.carabinerLoadAsymmetry = payload.loadAsymmetry;
    TelemetryState.pilotCgOffsetM = payload.effectiveCgOffsetM;
    TelemetryState.payloadRollRad = state.payload.rollRad;
    TelemetryState.payloadPitchRad = state.payload.pitchRad;
    TelemetryState.payloadCgOffsetLongitudinalM =
        payload.cgOffsetLongitudinalM;
    TelemetryState.suspensionPendulumPeriodS =
        2.0 * 3.14159265358979323846
            * std::sqrt(SuspensionLengthM / 9.80665);
    TelemetryState.canopyRelativeRollRateRadps =
        state.canopyRelativeRollRateRadps;
    TelemetryState.suspensionControlTransmission =
        suspensionControlTransmission;
    TelemetryState.leftSeparatedSpanState =
        state.leftSeparatedSpan;
    TelemetryState.rightSeparatedSpanState =
        state.rightSeparatedSpan;
    TelemetryState.canopyMassKg = Params.canopyMassKg;
    TelemetryState.payloadMassKg = std::max(
        0.0, Params.allUpMassKg - Params.canopyMassKg);
    TelemetryState.apparentMassKg = Params.apparentMassKg;
    TelemetryState.effectiveTranslationalMassKg = effectiveMass;
    TelemetryState.suspensionMomentNm = {
        suspension.rollMomentNm,
        suspension.pitchMomentNm,
        suspension.yawMomentNm
    };
    TelemetryState.payloadReactionMomentNm =
        -TelemetryState.suspensionMomentNm;
    TelemetryState.aerodynamicForceWorldN = aerodynamicForce;
    TelemetryState.gravityForceWorldN = gravityForce;
    TelemetryState.accelerationWorldMps2 = acceleration;
    TelemetryState.mechanicalEnergyJ = mechanicalEnergyJ;
    TelemetryState.aerodynamicPowerW = aerodynamicPowerW;
    TelemetryState.energyResidualW = energyResidualW;
    const double harnessShiftCm =
        22.0 * std::sin(state.harnessRollRad);
    const double harnessDropCm =
        10.0 * (1.0 - std::cos(state.harnessRollRad));
    TelemetryState.leftCarabinerLateralCm =
        -18.0 + harnessShiftCm;
    TelemetryState.rightCarabinerLateralCm =
        18.0 + harnessShiftCm;
    TelemetryState.carabinerVerticalCm =
        34.0 - harnessDropCm;
}
}
