#include "CoupledParagliderSolver.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double GravityMps2 = 9.80665;

Quaternion IntegrateAttitude(
    const Quaternion& q, const Vec3& bodyRate, double dt)
{
    const Vec3 worldRate = q.Rotate(bodyRate);
    const Quaternion omega{0.0, worldRate.x, worldRate.y, worldRate.z};
    const Quaternion derivative = omega * q;
    return Quaternion{
        q.w + 0.5 * dt * derivative.w,
        q.x + 0.5 * dt * derivative.x,
        q.y + 0.5 * dt * derivative.y,
        q.z + 0.5 * dt * derivative.z}.Normalized();
}
}

CoupledParagliderSolver::CoupledParagliderSolver(
    const CanopyGeometry& geometry, const LinePlanSpec& linePlan,
    const CoupledSchedule& schedule)
    : ScheduleValue(schedule),
      Aerodynamics(geometry, SectionPolarTable::Analytic(),
                   geometry.Spec().cellCount),
      Pressure(geometry.Spec().cellCount),
      Membrane(geometry.CellSpacingM()),
      Lines(BuildSuspensionGraph(geometry, linePlan)),
      ApparentMass(CanopyApparentMass(geometry))
{
    CanopyMassKg = 5.1;
    SystemMassKg = PayloadMass.TotalKg() + CanopyMassKg;
    ReferenceAreaM2 = Aerodynamics.ReferenceAreaM2();
    ReferenceSpanM = geometry.DevelopedSpanM();
    PendulumLengthM = SuspensionPendulumLengthM(Lines);
    Harness = Lines.plan.harness;
}

void CoupledParagliderSolver::Step(
    CoupledState& state, const CoupledControls& controls,
    const CoupledAtmosphere& atmosphere)
{
    const double dt = ScheduleValue.timeStepS;
    CoupledDiagnostics diagnostics;

    // A simulation that starts mid-flight starts with an inflated canopy.
    // Leaving the cells packed while the wing is already doing 10 m/s made
    // the pressure model report Cp near zero, which the section polars
    // correctly turned into 15% of the lift and a large drag penalty - so the
    // wing stalled on its first aerodynamic re-solve and never recovered. The
    // solvers were all right; the initial condition was not a wing.
    if (!state.initialised) Pressure.SeedInflated(state.pressure);

    // -- 1. controls -------------------------------------------------------
    // Nothing to integrate: brake and bar are rest-length changes, and they
    // reach the wing through the line network in step 6 and through the
    // section camber in step 3. Neither commands a moment (guiding rules 4
    // and 5), which is why there is no code here.

    // -- 2. atmosphere -----------------------------------------------------
    const Vec3 airVelocityWorld = state.velocityWorldMps - atmosphere.windWorldMps;
    const Vec3 airspeedBody = state.attitude.InverseRotate(airVelocityWorld);
    const double airspeed = Length(airspeedBody);
    const double dynamicPressure =
        0.5 * atmosphere.densityKgM3 * airspeed * airspeed;
    diagnostics.airspeedMps = airspeed;
    diagnostics.angleOfAttackRad = airspeed > 1.0e-6
        ? std::atan2(-airspeedBody.z, airspeedBody.x) : 0.0;

    // The staggered loop. Aero, then the structure it loads, then back again
    // with the load relaxed - which is what keeps a light structure in dense
    // flow from oscillating itself apart.
    Vec3 exchangedForceBody = state.heldAeroForceBodyN;
    Vec3 exchangedMomentBody = state.heldAeroMomentBodyNm;
    const bool solveAerodynamics =
        !state.initialised
        || state.stepsSinceAerodynamics >= ScheduleValue.aerodynamicsInterval;

    std::vector<double>& cellPressureCoefficient = state.heldPressureCoefficient;
    SuspensionSolution lineSolution;
    double previousExchangedMagnitude = Length(exchangedForceBody);

    Vec3 aeroTargetForce = exchangedForceBody;
    Vec3 aeroTargetMoment = exchangedMomentBody;

    // -- 3, 4, 5. aerodynamics, pressure, membrane -------------------------
    // These run once per aerodynamic interval, not once per coupling
    // iteration. What the staggered loop below iterates is the exchange
    // between that load and the structure it acts on, which is where the
    // added-mass instability lives; re-solving the whole aerodynamic side
    // three times a step would cost three times as much to relax the same
    // number.
    {
        if (solveAerodynamics)
        {
            VsmSolveInput aero;
            aero.airspeedBodyMps = airspeedBody;
            aero.angularVelocityBodyRadps = state.angularVelocityBodyRadps;
            aero.airDensityKgM3 = atmosphere.densityKgM3;
            aero.leftBrake = controls.leftBrake;
            aero.rightBrake = controls.rightBrake;
            aero.internalPressureCoefficient = cellPressureCoefficient;

            VsmSettings settings;
            // Warm-started by the separation state, so a handful of
            // iterations is enough once the first solve has landed.
            settings.maxIterations = state.initialised ? 40 : 600;
            const VsmSolution solved = Aerodynamics.SolveUnsteady(
                aero, state.separation, dt, settings);

            // Reject a solve that has not converged to anything usable. Deep
            // stall genuinely has no steady state - the separated branch has a
            // negative lift slope, which inverts the downwash feedback between
            // sections - so the VSM will not settle there and must not be
            // allowed to hand a diverged load to the structure. Holding the
            // previous load is wrong, but it is bounded and it is reported.
            const auto allFinite = [](const Vec3& v)
            {
                return std::isfinite(v.x) && std::isfinite(v.y)
                    && std::isfinite(v.z);
            };
            // The moment needs its own bound, and did not have one. A single
            // step near stall returned a converged solve carrying 34 kNm of
            // yaw - against a q S b of 14 kNm, so a moment coefficient of
            // nearly 2.5 - which was accepted, and every step after it was
            // rejected and held that number. Ten seconds of a held 26 kNm on
            // an inertia of 150 is a turn rate of 100 rad/s, which is where
            // the asymmetric-brake NaN came from.
            const double momentScaleNm = std::max(
                1.0, dynamicPressure * ReferenceAreaM2 * ReferenceSpanM);
            const bool usable =
                allFinite(solved.forceBodyN)
                && allFinite(solved.momentBodyNm)
                && Length(solved.forceBodyN) < 50.0 * SystemMassKg * GravityMps2
                && Length(solved.momentBodyNm) < 2.0 * momentScaleNm;
            if (!usable)
            {
                diagnostics.aerodynamicsRejected = true;
                diagnostics.vsmResidual = solved.residual;
                diagnostics.vsmConverged = false;
                diagnostics.meanPressureCoefficient =
                    state.heldPressureCoefficientMean;
                goto structureSolve;
            }

            aeroTargetForce = solved.forceBodyN;

            // Measure how much of the moment is the wing's rotation being
            // damped, by solving again with the rotation removed. The
            // difference over the rate is a damping derivative that can be
            // applied at the live rate every step while the rest is held.
            // The probes ask about the wing the unsteady solve just found, so
            // they hold its separation state and continue its circulation
            // rather than starting cold at the equilibrium separation for
            // whatever incidence they land on. Cold, capped at 40 iterations
            // where a cold solve needs about ninety, they were not converged
            // and not even the same wing - which is why the coefficient they
            // returned moved 10% between one interval and the next, and why
            // two mirror-image flights measured different damping.
            VsmSolveInput still = aero;
            still.angularVelocityBodyRadps = Vec3{};
            VsmSettings stillSettings = settings;
            stillSettings.maxIterations = 600;
            const VsmSolution stationary = Aerodynamics.SolveFrozen(
                still, state.separation, state.stationaryCirculation,
                stillSettings);

            aeroTargetMoment = stationary.momentBodyNm;
            const bool probeUsable = allFinite(stationary.momentBodyNm)
                && Length(stationary.momentBodyNm) < 2.0 * momentScaleNm;
            if (!probeUsable)
            {
                // The rotation-free probe is a second solve and can fail on
                // its own. Falling back to the previous derivative keeps the
                // damping bounded rather than letting a NaN through it.
                diagnostics.aerodynamicsRejected = true;
                aeroTargetMoment = exchangedMomentBody;
            }

            // The damping derivative, measured against a rate the solver
            // chooses rather than the one the wing happens to have.
            //
            // Dividing the live moment difference by the live rate is an
            // ill-posed estimator: near zero rate it is noise over nothing, and
            // the guard that suppressed it below 1e-3 rad/s made the whole
            // damping law discontinuous in the state. Two mirror-image flights
            // took different branches of that guard within four seconds and
            // stopped being mirror images, and the coefficient itself swung
            // between -2.3e4 and +2.5e3 Nm per rad/s between one aerodynamic
            // interval and the next.
            //
            // A fixed perturbation removes both problems. One axis is probed
            // per aerodynamic interval - each axis refreshed every 0.3 s, far
            // slower than the coefficient changes with airspeed - at a rate
            // typical of a real turn, so the difference is always well above
            // the solve's own noise and never divided by a small number.
            //
            // The probe is centred, +p and -p rather than +p against zero.
            // A one-sided probe is not odd in the rate, so the same wing
            // braked left and braked right measures two different damping
            // coefficients, and two flights that should be mirror images of
            // each other stop being so in the fourth second.
            if (probeUsable)
            {
                constexpr double ProbeRateRadps = 0.3;
                const int axis = state.dampingProbeAxis % 3;
                const auto component = [](const Vec3& v, int which)
                {
                    return which == 0 ? v.x : which == 1 ? v.y : v.z;
                };
                const auto setComponent = [](Vec3& v, int which, double value)
                {
                    if (which == 0) v.x = value;
                    else if (which == 1) v.y = value;
                    else v.z = value;
                };
                const auto probeAt = [&](double rate,
                                         std::vector<double>& warm)
                {
                    VsmSolveInput probe = still;
                    Vec3 probeRate{};
                    setComponent(probeRate, axis, rate);
                    probe.angularVelocityBodyRadps = probeRate;
                    return Aerodynamics.SolveFrozen(
                        probe, state.separation, warm, stillSettings);
                };
                const VsmSolution forward =
                    probeAt(ProbeRateRadps, state.forwardProbeCirculation);
                const VsmSolution backward =
                    probeAt(-ProbeRateRadps, state.backwardProbeCirculation);
                if (allFinite(forward.momentBodyNm)
                    && allFinite(backward.momentBodyNm)
                    && Length(forward.momentBodyNm) < 2.0 * momentScaleNm
                    && Length(backward.momentBodyNm) < 2.0 * momentScaleNm)
                {
                    const double delta = component(forward.momentBodyNm, axis)
                        - component(backward.momentBodyNm, axis);
                    setComponent(state.rotationalDampingNmPerRadps, axis,
                                 delta / (2.0 * ProbeRateRadps));
                }
                state.dampingProbeAxis = (axis + 1) % 3;
            }

            diagnostics.vsmResidual = solved.residual;
            diagnostics.vsmConverged = solved.converged;
            diagnostics.aerodynamicsSolvedThisStep = true;

            // -- 4. pressure ----------------------------------------------
            CellPressureInput cells;
            cells.dynamicPressurePa.resize(solved.sections.size());
            cells.angleOfAttackRad.resize(solved.sections.size());
            for (std::size_t i = 0; i < solved.sections.size(); ++i)
            {
                cells.dynamicPressurePa[i] = dynamicPressure;
                cells.angleOfAttackRad[i] = solved.sections[i].angleOfAttackRad;
            }
            const CellPressureResult pressureResult =
                Pressure.Step(state.pressure, cells, dt);
            cellPressureCoefficient = pressureResult.pressureCoefficient;
            state.heldPressureCoefficientMean =
                pressureResult.meanPressureCoefficient;
            diagnostics.meanPressureCoefficient =
                pressureResult.meanPressureCoefficient;

            // -- 5. membrane ----------------------------------------------
            // One representative station stands for the chord. The skin's own
            // dynamics are far faster than the flight's, so what matters here
            // is the shape it settles to under the pressure just solved.
            MembraneLoad skin;
            skin.internalPressurePa = pressureResult.gaugePressurePa.empty()
                ? 0.0 : pressureResult.gaugePressurePa[
                            pressureResult.gaugePressurePa.size() / 2];
            skin.brakeLineForceN = 40.0
                * (controls.leftBrake + controls.rightBrake);
            const MembraneResult skinResult = Membrane.Step(skin, dt);
            diagnostics.membraneStrain = skinResult.maximumStrain;
        }
    }

structureSolve:
    // The pressure coefficient is only recomputed when the aerodynamics run,
    // so it is carried rather than reported as zero on the steps between.
    if (!diagnostics.aerodynamicsSolvedThisStep)
        diagnostics.meanPressureCoefficient =
            state.heldPressureCoefficientMean;

    const int couplingIterations = std::max(1, ScheduleValue.couplingIterations);
    for (int coupling = 0; coupling < couplingIterations; ++coupling)
    {
        // Relax the load handed to the structure rather than applying it
        // whole. This is the damping the ram-air FSI literature calls for:
        // without it a light structure in dense flow drives itself unstable
        // through added mass.
        const double relax = std::clamp(
            ScheduleValue.couplingRelaxation, 0.05, 1.0);
        exchangedForceBody = exchangedForceBody
            + (aeroTargetForce - exchangedForceBody) * relax;
        exchangedMomentBody = exchangedMomentBody
            + (aeroTargetMoment - exchangedMomentBody) * relax;

        // -- 6. lines -------------------------------------------------------
        SuspensionSolveInput suspension;
        // The aerodynamic resultant reaches the lines in the payload frame,
        // which is where the network is solved.
        suspension.aeroForceN = state.attitude.Rotate(exchangedForceBody);
        suspension.canopyWeightN = CanopyMassKg * GravityMps2;
        suspension.accelerator = controls.accelerator;
        suspension.leftBrake = controls.leftBrake;
        suspension.rightBrake = controls.rightBrake;
        suspension.weightShift = controls.weightShift;

        SuspensionSolverSettings lineSettings;
        lineSettings.iterations = state.initialised
            ? std::max(1, ScheduleValue.suspensionIterations) : 12000;
        lineSolution = SolveSuspension(
            Lines, suspension, lineSettings, &state.suspension);
        diagnostics.suspensionResidualN = lineSolution.canopyForceResidualN;

        const double magnitude = Length(exchangedForceBody);
        diagnostics.couplingResidual = previousExchangedMagnitude > 1.0e-9
            ? std::fabs(magnitude - previousExchangedMagnitude)
                / previousExchangedMagnitude
            : 0.0;
        previousExchangedMagnitude = magnitude;
        diagnostics.couplingIterationsUsed = coupling + 1;
    }

    state.stepsSinceAerodynamics =
        solveAerodynamics ? 1 : state.stepsSinceAerodynamics + 1;
    state.heldAeroForceBodyN = exchangedForceBody;
    state.heldAeroMomentBodyNm = exchangedMomentBody;

    // -- 7. rigid motion ---------------------------------------------------
    // The payload, on its own pendulum under the carabiners.
    PayloadInput payloadInput;
    payloadInput.weightShift = controls.weightShift;
    payloadInput.suspendedLoadN =
        std::max(1.0, PayloadMass.TotalKg() * GravityMps2);
    payloadInput.loadFactor = 1.0;
    const PayloadLoads payloadLoads = StepPayload(
        state.payload, PayloadMass, Harness, payloadInput, dt);
    diagnostics.leftCarabinerLoadN = payloadLoads.leftCarabinerN;
    diagnostics.rightCarabinerLoadN = payloadLoads.rightCarabinerN;

    // Everything hanging below the wing. Lines, risers, harness and pilot are
    // 47% of the canopy's own drag on this aircraft, so leaving them out of
    // the coupled solve makes it glide at 14 where the wing glides at 9.5.
    const InstalledDragResult installed = EvaluateInstalledDrag(
        InstalledDrag, airspeedBody, atmosphere.densityKgM3);
    const Vec3 installedDragBody = airspeed > 1.0e-6
        ? airspeedBody * (-installed.totalDragN / airspeed) : Vec3{};
    exchangedForceBody += installedDragBody;
    exchangedMomentBody += installed.momentBodyNm;

    // Forces on the system. The line network is internal - it appears once on
    // the canopy and once on the payload, and cancels - so what accelerates
    // the system is the aerodynamic force and gravity, and nothing else.
    const Vec3 aeroWorld = state.attitude.Rotate(exchangedForceBody);
    const Vec3 weightWorld{0.0, 0.0, -SystemMassKg * GravityMps2};
    const Vec3 netForceWorld = aeroWorld + weightWorld;

    diagnostics.aeroForceBodyN = exchangedForceBody;
    diagnostics.lineForceBodyN = state.attitude.InverseRotate(
        lineSolution.leftCarabinerForceN + lineSolution.rightCarabinerForceN);
    diagnostics.weightForceWorldN = weightWorld;
    diagnostics.netForceWorldN = netForceWorld;

    // Internal closure. Every line tension acts on two things, so summing all
    // of them over both ends must give zero. The suspension solver measures
    // this itself; carrying it here is what makes it a system property rather
    // than one solver's private check.
    diagnostics.internalForceClosureN = Length(lineSolution.endpointForceSumN);
    diagnostics.internalMomentClosureNm = lineSolution.canopyMomentResidualNm;

    // Apparent mass. The air the canopy accelerates with it is a third of the
    // aircraft, so it belongs in the denominator - leaving it out overstates
    // every acceleration the wing makes.
    const Vec3 effectiveMass{
        SystemMassKg + ApparentMass.massKg.x,
        SystemMassKg + ApparentMass.massKg.y,
        SystemMassKg + ApparentMass.massKg.z};
    const Vec3 netForceBody = state.attitude.InverseRotate(netForceWorld);
    const Vec3 accelerationBody{
        netForceBody.x / effectiveMass.x,
        netForceBody.y / effectiveMass.y,
        netForceBody.z / effectiveMass.z};
    const Vec3 accelerationWorld = state.attitude.Rotate(accelerationBody);

    const double kineticBefore = 0.5 * SystemMassKg
        * Dot(state.velocityWorldMps, state.velocityWorldMps);
    const double potentialBefore =
        SystemMassKg * GravityMps2 * state.positionWorldM.z;

    state.velocityWorldMps += accelerationWorld * dt;
    state.positionWorldM += state.velocityWorldMps * dt;

    // Moments: the aerodynamic moment the VSM integrated, plus the payload's
    // weight acting off-centre through the carabiners. Neither is a control
    // term - there is no brake-to-yaw or shift-to-roll coefficient anywhere
    // in this file.
    // The held moment, plus the rotational damping evaluated at the rate the
    // wing actually has right now, plus the payload's weight acting through
    // the carabiners. No control term appears here.
    const Vec3 rate = state.angularVelocityBodyRadps;
    // The damping coefficients, as positive resistances. Only a moment that
    // opposes the rotation is damping; a measured derivative of the other sign
    // is the aerodynamic solve being asked a question it cannot answer, and it
    // is dropped rather than fed back as excitation.
    const Vec3 dampingNmPerRadps{
        -std::min(0.0, state.rotationalDampingNmPerRadps.x),
        -std::min(0.0, state.rotationalDampingNmPerRadps.y),
        -std::min(0.0, state.rotationalDampingNmPerRadps.z)};
    // The pendulum. The payload's weight does not act at the canopy - it acts
    // at a centre of mass eight metres below it, on the end of the lines. Any
    // rotation of the canopy away from having that mass directly beneath it
    // is resisted by W L sin(theta), which for this wing is about 7000 Nm per
    // radian and is far and away the dominant pitch and roll stiffness.
    //
    // Leaving it out is not a small omission. Without it the only pitch
    // moment is the wing's own, nothing restores attitude, and the canopy
    // pitches up until it is descending vertically at 7.5 m/s with no forward
    // speed at all - which is what it did.
    const Vec3 payloadOffsetBody{0.0, 0.0, -PendulumLengthM};
    const Vec3 payloadWeightWorld{
        0.0, 0.0, -PayloadMass.TotalKg() * GravityMps2};
    const Vec3 pendulumMoment = Cross(
        payloadOffsetBody, state.attitude.InverseRotate(payloadWeightWorld));

    const Vec3 undampedMomentBody{
        exchangedMomentBody.x + pendulumMoment.x + payloadLoads.rollMomentNm,
        exchangedMomentBody.y + pendulumMoment.y + payloadLoads.pitchMomentNm,
        exchangedMomentBody.z + pendulumMoment.z};
    // Inertia about the canopy, with the payload on its arm - m L^2 dominates
    // anything the canopy contributes on its own, and it is what sets the
    // pendulum period the Level 3 tests measured.
    const double payloadArmInertia =
        PayloadMass.TotalKg() * PendulumLengthM * PendulumLengthM;
    const Vec3 inertia{
        95.0 + payloadArmInertia,
        120.0 + payloadArmInertia,
        150.0};
    // Damping is integrated implicitly, the rest explicitly. The measured yaw
    // damping runs to 2e5 Nm per rad/s against an inertia of 150, which is a
    // time constant of under a millisecond - an explicit c*omega at 120 Hz
    // overshoots it by a factor of eleven, alternates sign and doubles every
    // step. That is what sent asymmetric brake to an infinite turn rate in
    // fourteen seconds. Backward Euler on the damping term alone is
    // unconditionally stable at any coefficient and costs one divide:
    //   omega' = (omega + M/I dt) / (1 + c/I dt)
    const Vec3 angularAcceleration{
        undampedMomentBody.x / inertia.x,
        undampedMomentBody.y / inertia.y,
        undampedMomentBody.z / inertia.z};
    const Vec3 unrelaxedRate =
        state.angularVelocityBodyRadps + angularAcceleration * dt;
    state.angularVelocityBodyRadps = Vec3{
        unrelaxedRate.x / (1.0 + dampingNmPerRadps.x * dt / inertia.x),
        unrelaxedRate.y / (1.0 + dampingNmPerRadps.y * dt / inertia.y),
        unrelaxedRate.z / (1.0 + dampingNmPerRadps.z * dt / inertia.z)};
    // The damping moment actually applied, at the rate it was applied to, so
    // the reported net moment is the one the state integrated.
    const Vec3 damped{
        -dampingNmPerRadps.x * state.angularVelocityBodyRadps.x,
        -dampingNmPerRadps.y * state.angularVelocityBodyRadps.y,
        -dampingNmPerRadps.z * state.angularVelocityBodyRadps.z};
    const Vec3 momentBody = undampedMomentBody + damped;
    // Structural damping: the wing is not a rigid body and its rotations are
    // resisted by the lines it hangs on. Without this the pendulum rings.
    const double damping = std::clamp(1.0 - 1.6 * dt, 0.0, 1.0);
    state.angularVelocityBodyRadps = state.angularVelocityBodyRadps * damping;
    state.attitude = IntegrateAttitude(
        state.attitude, state.angularVelocityBodyRadps, dt);

    diagnostics.netMomentBodyNm = momentBody;

    // Energy accounting. The work the aerodynamic force does on the system in
    // this step, against the change in kinetic plus potential energy. A
    // subsystem that creates energy cannot hide from this.
    const double kineticAfter = 0.5 * SystemMassKg
        * Dot(state.velocityWorldMps, state.velocityWorldMps);
    const double potentialAfter =
        SystemMassKg * GravityMps2 * state.positionWorldM.z;
    const double workDone = Dot(aeroWorld, state.velocityWorldMps * dt);
    diagnostics.kineticEnergyJ = kineticAfter;
    diagnostics.potentialEnergyJ = potentialAfter;
    diagnostics.energyResidualW =
        ((kineticAfter - kineticBefore) + (potentialAfter - potentialBefore)
         - workDone) / std::max(1.0e-9, dt);

    const Vec3 span = state.attitude.Rotate({0.0, 1.0, 0.0});
    diagnostics.bankRad = std::asin(std::clamp(-span.z, -1.0, 1.0));
    diagnostics.turnRateRadps = state.angularVelocityBodyRadps.z;

    state.initialised = true;
    LastDiagnostics = diagnostics;
}
}
