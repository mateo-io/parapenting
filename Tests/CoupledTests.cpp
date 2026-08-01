// Level 7: the six solvers run together, in order, and the books balance.
//
// The plan's Level 7 exit gates: no subsystem creates unaccounted net force or
// moment; still-air trim stays stable; a brake pulse exchanges speed, height
// and pitch energy without creating any; weight shift and brake turns emerge
// without direct turn moments; and the coupling iteration count can be reduced
// by one without a qualitative change - which is the difference between a
// converged solve and one tuned to its own budget.
#include "CanopyGeometry.h"
#include "CoupledParagliderSolver.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>

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

struct Run
{
    CoupledState state;
    CoupledDiagnostics last;
    double worstEnergyResidualW = 0.0;
    double worstClosureN = 0.0;
    double startAltitudeM = 0.0;
};

Run Fly(CoupledParagliderSolver& solver, const CoupledControls& controls,
        double seconds, CoupledState* carry = nullptr)
{
    Run run;
    if (carry) run.state = *carry;
    run.startAltitudeM = run.state.positionWorldM.z;
    const CoupledAtmosphere air;
    const int steps = static_cast<int>(seconds / solver.Schedule().timeStepS);
    for (int step = 0; step < steps; ++step)
    {
        solver.Step(run.state, controls, air);
        const CoupledDiagnostics& d = solver.Diagnostics();
        // Ignore the first half second, which is the wing settling from its
        // initial condition rather than flying.
        if (step > 60)
        {
            run.worstEnergyResidualW =
                std::max(run.worstEnergyResidualW, std::fabs(d.energyResidualW));
            run.worstClosureN =
                std::max(run.worstClosureN, d.internalForceClosureN);
        }
    }
    run.last = solver.Diagnostics();
    if (carry) *carry = run.state;
    return run;
}
}

int main()
{
    const CanopyGeometry canopy;
    CoupledParagliderSolver solver(canopy, Epic2MlLinePlan());
    std::printf("Coupled solver: %.1f kg all-up, schedule %.0f Hz, "
                "aero every %d steps, %d coupling iterations\n",
                solver.AllUpMassKg(), 1.0 / solver.Schedule().timeStepS,
                solver.Schedule().aerodynamicsInterval,
                solver.Schedule().couplingIterations);

    // -- hands-off flight --------------------------------------------------
    {
        const Run hands = Fly(solver, CoupledControls{}, 20.0);
        const double sink =
            (hands.startAltitudeM - hands.state.positionWorldM.z) / 20.0;
        std::printf("Hands off, 20 s: airspeed %.2f m/s, sink %.2f m/s, "
                    "alpha %.1f deg, Cp %.2f\n",
                    hands.last.airspeedMps, sink,
                    hands.last.angleOfAttackRad * 180.0 / 3.14159265358979,
                    hands.last.meanPressureCoefficient);
        std::printf("  worst energy residual %.3f W, worst internal closure "
                    "%.2e N\n",
                    hands.worstEnergyResidualW, hands.worstClosureN);

        Check(std::isfinite(hands.last.airspeedMps)
              && std::isfinite(hands.state.positionWorldM.z),
              "the coupled solve stays finite");
        Check(hands.last.airspeedMps > 5.0 && hands.last.airspeedMps < 25.0,
              "and flies at a speed a wing flies at");
        Check(sink > 0.0, "descending, as an unpowered wing does");

        // Equal and opposite, across six solvers. Every line tension appears
        // twice with opposite sign, so the sum over both ends is zero - and
        // that has to hold when the network is being driven by the coupling
        // rather than solved on its own.
        Check(hands.worstClosureN < 1.0e-6,
              "no subsystem creates unaccounted internal force - every line "
              "tension still appears twice with opposite sign");

        // Energy. The only work on the system is the aerodynamic force; the
        // rest is exchange between kinetic and potential.
        const double weightN = solver.AllUpMassKg() * 9.80665;
        Check(hands.worstEnergyResidualW < 0.02 * weightN,
              "and no subsystem creates energy - the work the air does "
              "accounts for the change in kinetic plus potential");

        Check(hands.last.meanPressureCoefficient > 0.7,
              "the cells are pressurised by the flight the solve is producing, "
              "not by an assumption");
    }

    // -- ten minutes -------------------------------------------------------
    {
        // The stability gate. Ten minutes at 120 Hz is 72000 coupled steps.
        CoupledParagliderSolver longRun(canopy, Epic2MlLinePlan());
        CoupledState state;
        const CoupledAtmosphere air;
        const CoupledControls hands;
        double worstEnergy = 0.0;
        double minSpeed = 1.0e9;
        double maxSpeed = 0.0;
        for (int step = 0; step < 120 * 600; ++step)
        {
            longRun.Step(state, hands, air);
            if (step < 120) continue;
            const CoupledDiagnostics& d = longRun.Diagnostics();
            worstEnergy = std::max(worstEnergy, std::fabs(d.energyResidualW));
            minSpeed = std::min(minSpeed, d.airspeedMps);
            maxSpeed = std::max(maxSpeed, d.airspeedMps);
        }
        std::printf("Ten minutes hands off: airspeed %.2f to %.2f m/s, "
                    "final %.2f, worst energy residual %.3f W\n",
                    minSpeed, maxSpeed, longRun.Diagnostics().airspeedMps,
                    worstEnergy);
        Check(std::isfinite(longRun.Diagnostics().airspeedMps),
              "ten minutes of still air stays finite");
        Check(maxSpeed - minSpeed < 6.0,
              "and bounded - the wing settles rather than wandering or "
              "diverging");
        Check(std::fabs(longRun.Diagnostics().bankRad) < 0.02,
              "a symmetric wing in symmetric air does not develop a bank over "
              "ten minutes");
    }

    // -- a brake pulse exchanges energy, it does not make it ---------------
    {
        CoupledParagliderSolver pulsed(canopy, Epic2MlLinePlan());
        CoupledState state;
        Fly(pulsed, CoupledControls{}, 8.0, &state);
        const double speedBefore = pulsed.Diagnostics().airspeedMps;
        const double altitudeBefore = state.positionWorldM.z;

        // 0.35, not 0.55. With this polar, half brake at trim incidence puts
        // the sections past their stall angle, and deep stall is the regime
        // the VSM cannot settle in - the separated branch has a negative lift
        // slope, so there is no steady state to find. That is a real
        // limitation and it is tested for separately below, rather than being
        // wrapped into a gate about energy.
        CoupledControls both;
        both.leftBrake = 0.35;
        both.rightBrake = 0.35;
        const Run pulling = Fly(pulsed, both, 1.5, &state);
        const Run releasing = Fly(pulsed, CoupledControls{}, 6.0, &state);

        std::printf("Brake pulse: %.2f -> %.2f m/s, %.1f m of height, worst "
                    "energy residual %.3f W\n",
                    speedBefore, releasing.last.airspeedMps,
                    altitudeBefore - state.positionWorldM.z,
                    std::max(pulling.worstEnergyResidualW,
                             releasing.worstEnergyResidualW));
        const double weightN = solver.AllUpMassKg() * 9.80665;
        Check(std::max(pulling.worstEnergyResidualW,
                       releasing.worstEnergyResidualW) < 0.05 * weightN,
              "a brake pulse exchanges speed and height without creating "
              "energy");
        Check(std::isfinite(releasing.last.airspeedMps),
              "and the wing survives the transient");
    }

    // -- turns emerge ------------------------------------------------------
    {
        // Guiding rule 2: no direct control-to-yaw or control-to-bank. There
        // is no such term anywhere in the coupled solver, so a turn can only
        // arrive through the line network and the spanwise aerodynamics.
        const auto turnWith = [&](const CoupledControls& controls)
        {
            CoupledParagliderSolver turning(canopy, Epic2MlLinePlan());
            CoupledState state;
            Fly(turning, CoupledControls{}, 6.0, &state);
            const Run run = Fly(turning, controls, 10.0, &state);
            return run;
        };

        CoupledControls rightBrake;
        rightBrake.rightBrake = 0.35;
        CoupledControls leftBrake;
        leftBrake.leftBrake = 0.35;
        CoupledControls rightShift;
        rightShift.weightShift = 1.0;

        const Run braked = turnWith(rightBrake);
        const Run mirrored = turnWith(leftBrake);
        const Run shifted = turnWith(rightShift);
        std::printf("Right brake: bank %+.3f rad, turn rate %+.3f rad/s\n",
                    braked.last.bankRad, braked.last.turnRateRadps);
        std::printf("Right weight shift: bank %+.3f rad, carabiner "
                    "%.0f / %.0f N\n",
                    shifted.last.bankRad, shifted.last.leftCarabinerLoadN,
                    shifted.last.rightCarabinerLoadN);

        Check(std::fabs(braked.last.bankRad) > 1.0e-4,
              "asymmetric brake banks the wing, through the spanwise "
              "aerodynamics rather than a roll term");
        Check(std::fabs(mirrored.last.bankRad + braked.last.bankRad)
                  < 1.0e-6,
              "and left and right mirror exactly");
        Check(shifted.last.rightCarabinerLoadN
                  > shifted.last.leftCarabinerLoadN,
              "weight shift loads the carabiner on the side it is shifted "
              "toward");
        Check(std::fabs(shifted.last.bankRad) > 1.0e-5,
              "and banks the wing through that load rather than a moment");
    }

    // -- heavy brake stalls the wing, and the solve survives it ------------
    {
        // Heavy brake takes the sections past stall. The VSM has no steady
        // answer on the separated branch on its own - a negative lift slope
        // inverts the downwash feedback between sections - but inside the
        // coupled solve it is not asked for one: Level 4's separation state is
        // carried between steps, so each solve sees a single-valued lift curve
        // and the coupling walks the wing into stall rather than jumping onto
        // the far branch.
        //
        // This check used to assert the opposite - that the numerical safety
        // envelope engaged here - because the rotation probes were solved cold
        // and unconverged, which put a different wing's load into the
        // structure and made the envelope necessary. With the probes taking
        // the wing's own separation state the envelope is no longer needed at
        // this brake setting, so what is asserted now is the physics.
        const auto brakedTo = [&](double brake)
        {
            CoupledParagliderSolver stalling(canopy, Epic2MlLinePlan());
            CoupledState state;
            Fly(stalling, CoupledControls{}, 6.0, &state);
            CoupledControls deep;
            deep.leftBrake = brake;
            deep.rightBrake = brake;
            const double topM = state.positionWorldM.z;
            const CoupledAtmosphere air;
            bool everRejected = false;
            for (int step = 0; step < 120 * 8; ++step)
            {
                stalling.Step(state, deep, air);
                if (stalling.Diagnostics().aerodynamicsRejected)
                    everRejected = true;
            }
            double separation = 0.0;
            for (const double s : state.separation.sectionSeparation)
                separation += s;
            separation /= static_cast<double>(
                std::max<std::size_t>(1, state.separation.sectionSeparation.size()));
            struct Stalled
            {
                double sinkMps;
                double separation;
                double angleOfAttackRad;
                double pressureCoefficient;
                double airspeedMps;
                bool rejected;
                bool finite;
            };
            return Stalled{
                (topM - state.positionWorldM.z) / 8.0, separation,
                stalling.Diagnostics().angleOfAttackRad,
                stalling.Diagnostics().meanPressureCoefficient,
                stalling.Diagnostics().airspeedMps, everRejected,
                std::isfinite(stalling.Diagnostics().airspeedMps)
                    && std::isfinite(state.positionWorldM.z)};
        };
        const auto light = brakedTo(0.35);
        const auto deep = brakedTo(0.9);
        std::printf("Brake 0.35: sink %.2f m/s, alpha %.1f deg, separation "
                    "%.2f, Cp %.2f\n",
                    light.sinkMps, light.angleOfAttackRad * 180.0 / 3.14159265358979,
                    light.separation, light.pressureCoefficient);
        std::printf("Brake 0.90: sink %.2f m/s, alpha %.1f deg, separation "
                    "%.2f, Cp %.2f, envelope engaged %s\n",
                    deep.sinkMps, deep.angleOfAttackRad * 180.0 / 3.14159265358979,
                    deep.separation, deep.pressureCoefficient,
                    deep.rejected ? "yes" : "no");

        Check(light.finite && deep.finite,
              "heavy brake stays finite - the structure is never handed a "
              "diverged aerodynamic load");
        Check(deep.separation > 0.95 && light.separation < 0.1,
              "heavy brake separates the sections and trim brake does not");
        Check(deep.sinkMps > 3.0 * light.sinkMps,
              "and a separated wing descends far faster than an attached one, "
              "which is what deep stall is");
        Check(deep.pressureCoefficient < 0.5
              && light.pressureCoefficient > 0.85,
              "the cells lose their pressure with the flow that fed them");
        Check(!deep.rejected,
              "and the solve holds there on its own - with the separation "
              "state carried, the numerical safety envelope is not needed to "
              "get through a stall");
    }

    // -- the coupling is converged, not budgeted ---------------------------
    {
        // The gate that separates a solved coupling from one that happens to
        // look right at the iteration count it was written with.
        const auto flyWith = [&](int couplingIterations)
        {
            CoupledSchedule schedule;
            schedule.couplingIterations = couplingIterations;
            CoupledParagliderSolver variant(
                canopy, Epic2MlLinePlan(), schedule);
            CoupledState state;
            const Run run = Fly(variant, CoupledControls{}, 12.0, &state);
            return run.last;
        };
        const CoupledDiagnostics three = flyWith(3);
        const CoupledDiagnostics two = flyWith(2);
        std::printf("Coupling iterations: airspeed %.3f at 3, %.3f at 2 "
                    "(residual %.2e)\n",
                    three.airspeedMps, two.airspeedMps,
                    three.couplingResidual);
        Check(std::fabs(three.airspeedMps - two.airspeedMps)
                  < 0.02 * three.airspeedMps,
              "taking one coupling iteration away changes nothing qualitative "
              "- the staggered solve has converged rather than been tuned to "
              "its budget");
        Check(three.couplingResidual < 0.05,
              "and the last iteration barely moves the exchanged load");
    }

    // -- determinism -------------------------------------------------------
    {
        const auto run = [&]()
        {
            CoupledParagliderSolver repeatable(canopy, Epic2MlLinePlan());
            CoupledState state;
            CoupledControls controls;
            controls.rightBrake = 0.4;
            controls.weightShift = 0.5;
            Fly(repeatable, controls, 6.0, &state);
            return state;
        };
        const CoupledState first = run();
        const CoupledState second = run();
        Check(first.positionWorldM.x == second.positionWorldM.x
              && first.positionWorldM.y == second.positionWorldM.y
              && first.positionWorldM.z == second.positionWorldM.z,
              "the same inputs give a bit-identical trajectory through all "
              "six solvers");
    }

    if (Failures == 0) std::printf("All coupled checks passed.\n");
    else std::printf("%d coupled check(s) failed.\n", Failures);
    return Failures == 0 ? 0 : 1;
}
