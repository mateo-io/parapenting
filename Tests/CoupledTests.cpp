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

struct Weather
{
    CoupledAtmosphere air;
    // Seconds the gust lasts. After it the atmosphere is still again, which is
    // what makes a gust a gust rather than a new trim condition.
    double gustSeconds = 0.0;
};

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

// What the wing did to itself over a flight, rather than where it ended up. A
// collapse is a transient: reading only the final step says a wing that folded
// and recovered never folded at all.
struct FoldRun
{
    CoupledState state;
    CoupledDiagnostics last;
    double worstLeftCollapse = 0.0;
    double worstRightCollapse = 0.0;
    double worstCravat = 0.0;
    double worstTurnRateRadps = 0.0;
    // Signed, at the moment the wing was most lopsided. Which way it went is
    // the whole question in an asymmetric collapse - and it has to be read
    // while the wing is folded, not at the largest rate anywhere in the
    // flight: the recovery surge afterwards swings harder than the collapse
    // did, in the other direction, and it is the wing coming back rather than
    // the collapse doing anything.
    double turnAtWorstFoldRadps = 0.0;
    double worstFoldAsymmetry = 0.0;
    double worstBankRad = 0.0;
    bool safetyEnvelopeEngaged = false;
    double leftBrakeRowTensionN = 0.0;
};

FoldRun FlyThrough(CoupledParagliderSolver& solver,
                   const CoupledControls& controls, const Weather& weather,
                   double seconds, CoupledState* carry = nullptr)
{
    FoldRun run;
    if (carry) run.state = *carry;
    const int steps = static_cast<int>(seconds / solver.Schedule().timeStepS);
    const int gustSteps = static_cast<int>(
        weather.gustSeconds / solver.Schedule().timeStepS);
    for (int step = 0; step < steps; ++step)
    {
        CoupledAtmosphere air = weather.air;
        if (step >= gustSteps) air.gustWorldMps = Vec3{};
        solver.Step(run.state, controls, air);
        const CoupledDiagnostics& d = solver.Diagnostics();
        run.worstLeftCollapse =
            std::max(run.worstLeftCollapse, d.collapseState.leftCollapse);
        run.worstRightCollapse =
            std::max(run.worstRightCollapse, d.collapseState.rightCollapse);
        const double lopsided = std::fabs(d.collapseState.leftCollapse
                                          - d.collapseState.rightCollapse);
        if (lopsided > run.worstFoldAsymmetry)
        {
            run.worstFoldAsymmetry = lopsided;
            run.turnAtWorstFoldRadps = d.turnRateRadps;
        }
        run.worstCravat = std::max(run.worstCravat,
            std::max(d.collapseState.leftCravat,
                     d.collapseState.rightCravat));
        if (step > 60)
        {
            run.worstTurnRateRadps = std::max(
                run.worstTurnRateRadps, std::fabs(d.turnRateRadps));
            run.worstBankRad =
                std::max(run.worstBankRad, std::fabs(d.bankRad));
            if (d.aerodynamicsRejected) run.safetyEnvelopeEngaged = true;
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

    // -- Level 8: incident benchmarks --------------------------------------
    //
    // The plan's Level 8 exit gates, asked of the whole aircraft rather than
    // of the collapse solver on its own. Nothing here scripts a collapse: the
    // only thing done to the wing is air arriving at part of it, and what the
    // wing does about that is the answer.
    {
        // The symmetric case first, as the control: a gust across the whole
        // span with nothing asymmetric about it.
        const auto flyGust = [&](double gustMps, double from, double to,
                                 double gustSeconds, double totalSeconds,
                                 const CoupledControls& controls)
        {
            CoupledParagliderSolver wing(canopy, Epic2MlLinePlan());
            CoupledState state;
            // Settle first, so the gust hits a flying wing rather than an
            // initial condition.
            Fly(wing, controls, 8.0, &state);
            Weather weather;
            weather.air.gustWorldMps = Vec3{0.0, 0.0, gustMps};
            weather.air.gustSpanFrom = from;
            weather.air.gustSpanTo = to;
            weather.gustSeconds = gustSeconds;
            return FlyThrough(wing, controls, weather, totalSeconds, &state);
        };

        const CoupledControls handsOff;
        const FoldRun calm = flyGust(0.0, -1.0, 1.0, 0.0, 12.0, handsOff);
        std::printf("Level 8, still air: worst collapse L %.3f R %.3f, "
                    "turn rate %.4f rad/s\n",
                    calm.worstLeftCollapse, calm.worstRightCollapse,
                    calm.worstTurnRateRadps);
        Check(calm.worstLeftCollapse < 1.0e-3
              && calm.worstRightCollapse < 1.0e-3,
              "a wing flying in still air folds nothing - the collapse model "
              "is inert until something happens to the canopy");
        Check(!calm.safetyEnvelopeEngaged,
              "and the numerical safety envelope does not engage in nominal "
              "flight (guiding rule 12)");

        // A nominal manoeuvre: a brake turn, which raises incidence and feeds
        // the inlets. Brake stalls a wing; it does not fold one, and that has
        // to survive being asked of the whole aircraft.
        CoupledControls turning;
        turning.rightBrake = 0.35;
        turning.weightShift = 0.5;
        const FoldRun turn = flyGust(0.0, -1.0, 1.0, 0.0, 12.0, turning);
        std::printf("Level 8, half brake and weight shift: worst collapse "
                    "L %.3f R %.3f, bank %.1f deg\n",
                    turn.worstLeftCollapse, turn.worstRightCollapse,
                    turn.worstBankRad * 180.0 / 3.14159265358979);
        Check(turn.worstLeftCollapse < 1.0e-3
              && turn.worstRightCollapse < 1.0e-3,
              "and a braked turn folds nothing either - brake raises "
              "incidence, which is the wrong direction for a collapse");
        Check(!turn.safetyEnvelopeEngaged,
              "the safety envelope stays out of a nominal manoeuvre");

        // The asymmetric benchmark. Descending air over the left half only,
        // which is a rotor edge: it takes the incidence off that half without
        // touching the other.
        //
        // Four metres per second, which is a real gust and not an extreme
        // one. Past about five the wing does not come back, and that is worth
        // saying plainly because it is not this level's doing: a hard enough
        // gust pitches the canopy into full separation, and a fully separated
        // wing in this model descends vertically at 7.5 m/s and stays there.
        // That is the deep-stall attractor Level 11's unsteady wake is for,
        // documented in PHYSICS_TODO item 5. A collapse is what puts the wing
        // there; it is not what keeps it there.
        const FoldRun asymmetric =
            flyGust(-4.0, -1.0, 0.0, 1.0, 14.0, handsOff);
        std::printf("Level 8, 4 m/s down over the left half for 1 s: worst "
                    "collapse L %.3f R %.3f, cravat %.3f\n",
                    asymmetric.worstLeftCollapse,
                    asymmetric.worstRightCollapse, asymmetric.worstCravat);
        std::printf("  worst turn rate %.3f rad/s, worst bank %.1f deg, "
                    "recovered to L %.3f after 13 s\n",
                    asymmetric.worstTurnRateRadps,
                    asymmetric.worstBankRad * 180.0 / 3.14159265358979,
                    asymmetric.last.collapseState.leftCollapse);
        Check(asymmetric.worstLeftCollapse > 0.1,
              "air arriving down the left half folds it - the incidence drops "
              "there, the stagnation point climbs over the nose, and the "
              "suction that was holding the skin out becomes pressure "
              "pushing it in");
        Check(asymmetric.worstRightCollapse
                  < 0.5 * asymmetric.worstLeftCollapse,
              "on the half the air arrived at, not across the wing");
        Check(asymmetric.last.collapseState.leftCollapse
                  < 0.2 * asymmetric.worstLeftCollapse,
              "and it reopens once the gust has gone - the inlet is fed again "
              "and the pressure balance comes back");
        Check(!asymmetric.safetyEnvelopeEngaged,
              "through a collapse and a recovery without the numerical safety "
              "envelope engaging");

        // Which way it went. A wing with one half folded turns toward the
        // folded half, because that half stopped making lift and stopped
        // making its share of the drag last. Nothing in the solver knows
        // this: there is no collapse-to-yaw term anywhere, and the sign comes
        // out of where the remaining lift is.
        std::printf("  turning %+.3f rad/s at the worst of the fold\n",
                    asymmetric.turnAtWorstFoldRadps);
        Check(asymmetric.turnAtWorstFoldRadps < -0.05,
              "and it turns toward the folded half - which is a spin or a "
              "spiral entry, not a barrel roll");
        Check(asymmetric.worstBankRad < 1.2,
              "banking rather than rolling inverted");

        // A collapsed half carries no load, and the load it is not carrying
        // has to go somewhere. This is the exit gate about slack lines, read
        // where the coupled solver can answer it: the imbalance the collapse
        // hands the line network, which is what takes the tension out of the
        // lines under the folded half.
        double worstAsymmetry = 0.0;
        {
            CoupledParagliderSolver wing(canopy, Epic2MlLinePlan());
            CoupledState state;
            Fly(wing, handsOff, 8.0, &state);
            Weather weather;
            weather.air.gustWorldMps = Vec3{0.0, 0.0, -4.0};
            weather.air.gustSpanFrom = -1.0;
            weather.air.gustSpanTo = 0.0;
            weather.gustSeconds = 1.0;
            const int steps = static_cast<int>(6.0
                / wing.Schedule().timeStepS);
            const int gustSteps = static_cast<int>(1.0
                / wing.Schedule().timeStepS);
            for (int step = 0; step < steps; ++step)
            {
                CoupledAtmosphere air = weather.air;
                if (step >= gustSteps) air.gustWorldMps = Vec3{};
                wing.Step(state, handsOff, air);
                worstAsymmetry = std::max(worstAsymmetry,
                    wing.Diagnostics().collapseLoadAsymmetry);
            }
        }
        std::printf("  worst load asymmetry handed to the lines: %+.3f\n",
                    worstAsymmetry);
        Check(worstAsymmetry > 0.2,
              "the folded half stops carrying its share, and the line network "
              "is told so - which is what leaves the lines under it slack");

        // The symmetric benchmark, for the same gust across the whole span. A
        // frontal, and it has to be symmetric: the same air over both halves
        // must not produce a turn.
        const FoldRun frontal = flyGust(-4.0, -1.0, 1.0, 1.0, 14.0, handsOff);
        std::printf("Level 8, 4 m/s down over the whole span: worst collapse "
                    "L %.3f R %.3f, turn %.3f rad/s, recovered to %.3f\n",
                    frontal.worstLeftCollapse, frontal.worstRightCollapse,
                    frontal.worstTurnRateRadps,
                    frontal.last.collapseState.symmetricCollapse);
        Check(frontal.worstLeftCollapse > 0.1,
              "the same air over the whole span folds it too - a frontal");
        Check(std::fabs(frontal.worstLeftCollapse
                        - frontal.worstRightCollapse)
                  < 0.02 * frontal.worstLeftCollapse,
              "symmetrically - the two halves are the same wing");
        // Not bit-identical, and the reason is worth stating rather than
        // widening a tolerance over. Through the fold and the first second of
        // the recovery the two halves agree to 1e-15. Then the wing passes
        // through a partly separated transient where the VSM does not
        // converge - the same negative-lift-slope branch that makes deep stall
        // have no steady state, PHYSICS_TODO item 5 - and a non-converged
        // nonlinear solve turns rounding into a real difference within two
        // aerodynamic intervals. Level 11's unsteady wake is the honest fix.
        Check(frontal.worstTurnRateRadps < 0.15,
              "and a symmetric collapse does not turn the wing - what is left "
              "is the recovery transient, not a heading change");

        // Brake pumping. The plan's gate is that brake only reaches a collapse
        // when the brake line has tension, and this wing has 120 mm of slack
        // sewn into a 620 mm travel - so the first 19% of the handle's travel
        // moves through air and can do nothing to a fold, while a real pull
        // holds it in.
        const auto foldWithBrake = [&](double leftBrake)
        {
            CoupledControls held;
            held.leftBrake = leftBrake;
            CoupledParagliderSolver wing(canopy, Epic2MlLinePlan());
            CoupledState state;
            Fly(wing, CoupledControls{}, 8.0, &state);
            Weather weather;
            weather.air.gustWorldMps = Vec3{0.0, 0.0, -4.0};
            weather.air.gustSpanFrom = -1.0;
            weather.air.gustSpanTo = 0.0;
            weather.gustSeconds = 1.0;
            return FlyThrough(wing, held, weather, 6.0, &state);
        };
        const FoldRun handsUp = foldWithBrake(0.0);
        const FoldRun slackBrake = foldWithBrake(0.15);
        const FoldRun realBrake = foldWithBrake(0.80);
        std::printf("Level 8, brake on the folded side after 6 s: hands up "
                    "%.4f, 15%% travel %.4f, 80%% travel %.4f\n",
                    handsUp.last.collapseState.leftCollapse,
                    slackBrake.last.collapseState.leftCollapse,
                    realBrake.last.collapseState.leftCollapse);
        Check(std::fabs(slackBrake.last.collapseState.leftCollapse
                        - handsUp.last.collapseState.leftCollapse) < 1.0e-9,
              "brake inside the sewn-in slack does nothing to a collapse at "
              "all - the line is not transmitting, so there is nothing for it "
              "to do (guiding rule 3)");
        Check(realBrake.last.collapseState.leftCollapse
                  > handsUp.last.collapseState.leftCollapse,
              "and brake past the slack holds the fold in, which is why the "
              "recovery is to release that side first");
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
