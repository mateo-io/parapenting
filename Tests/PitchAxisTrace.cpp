// Level 10, strand 4: an instrument for item 11.
//
// The pitch axis is the open physics problem, and this is what measures it.
//
// It was built to settle an argument about whether brake reaches the wing with
// the wrong sign. It answered a different question instead: NOTHING HERE HAD
// EVER BEEN SETTLED. This aircraft has a second, slow pitch mode - period
// 16.3 s, damping ratio 0.030 - and it needs eight to sixteen minutes to stand
// still. Every settle this project has used was 20, 40 or 60 seconds, and two
// conclusions were drawn off those runs and both were wrong: that brake had
// the wrong sign, and that the wing had a limit cycle. Settled to a criterion,
// brake raises incidence, which is correct.
//
// So the rule this file exists to enforce: FLY TO A CRITERION, NOT A CLOCK,
// and print how long it took beside every number. A fixed settle is a guess
// whatever value is in it. See PHYSICS_LEARNINGS section 33.
//
// What it reports:
//
//   * the slow mode itself, period and damping off successive peaks, against
//     the classical phugoid - which it disagrees with by 3.4x on period, and
//     that disagreement is now item 11's lead;
//   * what brake COMMANDS against what the wing does. Pulling brake shortens
//     the brake run and rotates the canopy nose-up on its suspension - that is
//     geometry, off the built graph, reported as `brakeCommandedSwingRad` -
//     while the section's flap couple pushes the other way;
//   * whether the aerodynamic interval or the swing damping ratio move any of
//     it.
//
// It is NOT a gate. Item 11 is registered as an open disagreement precisely so
// that nobody is tempted to gate this into agreeing with itself.
//
// A note on where this lives. This was going to be the in-engine research
// visualisation of PHYSICS_TODO item 16, until the obvious was checked:
// `ParagliderPawn` holds `ParagliderDynamics` and nothing else, so a view in
// the pawn shows the LEGACY model and cannot show any of the below. The
// coupled solver is not flown by the game at all - that is item 7, and
// removing the legacy path is item 17, blocked on this very problem. So the
// instrument for item 11 has to be headless until item 17 lands, and this is
// it.
#include "CanopyGeometry.h"
#include "CoupledParagliderSolver.h"
#include "SuspensionGraph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace Parapenting::Physics;

namespace
{
constexpr double Pi = 3.14159265358979323846;
constexpr double Degrees = 180.0 / Pi;

struct Settled
{
    double airspeedMps = 0.0;
    double incidenceRad = 0.0;
    double commandedSwingRad = 0.0;
    double actualSwingRad = 0.0;
    double aeroPitchMomentNm = 0.0;
    double linePitchMomentNm = 0.0;
    double linePitchStiffnessNmPerRad = 0.0;
    double liftCoefficient = 0.0;
    // Whether the last ten seconds actually stood still, and whether the
    // aircraft is still an aircraft. Without these two the table below reads
    // an oscillation as a trim point and a departure as a data row - and
    // reading a trim number off an unsettled run is the mistake this project
    // has already made once and written down (PHYSICS_LEARNINGS section 30).
    double incidenceSpreadRad = 0.0;
    int settleSeconds = 0;
    double airspeedSpreadMps = 0.0;
    bool settled = false;
    bool departed = false;
};

Settled Fly(const CanopyGeometry& canopy, const LinePlanSpec& linePlan,
            double brake, double swingDampingRatio,
            int aerodynamicsInterval = 12)
{
    CoupledSchedule schedule;
    schedule.aerodynamicsInterval = aerodynamicsInterval;
    CoupledParagliderSolver solver(canopy, linePlan, schedule);
    solver.SetSwingDampingRatio(swingDampingRatio);
    CoupledState state;
    CoupledControls controls;
    controls.leftBrake = brake;
    controls.rightBrake = brake;
    // Fly until it is settled, not for a fixed time, and report how long that
    // took. This wing's long mode decays with a time constant of minutes, so
    // ANY fixed settle is either wrong or a guess - and the guesses this
    // project has made were 20, 40 and 60 seconds, all far too short. Settling
    // to a criterion is the only version of this that is a measurement.
    constexpr int WindowSteps = 120 * 10;
    constexpr double SpreadToleranceRad = 1.7e-4;   // 0.01 deg
    constexpr int MaximumSeconds = 1200;

    double lowAlpha = 0.0, highAlpha = 0.0;
    double lowSpeed = 0.0, highSpeed = 0.0;
    int elapsedSeconds = 0;
    bool converged = false;
    while (elapsedSeconds < MaximumSeconds)
    {
        lowAlpha = 1.0e9; highAlpha = -1.0e9;
        lowSpeed = 1.0e9; highSpeed = -1.0e9;
        for (int step = 0; step < WindowSteps; ++step)
        {
            solver.Step(state, controls, CoupledAtmosphere{});
            const double alpha = solver.Diagnostics().angleOfAttackRad;
            const double speed = solver.Diagnostics().airspeedMps;
            lowAlpha = std::min(lowAlpha, alpha);
            highAlpha = std::max(highAlpha, alpha);
            lowSpeed = std::min(lowSpeed, speed);
            highSpeed = std::max(highSpeed, speed);
        }
        elapsedSeconds += 10;
        if (highAlpha - lowAlpha < SpreadToleranceRad
            && highSpeed - lowSpeed < 0.01)
        {
            converged = true;
            break;
        }
        if (solver.Diagnostics().angleOfAttackRad > 0.35) break;
    }

    const CoupledDiagnostics& d = solver.Diagnostics();
    Settled out;
    out.settleSeconds = elapsedSeconds;
    out.incidenceSpreadRad = highAlpha - lowAlpha;
    out.airspeedSpreadMps = highSpeed - lowSpeed;
    out.settled = converged;
    // Past about 20 degrees this wing is separated and what it reports is a
    // departure, not a trim. Comparing two departures and announcing which had
    // the higher incidence is nonsense, so it is labelled instead.
    out.departed = d.angleOfAttackRad > 0.35;
    out.airspeedMps = d.airspeedMps;
    out.incidenceRad = d.angleOfAttackRad;
    out.commandedSwingRad = d.brakeCommandedSwingRad;
    out.actualSwingRad = d.payloadSwingRad;
    out.aeroPitchMomentNm = d.aeroPitchMomentNm;
    out.linePitchMomentNm = d.linePitchMomentNm;
    out.linePitchStiffnessNmPerRad = d.linePitchStiffnessNmPerRad;
    const double dynamicPressure =
        0.5 * 1.12 * d.airspeedMps * d.airspeedMps;
    out.liftCoefficient = dynamicPressure > 1.0
        ? solver.AllUpMassKg() * 9.80665
            / (dynamicPressure * 27.0)
        : 0.0;
    return out;
}

// The slow mode itself: period and damping, off successive peaks of a long
// hands-up run. This is what identified it, and it is the measurement the item
// now turns on, so it lives here rather than in a scratch file.
void ReportSlowMode(const CanopyGeometry& canopy, const LinePlanSpec& linePlan)
{
    CoupledParagliderSolver solver(canopy, linePlan);
    CoupledState state;
    constexpr int Seconds = 1200;
    std::vector<double> alpha;
    std::vector<double> speed;
    alpha.reserve(Seconds);
    speed.reserve(Seconds);
    for (int second = 0; second < Seconds; ++second)
    {
        for (int step = 0; step < 120; ++step)
            solver.Step(state, CoupledControls{}, CoupledAtmosphere{});
        alpha.push_back(solver.Diagnostics().angleOfAttackRad * Degrees);
        speed.push_back(solver.Diagnostics().airspeedMps);
    }

    // Local maxima, one second apart being far finer than a mode this slow -
    // but ONLY those still carrying real amplitude. Once the mode has decayed
    // into the last thousandth of a degree the trace is numerical noise, and a
    // bare local-maximum test fires on every wiggle of it: the first version
    // of this found 106 peaks in 1200 s where the mode has about 75, pulling
    // the measured period down by a third. A peak detector needs a floor.
    const double settledAlpha = alpha.back();
    constexpr double MinimumAmplitudeDeg = 0.005;
    std::vector<int> peaks;
    for (std::size_t i = 3; i + 3 < alpha.size(); ++i)
    {
        if (alpha[i] - settledAlpha < MinimumAmplitudeDeg) continue;
        if (alpha[i] > alpha[i - 3] && alpha[i] > alpha[i + 3]
            && alpha[i] > alpha[i - 1] && alpha[i] >= alpha[i + 1])
        {
            peaks.push_back(static_cast<int>(i));
        }
    }

    std::printf("The slow mode: period and damping off %zu peaks of a %d s "
                "hands-up run\n", peaks.size(), Seconds);
    if (peaks.size() < 6)
    {
        std::printf("  too few peaks to measure\n\n");
        return;
    }

    // Skip the first peaks: the run starts on a transient that is not this
    // mode, and a log decrement measured across it is measuring both.
    const std::size_t first = 2;
    const std::size_t last = peaks.size() - 2;
    const double periodS =
        static_cast<double>(peaks[last] - peaks[first])
            / static_cast<double>(last - first);
    const double firstAmplitude = alpha[peaks[first]] - settledAlpha;
    const double lastAmplitude = alpha[peaks[last]] - settledAlpha;
    double dampingRatio = 0.0;
    if (firstAmplitude > 0.0 && lastAmplitude > 0.0)
    {
        const double decrement = std::log(firstAmplitude / lastAmplitude)
            / static_cast<double>(last - first);
        dampingRatio = decrement / (2.0 * Pi);
    }

    // Classical phugoid, for something to disagree with. Both formulas are for
    // a rigid aircraft with its mass at the wing, which this is not - so a
    // discrepancy is expected. Its SIZE is the finding.
    const double settledSpeed = speed.back();
    const double classicalPeriodS =
        Pi * std::sqrt(2.0) * settledSpeed / 9.80665;
    constexpr double GlideRatio = 11.33;
    const double classicalDamping = 1.0 / (std::sqrt(2.0) * GlideRatio);

    std::printf("  incidence settles at %.4f deg, airspeed %.4f m/s\n",
                settledAlpha, settledSpeed);
    std::printf("  period        %6.2f s   against %5.2f s classical "
                "(pi V sqrt2 / g)\n", periodS, classicalPeriodS);
    std::printf("  damping ratio %6.3f     against %5.3f classical "
                "(1 / (sqrt2 L/D))\n", dampingRatio, classicalDamping);
    std::printf("  period is %.1fx theory\n", periodS / classicalPeriodS);
    std::printf("\n  Incidence and airspeed move in antiphase, which is what "
                "makes this the\n  phugoid rather than the pendulum mode "
                "calibration_tests measures. Why the\n  period is so much "
                "longer than theory is item 11's open question.\n\n");
}
}

int main(int argc, char** argv)
{
    const bool slowModeOnly =
        argc > 1 && std::string(argv[1]) == "--slow-mode";
    std::printf("Level 10: the pitch axis, instrumented. PHYSICS_TODO item "
                "11.\n");
    std::printf("Nothing here asserts. Item 11 is an open disagreement and "
                "gating it would only\nteach the model to agree with "
                "itself.\n\n");

    const CanopyGeometry canopy;
    const LinePlanSpec linePlan = Epic2MlLinePlan();

    ReportSlowMode(canopy, linePlan);
    if (slowModeOnly) return 0;

    // -- what brake commands against what it gets --------------------------
    std::printf("Brake: what the line commands against what the wing does\n");
    std::printf("%8s %9s %11s %10s %9s %8s %s\n",
                "brake", "v m/s", "commanded", "alpha", "spread",
                "settle", "state");
    for (const double brake : {0.0, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35})
    {
        const Settled s = Fly(canopy, linePlan, brake, 0.35);
        std::printf("%8.2f %9.3f %10.2fd %9.3fd %8.4fd %7ds  %s\n",
                    brake, s.airspeedMps, s.commandedSwingRad * Degrees,
                    s.incidenceRad * Degrees,
                    s.incidenceSpreadRad * Degrees, s.settleSeconds,
                    s.departed ? "DEPARTED"
                        : s.settled ? "settled to 0.01d"
                        : "still moving at 1200 s");
    }
    std::printf("\n  Every row is flown until its incidence stops moving, not "
                "for a fixed time.\n  'settle' is how long that took. The "
                "20 to 60 second settles this project\n  has used everywhere "
                "else are off the bottom of this column.\n\n");

    // -- is the oscillation the wing, or the schedule? ---------------------
    //
    // Nothing above settles, and that is the finding. A phugoid with this
    // wing's measured period near 3 s and damping ratio near 0.28 is dead
    // after twenty periods; a residual still swinging half a degree at sixty
    // seconds in still air with no input is not a decaying mode.
    //
    // The first suspect is the schedule rather than the physics. The
    // aerodynamics run once every 12 steps and their loads are HELD in
    // between, so the structure is driven by a staircase at 10 Hz. A staircase
    // is an input, and an input sustains an oscillation. If the spread
    // collapses as the aerodynamic interval shortens, the limit cycle belongs
    // to the coupling scheme and not to the aircraft - which would mean every
    // trim number this project has quoted is a sample of a scheme artefact.
    std::printf("Hands-up spread against the aerodynamic interval\n");
    std::printf("%12s %11s %12s %12s %10s\n",
                "interval", "Hz", "alpha", "spread", "settle s");
    for (const int interval : {12, 6, 3, 2, 1})
    {
        const Settled s = Fly(canopy, linePlan, 0.0, 0.35, interval);
        std::printf("%12d %11.1f %11.2fd %11.4fd %10.3f\n",
                    interval, 120.0 / interval, s.incidenceRad * Degrees,
                    s.incidenceSpreadRad * Degrees,
                    static_cast<double>(s.settleSeconds));
    }
    std::printf("\n  And the same under 25%% brake, which takes far longer to "
                "settle:\n");
    std::printf("%12s %11s %12s %12s %10s\n",
                "interval", "Hz", "alpha", "spread", "settle s");
    for (const int interval : {12, 6, 3, 1})
    {
        const Settled s = Fly(canopy, linePlan, 0.25, 0.35, interval);
        std::printf("%12d %11.1f %11.2fd %11.4fd %10.3f\n",
                    interval, 120.0 / interval, s.incidenceRad * Degrees,
                    s.incidenceSpreadRad * Degrees,
                    static_cast<double>(s.settleSeconds));
    }
    std::printf("\n");

    // -- the one knob that is actually a knob ------------------------------
    //
    // The registry lists `linePitchStiffnessSpecificM` at 6.13 m, and this
    // file was written expecting to sweep it. It cannot be swept: it is not an
    // input anywhere in the solver. `LineStiffnessAt` interpolates a curve
    // MEASURED off the built suspension graph at four loads, and 6.13 is the
    // slope of that measurement written down afterwards. Changing it means
    // changing where the lines attach.
    //
    // So the sweep below is of the one number in the pitch axis that IS free:
    // `swingDampingRatio`, registered Tuned at 0.35, which the registry itself
    // says is standing in for a stabilising mechanism the model does not have.
    // The leading candidate for the limit cycle, and it is a LAG rather than a
    // stiffness. The link is damped against the WORLD, so it tracks apparent
    // gravity with a time constant of its own - the solver's own comment calls
    // that "a cost paid knowingly". A lag inside a feedback loop is the
    // textbook way to sustain an oscillation, and if that is what this is then
    // the cycle's amplitude must fall as the ratio rises.
    //
    // Ruled out without a run: the section's reattachment hysteresis, the
    // other classic limit-cycle mechanism here. Hands-up this wing flies at
    // 4.7 degrees against a section stall near 12, so the hysteresis loop is
    // nowhere near active and cannot be driving the hands-up cycle.
    std::printf("Swing damping ratio: what does the one tuned number actually buy?\n");
    std::printf("%10s %9s %10s %11s %12s\n",
                "ratio", "v m/s", "alpha", "settle", "vs trim");
    for (const double ratio : {0.25, 0.35, 0.50, 0.70, 0.90})
    {
        const Settled clean = Fly(canopy, linePlan, 0.0, ratio);
        const Settled braked = Fly(canopy, linePlan, 0.25, ratio);
        const char* verdict =
            (clean.departed || braked.departed)
                ? "DEPARTED - comparing two departures says nothing"
                : (!clean.settled || !braked.settled)
                    ? "NOT SETTLED - no verdict"
                    : braked.incidenceRad > clean.incidenceRad
                        ? "brake raises incidence, which is correct"
                        : "brake LOWERS incidence - wrong sign";
        std::printf("%10.2f %9.3f %9.3fd %9ds %11.3fd  %s\n",
                    ratio, braked.airspeedMps,
                    braked.incidenceRad * Degrees,
                    clean.settleSeconds,
                    (braked.incidenceRad - clean.incidenceRad) * Degrees,
                    verdict);
    }
    std::printf(
        "\n  The settle column is the answer, and the trim is NOT in it. From "
        "0.35 to 0.90\n  the wing settles at the same 5.72 deg and brake "
        "raises incidence by the same\n  0.79 deg - only the time to get "
        "there changes, 410 s down to 80 s. So the\n  ratio buys settling "
        "speed, not a trim, and the settled numbers do not depend\n  on the "
        "one tuned coefficient in this axis. That is worth more than it "
        "sounds.\n\n"
        "  At 0.25 the aircraft DEPARTS, which is what pins the value at 0.35 "
        "rather\n  than the 0.06 that pilot and line drag imply. Finding the "
        "stabilising\n  mechanism that would allow 0.06 is still the item.\n\n"
        "  Retracted: an earlier version of this text read the same sweep as "
        "evidence\n  of a LIMIT CYCLE whose amplitude the ratio suppressed. "
        "It is a decay rate,\n  not an amplitude. Sampling a decaying mode at "
        "a fixed time makes the two\n  look identical - see PHYSICS_LEARNINGS "
        "section 33.\n");

    return 0;
}
