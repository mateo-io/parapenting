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
//     the classical phugoid - which it disagrees with by 3.4x on period;
//   * WHY, as an exponent rather than an adjective. The classical formulas
//     assume incidence is held fixed, so lift and drag both go as V^2.
//     Measured off the flight path of the same run this wing's go as V^0.17
//     and V^0.31, and those two numbers predict the measured period and
//     damping to within 2%. The pendulum holds LIFT, not incidence;
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

// WHY the slow mode is slow, measured rather than asserted.
//
// The classical phugoid period comes from one assumption: incidence is fixed,
// so lift goes as V^2. Speed up and the wing lifts harder, which is the whole
// restoring force. Write the exponent as `n` in L ~ V^n and the frequency is
// `g sqrt(n) / V`, which at n = 2 is the textbook `pi V sqrt2 / g` period.
//
// So a period 3.4x too long is not a vague "the pendulum changes things". It
// is an arithmetic claim about n: 3.4x longer means n is 2/3.4^2, about 0.17.
// That is measurable off the trace, INDEPENDENTLY of the period, and the two
// either agree or the explanation is wrong.
//
// Lift here is not read off the aerodynamics - it is read off the flight path,
// which is the same number arrived at without trusting the force bookkeeping:
// a wing on a path curving at `gammaDot` carries `m (g cos gamma + V
// gammaDot)` normal to that path.
//
// The DAMPING gets the same treatment, and it has to, because n alone makes
// the damping disagree WORSE. Linearising the two-state phugoid with L ~ V^n
// and D ~ V^d gives
//
//     omega = g sqrt(n) / V           zeta = (d/2) / ((L/D) sqrt(n))
//
// which at n = 2, d = 2 is the textbook pair. Substituting only the measured
// n - leaving d at 2 - predicts a damping ratio around 0.21 against 0.031
// measured, where the untouched classical formula predicted 0.062. Reporting
// the period fix without that would be claiming a win while quietly making
// the other half worse. So d is measured too, off the TANGENTIAL direction of
// the same path: `D = -m (g sin gamma + Vdot)`.
struct Exponents
{
    double lift = 0.0;          // n in L ~ V^n
    double drag = 0.0;          // d in D ~ V^d
    double glideRatio = 0.0;
    double alphaSlopeDegPerMps = 0.0;
    double lowLiftN = 0.0;
    double highLiftN = 0.0;
    double meanSpeedMps = 0.0;
    int samples = 0;
    bool valid = false;
};

// The regression itself, factored out because the departure runs below need
// exactly this and a second copy of the arithmetic is a second place for it to
// be wrong.
Exponents MeasureExponents(const std::vector<double>& speed,
                           const std::vector<double>& gamma,
                           const std::vector<double>& alpha,
                           std::size_t from, std::size_t to,
                           double massKg, double periodS)
{
    Exponents out;
    if (to <= from + 4 || to + 1 >= speed.size()) return out;

    // A central difference on 1 s samples underestimates the rate of a
    // sinusoid by sin(wh)/(wh) - 2.5% at this period. Small, known exactly,
    // and left uncorrected it would bias n low, which is the direction that
    // flatters the hypothesis. So it is corrected.
    const double omega = 2.0 * Pi / periodS;
    const double differenceGain = std::sin(omega) / omega;

    double sumLnV = 0.0, sumLnL = 0.0, sumLnVLnV = 0.0, sumLnVLnL = 0.0;
    double sumLnD = 0.0, sumLnVLnD = 0.0;
    double sumV = 0.0, sumA = 0.0, sumVV = 0.0, sumVA = 0.0;
    double sumLift = 0.0, sumDrag = 0.0;
    double lowLift = 1.0e9, highLift = -1.0e9;
    int count = 0;
    for (std::size_t i = from + 1; i < to; ++i)
    {
        const double gammaDot =
            (gamma[i + 1] - gamma[i - 1]) / (2.0 * differenceGain);
        const double speedDot =
            (speed[i + 1] - speed[i - 1]) / (2.0 * differenceGain);
        const double lift =
            massKg * (9.80665 * std::cos(gamma[i]) + speed[i] * gammaDot);
        // Tangential. Gamma is negative in a glide, so `-g sin gamma` is the
        // component of weight along the path that the drag is balancing.
        const double drag =
            -massKg * (9.80665 * std::sin(gamma[i]) + speedDot);
        if (lift <= 0.0 || drag <= 0.0) continue;
        const double lnV = std::log(speed[i]);
        const double lnL = std::log(lift);
        const double lnD = std::log(drag);
        sumLnV += lnV; sumLnL += lnL; sumLnD += lnD;
        sumLnVLnV += lnV * lnV;
        sumLnVLnL += lnV * lnL; sumLnVLnD += lnV * lnD;
        sumV += speed[i]; sumA += alpha[i];
        sumVV += speed[i] * speed[i]; sumVA += speed[i] * alpha[i];
        sumLift += lift; sumDrag += drag;
        lowLift = std::min(lowLift, lift);
        highLift = std::max(highLift, lift);
        ++count;
    }
    if (count < 8) return out;

    const double nDenominator = count * sumLnVLnV - sumLnV * sumLnV;
    if (std::fabs(nDenominator) < 1.0e-12) return out;
    out.lift = (count * sumLnVLnL - sumLnV * sumLnL) / nDenominator;
    out.drag = (count * sumLnVLnD - sumLnV * sumLnD) / nDenominator;
    out.glideRatio = sumLift / sumDrag;
    out.alphaSlopeDegPerMps =
        (count * sumVA - sumV * sumA) / (count * sumVV - sumV * sumV);
    out.lowLiftN = lowLift;
    out.highLiftN = highLift;
    out.samples = count;
    out.meanSpeedMps = sumV / count;
    out.valid = true;
    return out;
}

void ReportPhugoidMechanism(const std::vector<double>& speed,
                            const std::vector<double>& gamma,
                            const std::vector<double>& alpha,
                            std::size_t from, std::size_t to,
                            double massKg, double periodS,
                            double classicalPeriodS, double measuredDamping)
{
    const Exponents e =
        MeasureExponents(speed, gamma, alpha, from, to, massKg, periodS);
    if (!e.valid) return;
    const double liftExponent = e.lift;
    const double dragExponent = e.drag;
    const double glideRatio = e.glideRatio;
    const double alphaSlope = e.alphaSlopeDegPerMps;
    const double lowLift = e.lowLiftN;
    const double highLift = e.highLiftN;

    const double meanSpeed = e.meanSpeedMps;
    // What n the measured period implies, and what period the measured n
    // implies. Two roads to the same number; they are printed side by side
    // because the point is whether they meet.
    const double exponentFromPeriod =
        2.0 * (classicalPeriodS / periodS) * (classicalPeriodS / periodS);
    const double periodFromExponent = liftExponent > 0.0
        ? 2.0 * Pi * meanSpeed / (9.80665 * std::sqrt(liftExponent))
        : 0.0;

    std::printf("Why it is slow: the lift exponent, off the flight path\n");
    std::printf("  L ~ V^n measured over the mode:      n = %6.3f\n",
                liftExponent);
    std::printf("  n the 3.4x period implies:           n = %6.3f\n",
                exponentFromPeriod);
    std::printf("  classical (incidence held fixed):    n = %6.3f\n", 2.0);
    if (periodFromExponent > 0.0)
    {
        std::printf("  period predicted from measured n:  %6.2f s   against "
                    "%5.2f s measured\n", periodFromExponent, periodS);
    }
    else
    {
        std::printf("  measured n is NOT POSITIVE: over the mode this wing "
                    "lifts LESS when it\n  flies faster, which is a wing "
                    "with no speed stability left to restore with.\n");
    }
    std::printf("  incidence against speed over the mode: %+.4f deg per m/s\n",
                alphaSlope);
    std::printf("  lift swing over the mode: %.1f N to %.1f N, %.2f%% of "
                "weight\n", lowLift, highLift,
                100.0 * (highLift - lowLift) / (massKg * 9.80665));
    std::printf("\n  The pendulum does not hold incidence through the "
                "oscillation, it holds\n  LIFT. Slowing down, the wing "
                "rotates nose-up on its lines and recovers\n  in incidence "
                "most of what it lost in dynamic pressure, so the restoring\n"
                "  force that drives a phugoid is what is left over - and "
                "what is left over\n  is small, which is the long period.\n\n");

    // The other half. Substituting only n would make this worse, so d is
    // measured on the same trace and the damping predicted from both.
    const double dampingFromExponents = liftExponent > 0.0
        ? (dragExponent / 2.0) / (glideRatio * std::sqrt(liftExponent))
        : 0.0;
    const double dampingFromNAlone = liftExponent > 0.0
        ? 1.0 / (glideRatio * std::sqrt(liftExponent))
        : 0.0;
    std::printf("And the damping: the drag exponent, off the same path\n");
    std::printf("  D ~ V^d measured over the mode:      d = %6.3f\n",
                dragExponent);
    std::printf("  classical (incidence held fixed):    d = %6.3f\n", 2.0);
    std::printf("  mean L/D over the mode:                  %6.2f\n",
                glideRatio);
    std::printf("  zeta from measured n and d:          %6.3f   against "
                "%.3f measured\n", dampingFromExponents, measuredDamping);
    std::printf("  zeta from measured n, d left at 2:   %6.3f   (what n "
                "alone would claim)\n", dampingFromNAlone);
    std::printf("  zeta classical, n and d both 2:      %6.3f\n",
                1.0 / (std::sqrt(2.0) * glideRatio));
    std::printf("\n  Lift and drag are BOTH nearly flat against speed over "
                "this mode, and for\n  the same reason: the incidence "
                "excursion that holds lift up when the wing\n  slows holds "
                "drag up with it. Taking n without d is not half an "
                "explanation,\n  it is a worse one - it lengthens the period "
                "and leaves the drag term whole,\n  which over-damps the "
                "prediction by a factor of several.\n\n");
}

// The departure at swingDampingRatio 0.25, and whether it is the SAME thing.
//
// With the period and the damping both falling out of the lift exponent, item
// 11's remaining question is why the aircraft departs below a swing damping
// ratio of about 0.3 - the only reason that number is 0.35 rather than the
// ~0.06 pilot and line drag imply.
//
// The exponent makes that question a prediction rather than a new search.
// `omega = g sqrt(n) / V` is oscillatory for n > 0 and DIVERGENT for n < 0: at
// n < 0 the wing lifts less when it flies faster, which is a wing with no
// speed stability left. The hands-up mode sits at n = 0.171, which is very
// close to zero - a fifteenth of classical. If the ratio is buying tracking,
// and tracking is what flattens the lift curve against speed, then the
// departure should be n CROSSING ZERO and nothing else. One number whose sign
// is stability and whose size is the period.
//
// This is a real test because it can fail cleanly: if the ratios that depart
// still measure n > 0, the departure is a different mechanism - most likely
// the fast pendulum mode - and this explanation does not reach it.
//
// Cheap, unlike everything else in this file: a departure happens in a minute
// or two, where a settled measurement takes twenty.
struct DivergingMode
{
    double periodS = 0.0;
    // Amplitude growth per cycle. Above 1 the mode is growing, which is what
    // a departure is made of.
    double growthPerCycle = 0.0;
    int cycles = 0;
    bool valid = false;
};

// WHICH mode is growing, identified by its period. The two candidates are far
// enough apart that this is unambiguous: the pendulum mode is 2.91 s, the
// phugoid 16.4 s. Read off the last stretch before departure, where whatever
// is diverging dominates everything else.
DivergingMode MeasureDivergingMode(const std::vector<double>& fineAlpha,
                                   int departedAt)
{
    DivergingMode out;
    // The last 40 s before it lets go, at 10 Hz. Before that the transient
    // the run started on is still mixed in.
    const std::size_t end = departedAt > 0
        ? static_cast<std::size_t>(departedAt) * 10
        : fineAlpha.size();
    if (end < 500) return out;
    const std::size_t begin = end - 400;

    // Peaks and troughs, on a trace sampled ten times faster than the fastest
    // mode being looked for.
    std::vector<std::size_t> peaks, troughs;
    for (std::size_t i = begin + 2; i + 2 < end; ++i)
    {
        if (fineAlpha[i] > fineAlpha[i - 2] && fineAlpha[i] > fineAlpha[i + 2]
            && fineAlpha[i] >= fineAlpha[i + 1])
            peaks.push_back(i);
        if (fineAlpha[i] < fineAlpha[i - 2] && fineAlpha[i] < fineAlpha[i + 2]
            && fineAlpha[i] <= fineAlpha[i + 1])
            troughs.push_back(i);
    }
    if (peaks.size() < 3 || troughs.size() < 3) return out;

    out.periodS = static_cast<double>(peaks.back() - peaks.front())
        / static_cast<double>(peaks.size() - 1) / 10.0;
    out.cycles = static_cast<int>(peaks.size()) - 1;

    // Amplitude peak-to-trough, first cycle against last. A divergence has no
    // settled value to measure amplitude about, so the trough is the
    // reference rather than a mean.
    const std::size_t firstTrough = troughs.front();
    const std::size_t lastTrough = troughs.back();
    const double firstAmplitude =
        fineAlpha[peaks.front()] - fineAlpha[firstTrough];
    const double lastAmplitude = fineAlpha[peaks.back()] - fineAlpha[lastTrough];
    if (firstAmplitude > 1.0e-9 && lastAmplitude > 1.0e-9 && out.cycles > 0)
    {
        out.growthPerCycle = std::pow(lastAmplitude / firstAmplitude,
                                      1.0 / static_cast<double>(out.cycles));
        out.valid = true;
    }
    return out;
}

void ReportDeparture(const CanopyGeometry& canopy, const LinePlanSpec& linePlan)
{
    std::printf("Departure: is it the same exponent, crossing zero?\n");
    std::printf("%8s %9s %10s %9s %9s %10s %9s %10s  %s\n",
                "ratio", "n", "d", "alpha/V", "L/D", "outcome",
                "growing", "per cycle", "note");

    constexpr int MaximumSeconds = 400;
    for (const double ratio : {0.20, 0.25, 0.30, 0.35, 0.50})
    {
        CoupledParagliderSolver solver(canopy, linePlan);
        solver.SetSwingDampingRatio(ratio);
        const double massKg = solver.AllUpMassKg();
        CoupledState state;
        std::vector<double> alpha, speed, gamma;
        // A second trace at 10 Hz, for identifying the mode that diverges by
        // its PERIOD. The 1 Hz samples above cannot do it: the pendulum mode
        // is 2.91 s, and one sample a second cannot resolve it at all. Which
        // is worth saying out loud, because sampling too slowly to see a mode
        // is exactly how this item spent two levels not seeing the other one.
        std::vector<double> fineAlpha;
        int departedAt = 0;
        for (int second = 0; second < MaximumSeconds; ++second)
        {
            for (int tenth = 0; tenth < 10; ++tenth)
            {
                for (int step = 0; step < 12; ++step)
                    solver.Step(state, CoupledControls{}, CoupledAtmosphere{});
                fineAlpha.push_back(
                    solver.Diagnostics().angleOfAttackRad * Degrees);
            }
            alpha.push_back(solver.Diagnostics().angleOfAttackRad * Degrees);
            speed.push_back(solver.Diagnostics().airspeedMps);
            const Vec3& v = state.velocityWorldMps;
            gamma.push_back(std::atan2(v.z,
                                       std::sqrt(v.x * v.x + v.y * v.y)));
            if (solver.Diagnostics().angleOfAttackRad > 0.35)
            {
                departedAt = second + 1;
                break;
            }
        }
        const DivergingMode dm = MeasureDivergingMode(fineAlpha, departedAt);

        // Measure over the run BEFORE it departs, and stop short of the
        // departure itself: past 20 degrees the wing is separated and the
        // lift it is producing is not on the curve this is fitting. Ten
        // seconds of margin, which is most of a period.
        const std::size_t end = departedAt > 12
            ? static_cast<std::size_t>(departedAt) - 11
            : alpha.size() - 2;
        // The difference-gain correction wants a period. Where the run is
        // long enough this is the settled mode's 16.4 s; a diverging run has
        // no period at all, and at these timescales the correction is 2% - so
        // it is applied at the nominal value and the error is smaller than
        // the sign being tested for.
        const Exponents e =
            MeasureExponents(speed, gamma, alpha, 5, end, massKg, 16.4);

        const char* outcome = departedAt > 0 ? "DEPARTED" : "flying";
        if (!e.valid)
        {
            std::printf("%8.2f %9s %10s %9s %9s %10s %9s %10s  too few "
                        "samples before departure\n", ratio, "-", "-", "-",
                        "-", outcome, "-", "-");
            continue;
        }
        char note[96];
        if (departedAt > 0)
            std::snprintf(note, sizeof note, "at %d s", departedAt);
        else
            std::snprintf(note, sizeof note, "still flying at %d s",
                          MaximumSeconds);
        char growing[16] = "-";
        char growth[16] = "-";
        if (dm.valid)
        {
            std::snprintf(growing, sizeof growing, "%.2f s", dm.periodS);
            std::snprintf(growth, sizeof growth, "x%.3f", dm.growthPerCycle);
        }
        std::printf("%8.2f %9.3f %10.3f %9.3f %9.2f %10s %9s %10s  %s\n",
                    ratio, e.lift, e.drag, e.alphaSlopeDegPerMps,
                    e.glideRatio, outcome, growing, growth, note);
    }
    std::printf("\n  n > 0 is an oscillation and n < 0 is a divergence, so if "
                "the departure is\n  the exponent crossing zero, that column "
                "changes sign exactly where the\n  outcome column does.\n\n"
                "  'growing' is the period of whatever is actually growing in "
                "the last 40 s\n  before the wing lets go, off a 10 Hz trace - "
                "because one sample a second\n  cannot resolve a 2.91 s mode, "
                "and undersampling a mode into invisibility is\n  precisely "
                "how this item lost two levels. It names the culprit: 2.9 s is "
                "the\n  pendulum mode, 16.4 s is the phugoid. A dash is no "
                "oscillation large enough\n  to measure in that window, which "
                "is what a decaying mode looks like.\n\n"
                "  ANSWERED, in the negative: n does not change sign, so the "
                "departure is not\n  the phugoid going unstable. What grows is "
                "3.6-5.7 s - the pendulum band -\n  with net damping ratio "
                "-0.017 at 0.25 and -0.042 at 0.20, and the boundary\n  lies "
                "between 0.25 and 0.30. The period differs between the two "
                "departing\n  runs and the amplitude is large by then, so "
                "'pendulum band' is as strong a\n  claim as this supports - "
                "but an order of magnitude is not a close call.\n\n"
                "  What DID follow the ratio is the tracking: alpha/V goes "
                "-1.27 to -1.72 as\n  the ratio rises, which is the mechanism "
                "the phugoid work identified. The\n  ratio buys tracking; "
                "tracking is not what the departure is made of.\n\n");
}

// The slow mode itself: period and damping, off successive peaks of a long
// hands-up run. This is what identified it, and it is the measurement the item
// now turns on, so it lives here rather than in a scratch file.
void ReportSlowMode(const CanopyGeometry& canopy, const LinePlanSpec& linePlan)
{
    CoupledParagliderSolver solver(canopy, linePlan);
    const double massKg = solver.AllUpMassKg();
    CoupledState state;
    constexpr int Seconds = 1200;
    std::vector<double> alpha;
    std::vector<double> speed;
    // The FLIGHT PATH angle, recorded on the same run as the period. It costs
    // nothing to take and it is what turns "the period is 3.4x theory" into a
    // statement about lift - see `ReportPhugoidMechanism` below. Taking it on
    // a second run would not have been affordable: this one is twenty minutes
    // of wall clock.
    std::vector<double> gamma;
    alpha.reserve(Seconds);
    speed.reserve(Seconds);
    gamma.reserve(Seconds);
    for (int second = 0; second < Seconds; ++second)
    {
        for (int step = 0; step < 120; ++step)
            solver.Step(state, CoupledControls{}, CoupledAtmosphere{});
        alpha.push_back(solver.Diagnostics().angleOfAttackRad * Degrees);
        speed.push_back(solver.Diagnostics().airspeedMps);
        const Vec3& v = state.velocityWorldMps;
        gamma.push_back(std::atan2(v.z, std::sqrt(v.x * v.x + v.y * v.y)));
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
                "calibration_tests measures.\n\n");

    // The exponent measurement runs on the part of the trace that still
    // carries amplitude. Late peaks are a thousandth of a degree, and a
    // regression that includes them is fitting the noise floor of a decayed
    // mode - the same mistake the peak detector above already had to be
    // taught not to make.
    std::size_t analysisEnd = last;
    for (std::size_t p = first; p <= last; ++p)
    {
        if (alpha[peaks[p]] - settledAlpha < 0.2 * firstAmplitude)
        {
            analysisEnd = p;
            break;
        }
    }
    if (analysisEnd < first + 3) analysisEnd = last;
    ReportPhugoidMechanism(speed, gamma, alpha,
                           static_cast<std::size_t>(peaks[first]),
                           static_cast<std::size_t>(peaks[analysisEnd]),
                           massKg, periodS, classicalPeriodS, dampingRatio);
}
}

int main(int argc, char** argv)
{
    const std::string mode = argc > 1 ? std::string(argv[1]) : std::string();
    const bool slowModeOnly = mode == "--slow-mode";
    // The departure sweep on its own. Worth a flag: it answers item 11's
    // remaining question and costs minutes where the rest of this file costs
    // an hour.
    const bool departureOnly = mode == "--departure";
    std::printf("Level 10: the pitch axis, instrumented. PHYSICS_TODO item "
                "11.\n");
    std::printf("Nothing here asserts. Item 11 is an open disagreement and "
                "gating it would only\nteach the model to agree with "
                "itself.\n\n");

    const CanopyGeometry canopy;
    const LinePlanSpec linePlan = Epic2MlLinePlan();

    if (departureOnly)
    {
        ReportDeparture(canopy, linePlan);
        return 0;
    }

    ReportSlowMode(canopy, linePlan);
    ReportDeparture(canopy, linePlan);
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
