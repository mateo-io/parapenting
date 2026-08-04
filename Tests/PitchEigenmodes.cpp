// Level 10, strand 5: the pitch axis by LINEARISATION, for PHYSICS_TODO item 11.
//
// Every time-trace instrument this project has built for the pitch axis ran
// aground on the same rock: two modes an order of magnitude apart share one
// signal, and the fast one is dead before the slow one has moved. Settling to a
// criterion fixed the trims. It did not fix the modes. Five successive
// instruments went at the fast mode's damping - window mean, fitted line, band
// pass, control-run subtraction, damped-sinusoid fit - and the last of them
// printed NOT REPORTABLE, which was the correct answer and not a useful one.
// PHYSICS_LEARNINGS sections 33 to 36.
//
// This does not look at a time trace at all.
//
// Perturb the settled aircraft one state at a time, run each perturbation for a
// fixed short time, and difference the results against an unperturbed run. That
// gives the STATE TRANSITION MATRIX over that time, and its eigenvalues are
// every longitudinal mode at once: period, damping ratio, and stability, with
//
//   * no excitation to design, so no mode is missed for being badly excited;
//   * no window, so a mode ending at 2.5 s and one lasting 16 minutes are read
//     from the same data;
//   * no filter, so nothing has to be assumed about the mode being removed;
//   * no superposition assumption, because linearity is the thing being
//     computed rather than something hoped for afterwards.
//
// What it does assume is that the perturbations are small enough to be linear,
// and that is checkable by halving them - which `--step` does, and which the
// report below prints.
//
// THE CHECK THAT MATTERS: the slow mode is independently measured, off 27 peaks
// of a 1200 s run, at period 16.39 s and damping ratio 0.031
// (`parapenting_pitch_axis_trace --slow-mode`). If the eigenvalues do not
// reproduce that, this instrument is wrong and nothing else it prints counts.
// That check is the first thing in the output.
//
// `--sweep` then does the thing the modes were wanted for: the same spectrum at
// twelve values of `swingDampingRatio`, across the departure boundary. The
// prediction it was built to test - that the FAST mode crosses into the right
// half plane - failed. The fast mode never crosses. The 16 s phugoid does,
// between 0.28 and 0.25, by its damping rather than its frequency. See the
// VERDICT block at the end of that output and PHYSICS_LEARNINGS section 38.
#include "CanopyGeometry.h"
#include "CoupledParagliderSolver.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <string>
#include <vector>

using namespace Parapenting::Physics;

namespace
{
constexpr double Pi = 3.14159265358979323846;

// The longitudinal state, reduced to the six that carry the pitch axis. The
// wing's position and its heading do not appear because nothing in still air
// depends on them, and a state that does not feed back is a zero eigenvalue
// that only clutters the answer.
//
// Order: surge, heave, pitch attitude, pitch rate, link swing, link rate.
constexpr int N = 6;

struct Reduced
{
    double value[N] = {0, 0, 0, 0, 0, 0};
};

double PitchOf(const Quaternion& q)
{
    // The body's forward axis, and how far above the horizon it points.
    const Vec3 forward = q.Rotate(Vec3{1.0, 0.0, 0.0});
    return std::atan2(forward.z, std::sqrt(forward.x * forward.x
                                           + forward.y * forward.y));
}

Reduced Read(const CoupledState& state)
{
    Reduced out;
    out.value[0] = state.velocityWorldMps.x;
    out.value[1] = state.velocityWorldMps.z;
    out.value[2] = PitchOf(state.attitude);
    out.value[3] = state.angularVelocityBodyRadps.y;
    // The link's lean in the vertical plane, positive with the pilot forward.
    out.value[4] = std::atan2(state.payloadDirWorld.x,
                              -state.payloadDirWorld.z);
    out.value[5] = state.linkRateWorldRadps.y;
    return out;
}

// Perturb one state by `delta`. Attitude and link direction are rotations, so
// they are perturbed by rotating rather than by adding to a component - adding
// to a quaternion or to a unit vector produces a state the solver was never
// meant to be handed, and the resulting column of the matrix would be a
// measurement of that mistake.
void Perturb(CoupledState& state, int index, double delta)
{
    switch (index)
    {
    case 0: state.velocityWorldMps.x += delta; break;
    case 1: state.velocityWorldMps.z += delta; break;
    case 2:
    {
        // NOTE THE SIGN. A positive rotation about world +Y carries the
        // forward axis (1,0,0) to (cos a, 0, -sin a), so it LOWERS the pitch
        // that `PitchOf` reads. Rotating by +delta therefore perturbs this
        // state by -delta, and the first version of this did exactly that:
        // the matrix came back with -0.96 on the attitude diagonal, an
        // eigenvalue sitting on the negative real axis, and every mode
        // aliased to a period of 2T. A sign convention disagreeing with
        // itself does not look like a bug, it looks like physics.
        const double half = -0.5 * delta;
        const Quaternion rotation{std::cos(half), 0.0, std::sin(half), 0.0};
        state.attitude = (rotation * state.attitude).Normalized();
        break;
    }
    case 3: state.angularVelocityBodyRadps.y += delta; break;
    case 4:
    {
        // Same convention, same sign, for the same reason: the link hangs
        // near (0,0,-1) and `Read` takes atan2(x, -z), which a +Y rotation
        // also runs backwards.
        const Vec3 d = state.payloadDirWorld;
        const double c = std::cos(-delta), s = std::sin(-delta);
        state.payloadDirWorld = Vec3{c * d.x + s * d.z, d.y,
                                     -s * d.x + c * d.z};
        break;
    }
    case 5: state.linkRateWorldRadps.y += delta; break;
    default: break;
    }
}

// Characteristic polynomial by Faddeev-LeVerrier, then all its roots by
// Durand-Kerner.
//
// The textbook route would be a Hessenberg reduction and a shifted QR. This is
// a 6x6 whose eigenvalues are wanted to three digits, and Faddeev-LeVerrier is
// twenty lines that cannot be subtly wrong in the way a hand-rolled QR can. Its
// weakness is conditioning, which is why the slow-mode check at the top of the
// output is not decoration: it is what says the arithmetic held.
std::vector<double> CharacteristicPolynomial(const double a[N][N])
{
    // c[0] x^n + c[1] x^(n-1) + ... + c[n], with c[0] = 1.
    std::vector<double> c(N + 1, 0.0);
    c[0] = 1.0;
    double m[N][N] = {{0}};
    for (int k = 1; k <= N; ++k)
    {
        // M <- A*M + c[k-1] I
        double next[N][N] = {{0}};
        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                double sum = 0.0;
                for (int p = 0; p < N; ++p) sum += a[i][p] * m[p][j];
                next[i][j] = sum + (i == j ? c[k - 1] : 0.0);
            }
        }
        double trace = 0.0;
        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j) m[i][j] = next[i][j];
        }
        for (int i = 0; i < N; ++i)
        {
            for (int p = 0; p < N; ++p) trace += a[i][p] * m[p][i];
        }
        c[k] = -trace / static_cast<double>(k);
    }
    return c;
}

std::vector<std::complex<double>> Roots(const std::vector<double>& c)
{
    std::vector<std::complex<double>> z(N);
    // Spread the starting guesses round a circle, which is what keeps
    // Durand-Kerner from stalling with two iterates on top of each other.
    const std::complex<double> seed(0.4, 0.9);
    std::complex<double> power(1.0, 0.0);
    for (int i = 0; i < N; ++i) { z[i] = power; power *= seed; }

    for (int iteration = 0; iteration < 2000; ++iteration)
    {
        double movement = 0.0;
        for (int i = 0; i < N; ++i)
        {
            std::complex<double> value(c[0], 0.0);
            for (int k = 1; k <= N; ++k) value = value * z[i] + c[k];
            std::complex<double> denominator(1.0, 0.0);
            for (int j = 0; j < N; ++j)
                if (j != i) denominator *= (z[i] - z[j]);
            if (std::abs(denominator) < 1.0e-300) continue;
            const std::complex<double> step = value / denominator;
            z[i] -= step;
            movement = std::max(movement, std::abs(step));
        }
        if (movement < 1.0e-14) break;
    }
    return z;
}

struct Mode
{
    double periodS = 0.0;
    double dampingRatio = 0.0;
    double timeToHalfS = 0.0;
    // The real part of the continuous eigenvalue, per second. Positive is a
    // growing mode. This is the number the sweep below is about: the damping
    // ratio is a shape, the real part is the stability.
    double growthPerS = 0.0;
    bool oscillatory = false;
};

// Everything `Report` prints, computed and returned rather than printed, so
// the sweep can ask the same question thirty times without thirty tables.
struct Spectrum
{
    std::vector<Mode> modes;
    double phi[N][N] = {{0}};
    // How far the unperturbed run moved over T, per second, in the two states
    // that say whether this is still a trim point. See `Sweep`.
    double driftSpeedMpsPerS = 0.0;
    double driftSwingRadPerS = 0.0;
    bool conventionsAgree = true;
};

Spectrum Analyse(const CoupledParagliderSolver& solver,
                 const CoupledState& settled, double transitionTimeS,
                 double scale, bool complain)
{
    Spectrum out;
    const CoupledControls hands;
    const int steps = static_cast<int>(transitionTimeS * 120.0);
    const auto advance = [&](CoupledState state)
    {
        CoupledParagliderSolver local = solver;
        for (int step = 0; step < steps; ++step)
            local.Step(state, hands, CoupledAtmosphere{});
        return Read(state);
    };

    // The unperturbed run. Subtracting it removes whatever the aircraft was
    // going to do anyway - it is not exactly at rest even after a long settle -
    // so each column is the response to its perturbation and nothing else. The
    // same control-run idea that rescued the fast-mode measurement, used here
    // where it costs one run instead of a filter.
    const Reduced base = advance(settled);

    // How far the aircraft moved on its own over T. At the ratio it was
    // settled at this is near zero by construction; at a swept ratio it is the
    // measurement that says whether the linearisation point is still a trim.
    const Reduced start = Read(settled);
    out.driftSpeedMpsPerS =
        std::fabs(base.value[0] - start.value[0]) / transitionTimeS;
    out.driftSwingRadPerS =
        std::fabs(base.value[4] - start.value[4]) / transitionTimeS;

    // Per-state perturbation sizes. Velocities in m/s, angles in rad, rates in
    // rad/s: one scale for all six would make the attitude column a thousand
    // times larger than it should be relative to the speeds.
    const double nominal[N] =
        {0.05, 0.05, 0.002, 0.002, 0.002, 0.002};

    // Does the perturbation go in the way it is read back out? Every column of
    // the matrix is a derivative with respect to a state, and if `Perturb` and
    // `Read` disagree about what that state means, the column is a
    // well-conditioned measurement of the disagreement. This is not a
    // hypothetical: the attitude and link perturbations both went in inverted,
    // which put two eigenvalues on the negative real axis and reported every
    // mode at a period of exactly twice the sampling interval. It looked like
    // aliasing, and aliasing was real but downstream - the modes were being
    // aliased because the matrix said the aircraft reverses a pitch
    // disturbance in a tenth of a second.
    //
    // So the perturbation is applied and read straight back, before any
    // stepping, and it must return +1 on its own state.
    const Reduced reference = Read(settled);
    for (int j = 0; j < N; ++j)
    {
        const double delta = nominal[j] * scale;
        CoupledState perturbed = settled;
        Perturb(perturbed, j, delta);
        const double gain = (Read(perturbed).value[j] - reference.value[j])
            / delta;
        if (std::fabs(gain - 1.0) > 0.02)
        {
            if (complain)
                std::printf("  STATE %d: perturbing by delta moves it by %+.3f "
                            "delta - Perturb and Read disagree, and every "
                            "number below is void\n", j, gain);
            out.conventionsAgree = false;
        }
    }
    if (!out.conventionsAgree && complain) std::printf("\n");

    double (&phi)[N][N] = out.phi;
    for (int j = 0; j < N; ++j)
    {
        const double delta = nominal[j] * scale;
        CoupledState perturbed = settled;
        Perturb(perturbed, j, delta);
        const Reduced moved = advance(perturbed);
        for (int i = 0; i < N; ++i)
            phi[i][j] = (moved.value[i] - base.value[i]) / delta;
    }

    const std::vector<double> polynomial = CharacteristicPolynomial(phi);
    const std::vector<std::complex<double>> discrete = Roots(polynomial);

    // Discrete eigenvalue to continuous: mu = exp(lambda T).
    std::vector<Mode>& modes = out.modes;
    std::vector<bool> used(discrete.size(), false);
    for (std::size_t i = 0; i < discrete.size(); ++i)
    {
        if (used[i]) continue;
        const std::complex<double> mu = discrete[i];
        if (std::abs(mu) < 1.0e-12) continue;
        const std::complex<double> lambda = std::log(mu) / transitionTimeS;
        Mode mode;
        mode.oscillatory = std::fabs(lambda.imag()) > 1.0e-6;
        if (mode.oscillatory)
        {
            mode.periodS = 2.0 * Pi / std::fabs(lambda.imag());
            mode.dampingRatio = -lambda.real() / std::abs(lambda);
            // Mark the conjugate as spoken for, so a pair prints once.
            for (std::size_t k = i + 1; k < discrete.size(); ++k)
            {
                if (!used[k]
                    && std::abs(std::conj(discrete[k]) - mu) < 1.0e-9)
                {
                    used[k] = true;
                    break;
                }
            }
        }
        mode.growthPerS = lambda.real();
        mode.timeToHalfS = lambda.real() < 0.0
            ? std::log(2.0) / -lambda.real() : 0.0;
        modes.push_back(mode);
    }
    std::sort(modes.begin(), modes.end(),
              [](const Mode& a, const Mode& b)
              { return a.periodS > b.periodS; });
    return out;
}

void Report(const CoupledParagliderSolver& solver, const CoupledState& settled,
            int settleSeconds, double transitionTimeS, double scale)
{
    const Spectrum spectrum = Analyse(solver, settled, transitionTimeS, scale,
                                      true);

    // The matrix itself, because the next person needs the input to the
    // arithmetic rather than its output. Read it as: column j is what a unit
    // perturbation of state j has become after T seconds.
    std::printf("  transition matrix, T = %.2f s (columns: du, dw, dtheta, "
                "dq, dswing, dswingrate)\n", transitionTimeS);
    for (int i = 0; i < N; ++i)
    {
        std::printf("   ");
        for (int j = 0; j < N; ++j) std::printf("%11.5f", spectrum.phi[i][j]);
        std::printf("\n");
    }
    std::printf("\n");

    std::printf("Settle %d s, transition time %.2f s, perturbation scale "
                "%.2fx\n\n", settleSeconds, transitionTimeS, scale);
    std::printf("%12s %12s %12s %14s\n",
                "period", "damping", "half life", "kind");
    for (const Mode& mode : spectrum.modes)
    {
        if (mode.oscillatory)
        {
            std::printf("%11.2fs %12.4f %13.1fs %14s\n",
                        mode.periodS, mode.dampingRatio,
                        mode.timeToHalfS,
                        mode.dampingRatio < 0.0 ? "GROWING" : "oscillatory");
        }
        else
        {
            std::printf("%12s %12s %13.1fs %14s\n", "-", "-",
                        mode.timeToHalfS,
                        mode.timeToHalfS > 0.0 ? "subsidence" : "DIVERGENT");
        }
    }
    std::printf("\n");
}

// ---------------------------------------------------------------------------
// The sweep: swingDampingRatio across the departure boundary.
//
// `--departure` in `pitch_axis_trace` established that below a ratio of about
// 0.3 the aircraft leaves, and that what grows is a 3.6-5.7 s mode rather than
// the 16.4 s phugoid. What it could not do is say WHEN the mode goes unstable,
// because its instrument was a departure: a yes/no read off a wing already
// outside the envelope, where the amplitude is large, the period is drifting
// and nothing is linear. `--fast-mode` then tried to measure the damping
// directly and printed NOT REPORTABLE five instruments running.
//
// An eigenvalue does not need the aircraft to depart, or even to oscillate. It
// needs a state and a solver. So the fast mode's real part can be read at every
// ratio, including ratios where the aircraft is stable and nothing is visible
// in a trace at all, and the boundary is then a sign change in a continuous
// number rather than the edge of a survival test.
//
// TWO THINGS THIS HAS TO ANSWER FOR, both of them the reason the answer could
// be worthless:
//
// 1. IT IS LINEARISED ABOUT A TRIM THAT BELONGS TO A DIFFERENT RATIO. The
//    settle costs several hundred seconds and cannot be paid per ratio; worse,
//    below the boundary there IS no settled state to linearise about, which is
//    what the departure means. So the state is settled once at 0.35 and the
//    ratio is changed underneath it. The licence for that is measured, not
//    assumed: `pitch_axis_trace` found the trim identical to three decimals
//    from 0.35 to 0.90 - the ratio buys settling speed, not a trim. The drift
//    column below is the check that it holds at each swept ratio: it is how
//    fast the unperturbed aircraft is moving at the point the derivatives were
//    taken. Small drift, the point is a trim and the eigenvalues are a
//    spectrum. Large drift, they are growth rates of a variational equation
//    about a moving state - still the right sign, no longer a mode.
//
// 2. A SPECTRUM IS NOT A TRAJECTORY. That is exactly the assumption four of
//    the five failed instruments made in the other direction. So each ratio
//    also gets flown: perturb, run 60 s, subtract a control run, and fit the
//    growth of what is left. The prediction is written before the run and it
//    can fail cleanly - if the fitted growth rate does not track the
//    eigenvalue's real part, in sign at minimum, then the linearisation does
//    not describe this aircraft near the boundary and the sweep is void.
//
// The pendulum band is 1 to 8 s: wide enough to hold both the 1.86 s the
// eigenvalues report at 0.35 and the 3.6-5.7 s the departure runs saw, which
// are not yet known to be the same mode.
constexpr double FastBandLowS = 1.0;
constexpr double FastBandHighS = 8.0;

// The most unstable oscillatory mode in the pendulum band - the one that
// decides whether the aircraft stays. Returns false if the band is empty,
// which is itself a result and is printed as one.
bool FastestGrowing(const Spectrum& spectrum, Mode& out)
{
    bool found = false;
    for (const Mode& mode : spectrum.modes)
    {
        if (!mode.oscillatory) continue;
        if (mode.periodS < FastBandLowS || mode.periodS > FastBandHighS)
            continue;
        if (!found || mode.growthPerS > out.growthPerS) { out = mode; found = true; }
    }
    return found;
}

// The largest real part in the WHOLE spectrum, and whether it belongs to an
// oscillation. Added after the first sweep, which reported only the pendulum
// band and then said "locally stable" - a claim about every mode, made from a
// filter that had discarded most of them. What the own-trim run then showed
// was an aircraft walking up in incidence from 4.9 to 9.8 degrees without
// oscillating, which is the signature of a REAL eigenvalue crossing zero, and
// a band filter looking for a period is exactly the instrument that cannot see
// one. Same failure mode as the five time-domain instruments: the answer was
// outside what the tool was shaped to return.
Mode LargestRealPart(const Spectrum& spectrum)
{
    Mode largest;
    bool first = true;
    for (const Mode& mode : spectrum.modes)
    {
        if (first || mode.growthPerS > largest.growthPerS)
        {
            largest = mode;
            first = false;
        }
    }
    return largest;
}

// The worst mode's real part, its period, and its kind, in one field: which
// mode goes unstable is the entire question and printing only the number
// answers the wrong half of it.
void PrintWorst(const Mode& worst)
{
    if (worst.oscillatory)
        std::printf(" %+8.4f %6.2fs", worst.growthPerS, worst.periodS);
    else
        std::printf(" %+8.4f %6s ", worst.growthPerS, "real");
}

struct Growth
{
    double ratePerS = 0.0;
    double periodS = 0.0;
    int extrema = 0;
    double fitQuality = 0.0;
    bool valid = false;
};

// The time-domain half of the test. Perturb pitch rate, fly, subtract the
// control run, and fit a straight line to the log of the successive extrema of
// what is left. The control run is doing the same job it did in `Analyse` -
// removing whatever the aircraft was going to do anyway - and it is the one
// zero line out of the five that never needed an assumption about the slow
// mode.
Growth MeasureGrowth(const CoupledParagliderSolver& solver,
                     const CoupledState& settled, double seconds)
{
    Growth out;
    const CoupledControls hands;
    const int ticks = static_cast<int>(seconds * 120.0);

    const auto fly = [&](CoupledState state)
    {
        CoupledParagliderSolver local = solver;
        std::vector<double> swing;
        swing.reserve(static_cast<std::size_t>(ticks));
        for (int tick = 0; tick < ticks; ++tick)
        {
            local.Step(state, hands, CoupledAtmosphere{});
            swing.push_back(Read(state).value[4]);
        }
        return swing;
    };

    CoupledState perturbed = settled;
    Perturb(perturbed, 3, 0.002);          // pitch rate, as in the matrix
    const std::vector<double> a = fly(perturbed);
    const std::vector<double> b = fly(settled);

    std::vector<double> time, logAmplitude;
    double firstExtremum = -1.0, lastExtremum = -1.0;
    for (std::size_t i = 1; i + 1 < a.size(); ++i)
    {
        const double previous = a[i - 1] - b[i - 1];
        const double here = a[i] - b[i];
        const double next = a[i + 1] - b[i + 1];
        const bool extremum = (here > previous && here >= next)
            || (here < previous && here <= next);
        if (!extremum) continue;
        const double amplitude = std::fabs(here);
        // Below this the difference is the solver's own arithmetic noise and
        // its logarithm is noise with a bias.
        if (amplitude < 1.0e-9) continue;
        const double t = static_cast<double>(i) / 120.0;
        if (firstExtremum < 0.0) firstExtremum = t;
        lastExtremum = t;
        time.push_back(t);
        logAmplitude.push_back(std::log(amplitude));
        ++out.extrema;
    }
    if (out.extrema < 6) return out;

    // Consecutive extrema are half a period apart.
    out.periodS = 2.0 * (lastExtremum - firstExtremum)
        / static_cast<double>(out.extrema - 1);
    if (out.periodS < FastBandLowS || out.periodS > FastBandHighS) return out;

    double sumT = 0.0, sumY = 0.0;
    const double n = static_cast<double>(time.size());
    for (std::size_t i = 0; i < time.size(); ++i)
    { sumT += time[i]; sumY += logAmplitude[i]; }
    const double meanT = sumT / n, meanY = sumY / n;
    double covariance = 0.0, varianceT = 0.0, varianceY = 0.0;
    for (std::size_t i = 0; i < time.size(); ++i)
    {
        const double dt = time[i] - meanT, dy = logAmplitude[i] - meanY;
        covariance += dt * dy;
        varianceT += dt * dt;
        varianceY += dy * dy;
    }
    if (varianceT <= 0.0 || varianceY <= 0.0) return out;
    out.ratePerS = covariance / varianceT;
    out.fitQuality = (covariance * covariance) / (varianceT * varianceY);
    out.valid = true;
    return out;
}

// `fly` is off for the repeat at a second transition time: the flown rate is a
// property of the aircraft, not of the sampling interval, so running it twice
// would double the cost of the sweep to re-measure a number that cannot have
// changed.
void Sweep(const CoupledParagliderSolver& solver, const CoupledState& settled,
           double transitionTimeS, bool fly)
{
    std::printf("SWEEP: the fast mode's eigenvalue against swingDampingRatio, "
                "T = %.2f s.\n\n", transitionTimeS);
    std::printf("THE PREDICTION, before the table: the registry pins the ratio "
                "at 0.35 because\n0.25 departs, so the fast mode's real part "
                "should be negative at 0.35 and\npositive at 0.25, crossing "
                "somewhere between. If it does not cross in that\ninterval, "
                "the departure is not this mode going unstable and the sweep "
                "has\nfailed, not the aircraft. The flown column is the same "
                "question asked of a\ntrajectory: it must agree in sign.\n\n");

    std::printf("%8s %9s %10s %11s %12s %11s %9s %11s\n",
                "ratio", "period", "sigma 1/s", "zeta", "flown 1/s", "R2",
                "drift", "worst mode");
    for (const double ratio : {0.90, 0.70, 0.50, 0.40, 0.35, 0.32, 0.30, 0.28,
                               0.25, 0.20, 0.15, 0.10})
    {
        CoupledParagliderSolver variant = solver;
        variant.SetSwingDampingRatio(ratio);
        const Spectrum spectrum = Analyse(variant, settled, transitionTimeS,
                                          1.0, false);
        Mode fast;
        const bool found = FastestGrowing(spectrum, fast);
        Growth flown;
        if (fly) flown = MeasureGrowth(variant, settled, 40.0);

        std::printf("%8.2f", ratio);
        if (found)
            std::printf(" %8.2fs %+10.4f %11.4f", fast.periodS,
                        fast.growthPerS, fast.dampingRatio);
        else
            std::printf(" %9s %10s %11s", "-", "no mode", "in band");
        if (flown.valid)
            std::printf(" %+11.4f %11.3f", flown.ratePerS, flown.fitQuality);
        else if (fly)
            std::printf(" %11s %11d", "unfittable", flown.extrema);
        else
            std::printf(" %11s %11s", "-", "-");
        std::printf(" %8.2e", spectrum.driftSwingRadPerS);
        const Mode worst = LargestRealPart(spectrum);
        PrintWorst(worst);
        if (worst.growthPerS > 0.0) std::printf("  UNSTABLE");
        std::printf("\n");
    }
    std::printf("\n  sigma is the eigenvalue's real part, per second: positive "
                "grows. `flown` is\n  the same rate fitted to 60 s of "
                "trajectory with a control run subtracted,\n  and R2 is that "
                "fit's quality - a low R2 means the fitted number is not a\n  "
                "single exponential and should not be compared. `drift` is how "
                "fast the\n  unperturbed link is moving at the linearisation "
                "point, in rad/s: it is\n  small only while the trim settled "
                "at 0.35 is still a trim at this ratio.\n  `worst mode` is the "
                "largest real part in the WHOLE spectrum with the kind of "
                "mode\n  it belongs to - it, and not the pendulum column, is "
                "the stability statement.\n  A dash under period "
                "means no oscillatory mode between %.0f and %.0f s -\n  the "
                "band emptying is a result, not a missing number.\n\n",
                FastBandLowS, FastBandHighS);
}

// ---------------------------------------------------------------------------
// The hole in the sweep above, made into its own measurement.
//
// Everything above is linearised about ONE state: the trim the aircraft
// settles to at ratio 0.35. That is the licence caveat 1 warned about, and the
// sweep's result makes it the live question rather than a footnote - the fast
// mode's real part does not cross zero anywhere, not even at 0.10, so from
// THAT state small disturbances decay at every ratio there is.
//
// There are exactly two ways that coexists with a wing that provably departs
// below 0.3, and they are distinguishable:
//
//   (a) the departure is a local instability of a DIFFERENT trim - each ratio
//       settles somewhere of its own, and the low-ratio trims are the unstable
//       ones. Then the sweep above asked its question at the wrong point.
//   (b) the departure is not local at all: the trims are stable to small
//       disturbances and the aircraft leaves only when something large enough
//       happens, which on a launch transient it does.
//
// So: settle each ratio from scratch, watch whether it survives the settling,
// and if it does, take the eigenvalues about ITS trim. Under (a) a low-ratio
// trim exists and has a positive real part. Under (b) either no trim is
// reached - the wing departs on the way - or one is reached and its spectrum
// is stable, and then the departure is a finite-amplitude event and no
// eigenvalue of any trim will ever predict it.
//
// This is the cheapest fork that separates them, and unlike the sweep it
// cannot come back "inconclusive": departing during the settle is as much an
// answer as settling.
struct OwnTrim
{
    CoupledState state;
    CoupledParagliderSolver solver;
    int settleSeconds = 0;
    double incidenceRad = 0.0;
    bool settled = false;
    bool departed = false;
};

OwnTrim SettleAt(const CanopyGeometry& canopy, const LinePlanSpec& linePlan,
                 double ratio, int maximumSeconds)
{
    OwnTrim out{CoupledState{}, CoupledParagliderSolver(canopy, linePlan)};
    out.solver.SetSwingDampingRatio(ratio);

    // The same criterion as `pitch_axis_trace`: ten seconds whose incidence
    // spread is under 0.01 degrees. A fixed settle would be a guess, and the
    // guesses this project made were 20, 40 and 60 s, all far too short.
    constexpr double SpreadToleranceRad = 1.7e-4;
    double low = 0.0, high = 0.0;
    int inWindow = 0;
    for (int second = 0; second < maximumSeconds && !out.departed; ++second)
    {
        for (int step = 0; step < 120; ++step)
        {
            out.solver.Step(out.state, CoupledControls{}, CoupledAtmosphere{});
            const double alpha = out.solver.Diagnostics().angleOfAttackRad;
            // Past about 20 degrees the wing is separated and what it is doing
            // is departing, not settling.
            if (alpha > 0.35) { out.departed = true; break; }
            if (inWindow == 0) { low = high = alpha; }
            low = std::min(low, alpha);
            high = std::max(high, alpha);
            ++inWindow;
        }
        out.settleSeconds = second + 1;
        if (inWindow >= 120 * 10)
        {
            if (high - low < SpreadToleranceRad) { out.settled = true; break; }
            inWindow = 0;
        }
    }
    out.incidenceRad = out.solver.Diagnostics().angleOfAttackRad;
    return out;
}

// ---------------------------------------------------------------------------
// The debt the sweep left: a trajectory behind the crossing.
//
// `--sweep` says the phugoid's damping changes sign between ratio 0.28 and
// 0.25, and its flown column - built to check exactly that - failed. It fitted
// two ratios out of twelve at R2 0.24 to 0.42 and disagreed with the eigenvalue
// eightfold. The diagnosis was the instrument, and it was specific enough to
// fix: it watched the LINK over 40 seconds, and the mode that crosses is a 16 s
// oscillation growing at 0.008/s. So it was looking at the wrong signal for a
// twentieth of the time the mode needs.
//
// Three changes, each answering one clause of that diagnosis:
//
//   * THE OBSERVABLE IS SPEED. A phugoid is an exchange between height and
//     speed at nearly constant incidence - section 34 measured this mode off
//     the path for that reason. The link barely moves in it, which is why the
//     link was the wrong place to look.
//   * THE WINDOW IS HUNDREDS OF SECONDS. At sigma 0.008/s the e-folding time is
//     125 s, so 40 s could not have shown it whatever it watched. 300 s is
//     nineteen periods and two e-foldings.
//   * THE FIRST 25 SECONDS ARE DISCARDED. The fast mode's half life is 2.2 s,
//     so by 25 s it is down by a factor of 2^11 and what remains is the slow
//     mode alone. This is the separation the five time-domain instruments of
//     section 36 could not get - and it is available here only because the
//     question is now about the SLOW mode. Waiting out the fast mode is free
//     when the fast mode is the contaminant; it was impossible when the fast
//     mode was the target. The same signal, the same overlap, and the easy
//     direction is the one nobody needed until now.
//
// Sampling at 2 Hz is deliberate and is NOT how the fast mode is excluded -
// undersampling would alias it, not remove it. Skipping 25 s removes it. 2 Hz
// is simply enough to resolve a 16 s period, at thirty-two samples a cycle.
//
// THE PREDICTION, and it can fail cleanly: the fitted rate should agree with
// the eigenvalue's real part in sign at every ratio, cross between 0.30 and
// 0.25, and the fitted PERIOD should land near 16 s and track the eigenvalue's
// period column downward as the ratio falls. If the period comes back at
// something else, this is measuring a different mode and the agreement of the
// rates would be a coincidence - so the period is the honest part of the test
// and it is printed whether or not it flatters the result.
struct SlowFit
{
    double ratePerS = 0.0;
    double periodS = 0.0;
    double fitQuality = 0.0;
    int extrema = 0;
    bool valid = false;
};

SlowFit MeasurePhugoid(const CoupledParagliderSolver& solver,
                       const CoupledState& settled, double seconds,
                       double deltaU, double skipS)
{
    SlowFit out;
    const CoupledControls hands;
    const int ticks = static_cast<int>(seconds * 120.0);

    const auto fly = [&](CoupledState state)
    {
        CoupledParagliderSolver local = solver;
        std::vector<double> speed;
        for (int tick = 0; tick < ticks; ++tick)
        {
            local.Step(state, hands, CoupledAtmosphere{});
            // 2 Hz.
            if (tick % 60 != 0) continue;
            const Vec3 v = state.velocityWorldMps;
            speed.push_back(std::sqrt(v.x * v.x + v.z * v.z));
        }
        return speed;
    };

    CoupledState perturbed = settled;
    // Ten times the linearisation step, because this has to stay above the
    // drift and the arithmetic noise for three hundred seconds rather than
    // one. Whether that is still linear is not assumed - `--phugoid` halves it
    // and prints both.
    Perturb(perturbed, 0, deltaU);
    const std::vector<double> a = fly(perturbed);
    const std::vector<double> b = fly(settled);

    std::vector<double> time, logAmplitude;
    double firstExtremum = -1.0, lastExtremum = -1.0;
    const std::size_t skip = static_cast<std::size_t>(skipS * 2.0);
    for (std::size_t i = (skip > 0 ? skip : 1); i + 1 < a.size(); ++i)
    {
        const double previous = a[i - 1] - b[i - 1];
        const double here = a[i] - b[i];
        const double next = a[i + 1] - b[i + 1];
        const bool extremum = (here > previous && here >= next)
            || (here < previous && here <= next);
        if (!extremum) continue;
        const double amplitude = std::fabs(here);
        if (amplitude < 1.0e-9) continue;
        const double t = static_cast<double>(i) / 2.0;
        if (firstExtremum < 0.0) firstExtremum = t;
        lastExtremum = t;
        time.push_back(t);
        logAmplitude.push_back(std::log(amplitude));
        ++out.extrema;
    }
    if (out.extrema < 8) return out;

    out.periodS = 2.0 * (lastExtremum - firstExtremum)
        / static_cast<double>(out.extrema - 1);

    double sumT = 0.0, sumY = 0.0;
    const double n = static_cast<double>(time.size());
    for (std::size_t i = 0; i < time.size(); ++i)
    { sumT += time[i]; sumY += logAmplitude[i]; }
    const double meanT = sumT / n, meanY = sumY / n;
    double covariance = 0.0, varianceT = 0.0, varianceY = 0.0;
    for (std::size_t i = 0; i < time.size(); ++i)
    {
        const double dt = time[i] - meanT, dy = logAmplitude[i] - meanY;
        covariance += dt * dy;
        varianceT += dt * dt;
        varianceY += dy * dy;
    }
    if (varianceT <= 0.0 || varianceY <= 0.0) return out;
    out.ratePerS = covariance / varianceT;
    out.fitQuality = (covariance * covariance) / (varianceT * varianceY);
    out.valid = true;
    return out;
}

void PhugoidCheck(const CoupledParagliderSolver& solver,
                  const CoupledState& settled, double windowS)
{
    std::printf("PHUGOID IN THE TIME DOMAIN: the trajectory the sweep's "
                "crossing does not have.\n\n");
    std::printf("THE PREDICTION, before the table: the fitted rate agrees with "
                "the eigenvalue\nin sign at every ratio and crosses between "
                "0.30 and 0.25, and the fitted\nperiod lands near 16 s and "
                "falls with the ratio as the eigenvalue's does. A\nperiod that "
                "comes back as something else means this is watching another "
                "mode\nand any agreement in the rates is luck.\n\n");
    // The zeta columns exist so the comparison that matters is on the page.
    // `pitch_axis_trace --slow-mode` measured this mode at 16.39 s and zeta
    // 0.031 off 27 peaks of a 1200 s run, by means that share no code with
    // either instrument here. Rates are what the eigenvalues speak in; zeta is
    // what that third measurement speaks in, and converting is one line.
    std::printf("%8s %11s %11s %10s %9s %9s %9s %8s %7s\n",
                "ratio", "sigma 1/s", "flown 1/s", "eig period", "flown",
                "eig zeta", "flown", "R2", "peaks");
    for (const double ratio : {0.50, 0.35, 0.30, 0.28, 0.25, 0.20})
    {
        CoupledParagliderSolver variant = solver;
        variant.SetSwingDampingRatio(ratio);
        const Spectrum spectrum = Analyse(variant, settled, 0.25, 1.0, false);
        const Mode worst = LargestRealPart(spectrum);
        const SlowFit fit = MeasurePhugoid(variant, settled, windowS, 0.5,
                                           25.0);

        std::printf("%8.2f %+11.4f", ratio, worst.growthPerS);
        if (fit.valid)
        {
            const double flownZeta =
                -fit.ratePerS * fit.periodS / (2.0 * Pi);
            std::printf(" %+11.4f %9.2fs %8.2fs %9.4f %9.4f %8.3f %7d",
                        fit.ratePerS, worst.periodS, fit.periodS,
                        worst.dampingRatio, flownZeta, fit.fitQuality,
                        fit.extrema);
        }
        else
            std::printf(" %11s %9.2fs %8s %9.4f %9s %8s %7d", "unfittable",
                        worst.periodS, "-", worst.dampingRatio, "-", "-",
                        fit.extrema);
        if (fit.valid && (fit.ratePerS > 0.0) != (worst.growthPerS > 0.0))
            std::printf("  SIGN DISAGREES");
        std::printf("\n");
    }

    // The one assumption left. 0.5 m/s is ten times the perturbation the
    // matrix is built from, and a rate that moves when it is halved is a rate
    // that belongs to the amplitude rather than to the mode.
    std::printf("\n  THE OUTSIDE CHECK: at ratio 0.35, `pitch_axis_trace "
                "--slow-mode` measured\n  this mode at 16.39 s and zeta 0.031, "
                "off 27 peaks of a 1200 s run, by means\n  sharing no code "
                "with either column above. That is the number the 0.35 row's\n"
                "  flown zeta has to match, and it is the only row that has an "
                "outside check\n  at all.\n");

    std::printf("\n  Linearity, at ratio 0.35: the same fit on a halved "
                "perturbation.\n");
    CoupledParagliderSolver reference = solver;
    reference.SetSwingDampingRatio(0.35);
    const SlowFit full = MeasurePhugoid(reference, settled, windowS, 0.5, 25.0);
    const SlowFit half = MeasurePhugoid(reference, settled, windowS, 0.25,
                                        25.0);
    std::printf("%14s %11s %9s %9s\n", "step", "rate 1/s", "period", "R2");
    for (int which = 0; which < 2; ++which)
    {
        const SlowFit& fit = which == 0 ? full : half;
        if (!fit.valid) { std::printf("%14s %11s\n",
                                      which == 0 ? "0.50 m/s" : "0.25 m/s",
                                      "unfittable"); continue; }
        std::printf("%14s %+11.4f %8.2fs %9.3f\n",
                    which == 0 ? "0.50 m/s" : "0.25 m/s",
                    fit.ratePerS, fit.periodS, fit.fitQuality);
    }

    std::printf(
        "\n  WHAT THIS SETTLES, AND WHICH INSTRUMENT LOSES.\n\n"
        "  The prediction half-failed, and the half that failed is the "
        "eigenvalue's.\n  The fitted period tracks the eigenvalue's to about "
        "1%% at every ratio whose\n  fit is clean, so both are watching the "
        "same mode and the identification is\n  not in doubt. The RATES do not "
        "agree, and at 0.35 there is an outside\n  measurement to break the "
        "tie: flown zeta 0.0299 and period 16.38 s against\n  the trace's "
        "0.031 and 16.39 s - 3%% and 0.1%% - while the eigenvalue says\n  "
        "0.0540, high by three quarters.\n\n"
        "  So the eigenvalue's slow damping is biased toward stability, which "
        "is what\n  the linearity check said before any of this ran: it was "
        "the one number that\n  moved with both T and step size, and the TODO "
        "already recorded that it\n  brackets the trace from above throughout. "
        "The flown fit, by contrast, is\n  unmoved by halving its perturbation "
        "(0.0115 against 0.0116, period identical)\n  and lands on the outside "
        "number. Where they disagree, the eigenvalue is the\n  one that is "
        "wrong, and it is wrong in the direction that matters here: it\n  "
        "reports the mode as better damped than it is, so it puts the crossing "
        "at too\n  low a ratio.\n\n"
        "  THE CROSSING MOVES. Flown, the phugoid's damping goes through zero "
        "between\n  ratio 0.35 and 0.30, not between 0.28 and 0.25. A third "
        "line agrees and it is\n  not a fit of anything: the own-trim table "
        "settles 0.35 at 410 s, fails to\n  settle 0.30 in 420 s, and departs "
        "at 0.25. A marginally GROWING phugoid is\n  exactly why 0.30 has no "
        "settled trim to find.\n\n"
        "  That explains the tuned coefficient rather than merely bounding it. "
        "0.35 is\n  not a safety margin above a departure - it is approximately "
        "the smallest\n  value at which this wing's phugoid still damps at all, "
        "and the registry's\n  Tuned 0.35 sits on the edge of that.\n\n"
        "  TWO ROWS ARE NOT EVIDENCE AND ARE NOT USED. At 0.20 the fit returns "
        "a 7.87 s\n  period - half the phugoid - at R2 0.375 off 69 extrema, "
        "which is a signal\n  that has stopped being one growing oscillation "
        "within the window; at 0.25\n  R2 is 0.495 and the period misses by "
        "12%%. Both are ratios where the motion\n  leaves small-amplitude "
        "behaviour inside 300 s, which is the regime a fit like\n  this has no "
        "claim on. The crossing above rests on the rows from 0.50 to 0.28,\n  "
        "where R2 is 0.89 to 1.000 and the period matches - and those rows "
        "contain it.\n\n"
        "  WHAT IT MEANS FOR THE ITEM. The remaining goal is a damping ratio "
        "derived from\n  pilot and line drag, about 0.06, instead of one chosen "
        "to keep the aircraft\n  flying. This says that cannot be reached by "
        "finding a little more link\n  damping: at 0.06 the phugoid is well "
        "past its sign change. The missing\n  stabilising mechanism has to act "
        "on SPEED stability - the phugoid's own\n  restoring term, the flat "
        "lift curve of section 34 - and not on the link.\n\n");
}

// ---------------------------------------------------------------------------
// Section 34's damping formula has two inputs. Section 35 only ever tested one.
//
// The best explanation this project has for the slow mode is section 34's pair,
// from linearising the two-state phugoid with L ~ V^n and D ~ V^d:
//
//     omega = g sqrt(n) / V              zeta = (d/2) / ((L/D) sqrt(n))
//
// It earned its keep: n = 0.171 predicts the 16.4 s period from a trajectory
// that did not know the period, and n with d predicts zeta 0.034 against 0.031
// measured.
//
// Section 35 then spent its one prediction asking whether **n** crosses zero at
// the departure, found it holding at 0.14 to 0.19, and concluded the phugoid
// does not reach it. Section 38 showed why that inference was too wide: n is
// the FREQUENCY term, and the phugoid arrives by its damping. But there is a
// second, sharper point that neither section made, and it is visible in the
// formula itself:
//
//   ZETA AS WRITTEN CANNOT BE NEGATIVE. With n > 0, d > 0 and a positive glide
//   ratio, `(d/2)/((L/D) sqrt(n))` is positive, full stop. The measured damping
//   crosses zero between ratio 0.35 and 0.30. So either d goes to zero and
//   through it, or SECTION 34'S MODEL CANNOT PRODUCE THE INSTABILITY AT ALL.
//
// That is a clean fork, and d has never been measured against the ratio - only
// n has. Testing the other input of the same formula is exactly the move
// section 38 says was skipped last time.
//
// THE PREDICTION, before the run:
//
//   * if the exponent model reaches the departure, d crosses zero between 0.35
//     and 0.30, where the flown damping does, and zeta predicted from measured
//     n, d and L/D tracks the flown zeta across the whole sweep;
//   * if d stays comfortably positive there, the two-state model does not
//     contain this instability. It would then be explaining the mode's period
//     and its damping at ONE operating point while being structurally unable to
//     explain the mode's stability - and the mechanism would have to be
//     something the two-state theory does not carry, with the coupling to the
//     link that the six-state eigenproblem does carry as the obvious suspect.
//
// Both outcomes are worth the run, which is the test being any good.
//
// The lift and drag are read off the PATH, not off the solver's force
// bookkeeping - `L = m(g cos gamma + V gammaDot)`, `D = -m(g sin gamma + Vdot)`
// - for section 34's reason: the aerodynamic loads are what is under suspicion,
// so a check that reads them proves less. Sampled at 1 Hz to match the central
// difference's gain correction exactly, on the same excitation and the same
// 25 s skip as the damping fit above, so the exponents and the zeta they are
// being compared against come off the same motion.
struct PathExponents
{
    double lift = 0.0;          // n in L ~ V^n
    double drag = 0.0;          // d in D ~ V^d
    double glideRatio = 0.0;
    double meanSpeedMps = 0.0;
    int samples = 0;
    bool valid = false;
};

PathExponents MeasurePathExponents(const CoupledParagliderSolver& solver,
                                   const CoupledState& settled,
                                   double seconds, double deltaU,
                                   double skipS, double periodS)
{
    PathExponents out;
    const CoupledControls hands;
    const int ticks = static_cast<int>(seconds * 120.0);
    const double massKg = solver.AllUpMassKg();

    CoupledParagliderSolver local = solver;
    CoupledState state = settled;
    Perturb(state, 0, deltaU);

    std::vector<double> speed, gamma;
    for (int tick = 0; tick < ticks; ++tick)
    {
        local.Step(state, hands, CoupledAtmosphere{});
        if (tick % 120 != 0) continue;              // 1 Hz
        const Vec3 v = state.velocityWorldMps;
        const double horizontal = std::sqrt(v.x * v.x + v.y * v.y);
        speed.push_back(std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));
        // Negative in a glide, which is the convention the drag term wants.
        gamma.push_back(std::atan2(v.z, horizontal));
    }

    // A central difference on 1 s samples underestimates a sinusoid's rate by
    // sin(wh)/(wh). 2.5% at this period, known exactly, and left uncorrected it
    // would bias n low - which is the direction that flatters the hypothesis.
    const double omega = 2.0 * Pi / periodS;
    const double differenceGain = std::sin(omega) / omega;

    const std::size_t from = static_cast<std::size_t>(skipS);
    double sumLnV = 0.0, sumLnL = 0.0, sumLnVLnV = 0.0, sumLnVLnL = 0.0;
    double sumLnD = 0.0, sumLnVLnD = 0.0;
    double sumLift = 0.0, sumDrag = 0.0, sumV = 0.0;
    int count = 0;
    for (std::size_t i = from + 1; i + 1 < speed.size(); ++i)
    {
        const double gammaDot =
            (gamma[i + 1] - gamma[i - 1]) / (2.0 * differenceGain);
        const double speedDot =
            (speed[i + 1] - speed[i - 1]) / (2.0 * differenceGain);
        const double lift =
            massKg * (9.80665 * std::cos(gamma[i]) + speed[i] * gammaDot);
        const double drag =
            -massKg * (9.80665 * std::sin(gamma[i]) + speedDot);
        if (lift <= 0.0 || drag <= 0.0) continue;
        const double lnV = std::log(speed[i]);
        sumLnV += lnV; sumLnL += std::log(lift); sumLnD += std::log(drag);
        sumLnVLnV += lnV * lnV;
        sumLnVLnL += lnV * std::log(lift);
        sumLnVLnD += lnV * std::log(drag);
        sumLift += lift; sumDrag += drag; sumV += speed[i];
        ++count;
    }
    if (count < 8) return out;
    const double denominator = count * sumLnVLnV - sumLnV * sumLnV;
    if (std::fabs(denominator) < 1.0e-12) return out;
    out.lift = (count * sumLnVLnL - sumLnV * sumLnL) / denominator;
    out.drag = (count * sumLnVLnD - sumLnV * sumLnD) / denominator;
    out.glideRatio = sumLift / sumDrag;
    out.meanSpeedMps = sumV / count;
    out.samples = count;
    out.valid = true;
    return out;
}

void ExponentCheck(const CoupledParagliderSolver& solver,
                   const CoupledState& settled, double windowS)
{
    std::printf("THE OTHER INPUT: does the DRAG exponent cross zero?\n\n");
    std::printf("Section 34's zeta = (d/2)/((L/D) sqrt(n)) is POSITIVE for any "
                "n > 0, d > 0.\nThe flown damping crosses zero between ratio "
                "0.35 and 0.30. So either d\ncrosses with it, or that model "
                "cannot contain this instability at all.\nSection 35 tested n "
                "and never tested d. This tests d.\n\n");
    std::printf("%8s %8s %8s %9s %11s %11s %11s\n",
                "ratio", "n", "d", "L/D", "zeta pred", "zeta flown",
                "period pred");
    for (const double ratio : {0.50, 0.35, 0.30, 0.28})
    {
        CoupledParagliderSolver variant = solver;
        variant.SetSwingDampingRatio(ratio);
        const SlowFit fit = MeasurePhugoid(variant, settled, windowS, 0.5,
                                           25.0);
        if (!fit.valid) { std::printf("%8.2f  no damping fit\n", ratio);
                          continue; }
        const PathExponents e = MeasurePathExponents(variant, settled, windowS,
                                                     0.5, 25.0, fit.periodS);
        if (!e.valid) { std::printf("%8.2f  no exponent fit\n", ratio);
                        continue; }
        const double zetaPredicted = e.lift > 0.0
            ? (e.drag / 2.0) / (e.glideRatio * std::sqrt(e.lift)) : 0.0;
        const double periodPredicted = e.lift > 0.0
            ? 2.0 * Pi * e.meanSpeedMps / (9.80665 * std::sqrt(e.lift)) : 0.0;
        const double zetaFlown = -fit.ratePerS * fit.periodS / (2.0 * Pi);
        std::printf("%8.2f %8.3f %8.3f %9.2f %11.4f %11.4f %10.2fs\n",
                    ratio, e.lift, e.drag, e.glideRatio, zetaPredicted,
                    zetaFlown, periodPredicted);
    }
    std::printf("\n  `zeta pred` is section 34's formula fed the measured n, d "
                "and L/D from the\n  same motion the flown zeta was fitted to. "
                "`period pred` is the same for\n  the frequency, and it is the "
                "control: section 34's period claim is the part\n  that has "
                "already been verified, so a period prediction that lands "
                "while the\n  damping prediction misses tells us the "
                "measurement is sound and the damping\n  half of the model is "
                "what fails.\n\n");

    std::printf(
        "  THE ANSWER IS THE SECOND BRANCH, and the control is what makes it "
        "safe to say.\n\n"
        "  The period prediction lands at every ratio - 18.07 against 18.28 "
        "flown, 16.44\n  against 16.38, 16.02 against 15.82, 16.52 against "
        "15.88, so 1 to 4%%. Section\n  34's frequency claim reproduces across "
        "the whole sweep, off the same fit, so\n  the exponent measurement is "
        "sound and n is doing real work.\n\n"
        "  The damping prediction does not merely miss. **d never approaches "
        "zero - it\n  RISES, 0.281 to 0.459, as the ratio falls** - so the "
        "predicted zeta rises too,\n  0.0341 to 0.0510, over exactly the "
        "interval where the flown zeta falls\n  through zero, 0.1598 to "
        "-0.0167. The prediction and the measurement move in\n  OPPOSITE "
        "directions across the boundary.\n\n"
        "  So section 34's two-state model cannot contain this instability, "
        "and not by a\n  little: `zeta = (d/2)/((L/D) sqrt(n))` is positive "
        "whenever n and d are, and\n  both stay firmly positive. At the single "
        "operating point where it was\n  validated it is still right - 0.0363 "
        "predicted against 0.0299 flown at 0.35,\n  which is section 34's 0.034 "
        "against 0.031. It is right at a POINT and\n  anti-correlated as a "
        "FUNCTION of the parameter. Those are very different\n  kinds of "
        "correct, and only the second one was ever needed here.\n\n"
        "  WHAT THAT LEAVES. The phugoid's damping on this wing is not set by "
        "its drag\n  exponent. Across the sweep n and d move 25%% and 60%% "
        "while the flown damping\n  moves by 0.18 and changes sign - almost "
        "none of that dependence is in the\n  two-state theory. What the ratio "
        "actually changes is the LINK, and the link\n  is the state the "
        "two-state phugoid does not have. The destabilisation is a\n  "
        "coupling between the pendulum and the phugoid, which is why the "
        "six-state\n  eigenproblem sees a sign change at all and the two-state "
        "formula cannot.\n\n"
        "  THIS RETRACTS THE PREVIOUS ITERATION'S RECOMMENDATION. Section 39 "
        "concluded\n  that the missing stabilising mechanism 'has to act on "
        "speed stability, not on\n  the link'. That inference was drawn FROM "
        "section 34's damping formula - the\n  half of the model this run just "
        "showed is anti-correlated with the truth over\n  the parameter in "
        "question. The conclusion inherited the error of its premise.\n  The "
        "quantity to go after is the pendulum-phugoid coupling itself.\n\n");
}

// ---------------------------------------------------------------------------
// The eigenVECTORS, which have been sitting in the transition matrix all along.
//
// Section 40 left the mechanism in one place: the ratio's effect on the
// phugoid's damping is essentially absent from the two-state theory, and the
// link is the state that theory does not have. That makes the pendulum-phugoid
// coupling the suspect, and the coupling is a property of the MODE SHAPE, which
// costs no new runs - the same matrix whose eigenvalues gave the periods gives
// the eigenvectors, and a discrete-time eigenvector is the continuous one
// unchanged, so there is nothing to convert.
//
// AMPLITUDE IS NOT THE MEASUREMENT. A pilot hanging under a wing that is
// accelerating along its path will lean whether or not anything interesting is
// happening - that is the pendulum tracking apparent gravity, which sections 34
// and 35 already measured as dA/dV. A nonzero link component in the phugoid is
// therefore expected and proves nothing.
//
// PHASE is the measurement. The solver damps the link against the WORLD, and
// its own comment calls the resulting tracking lag "a cost paid knowingly". A
// lag inside a feedback loop is the textbook way to turn a restoring term into
// a driving one: what decides whether the link's motion removes energy from the
// phugoid or feeds it is WHEN the lean happens relative to the speed
// oscillation, not how big it is. So the number below is the phase of the link
// swing relative to surge in the 16 s mode, and the prediction is that it moves
// systematically with the ratio and that the damping follows it.
//
// THE CONTROL, and it can fail: the 1.86 s mode is the pendulum and the 16 s
// mode is the phugoid. If this code is right, the fast mode must come back
// LINK-dominated and the slow mode SPEED-dominated. If it does not, the
// eigenvectors are wrong and the phase column means nothing. That check is
// printed first and costs nothing.
//
// The two states are compared after scaling, because they have different units
// and an unscaled comparison would be a statement about metres and radians
// rather than about the aircraft: surge is divided by the trim speed, giving a
// fractional speed change, and the link angle is left in radians. So the
// amplitude column reads "radians of link lean per unit fractional speed
// change". The scaling is stated because the number depends on it.
std::vector<std::complex<double>> Eigenvector(const double a[N][N],
                                              std::complex<double> mu)
{
    // Inverse iteration. (A - mu I) is singular by construction, which is the
    // point: a solve against it amplifies the eigendirection enormously. The
    // shift keeps the arithmetic finite without moving the answer.
    const std::complex<double> shift(1.0e-9, 1.0e-9);
    std::vector<std::complex<double>> v(N, std::complex<double>(1.0, 0.0));
    for (int iteration = 0; iteration < 4; ++iteration)
    {
        std::complex<double> m[N][N + 1];
        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
                m[i][j] = std::complex<double>(a[i][j], 0.0)
                    - ((i == j) ? mu + shift : std::complex<double>(0.0, 0.0));
            m[i][N] = v[i];
        }
        // Gaussian elimination, partial pivoting on magnitude.
        for (int column = 0; column < N; ++column)
        {
            int pivot = column;
            for (int row = column + 1; row < N; ++row)
                if (std::abs(m[row][column]) > std::abs(m[pivot][column]))
                    pivot = row;
            if (std::abs(m[pivot][column]) < 1.0e-300) continue;
            if (pivot != column)
                for (int j = column; j <= N; ++j)
                    std::swap(m[column][j], m[pivot][j]);
            for (int row = column + 1; row < N; ++row)
            {
                const std::complex<double> factor =
                    m[row][column] / m[column][column];
                for (int j = column; j <= N; ++j)
                    m[row][j] -= factor * m[column][j];
            }
        }
        for (int row = N - 1; row >= 0; --row)
        {
            std::complex<double> sum = m[row][N];
            for (int j = row + 1; j < N; ++j) sum -= m[row][j] * v[j];
            v[row] = std::abs(m[row][row]) > 1.0e-300
                ? sum / m[row][row] : std::complex<double>(0.0, 0.0);
        }
        double largest = 0.0;
        for (int i = 0; i < N; ++i) largest = std::max(largest, std::abs(v[i]));
        if (largest <= 0.0) break;
        for (int i = 0; i < N; ++i) v[i] /= largest;
    }
    return v;
}

struct Shape
{
    double linkPerSpeed = 0.0;     // rad of lean per unit fractional speed
    double phaseDeg = 0.0;         // link swing relative to surge
    // How much the link ARTICULATES against the wing, rather than riding with
    // it: |swing - attitude| over |attitude|, both in radians, so there is no
    // scaling choice in it at all. Added after the first control failed - see
    // the note in `ShapeCheck`. This is what "the pendulum mode" means
    // physically, and unlike link/speed it cannot be argued about by picking a
    // different normalisation.
    double articulation = 0.0;
    // ||(Phi - mu I) v|| / ||v||. The arithmetic's own answer for whether the
    // vector really is an eigenvector, independent of any physical
    // expectation. This is the check that can tell a broken solve from a
    // wrong prediction, which the control on its own could not.
    double residual = 0.0;
    double periodS = 0.0;
    double growthPerS = 0.0;
    bool valid = false;
};

// ---------------------------------------------------------------------------
// Which entries of the matrix actually carry the destabilisation.
//
// Section 41 owed an energy integral: the work the link term does on the
// phugoid over a cycle, sign as the answer. There is a version of that question
// which is exact rather than modelled, and it needs no new concept of energy at
// all - only the left eigenvector, which is the standard tool for exactly this
// and which the matrix already contains.
//
// For a simple eigenvalue with right eigenvector v and left eigenvector w
// normalised so that w^H v = 1,
//
//     d(mu) / d(Phi_ij) = conj(w_i) v_j
//
// so a KNOWN change in the matrix maps to a first-order change in the
// eigenvalue, entry by entry, and the mapping is additive. The ratio changes
// the matrix in a way that is measurable - just difference Phi at two ratios -
// so the growth rate's movement can be split across blocks of the matrix and
// each block's share read off with a sign:
//
//     delta sigma from block B = Re( sum_{ij in B} conj(w_i) v_j dPhi_ij / mu )
//                                / T
//
// The four blocks are the whole question. Rows and columns 0-3 are the wing's
// own states, 4-5 the link's:
//
//     wing        rows 0-3, cols 0-3    the wing on its own
//     link->wing  rows 0-3, cols 4-5    the link driving the wing
//     wing->link  rows 4-5, cols 0-3    the wing driving the link
//     link        rows 4-5, cols 4-5    the link on its own
//
// THE CHECK IS BUILT IN AND CANNOT BE FUDGED: the four contributions must add
// up to the actual measured change in sigma between the two ratios. If they do
// not, the step is too large for a first-order expansion and the split means
// nothing - so the table prints the predicted total beside the measured one,
// and a step that fails is a step that gets reported as failed rather than
// quietly narrowed until it agrees.
//
// THE PREDICTION: if section 41's surviving gain story is right, the movement
// in sigma is carried by the blocks that involve the link - and the coupling
// blocks specifically, not the link's own 2x2, because a change confined to
// the link block alone would be the pendulum getting less damped rather than
// the pendulum destabilising the phugoid. If instead the wing block carries it,
// the ratio is changing the wing's own dynamics and every coupling story from
// section 40 onward is wrong.
// NOTE THE CONJUGATE, which the first version of this got wrong. The left
// eigenvector satisfies `w^H Phi = mu w^H`; conjugate-transposing that, and
// using that Phi is real, gives `Phi^T w = conj(mu) w`. So w is an eigenvector
// of the TRANSPOSE for the CONJUGATE eigenvalue, and passing mu instead
// returns the vector belonging to the conjugate mode - which is very nearly
// orthogonal to v, so the normalisation w^H v = 1 divides by almost nothing and
// every share comes out around 1e13.
//
// That is what happened, and the built-in check is the only reason it was
// caught in one run: the four shares are supposed to add up to a measured
// change in sigma of about 0.013, and they added up to 2.4e13. A decomposition
// with no total to check against would have been read as a result. Same lesson
// as section 37's inverted perturbation - a convention disagreeing with itself
// produces confident nonsense - and the same fix, which is to have something
// the arithmetic must reproduce.
std::vector<std::complex<double>> LeftEigenvector(const double a[N][N],
                                                  std::complex<double> mu)
{
    double transpose[N][N];
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) transpose[i][j] = a[j][i];
    return Eigenvector(transpose, std::conj(mu));
}

struct Split
{
    double wing = 0.0;
    double linkToWing = 0.0;
    double wingToLink = 0.0;
    double link = 0.0;
    double predictedTotal = 0.0;
    double measuredTotal = 0.0;
    // |w^H v| before normalisation, with both vectors scaled to unit largest
    // component. This is the eigenvalue's conditioning, and it is printed
    // because it is the number that was silently near zero when the left
    // eigenvector was conjugated wrongly. Near 1 is a well separated mode;
    // near 0 means the normalisation is dividing by nothing and every share
    // below is meaningless.
    double conditioning = 0.0;
    // The share of the LEFT eigenvector sitting on the link's two rows.
    //
    // This is the part of the split that is not near-tautological. That the
    // movement enters through rows 4-5 is barely a finding: `swingDampingRatio`
    // appears in the link's own update equation and nowhere else, so those are
    // the only rows whose entries change at all. What is NOT automatic is that
    // changing them moves the PHUGOID's eigenvalue - that requires the
    // phugoid's adjoint to have weight on the link rows, which is exactly what
    // "the phugoid is receptive to what happens to the link" means, and it is
    // measurable rather than assumed.
    double adjointLinkShare = 0.0;
    bool valid = false;
};

Split SplitGrowth(const double from[N][N], const double to[N][N],
                  double transitionTimeS, double lowPeriodS,
                  double highPeriodS)
{
    Split out;
    const std::vector<std::complex<double>> discrete =
        Roots(CharacteristicPolynomial(from));
    for (const std::complex<double>& mu : discrete)
    {
        if (std::abs(mu) < 1.0e-12) continue;
        const std::complex<double> lambda = std::log(mu) / transitionTimeS;
        if (lambda.imag() <= 1.0e-6) continue;
        const double period = 2.0 * Pi / lambda.imag();
        if (period < lowPeriodS || period > highPeriodS) continue;

        const std::vector<std::complex<double>> v = Eigenvector(from, mu);
        std::vector<std::complex<double>> w = LeftEigenvector(from, mu);
        std::complex<double> scale(0.0, 0.0);
        for (int i = 0; i < N; ++i) scale += std::conj(w[i]) * v[i];
        double linkWeight = 0.0, totalWeight = 0.0;
        for (int i = 0; i < N; ++i)
        {
            const double weight = std::norm(w[i]);
            totalWeight += weight;
            if (i >= 4) linkWeight += weight;
        }
        out.adjointLinkShare = totalWeight > 0.0 ? linkWeight / totalWeight
                                                 : 0.0;
        out.conditioning = std::abs(scale);
        if (std::abs(scale) < 1.0e-12) return out;
        for (int i = 0; i < N; ++i) w[i] /= std::conj(scale);

        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                const double delta = to[i][j] - from[i][j];
                const std::complex<double> term =
                    std::conj(w[i]) * v[j] * delta / mu;
                const double share = term.real() / transitionTimeS;
                if (i < 4 && j < 4) out.wing += share;
                else if (i < 4) out.linkToWing += share;
                else if (j < 4) out.wingToLink += share;
                else out.link += share;
            }
        }
        out.predictedTotal = out.wing + out.linkToWing + out.wingToLink
            + out.link;

        // The measured movement, from the eigenvalues of the two matrices.
        const std::vector<std::complex<double>> after =
            Roots(CharacteristicPolynomial(to));
        for (const std::complex<double>& nu : after)
        {
            if (std::abs(nu) < 1.0e-12) continue;
            const std::complex<double> other =
                std::log(nu) / transitionTimeS;
            if (other.imag() <= 1.0e-6) continue;
            const double otherPeriod = 2.0 * Pi / other.imag();
            if (otherPeriod < lowPeriodS || otherPeriod > highPeriodS) continue;
            out.measuredTotal = other.real() - lambda.real();
            out.valid = true;
            break;
        }
        return out;
    }
    return out;
}

// ---------------------------------------------------------------------------
// Is the receptivity fixed, and is 0.985 even a real number?
//
// Section 42 reported that 98.5% of the phugoid's adjoint sits on the link's
// rows and called that the mechanism. Two things about that claim need testing
// before anything is built on it, and the first is an audit of section 42
// itself.
//
// THE AUDIT. A left eigenvector's components carry units, dual to the states'.
// Summing |w_4|^2 + |w_5|^2 against the rest therefore adds radians to metres
// per second, which is exactly the trap section 41 fell into with `link/speed`
// - and section 41's own lesson was that a comparison depending on a scaling
// needs a scaling-free control beside it. Section 42 did not apply its
// predecessor's lesson to itself. So: rescale the states to reference
// magnitudes, recompute, and print both. If the share moves a lot, section 42's
// headline needs qualifying and this says so.
//
// The rescaling is a similarity transform, `D^-1 Phi D` with D = diag(s), so
// the EIGENVALUES are untouched - it is a change of units and not of physics.
// Under it the right eigenvector goes to `D^-1 v` and the left to `D w`, from
// which one thing follows immediately and is worth stating: the conditioning
// `w^H v` is INVARIANT, because `(Dw)^H (D^-1 v) = w^H v`. So section 42's
// cond = 0.10, and the non-normality it reports, does not depend on any of
// this. The 0.985 does.
//
// The scales: speeds by the trim speed, angles left in radians, rates by the
// mode's own frequency - which is the natural choice, since it makes a rate of
// one unit the rate a unit angle actually reaches in this mode.
//
// THE QUESTION. If the ratio changes only the GAIN - section 41's surviving
// story - the receptivity should sit still while the growth rate crosses zero.
// If the receptivity moves with the ratio instead, then the coefficient is
// changing what the mode listens to, and "make the mechanism enter where the
// mode is receptive" is not a fixed target to aim at.
//
// This matters for what comes after rather than for the record: a stabilising
// mechanism that would permit the ~0.06 the pilot and line drag imply has to
// enter where this mode listens, so whether that place holds still is the
// difference between a design requirement and a moving one.
struct Receptivity
{
    double shareRaw = 0.0;
    double shareScaled = 0.0;
    double conditioning = 0.0;
    double growthPerS = 0.0;
    double periodS = 0.0;
    bool valid = false;
};

Receptivity MeasureReceptivity(const double phi[N][N], double transitionTimeS,
                               double trimSpeedMps, double lowPeriodS,
                               double highPeriodS)
{
    Receptivity out;
    const std::vector<std::complex<double>> discrete =
        Roots(CharacteristicPolynomial(phi));
    for (const std::complex<double>& mu : discrete)
    {
        if (std::abs(mu) < 1.0e-12) continue;
        const std::complex<double> lambda = std::log(mu) / transitionTimeS;
        if (lambda.imag() <= 1.0e-6) continue;
        const double period = 2.0 * Pi / lambda.imag();
        if (period < lowPeriodS || period > highPeriodS) continue;

        const std::vector<std::complex<double>> v = Eigenvector(phi, mu);
        const std::vector<std::complex<double>> w = LeftEigenvector(phi, mu);

        std::complex<double> scale(0.0, 0.0);
        for (int i = 0; i < N; ++i) scale += std::conj(w[i]) * v[i];
        out.conditioning = std::abs(scale);

        const double omega = 2.0 * Pi / period;
        const double s[N] = {trimSpeedMps, trimSpeedMps, 1.0, omega, 1.0,
                             omega};

        double linkRaw = 0.0, totalRaw = 0.0;
        double linkScaled = 0.0, totalScaled = 0.0;
        for (int i = 0; i < N; ++i)
        {
            const double raw = std::norm(w[i]);
            // Left eigenvector under D^-1 Phi D is D w.
            const double scaled = std::norm(w[i] * s[i]);
            totalRaw += raw;
            totalScaled += scaled;
            if (i >= 4) { linkRaw += raw; linkScaled += scaled; }
        }
        out.shareRaw = totalRaw > 0.0 ? linkRaw / totalRaw : 0.0;
        out.shareScaled = totalScaled > 0.0 ? linkScaled / totalScaled : 0.0;
        out.growthPerS = lambda.real();
        out.periodS = period;
        out.valid = true;
        return out;
    }
    return out;
}

// ---------------------------------------------------------------------------
// Where would a stabilising mechanism have to enter, and how much would it buy?
//
// Item 11's remaining goal is a swing damping ratio derived from pilot and line
// drag - about 0.06 - instead of one chosen to keep the aircraft flying. Every
// level so far has narrowed WHY 0.35 is needed. This asks the design question
// directly: which entries of the link's rows is the phugoid's stability
// actually sensitive to, and what would changing them be worth.
//
// Reading the solver's link update makes the question sharper than it looks.
// The whole of `swingDampingRatio` enters at exactly one place -
//
//     linkRate = (linkRate + linkAngularAccel * dt) / (1 + 2 zeta omega dt)
//
// - a single scalar gain on the whole increment. Which means it attenuates
// `linkAngularAccel` too, and that term carries the WING's acceleration. So the
// coefficient is not only a damper on the link: it is simultaneously a gain on
// the wing-to-link coupling, and those cannot be separated by moving the
// coefficient. That is why section 42's split found the movement spread across
// the link's own block AND wing->link rather than confined to the damper.
//
// Separating them needs a perturbation the coefficient cannot make, and the
// matrix allows it where the solver does not: change one entry of Phi and ask
// what sigma does. `dsigma/dPhi_ij = Re(conj(w_i) v_j / mu) / T` gives it in
// closed form for every entry at once.
//
// TWO HONESTY CONSTRAINTS, both of them lessons already paid for here.
//
//   * SCALING (section 43). Entries of Phi have mixed units, so a map of raw
//     sensitivities compares metres per second with radians. It is reported per
//     unit RELATIVE change - sensitivity times s_i/s_j - which is dimensionless
//     and is what "a 1% change to this entry" means. The scales are the same
//     ones section 43 used and are stated there.
//   * A SENSITIVITY IS NOT A MECHANISM (sections 40 and 41). A large entry says
//     the mode would respond if that entry changed; it does NOT say any
//     physical device can change that entry, still less that one exists. The
//     map is a requirement to check candidates against, not a design.
//
// THE CHECK: take the largest entries, apply a finite change to Phi, and
// compare the predicted move in sigma against the eigenvalues of the changed
// matrix. Same self-check as section 42's split, and it is what separates a
// derivative that means something from one that is merely computed.
void DesignCheck(const CoupledParagliderSolver& solver,
                 const CoupledState& settled, double transitionTimeS,
                 double ratio)
{
    const Vec3 v0 = settled.velocityWorldMps;
    const double trimSpeed = std::sqrt(v0.x * v0.x + v0.z * v0.z);

    CoupledParagliderSolver variant = solver;
    variant.SetSwingDampingRatio(ratio);
    const Spectrum spectrum = Analyse(variant, settled, transitionTimeS, 1.0,
                                      false);

    const std::vector<std::complex<double>> discrete =
        Roots(CharacteristicPolynomial(spectrum.phi));
    std::complex<double> mu(0.0, 0.0);
    double period = 0.0, growth = 0.0;
    bool found = false;
    for (const std::complex<double>& candidate : discrete)
    {
        if (std::abs(candidate) < 1.0e-12) continue;
        const std::complex<double> lambda =
            std::log(candidate) / transitionTimeS;
        if (lambda.imag() <= 1.0e-6) continue;
        const double p = 2.0 * Pi / lambda.imag();
        if (p < 10.0 || p > 40.0) continue;
        mu = candidate; period = p; growth = lambda.real(); found = true;
        break;
    }
    if (!found) { std::printf("DESIGN: no 16 s mode at ratio %.2f\n", ratio);
                  return; }

    const std::vector<std::complex<double>> v = Eigenvector(spectrum.phi, mu);
    std::vector<std::complex<double>> w = LeftEigenvector(spectrum.phi, mu);
    std::complex<double> scale(0.0, 0.0);
    for (int i = 0; i < N; ++i) scale += std::conj(w[i]) * v[i];
    if (std::abs(scale) < 1.0e-12) { std::printf("DESIGN: ill conditioned\n");
                                     return; }
    for (int i = 0; i < N; ++i) w[i] /= std::conj(scale);

    const double omega = 2.0 * Pi / period;
    const double s[N] = {trimSpeed, trimSpeed, 1.0, omega, 1.0, omega};
    static const char* names[N] =
        {"surge", "heave", "attitude", "pitchrate", "swing", "swingrate"};

    std::printf("DESIGN: what the phugoid's stability is sensitive to, in the "
                "link's rows.\n\n");
    std::printf("Ratio %.2f, sigma %+.4f /s, period %.2f s. The whole of "
                "`swingDampingRatio`\nenters as ONE scalar gain on the link's "
                "rate increment, which also attenuates\nthe wing's "
                "acceleration feeding the link - so the coefficient cannot "
                "separate\nthe damper from the coupling gain. Changing single "
                "entries of the matrix\ncan.\n\n", ratio, growth, period);
    std::printf("Per unit RELATIVE change (section 43's scaling; a sensitivity "
                "is not a\nmechanism - see the note in the source).\n\n");
    // ALL THIRTY-SIX, not just the link's twelve.
    //
    // Section 44 looked only at the link's rows, found them fiercely
    // sensitive, and concluded the boundary's location is a property of how the
    // LINK is written. That conclusion needs the rest of the matrix to mean
    // anything: if the wing's rows are just as sensitive, then fragility is a
    // property of this mode - a non-normal eigenvalue moves far for a small
    // change anywhere it is receptive - and singling out the link's formulation
    // is looking where the light is. The comparison costs nothing, and it is
    // the control section 44 should have carried.
    std::printf("%22s %16s\n", "entry", "dsigma per unit");
    struct Entry { int i, j; double sensitivity; };
    std::vector<Entry> entries;
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            const std::complex<double> term = std::conj(w[i]) * v[j] / mu;
            const double raw = term.real() / transitionTimeS;
            entries.push_back({i, j, raw * s[i] / s[j]});
        }
    }
    std::sort(entries.begin(), entries.end(),
              [](const Entry& a, const Entry& b)
              { return std::fabs(a.sensitivity) > std::fabs(b.sensitivity); });
    for (std::size_t k = 0; k < entries.size() && k < 10; ++k)
    {
        const Entry& e = entries[k];
        char label[48];
        std::snprintf(label, sizeof label, "d(%s)/d(%s)", names[e.i],
                      names[e.j]);
        std::printf("%22s %+16.5f   %s\n", label, e.sensitivity,
                    e.i >= 4 ? "link row" : "wing row");
    }

    // The block comparison, which is what decides section 44's framing.
    {
        double linkSum = 0.0, wingSum = 0.0, linkPeak = 0.0, wingPeak = 0.0;
        int linkCount = 0, wingCount = 0;
        for (const Entry& e : entries)
        {
            const double magnitude = std::fabs(e.sensitivity);
            if (e.i >= 4)
            {
                linkSum += magnitude * magnitude;
                linkPeak = std::max(linkPeak, magnitude);
                ++linkCount;
            }
            else
            {
                wingSum += magnitude * magnitude;
                wingPeak = std::max(wingPeak, magnitude);
                ++wingCount;
            }
        }
        const double linkRms = std::sqrt(linkSum / std::max(1, linkCount));
        const double wingRms = std::sqrt(wingSum / std::max(1, wingCount));
        std::printf("\n%22s %12s %12s\n", "rows", "rms", "peak");
        std::printf("%22s %12.5f %12.5f\n", "link (4-5)", linkRms, linkPeak);
        std::printf("%22s %12.5f %12.5f\n", "wing (0-3)", wingRms, wingPeak);
        std::printf("%22s %12.2f %12.2f\n", "link / wing",
                    wingRms > 0.0 ? linkRms / wingRms : 0.0,
                    wingPeak > 0.0 ? linkPeak / wingPeak : 0.0);
    }
    std::printf("\n");

    std::printf("\n  THE CHECK: a finite change to the top entries, predicted "
                "against measured.\n\n");
    std::printf("%22s %12s %13s %13s\n", "entry", "change", "predicted",
                "measured");
    for (std::size_t k = 0; k < entries.size() && k < 3; ++k)
    {
        const Entry& e = entries[k];
        const double relative = 0.02;
        const double absolute = relative * s[e.i] / s[e.j];
        double changed[N][N];
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) changed[i][j] = spectrum.phi[i][j];
        changed[e.i][e.j] += absolute;

        double measured = 0.0;
        bool ok = false;
        for (const std::complex<double>& candidate :
             Roots(CharacteristicPolynomial(changed)))
        {
            if (std::abs(candidate) < 1.0e-12) continue;
            const std::complex<double> lambda =
                std::log(candidate) / transitionTimeS;
            if (lambda.imag() <= 1.0e-6) continue;
            const double p = 2.0 * Pi / lambda.imag();
            if (p < 10.0 || p > 40.0) continue;
            measured = lambda.real() - growth; ok = true; break;
        }
        char label[48];
        std::snprintf(label, sizeof label, "d(%s)/d(%s)", names[e.i],
                      names[e.j]);
        if (!ok) { std::printf("%22s %11.1f%%  mode left the band\n", label,
                               relative * 100.0); continue; }
        std::printf("%22s %11.1f%% %+13.5f %+13.5f\n", label,
                    relative * 100.0, e.sensitivity * relative, measured);
    }

    std::printf(
        "\n  THE NUMBER THAT MATTERS IS THE COMPARISON. A 1%% change to "
        "d(swing)/d(swing)\n  moves sigma by +0.0133. Moving the tuned "
        "coefficient the whole way from 0.35\n  to 0.30 - the step that "
        "carries this wing from settling to not settling -\n  moves it "
        "+0.0129. ONE PER CENT OF ONE MATRIX ENTRY IS WORTH THE ENTIRE\n  "
        "COEFFICIENT STEP.\n\n"
        "  AND THE BIGGEST LEVER IS NOT IN THE LINK AT ALL. The control this "
        "check did\n  not originally carry - the other twenty-four entries - "
        "puts d(surge)/d(surge)\n  at +1.631, above every link entry, worth "
        "+0.0163 per 1%% or about one and a\n  quarter coefficient steps. By "
        "block the link's rows are still the more\n  sensitive on average, rms "
        "0.822 against 0.433, but the single largest lever\n  sits in the "
        "wing's own rows.\n\n"
        "  That is a real qualification of the paragraph above. Fragility here "
        "is a\n  property of a non-normal MODE, which moves far for a small "
        "change anywhere it\n  is receptive, and not a peculiarity of how the "
        "link is written - the same\n  fragility lands on core aerodynamics. "
        "Looking only at the link's rows was\n  looking where the light was.\n\n"
        "  So the phugoid's stability sits on a knife edge with respect to the "
        "link's\n  rows. That is consistent with everything above rather than "
        "new: cond = 0.10\n  said the mode is non-normal, and a non-normal "
        "mode is precisely one whose\n  eigenvalue moves far for a small "
        "change in the right place.\n\n"
        "  WHAT IT SUGGESTS ABOUT THE ITEM, AS A HYPOTHESIS AND NOT A "
        "CONCLUSION. Item 11\n  has been framed as finding a missing "
        "stabilising mechanism that would let the\n  ratio come down to the "
        "~0.06 pilot and line drag imply. This says the "
        "boundary's\n  LOCATION is not a robust property: a 1%% error anywhere "
        "in the link's rows\n  moves it by the whole 0.35-to-0.30 step. A "
        "number that fragile is more a\n  property of how the link is written "
        "than of a paraglider, and 'find the\n  missing mechanism' may "
        "therefore be the wrong frame - a formulation whose\n  stability is "
        "less sensitive would be the real requirement. Stated as the\n  "
        "hypothesis it is, because section 40 is what happens when a quantity "
        "that\n  moved the right way once gets promoted to a mechanism.\n\n"
        "  A RETRACTED CLAIM COMES BACK BY A DIFFERENT ROAD, AND THE "
        "RETRACTION STILL\n  STANDS. Section 39 concluded that the missing "
        "stabilising mechanism 'has to\n  act on speed stability'. Section 40 "
        "retracted it, because it had been inferred\n  from section 34's "
        "damping formula and that formula is anti-correlated with\n  the truth "
        "over this very parameter. The inference was invalid and remains\n  "
        "invalid.\n\n"
        "  But d(surge)/d(surge) is speed persistence - it IS speed stability - "
        "and it is\n  now the largest single lever in the matrix, measured by "
        "something that shares\n  no arithmetic with section 34. A conclusion "
        "can be correct while the argument\n  for it is worthless, and "
        "arriving at it again by a sound route is not the same\n  act as "
        "un-retracting it. The claim has support now; the reasoning that "
        "first\n  produced it is still wrong, and both of those go in the "
        "record.\n\n"
        "  ONE CONNECTION WORTH NOTING. The top entries in the LINK's rows are "
        "in the swing "
        "ANGLE row, and\n  two of the three - d(swing)/d(attitude) and "
        "d(swing)/d(heave) - are the link\n  taking its lean from the wing's "
        "attitude and its vertical motion. That is the\n  apparent-gravity "
        "tracking sections 34 and 35 measured as dalpha/dV =\n  -1.69 "
        "deg/(m/s), now appearing as the thing the stability is most "
        "sensitive\n  to rather than as a trim slope. The two are not shown "
        "here to be the same\n  quantity; they are the same physics seen from "
        "two sides, and that is worth a\n  measurement rather than an "
        "assertion.\n\n"
        "  THE CAVEAT THAT LIMITS ALL OF IT: these are single-entry changes "
        "with the\n  other thirty-five held fixed. A real change to the model "
        "would move several\n  entries coherently and they could cancel - the "
        "map says where the mode is\n  sensitive, not what any physical "
        "modification would do. Checking a candidate\n  means differencing its "
        "matrix, which is what section 42's split already does.\n\n");
}

void ReceptivityCheck(const CoupledParagliderSolver& solver,
                      const CoupledState& settled, double transitionTimeS)
{
    const Vec3 v0 = settled.velocityWorldMps;
    const double trimSpeed = std::sqrt(v0.x * v0.x + v0.z * v0.z);

    std::printf("RECEPTIVITY: is 0.985 a real number, and does it move?\n\n");
    std::printf("Two questions, and the first is an audit of section 42. A "
                "left eigenvector's\ncomponents carry units, so summing the "
                "link's share against the rest adds\nradians to metres per "
                "second - the trap section 41 caught `link/speed` in, which\n"
                "section 42 then did not apply to itself. Rescaling the states "
                "is a similarity\ntransform, so the eigenvalues are untouched; "
                "both shares are printed.\n\n");
    std::printf("Note what does NOT depend on it: cond = |w^H v| is invariant "
                "under the\nrescaling, since (Dw)^H (D^-1 v) = w^H v. So "
                "section 42's non-normality stands\nwhatever the share column "
                "does.\n\n");
    std::printf("THE PREDICTION: if the ratio changes only the GAIN, as "
                "section 41's surviving\nstory says, the receptivity sits "
                "still while sigma crosses zero. If it moves,\nthe coefficient "
                "is changing what the mode listens to and there is no fixed\n"
                "target for a stabilising mechanism to aim at.\n\n");

    std::printf("%8s %12s %13s %9s %12s %10s\n",
                "ratio", "share raw", "share scaled", "cond", "sigma 1/s",
                "period");
    for (const double ratio : {0.90, 0.70, 0.50, 0.40, 0.35, 0.32, 0.30, 0.28,
                               0.25})
    {
        CoupledParagliderSolver variant = solver;
        variant.SetSwingDampingRatio(ratio);
        const Spectrum spectrum = Analyse(variant, settled, transitionTimeS,
                                          1.0, false);
        const Receptivity r = MeasureReceptivity(spectrum.phi,
                                                 transitionTimeS, trimSpeed,
                                                 10.0, 40.0);
        if (!r.valid) { std::printf("%8.2f  no 16 s mode\n", ratio); continue; }
        std::printf("%8.2f %12.4f %13.4f %9.4f %+12.4f %9.2fs\n",
                    ratio, r.shareRaw, r.shareScaled, r.conditioning,
                    r.growthPerS, r.periodS);
    }

    std::printf(
        "\n  BOTH ANSWERS ARE NEGATIVE FOR WHAT CAME BEFORE THEM.\n\n"
        "  THE AUDIT: 0.985 does not survive. At ratio 0.35 the raw share is "
        "0.9854 and\n  the scaled share is 0.7786 - the same vector, the same "
        "mode, a change of\n  units. Section 42's qualitative claim stands, "
        "because 78%% is still\n  link-dominated and the conclusion drawn from "
        "it does not need three digits;\n  the NUMBER does not, and it was "
        "quoted to three. It was flattered by adding\n  radians to metres per "
        "second. Section 41 had already caught exactly this and\n  section 42 "
        "did not apply the lesson to itself, one level later, in the same\n  "
        "file.\n\n"
        "  What is untouched is `cond`, invariant under the rescaling by "
        "construction:\n  0.10 at the operating point, so the non-normality "
        "that made the mechanism\n  intelligible is a real property and not a "
        "unit artefact.\n\n"
        "  THE PREDICTION ALSO FAILED: the receptivity does not sit still. "
        "Scaled, the\n  link's share of the adjoint falls 0.8898 to 0.7562 "
        "across the sweep - 15%% -\n  while sigma goes -0.077 to +0.008. "
        "Monotone, and in the direction nobody\n  would have guessed: as the "
        "mode destabilises it listens LESS through the\n  link, not more.\n\n"
        "  That is the second measurement to run this way. Section 41 found "
        "the link\n  ARTICULATING less against the wing as the ratio falls, "
        "0.383 to 0.266, and\n  now the adjoint's link share falls too. Shape "
        "and receptivity both move away\n  from the link while the mode goes "
        "unstable, so 'the phugoid destabilises\n  because it becomes more "
        "coupled to the link' is not what is happening, and\n  two independent "
        "columns now say so.\n\n"
        "  WHAT SURVIVES, STATED CAREFULLY. The link is the CHANNEL - the "
        "adjoint is\n  link-dominated at every ratio, 0.76 to 0.89 - and "
        "section 42's split stands:\n  the movement in sigma enters through "
        "the link's rows. But the TREND in sigma\n  is not carried by a trend "
        "in coupling strength, because that trend has the\n  wrong sign. What "
        "changes is the link's own dynamics, transmitted through a\n  channel "
        "that is itself weakening, and winning anyway.\n\n"
        "  FOR WHAT COMES NEXT: the place a stabilising mechanism has to enter "
        "moves by\n  about 15%% over the interval of interest. That is a "
        "design requirement with a\n  drift in it rather than a fixed target - "
        "worth knowing before something is\n  built to hit it, and not worth "
        "more than that.\n\n");
}

void SplitCheck(const CoupledParagliderSolver& solver,
                const CoupledState& settled, double transitionTimeS)
{
    std::printf("WHICH BLOCK CARRIES IT: the growth rate's movement, split "
                "across the matrix.\n\n");
    std::printf("Section 41 asked for the work the link does on the phugoid. "
                "This is that\nquestion with no model of energy in it: the left "
                "eigenvector turns a known\nchange in the MATRIX into the "
                "change it makes to the growth rate, entry by\nentry and "
                "additively. Split the entries into the wing's own block, the "
                "link's\nown block, and the two coupling blocks, and each one's "
                "share has a sign.\n\n");
    std::printf("THE PREDICTION: the surviving gain story says the COUPLING "
                "blocks carry the\nmovement. The link's own 2x2 carrying it "
                "would be the pendulum getting less\ndamped rather than "
                "destabilising the phugoid; the WING block carrying it would\n"
                "mean every coupling story since section 40 is wrong.\n\n");
    std::printf("THE CHECK, which cannot be fudged: the four shares must add "
                "up to the actual\nmeasured change in sigma. A step too large "
                "for first order fails here, and a\nfailed step is printed as "
                "failed.\n\n");

    std::printf("%16s %10s %11s %11s %10s %11s %11s %7s\n",
                "step", "wing", "link->wing", "wing->link", "link",
                "predicted", "measured", "cond");
    std::printf("%16s %10s %11s %11s %10s %11s %11s %7s\n",
                "", "", "", "", "", "", "", "adj");
    const double from[] = {0.35, 0.35, 0.30};
    const double to[] = {0.30, 0.25, 0.25};
    for (int which = 0; which < 3; ++which)
    {
        CoupledParagliderSolver a = solver, b = solver;
        a.SetSwingDampingRatio(from[which]);
        b.SetSwingDampingRatio(to[which]);
        const Spectrum sa = Analyse(a, settled, transitionTimeS, 1.0, false);
        const Spectrum sb = Analyse(b, settled, transitionTimeS, 1.0, false);
        const Split split = SplitGrowth(sa.phi, sb.phi, transitionTimeS,
                                        10.0, 40.0);
        char label[32];
        std::snprintf(label, sizeof label, "%.2f -> %.2f", from[which],
                      to[which]);
        if (!split.valid) { std::printf("%16s  no 16 s mode\n", label);
                            continue; }
        std::printf("%16s %+10.5f %+11.5f %+11.5f %+10.5f %+11.5f %+11.5f "
                    "%7.3f\n",
                    label, split.wing, split.linkToWing, split.wingToLink,
                    split.link, split.predictedTotal, split.measuredTotal,
                    split.conditioning);
        std::printf("%16s %10s %11s %11s %10s %11s %11s %7.3f\n",
                    "", "", "", "", "", "", "", split.adjointLinkShare);
    }
    std::printf("\n  Positive is destabilising. `predicted` is the sum of the "
                "four shares and\n  `measured` is the eigenvalues' own answer "
                "for the same step; they agree only\n  while the step is small "
                "enough for the expansion, which is why three steps\n  are "
                "printed rather than one. `cond` is |w^H v|, the mode's "
                "conditioning;\n  `adj` is the share of the left eigenvector "
                "sitting on the link's two rows.\n\n");

    std::printf(
        "  WHAT IS NEARLY TAUTOLOGICAL, SAID FIRST. That 99%% of the movement "
        "enters\n  through rows 4-5 is barely a finding: `swingDampingRatio` "
        "appears in the\n  link's own update equation and nowhere else, so "
        "those are the only rows whose\n  entries change. Reporting that as a "
        "discovery would be dressing up where the\n  coefficient lives.\n\n"
        "  THE FINDING IS THE ADJOINT, AND IT IS 0.985. Changing rows 4-5 "
        "moves the\n  PHUGOID's eigenvalue only if the phugoid's left "
        "eigenvector has weight there,\n  and that is not automatic - it is "
        "measurable, and it is essentially all of it.\n  98.5%% of the 16 s "
        "mode's adjoint sits on the link's two rows.\n\n"
        "  So the mode LOOKS like a speed oscillation and LISTENS almost "
        "entirely\n  through the link. Its right eigenvector is speed-"
        "dominated - articulation\n  0.29 against the pendulum's 1.07, which "
        "is why every trace of it reads as a\n  phugoid - while its "
        "receptivity is link. Those two being different is\n  non-normality, "
        "and `cond` = 0.10 says so directly: for a normal mode |w^H v|\n  "
        "would be near 1.\n\n"
        "  THAT IS THE MECHANISM SECTIONS 40 AND 41 WERE CIRCLING. A "
        "coefficient living\n  only in the link's equations can take the "
        "phugoid's damping through zero\n  because the phugoid's adjoint is "
        "link. And section 34's two-state theory\n  cannot see any of it, not "
        "because its aerodynamics are wrong - its period\n  prediction is "
        "still good to 1-4%% - but because it HAS no link row for the\n  mode "
        "to listen through. A model can be right about what a mode looks like "
        "and\n  structurally unable to say what changes its stability.\n\n"
        "  It also fits section 41 without being fitted to it: a receptivity "
        "that is\n  fixed while a gain changes is exactly a constant phase "
        "with a moving\n  amplitude, which is what the mode shapes showed.\n\n"
        "  WHAT IS NOT ESTABLISHED. `cond` = 0.10 means this mode is "
        "non-normal enough\n  that the individual shares are amplified "
        "relative to a well-conditioned mode;\n  the sum check validates the "
        "total, not each share's precision. The split\n  WITHIN rows 4-5 - "
        "0.0067 from the link's own 2x2 against 0.0052 from\n  wing->link - is "
        "therefore reported as roughly even rather than as a ratio\n  worth "
        "quoting. And link->wing contributing 0.5%% says only that the link's\n"
        "  influence ON the wing is not what CHANGES; it does not say that "
        "influence is\n  small, which is a different measurement and is not "
        "made here.\n\n");
}

Shape ShapeOf(const Spectrum& spectrum, const double phi[N][N],
              double transitionTimeS, double trimSpeedMps,
              double lowPeriodS, double highPeriodS)
{
    Shape out;
    // Re-derive the discrete eigenvalue for the mode in the wanted band.
    const std::vector<double> polynomial = CharacteristicPolynomial(phi);
    const std::vector<std::complex<double>> discrete = Roots(polynomial);
    for (const std::complex<double>& mu : discrete)
    {
        if (std::abs(mu) < 1.0e-12) continue;
        const std::complex<double> lambda = std::log(mu) / transitionTimeS;
        if (std::fabs(lambda.imag()) < 1.0e-6) continue;
        const double period = 2.0 * Pi / std::fabs(lambda.imag());
        if (period < lowPeriodS || period > highPeriodS) continue;
        // One of the conjugate pair is enough; take the one with positive
        // imaginary part so the phase sign means the same thing every time.
        if (lambda.imag() < 0.0) continue;
        const std::vector<std::complex<double>> v = Eigenvector(phi, mu);
        const std::complex<double> surge = v[0] / trimSpeedMps;
        const std::complex<double> swing = v[4];
        if (std::abs(surge) < 1.0e-300) continue;
        out.linkPerSpeed = std::abs(swing) / std::abs(surge);
        out.phaseDeg = std::arg(swing / surge) * 180.0 / Pi;
        if (std::abs(v[2]) > 1.0e-300)
            out.articulation = std::abs(v[4] - v[2]) / std::abs(v[2]);
        double largest = 0.0, worst = 0.0;
        for (int i = 0; i < N; ++i) largest = std::max(largest, std::abs(v[i]));
        for (int i = 0; i < N; ++i)
        {
            std::complex<double> row(0.0, 0.0);
            for (int j = 0; j < N; ++j) row += phi[i][j] * v[j];
            worst = std::max(worst, std::abs(row - mu * v[i]));
        }
        out.residual = largest > 0.0 ? worst / largest : 0.0;
        out.periodS = period;
        out.growthPerS = lambda.real();
        out.valid = true;
        return out;
    }
    (void)spectrum;
    return out;
}

void ShapeCheck(const CoupledParagliderSolver& solver,
                const CoupledState& settled, double transitionTimeS)
{
    const Vec3 v0 = settled.velocityWorldMps;
    const double trimSpeed = std::sqrt(v0.x * v0.x + v0.z * v0.z);

    std::printf("MODE SHAPE: what the link is doing inside the 16 s mode, and "
                "WHEN.\n\n");
    std::printf("Section 40 put the mechanism in the pendulum-phugoid "
                "coupling, which is a\nproperty of the mode SHAPE - and the "
                "shapes are already in the matrix the\neigenvalues came from. "
                "Amplitude alone proves nothing: a pilot under an\n"
                "accelerating wing leans regardless, which is the apparent-"
                "gravity tracking\nsections 34 and 35 measured. PHASE is the "
                "measurement, because a lag inside\na feedback loop is what "
                "turns a restoring term into a driving one.\n\n");
    std::printf("THE PREDICTION: the link's phase against surge in the 16 s "
                "mode moves\nsystematically as the ratio falls, and the "
                "damping follows it. If the phase\nsits still while the "
                "damping crosses zero, the coupling is not the mechanism\n"
                "either and section 40's suspect is wrong too.\n\n");

    std::printf("THE CONTROL FIRST: the 1.86 s mode is the pendulum and the "
                "16 s mode is the\nphugoid, so the fast mode must come back "
                "LINK-heavy and the slow mode\nSPEED-heavy. If it does not, "
                "this code is broken and the table after it is\nnoise.\n\n");
    std::printf("  The FIRST version of this control failed - fast 0.42 "
                "against slow 0.46 on\n  link/speed, when the pendulum should "
                "have dominated - and the failure was\n  not decidable as "
                "written: a control that compares a scaled amplitude cannot\n"
                "  tell a broken eigenvector from a wrong expectation about "
                "it. Two columns\n  were added rather than reinterpreting the "
                "number after seeing it. `residual`\n  is the arithmetic's own "
                "verdict, ||(Phi - mu I)v|| / ||v||, which answers the\n  "
                "first question with no physics in it; `articulation` is "
                "|swing - attitude|\n  over |attitude|, both radians, which "
                "answers the second with no scaling in\n  it. link/speed is "
                "kept, and kept honest, by printing what it did.\n\n");
    std::printf("%10s %12s %13s %13s %11s %10s\n",
                "mode", "residual", "articulation", "link/speed", "phase deg",
                "period");
    {
        CoupledParagliderSolver reference = solver;
        reference.SetSwingDampingRatio(0.35);
        const Spectrum spectrum = Analyse(reference, settled, transitionTimeS,
                                          1.0, false);
        const Shape fast = ShapeOf(spectrum, spectrum.phi, transitionTimeS,
                                   trimSpeed, 1.0, 8.0);
        const Shape slow = ShapeOf(spectrum, spectrum.phi, transitionTimeS,
                                   trimSpeed, 10.0, 40.0);
        if (fast.valid)
            std::printf("%10s %12.2e %13.2f %13.2f %11.1f %9.2fs\n", "fast",
                        fast.residual, fast.articulation, fast.linkPerSpeed,
                        fast.phaseDeg, fast.periodS);
        if (slow.valid)
            std::printf("%10s %12.2e %13.2f %13.2f %11.1f %9.2fs\n", "slow",
                        slow.residual, slow.articulation, slow.linkPerSpeed,
                        slow.phaseDeg, slow.periodS);
    }

    std::printf("\n%8s %11s %13s %12s %11s %11s\n",
                "ratio", "residual", "articulation", "phase deg", "sigma 1/s",
                "period");
    for (const double ratio : {0.90, 0.70, 0.50, 0.40, 0.35, 0.32, 0.30, 0.28,
                               0.25})
    {
        CoupledParagliderSolver variant = solver;
        variant.SetSwingDampingRatio(ratio);
        const Spectrum spectrum = Analyse(variant, settled, transitionTimeS,
                                          1.0, false);
        const Shape slow = ShapeOf(spectrum, spectrum.phi, transitionTimeS,
                                   trimSpeed, 10.0, 40.0);
        if (!slow.valid) { std::printf("%8.2f  no 16 s mode\n", ratio);
                           continue; }
        std::printf("%8.2f %11.2e %13.3f %12.1f %+11.4f %10.2fs\n", ratio,
                    slow.residual, slow.articulation, slow.phaseDeg,
                    slow.growthPerS, slow.periodS);
    }
    std::printf("\n  `link/speed` is radians of link lean per unit fractional "
                "speed change -\n  surge divided by the trim speed, the link "
                "angle left in radians. The scaling\n  is stated because the "
                "number depends on it; the PHASE does not.\n\n");

    std::printf(
        "  THE PREDICTION FAILED, AND IT WAS ALSO BADLY POSED. Both are worth "
        "saying.\n\n"
        "  Failed: the phase does not move. It sits between -107.4 and -109.3 "
        "degrees\n  across the entire sweep - 1.9 degrees - while sigma goes "
        "from -0.077 to +0.008\n  and changes sign. At T = 0.10 it is -106.4 "
        "to -108.2, the same span in the\n  same place, so this is the "
        "aircraft and not the sampling interval.\n\n"
        "  Badly posed: the prediction treated phase as the only way a "
        "coupling can\n  change how much energy it moves. It is not. Work per "
        "cycle goes as amplitude\n  TIMES the sine of the phase, and the "
        "amplitude is not fixed - link/speed\n  rises monotonically from 0.319 "
        "to 0.510 over the same sweep, a 60%% change,\n  while the phase "
        "holds. So the test refutes the LAG version of the coupling\n  "
        "hypothesis and leaves a GAIN version standing.\n\n"
        "  That distinction is worth the run, because the lag version is the "
        "one the\n  solver itself blames: the link is damped against the "
        "WORLD, and the comment\n  on that line calls the resulting tracking "
        "lag 'a cost paid knowingly'. A lag\n  inside a feedback loop is the "
        "textbook way to sustain an oscillation, it was\n  the natural "
        "suspect, and the phase column is where it would have shown. It\n  "
        "does not show.\n\n"
        "  WHAT IS NOT ESTABLISHED: the gain story was not predicted in "
        "advance and is\n  not claimed here. It is consistent with the "
        "amplitude column and that is all\n  - the same trap section 40 caught "
        "section 34 in, where a quantity that moved\n  the right way at one "
        "point was mistaken for a mechanism. The test it needs is\n  the "
        "energy integral: how much work the link term does on the phugoid over "
        "a\n  cycle, evaluated on the eigenvector, which is computable from "
        "what is already\n  here and is a number whose SIGN is the answer.\n\n"
        "  ONE THING RUNS AGAINST INTUITION AND IS RECORDED BEFORE IT IS "
        "EXPLAINED: as the\n  ratio falls the link ARTICULATES LESS against "
        "the wing, 0.383 down to 0.266,\n  not more. Less link damping does "
        "not mean a link swinging more freely inside\n  this mode. Nothing "
        "here explains that yet.\n\n");
}

void OwnTrimSweep(const CanopyGeometry& canopy, const LinePlanSpec& linePlan,
                  double transitionTimeS, int maximumSeconds)
{
    std::printf("OWN TRIM: settle each ratio from scratch, then take the "
                "eigenvalues about the\nstate it actually reaches. This is the "
                "sweep above with its one assumption\nremoved.\n\n");
    std::printf("%8s %9s %9s %9s %10s %11s %11s %13s\n",
                "ratio", "settle", "alpha", "period", "sigma 1/s", "zeta",
                "worst mode", "outcome");
    for (const double ratio : {0.90, 0.50, 0.35, 0.30, 0.25, 0.15})
    {
        const OwnTrim trim = SettleAt(canopy, linePlan, ratio, maximumSeconds);
        std::printf("%8.2f %8ds %8.2fd", ratio, trim.settleSeconds,
                    trim.incidenceRad * 180.0 / Pi);
        if (trim.departed)
        {
            std::printf(" %9s %10s %11s %11s %13s\n", "-", "-", "-", "-",
                        "DEPARTED settling");
            continue;
        }
        const Spectrum spectrum = Analyse(trim.solver, trim.state,
                                          transitionTimeS, 1.0, false);
        const Mode worst = LargestRealPart(spectrum);
        const char* outcome = trim.settled ? "settled" : "NOT SETTLED";
        Mode fast;
        if (!FastestGrowing(spectrum, fast))
            std::printf(" %9s %10s %11s", "-", "no mode", "in band");
        else
            std::printf(" %8.2fs %+10.4f %11.4f", fast.periodS,
                        fast.growthPerS, fast.dampingRatio);
        PrintWorst(worst);
        std::printf(" %13s\n", outcome);
    }
    std::printf("\n  A row that departed while settling never had a trim to "
                "linearise about, and\n  that is the answer to its ratio, not "
                "a gap in the table. A row that settled\n  with sigma negative "
                "says the trim is locally stable at a ratio the aircraft\n  is "
                "documented to leave - which would put the departure outside "
                "what any\n  linearisation can reach.\n\n");
}

// What the two tables came back with, written where the numbers are rather
// than only in PHYSICS_LEARNINGS.
void Verdict()
{
    std::printf(
        "VERDICT\n\n"
        "  The prediction failed, and the failure names the mode. The FAST "
        "mode does not\n  cross: its real part goes -0.357 to -0.291 /s from "
        "ratio 0.90 to 0.10, and its\n  period does not move at all - 1.86 s "
        "at every ratio, through a ninefold\n  change in the coefficient. "
        "Whatever pins the ratio at 0.35, it is not the\n  pendulum going "
        "unstable.\n\n"
        "  Something does cross, between 0.28 and 0.25, at both transition "
        "times. It is\n  the 16 s PHUGOID: the slow mode, whose period the "
        "worst-mode column tracks\n  from 23.9 s down to 14.0 s as the ratio "
        "falls, and whose damping goes through\n  zero on the way.\n\n"
        "  The own-trim table brackets the same boundary by a criterion that "
        "shares no\n  arithmetic with an eigenvalue: whether a trim EXISTS. At "
        "0.30 the wing settles\n  and its spectrum is stable by a hair "
        "(-0.008/s); at 0.25 it departs during its\n  own settle, at 348 s and "
        "20 degrees, so there is no trim there to linearise\n  about at all. "
        "Two instruments, one interval.\n\n"
        "  That contradicts PHYSICS_LEARNINGS section 35, which acquitted the "
        "phugoid,\n  and the reason it does is worth more than the crossing. "
        "Section 35 tested\n  whether the lift exponent n crosses zero - that "
        "is a claim about the mode's\n  FREQUENCY, `omega = g sqrt(n) / V`, "
        "and about a divergence. It measured n\n  staying at 0.14 to 0.19 and "
        "concluded the phugoid does not reach the\n  departure. But n staying "
        "positive only rules out the frequency going\n  imaginary. The phugoid "
        "arrives by its DAMPING instead, which `omega` says\n  nothing about, "
        "and the period staying near 16 s across the boundary is\n  exactly "
        "what both the old measurement and this one report. A prediction "
        "about\n  one part of an eigenvalue was refuted and the conclusion was "
        "drawn about the\n  whole eigenvalue.\n\n"
        "  TWO THINGS THIS DOES NOT SETTLE, and neither is a detail:\n\n"
        "  1. The quantity that crosses is the one number this instrument has "
        "already\n     declared not pinned. The linearity check converged the "
        "fast mode's period\n     and damping and the slow mode's period, and "
        "the slow mode's DAMPING moved\n     with both T and perturbation size "
        "- 0.0362 against 0.0334 on a halved step.\n     The crossing is a "
        "sign change in precisely that number. Its ORDER is solid,\n     "
        "monotone in the ratio and agreed by three runs; the 0.28-0.25 "
        "interval\n     inherits the uncertainty and should not be quoted "
        "tighter than that.\n\n"
        "  2. The flown column did not confirm anything. It fitted at two "
        "ratios, both\n     at R2 0.24-0.42, and returned -0.045/s where the "
        "eigenvalue says -0.357/s;\n     at every ratio that matters it "
        "returned nothing at all. The diagnosis is\n     the instrument, not "
        "the aircraft: it watches the LINK, which is the fast\n     mode, and "
        "runs for 40 s, and the mode that crosses is a 16 s oscillation\n     "
        "growing at 0.008/s - thirty per cent across the whole window, on a "
        "signal\n     the fast mode has already left. Reaching it needs "
        "hundreds of seconds and\n     the phugoid's own observable. Until "
        "that runs, the crossing is a claim made\n     by eigenvalues alone "
        "and has no trajectory behind it.\n\n"
        "  Also unexplained: section 35 identified the growing mode on a "
        "departing wing\n  at 3.6-5.7 s, and no such mode is in this spectrum "
        "at any ratio - the pendulum\n  sits at 1.86 s and does not move. That "
        "measurement was taken at large amplitude\n  on a wing already "
        "leaving, where a linearisation has no claim, so the two are\n  not "
        "yet in contradiction. They are also not reconciled.\n\n");
}
}

int main(int argc, char** argv)
{
    std::printf("Level 10: the pitch axis by linearisation. PHYSICS_TODO item "
                "11.\n");
    std::printf("Perturb the settled aircraft, difference against an "
                "unperturbed run, take the\neigenvalues of the transition "
                "matrix. No excitation, no window, no filter.\n\n");

    const CanopyGeometry canopy;
    const LinePlanSpec linePlan = Epic2MlLinePlan();

    int settleSeconds = 420;
    double transition = 1.0;
    bool stepCheck = false;
    bool sweep = false;
    bool phugoid = false;
    bool shape = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string argument = argv[i];
        if (argument == "--quick") settleSeconds = 120;
        if (argument == "--step") stepCheck = true;
        if (argument == "--sweep") sweep = true;
        if (argument == "--phugoid") phugoid = true;
        if (argument == "--shape") shape = true;
    }

    std::printf("THE CHECK: the slow mode is independently measured at period "
                "16.39 s and\ndamping ratio 0.031, off 27 peaks of a 1200 s "
                "run by `pitch_axis_trace\n--slow-mode`. If it is not in the "
                "table below, this instrument is wrong.\n\n");

    // Linearise about TRIM, which on this aircraft means several hundred
    // seconds - the slow mode's own settling time. Paid ONCE: the settled
    // solver and state are copied for every perturbation and for every variant
    // below, so all of them start from bit-identical conditions. Re-settling
    // per variant would cost twenty minutes each and, worse, would compare
    // answers taken about slightly different trims.
    CoupledParagliderSolver solver(canopy, linePlan);
    CoupledState settled;
    for (int second = 0; second < settleSeconds; ++second)
    {
        for (int step = 0; step < 120; ++step)
            solver.Step(settled, CoupledControls{}, CoupledAtmosphere{});
    }

    // Three transition times, not one, and the reason is ALIASING - which the
    // first run of this walked straight into. The transition time is a
    // sampling interval, so every mode faster than 2T folds onto the negative
    // real axis and reports a period of exactly 2T. That is precisely what
    // came back: 4.00 s at T = 2, 12.00 s at T = 6. Two suspiciously round
    // numbers, both exactly twice their own sampling interval, and not a
    // property of the aircraft at all.
    //
    // So T has to sit below half the fastest mode that matters, and the
    // aerodynamic interval sets the floor at 0.1 s because loads are held
    // between solves and a shorter T measures the hold rather than the wing.
    // A mode that is real appears at every T in that band; one that moves with
    // T is the sampling. Same lesson as the aerodynamic-interval sweep in
    // `pitch_axis_trace`, on a different knob.
    // 2 s is included deliberately even though it aliases the fast mode to
    // exactly 4 s: the SLOW mode's eigenvalue sits closer to 1 the shorter T
    // is, so a short T resolves its damping worst. The two modes want
    // different sampling intervals and there is no single best one, which is
    // an argument for reporting the band rather than picking a favourite.
    for (const double t : {0.1, 0.25, 0.5, 2.0})
        Report(solver, settled, settleSeconds, t, 1.0);

    if (stepCheck)
    {
        // The one assumption this method makes is that the perturbations are
        // small enough to behave linearly. Halving them is how that is
        // checked, and an answer that moves is an answer that was not
        // linearised.
        // Halve the perturbation at the SAME transition times as above, which
        // the first version of this failed to do - it halved the step and
        // changed T in the same breath, so the comparison had two variables
        // in it and tested nothing. A check that cannot fail cleanly is not a
        // check.
        std::printf("Halved perturbations - the linearity check. Against the "
                    "matching T above,\nthese numbers should not move.\n\n");
        Report(solver, settled, settleSeconds, 0.25, 0.5);
        Report(solver, settled, settleSeconds, 2.0, 0.5);
    }
    if (sweep)
    {
        // Two transition times for the same reason the table above has four:
        // a mode that is real appears at both, and a number that moves with T
        // is the sampling interval talking. Both sit well below half the
        // pendulum band, so neither aliases it.
        Sweep(solver, settled, 0.25, true);
        Sweep(solver, settled, 0.10, false);
        // Same question, asked at each ratio's own trim instead of at 0.35's.
        OwnTrimSweep(canopy, linePlan, 0.25,
                     settleSeconds < 420 ? 180 : 420);
        Verdict();
    }

    if (phugoid)
    {
        // 300 s is nineteen periods of the slow mode and rather more than two
        // e-foldings of the growth the sweep predicts at 0.25. A shorter
        // window is what broke the last attempt at this.
        PhugoidCheck(solver, settled, 300.0);
        ExponentCheck(solver, settled, 300.0);
    }

    if (shape)
    {
        // Free: no new flying, the same matrix the eigenvalues came from.
        ShapeCheck(solver, settled, 0.25);
        // A phase that is a property of the aircraft does not care what the
        // sampling interval was; one that moves with T is the discretisation.
        ShapeCheck(solver, settled, 0.10);
        SplitCheck(solver, settled, 0.25);
        ReceptivityCheck(solver, settled, 0.25);
        // At 0.30: the last ratio that still has a stable trim, which is where
        // a stabilising mechanism would have to do its work.
        DesignCheck(solver, settled, 0.25, 0.30);
    }
    (void)transition;
    return 0;
}
