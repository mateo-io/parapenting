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
        "mode does not\n  cross: its real part sits at -0.32/s from ratio 0.90 "
        "to 0.10, moving 12%% over\n  a ninefold change in the coefficient, "
        "and its period does not move at all.\n  Whatever pins the ratio at "
        "0.35, it is not the pendulum going unstable.\n\n"
        "  Something does cross, between 0.28 and 0.25, at both transition "
        "times and\n  again about each ratio's own trim - the documented "
        "boundary, from three runs\n  that did not know about each other. It "
        "is the 16 s PHUGOID: the slow mode,\n  whose period the worst-mode "
        "column tracks from 16.3 s down to 13.8 s as the\n  ratio falls, and "
        "whose damping goes through zero on the way.\n\n"
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
        "ratios, both\n     at R2 0.26-0.43, and returned -0.047/s where the "
        "eigenvalue says -0.37/s;\n     at every ratio that matters it "
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
    for (int i = 1; i < argc; ++i)
    {
        const std::string argument = argv[i];
        if (argument == "--quick") settleSeconds = 120;
        if (argument == "--step") stepCheck = true;
        if (argument == "--sweep") sweep = true;
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
    (void)transition;
    return 0;
}
