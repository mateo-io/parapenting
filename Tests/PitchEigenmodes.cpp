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
    bool oscillatory = false;
};

void Report(const CoupledParagliderSolver& solver, const CoupledState& settled,
            int settleSeconds, double transitionTimeS, double scale)
{
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
    bool conventionsAgree = true;
    for (int j = 0; j < N; ++j)
    {
        const double delta = nominal[j] * scale;
        CoupledState perturbed = settled;
        Perturb(perturbed, j, delta);
        const double gain = (Read(perturbed).value[j] - reference.value[j])
            / delta;
        if (std::fabs(gain - 1.0) > 0.02)
        {
            std::printf("  STATE %d: perturbing by delta moves it by %+.3f "
                        "delta - Perturb and Read disagree, and every number "
                        "below is void\n", j, gain);
            conventionsAgree = false;
        }
    }
    if (!conventionsAgree) std::printf("\n");

    double phi[N][N] = {{0}};
    for (int j = 0; j < N; ++j)
    {
        const double delta = nominal[j] * scale;
        CoupledState perturbed = settled;
        Perturb(perturbed, j, delta);
        const Reduced moved = advance(perturbed);
        for (int i = 0; i < N; ++i)
            phi[i][j] = (moved.value[i] - base.value[i]) / delta;
    }

    // The matrix itself, because the eigenvalues are currently wrong and the
    // next person needs the input to the arithmetic rather than its output.
    // Read it as: column j is what a unit perturbation of state j has become
    // after T seconds.
    std::printf("  transition matrix, T = %.2f s (columns: du, dw, dtheta, "
                "dq, dswing, dswingrate)\n", transitionTimeS);
    for (int i = 0; i < N; ++i)
    {
        std::printf("   ");
        for (int j = 0; j < N; ++j) std::printf("%11.5f", phi[i][j]);
        std::printf("\n");
    }
    std::printf("\n");

    const std::vector<double> polynomial = CharacteristicPolynomial(phi);
    const std::vector<std::complex<double>> discrete = Roots(polynomial);

    // Discrete eigenvalue to continuous: mu = exp(lambda T).
    std::vector<Mode> modes;
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
        mode.timeToHalfS = lambda.real() < 0.0
            ? std::log(2.0) / -lambda.real() : 0.0;
        modes.push_back(mode);
    }
    std::sort(modes.begin(), modes.end(),
              [](const Mode& a, const Mode& b)
              { return a.periodS > b.periodS; });

    std::printf("Settle %d s, transition time %.2f s, perturbation scale "
                "%.2fx\n\n", settleSeconds, transitionTimeS, scale);
    std::printf("%12s %12s %12s %14s\n",
                "period", "damping", "half life", "kind");
    for (const Mode& mode : modes)
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
    for (int i = 1; i < argc; ++i)
    {
        const std::string argument = argv[i];
        if (argument == "--quick") settleSeconds = 120;
        if (argument == "--step") stepCheck = true;
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
    (void)transition;
    return 0;
}
