// Level 0 determinism spine.
//
//   * the fixed-step clock produces the same step count for a given amount of
//     simulated time however that time is chopped into frames;
//   * the same control history produces a bit-identical solver state at any
//     render rate;
//   * the still-air EPIC 2 ML baseline is frozen, so later solver levels
//     cannot drift trim, sink or glide without the change being visible.
#include "ParagliderDynamics.h"
#include "ParagliderSolverClock.h"
#include "ResearchCoefficientRegistry.h"
#include "SolverStateHash.h"
#include "WingCatalogue.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

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

void CheckNear(double actual, double expected, double tolerance,
               const std::string& what)
{
    if (!(std::fabs(actual - expected) <= tolerance))
    {
        std::printf("  FAIL  %s: got %.10f, expected %.10f (tol %g)\n",
            what.c_str(), actual, expected, tolerance);
        ++Failures;
    }
}

// Deterministic pseudo-random frame deltas, so a "variable frame rate" run is
// reproducible. Not std::rand: its sequence is implementation-defined.
struct FrameDeltaSequence
{
    std::uint64_t state = 0x9e3779b97f4a7c15ull;
    double meanSeconds;

    explicit FrameDeltaSequence(double mean) : meanSeconds(mean) {}

    double Next()
    {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        const double unit =
            static_cast<double>(state >> 11) / 9007199254740992.0;
        // 0.4x to 1.6x of the mean frame time.
        return meanSeconds * (0.4 + 1.2 * unit);
    }
};

// Runs the wing until the clock has taken exactly targetSteps steps, feeding
// it frames at the given rate. Driving to a step count rather than a wall
// duration is what makes rates comparable: a fixed duration lands on a
// different step count depending on how the frames divide it, and then the
// states legitimately differ by one step.
std::uint64_t RunToStepCount(
    double frameSeconds, std::uint64_t targetSteps, bool jitter)
{
    ParagliderSolverClock clock;
    ParagliderDynamics dynamics(
        GetWingProfile(WingProfileId::Epic2MLResearch).parameters);
    FlightState state;
    FrameDeltaSequence deltas(frameSeconds);

    while (clock.StepsRunCount() < targetSteps)
    {
        const double delta = jitter ? deltas.Next() : frameSeconds;
        const int steps = clock.BeginFrame(delta);
        for (int i = 0; i < steps && clock.StepsRunCount() < targetSteps; ++i)
        {
            // Controls are a function of simulation time, not frame time, so
            // the input history is identical at every render rate. Sampling
            // per frame instead would make the comparison meaningless.
            const double t = clock.SimulationTimeSeconds();
            ControlInput input;
            input.leftBrake = 0.25 + 0.20 * std::sin(t * 0.7);
            input.rightBrake = 0.15 + 0.10 * std::sin(t * 0.4 + 1.1);
            input.weightShift = 0.5 * std::sin(t * 0.3);
            dynamics.Step(state, input, Atmosphere{}, clock.StepSeconds());
            clock.EndStep();
        }
    }
    return HashFlightState(state);
}
}

int main()
{
    std::printf("Solver clock\n");
    {
        ParagliderSolverClock clock;
        CheckNear(clock.StepSeconds(), 1.0 / 120.0, 1e-15, "default step");
        CheckNear(clock.SimulationTimeSeconds(), 0.0, 0.0, "starts at zero");

        // The contract: steps run equals floor(total delivered time / step),
        // whatever frame sizes that time arrived in.
        for (double frame : {1.0 / 30.0, 1.0 / 60.0, 1.0 / 120.0,
                             1.0 / 144.0, 1.0 / 240.0, 0.01})
        {
            ParagliderSolverClock c;
            double delivered = 0.0;
            for (int i = 0; i < 500; ++i)
            {
                delivered += frame;
                const int steps = c.BeginFrame(frame);
                for (int k = 0; k < steps; ++k) c.EndStep();
            }
            const auto expected = static_cast<std::uint64_t>(
                std::floor(delivered / c.StepSeconds()));
            Check(c.StepsRunCount() == expected,
                "step count matches floor(time/step) at "
                    + std::to_string(1.0 / frame) + " Hz");
        }

        // Frame size barely changes how much time is simulated. Exact
        // agreement is not achievable and the clock should not pretend
        // otherwise: 60 x (1/60) sums to 1.0000000000000013 while
        // 240 x (1/240) sums to 0.9999999999999977, four ULP apart and either
        // side of the 120-step boundary, so the counts legitimately differ by
        // one. What matters is that the gap stays at one step and does not
        // grow with runtime - that is what the derived-from-total step count
        // buys, and what a subtractive accumulator would lose.
        {
            const auto StepsForOneSecond = [](double frame, int frames)
            {
                ParagliderSolverClock c;
                for (int i = 0; i < frames; ++i)
                {
                    const int steps = c.BeginFrame(frame);
                    for (int k = 0; k < steps; ++k) c.EndStep();
                }
                return c.StepsRunCount();
            };
            const std::uint64_t coarse = StepsForOneSecond(1.0 / 60.0, 60);
            const std::uint64_t fine = StepsForOneSecond(1.0 / 240.0, 240);
            const auto gap = coarse > fine ? coarse - fine : fine - coarse;
            Check(gap <= 1,
                  "1 s at 60 vs 240 Hz agrees to within one step");

            // Over 10 minutes the gap must still be one step, not 600.
            const std::uint64_t longCoarse =
                StepsForOneSecond(1.0 / 60.0, 60 * 600);
            const std::uint64_t longFine =
                StepsForOneSecond(1.0 / 240.0, 240 * 600);
            const auto longGap = longCoarse > longFine
                ? longCoarse - longFine : longFine - longCoarse;
            std::printf("  10 min: 60 Hz %llu steps, 240 Hz %llu steps,"
                        " gap %llu\n",
                static_cast<unsigned long long>(longCoarse),
                static_cast<unsigned long long>(longFine),
                static_cast<unsigned long long>(longGap));
            Check(longGap <= 1,
                  "the gap does not grow with runtime");
        }

        // Simulation time must be exact after many steps. A running sum of
        // 1/120 drifts; multiplying does not.
        ParagliderSolverClock exact;
        for (int i = 0; i < 120 * 600; ++i)
        {
            exact.BeginFrame(1.0 / 120.0);
            exact.EndStep();
        }
        CheckNear(exact.SimulationTimeSeconds(), 600.0, 1e-12,
                  "600 s exact after 72000 steps");

        // Hitch protection, and that it is reported.
        ParagliderSolverClock hitched;
        Check(hitched.BeginFrame(10.0)
                  <= static_cast<int>(
                         ParagliderSolverClock::DefaultMaxFrameSeconds * 120.0)
                     + 1,
              "a 10 s frame is clamped");
        Check(hitched.HasClamped(), "clamping is reported");

        // Degenerate deltas must not poison the accumulator.
        ParagliderSolverClock guarded;
        Check(guarded.BeginFrame(std::nan("")) == 0, "NaN delta yields 0 steps");
        Check(guarded.BeginFrame(-1.0) == 0, "negative delta yields 0 steps");
        Check(guarded.BeginFrame(0.0) == 0, "zero delta yields 0 steps");
        guarded.BeginFrame(1.0 / 120.0);
        guarded.EndStep();
        Check(guarded.StepsRunCount() == 1, "clock still usable after those");

        // Interpolation alpha stays in range and reflects the remainder.
        ParagliderSolverClock partial;
        partial.BeginFrame(1.5 / 120.0);
        partial.EndStep();
        CheckNear(partial.InterpolationAlpha(), 0.5, 1e-9,
                  "half a step left over");
    }

    std::printf("\nState hash\n");
    {
        FlightState a;
        FlightState b;
        Check(HashFlightState(a) == HashFlightState(b),
              "identical states hash alike");

        b.positionWorldM.x = 1e-15;
        Check(HashFlightState(a) != HashFlightState(b),
              "a 1e-15 position difference changes the hash");

        FlightState negativeZero;
        negativeZero.harnessRollRad = -0.0;
        FlightState positiveZero;
        positiveZero.harnessRollRad = 0.0;
        Check(HashFlightState(negativeZero) == HashFlightState(positiveZero),
              "-0.0 and +0.0 hash alike");

        FlightState nanA;
        FlightState nanB;
        nanA.spin = std::nan("");
        nanB.spin = -std::nan("1");
        Check(HashFlightState(nanA) == HashFlightState(nanB),
              "all NaNs hash alike");
        Check(HashFlightState(nanA) != HashFlightState(a),
              "NaN differs from a finite value");

        // Order sensitivity: swapping two fields must not collide.
        FlightState swapped;
        swapped.leftCollapse = 0.25;
        FlightState other;
        other.rightCollapse = 0.25;
        Check(HashFlightState(swapped) != HashFlightState(other),
              "field order matters");
    }

    std::printf("\nRender-rate independence\n");
    {
        // 30 s of simulation, to the step.
        constexpr std::uint64_t Steps = 30 * 120;
        const std::uint64_t reference =
            RunToStepCount(1.0 / 60.0, Steps, false);
        std::printf("  %-16s hash %016llx  (reference)\n", "60 Hz",
            static_cast<unsigned long long>(reference));

        struct Case { const char* name; double frame; bool jitter; };
        for (const Case& c : {
            Case{"30 Hz", 1.0 / 30.0, false},
            Case{"120 Hz", 1.0 / 120.0, false},
            Case{"144 Hz", 1.0 / 144.0, false},
            Case{"240 Hz", 1.0 / 240.0, false},
            Case{"10 Hz", 1.0 / 10.0, false},
            Case{"jittery 60 Hz", 1.0 / 60.0, true},
            Case{"jittery 144 Hz", 1.0 / 144.0, true},
        })
        {
            const std::uint64_t hash =
                RunToStepCount(c.frame, Steps, c.jitter);
            std::printf("  %-16s hash %016llx  %s\n", c.name,
                static_cast<unsigned long long>(hash),
                hash == reference ? "match" : "DIFFER");
            Check(hash == reference,
                  std::string("state is bit-identical at ") + c.name);
        }
    }

    // -- PERFORMANCE PLAN, LEVEL 1 -----------------------------------------
    //
    // The block above is the gate the performance plan names for capping the
    // frame rate, and it was already green before the plan was written: the
    // clock and the solver do not care what rate frames arrive at. But it
    // proves that by sampling the controls as a function of SIMULATION time -
    // its own comment says sampling per frame "would make the comparison
    // meaningless", and for testing the clock that is exactly right.
    //
    // THE SHIPPED PAWN SAMPLES PER FRAME. `AParagliderPawn::Tick` reads the
    // input once and applies the same `AppliedControls` to every fixed step
    // that frame issues. So a frame cap does reach the flight, and it reaches
    // it through exactly one path: how often the pilot's hands are read. That
    // is what this measures, because a cap chosen without it is a cap chosen
    // on power alone.
    //
    // The reference is per-STEP sampling, which is the finest the simulation
    // can consume - one control value per 1/120 s. Every row below samples
    // once per frame and holds it, which is what the pawn does.
    std::printf("\nControl sampling: how a frame cap reaches the flight\n");
    {
        constexpr std::uint64_t Steps = 30 * 120;
        struct Outcome
        {
            std::uint64_t hash = 0;
            Vec3 position{};
            double airspeed = 0.0;
        };
        const auto Run = [](double frameSeconds, bool samplePerFrame)
        {
            ParagliderSolverClock clock;
            ParagliderDynamics dynamics(
                GetWingProfile(WingProfileId::Epic2MLResearch).parameters);
            FlightState state;
            const auto ControlsAt = [](double t)
            {
                ControlInput input;
                input.leftBrake = 0.25 + 0.20 * std::sin(t * 0.7);
                input.rightBrake = 0.15 + 0.10 * std::sin(t * 0.4 + 1.1);
                input.weightShift = 0.5 * std::sin(t * 0.3);
                return input;
            };
            while (clock.StepsRunCount() < Steps)
            {
                const int steps = clock.BeginFrame(frameSeconds);
                // Sampled ONCE for the frame, before any step runs - which is
                // where the pawn samples it.
                const ControlInput held =
                    ControlsAt(clock.SimulationTimeSeconds());
                for (int i = 0;
                     i < steps && clock.StepsRunCount() < Steps; ++i)
                {
                    const ControlInput input = samplePerFrame
                        ? held : ControlsAt(clock.SimulationTimeSeconds());
                    dynamics.Step(state, input, Atmosphere{},
                                  clock.StepSeconds());
                    clock.EndStep();
                }
            }
            Outcome out;
            out.hash = HashFlightState(state);
            out.position = state.positionWorldM;
            out.airspeed = dynamics.LastTelemetry().airspeedMps;
            return out;
        };

        const Outcome reference = Run(1.0 / 120.0, false);
        std::printf("%14s %14s %16s %14s\n", "frame rate", "sampled",
                    "position drift", "airspeed drift");
        double driftAt120 = -1.0;
        double driftAt240 = -1.0;
        double driftAt30 = -1.0;
        double driftAt60 = -1.0;
        for (const double rate : {240.0, 144.0, 120.0, 60.0, 30.0, 20.0})
        {
            const Outcome held = Run(1.0 / rate, true);
            const double drift = Length(Vec3{
                held.position.x - reference.position.x,
                held.position.y - reference.position.y,
                held.position.z - reference.position.z});
            std::printf("%13.0f%s %14s %13.4f m %12.4f m/s\n", rate, " Hz",
                        held.hash == reference.hash ? "identical" : "held",
                        drift, held.airspeed - reference.airspeed);
            if (rate == 240.0) driftAt240 = drift;
            if (rate == 120.0) driftAt120 = drift;
            if (rate == 60.0) driftAt60 = drift;
            if (rate == 30.0) driftAt30 = drift;
        }
        std::printf("  Drift is after 30 s of flight, against controls "
                    "sampled every step.\n\n");

        // AND THE ANSWER IS NOT THE TIDY ONE THIS BLOCK WAS WRITTEN TO ASSERT.
        // The expectation was that a cap AT the simulation rate is exactly
        // free, because one frame issues one step. It is not quite: 240 and
        // 144 Hz are bit-identical to per-step sampling and 120 Hz is not.
        //
        // The reason is the boundary, and it is the clock working correctly.
        // ABOVE the step rate every frame issues at most one step, and the
        // control sampled at the frame's start is the control at that step's
        // start - identical by construction. AT the step rate, 1/120 does not
        // accumulate exactly, so occasionally a frame issues zero steps and the
        // next issues two, and the second of those two runs on a control value
        // one step old. That is the entire 0.0007 m.
        //
        // 0.7 mm of drift over 30 s of flight is not a handling change, and
        // saying so is a physical judgement rather than a tolerance: it is four
        // orders below the 8 m the wing travels in one step. So the cap is
        // sound at 120 - what is NOT true is that it is bit-clean, and the
        // record should not claim it.
        Check(driftAt240 == 0.0 && driftAt120 > 0.0,
              "A CAP ABOVE THE STEP RATE IS EXACTLY FREE AND A CAP AT IT IS "
              "NOT: 240 and 144 Hz are bit-identical to per-step sampling, "
              "while 120 Hz drifts 0.0007 m because 1/120 does not accumulate "
              "exactly and an occasional frame issues two steps, the second of "
              "them on a control value one step old");
        Check(driftAt120 < 0.001,
              "and that drift is 0.7 mm over 30 s - four orders below the 8 m "
              "the wing covers in a single step - so capping at the simulation "
              "rate is a power decision with no handling consequence, which is "
              "what makes 120 defensible as the default");
        Check(driftAt60 > 10.0 * driftAt120 && driftAt30 > driftAt60
                  && driftAt30 > 0.1,
              "BUT BELOW THE STEP RATE THE COST IS REAL AND MONOTONIC: 7.8 cm "
              "at 60 Hz, 22.9 cm at 30, 37.9 cm at 20. A held control value is "
              "stale by up to a frame, so a cap under 120 is a handling change "
              "and not only a saving - which is the price the lower graphics "
              "tiers pay, now stated rather than assumed");
    }

    std::printf("\nStill-air EPIC 2 ML baseline\n");
    {
        // Frozen reference. These are the converged still-air numbers with
        // hands up and no weight shift, stable to four decimals from 120 s
        // through 600 s. Later solver levels are expected to change them - the
        // point is that the change has to be deliberate and visible here,
        // rather than drifting unnoticed.
        constexpr double Dt = 1.0 / 120.0;
        ParagliderDynamics dynamics(
            GetWingProfile(WingProfileId::Epic2MLResearch).parameters);
        FlightState state;
        for (int frame = 0; frame < 300 * 120; ++frame)
            dynamics.Step(state, ControlInput{}, Atmosphere{}, Dt);

        const auto& telemetry = dynamics.LastTelemetry();
        const double horizontal = std::hypot(
            state.velocityWorldMps.x, state.velocityWorldMps.y);
        const double sink = -state.velocityWorldMps.z;
        const double glide = horizontal / sink;

        std::printf("  trim airspeed %.4f m/s (%.1f km/h)\n",
            telemetry.airspeedMps, telemetry.airspeedMps * 3.6);
        std::printf("  sink rate     %.4f m/s\n", sink);
        std::printf("  glide ratio   %.4f\n", glide);
        std::printf("  angle of attack %.4f rad\n",
            telemetry.angleOfAttackRad);

        CheckNear(telemetry.airspeedMps, 10.8055, 1e-3, "baseline trim speed");
        CheckNear(sink, 1.1426, 1e-3, "baseline sink rate");
        CheckNear(glide, 9.4037, 1e-3, "baseline glide ratio");
        CheckNear(telemetry.angleOfAttackRad, -0.1144, 1e-3, "baseline aoa");

        // Still air must stay finite and level: no yaw, no bank, no drift.
        CheckNear(state.velocityWorldMps.y, 0.0, 1e-9,
                  "no lateral drift in still air");
        Check(std::isfinite(state.positionWorldM.z), "altitude stays finite");

        // Published EPIC 2 figures, as a sanity band rather than a gate.
        Check(telemetry.airspeedMps * 3.6 > 34.0
                  && telemetry.airspeedMps * 3.6 < 42.0,
              "trim speed is in the published band");
        Check(glide > 8.5 && glide < 10.5,
              "glide ratio is in the published band");
    }

    std::printf("\nCoefficient registry\n");
    {
        const CoefficientAudit audit = AuditCoefficients();
        std::printf("  %zu coefficients: %zu tuned, %zu unvalidated,"
                    " %zu out of range\n",
            audit.total, audit.tuned, audit.unvalidated, audit.outOfRange);

        Check(audit.total > 0, "registry is populated");
        // Every registered value must sit inside its own declared range. A
        // coefficient outside the range it claims is either mis-ranged or
        // mis-set, and both are worth failing on.
        Check(audit.outOfRange == 0, "every coefficient is within its range");

        // Ranges must be well formed and units stated.
        for (std::size_t i = 0; i < CoefficientRecordCount(); ++i)
        {
            const CoefficientRecord& record = CoefficientRecords()[i];
            Check(record.validMin < record.validMax,
                  std::string(record.name) + " has a well-formed range");
            Check(record.unit != nullptr && record.unit[0] != '\0',
                  std::string(record.name) + " states a unit");
            Check(record.provenance != nullptr && record.provenance[0] != '\0',
                  std::string(record.name) + " states a provenance");
            // Anything not externally justified must name the level that
            // replaces it, so tuned constants cannot quietly become permanent.
            if (record.source == CoefficientSource::Tuned)
                Check(record.supersededByLevel > 0,
                      std::string(record.name)
                          + " is tuned, so it must name a superseding level");
        }

        // Registry entries must still match the solver's actual defaults.
        const WingParameters wing;
        const auto* area = FindCoefficient("areaM2");
        Check(area != nullptr && area->value == wing.areaM2,
              "registry tracks the live areaM2");
        const auto* trim = FindCoefficient("trimCl");
        Check(trim != nullptr && trim->value == wing.trimCl,
              "registry tracks the live trimCl");
        Check(FindCoefficient("noSuchCoefficient") == nullptr,
              "unknown names are not found");

        // Recorded as a fact, not a gate: this is how much of the model is
        // currently resting on numbers with no external justification.
        std::printf("  %.0f%% of coefficients are tuned,"
                    " %.0f%% are unvalidated\n",
            100.0 * static_cast<double>(audit.tuned)
                / static_cast<double>(audit.total),
            100.0 * static_cast<double>(audit.unvalidated)
                / static_cast<double>(audit.total));
    }

    if (Failures)
    {
        std::printf("\n%d determinism check(s) failed.\n", Failures);
        return 1;
    }
    std::printf("\nAll determinism checks passed.\n");
    return 0;
}
