#pragma once

#include <cstdint>

namespace Parapenting::Physics
{
// Fixed-step clock for the physics solver. Level 0 of the master plan makes
// determinism at a fixed 120 Hz mandatory, and requires the physics rate to be
// decoupled from the engine tick.
//
// The pawn already ran an accumulator inline. Pulling it out here buys three
// things it did not have:
//
//   * simulation time is stepCount * stepSeconds, not a running sum. Summing
//     the step repeatedly accumulates rounding, so two runs that took the same
//     number of steps could report different times and diverge in anything
//     time-dependent. Multiplying cannot drift.
//   * step count is floor(total delivered time / step), derived rather than
//     produced by subtracting a step from a residual each frame. The
//     subtractive form re-rounds its remainder every frame, so the count
//     drifts from the total and two clocks fed the same time in different
//     chunk sizes disagree about how much to simulate.
//   * the accumulator clamp is reported rather than silent. Clamping discards
//     real time to avoid a death spiral, which breaks replay equivalence, so a
//     replay must be able to see that it happened.
//   * it is testable without the engine.
//
// Usage per rendered frame:
//
//     const int steps = clock.BeginFrame(deltaSeconds);
//     for (int i = 0; i < steps; ++i)
//     {
//         Step(clock.StepSeconds());
//         clock.EndStep();
//     }
//     Render(clock.InterpolationAlpha());
class ParagliderSolverClock
{
public:
    static constexpr double DefaultStepSeconds = 1.0 / 120.0;
    // Above this, a frame is treated as a hitch: the excess is discarded
    // rather than simulated, so one long frame cannot queue hundreds of steps
    // and stall every frame after it.
    static constexpr double DefaultMaxFrameSeconds = 0.25;

    explicit ParagliderSolverClock(
        double stepSeconds = DefaultStepSeconds,
        double maxFrameSeconds = DefaultMaxFrameSeconds);

    // Adds a frame delta and returns how many fixed steps are now due.
    int BeginFrame(double frameDeltaSeconds);
    // Call once per step actually run.
    void EndStep();

    double StepSeconds() const { return StepSecondsValue; }
    // Exact: step count times step length, never a running sum.
    double SimulationTimeSeconds() const
    {
        return static_cast<double>(StepsRun) * StepSecondsValue;
    }
    std::uint64_t StepsRunCount() const { return StepsRun; }

    // Fraction of a step left unconsumed, in [0, 1). Render state should be
    // interpolated by this to avoid stutter at non-multiple frame rates.
    double InterpolationAlpha() const;

    // True if any frame has been clamped since the last Reset. A replay that
    // reports true is not required to match one that reports false.
    bool HasClamped() const { return ClampedFrames > 0; }
    std::uint64_t ClampedFrameCount() const { return ClampedFrames; }

    void Reset();

private:
    double StepSecondsValue;
    double MaxFrameSecondsValue;
    // Total time delivered through BeginFrame, after clamping. Step count is
    // derived from this rather than from a residual accumulator.
    double DeliveredSeconds = 0.0;
    std::uint64_t StepsIssued = 0;
    std::uint64_t StepsRun = 0;
    std::uint64_t ClampedFrames = 0;
};
}
