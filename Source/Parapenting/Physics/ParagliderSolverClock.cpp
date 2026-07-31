#include "ParagliderSolverClock.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
ParagliderSolverClock::ParagliderSolverClock(
    double stepSeconds, double maxFrameSeconds)
    : StepSecondsValue(stepSeconds > 0.0 ? stepSeconds : DefaultStepSeconds)
    , MaxFrameSecondsValue(
          maxFrameSeconds > 0.0 ? maxFrameSeconds : DefaultMaxFrameSeconds)
{
}

int ParagliderSolverClock::BeginFrame(double frameDeltaSeconds)
{
    // Reject non-finite and negative deltas rather than letting them poison
    // the accumulator. A NaN delta would make every later comparison false and
    // silently stop the simulation.
    if (!std::isfinite(frameDeltaSeconds) || frameDeltaSeconds <= 0.0)
        return 0;

    if (frameDeltaSeconds > MaxFrameSecondsValue)
    {
        frameDeltaSeconds = MaxFrameSecondsValue;
        ++ClampedFrames;
    }

    DeliveredSeconds += frameDeltaSeconds;

    // Step count is derived from total delivered time, not by subtracting a
    // step length from a residual accumulator. Subtracting leaves a remainder
    // that is re-rounded every frame, so the count slowly diverges from
    // floor(total / step) and two clocks fed the same total in different chunk
    // sizes disagree. Deriving it makes the count an exact function of the
    // total, which is the property replay comparison needs.
    const double totalDue = std::floor(DeliveredSeconds / StepSecondsValue);
    const auto due = static_cast<std::uint64_t>(std::max(0.0, totalDue));
    if (due <= StepsIssued) return 0;

    const std::uint64_t pending = due - StepsIssued;
    StepsIssued = due;
    return static_cast<int>(pending);
}

void ParagliderSolverClock::EndStep()
{
    ++StepsRun;
}

double ParagliderSolverClock::InterpolationAlpha() const
{
    const double position = DeliveredSeconds / StepSecondsValue;
    return std::clamp(position - std::floor(position), 0.0, 1.0);
}

void ParagliderSolverClock::Reset()
{
    DeliveredSeconds = 0.0;
    StepsIssued = 0;
    StepsRun = 0;
    ClampedFrames = 0;
}
}
