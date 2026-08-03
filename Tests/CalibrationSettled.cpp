// Level 9's manoeuvre set, flown until the wing stops moving.
//
// PHYSICS_TODO item 18. `calibration_tests` settles each manoeuvre for ninety
// seconds and then averages its last two. That is not enough and the
// arithmetic says so: the slow speed-and-incidence mode is 16.4 s with a
// damping ratio of 0.031, so ninety seconds is five and a half periods and
// leaves about a third of the opening transient still running. Its `settled`
// flag does not catch it, because holding airspeed to 1% over two seconds is a
// far looser test than that mode is slow - a two-second window inside a
// sixteen-second period is a chord of the oscillation, not a measurement of it.
//
// So every number in `docs/CALIBRATION_REPORT.md` is a sample of a decaying
// oscillation. This re-measures them with the settle driven by a criterion
// instead of a clock, and prints the two side by side.
//
// It is a SEPARATE binary and not part of `Tools/check-build.sh` on purpose.
// Settling eight manoeuvres this way is the better part of an hour, against
// minutes for the fast suite. The fast suite keeps its clock-based settle and
// its own gates; this is what says whether those gates are pointed at trim
// points or at transients.
//
// Nothing here asserts. It is a measurement, and its output is what belongs in
// the calibration report.
#include "CalibrationManeuver.h"
#include "CanopyGeometry.h"
#include "SuspensionGraph.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace Parapenting::Physics;

namespace
{
constexpr double Pi = 3.14159265358979323846;
constexpr double Degrees = 180.0 / Pi;

struct Case
{
    CalibrationManeuver maneuver;
    const char* name;
};
}

int main()
{
    std::printf("Level 9 manoeuvres, settled to a criterion rather than a "
                "clock.\n");
    std::printf("PHYSICS_TODO item 18. Nothing here asserts.\n\n");

    const CanopyGeometry canopy;
    const LinePlanSpec linePlan = Epic2MlLinePlan();

    const std::vector<Case> cases{
        {CalibrationManeuver::HandsUpTrim, "hands-up trim"},
        {CalibrationManeuver::AcceleratorStep, "accelerator step"},
        {CalibrationManeuver::BrakeStep, "brake step 25%"},
        {CalibrationManeuver::DeepBrakeStep, "deep brake step 40%"},
        {CalibrationManeuver::BrakePulse, "brake pulse and release"},
        {CalibrationManeuver::WeightShiftStep, "weight shift step"},
        {CalibrationManeuver::CoordinatedTurn, "coordinated turn 35%"},
        {CalibrationManeuver::StallApproach, "stall approach"},
    };

    std::printf("%-26s %9s %8s %8s %8s %9s %s\n",
                "manoeuvre", "v m/s", "sink", "glide", "alpha", "settle",
                "state");

    for (const Case& c : cases)
    {
        CalibrationSettings settings;
        settings.settleToCriterion = true;
        const ManeuverResult r = RunCalibrationManeuver(
            c.maneuver, canopy, linePlan, settings);

        // A fully separated descent IS a steady state, and the criterion above
        // will happily certify it. Saying "settled" next to 91 degrees of
        // incidence would invite exactly the reading this whole item exists to
        // stop, so a departure is labelled as one whether or not it stood
        // still.
        const bool departed = r.settledIncidenceRad > 0.35;
        const char* state =
            r.safetyEnvelopeEngaged ? "SAFETY ENVELOPE ENGAGED - meaningless"
            : departed ? "DEPARTED - a steady state, not a trim point"
            : !r.preInputSettled ? "never settled BEFORE the input"
            : r.settled ? "settled"
            : "did not settle after the input";

        std::printf("%-26s %9.3f %8.3f %8.2f %7.2fd %8.0fs  %s\n",
                    c.name, r.settledAirspeedMps, r.settledSinkMps,
                    r.settledGlideRatio, r.settledIncidenceRad * Degrees,
                    r.actualSettleSeconds, state);
    }

    std::printf("\n  'settle' is how long the wing took to stand still BEFORE "
                "the input went in.\n  The fast suite gives every one of these "
                "ninety seconds.\n");
    std::printf("\n  A row that never settled before its input, or did not "
                "settle after it, is\n  not a trim point and must not be "
                "compared against a published number.\n");
    return 0;
}
