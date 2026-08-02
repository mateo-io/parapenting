#pragma once

#include "CanopyGeometry.h"
#include "CoupledParagliderSolver.h"

#include <string>
#include <vector>

namespace Parapenting::Physics
{
// Level 9 of the master plan: stop asking whether the model runs and start
// asking what it says.
//
// Levels 1-8 each proved their own internal consistency - forces close,
// energy closes, symmetric cases stay symmetric, a collapse comes from a
// pressure balance. None of that says the wing flies like the wing. This
// level runs repeatable still-air manoeuvres, exports what the aircraft did,
// and compares the numbers with published data and with physics that has a
// closed form.
//
// It is deliberately built on the COUPLED solver rather than the legacy
// model. `ResearchManeuver` already does this for `ParagliderDynamics`, and
// its numbers describe a fitted polar rather than the geometry-driven stack.
//
// Two rules from the plan govern everything here:
//
//   * Fit only bounded, identified parameters, and record every fit in the
//     coefficient registry with its residual (guiding rule 9).
//   * Document known disagreements rather than tuning them invisibly. A
//     manoeuvre that disagrees with the published wing is a finding; a
//     manoeuvre that has been adjusted until it agrees is nothing at all.
//
// The manoeuvres are system identification, not flying: each one holds still
// air, starts from settled trim, applies ONE input, and records the response.
// A response with two inputs in it identifies nothing.

enum class CalibrationManeuver
{
    // Hands up, nothing applied. Trim speed, sink, glide and incidence.
    HandsUpTrim,
    // Full accelerator, held. The other end of the speed system.
    AcceleratorStep,
    // Symmetric brake to 40%, held. Speed and sink at a known brake setting.
    BrakeStep,
    // Symmetric brake to 70% for 2 s, then released. This is the pitch
    // identification manoeuvre: the wing swings back, then surges, and the
    // period and damping of that oscillation are what the pendulum and the
    // line stiffness together produce.
    BrakePulse,
    // Weight shift hard right, held. Roll and turn response with no brake.
    WeightShiftStep,
    // Right brake to 35%, held. Turn rate, bank and the time to reach them.
    CoordinatedTurn,
    // Symmetric brake ramped in slowly to 90%. Where separation begins and
    // what the wing does as it arrives.
    StallApproach,
};

const char* CalibrationManeuverName(CalibrationManeuver maneuver);

// One row of the time series. Everything the plan asks to export - airspeed,
// sink, attitude, brake, line tension, pressure and energy - plus the two
// states that did not exist before the wing and the pilot became two bodies.
struct ManeuverSample
{
    double timeS = 0.0;
    double airspeedMps = 0.0;
    double sinkMps = 0.0;
    double glideRatio = 0.0;
    double angleOfAttackRad = 0.0;
    double bankRad = 0.0;
    double turnRateRadps = 0.0;
    // The pendulum between wing and pilot.
    double payloadSwingRad = 0.0;
    double canopyLeadM = 0.0;
    // Inputs, so a row is self-describing and a CSV can be read without the
    // manoeuvre definition beside it.
    double leftBrake = 0.0;
    double rightBrake = 0.0;
    double weightShift = 0.0;
    double accelerator = 0.0;
    // Subsystem state.
    double meanPressureCoefficient = 0.0;
    double worstCollapse = 0.0;
    double leftCarabinerLoadN = 0.0;
    double rightCarabinerLoadN = 0.0;
    double energyResidualW = 0.0;
    double altitudeM = 0.0;
};

// What a manoeuvre identified. Every field is measured off the time series
// rather than read from the solver's own state, so a number here is something
// an instrumented flight could also produce.
struct ManeuverResult
{
    CalibrationManeuver maneuver = CalibrationManeuver::HandsUpTrim;
    std::vector<ManeuverSample> samples;

    // Settled values, averaged over the last two seconds. "Settled" is
    // checked rather than assumed - see `settled` below.
    double settledAirspeedMps = 0.0;
    double settledSinkMps = 0.0;
    double settledGlideRatio = 0.0;
    double settledIncidenceRad = 0.0;
    double settledBankRad = 0.0;
    double settledTurnRateRadps = 0.0;
    // True when the last two seconds held airspeed to within 1% and the turn
    // rate to within 0.01 rad/s. A manoeuvre that has not settled cannot
    // identify a steady-state number, and saying so is the point.
    bool settled = false;

    // Pitch identification, from the swing angle's free oscillation after the
    // input is released. Zero when the manoeuvre does not excite one.
    double pitchPeriodS = 0.0;
    double pitchDampingRatio = 0.0;
    int pitchOscillationsMeasured = 0;

    // Transient extremes over the whole run.
    double peakAirspeedMps = 0.0;
    double minimumAirspeedMps = 0.0;
    double peakCanopyLeadM = 0.0;
    double leastCanopyLeadM = 0.0;
    double peakAbsTurnRateRadps = 0.0;
    double peakWorstCollapse = 0.0;
    double worstEnergyResidualW = 0.0;
    // Set if the numerical safety envelope engaged at any point. A calibration
    // number from a run where it did engage means nothing (guiding rule 12).
    bool safetyEnvelopeEngaged = false;
};

struct CalibrationSettings
{
    // Seconds of settling before the input is applied. The wing must be at
    // trim, or the response contains the initial transient as well.
    double settleSeconds = 15.0;
    // Seconds of recording after it.
    double recordSeconds = 20.0;
    // Rows per second in the exported series. The solver still runs at 120 Hz;
    // this only decimates the output.
    double sampleRateHz = 20.0;
};

ManeuverResult RunCalibrationManeuver(
    CalibrationManeuver maneuver, const CanopyGeometry& geometry,
    const LinePlanSpec& linePlan, const CalibrationSettings& settings = {});

// The time series as CSV, one row per sample, with a header naming units.
// Returns false if the file could not be opened.
bool WriteManeuverCsv(const ManeuverResult& result, const std::string& path);

// A one-line summary per manoeuvre, for the calibration report.
std::string ManeuverSummaryLine(const ManeuverResult& result);
}
