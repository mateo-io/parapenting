#include "CalibrationManeuver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace Parapenting::Physics
{
namespace
{
// The input schedule for each manoeuvre, as a function of time since the
// input began. Written as one function so a manoeuvre is a definition rather
// than a branch scattered through the runner.
CoupledControls ControlsAt(CalibrationManeuver maneuver, double t)
{
    CoupledControls controls;
    switch (maneuver)
    {
    case CalibrationManeuver::HandsUpTrim:
        break;
    case CalibrationManeuver::AcceleratorStep:
        controls.accelerator = 1.0;
        break;
    case CalibrationManeuver::BrakeStep:
        controls.leftBrake = 0.40;
        controls.rightBrake = 0.40;
        break;
    case CalibrationManeuver::BrakePulse:
        // Two seconds in, then hands up. The release is the identification -
        // what follows is a free oscillation with no input in it.
        if (t < 2.0)
        {
            controls.leftBrake = 0.70;
            controls.rightBrake = 0.70;
        }
        break;
    case CalibrationManeuver::WeightShiftStep:
        controls.weightShift = 1.0;
        break;
    case CalibrationManeuver::CoordinatedTurn:
        controls.rightBrake = 0.35;
        break;
    case CalibrationManeuver::StallApproach:
        // Ramped, not stepped. A step into deep brake is a dynamic stall and
        // identifies the entry rate rather than the stall.
        controls.leftBrake = std::clamp(t / 12.0, 0.0, 0.90);
        controls.rightBrake = controls.leftBrake;
        break;
    }
    return controls;
}

// Period and damping of a decaying oscillation, from its zero crossings and
// successive peaks. Measured rather than fitted: the period is twice the mean
// interval between crossings of the settled value, and the damping ratio comes
// from the logarithmic decrement of successive peaks on the same side.
void IdentifyOscillation(
    const std::vector<ManeuverSample>& samples, double fromTimeS,
    double& periodS, double& dampingRatio, int& oscillations)
{
    periodS = 0.0;
    dampingRatio = 0.0;
    oscillations = 0;
    if (samples.size() < 8) return;

    // The value it is settling toward: the mean over the last quarter of the
    // record, which is far enough from the input for the transient to be gone.
    const std::size_t tailStart = samples.size() - samples.size() / 4;
    double settledValue = 0.0;
    for (std::size_t i = tailStart; i < samples.size(); ++i)
        settledValue += samples[i].payloadSwingRad;
    settledValue /= static_cast<double>(samples.size() - tailStart);

    std::vector<double> crossingTimes;
    std::vector<double> peakTimes;
    std::vector<double> peakValues;
    double previous = 0.0;
    bool havePrevious = false;
    for (std::size_t i = 1; i + 1 < samples.size(); ++i)
    {
        if (samples[i].timeS < fromTimeS) continue;
        const double a = samples[i - 1].payloadSwingRad - settledValue;
        const double b = samples[i].payloadSwingRad - settledValue;
        const double c = samples[i + 1].payloadSwingRad - settledValue;
        // Zero crossing, linearly interpolated.
        if ((a < 0.0 && b >= 0.0) || (a > 0.0 && b <= 0.0))
        {
            const double span = b - a;
            const double fraction = std::fabs(span) > 1.0e-12
                ? -a / span : 0.0;
            crossingTimes.push_back(samples[i - 1].timeS + fraction
                * (samples[i].timeS - samples[i - 1].timeS));
        }
        // Local extremum.
        if ((b > a && b > c) || (b < a && b < c))
        {
            peakTimes.push_back(samples[i].timeS);
            peakValues.push_back(b);
        }
        previous = b;
        havePrevious = true;
    }
    (void)previous;
    (void)havePrevious;

    if (crossingTimes.size() >= 3)
    {
        // Half a period per crossing, so the period is twice the mean gap.
        const double span = crossingTimes.back() - crossingTimes.front();
        const double halfPeriods =
            static_cast<double>(crossingTimes.size() - 1);
        periodS = 2.0 * span / halfPeriods;
        oscillations = static_cast<int>(halfPeriods / 2.0);
    }

    // Logarithmic decrement, from successive peaks on the SAME side - peaks
    // two apart. Using adjacent peaks measures a half cycle and doubles the
    // apparent damping.
    double decrementSum = 0.0;
    int decrementCount = 0;
    for (std::size_t i = 0; i + 2 < peakValues.size(); ++i)
    {
        const double first = std::fabs(peakValues[i]);
        const double second = std::fabs(peakValues[i + 2]);
        if (first > 1.0e-6 && second > 1.0e-9 && first > second)
        {
            decrementSum += std::log(first / second);
            ++decrementCount;
        }
    }
    if (decrementCount > 0)
    {
        const double decrement =
            decrementSum / static_cast<double>(decrementCount);
        // zeta = delta / sqrt(4 pi^2 + delta^2)
        constexpr double Pi = 3.14159265358979323846;
        dampingRatio = decrement
            / std::sqrt(4.0 * Pi * Pi + decrement * decrement);
    }
}
}

const char* CalibrationManeuverName(CalibrationManeuver maneuver)
{
    switch (maneuver)
    {
    case CalibrationManeuver::HandsUpTrim: return "hands-up trim";
    case CalibrationManeuver::AcceleratorStep: return "accelerator step";
    case CalibrationManeuver::BrakeStep: return "brake step 40%";
    case CalibrationManeuver::BrakePulse: return "brake pulse and release";
    case CalibrationManeuver::WeightShiftStep: return "weight shift step";
    case CalibrationManeuver::CoordinatedTurn: return "coordinated turn 35%";
    case CalibrationManeuver::StallApproach: return "stall approach";
    }
    return "unknown";
}

ManeuverResult RunCalibrationManeuver(
    CalibrationManeuver maneuver, const CanopyGeometry& geometry,
    const LinePlanSpec& linePlan, const CalibrationSettings& settings)
{
    ManeuverResult result;
    result.maneuver = maneuver;

    CoupledParagliderSolver solver(geometry, linePlan);
    CoupledState state;
    const CoupledAtmosphere stillAir;
    const double dt = solver.Schedule().timeStepS;

    // Settle. Every manoeuvre starts from the same place, hands up, or the
    // response carries the initial condition in it.
    const int settleSteps = static_cast<int>(settings.settleSeconds / dt);
    for (int step = 0; step < settleSteps; ++step)
        solver.Step(state, CoupledControls{}, stillAir);

    const int recordSteps = static_cast<int>(settings.recordSeconds / dt);
    const int stride = std::max(1, static_cast<int>(
        1.0 / std::max(1.0e-6, settings.sampleRateHz * dt)));

    result.minimumAirspeedMps = 1.0e9;
    result.leastCanopyLeadM = 1.0e9;
    result.peakCanopyLeadM = -1.0e9;

    double previousAltitude = state.positionWorldM.z;
    double previousSampleTime = 0.0;
    for (int step = 0; step < recordSteps; ++step)
    {
        const double t = static_cast<double>(step) * dt;
        const CoupledControls controls = ControlsAt(maneuver, t);
        solver.Step(state, controls, stillAir);
        const CoupledDiagnostics& d = solver.Diagnostics();

        result.peakAirspeedMps =
            std::max(result.peakAirspeedMps, d.airspeedMps);
        result.minimumAirspeedMps =
            std::min(result.minimumAirspeedMps, d.airspeedMps);
        result.peakCanopyLeadM =
            std::max(result.peakCanopyLeadM, d.canopyLeadM);
        result.leastCanopyLeadM =
            std::min(result.leastCanopyLeadM, d.canopyLeadM);
        result.peakAbsTurnRateRadps = std::max(
            result.peakAbsTurnRateRadps, std::fabs(d.turnRateRadps));
        result.peakWorstCollapse = std::max(
            result.peakWorstCollapse, d.collapseState.worstCollapse);
        result.worstEnergyResidualW = std::max(
            result.worstEnergyResidualW, std::fabs(d.energyResidualW));
        if (d.aerodynamicsRejected) result.safetyEnvelopeEngaged = true;

        if (step % stride != 0) continue;

        ManeuverSample sample;
        sample.timeS = t;
        sample.airspeedMps = d.airspeedMps;
        sample.altitudeM = state.positionWorldM.z;
        // Sink from the altitude the aircraft actually lost between samples,
        // rather than from the velocity vector - it is what a vario reads.
        const double interval = t - previousSampleTime;
        sample.sinkMps = interval > 1.0e-9
            ? (previousAltitude - state.positionWorldM.z) / interval : 0.0;
        sample.glideRatio = sample.sinkMps > 1.0e-6
            ? std::sqrt(std::max(0.0,
                  d.airspeedMps * d.airspeedMps
                      - sample.sinkMps * sample.sinkMps)) / sample.sinkMps
            : 0.0;
        previousAltitude = state.positionWorldM.z;
        previousSampleTime = t;

        sample.angleOfAttackRad = d.angleOfAttackRad;
        sample.bankRad = d.bankRad;
        sample.turnRateRadps = d.turnRateRadps;
        sample.payloadSwingRad = d.payloadSwingRad;
        sample.canopyLeadM = d.canopyLeadM;
        sample.leftBrake = controls.leftBrake;
        sample.rightBrake = controls.rightBrake;
        sample.weightShift = controls.weightShift;
        sample.accelerator = controls.accelerator;
        sample.meanPressureCoefficient = d.meanPressureCoefficient;
        sample.worstCollapse = d.collapseState.worstCollapse;
        sample.leftCarabinerLoadN = d.leftCarabinerLoadN;
        sample.rightCarabinerLoadN = d.rightCarabinerLoadN;
        sample.energyResidualW = d.energyResidualW;
        result.samples.push_back(sample);
    }

    if (result.samples.size() < 4) return result;

    // Settled values, over the last two seconds.
    const double endTime = result.samples.back().timeS;
    double airspeedSum = 0.0;
    double sinkSum = 0.0;
    double incidenceSum = 0.0;
    double bankSum = 0.0;
    double turnSum = 0.0;
    double airspeedMin = 1.0e9;
    double airspeedMax = -1.0e9;
    double turnMin = 1.0e9;
    double turnMax = -1.0e9;
    int count = 0;
    for (const ManeuverSample& sample : result.samples)
    {
        if (sample.timeS < endTime - 2.0) continue;
        airspeedSum += sample.airspeedMps;
        sinkSum += sample.sinkMps;
        incidenceSum += sample.angleOfAttackRad;
        bankSum += sample.bankRad;
        turnSum += sample.turnRateRadps;
        airspeedMin = std::min(airspeedMin, sample.airspeedMps);
        airspeedMax = std::max(airspeedMax, sample.airspeedMps);
        turnMin = std::min(turnMin, sample.turnRateRadps);
        turnMax = std::max(turnMax, sample.turnRateRadps);
        ++count;
    }
    if (count > 0)
    {
        const double n = static_cast<double>(count);
        result.settledAirspeedMps = airspeedSum / n;
        result.settledSinkMps = sinkSum / n;
        result.settledIncidenceRad = incidenceSum / n;
        result.settledBankRad = bankSum / n;
        result.settledTurnRateRadps = turnSum / n;
        result.settledGlideRatio = result.settledSinkMps > 1.0e-6
            ? std::sqrt(std::max(0.0,
                  result.settledAirspeedMps * result.settledAirspeedMps
                      - result.settledSinkMps * result.settledSinkMps))
                  / result.settledSinkMps
            : 0.0;
        // Settled is a measurement, not an assumption: the last two seconds
        // must have held speed and turn rate.
        result.settled =
            (airspeedMax - airspeedMin)
                < 0.01 * std::max(1.0, result.settledAirspeedMps)
            && (turnMax - turnMin) < 0.01;
    }

    // Pitch identification. Only the manoeuvres that release an input leave a
    // free oscillation to measure; for the rest this stays zero.
    if (maneuver == CalibrationManeuver::BrakePulse)
    {
        IdentifyOscillation(result.samples, 2.0, result.pitchPeriodS,
                            result.pitchDampingRatio,
                            result.pitchOscillationsMeasured);
    }
    return result;
}

bool WriteManeuverCsv(const ManeuverResult& result, const std::string& path)
{
    std::ofstream file(path);
    if (!file) return false;
    file << "time_s,airspeed_mps,sink_mps,glide_ratio,alpha_rad,bank_rad,"
            "turn_rate_radps,payload_swing_rad,canopy_lead_m,left_brake,"
            "right_brake,weight_shift,accelerator,cell_pressure_cp,"
            "worst_collapse,left_carabiner_n,right_carabiner_n,"
            "energy_residual_w,altitude_m\n";
    file.setf(std::ios::fixed);
    file.precision(6);
    for (const ManeuverSample& s : result.samples)
    {
        file << s.timeS << ',' << s.airspeedMps << ',' << s.sinkMps << ','
             << s.glideRatio << ',' << s.angleOfAttackRad << ','
             << s.bankRad << ',' << s.turnRateRadps << ','
             << s.payloadSwingRad << ',' << s.canopyLeadM << ','
             << s.leftBrake << ',' << s.rightBrake << ',' << s.weightShift
             << ',' << s.accelerator << ',' << s.meanPressureCoefficient
             << ',' << s.worstCollapse << ',' << s.leftCarabinerLoadN << ','
             << s.rightCarabinerLoadN << ',' << s.energyResidualW << ','
             << s.altitudeM << '\n';
    }
    return true;
}

std::string ManeuverSummaryLine(const ManeuverResult& result)
{
    char buffer[320];
    std::snprintf(buffer, sizeof(buffer),
        "%-24s v %5.2f m/s (%5.1f km/h)  sink %4.2f  glide %5.2f  "
        "alpha %5.1f deg  bank %5.1f deg  turn %+6.3f rad/s  %s",
        CalibrationManeuverName(result.maneuver),
        result.settledAirspeedMps, result.settledAirspeedMps * 3.6,
        result.settledSinkMps, result.settledGlideRatio,
        result.settledIncidenceRad * 180.0 / 3.14159265358979,
        result.settledBankRad * 180.0 / 3.14159265358979,
        result.settledTurnRateRadps,
        result.settled ? "settled" : "NOT SETTLED");
    return std::string(buffer);
}
}
