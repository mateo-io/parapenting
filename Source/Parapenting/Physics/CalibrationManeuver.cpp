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
        // 25% of handle travel, which after the 19% of sewn-in slack is 7% of
        // brake at the trailing edge. It used to be 40%, and 40% still does
        // not identify anything - but the reason has changed and the old one
        // is worth keeping because it is no longer true.
        //
        // It used to be lift: the analytic section polars peaked at CL 0.866
        // at 11 degrees where this wing's own profile carries 1.32, so trim
        // left barely six degrees of brake before the wing was past its own
        // stall, with no steady state to come back to (limitation 6). The
        // computed polars closed that - the section carries 2.36 at 40% brake
        // and there is a steady state there.
        //
        // What stops 40% now is pitch, not lift: the section's nose-down
        // moment under brake rotates the canopy on its lines faster than the
        // camber buys lift back, so the wing accelerates into the first fifth
        // of the travel and departs past a quarter of it. PHYSICS_TODO item
        // 11.
        //
        // So the step was moved to where the model can hold it, and the
        // manoeuvre that cannot is kept and reported: see the deep-brake
        // finding in `calibration_tests`. Reducing the input until the test
        // passes would be tuning; reducing it to where the aircraft still
        // flies, and stating loudly what it cannot do, is identification.
        controls.leftBrake = 0.25;
        controls.rightBrake = 0.25;
        break;
    case CalibrationManeuver::BrakePulse:
        // Two seconds in, then hands up. The release is the identification -
        // what follows is a free oscillation with no input in it. 30% for the
        // same reason the step is 25%: deep enough to swing the pilot well
        // clear of trim, shallow enough that the wing is still flying when it
        // is released. 45% was not - it stalled on the way in.
        if (t < 2.0)
        {
            controls.leftBrake = 0.30;
            controls.rightBrake = 0.30;
        }
        break;
    case CalibrationManeuver::DeepBrakeStep:
        // The one that does not work, run deliberately and reported. 40% of
        // travel, which a real EN-B wing flies all day.
        controls.leftBrake = 0.40;
        controls.rightBrake = 0.40;
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
    double toTimeS, double& periodS, double& dampingRatio, int& oscillations)
{
    periodS = 0.0;
    dampingRatio = 0.0;
    oscillations = 0;
    if (samples.size() < 8) return;

    // The value it is settling toward. Taken as the mean over the
    // identification window itself rather than over the tail of the record:
    // the tail is forty seconds later and the slow mode has moved the swing
    // angle by more than the surge being measured, so a tail mean puts the
    // zero line outside the oscillation and no crossings are found at all.
    double settledValue = 0.0;
    int settledCount = 0;
    for (const ManeuverSample& sample : samples)
    {
        if (sample.timeS < fromTimeS || sample.timeS > toTimeS) continue;
        settledValue += sample.payloadSwingRad;
        ++settledCount;
    }
    if (settledCount == 0) return;
    settledValue /= static_cast<double>(settledCount);

    std::vector<double> crossingTimes;
    std::vector<double> peakTimes;
    std::vector<double> peakValues;
    double previous = 0.0;
    bool havePrevious = false;
    for (std::size_t i = 1; i + 1 < samples.size(); ++i)
    {
        if (samples[i].timeS < fromTimeS || samples[i].timeS > toTimeS)
            continue;
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
    case CalibrationManeuver::BrakeStep: return "brake step 25%";
    case CalibrationManeuver::DeepBrakeStep: return "deep brake step 40%";
    case CalibrationManeuver::BrakePulse: return "brake pulse and release";
    case CalibrationManeuver::WeightShiftStep: return "weight shift step";
    case CalibrationManeuver::CoordinatedTurn: return "coordinated turn 35%";
    case CalibrationManeuver::StallApproach: return "stall approach";
    }
    return "unknown";
}

PayloadMassProperties CalibrationPayload(
    double allUpMassKg, double canopyMassKg)
{
    // Ballast, not a bigger pilot: the harness geometry and the payload's own
    // inertia belong to the pilot who is flying, and changing those would
    // change weight shift and the harness pendulum as well as the wing
    // loading. Ballast sits near the payload's centre of mass and does one
    // thing, which is what an identification parameter has to do.
    PayloadMassProperties payload;
    payload.ballastKg = std::max(
        0.0, allUpMassKg - canopyMassKg - payload.TotalKg());
    return payload;
}

ManeuverResult RunCalibrationManeuver(
    CalibrationManeuver maneuver, const CanopyGeometry& geometry,
    const LinePlanSpec& linePlan, const CalibrationSettings& settings)
{
    ManeuverResult result;
    result.maneuver = maneuver;

    CoupledParagliderSolver solver(
        geometry, linePlan, CoupledSchedule{},
        CalibrationPayload(settings.allUpMassKg));
    CoupledState state;
    const CoupledAtmosphere stillAir;
    const double dt = solver.Schedule().timeStepS;

    // Settle. Every manoeuvre starts from the same place, hands up, or the
    // response carries the initial condition in it.
    if (settings.settleToCriterion)
    {
        // Fly windows until one of them stands still. Reported either way:
        // "did not settle in 1500 s" is a result, and a far more useful one
        // than a number averaged out of an oscillation.
        const int windowSteps = std::max(1, static_cast<int>(
            settings.settleWindowSeconds / dt));
        double elapsed = 0.0;
        while (elapsed < settings.maximumSettleSeconds)
        {
            double lowAlpha = 1.0e9, highAlpha = -1.0e9;
            double lowSpeed = 1.0e9, highSpeed = -1.0e9;
            for (int step = 0; step < windowSteps; ++step)
            {
                solver.Step(state, CoupledControls{}, stillAir);
                const CoupledDiagnostics& d = solver.Diagnostics();
                lowAlpha = std::min(lowAlpha, d.angleOfAttackRad);
                highAlpha = std::max(highAlpha, d.angleOfAttackRad);
                lowSpeed = std::min(lowSpeed, d.airspeedMps);
                highSpeed = std::max(highSpeed, d.airspeedMps);
            }
            elapsed += settings.settleWindowSeconds;
            if (highAlpha - lowAlpha < settings.settleIncidenceToleranceRad
                && highSpeed - lowSpeed < settings.settleAirspeedToleranceMps)
            {
                result.preInputSettled = true;
                break;
            }
        }
        result.actualSettleSeconds = elapsed;
    }
    else
    {
        const int settleSteps = static_cast<int>(settings.settleSeconds / dt);
        for (int step = 0; step < settleSteps; ++step)
            solver.Step(state, CoupledControls{}, stillAir);
    }

    // The record phase needs the same treatment as the settle. A step input on
    // an aircraft whose slow mode is 16.4 s at damping 0.031 is not over in
    // forty-five seconds either, and the settled values below are averaged out
    // of the tail of this window - so with a fixed record they are averages of
    // an oscillation no matter how long the pre-input settle was.
    const bool criterionRecord = settings.settleToCriterion;
    const int recordSteps = static_cast<int>(
        (criterionRecord ? settings.maximumSettleSeconds
                         : settings.recordSeconds) / dt);
    // Do not start testing for a settled state until the input has been in for
    // a while, or a manoeuvre whose ramp has not begun reads as settled at trim
    // and stops before it has done anything.
    const double earliestFinishS =
        std::max(30.0, settings.settleWindowSeconds * 3.0);
    const int criterionWindowSteps = std::max(1, static_cast<int>(
        settings.settleWindowSeconds / dt));
    double windowLowAlpha = 1.0e9, windowHighAlpha = -1.0e9;
    double windowLowSpeed = 1.0e9, windowHighSpeed = -1.0e9;
    int recordedSteps = 0;
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

        ++recordedSteps;
        if (criterionRecord)
        {
            windowLowAlpha = std::min(windowLowAlpha, d.angleOfAttackRad);
            windowHighAlpha = std::max(windowHighAlpha, d.angleOfAttackRad);
            windowLowSpeed = std::min(windowLowSpeed, d.airspeedMps);
            windowHighSpeed = std::max(windowHighSpeed, d.airspeedMps);
            if (recordedSteps % criterionWindowSteps == 0)
            {
                const bool stopped =
                    windowHighAlpha - windowLowAlpha
                        < settings.settleIncidenceToleranceRad
                    && windowHighSpeed - windowLowSpeed
                        < settings.settleAirspeedToleranceMps;
                windowLowAlpha = 1.0e9; windowHighAlpha = -1.0e9;
                windowLowSpeed = 1.0e9; windowHighSpeed = -1.0e9;
                if (stopped && t > earliestFinishS)
                {
                    // Record the last window's worth of samples as the settled
                    // state, then stop. Everything after this is the same
                    // number repeated.
                    result.settled = true;
                    break;
                }
            }
        }

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
    const double averagingWindowS = settings.settleToCriterion
        ? settings.settleWindowSeconds : 2.0;
    for (const ManeuverSample& sample : result.samples)
    {
        // Two seconds normally; the full settling window in criterion mode,
        // because two seconds inside a 16 s period is a chord of the
        // oscillation rather than a measurement of it.
        if (sample.timeS < endTime - averagingWindowS) continue;
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
        const bool heldOverWindow =
            (airspeedMax - airspeedMin)
                < 0.01 * std::max(1.0, result.settledAirspeedMps)
            && (turnMax - turnMin) < 0.01;
        // In criterion mode the loop above already proved the tighter test and
        // set this; do not let the looser one downgrade it, and do not let it
        // UPGRADE a run that ran out of time either.
        result.settled = settings.settleToCriterion
            ? (result.settled && heldOverWindow) : heldOverWindow;
    }

    // Pitch identification. Only the manoeuvres that release an input leave a
    // free oscillation to measure; for the rest this stays zero.
    if (maneuver == CalibrationManeuver::BrakePulse)
    {
        // From the release to twelve seconds after it, and the window matters.
        // This aircraft has TWO pitch modes and they are an order of magnitude
        // apart: the wing swinging against the pilot on its lines, which is
        // the surge a pilot sees and which runs at about four and a half
        // seconds, and a slow speed-and-incidence mode near twenty. Given the
        // whole forty-five second record the identifier locked onto the slow
        // one and reported a 20.4 s "pendulum" with a damping ratio of 0.05,
        // which is a true statement about the wrong mode.
        //
        // Seven seconds is four periods of the fast mode and a third of one of
        // the slow, so the crossings it counts belong to the surge.
        IdentifyOscillation(result.samples, 2.0, 9.0, result.pitchPeriodS,
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
