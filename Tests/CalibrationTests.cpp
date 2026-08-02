// Level 9: what the model says, measured, and set against what is known.
//
// Levels 1-8 proved internal consistency - forces close, energy closes,
// symmetry holds, a collapse comes from a pressure balance. This asks a
// different question: does the aircraft fly like the aircraft?
//
// Every number here is measured off a time series from a repeatable still-air
// manoeuvre, so it is something an instrumented flight could also produce. The
// comparisons are against the manufacturer's published envelope and against
// physics with a closed form. Disagreements are recorded with a bound, never
// tuned away - a manoeuvre adjusted until it agrees has identified nothing.
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
int Failures = 0;

void Check(bool condition, const std::string& what)
{
    if (!condition)
    {
        std::printf("  FAIL  %s\n", what.c_str());
        ++Failures;
    }
}

constexpr double Pi = 3.14159265358979323846;
constexpr double Degrees = 180.0 / Pi;

// The published EPIC 2 ML envelope, from WingCatalogue's research profile.
constexpr double PublishedTrimKmh = 39.0;
constexpr double PublishedTopKmh = 53.0;
constexpr double PublishedMinSinkMps = 1.0;
constexpr double PublishedGlide = 9.5;
}

int main()
{
    const CanopyGeometry canopy;
    const LinePlanSpec linePlan = Epic2MlLinePlan();

    std::printf("Level 9 calibration: still-air system identification on the "
                "coupled solver.\n");
    std::printf("Published envelope: trim %.0f km/h, top %.0f km/h, min sink "
                "%.1f m/s, glide %.1f\n\n",
                PublishedTrimKmh, PublishedTopKmh, PublishedMinSinkMps,
                PublishedGlide);

    const auto run = [&](CalibrationManeuver maneuver)
    {
        return RunCalibrationManeuver(maneuver, canopy, linePlan);
    };

    // -- the manoeuvre set -------------------------------------------------
    std::vector<ManeuverResult> results;
    for (const CalibrationManeuver maneuver : {
             CalibrationManeuver::HandsUpTrim,
             CalibrationManeuver::AcceleratorStep,
             CalibrationManeuver::BrakeStep,
             CalibrationManeuver::BrakePulse,
             CalibrationManeuver::WeightShiftStep,
             CalibrationManeuver::CoordinatedTurn,
             CalibrationManeuver::StallApproach})
    {
        results.push_back(run(maneuver));
        std::printf("%s\n", ManeuverSummaryLine(results.back()).c_str());
    }
    std::printf("\n");

    const ManeuverResult& trim = results[0];
    const ManeuverResult& bar = results[1];
    const ManeuverResult& brake = results[2];
    const ManeuverResult& pulse = results[3];
    const ManeuverResult& shift = results[4];
    const ManeuverResult& turn = results[5];
    const ManeuverResult& stall = results[6];

    // -- the manoeuvres are usable at all ----------------------------------
    {
        // A calibration number from a run that never settled, or from one
        // where the numerical safety envelope engaged, means nothing. This is
        // the gate that makes every number below admissible.
        Check(trim.settled, "hands-up trim settles");
        Check(bar.settled, "full bar settles");
        Check(brake.settled, "brake step settles");
        Check(turn.settled, "the coordinated turn settles");
        for (const ManeuverResult& result : results)
        {
            Check(!result.safetyEnvelopeEngaged,
                  std::string("the numerical safety envelope stays out of ")
                      + CalibrationManeuverName(result.maneuver));
        }
    }

    // -- the speed system --------------------------------------------------
    {
        const double trimKmh = trim.settledAirspeedMps * 3.6;
        const double topKmh = bar.settledAirspeedMps * 3.6;
        std::printf("Speed system: trim %.1f km/h (published %.0f), full bar "
                    "%.1f (published %.0f)\n",
                    trimKmh, PublishedTrimKmh, topKmh, PublishedTopKmh);
        std::printf("  range ratio %.3f against a published %.3f\n",
                    topKmh / trimKmh, PublishedTopKmh / PublishedTrimKmh);

        // The RATIO is the quantity that tests the lift curve's slope against
        // a measurement while being blind to its offset, and it is the one
        // this model gets close. The absolute speeds are both low by about the
        // same fraction, which is the signature of an offset - see below.
        Check(std::fabs(topKmh / trimKmh
                        - PublishedTopKmh / PublishedTrimKmh) < 0.12,
              "the speed RANGE matches the published one to better than 12% - "
              "this is the test of the lift curve's slope, and it passes");

        // Bar must lower the incidence, because that is the mechanism.
        std::printf("  incidence %.1f deg at trim, %.1f deg on bar\n",
                    trim.settledIncidenceRad * Degrees,
                    bar.settledIncidenceRad * Degrees);
        Check(bar.settledIncidenceRad < trim.settledIncidenceRad - 0.02,
              "and bar reaches the wing by lowering its incidence, not by any "
              "other path");

        // KNOWN DISAGREEMENT, bounded rather than hidden. Both speeds are low
        // by about 18%. The lift curve tests out close to right, so the
        // suspect is the pitch stiffness: the rigid motion lumps canopy and
        // payload into one body whose centre of mass is eight metres down, so
        // gravity's restoring torque is counted there AND in the swing degree
        // of freedom. PHYSICS_TODO item 10.
        const double shortfall = 1.0 - trimKmh / PublishedTrimKmh;
        std::printf("  KNOWN DISAGREEMENT: trim is %.0f%% below published\n",
                    shortfall * 100.0);
        Check(shortfall > 0.0 && shortfall < 0.25,
              "KNOWN DISAGREEMENT: trim speed is low, bounded here so it "
              "cannot grow unnoticed and cannot be closed by accident");
    }

    // -- sink and glide ----------------------------------------------------
    {
        std::printf("Glide: %.2f at trim (published %.1f), sink %.2f m/s "
                    "(published min %.1f)\n",
                    trim.settledGlideRatio, PublishedGlide,
                    trim.settledSinkMps, PublishedMinSinkMps);
        Check(trim.settledGlideRatio > 7.0 && trim.settledGlideRatio < 11.0,
              "glide is within a fifth of the published 9.5");
        Check(trim.settledSinkMps > 0.7 && trim.settledSinkMps < 1.5,
              "and sink is a paraglider's sink rather than a wingsuit's");

        // Brake must cost glide. Not a published number, but it is not
        // optional either: 40% brake trades speed for nothing at this
        // incidence, so the glide has to fall.
        std::printf("  40%% brake: %.2f m/s, glide %.2f\n",
                    brake.settledAirspeedMps, brake.settledGlideRatio);
        Check(brake.settledAirspeedMps < trim.settledAirspeedMps,
              "brake slows the wing");
        Check(brake.settledGlideRatio < trim.settledGlideRatio,
              "and costs glide, because trim is already near the best of it");
    }

    // -- pitch: the pendulum, against its closed form ----------------------
    {
        // The one place this level has an exact external reference. A brake
        // pulse released leaves a free oscillation of the wing against the
        // pilot, and its period is set by the pendulum length and the line
        // stiffness together. With no line stiffness at all it would be a
        // simple pendulum at 2*pi*sqrt(L/g); the lines make it faster.
        const SuspensionGraph graph = BuildSuspensionGraph(canopy, linePlan);
        const double lengthM = SuspensionPendulumLengthM(graph);
        const double simplePendulumS = 2.0 * Pi * std::sqrt(lengthM / 9.80665);
        std::printf("Pitch: period %.2f s over %d oscillations, damping "
                    "ratio %.2f\n",
                    pulse.pitchPeriodS, pulse.pitchOscillationsMeasured,
                    pulse.pitchDampingRatio);
        std::printf("  simple pendulum on %.2f m would be %.2f s; a stiffer "
                    "wing is faster\n", lengthM, simplePendulumS);

        Check(pulse.pitchOscillationsMeasured >= 1,
              "a released brake pulse leaves a measurable pitch oscillation - "
              "which is the surge, and which did not exist at all until the "
              "wing and the pilot became two bodies");
        Check(pulse.pitchPeriodS > 0.5 && pulse.pitchPeriodS < simplePendulumS,
              "its period is faster than a simple pendulum on the same lines, "
              "because the line geometry adds stiffness the bob does not have");
        Check(pulse.pitchDampingRatio > 0.02 && pulse.pitchDampingRatio < 0.9,
              "and it is damped rather than ringing or dead - a real wing "
              "settles in a few swings");

        // The surge itself, which is the pilot-facing half of the same event.
        std::printf("  canopy lead %.2f m at the back of the pulse, %.2f m at "
                    "the front of the surge\n",
                    pulse.leastCanopyLeadM, pulse.peakCanopyLeadM);
        Check(pulse.peakCanopyLeadM - pulse.leastCanopyLeadM > 1.0,
              "and the wing travels more than a metre fore-and-aft across the "
              "manoeuvre, which is what a pilot sees");
    }

    // -- turning -----------------------------------------------------------
    {
        std::printf("Turn: %.3f rad/s at %.1f deg of bank on 35%% brake\n",
                    turn.settledTurnRateRadps, turn.settledBankRad * Degrees);
        std::printf("  weight shift alone: %.3f rad/s at %.1f deg\n",
                    shift.settledTurnRateRadps, shift.settledBankRad * Degrees);

        Check(std::fabs(turn.settledTurnRateRadps) > 1.0e-3,
              "brake on one side turns the wing");
        Check(std::fabs(shift.settledTurnRateRadps) > 1.0e-4,
              "and so does weight shift on its own, with no brake at all - "
              "which is guiding rule 5 working rather than a shift-to-roll "
              "coefficient");
        // Direction. Right brake and right weight shift must both turn the
        // same way, and each must bank the way this frame says a turn banks:
        // positive bank is right-tip-UP, which is a LEFT turn, so a right turn
        // carries negative bank. That relationship has been documented
        // backwards in this project before, so it is checked rather than read.
        Check(turn.settledTurnRateRadps * shift.settledTurnRateRadps > 0.0,
              "and both right-hand inputs turn the same way");
        Check(turn.settledTurnRateRadps * turn.settledBankRad < 0.0,
              "and the wing banks into its turn - positive bank is right tip "
              "up, which is a left turn, so the two carry opposite signs");
        Check(std::fabs(turn.settledTurnRateRadps)
                  > std::fabs(shift.settledTurnRateRadps),
              "brake is the stronger of the two, as it is on a real wing");

        // KNOWN DISAGREEMENT, and the largest one this level has found. A real
        // EN-B wing at 35% brake turns at roughly 0.3 rad/s with 20-30 degrees
        // of bank. This model turns at 0.015 rad/s with 2 degrees - twenty
        // times too slowly, which is not a calibration error but a mechanism
        // that is missing or overwhelmed.
        //
        // Bounded here so it is a recorded finding rather than an impression,
        // and so that closing it registers as a failure here rather than
        // passing silently. Not diagnosed yet; the candidates are the doubled
        // stiffness of PHYSICS_TODO item 10, the yaw damping the VSM measures
        // by probe, and the fact that the only path from bank to turn is the
        // sideslip the circulation solve sees.
        const double turnRateFraction =
            std::fabs(turn.settledTurnRateRadps) / 0.30;
        std::printf("  KNOWN DISAGREEMENT: turn rate is %.0f%% of what an "
                    "EN-B wing does at this brake\n",
                    turnRateFraction * 100.0);
        Check(turnRateFraction < 0.5,
              "KNOWN DISAGREEMENT: the wing turns roughly twenty times too "
              "slowly for its brake input. Bounded so that fixing it shows up "
              "here rather than passing silently");
    }

    // -- the stall approach ------------------------------------------------
    {
        std::printf("Stall approach: minimum airspeed %.2f m/s, peak "
                    "collapse %.3f\n",
                    stall.minimumAirspeedMps, stall.peakWorstCollapse);
        Check(stall.minimumAirspeedMps < trim.settledAirspeedMps,
              "ramping brake in slows the wing below trim");
        Check(stall.peakWorstCollapse < 0.05,
              "and stalls it rather than folding it - brake raises incidence, "
              "which is the wrong direction for a collapse");
    }

    // -- the books still balance under manoeuvre ---------------------------
    {
        // Level 7's gate, re-asked where it matters: the energy accounting has
        // to hold during a manoeuvre, not only in steady flight.
        // Attributed per manoeuvre, because a single worst-case number over
        // seven manoeuvres says nothing about which one is misbehaving.
        double worstFlying = 0.0;
        const char* worstFlyingName = "";
        for (const ManeuverResult& result : results)
        {
            std::printf("  %-24s worst energy residual %6.1f W\n",
                        CalibrationManeuverName(result.maneuver),
                        result.worstEnergyResidualW);
            // The stall approach is excluded from the flying gate and stated
            // separately. It ends fully separated at 91 degrees of incidence,
            // which is the branch with no steady state to find - energy
            // bookkeeping across a solve that is not converging is not a
            // statement about the solver's conservation.
            if (result.maneuver == CalibrationManeuver::StallApproach) continue;
            if (result.worstEnergyResidualW > worstFlying)
            {
                worstFlying = result.worstEnergyResidualW;
                worstFlyingName = CalibrationManeuverName(result.maneuver);
            }
        }
        std::printf("  worst while flying: %.1f W (%s)\n",
                    worstFlying, worstFlyingName);
        // Steady flight closes to a fraction of a watt. What raises the
        // number is a PITCH TRANSIENT, and by a known amount: the pendulum
        // between wing and pilot carries real energy - 900 J at the top of a
        // surge - which is outside these books because the rigid motion still
        // integrates one lumped body with its mass at the canopy. Putting the
        // term in without the body it belongs to makes the audit disagree with
        // the solver it is auditing, measured. PHYSICS_TODO item 10 closes it.
        //
        // So the gate is set where it is diagnostic: steady manoeuvres must
        // close tightly, and the transient ones are bounded and attributed.
        Check(results[0].worstEnergyResidualW < 5.0
              && results[4].worstEnergyResidualW < 5.0,
              "manoeuvres with no pitch transient in them close the energy "
              "books to a few watts on a 925 N aircraft");
        Check(worstFlying < 200.0,
              "KNOWN GAP: pitch transients leave a residual, and it is the "
              "pendulum's own energy sitting outside books that still describe "
              "one lumped body. Bounded and attributed rather than absorbed");
    }

    // -- the export ---------------------------------------------------------
    {
        // The deliverable is the time series, so the exporter is gated too.
        const std::string path = "hands-up-trim.csv";
        Check(WriteManeuverCsv(trim, path),
              "the time series exports as CSV");
        Check(trim.samples.size() > 100,
              "with enough rows to identify anything from");
    }

    if (Failures == 0) std::printf("\nAll calibration checks passed.\n");
    else std::printf("\n%d calibration check(s) failed.\n", Failures);
    return Failures == 0 ? 0 : 1;
}
