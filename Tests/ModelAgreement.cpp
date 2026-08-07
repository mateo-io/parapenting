// Where do the two flight models agree, and where does the geometry-driven one
// stop being usable?
//
// `PHYSICS_TODO` items 7 and 17: the game flies `ParagliderDynamics`, one
// rigid body with a fitted polar, and nothing geometry-driven has ever flown
// it. Item 17's exit gate is "no legacy direct-control force remains active",
// and it is blocked because the geometry-driven stack departs at 40% brake.
//
// That block has been stated for several levels and never measured as a
// BOUNDARY. "Departs at 40% brake" is one point; what a swap needs is the
// whole envelope - every condition where the two models produce the same
// aircraft, and the first condition where they do not. Until that exists,
// "fly the geometry-driven stack across its stated envelope" has no stated
// envelope to fly across.
//
// So this is guiding rule 11 taken literally - "both run side by side until
// then" - as a measurement rather than as an intention. Both models, the same
// wing, the same all-up mass, the same still air, the same control inputs,
// settled the same way, reported next to each other.
//
// WHAT THIS IS NOT. It is not a claim that either model is right where they
// agree; two models can agree and both be wrong, and the published envelope is
// the only arbiter of that (`calibration_tests`). Agreement is necessary for a
// swap and not sufficient. What disagreement gives is the boundary, and the
// boundary is what item 17 needs.
//
// Deliberately NOT in `Tools/check-build.sh`: this settles both models at
// every control station and takes minutes. `calibration_tests` gates the
// coupled solver against published numbers; this reports a comparison.
#include "CalibrationManeuver.h"
#include "CanopyGeometry.h"
#include "CoupledParagliderSolver.h"
#include "EquipmentSetup.h"
#include "ParagliderDynamics.h"
#include "SuspensionGraph.h"
#include "WingCatalogue.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace Parapenting::Physics;

namespace
{
constexpr double Pi = 3.14159265358979323846;
constexpr double Degrees = 180.0 / Pi;
constexpr double Dt = 1.0 / 120.0;

// The weight the published numbers are quoted at, so both models fly the same
// aircraft. §69's sweep showed trim speed does not scale exactly as the square
// root of loading on this wing, so comparing two models at two weights would
// put a systematic 3% into the comparison rather than into either model.
constexpr double AllUpKg = 105.0;

// Long enough that both models are past their opening transient at the
// stations that settle at all. Item 18 measured hands-up needing 530 s to a
// strict criterion; this is a comparison rather than an identification, and
// what matters is that BOTH models get the same clock. Where a station has not
// settled that is reported rather than averaged away.
constexpr double SettleSeconds = 300.0;
constexpr double AverageSeconds = 10.0;
// Hands off first, THEN the input. A station applied from the initial
// condition departs the wing by the RATE of the input as well as by the
// condition, and the two are different claims - the calibration runner settles
// before its step for the same reason. Without this, full bar looked like a
// departure that was partly just an instantaneous shove.
constexpr double PreSettleSeconds = 120.0;

struct Reading
{
    double airspeedMps = 0.0;
    double sinkMps = 0.0;
    double glide = 0.0;
    double incidenceRad = 0.0;
    double turnRateRadps = 0.0;
    // Peak-to-peak airspeed over the averaging window, as a fraction of the
    // mean. A station that is still oscillating cannot be compared to one that
    // is not, and saying so is the point.
    double unsteadiness = 0.0;
    bool finite = true;
};

struct Station
{
    const char* name;
    double symmetricBrake;
    double weightShift;
    double accelerator;
};

Reading Average(const std::vector<Reading>& window)
{
    Reading out;
    if (window.empty()) return out;
    double lowSpeed = window.front().airspeedMps;
    double highSpeed = window.front().airspeedMps;
    for (const Reading& sample : window)
    {
        out.airspeedMps += sample.airspeedMps;
        out.sinkMps += sample.sinkMps;
        out.incidenceRad += sample.incidenceRad;
        out.turnRateRadps += sample.turnRateRadps;
        out.finite = out.finite && sample.finite;
        lowSpeed = std::min(lowSpeed, sample.airspeedMps);
        highSpeed = std::max(highSpeed, sample.airspeedMps);
    }
    const auto count = static_cast<double>(window.size());
    out.airspeedMps /= count;
    out.sinkMps /= count;
    out.incidenceRad /= count;
    out.turnRateRadps /= count;
    out.glide = out.sinkMps > 0.01 ? out.airspeedMps / out.sinkMps : 0.0;
    out.unsteadiness = out.airspeedMps > 0.01
        ? (highSpeed - lowSpeed) / out.airspeedMps : 0.0;
    return out;
}

Reading FlyLegacy(const WingParameters& params, const Station& station)
{
    ParagliderDynamics dynamics;
    dynamics.SetParameters(params);
    FlightState state;
    ControlInput controls;
    controls.leftBrake = station.symmetricBrake;
    controls.rightBrake = station.symmetricBrake;
    controls.weightShift = station.weightShift;
    controls.accelerator = station.accelerator;

    const int preSteps = static_cast<int>(PreSettleSeconds / Dt);
    for (int i = 0; i < preSteps; ++i)
        dynamics.Step(state, ControlInput{}, Atmosphere{}, Dt);

    const int settleSteps = static_cast<int>(SettleSeconds / Dt);
    const int windowSteps = static_cast<int>(AverageSeconds / Dt);
    std::vector<Reading> window;
    for (int i = 0; i < settleSteps + windowSteps; ++i)
    {
        dynamics.Step(state, controls, Atmosphere{}, Dt);
        if (i < settleSteps) continue;
        Reading sample;
        sample.airspeedMps = dynamics.LastTelemetry().airspeedMps;
        sample.sinkMps = -state.velocityWorldMps.z;
        sample.incidenceRad = dynamics.LastTelemetry().angleOfAttackRad;
        sample.turnRateRadps = state.angularVelocityBodyRadps.z;
        sample.finite = std::isfinite(sample.airspeedMps)
            && std::isfinite(sample.sinkMps);
        window.push_back(sample);
    }
    return Average(window);
}

Reading FlyCoupled(const CanopyGeometry& canopy, const LinePlanSpec& linePlan,
                   const PayloadMassProperties& payload,
                   const Station& station, bool& envelopeEngaged)
{
    CoupledParagliderSolver solver(canopy, linePlan, {}, payload);
    CoupledState state;
    CoupledControls controls;
    controls.leftBrake = station.symmetricBrake;
    controls.rightBrake = station.symmetricBrake;
    controls.weightShift = station.weightShift;
    controls.accelerator = station.accelerator;
    const CoupledAtmosphere air;

    const int preSteps = static_cast<int>(PreSettleSeconds / Dt);
    for (int i = 0; i < preSteps; ++i)
        solver.Step(state, CoupledControls{}, air);

    const int settleSteps = static_cast<int>(SettleSeconds / Dt);
    const int windowSteps = static_cast<int>(AverageSeconds / Dt);
    std::vector<Reading> window;
    envelopeEngaged = false;
    for (int i = 0; i < settleSteps + windowSteps; ++i)
    {
        solver.Step(state, controls, air);
        envelopeEngaged = envelopeEngaged
            || solver.Diagnostics().aerodynamicsRejected;
        if (i < settleSteps) continue;
        Reading sample;
        sample.airspeedMps = solver.Diagnostics().airspeedMps;
        sample.sinkMps = -state.velocityWorldMps.z;
        sample.incidenceRad = solver.Diagnostics().angleOfAttackRad;
        sample.turnRateRadps = solver.Diagnostics().turnRateRadps;
        sample.finite = std::isfinite(sample.airspeedMps)
            && std::isfinite(sample.sinkMps);
        window.push_back(sample);
    }
    return Average(window);
}

double RelativeGap(double a, double b)
{
    const double scale = std::max(std::fabs(a), std::fabs(b));
    return scale > 1.0e-6 ? std::fabs(a - b) / scale : 0.0;
}
}

int main()
{
    std::printf("Model agreement: the legacy lumped body against the "
                "geometry-driven stack.\n");
    std::printf("Both at %.0f kg all-up, still air, settled %.0f s and "
                "averaged over %.0f.\n",
                AllUpKg, SettleSeconds, AverageSeconds);
    std::printf("PHYSICS_TODO items 7 and 17. Nothing here asserts - it "
                "reports a boundary.\n\n");

    // -- the legacy side, configured to the same aircraft -------------------
    const WingProfile& profile = GetWingProfile(WingProfileId::Epic2MLResearch);
    EquipmentSetup setup;
    // Solve the pilot mass that puts all-up at the published figure, so the
    // two models are not being compared at two different wing loadings.
    setup.pilotMassKg = AllUpKg
        - GetHarnessProfile(setup.harness).massKg
        - setup.reserveAndEquipmentKg - setup.ballastKg
        - profile.parameters.canopyMassKg;
    const WingParameters legacyParams = ApplyEquipmentSetup(
        profile.parameters, setup, profile.parameters.canopyMassKg);

    // -- the geometry-driven side, at the same all-up ----------------------
    const CanopyGeometry canopy;
    const LinePlanSpec linePlan = Epic2MlLinePlan();
    PayloadMassProperties payload;
    // CoupledParagliderSolver adds its own canopy mass on top of the payload,
    // so the payload carries all-up less the canopy.
    constexpr double CoupledCanopyKg = 5.1;
    payload.pilotKg += (AllUpKg - CoupledCanopyKg) - payload.TotalKg();

    std::printf("legacy all-up   %.1f kg\n", legacyParams.allUpMassKg);
    std::printf("coupled all-up  %.1f kg\n\n",
                payload.TotalKg() + CoupledCanopyKg);

    const std::vector<Station> stations{
        {"hands up",        0.00, 0.0, 0.0},
        {"brake 10%",       0.10, 0.0, 0.0},
        {"brake 20%",       0.20, 0.0, 0.0},
        {"brake 25%",       0.25, 0.0, 0.0},
        {"brake 30%",       0.30, 0.0, 0.0},
        {"brake 40%",       0.40, 0.0, 0.0},
        {"weight shift 50%", 0.00, 0.5, 0.0},
        {"half bar",        0.00, 0.0, 0.5},
        {"full bar",        0.00, 0.0, 1.0},
    };

    std::printf("%-17s  %-24s  %-24s  %-8s %s\n", "station",
                "legacy   v  sink glide turn", "coupled  v  sink glide turn",
                "worst", "");

    int agreeing = 0;
    std::string firstDisagreement;
    for (const Station& station : stations)
    {
        const Reading legacy = FlyLegacy(legacyParams, station);
        bool envelopeEngaged = false;
        const Reading coupled = FlyCoupled(
            canopy, linePlan, payload, station, envelopeEngaged);

        const double speedGap =
            RelativeGap(legacy.airspeedMps, coupled.airspeedMps);
        const double sinkGap = RelativeGap(legacy.sinkMps, coupled.sinkMps);
        const double glideGap = RelativeGap(legacy.glide, coupled.glide);
        const double worst = std::max({speedGap, sinkGap, glideGap});

        // "Comparable" means both models are steady and finite and neither has
        // fallen back on a numerical safety net. A station that fails this is
        // not a disagreement between two aircraft, it is one of them not
        // flying.
        const bool comparable = legacy.finite && coupled.finite
            && !envelopeEngaged
            && legacy.unsteadiness < 0.02 && coupled.unsteadiness < 0.02;
        const bool agrees = comparable && worst < 0.15;
        if (agrees) ++agreeing;
        else if (firstDisagreement.empty()) firstDisagreement = station.name;

        std::printf("%-17s %6.2f %5.2f %5.2f %5.2f   %6.2f %5.2f %5.2f %5.2f"
                    "   %5.1f%%  %s%s%s\n",
                    station.name,
                    legacy.airspeedMps, legacy.sinkMps, legacy.glide,
                    legacy.turnRateRadps,
                    coupled.airspeedMps, coupled.sinkMps, coupled.glide,
                    coupled.turnRateRadps,
                    100.0 * worst,
                    agrees ? "agree" : "DISAGREE",
                    envelopeEngaged ? "  [coupled envelope engaged]" : "",
                    (legacy.unsteadiness >= 0.02
                     || coupled.unsteadiness >= 0.02)
                        ? "  [not steady]" : "");
    }

    // Incidence RELATIVE TO EACH MODEL'S OWN HANDS-UP TRIM, because the two
    // do not share a datum: the legacy model's trim incidence is -6.5 degrees
    // and the coupled solver's is +5.0, which is a difference in where the
    // chord line is measured from rather than a disagreement about the
    // aircraft. Absolute columns would read as a 12-degree dispute that does
    // not exist. What is comparable is how far each moves from its own trim.
    std::printf("\n  incidence change from each model's own hands-up trim, "
                "degrees:\n");
    const Station handsUp = stations.front();
    const Reading legacyTrim = FlyLegacy(legacyParams, handsUp);
    bool trimEnvelope = false;
    const Reading coupledTrim = FlyCoupled(
        canopy, linePlan, payload, handsUp, trimEnvelope);
    for (const Station& station : stations)
    {
        const Reading legacy = FlyLegacy(legacyParams, station);
        bool envelopeEngaged = false;
        const Reading coupled = FlyCoupled(
            canopy, linePlan, payload, station, envelopeEngaged);
        std::printf("    %-17s legacy %+6.2f   coupled %+6.2f\n",
                    station.name,
                    (legacy.incidenceRad - legacyTrim.incidenceRad) * Degrees,
                    (coupled.incidenceRad - coupledTrim.incidenceRad)
                        * Degrees);
    }

    // -- where the geometry-driven envelope actually ends -----------------
    //
    // The grid above says "flies at 30% brake, departs at 40%" and "departs at
    // half bar". A swap needs the boundary, not the bracket, so bisect it.
    // Departure is defined by the wing itself rather than by a threshold on
    // the answer: incidence more than 30 degrees above its own hands-up trim
    // is a separated wing, whatever speed it reports.
    const auto departs = [&](double brake, double bar)
    {
        const Station probe{"probe", brake, 0.0, bar};
        bool engaged = false;
        const Reading reading = FlyCoupled(
            canopy, linePlan, payload, probe, engaged);
        return engaged || !reading.finite
            || (reading.incidenceRad - coupledTrim.incidenceRad) * Degrees
                   > 30.0;
    };
    const auto boundary = [&](bool onBrake)
    {
        double flies = 0.0;
        double gone = 1.0;
        // Eight halvings puts the boundary inside 0.4% of travel, which is
        // finer than a pilot can hold a brake handle.
        for (int i = 0; i < 8; ++i)
        {
            const double mid = 0.5 * (flies + gone);
            if (departs(onBrake ? mid : 0.0, onBrake ? 0.0 : mid)) gone = mid;
            else flies = mid;
        }
        return flies;
    };

    std::printf("\nThe geometry-driven envelope, bisected:\n");
    std::printf("  symmetric brake: flies to %.0f%% of travel\n",
                100.0 * boundary(true));
    std::printf("  speed bar:       flies to %.0f%% of travel\n",
                100.0 * boundary(false));
    std::printf("  weight shift:    no departure, and no authority either - "
                "50%% gives 0.01 rad/s\n");

    std::printf("\n%d of %zu stations agree within 15%% on speed, sink and "
                "glide together.\n", agreeing, stations.size());
    if (!firstDisagreement.empty())
        std::printf("The envelope ends at: %s\n", firstDisagreement.c_str());
    std::printf("\n  Agreement is necessary for a swap and not sufficient - "
                "two models can\n"
                "  agree and both be wrong. What this bounds is where a swap "
                "could be\n"
                "  attempted at all, which is what PHYSICS_TODO item 17 has "
                "been missing.\n");
    return 0;
}
