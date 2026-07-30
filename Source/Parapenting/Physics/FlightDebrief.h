#pragma once

#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
enum class FlightPhase
{
    Launch,
    Glide,
    Thermal,
    Rotor,
    Incident,
    Approach,
    Landed
};

struct FlightDebriefSample
{
    Vec3 positionWorldM{};
    double verticalSpeedMps = 0.0;
    double airspeedMps = 0.0;
    double loadFactor = 1.0;
    double canopyPressure = 1.0;
    double thermalLiftMps = 0.0;
    double rotorStrength = 0.0;
    double turbulence = 0.0;
    double leftCollapse = 0.0;
    double rightCollapse = 0.0;
    double frontalCollapse = 0.0;
    double leftCravat = 0.0;
    double rightCravat = 0.0;
    double deepStall = 0.0;
    double spin = 0.0;
    double leftBrake = 0.0;
    double rightBrake = 0.0;
    double weightShift = 0.0;
    double groundClearanceM = 1000.0;
    double distanceToLandingM = 10000.0;
    double approachQuality = 0.0;
    bool stabilizedApproach = false;
    bool groundLaunching = false;
};

struct FlightDebriefSummary
{
    FlightPhase currentPhase = FlightPhase::Launch;
    double durationS = 0.0;
    double horizontalDistanceM = 0.0;
    double altitudeGainM = 0.0;
    double altitudeLossM = 0.0;
    double thermalGainM = 0.0;
    double thermalTimeS = 0.0;
    double rotorExposureS = 0.0;
    double severeRotorExposureS = 0.0;
    double stabilizedFinalS = 0.0;
    double averageApproachQuality = 0.0;
    double maximumClimbMps = 0.0;
    double maximumSinkMps = 0.0;
    double maximumAirspeedMps = 0.0;
    double maximumLoadFactor = 1.0;
    double minimumCanopyPressure = 1.0;
    int asymmetricCollapseEvents = 0;
    int frontalCollapseEvents = 0;
    int cravatEvents = 0;
    int stallOrSpinEvents = 0;
    double highLoadTimeS = 0.0;
    double controlActivity = 0.0;
    double landingDistanceM = 0.0;
    double touchdownVerticalSpeedMps = 0.0;
    double touchdownHorizontalSpeedMps = 0.0;
    double safetyRating = 100.0;
    double efficiencyRating = 50.0;
    double thermalRating = 50.0;
    double landingRating = 50.0;
    double overallRating = 50.0;
    bool landed = false;
};

class FlightDebrief
{
public:
    void Reset();
    void Step(const FlightDebriefSample& sample, double dt);
    void FinalizeLanding(
        double distanceM, double verticalSpeedMps, double horizontalSpeedMps);
    const FlightDebriefSummary& Summary() const { return summary_; }

private:
    void UpdateRatings();

    FlightDebriefSummary summary_{};
    Vec3 previousPosition_{};
    double previousLeftBrake_ = 0.0;
    double previousRightBrake_ = 0.0;
    double previousWeightShift_ = 0.0;
    double approachQualityIntegral_ = 0.0;
    double approachTimeS_ = 0.0;
    bool hasPreviousPosition_ = false;
    bool collapseActive_ = false;
    bool frontalActive_ = false;
    bool cravatActive_ = false;
    bool stallSpinActive_ = false;
};

const char* FlightPhaseName(FlightPhase phase);
const char* FlightDebriefFocusText(const FlightDebriefSummary& summary);
}
