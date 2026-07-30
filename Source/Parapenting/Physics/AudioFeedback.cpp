#include "AudioFeedback.h"

#include <algorithm>

namespace Parapenting::Physics
{
AudioFeedback EvaluateAudioFeedback(const AudioFeedbackInput& i)
{
    AudioFeedback result;
    const double climb = std::clamp(i.verticalSpeedMps, 0.0, 6.0);
    const bool climbing = i.verticalSpeedMps > 0.25;
    const bool strongSink = i.verticalSpeedMps < -2.2;
    result.varioFrequencyHz = climbing ? 620.0 + climb * 95.0 : 280.0;
    result.varioBeepRateHz = climbing ? 1.5 + climb * 0.48 : 0.0;
    result.varioLevel = climbing ? 0.11 : (strongSink ? 0.07 : 0.0);
    result.windLevel = std::clamp(
        (i.airspeedMps - 4.0) / 17.0, 0.012, 0.23);
    result.windFilterAmount = std::clamp(
        0.018 + i.airspeedMps * 0.0022, 0.02, 0.075);

    const double pressureRustle =
        std::clamp(1.0 - i.canopyPressure, 0.0, 1.0) * 0.09;
    const double unloadingHiss =
        std::clamp(i.aerodynamicUnloading, 0.0, 1.0) * 0.055;
    const double sharedFabric = 0.035 * std::clamp(i.turbulence, 0.0, 1.0)
        + pressureRustle + unloadingHiss
        + std::clamp(i.highLoadDeformation, 0.0, 1.0) * 0.018
        + std::clamp(i.highFrequencyGustMps / 2.0, 0.0, 1.0) * 0.025;
    result.leftFabricLevel = std::clamp(
        sharedFabric + 0.09 * i.leftCollapse + 0.14 * i.leftCravat,
        0.0, 0.32);
    result.rightFabricLevel = std::clamp(
        sharedFabric + 0.09 * i.rightCollapse + 0.14 * i.rightCravat,
        0.0, 0.32);

    result.lineFrequencyHz =
        72.0 + std::clamp(i.lineLoadN / 18.0, 0.0, 120.0);
    const double commonLine = std::clamp(
        (i.lineLoadN - 450.0) / 5000.0, 0.0, 0.026)
        + std::clamp(i.highLoadDeformation, 0.0, 1.0) * 0.009;
    result.leftLineLevel = std::clamp(
        commonLine + i.leftBrakeForceN / 4000.0, 0.0, 0.045);
    result.rightLineLevel = std::clamp(
        commonLine + i.rightBrakeForceN / 4000.0, 0.0, 0.045);
    result.thermalBreathLevel = std::clamp(
        i.thermalCoreMps / 4.0, 0.0, 1.0) * 0.055;
    result.surgeRushLevel = std::clamp(
        i.recoverySurge, 0.0, 0.45) * result.windLevel * 0.70;
    return result;
}
}
