#include "CameraFeedback.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
CameraFeedback EvaluateCameraFeedback(
    const Telemetry& t, const Vec3& acceleration, double timeSeconds,
    const AccessibilityProfile& accessibility)
{
    const double motion = accessibility.inertialCameraScale;
    const double buffetScale = accessibility.rotorBuffetScale;
    const double rotorWave =
        (0.58 * std::sin(timeSeconds * 12.7)
         + 0.29 * std::sin(timeSeconds * 27.1 + 0.8)
         + 0.13 * std::sin(timeSeconds * 47.3 + 2.1));
    const double roughAir = std::clamp(
        t.rotorStrength * 0.72 + t.turbulence * 0.24
            + t.lowFrequencyGustMps / 3.0 * 0.18,
        0.0, 1.0);
    const double buffet = roughAir * buffetScale * rotorWave;
    const double collapseAsymmetry = std::clamp(
        t.rightCollapse + t.rightCravat
            - t.leftCollapse - t.leftCravat,
        -1.0, 1.0);
    const double loadCompression = std::clamp(
        t.loadFactor - 1.0, 0.0, 3.0);
    const double unloadingPulse = t.aerodynamicUnloading
        * std::sin(timeSeconds * 38.0);
    const double asymmetricReinflation = std::clamp(
        t.rightReinflationRatePerS - t.leftReinflationRatePerS,
        -0.7, 0.7);
    const double reinflationKick = std::clamp(
        0.5 * (t.leftReinflationRatePerS + t.rightReinflationRatePerS)
            + 0.8 * t.frontalReinflationRatePerS,
        0.0, 0.75);

    CameraFeedback result;
    result.positionOffsetCm = {
        std::clamp(acceleration.x * -9.0, -75.0, 75.0) * motion
            - t.recoverySurge * 38.0 * motion,
        std::clamp(acceleration.y * -13.0, -95.0, 95.0) * motion
            + buffet * 17.0
            + collapseAsymmetry * 48.0 * motion
            + unloadingPulse * 8.0 * motion,
        std::clamp(acceleration.z * -7.0, -60.0, 60.0) * motion
            - loadCompression * 9.0 * motion
            - t.flareAuthority * 12.0 * motion
            - t.highLoadDeformation * 7.0 * motion
            - t.frontalCollapse * 22.0 * motion
            - reinflationKick * 18.0 * motion
    };
    result.pitchDegrees =
        -t.harnessPitchRad * 18.0 * motion
        - t.flareAuthority * 4.5 * motion
        - t.recoverySurge * 14.0 * motion
        + t.frontalCollapse * 9.0 * motion
        - reinflationKick * 8.0 * motion
        + t.deepStall * std::sin(timeSeconds * 3.2) * 2.2 * motion;
    result.yawDegrees =
        collapseAsymmetry * 5.5 * motion
        + asymmetricReinflation * 2.8 * motion + buffet * 0.65;
    result.rollDegrees =
        -t.harnessRollRad * 24.0 * motion
        + collapseAsymmetry * 11.0 * motion
        + asymmetricReinflation * 5.0 * motion
        + buffet * 2.1;
    result.fieldOfViewDeltaDegrees =
        std::clamp(t.airspeedMps - 10.5, -2.0, 8.0) * 0.70 * motion
        + loadCompression * 0.45 * motion
        - t.highLoadDeformation * 1.1 * motion
        + std::max(0.0, t.recoverySurge) * 3.0 * motion
        + reinflationKick * 1.8 * motion;
    return result;
}
}
