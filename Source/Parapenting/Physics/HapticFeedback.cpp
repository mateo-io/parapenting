#include "HapticFeedback.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
HapticOutput HapticFeedbackModel::Evaluate(
    const Telemetry& t, double timeSeconds)
{
    const double deltaTime = previousTimeSeconds < 0.0
        ? 1.0 / 60.0
        : std::clamp(timeSeconds - previousTimeSeconds, 0.0, 0.25);
    previousTimeSeconds = timeSeconds;
    const double leftOnset =
        std::max(0.0, t.leftCollapse - previousLeftCollapse);
    const double rightOnset =
        std::max(0.0, t.rightCollapse - previousRightCollapse);
    previousLeftCollapse = t.leftCollapse;
    previousRightCollapse = t.rightCollapse;
    const double impulseDecay = std::exp(-deltaTime / 0.11);
    leftImpulse = std::max(leftImpulse * impulseDecay, leftOnset * 2.7);
    rightImpulse = std::max(rightImpulse * impulseDecay, rightOnset * 2.7);

    const double rotorTexture = t.rotorStrength
        * (0.14 + 0.10 * std::abs(std::sin(timeSeconds * 19.0)));
    const double gustTexture = std::clamp(
        t.highFrequencyGustMps / 1.5, 0.0, 1.0)
        * (0.035 + 0.035 * std::abs(std::sin(timeSeconds * 31.0)));
    const double pressureFlutter =
        std::max(0.0, 0.72 - t.canopyPressure) * 0.38;
    const double loadPulse = std::max(0.0, t.loadFactor - 1.15) * 0.045;
    const double loadStrain = t.highLoadDeformation
        * (0.035 + 0.025 * std::abs(std::sin(timeSeconds * 33.0)));
    const double surge = std::max(0.0, t.recoverySurge) * 0.28;
    const double frontalReinflation = std::clamp(
        t.frontalReinflationRatePerS / 0.35, 0.0, 1.0)
        * (0.10 + 0.07 * std::abs(std::sin(timeSeconds * 35.0)));
    // A brief high-frequency lightening cue begins at aerodynamic unloading,
    // just before visible fabric loss. This gives the controller pilot a
    // chance to react instead of only reporting a collapse after it exists.
    const double unloadingCue = t.aerodynamicUnloading
        * (0.12 + 0.10 * std::abs(std::sin(timeSeconds * 41.0)));
    const double pressureDropCue = std::clamp(
        t.dynamicPressureDropPaPerS / 900.0, 0.0, 1.0) * 0.08;
    const double flareLoad = t.flareAuthority * 0.16;
    const double shared = rotorTexture + gustTexture
        + pressureFlutter + loadPulse + loadStrain
        + surge + unloadingCue + pressureDropCue + flareLoad
        + t.frontalCollapse * 0.48 + frontalReinflation;

    // Brake load is a smooth same-side pull. Collapse onset is a short thump;
    // sustained unloading adds a rough lighter vibration on the affected side.
    const double leftUnload = (t.leftCollapse * 0.42 + t.leftCravat * 0.52)
        * (0.62 + 0.38 * std::abs(std::sin(timeSeconds * 27.0)));
    const double rightUnload = (t.rightCollapse * 0.42 + t.rightCravat * 0.52)
        * (0.62 + 0.38 * std::abs(std::sin(timeSeconds * 29.0 + 0.8)));
    const double asymmetry = std::clamp(t.spanwiseLoadAsymmetry, -1.0, 1.0);
    // Tip-scale airflow discontinuity is felt as a light, fast same-side
    // texture before the fabric visibly folds. Keep it below the collapse
    // impulse so the cue informs rather than overwhelms.
    const double leftShearCue = t.leftAirflowDisturbance
        * (0.055 + 0.045 * std::abs(std::sin(timeSeconds * 37.0)));
    const double rightShearCue = t.rightAirflowDisturbance
        * (0.055 + 0.045 * std::abs(std::sin(timeSeconds * 39.0 + 0.4)));
    const double leftReinflationCue = std::clamp(
        t.leftReinflationRatePerS / 0.35, 0.0, 1.0)
        * (0.12 + 0.08 * std::abs(std::sin(timeSeconds * 43.0)));
    const double rightReinflationCue = std::clamp(
        t.rightReinflationRatePerS / 0.35, 0.0, 1.0)
        * (0.12 + 0.08 * std::abs(std::sin(timeSeconds * 45.0 + 0.6)));

    return {
        std::clamp(shared + t.leftBrakePressure * 0.28 + leftUnload
            + leftImpulse + leftShearCue
            + leftReinflationCue
            + std::max(0.0, -asymmetry) * 0.16, 0.0, 1.0),
        std::clamp(shared + t.rightBrakePressure * 0.28 + rightUnload
            + rightImpulse + rightShearCue
            + rightReinflationCue
            + std::max(0.0, asymmetry) * 0.16, 0.0, 1.0)
    };
}

void HapticFeedbackModel::Reset()
{
    previousLeftCollapse = 0.0;
    previousRightCollapse = 0.0;
    leftImpulse = 0.0;
    rightImpulse = 0.0;
    previousTimeSeconds = -1.0;
}
}
