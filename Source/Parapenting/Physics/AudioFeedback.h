#pragma once

namespace Parapenting::Physics
{
struct AudioFeedbackInput
{
    double verticalSpeedMps = 0.0;
    double airspeedMps = 0.0;
    double turbulence = 0.0;
    double leftCollapse = 0.0;
    double rightCollapse = 0.0;
    double leftCravat = 0.0;
    double rightCravat = 0.0;
    double canopyPressure = 1.0;
    double lineLoadN = 0.0;
    double recoverySurge = 0.0;
    double leftBrakeForceN = 0.0;
    double rightBrakeForceN = 0.0;
    double thermalCoreMps = 0.0;
    double aerodynamicUnloading = 0.0;
    double highLoadDeformation = 0.0;
    double highFrequencyGustMps = 0.0;
};

struct AudioFeedback
{
    double varioFrequencyHz = 280.0;
    double varioBeepRateHz = 0.0;
    double varioLevel = 0.0;
    double windLevel = 0.0;
    double windFilterAmount = 0.02;
    double leftFabricLevel = 0.0;
    double rightFabricLevel = 0.0;
    double lineFrequencyHz = 72.0;
    double leftLineLevel = 0.0;
    double rightLineLevel = 0.0;
    double thermalBreathLevel = 0.0;
    double surgeRushLevel = 0.0;
};

AudioFeedback EvaluateAudioFeedback(const AudioFeedbackInput& input);
}
