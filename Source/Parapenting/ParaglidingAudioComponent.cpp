#include "ParaglidingAudioComponent.h"
#include "Physics/AudioFeedback.h"

void UParaglidingAudioComponent::SetFlightAudio(
    float VarioMps, float AirspeedMps, float Turbulence,
    float LeftCollapse, float RightCollapse, float LeftCravat,
    float RightCravat, float CanopyPressure, float LineLoadN,
    float RecoverySurge, float LeftBrakeForceN, float RightBrakeForceN,
    float ThermalCoreMps, float AerodynamicUnloading,
    float HighLoadDeformation, float HighFrequencyGustMps)
{
    TargetVario.Store(VarioMps);
    TargetAirspeed.Store(AirspeedMps);
    TargetTurbulence.Store(Turbulence);
    TargetLeftCollapse.Store(LeftCollapse);
    TargetRightCollapse.Store(RightCollapse);
    TargetLeftCravat.Store(LeftCravat);
    TargetRightCravat.Store(RightCravat);
    TargetCanopyPressure.Store(CanopyPressure);
    TargetLineLoadN.Store(LineLoadN);
    TargetRecoverySurge.Store(RecoverySurge);
    TargetLeftBrakeForceN.Store(LeftBrakeForceN);
    TargetRightBrakeForceN.Store(RightBrakeForceN);
    TargetThermalCoreMps.Store(ThermalCoreMps);
    TargetAerodynamicUnloading.Store(AerodynamicUnloading);
    TargetHighLoadDeformation.Store(HighLoadDeformation);
    TargetHighFrequencyGustMps.Store(HighFrequencyGustMps);
}

bool UParaglidingAudioComponent::Init(int32& SampleRate)
{
    AudioSampleRate = SampleRate;
    NumChannels = 2;
    return true;
}

int32 UParaglidingAudioComponent::OnGenerateAudio(float* OutAudio, int32 NumSamples)
{
    constexpr double TwoPi = 6.283185307179586;
    const float VarioTarget = TargetVario.Load();
    const float AirspeedTarget = TargetAirspeed.Load();
    const float Turbulence = TargetTurbulence.Load();
    const float LeftCollapse = TargetLeftCollapse.Load();
    const float RightCollapse = TargetRightCollapse.Load();
    const float LeftCravat = TargetLeftCravat.Load();
    const float RightCravat = TargetRightCravat.Load();
    const float Collapse = FMath::Max(LeftCollapse, RightCollapse);
    const float CanopyPressure = TargetCanopyPressure.Load();
    const float LineLoadN = TargetLineLoadN.Load();
    const float RecoverySurge = TargetRecoverySurge.Load();
    const float LeftBrakeForceN = TargetLeftBrakeForceN.Load();
    const float RightBrakeForceN = TargetRightBrakeForceN.Load();
    const float ThermalCoreMps = TargetThermalCoreMps.Load();
    const float AerodynamicUnloading =
        TargetAerodynamicUnloading.Load();
    const float HighLoadDeformation =
        TargetHighLoadDeformation.Load();
    const float HighFrequencyGustMps =
        TargetHighFrequencyGustMps.Load();
    const auto Mix = Parapenting::Physics::EvaluateAudioFeedback({
        VarioTarget, AirspeedTarget, Turbulence,
        LeftCollapse, RightCollapse, LeftCravat, RightCravat,
        CanopyPressure, LineLoadN, RecoverySurge,
        LeftBrakeForceN, RightBrakeForceN,
        ThermalCoreMps, AerodynamicUnloading, HighLoadDeformation,
        HighFrequencyGustMps});
    const float CollapseImpulse = FMath::Max(0.0f, Collapse - PreviousCollapse);
    FabricEnvelope = FMath::Max(FabricEnvelope, CollapseImpulse * 1.8f);
    PreviousCollapse = Collapse;

    for (int32 Index = 0; Index < NumSamples; Index += 2)
    {
        CurrentVario += (VarioTarget - CurrentVario) * 0.0009f;
        CurrentAirspeed += (AirspeedTarget - CurrentAirspeed) * 0.0005f;
        const bool bClimbing = CurrentVario > 0.25f;
        const bool bSinking = CurrentVario < -2.2f;
        TonePhase += TwoPi * Mix.varioFrequencyHz / AudioSampleRate;
        LinePhase += TwoPi * Mix.lineFrequencyHz / AudioSampleRate;
        GatePhase += Mix.varioBeepRateHz / AudioSampleRate;
        if (TonePhase > TwoPi) TonePhase -= TwoPi;
        if (LinePhase > TwoPi) LinePhase -= TwoPi;
        if (GatePhase > 1.0) GatePhase -= 1.0;
        const float Gate = bClimbing
            ? (GatePhase < 0.42 ? 1.0f : 0.0f)
            : (bSinking ? 0.52f : 0.0f);
        const float Tone = FMath::Sin(static_cast<float>(TonePhase))
            * Gate * static_cast<float>(Mix.varioLevel);

        NoiseState = NoiseState * 1664525u + 1013904223u;
        const float White = (static_cast<float>((NoiseState >> 8) & 0xffffu)
                            / 32767.5f) - 1.0f;
        NoiseState = NoiseState * 1664525u + 1013904223u;
        const float WhiteRight =
            (static_cast<float>((NoiseState >> 8) & 0xffffu) / 32767.5f) - 1.0f;
        const float WindLevel = static_cast<float>(Mix.windLevel);
        const float FilterAmount =
            static_cast<float>(Mix.windFilterAmount);
        WindFilterLeft += (White - WindFilterLeft) * FilterAmount;
        WindFilterRight += (WhiteRight - WindFilterRight) * FilterAmount;
        ThermalFilter += (
            0.5f * (White + WhiteRight) - ThermalFilter) * 0.0035f;
        const float ThermalBreath = ThermalFilter
            * static_cast<float>(Mix.thermalBreathLevel);
        const float LeftFabric = static_cast<float>(Mix.leftFabricLevel)
            + FabricEnvelope * 0.22f;
        const float RightFabric = static_cast<float>(Mix.rightFabricLevel)
            + FabricEnvelope * 0.22f;
        const float LineWave = FMath::Sin(static_cast<float>(LinePhase));
        const float LeftLine =
            LineWave * static_cast<float>(Mix.leftLineLevel);
        const float RightLine =
            LineWave * static_cast<float>(Mix.rightLineLevel);
        const float SurgeRush = static_cast<float>(Mix.surgeRushLevel);
        const float Left = Tone + LeftLine + ThermalBreath
            + WindFilterLeft * (WindLevel + SurgeRush)
            + White * LeftFabric;
        const float Right = Tone + RightLine + ThermalBreath
            + WindFilterRight * (WindLevel + SurgeRush)
            + WhiteRight * RightFabric;
        OutAudio[Index] = FMath::Clamp(Left, -0.8f, 0.8f);
        if (Index + 1 < NumSamples)
            OutAudio[Index + 1] = FMath::Clamp(Right, -0.8f, 0.8f);
        FabricEnvelope *= 0.99955f;
    }
    return NumSamples;
}
