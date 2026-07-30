#pragma once

#include "CoreMinimal.h"
#include "Components/SynthComponent.h"
#include "ParaglidingAudioComponent.generated.h"

UCLASS(ClassGroup=Synth, meta=(BlueprintSpawnableComponent))
class PARAPENTING_API UParaglidingAudioComponent : public USynthComponent
{
    GENERATED_BODY()

public:
    void SetFlightAudio(float VarioMps, float AirspeedMps,
                        float Turbulence, float LeftCollapse,
                        float RightCollapse, float LeftCravat,
                        float RightCravat, float CanopyPressure,
                        float LineLoadN, float RecoverySurge,
                        float LeftBrakeForceN, float RightBrakeForceN,
                        float ThermalCoreMps, float AerodynamicUnloading,
                        float HighLoadDeformation,
                        float HighFrequencyGustMps);

protected:
    virtual bool Init(int32& SampleRate) override;
    virtual int32 OnGenerateAudio(float* OutAudio, int32 NumSamples) override;

private:
    TAtomic<float> TargetVario{0.0f};
    TAtomic<float> TargetAirspeed{0.0f};
    TAtomic<float> TargetTurbulence{0.0f};
    TAtomic<float> TargetLeftCollapse{0.0f};
    TAtomic<float> TargetRightCollapse{0.0f};
    TAtomic<float> TargetLeftCravat{0.0f};
    TAtomic<float> TargetRightCravat{0.0f};
    TAtomic<float> TargetCanopyPressure{1.0f};
    TAtomic<float> TargetLineLoadN{900.0f};
    TAtomic<float> TargetRecoverySurge{0.0f};
    TAtomic<float> TargetLeftBrakeForceN{0.0f};
    TAtomic<float> TargetRightBrakeForceN{0.0f};
    TAtomic<float> TargetThermalCoreMps{0.0f};
    TAtomic<float> TargetAerodynamicUnloading{0.0f};
    TAtomic<float> TargetHighLoadDeformation{0.0f};
    TAtomic<float> TargetHighFrequencyGustMps{0.0f};
    float CurrentVario = 0.0f;
    float CurrentAirspeed = 0.0f;
    double TonePhase = 0.0;
    double LinePhase = 0.0;
    double GatePhase = 0.0;
    float WindFilterLeft = 0.0f;
    float WindFilterRight = 0.0f;
    float ThermalFilter = 0.0f;
    float PreviousCollapse = 0.0f;
    float FabricEnvelope = 0.0f;
    uint32 NoiseState = 0x715517u;
    int32 AudioSampleRate = 48000;
};
