#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "ParapentingGameMode.generated.h"

UCLASS()
class PARAPENTING_API AParapentingGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    AParapentingGameMode();
    virtual void InitGame(
        const FString& MapName, const FString& Options,
        FString& ErrorMessage) override;
    virtual void Tick(float DeltaSeconds) override;

protected:
    virtual void BeginPlay() override;

private:
    void StartVisualQACapture(class AParagliderPawn& Glider);
    void SaveVisualQACapture(
        int32 Width, int32 Height, const TArray<FColor>& Colors);
    void FinishVisualQACapture();

    bool bLoadedSurveyedTerrain = false;
    bool bVisualQACaptureRequested = false;
    bool bVisualQATimeApplied = false;
    bool bVisualQAScreenshotRequested = false;
    float VisualQAWarmupSeconds = 4.0f;
    double VisualQALocalHour = 13.0;
    FString VisualQACaptureName;
    FString VisualQACapturePath;

    UPROPERTY()
    TObjectPtr<class UVolumetricCloudComponent> CloudComponent;

    UPROPERTY()
    TObjectPtr<class UDirectionalLightComponent> SunComponent;

    UPROPERTY()
    TObjectPtr<class USkyLightComponent> SkyLightComponent;

    UPROPERTY()
    TObjectPtr<class UMaterialInstanceDynamic> CloudMaterialInstance;

    UPROPERTY()
    TArray<TObjectPtr<class UStaticMeshComponent>> WindsockComponents;

    TArray<FVector> WindsockAnchorsCm;
    TArray<FVector2D> WindsockSamplePositionsM;
};
