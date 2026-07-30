#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ParapentingTerrain.generated.h"

class UProceduralMeshComponent;
class USceneComponent;

UCLASS()
class PARAPENTING_API AParapentingTerrain : public AActor
{
    GENERATED_BODY()

public:
    AParapentingTerrain();

private:
    UPROPERTY()
    USceneComponent* TerrainRoot;

    UPROPERTY()
    TArray<UProceduralMeshComponent*> TerrainTiles;
};
