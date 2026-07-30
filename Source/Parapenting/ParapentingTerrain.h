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

protected:
    virtual void BeginPlay() override;

private:
    // Sampling the heightfield and building the tile meshes. Deliberately not
    // called from the constructor: constructors also run for the class default
    // object at module load, which is long before InitGame reads the surveyed
    // .asc, so a constructor-time build silently bakes the analytic fallback
    // and then throws it away.
    void BuildTerrainMesh();

    UPROPERTY()
    USceneComponent* TerrainRoot;

    UPROPERTY()
    TArray<UProceduralMeshComponent*> TerrainTiles;
};
