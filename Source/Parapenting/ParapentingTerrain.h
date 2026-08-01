#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Physics/TerrainRenderLayout.h"
#include "ParapentingTerrain.generated.h"

class UProceduralMeshComponent;
class USceneComponent;

UCLASS()
class PARAPENTING_API AParapentingTerrain : public AActor
{
    GENERATED_BODY()

public:
    AParapentingTerrain();

    // Draw the surveyed region containing this local position, rebuilding the
    // tile meshes if it is not the region already drawn. Interlaken and
    // Grindelwald are 20 km apart, so only the one being flown is built - the
    // ground between them is never seen and would cost the whole vertex
    // budget. Called on every flight reset, which is where route changes land.
    void BuildForRegionAt(double XM, double YM);

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

    // The region currently drawn. Defaults to Interlaken, which is what the
    // eight Interlaken routes have always rendered.
    Parapenting::Physics::TerrainRenderLayout ActiveLayout;

    // Bounds of the region currently built, so a reset that stays in the same
    // region does not rebuild sixty-four procedural meshes for nothing.
    double BuiltXMinM = 0.0;
    double BuiltYMinM = 0.0;
    bool bHasBuilt = false;
    // Component names must be unique within this actor, and the previous
    // build's components are only pending destruction rather than gone, so a
    // rebuild cannot reuse TerrainTile_00_00.
    int32 BuildSerial = 0;
};
