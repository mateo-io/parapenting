#include "ParapentingTerrain.h"
#include "Physics/TerrainModel.h"
#include "Physics/TerrainRenderLayout.h"

#include "Components/SceneComponent.h"
#include "Materials/Material.h"
#include "ProceduralMeshComponent.h"
#include "UObject/UObjectGlobals.h"

namespace
{
using Layout = Parapenting::Physics::TerrainRenderLayout;

FLinearColor TerrainColour(double X, double Y, double Z,
                           const Parapenting::Physics::Vec3& N)
{
    const float HeightTint = FMath::Clamp(
        static_cast<float>((Z - 80.0) / 1200.0), 0.0f, 1.0f);
    const float Steepness = FMath::Clamp(
        static_cast<float>(1.0 - N.z) * 3.1f, 0.0f, 1.0f);
    const float BroadNoise = 0.5f + 0.5f * FMath::PerlinNoise2D(
        FVector2D(X * 0.00075, Y * 0.00075));
    const float DetailNoise = 0.5f + 0.5f * FMath::PerlinNoise2D(
        FVector2D(X * 0.011 + 17.0, Y * 0.011 - 9.0));
    const float MicroNoise = 0.5f + 0.5f * FMath::PerlinNoise2D(
        FVector2D(X * 0.041 - 31.0, Y * 0.041 + 23.0));
    const float FieldBands = 0.5f + 0.5f
        * FMath::Sin(static_cast<float>(X * 0.018 + Y * 0.006));
    const float ParcelX = FMath::Frac(
        static_cast<float>((X - Layout::xMinM) / 145.0));
    const float ParcelY = FMath::Frac(
        static_cast<float>((Y - Layout::yMinM) / 110.0));
    const float FieldBoundary =
        (ParcelX < 0.025f || ParcelY < 0.035f) ? 1.0f : 0.0f;
    const float ValleyField = FMath::Clamp(
        static_cast<float>((260.0 - Z) / 220.0)
            * static_cast<float>((N.z - 0.88) / 0.10),
        0.0f, 1.0f);
    const float ForestBlend = FMath::Clamp(
        (BroadNoise - 0.44f) * 2.8f
            * (1.0f - Steepness) * (1.0f - ValleyField * 0.65f),
        0.0f, 1.0f);
    // North-facing bowls retain snow lower than sun-exposed faces.
    const float NorthAspect = FMath::Clamp(
        static_cast<float>(0.5 - 0.5 * N.y), 0.0f, 1.0f);
    const float SnowBlend = FMath::Clamp(
        static_cast<float>((Z - (1160.0 - 150.0 * NorthAspect)) / 260.0)
            * (0.68f + 0.32f * static_cast<float>(N.z)),
        0.0f, 1.0f);

    // A small surveyed-height Laplacian adds contact-like depth in gullies
    // and keeps convex ridges brighter without baking a fixed sun direction.
    constexpr double CurvatureStepM = 32.0;
    const double NeighbourMean = 0.25 * (
        Parapenting::Physics::TerrainModel::HeightM(X + CurvatureStepM, Y)
        + Parapenting::Physics::TerrainModel::HeightM(X - CurvatureStepM, Y)
        + Parapenting::Physics::TerrainModel::HeightM(X, Y + CurvatureStepM)
        + Parapenting::Physics::TerrainModel::HeightM(X, Y - CurvatureStepM));
    const float Curvature = FMath::Clamp(
        static_cast<float>((Z - NeighbourMean) / 14.0), -1.0f, 1.0f);
    const float RockStrata = 0.5f + 0.5f * FMath::Sin(
        static_cast<float>(Z * 0.105 + X * 0.008 - Y * 0.004));

    const FLinearColor Meadow(0.10f, 0.42f, 0.055f);
    const FLinearColor FieldA(0.24f, 0.48f, 0.075f);
    const FLinearColor FieldB(0.10f, 0.37f, 0.040f);
    const FLinearColor ForestFloor(0.035f, 0.12f, 0.035f);
    const FLinearColor Alpine(0.30f, 0.34f, 0.18f);
    const FLinearColor RockA(0.30f, 0.285f, 0.265f);
    const FLinearColor RockB(0.39f, 0.37f, 0.34f);
    const FLinearColor Snow(0.80f, 0.84f, 0.86f);

    FLinearColor Colour = FMath::Lerp(Meadow, Alpine, HeightTint);
    Colour = FMath::Lerp(
        Colour, FMath::Lerp(FieldA, FieldB, FieldBands),
        ValleyField * 0.78f);
    Colour = FMath::Lerp(
        Colour, FLinearColor(0.055f, 0.16f, 0.035f),
        FieldBoundary * ValleyField * 0.65f);
    Colour = FMath::Lerp(Colour, ForestFloor, ForestBlend * 0.72f);
    Colour = FMath::Lerp(
        Colour, FMath::Lerp(RockA, RockB, RockStrata),
        Steepness * (0.72f + 0.28f * DetailNoise));
    Colour = FMath::Lerp(Colour, Snow, SnowBlend);

    const float SunExposure = FMath::Clamp(
        static_cast<float>(N.x * -0.48 + N.y * -0.25 + N.z * 0.84),
        0.35f, 1.0f);
    const float CurvatureLight = 0.92f + Curvature * 0.09f;
    Colour *= (0.76f + 0.25f * SunExposure)
        * (0.91f + 0.13f * DetailNoise + 0.04f * MicroNoise)
        * CurvatureLight;
    return Colour;
}
}

AParapentingTerrain::AParapentingTerrain()
{
    TerrainRoot = CreateDefaultSubobject<USceneComponent>(TEXT("TerrainRoot"));
    SetRootComponent(TerrainRoot);
    TerrainTiles.Reserve(Layout::TileCount());

    UMaterial* VertexMaterial = LoadObject<UMaterial>(
        nullptr,
        TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));

    const int32 VertexSide = Layout::cellsPerTile + 1;
    const double TileWidth =
        (Layout::xMaxM - Layout::xMinM) / Layout::tileCountX;
    const double TileHeight =
        (Layout::yMaxM - Layout::yMinM) / Layout::tileCountY;

    for (int32 TileX = 0; TileX < Layout::tileCountX; ++TileX)
    {
        for (int32 TileY = 0; TileY < Layout::tileCountY; ++TileY)
        {
            const FName TileName(*FString::Printf(
                TEXT("TerrainTile_%02d_%02d"), TileX, TileY));
            UProceduralMeshComponent* Tile =
                CreateDefaultSubobject<UProceduralMeshComponent>(TileName);
            Tile->SetupAttachment(TerrainRoot);
            Tile->bUseAsyncCooking = false;
            Tile->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            Tile->SetCastShadow(true);

            TArray<FVector> Vertices;
            TArray<int32> Triangles;
            TArray<FVector> Normals;
            TArray<FVector2D> UVs;
            TArray<FColor> Colours;
            TArray<FProcMeshTangent> Tangents;
            Vertices.Reserve(VertexSide * VertexSide);
            Normals.Reserve(VertexSide * VertexSide);
            UVs.Reserve(VertexSide * VertexSide);
            Colours.Reserve(VertexSide * VertexSide);
            Tangents.Reserve(VertexSide * VertexSide);
            Triangles.Reserve(
                Layout::cellsPerTile * Layout::cellsPerTile * 6);

            for (int32 LocalX = 0; LocalX <= Layout::cellsPerTile; ++LocalX)
            {
                const double X = Layout::xMinM + TileWidth
                    * (TileX + static_cast<double>(LocalX)
                        / Layout::cellsPerTile);
                for (int32 LocalY = 0;
                     LocalY <= Layout::cellsPerTile; ++LocalY)
                {
                    const double Y = Layout::yMinM + TileHeight
                        * (TileY + static_cast<double>(LocalY)
                            / Layout::cellsPerTile);
                    const double Z =
                        Parapenting::Physics::TerrainModel::HeightM(X, Y);
                    const auto N =
                        Parapenting::Physics::TerrainModel::Normal(X, Y);
                    const FVector Normal(N.x, N.y, N.z);
                    const FVector Tangent = (
                        FVector::ForwardVector
                        - Normal * FVector::DotProduct(
                            FVector::ForwardVector, Normal)).GetSafeNormal();
                    Vertices.Add(FVector(X, Y, Z) * 100.0);
                    Normals.Add(Normal);
                    Tangents.Add(FProcMeshTangent(Tangent, false));
                    UVs.Add(FVector2D(X / 48.0, Y / 48.0));
                    Colours.Add(TerrainColour(X, Y, Z, N).ToFColor(true));
                }
            }

            for (int32 LocalX = 0; LocalX < Layout::cellsPerTile; ++LocalX)
            {
                for (int32 LocalY = 0;
                     LocalY < Layout::cellsPerTile; ++LocalY)
                {
                    const int32 A = LocalX * VertexSide + LocalY;
                    const int32 B = (LocalX + 1) * VertexSide + LocalY;
                    const int32 C = B + 1;
                    const int32 D = A + 1;
                    Triangles.Append({A, B, C, A, C, D});
                }
            }

            Tile->CreateMeshSection(
                0, Vertices, Triangles, Normals, UVs, Colours, Tangents, false);
            if (VertexMaterial)
                Tile->SetMaterial(0, VertexMaterial);
            TerrainTiles.Add(Tile);
        }
    }
}
