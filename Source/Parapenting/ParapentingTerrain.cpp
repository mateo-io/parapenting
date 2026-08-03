#include "ParapentingTerrain.h"
#include "ParapentingMaterials.h"
#include "Physics/TerrainModel.h"
#include "Physics/TerrainRenderLayout.h"

#include "Components/SceneComponent.h"
#include "HAL/PlatformTime.h"
#include "Materials/Material.h"
#include "ProceduralMeshComponent.h"
#include "UObject/UObjectGlobals.h"

namespace
{
using Layout = Parapenting::Physics::TerrainRenderLayout;

// bBakeShading exists only for the unlit fallback material, which cannot
// receive light and therefore needs a fixed key direction baked into the
// vertex colour. With the lit material the renderer does this properly and
// baking it again would shade the terrain twice.
FLinearColor TerrainColour(double X, double Y, double Z,
                           const Parapenting::Physics::Vec3& N,
                           bool bBakeShading)
{
    // Heights in the simulation frame are relative to Lehn (565 m MSL), not
    // absolute elevations. Classify the landscape in MSL so Interlaken's
    // valley stays green while the upper shelves turn to alpine pasture.
    constexpr double LandingElevationMsl = 565.0;
    const double ElevationMsl = Z + LandingElevationMsl;
    const float HeightTint = FMath::Clamp(
        static_cast<float>((ElevationMsl - 1350.0) / 850.0), 0.0f, 1.0f);
    const float Steepness = FMath::Clamp(
        static_cast<float>((0.78 - N.z) / 0.30), 0.0f, 1.0f);
    const float BroadNoise = 0.5f + 0.5f * FMath::PerlinNoise2D(
        FVector2D(X * 0.00075, Y * 0.00075));
    const float DetailNoise = 0.5f + 0.5f * FMath::PerlinNoise2D(
        FVector2D(X * 0.011 + 17.0, Y * 0.011 - 9.0));
    const float MicroNoise = 0.5f + 0.5f * FMath::PerlinNoise2D(
        FVector2D(X * 0.041 - 31.0, Y * 0.041 + 23.0));
    const float FieldBands = 0.5f + 0.5f
        * FMath::Sin(static_cast<float>(X * 0.018 + Y * 0.006));
    const float ParcelX = FMath::Frac(
        static_cast<float>((X - Layout::xMinInterlakenM) / 145.0));
    const float ParcelY = FMath::Frac(
        static_cast<float>((Y - Layout::yMinInterlakenM) / 110.0));
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
    // Summer snow line in the Bernese Oberland sits around 2600-2900 m MSL,
    // and north-facing bowls hold it a few hundred metres lower.
    constexpr double SnowLineM = 2650.0;
    constexpr double NorthAspectDropM = 260.0;
    constexpr double SnowTransitionM = 340.0;
    const float SnowBlend = FMath::Clamp(
        static_cast<float>(
            (ElevationMsl - (SnowLineM - NorthAspectDropM * NorthAspect))
                / SnowTransitionM)
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

    const FLinearColor Meadow(0.12f, 0.48f, 0.055f);
    const FLinearColor FieldA(0.28f, 0.54f, 0.075f);
    const FLinearColor FieldB(0.11f, 0.43f, 0.035f);
    const FLinearColor ForestFloor(0.025f, 0.15f, 0.025f);
    const FLinearColor Alpine(0.24f, 0.38f, 0.12f);
    // Limestone retains lichen and scrub on ledges, but steep exposed faces
    // must remain visibly distinct from grass in the flight view.
    const FLinearColor RockA(0.25f, 0.265f, 0.22f);
    const FLinearColor RockB(0.39f, 0.37f, 0.30f);
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

    // Surface variation is albedo, not lighting, so it applies either way.
    Colour *= (0.91f + 0.13f * DetailNoise + 0.04f * MicroNoise);
    // Cavity darkening reads as ambient occlusion and is cheap to keep.
    Colour *= 0.92f + Curvature * 0.09f;

    if (bBakeShading)
    {
        const float SunExposure = FMath::Clamp(
            static_cast<float>(N.x * -0.48 + N.y * -0.25 + N.z * 0.84),
            0.35f, 1.0f);
        Colour *= 0.76f + 0.25f * SunExposure;
    }
    // RGB is terrain albedo; alpha is an authored surface-data channel. Keep
    // the snow decision beside the geospatial palette instead of duplicating
    // altitude/aspect thresholds in the material graph.
    Colour.A = SnowBlend;
    return Colour;
}
}

AParapentingTerrain::AParapentingTerrain()
{
    PrimaryActorTick.bCanEverTick = false;
    TerrainRoot = CreateDefaultSubobject<USceneComponent>(TEXT("TerrainRoot"));
    SetRootComponent(TerrainRoot);
    TerrainTiles.Reserve(Layout{}.TileCount());
}

void AParapentingTerrain::BeginPlay()
{
    Super::BeginPlay();
    // The origin is the Amisbuehl launch, so this is the Interlaken region.
    // A pawn starting on a Grindelwald route rebuilds on its first reset.
    BuildForRegionAt(0.0, 0.0);
}

void AParapentingTerrain::BuildForRegionAt(double XM, double YM)
{
    const Layout Region = Parapenting::Physics::LayoutFor(XM, YM);
    if (bHasBuilt && Region.xMinM == BuiltXMinM && Region.yMinM == BuiltYMinM)
        return;

    for (UProceduralMeshComponent* Tile : TerrainTiles)
        if (Tile) Tile->DestroyComponent();
    TerrainTiles.Reset();

    ActiveLayout = Region;
    BuiltXMinM = Region.xMinM;
    BuiltYMinM = Region.yMinM;
    bHasBuilt = true;
    ++BuildSerial;
    const double BuildStartSeconds = FPlatformTime::Seconds();
    BuildTerrainMesh();
    const double BuildMilliseconds =
        (FPlatformTime::Seconds() - BuildStartSeconds) * 1000.0;
    constexpr double RouteSwitchBudgetMilliseconds = 250.0;
    if (BuildMilliseconds <= RouteSwitchBudgetMilliseconds)
    {
        UE_LOG(LogTemp, Display,
            TEXT("Parapenting terrain rebuild: %.1f ms, %d tiles, %d "
                 "vertices (budget %.0f ms)"),
            BuildMilliseconds, ActiveLayout.TileCount(),
            ActiveLayout.TotalVertices(), RouteSwitchBudgetMilliseconds);
    }
    else
    {
        UE_LOG(LogTemp, Warning,
            TEXT("Parapenting terrain rebuild exceeded budget: %.1f ms, "
                 "%d tiles, %d vertices (budget %.0f ms)"),
            BuildMilliseconds, ActiveLayout.TileCount(),
            ActiveLayout.TotalVertices(), RouteSwitchBudgetMilliseconds);
    }
}

void AParapentingTerrain::BuildTerrainMesh()
{
    UMaterialInterface* VertexMaterial =
        Parapenting::LoadTerrainMaterial();
    const bool bBakeShading = !Parapenting::bVertexColourMaterialIsLit;

    const Layout& Active = ActiveLayout;
    const int32 VertexSide = Active.cellsPerTile + 1;
    const double TileWidth =
        (Active.xMaxM - Active.xMinM) / Active.tileCountX;
    const double TileHeight =
        (Active.yMaxM - Active.yMinM) / Active.tileCountY;

    for (int32 TileX = 0; TileX < Active.tileCountX; ++TileX)
    {
        for (int32 TileY = 0; TileY < Active.tileCountY; ++TileY)
        {
            const FName TileName(*FString::Printf(
                TEXT("TerrainTile_%d_%02d_%02d"), BuildSerial, TileX, TileY));
            UProceduralMeshComponent* Tile =
                NewObject<UProceduralMeshComponent>(this, TileName);
            Tile->SetupAttachment(TerrainRoot);
            Tile->bUseAsyncCooking = false;
            Tile->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            Tile->SetCastShadow(true);
            Tile->RegisterComponent();

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
                Active.cellsPerTile * Active.cellsPerTile * 6);

            for (int32 LocalX = 0; LocalX <= Active.cellsPerTile; ++LocalX)
            {
                const double X = Active.xMinM + TileWidth
                    * (TileX + static_cast<double>(LocalX)
                        / Active.cellsPerTile);
                for (int32 LocalY = 0;
                     LocalY <= Active.cellsPerTile; ++LocalY)
                {
                    const double Y = Active.yMinM + TileHeight
                        * (TileY + static_cast<double>(LocalY)
                            / Active.cellsPerTile);
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
                    // Vertex colours are read raw by the material; the engine
                    // FColor is an 8-bit storage format and the material's
                    // VertexColor expression converts those stored sRGB
                    // values back to linear space. Encode the authored linear
                    // albedo here; writing raw linear bytes crushes meadow
                    // green (0.48 becomes roughly 0.19 after decoding) and
                    // makes the whole range read blue-black.
                    Colours.Add(
                        TerrainColour(X, Y, Z, N, bBakeShading)
                            .ToFColor(true));
                }
            }

            for (int32 LocalX = 0; LocalX < Active.cellsPerTile; ++LocalX)
            {
                for (int32 LocalY = 0;
                     LocalY < Active.cellsPerTile; ++LocalY)
                {
                    const int32 A = LocalX * VertexSide + LocalY;
                    const int32 B = (LocalX + 1) * VertexSide + LocalY;
                    const int32 C = B + 1;
                    const int32 D = A + 1;
                    // Unreal's procedural-mesh front face is clockwise in
                    // this local X/Y frame. The previous counter-clockwise
                    // order made the entire terrain back-facing: a one-sided
                    // material showed sky through distant slopes, while a
                    // two-sided diagnostic rendered their unlit backs black.
                    Triangles.Append({A, C, B, A, D, C});
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
