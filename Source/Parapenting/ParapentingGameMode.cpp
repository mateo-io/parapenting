#include "ParapentingGameMode.h"
#include "ParapentingHUD.h"
#include "ParagliderPawn.h"
#include "ParapentingTerrain.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/VolumetricCloudComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "EngineUtils.h"
#include "Physics/TerrainModel.h"
#include "Physics/RouteCatalogue.h"
#include "Physics/WindsockModel.h"
#include "UObject/ConstructorHelpers.h"
#include "Misc/Paths.h"

AParapentingGameMode::AParapentingGameMode()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickInterval = 0.2f;
    DefaultPawnClass = AParagliderPawn::StaticClass();
    HUDClass = AParapentingHUD::StaticClass();
}

void AParapentingGameMode::InitGame(
    const FString& MapName, const FString& Options, FString& ErrorMessage)
{
    Super::InitGame(MapName, Options, ErrorMessage);

    // InitGame runs before the default pawn is spawned. Loading the
    // authoritative terrain here guarantees the pawn's BeginPlay/ResetFlight
    // queries the surveyed launch elevation rather than the analytic fallback.
    const FString HeightfieldPath = FPaths::Combine(
        FPaths::ProjectContentDir(), TEXT("Terrain/interlaken.asc"));
    bLoadedSurveyedTerrain =
        Parapenting::Physics::TerrainModel::LoadHeightfieldAscii(
            TCHAR_TO_UTF8(*HeightfieldPath));
    UE_LOG(LogTemp, Display, TEXT("Parapenting terrain source: %s (%s)"),
        bLoadedSurveyedTerrain
            ? TEXT("surveyed swissALTI3D") : TEXT("analytic fallback"),
        *HeightfieldPath);
}

void AParapentingGameMode::BeginPlay()
{
    Super::BeginPlay();

    UWorld* World = GetWorld();
    if (!World) return;

    // The map may contain editor-authored preview suns. Forward shading can
    // only choose one primary directional light, so remove those before
    // creating the single simulation-controlled atmosphere sun.
    for (TActorIterator<ADirectionalLight> It(World); It; ++It)
        It->Destroy();
    for (TActorIterator<ASkyLight> It(World); It; ++It)
        It->Destroy();
    for (TActorIterator<AExponentialHeightFog> It(World); It; ++It)
        It->Destroy();
    for (TActorIterator<ASkyAtmosphere> It(World); It; ++It)
        It->Destroy();
    ADirectionalLight* Sun = World->SpawnActor<ADirectionalLight>(
        FVector::ZeroVector, FRotator(-38.0, -28.0, 0.0));
    Sun->GetLightComponent()->SetIntensity(7.5f);
    Sun->GetLightComponent()->SetLightColor(FLinearColor(1.0f, 0.98f, 0.94f));
    Sun->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    if (UDirectionalLightComponent* SpawnedSunComponent =
        Cast<UDirectionalLightComponent>(Sun->GetLightComponent()))
    {
        SunComponent = SpawnedSunComponent;
        SpawnedSunComponent->SetAtmosphereSunLight(true);
        SpawnedSunComponent->SetLightSourceAngle(1.2f);
        SpawnedSunComponent->SetDynamicShadowDistanceMovableLight(1800000.0f);
        SpawnedSunComponent->bCastCloudShadows = true;
        SpawnedSunComponent->CloudShadowStrength = 0.22f;
        SpawnedSunComponent->CloudShadowOnSurfaceStrength = 0.22f;
    }

    ASkyLight* Sky = World->SpawnActor<ASkyLight>();
    Sky->GetLightComponent()->SetIntensity(0.42f);
    Sky->GetLightComponent()->SetRealTimeCapture(true);
    Sky->GetLightComponent()->SetMobility(EComponentMobility::Movable);

    ASkyAtmosphere* AtmosphereActor = World->SpawnActor<ASkyAtmosphere>();
    USkyAtmosphereComponent* Atmosphere = AtmosphereActor->GetComponent();
    Atmosphere->SetRayleighScatteringScale(1.25f);
    Atmosphere->SetMieScatteringScale(0.22f);
    Atmosphere->SetMieAnisotropy(0.72f);

    AActor* CloudActor = World->SpawnActor<AActor>();
    UVolumetricCloudComponent* Clouds =
        NewObject<UVolumetricCloudComponent>(CloudActor, TEXT("AlpineClouds"));
    CloudComponent = Clouds;
    CloudActor->SetRootComponent(Clouds);
    Clouds->SetLayerBottomAltitude(2.2f);
    Clouds->SetLayerHeight(0.7f);
    if (UMaterialInterface* CloudMaterial = LoadObject<UMaterialInterface>(
        nullptr, TEXT("/Engine/EngineSky/VolumetricClouds/m_SimpleVolumetricCloud.m_SimpleVolumetricCloud")))
    {
        CloudMaterialInstance =
            UMaterialInstanceDynamic::Create(CloudMaterial, CloudActor);
        Clouds->SetMaterial(CloudMaterialInstance);
    }
    Clouds->RegisterComponent();

    AExponentialHeightFog* Fog = World->SpawnActor<AExponentialHeightFog>();
    Fog->GetComponent()->SetFogDensity(0.00025f);
    Fog->GetComponent()->SetFogHeightFalloff(0.12f);
    Fog->GetComponent()->SetFogInscatteringColor(
        FLinearColor(0.55f, 0.72f, 0.88f));
    Fog->GetComponent()->SetStartDistance(18000.0f);
    Fog->GetComponent()->SetVolumetricFog(false);

    UStaticMesh* CylinderMesh = LoadObject<UStaticMesh>(
        nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    UStaticMesh* ConeMesh = LoadObject<UStaticMesh>(
        nullptr, TEXT("/Engine/BasicShapes/Cone.Cone"));
    UStaticMesh* PlaneMesh = LoadObject<UStaticMesh>(
        nullptr, TEXT("/Engine/BasicShapes/Plane.Plane"));
    UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(
        nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(
        nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    UMaterialInterface* ShapeMaterial = LoadObject<UMaterialInterface>(
        nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));

    World->SpawnActor<AParapentingTerrain>();

    if (ConeMesh && CylinderMesh)
    {
        AActor* Forest = World->SpawnActor<AActor>();
        UHierarchicalInstancedStaticMeshComponent* Crowns =
            NewObject<UHierarchicalInstancedStaticMeshComponent>(Forest, TEXT("ForestCrowns"));
        UHierarchicalInstancedStaticMeshComponent* Trunks =
            NewObject<UHierarchicalInstancedStaticMeshComponent>(Forest, TEXT("ForestTrunks"));
        UHierarchicalInstancedStaticMeshComponent* Deciduous =
            NewObject<UHierarchicalInstancedStaticMeshComponent>(
                Forest, TEXT("DeciduousCrowns"));
        Forest->SetRootComponent(Crowns);
        Crowns->SetStaticMesh(ConeMesh);
        Trunks->SetupAttachment(Crowns);
        Trunks->SetStaticMesh(CylinderMesh);
        Deciduous->SetupAttachment(Crowns);
        Deciduous->SetStaticMesh(SphereMesh);
        Crowns->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Trunks->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Deciduous->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Crowns->SetCullDistances(120000, 950000);
        Trunks->SetCullDistances(90000, 650000);
        Deciduous->SetCullDistances(100000, 750000);
        Crowns->RegisterComponent();
        Trunks->RegisterComponent();
        Deciduous->RegisterComponent();
        if (ShapeMaterial)
        {
            UMaterialInstanceDynamic* CrownMaterial =
                UMaterialInstanceDynamic::Create(ShapeMaterial, Forest);
            CrownMaterial->SetVectorParameterValue(
                TEXT("Color"), FLinearColor(0.035f, 0.16f, 0.045f));
            Crowns->SetMaterial(0, CrownMaterial);
            UMaterialInstanceDynamic* TrunkMaterial =
                UMaterialInstanceDynamic::Create(ShapeMaterial, Forest);
            TrunkMaterial->SetVectorParameterValue(
                TEXT("Color"), FLinearColor(0.12f, 0.055f, 0.018f));
            Trunks->SetMaterial(0, TrunkMaterial);
            UMaterialInstanceDynamic* DeciduousMaterial =
                UMaterialInstanceDynamic::Create(ShapeMaterial, Forest);
            DeciduousMaterial->SetVectorParameterValue(
                TEXT("Color"), FLinearColor(0.075f, 0.24f, 0.045f));
            Deciduous->SetMaterial(0, DeciduousMaterial);
        }

        for (int32 XIndex = -10; XIndex <= 48; ++XIndex)
        {
            for (int32 YIndex = -18; YIndex <= 18; ++YIndex)
            {
                const float X = XIndex * 85.0f
                    + FMath::Sin(XIndex * 9.17f + YIndex * 2.31f) * 28.0f;
                const float Y = YIndex * 95.0f
                    + FMath::Sin(YIndex * 5.73f - XIndex * 1.61f) * 33.0f;
                const float CorridorWidth = X < 850.0f ? 230.0f : 520.0f;
                if (FMath::Abs(Y) < CorridorWidth) continue;
                const float Density = FMath::PerlinNoise2D(
                    FVector2D(X * 0.0031f, Y * 0.0031f));
                if (Density < -0.12f) continue;
                const float Ground = static_cast<float>(
                    Parapenting::Physics::TerrainModel::HeightM(X, Y));
                const auto GroundNormal =
                    Parapenting::Physics::TerrainModel::Normal(X, Y);
                // Keep water, landing basins and exposed steep faces readable.
                if (Ground < 12.0f || GroundNormal.z < 0.72) continue;
                const float HeightScale = 0.75f + 0.35f
                    * FMath::Abs(FMath::Sin(X * 0.017f + Y * 0.013f));
                const bool bDeciduous =
                    Ground < 480.0f && Density > 0.18f && SphereMesh;
                if (bDeciduous)
                {
                    Deciduous->AddInstance(FTransform(
                        FRotator::ZeroRotator,
                        FVector(X, Y, Ground + 8.0f) * 100.0f,
                        FVector(4.6f, 4.6f, 5.8f) * HeightScale));
                }
                else
                {
                    Crowns->AddInstance(FTransform(
                        FRotator::ZeroRotator,
                        FVector(X, Y, Ground + 8.5f) * 100.0f,
                        FVector(2.8f, 2.8f, 9.5f) * HeightScale));
                }
                Trunks->AddInstance(FTransform(
                    FRotator::ZeroRotator,
                    FVector(X, Y, Ground + 3.8f) * 100.0f,
                    FVector(0.45f, 0.45f, 7.6f) * HeightScale));
            }
        }
        // Sparse Grindelwald regional lane. It shares the global orientation
        // but is translated north so real wind bearings remain valid.
        for (int32 XIndex = -4; XIndex <= 56; ++XIndex)
        {
            for (int32 YIndex = 58; YIndex <= 98; ++YIndex)
            {
                const float X = XIndex * 85.0f
                    + FMath::Sin(XIndex * 7.31f + YIndex) * 24.0f;
                const float Y = YIndex * 90.0f
                    + FMath::Sin(YIndex * 3.97f - XIndex) * 29.0f;
                const float RouteY = 8500.0f - 0.547f * X;
                if (FMath::Abs(Y - RouteY) < 420.0f) continue;
                const float Density = FMath::PerlinNoise2D(
                    FVector2D(X * 0.0037f + 12.0f, Y * 0.0037f));
                if (Density < -0.08f) continue;
                const float Ground = static_cast<float>(
                    Parapenting::Physics::TerrainModel::HeightM(X, Y));
                const auto Normal =
                    Parapenting::Physics::TerrainModel::Normal(X, Y);
                if (Ground < 430.0f || Normal.z < 0.70) continue;
                const float Scale = 0.72f + 0.32f
                    * FMath::Abs(FMath::Sin(X * 0.014f + Y * 0.009f));
                Crowns->AddInstance(FTransform(
                    FRotator::ZeroRotator,
                    FVector(X, Y, Ground + 8.5f) * 100.0f,
                    FVector(2.8f, 2.8f, 9.5f) * Scale));
                Trunks->AddInstance(FTransform(
                    FRotator::ZeroRotator,
                    FVector(X, Y, Ground + 3.8f) * 100.0f,
                    FVector(0.45f, 0.45f, 7.6f) * Scale));
            }
        }
    }

    // Low-cost valley settlements give speed/height cues and make the Lehn
    // and Interlaken basins readable from approach altitude.
    if (CubeMesh && ShapeMaterial)
    {
        AActor* Settlements = World->SpawnActor<AActor>();
        UHierarchicalInstancedStaticMeshComponent* Buildings =
            NewObject<UHierarchicalInstancedStaticMeshComponent>(
                Settlements, TEXT("ValleyBuildings"));
        Settlements->SetRootComponent(Buildings);
        UHierarchicalInstancedStaticMeshComponent* Roofs =
            NewObject<UHierarchicalInstancedStaticMeshComponent>(
                Settlements, TEXT("ValleyRoofs"));
        Buildings->SetStaticMesh(CubeMesh);
        Roofs->SetupAttachment(Buildings);
        Roofs->SetStaticMesh(CubeMesh);
        Buildings->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Buildings->SetCullDistances(120000, 900000);
        Roofs->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Roofs->SetCullDistances(120000, 900000);
        Buildings->RegisterComponent();
        Roofs->RegisterComponent();
        UMaterialInstanceDynamic* BuildingMaterial =
            UMaterialInstanceDynamic::Create(ShapeMaterial, Settlements);
        BuildingMaterial->SetVectorParameterValue(
            TEXT("Color"), FLinearColor(0.52f, 0.45f, 0.36f));
        Buildings->SetMaterial(0, BuildingMaterial);
        UMaterialInstanceDynamic* RoofMaterial =
            UMaterialInstanceDynamic::Create(ShapeMaterial, Settlements);
        RoofMaterial->SetVectorParameterValue(
            TEXT("Color"), FLinearColor(0.24f, 0.075f, 0.035f));
        Roofs->SetMaterial(0, RoofMaterial);

        for (int32 Block = 0; Block < 170; ++Block)
        {
            const float X = 2050.0f + (Block % 34) * 105.0f
                + FMath::Sin(Block * 2.17f) * 28.0f;
            const float Y = -720.0f + (Block / 34) * 330.0f
                + FMath::Sin(Block * 4.71f) * 72.0f;
            const float Ground = static_cast<float>(
                Parapenting::Physics::TerrainModel::HeightM(X, Y));
            const auto Normal =
                Parapenting::Physics::TerrainModel::Normal(X, Y);
            if (Ground > 170.0f || Normal.z < 0.94) continue;
            const float BuildingHeight = 7.0f + 2.4f * (Block % 4);
            const float Width = 7.0f + 1.6f * (Block % 3);
            const float Depth = 5.0f + 1.2f * ((Block / 3) % 3);
            const float Yaw = static_cast<float>((Block * 37) % 180);
            Buildings->AddInstance(FTransform(
                FRotator(0.0f, Yaw, 0.0f),
                FVector(X, Y, Ground + BuildingHeight * 0.5f) * 100.0f,
                FVector(Width, Depth, BuildingHeight)));
            Roofs->AddInstance(FTransform(
                FRotator(0.0f, Yaw, 45.0f),
                FVector(X, Y, Ground + BuildingHeight + 1.2f) * 100.0f,
                FVector(Width * 0.78f, Depth * 0.78f, 1.8f)));
        }
        for (int32 Block = 0; Block < 120; ++Block)
        {
            const bool bBodmi = Block >= 80;
            const int32 Local = bBodmi ? Block - 80 : Block;
            const float X = (bBodmi ? 2860.0f : 3650.0f)
                + (Local % 10) * 72.0f
                + FMath::Sin(Block * 2.41f) * 18.0f;
            const float Y = (bBodmi ? 7180.0f : 6070.0f)
                + (Local / 10) * 88.0f
                + FMath::Sin(Block * 3.19f) * 24.0f;
            const float Ground = static_cast<float>(
                Parapenting::Physics::TerrainModel::HeightM(X, Y));
            const auto Normal =
                Parapenting::Physics::TerrainModel::Normal(X, Y);
            if (Normal.z < 0.88) continue;
            const float Height = 6.0f + 1.8f * (Block % 4);
            const float Width = 6.0f + 1.2f * (Block % 3);
            const float Depth = 5.0f + 1.1f * ((Block / 3) % 3);
            const float Yaw = static_cast<float>((Block * 29) % 180);
            Buildings->AddInstance(FTransform(
                FRotator(0.0f, Yaw, 0.0f),
                FVector(X, Y, Ground + Height * 0.5f) * 100.0f,
                FVector(Width, Depth, Height)));
            Roofs->AddInstance(FTransform(
                FRotator(0.0f, Yaw, 45.0f),
                FVector(X, Y, Ground + Height + 1.0f) * 100.0f,
                FVector(Width * 0.78f, Depth * 0.78f, 1.6f)));
        }
    }

    if (PlaneMesh && ShapeMaterial)
    {
        // Lake Thun lies route-right/west of the southbound Amisbuehl line.
        // Its 558 m MSL surface is approximately -7 m in the Lehn datum.
        AStaticMeshActor* Lake = World->SpawnActor<AStaticMeshActor>(
            FVector(70000.0, -145000.0, -680.0), FRotator::ZeroRotator);
        Lake->GetStaticMeshComponent()->SetStaticMesh(PlaneMesh);
        Lake->GetStaticMeshComponent()->SetWorldScale3D(
            FVector(3100.0f, 1200.0f, 1.0f));
        Lake->GetStaticMeshComponent()->SetCollisionEnabled(
            ECollisionEnabled::NoCollision);
        UMaterialInstanceDynamic* WaterMaterial =
            UMaterialInstanceDynamic::Create(ShapeMaterial, Lake);
        WaterMaterial->SetVectorParameterValue(
            TEXT("Color"), FLinearColor(0.018f, 0.19f, 0.31f, 1.0f));
        Lake->GetStaticMeshComponent()->SetMaterial(0, WaterMaterial);

        // The Aare and the main valley road form strong visual references
        // during the final glide into Interlaken and Lehn.
        AActor* ValleyLines = World->SpawnActor<AActor>();
        UHierarchicalInstancedStaticMeshComponent* River =
            NewObject<UHierarchicalInstancedStaticMeshComponent>(
                ValleyLines, TEXT("AareRiver"));
        UHierarchicalInstancedStaticMeshComponent* Road =
            NewObject<UHierarchicalInstancedStaticMeshComponent>(
                ValleyLines, TEXT("ValleyRoad"));
        ValleyLines->SetRootComponent(River);
        River->SetStaticMesh(CubeMesh);
        Road->SetupAttachment(River);
        Road->SetStaticMesh(CubeMesh);
        River->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Road->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        River->SetCullDistances(80000, 1200000);
        Road->SetCullDistances(60000, 900000);
        River->RegisterComponent();
        Road->RegisterComponent();
        UMaterialInstanceDynamic* RiverMaterial =
            UMaterialInstanceDynamic::Create(ShapeMaterial, ValleyLines);
        RiverMaterial->SetVectorParameterValue(
            TEXT("Color"), FLinearColor(0.025f, 0.28f, 0.42f));
        River->SetMaterial(0, RiverMaterial);
        UMaterialInstanceDynamic* RoadMaterial =
            UMaterialInstanceDynamic::Create(ShapeMaterial, ValleyLines);
        RoadMaterial->SetVectorParameterValue(
            TEXT("Color"), FLinearColor(0.12f, 0.125f, 0.12f));
        Road->SetMaterial(0, RoadMaterial);
        for (int32 Segment = 0; Segment < 52; ++Segment)
        {
            const float X = 1500.0f + Segment * 92.0f;
            const float RiverY = -610.0f
                + 125.0f * FMath::Sin(Segment * 0.21f);
            const float NextY = -610.0f
                + 125.0f * FMath::Sin((Segment + 1) * 0.21f);
            const float RiverYaw = FMath::RadiansToDegrees(
                FMath::Atan2(NextY - RiverY, 92.0f));
            const float RiverGround = static_cast<float>(
                Parapenting::Physics::TerrainModel::HeightM(X, RiverY));
            River->AddInstance(FTransform(
                FRotator(0.0f, RiverYaw, 0.0f),
                FVector(X, RiverY, RiverGround + 0.35f) * 100.0f,
                FVector(96.0f, 13.0f, 0.22f)));

            const float RoadY = 310.0f
                + 70.0f * FMath::Sin(Segment * 0.17f + 1.4f);
            const float NextRoadY = 310.0f
                + 70.0f * FMath::Sin((Segment + 1) * 0.17f + 1.4f);
            const float RoadYaw = FMath::RadiansToDegrees(
                FMath::Atan2(NextRoadY - RoadY, 92.0f));
            const float RoadGround = static_cast<float>(
                Parapenting::Physics::TerrainModel::HeightM(X, RoadY));
            Road->AddInstance(FTransform(
                FRotator(0.0f, RoadYaw, 0.0f),
                FVector(X, RoadY, RoadGround + 0.42f) * 100.0f,
                FVector(96.0f, 5.2f, 0.18f)));
        }
    }

    if (CylinderMesh && ConeMesh && ShapeMaterial)
    {
        TSet<FString> MarkedLandings;
        for (std::size_t Index = 0;
             Index < Parapenting::Physics::RouteProfileCount(); ++Index)
        {
            const auto& Route =
                Parapenting::Physics::GetRouteProfileByIndex(Index);
            const FString LandingId = ANSI_TO_TCHAR(Route.landing.id);
            if (MarkedLandings.Contains(LandingId)) continue;
            MarkedLandings.Add(LandingId);
            const auto Position =
                Parapenting::Physics::RouteLandingLocalM(Route);
            const float Ground = static_cast<float>(
                Parapenting::Physics::TerrainModel::HeightM(
                    Position.x, Position.y));
            AStaticMeshActor* Marker = World->SpawnActor<AStaticMeshActor>(
                FVector(Position.x, Position.y, Ground + 0.18) * 100.0,
                FRotator::ZeroRotator);
            Marker->GetStaticMeshComponent()->SetStaticMesh(CylinderMesh);
            Marker->GetStaticMeshComponent()->SetWorldScale3D(
                FVector(3.8f, 3.8f, 0.025f));
            Marker->GetStaticMeshComponent()->SetCollisionEnabled(
                ECollisionEnabled::NoCollision);
            UMaterialInstanceDynamic* MarkerMaterial =
                UMaterialInstanceDynamic::Create(ShapeMaterial, Marker);
            MarkerMaterial->SetVectorParameterValue(
                TEXT("Color"),
                Route.advancedLanding
                    ? FLinearColor(0.95f, 0.55f, 0.05f)
                    : FLinearColor(0.92f, 0.92f, 0.88f));
            Marker->GetStaticMeshComponent()->SetMaterial(0, MarkerMaterial);

            // Every selectable landing gets a locally sampled surface-wind
            // cue. Offset it across the field so the pole does not cover the
            // route marker; this is presentation, not a surveyed sock position.
            const FVector2D SockPosition(
                Position.x, Position.y + 20.0);
            const float SockGround = static_cast<float>(
                Parapenting::Physics::TerrainModel::HeightM(
                    SockPosition.X, SockPosition.Y));
            AStaticMeshActor* Pole = World->SpawnActor<AStaticMeshActor>(
                FVector(SockPosition.X, SockPosition.Y, SockGround + 3.0)
                    * 100.0,
                FRotator::ZeroRotator);
            Pole->GetStaticMeshComponent()->SetStaticMesh(CylinderMesh);
            Pole->GetStaticMeshComponent()->SetWorldScale3D(
                FVector(0.06f, 0.06f, 6.0f));
            Pole->GetStaticMeshComponent()->SetCollisionEnabled(
                ECollisionEnabled::NoCollision);

            AStaticMeshActor* Windsock = World->SpawnActor<AStaticMeshActor>(
                FVector(SockPosition.X, SockPosition.Y, SockGround + 5.9)
                    * 100.0,
                FRotator(0.0, 0.0, 90.0));
            UStaticMeshComponent* SockComponent =
                Windsock->GetStaticMeshComponent();
            SockComponent->SetStaticMesh(ConeMesh);
            SockComponent->SetWorldScale3D(FVector(0.35f, 0.35f, 3.0f));
            SockComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            UMaterialInstanceDynamic* SockMaterial =
                UMaterialInstanceDynamic::Create(ShapeMaterial, Windsock);
            SockMaterial->SetVectorParameterValue(
                TEXT("Color"), FLinearColor(1.0f, 0.18f, 0.03f));
            SockComponent->SetMaterial(0, SockMaterial);
            WindsockComponents.Add(SockComponent);
            WindsockAnchorsCm.Add(Windsock->GetActorLocation());
            WindsockSamplePositionsM.Add(SockPosition);
        }
    }
}

void AParapentingGameMode::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    const AParagliderPawn* Glider =
        Cast<AParagliderPawn>(GetWorld() ? GetWorld()->GetFirstPlayerController()
            ? GetWorld()->GetFirstPlayerController()->GetPawn() : nullptr
            : nullptr);
    if (!Glider) return;
    const auto Cloud = Glider->GetCloudFieldState();
    if (CloudComponent)
    {
        CloudComponent->SetLayerBottomAltitude(
            static_cast<float>(Cloud.baseAltitudeM / 1000.0));
        CloudComponent->SetLayerHeight(
            static_cast<float>(Cloud.layerThicknessM / 1000.0));
    }
    if (CloudMaterialInstance)
    {
        CloudMaterialInstance->SetScalarParameterValue(
            TEXT("Coverage"), static_cast<float>(Cloud.coverage));
        CloudMaterialInstance->SetScalarParameterValue(
            TEXT("Density"), static_cast<float>(
                0.35 + 0.65 * Cloud.development));
        CloudMaterialInstance->SetVectorParameterValue(
            TEXT("WindOffset"),
            FLinearColor(
                static_cast<float>(Cloud.driftM.x / 10000.0),
                static_cast<float>(Cloud.driftM.y / 10000.0),
                0.0f, 0.0f));
    }
    if (SunComponent)
    {
        const auto Diurnal = Glider->GetDiurnalState();
        SunComponent->SetWorldRotation(FRotator(
            static_cast<float>(-Diurnal.sunElevationDegrees),
            static_cast<float>(Diurnal.sunAzimuthDegrees - 90.0),
            0.0f));
        SunComponent->SetIntensity(static_cast<float>(
            0.12 + 7.38 * Diurnal.ambientLight));
        const FLinearColor Neutral(1.0f, 0.98f, 0.94f);
        const FLinearColor Golden(1.0f, 0.72f, 0.48f);
        SunComponent->SetLightColor(FLinearColor::LerpUsingHSV(
            Neutral, Golden,
            static_cast<float>(Diurnal.warmLight * 0.38)));
        SunComponent->CloudShadowStrength =
            static_cast<float>(
                Cloud.shadowStrength * Diurnal.ambientLight);
        SunComponent->CloudShadowOnSurfaceStrength =
            static_cast<float>(
                Cloud.shadowStrength * Diurnal.ambientLight);
    }
    for (int32 Index = 0;
         Index < WindsockComponents.Num()
         && Index < WindsockAnchorsCm.Num()
         && Index < WindsockSamplePositionsM.Num();
         ++Index)
    {
        UStaticMeshComponent* WindsockComponent = WindsockComponents[Index];
        if (!WindsockComponent) continue;
        const FVector2D SamplePosition = WindsockSamplePositionsM[Index];
        const double GroundM =
            Parapenting::Physics::TerrainModel::HeightM(
                SamplePosition.X, SamplePosition.Y);
        const auto Air = Glider->SampleAtmosphereAt(
            {SamplePosition.X, SamplePosition.Y, GroundM + 6.0});
        const auto Sock = Parapenting::Physics::EvaluateWindsockPose(
            Air.windWorldMps, Air.gustEnergyMps,
            Glider->GetSimulationTimeSeconds());
        const FVector Direction(
            Sock.directionWorld.x,
            Sock.directionWorld.y,
            Sock.directionWorld.z);
        const float LengthCm =
            static_cast<float>(300.0 * Sock.lengthScale);
        WindsockComponent->SetWorldRotation(
            FQuat::FindBetweenNormals(FVector::UpVector, Direction));
        WindsockComponent->SetWorldScale3D(FVector(
            static_cast<float>(Sock.radiusScale),
            static_cast<float>(Sock.radiusScale),
            static_cast<float>(3.0 * Sock.lengthScale)));
        WindsockComponent->SetWorldLocation(
            WindsockAnchorsCm[Index] + Direction * (LengthCm * 0.5f));
    }
}
