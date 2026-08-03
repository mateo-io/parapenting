#pragma once

#include "CoreMinimal.h"
#include "Materials/MaterialInterface.h"

namespace Parapenting
{
// Terrain and canopy are procedural meshes whose colour lives entirely in
// vertex colours. Unreal ships no *lit* stock material that reads vertex
// colour, so both meshes historically used
// /Engine/EngineDebugMaterials/VertexColorMaterial. That material is unlit:
// it routes vertex colour straight to emissive, which means the ground and
// the wing ignore the sun completely. Shading has to be faked in the vertex
// colours instead (see TerrainColour), the diurnal cycle cannot reach the
// terrain at all, and nothing casts or receives light.
//
// The fix is a two-node project material, which has to be authored once in
// the editor because materials cannot be compiled at runtime:
//
//   Content/Materials/M_VertexLit
//     Shading Model : Default Lit
//     Two Sided     : true        (canopy is a single-thickness skin)
//     VertexColor.RGB -> Base Color
//     Constant 0.92   -> Roughness
//     Constant 0.0    -> Metallic
//
// Once that asset exists this helper picks it up automatically and the
// fallback stops being used. Until then the scene still renders, just unlit.
// Set to false when the lit project material is missing and the unlit
// fallback is in use, so callers can bake a stand-in key light into their
// vertex colours instead. Valid after the first material load.
inline bool bVertexColourMaterialIsLit = false;

inline UMaterialInterface* LoadCanopyMaterial()
{
    if (UMaterialInterface* Fabric = LoadObject<UMaterialInterface>(
        nullptr, TEXT("/Game/Materials/M_CanopyFabric.M_CanopyFabric")))
    {
        bVertexColourMaterialIsLit = true;
        return Fabric;
    }
    if (UMaterialInterface* Lit = LoadObject<UMaterialInterface>(
        nullptr, TEXT("/Game/Materials/M_VertexLit.M_VertexLit")))
    {
        bVertexColourMaterialIsLit = true;
        return Lit;
    }
    bVertexColourMaterialIsLit = false;

    static bool bWarned = false;
    if (!bWarned)
    {
        bWarned = true;
        UE_LOG(LogTemp, Warning,
            TEXT("Parapenting: /Game/Materials/M_VertexLit not found; "
                 "falling back to the unlit engine vertex-colour material. "
                 "Terrain and canopy will not respond to the sun. "
                 "See ParapentingMaterials.h for the asset recipe."));
    }
    return LoadObject<UMaterialInterface>(nullptr,
        TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));
}

inline UMaterialInterface* LoadLineMaterial()
{
    return LoadObject<UMaterialInterface>(
        nullptr, TEXT("/Game/Materials/M_VertexLit.M_VertexLit"));
}

inline UMaterialInterface* LoadTerrainMaterial()
{
    if (UMaterialInterface* Terrain = LoadObject<UMaterialInterface>(
        nullptr, TEXT("/Game/Materials/M_TerrainLit.M_TerrainLit")))
    {
        bVertexColourMaterialIsLit = true;
        return Terrain;
    }

    // M_TerrainLit is an optional specialization. Falling back to the lit
    // canopy material preserves correct sun/shadow response and must not
    // reactivate the legacy baked key light.
    return LoadCanopyMaterial();
}
}
