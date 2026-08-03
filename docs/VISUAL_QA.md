# Visual QA baseline

## Level 1 lighting decision

Parapenting uses **fixed exposure**. Project-wide auto exposure and bloom stay
disabled in `DefaultEngine.ini` for Level 1. Paragliding depends on stable
terrain and horizon contrast; adapting exposure independently of the
deterministic time/weather state would make identical replay frames render
differently after camera cuts. If exposure becomes state-driven later, the
values belong in the replay-safe visual weather snapshot.

The first production material library lives in `/Game/Materials`:

- `M_VertexLit`: two-sided Default Lit vertex-colour material used by the
  procedural terrain and canopy;
- `M_VisualError`: unmistakable magenta diagnostic material;
- `M_SurfaceMaster`: parameterized PBR base for the first calibrated swatches;
- `MI_*`: grass, soil, limestone, snow, water, ripstop nylon, webbing, metal
  and clothing reference swatches.

`Tools/Visual/create_level1_materials.py` is the reproducible source for these
binary assets. It creates missing assets and safely re-applies swatch instance
parameters. Unreal 5.8 commandlet mode cannot reliably rewrite a loaded
material graph, so run `reset_level1_surface_assets.py` in a separate Editor
commandlet process before regenerating a deliberately changed surface master.
`M_VertexLit` is loaded at startup and must be changed in the Editor or deleted
before launch. Commit the resulting `.uasset` files with the scripts.

## Required Level 1 captures

Capture the same route, replay and camera before and after enabling
`M_VertexLit`:

1. Amisbühl launch, morning, canopy filling the upper third.
2. Amisbühl–Lehn cruise, midday, terrain at grazing angle.
3. Lehn final, evening, landing field and canopy both visible.
4. Cloud-shadow pass over an exposed slope.
5. Canopy top and underside close-ups.

The first comparison intentionally straddles the terrain de-bake flip in
`ParapentingTerrain.cpp`: the fallback vertex colours contain a stand-in key
light and the lit path does not. Compare form, shadow direction, snow clipping,
forest crushing and landing-field readability—not average brightness.

## Pass/fail checks

- No `M_VertexLit not found` warning in Development or Shipping logs.
- Terrain and canopy receive the directional sun and cloud shadows.
- Canopy remains visible from below; `M_VertexLit` is two-sided.
- Fixed morning, midday and evening frames do not clip snow or crush forest
  detail.
- Shader complexity and texture-density editor views contain no unexpected
  fallback/error material.
- `/Game/Materials` exists in a packaged build.
- Headless physics and replay trajectories are unchanged.
