# Visual QA baseline

## Level 1 lighting decision

Parapenting uses **fixed exposure**. Project-wide auto exposure and bloom stay
disabled in `DefaultEngine.ini` for Level 1. Paragliding depends on stable
terrain and horizon contrast; adapting exposure independently of the
deterministic time/weather state would make identical replay frames render
differently after camera cuts. If exposure becomes state-driven later, the
values belong in the replay-safe visual weather snapshot.

The lit-material transition deliberately removed the old terrain vertex-colour
key light. Shadow readability now comes from the real-time skylight: intensity
tracks the deterministic diurnal ambient term from 0.42 to 1.35, with a dark
blue-grey lower-hemisphere fill for steep valley walls and canopy undersides.
This is diffuse sky/ground bounce, not a second directional light. Foreground
forest and rock should remain visibly coloured in shade; near-black terrain is
a failed lighting calibration.

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

The Shipping build has a deterministic one-shot capture mode. For an individual
frame, launch it with:

```text
-windowed -ResX=1600 -ResY=900 -VisualQACapture=midday \
    -VisualQAHour=13 -VisualQAWarmup=6
```

It fixes the simulation's local hour, hides the HUD, waits for terrain and
render resources, writes `Saved/VisualQA/midday_13.00h.png` using the captured
pixel buffer (rather than relying on a development-only console command), then
exits. Use `Tools/Visual/capture_level1.sh` to collect morning (08:00), midday
(13:00) and evening (19:00) from the same packaged executable. The mechanism
deliberately uses the game viewport rather than desktop automation. A
missing/empty PNG makes the one-shot process exit with status 2, so CI cannot
silently accept a processed-but-unsaved screenshot.

On macOS, use the app under `Saved/StagedBuilds/Mac`. Unreal 5.8's archive step
in this environment copies the thin app from `Binaries/Mac` and omits staged
runtime libraries such as `libtbb.12.dylib`; that archived copy cannot be used
as evidence until the engine packaging issue is resolved.

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
- The three one-shot capture jobs exit successfully and produce 1600x900 PNGs.
