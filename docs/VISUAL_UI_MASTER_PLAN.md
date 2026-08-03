# Visual and UI master plan

> **Revision 2 — checked against the build.** Revision 1 was written from the
> project description. This revision was written with the source open, and six
> findings exposed several statements about the current state as wrong or
> understated. Those are corrected below and the level order changed because of
> them. Grounded in Unreal 5.8 and Epic's current rendering, world-building,
> VFX and UI guidance.
>
> This is a summit ladder: every level ships a better-looking playable build;
> the upper levels are research programmes, and the final levels are
> deliberately unreasonable.

Last reviewed: 2026-08-03

## Purpose

Turn the current physics-first flight laboratory into a visually convincing,
legible and atmospheric alpine flying experience without weakening its
deterministic simulation or its first-class Apple Silicon support.

This plan covers:

- terrain geometry, materials, ground cover and distant vistas;
- vegetation, settlements, roads, water, launches and landing fields;
- canopy fabric, cells, seams, suspension lines, risers, harness and pilot;
- clouds, haze, light, wind, thermals, precipitation and particles;
- camera presentation, motion language and cinematic replay;
- in-flight HUD, front end, briefing, debrief, maps and accessibility;
- asset pipelines, scalability, profiling and visual regression.

It does **not** own flight dynamics, collision, airflow, scoring or the
authoritative canopy state. Presentation consumes those systems through narrow
contracts. A visual effect may reveal a physical state; it may never invent a
different physical state and feed it back into flight.

Audio is adjacent and not owned here, but it is not absent either:
`ParaglidingAudioComponent` and `Physics/AudioFeedback.h` exist, and Level 5's
exit gate depends on a visual/audio/motion bundle arriving together. Where a
level needs audio it says so and names the owner as the audio track.

## What the build actually does today

Read from the source on 2026-08-03. Numbers and file references, not
impressions.

**The strong raw material is real.** Two provenance-tracked swissALTI3D regions
shared by rendering, collision and airflow; deterministic diurnal, cloud-field,
wind, thermal, rotor, collapse, line-load, pilot-pose, launch, landing and
replay state; a deforming procedural canopy over a real suspension graph;
tiled procedural terrain, HISM vegetation, `SkyAtmosphere`, `VolumetricCloud`,
`ExponentialHeightFog`; four authored scalability tiers; compact, expanded and
minimal Canvas HUD modes.

**The visible implementation is thinner than revision 1 said.**

| what | where | actual state |
|---|---|---|
| Project content | `Content/` | **Zero `.uasset`, zero `.umap`.** Apart from metadata such as `.DS_Store`, the only files are two `.asc` heightfields, their provenance JSON and a terrain README. |
| Startup map | `Config/DefaultEngine.ini` | `GameDefaultMap=/Engine/Maps/Entry`. The entire world is spawned from C++ in `InitGame`/`BeginPlay`. |
| Terrain shading | `ParapentingTerrain.cpp:158` | Vertex colours, with a **key light baked in** whenever the lit material is missing — which is always, see below. |
| Lit material | `ParapentingMaterials.h:32` | `/Game/Materials/M_VertexLit` **does not exist**. `bVertexColourMaterialIsLit` is false in every run to date. |
| Canopy mesh | `ParagliderPawn.cpp:1397` | 21 spanwise × 9 chordwise stations, two surfaces. 189 vertices per surface. |
| Canopy colour | `ParagliderPawn.cpp:1445` | Two flat stripe colours, identical top and bottom, no shading term at all. |
| Suspension lines | `ParagliderPawn.cpp:836–879` | `DrawDebugLine` / `DrawDebugSphere`. Not geometry. |
| Pilot | `ParagliderPawn.cpp:55–106` | Eight `/Engine/BasicShapes/Cylinder` static meshes posed from `PilotPose`. |
| Trees, buildings, roads, water | `ParapentingGameMode.cpp:166–412` | `LoadObject` on `/Engine/BasicShapes/{Cone,Cylinder,Cube,Plane,Sphere}` into HISM components. |
| Expanded HUD | `ParapentingHUD.cpp:307` | A fixed 560×440 rect at (24,24), `GEngine->GetSmallFont()`, hard-coded pixel offsets, no DPI scaling. |
| UI framework | `Parapenting.Build.cs` | No `UMG`, no `Slate`, no `SlateCore`. No front end, no settings screen, no controller calibration screen. |
| VFX | `Parapenting.uproject` | No Niagara. The only enabled plugin is `ProceduralMeshComponent`. |
| Exposure and bloom | `Config/DefaultEngine.ini` | `r.DefaultFeature.AutoExposure=False`, `r.DefaultFeature.Bloom=False`, project-wide. |
| Mac RHI | `Config/DefaultEngine.ini` | `SF_METAL_SM6` only. |
| Cook list | `Config/DefaultGame.ini` | One entry: `/Engine/EngineSky/VolumetricClouds`. |

Everything visible in the game is either a procedural mesh with vertex colours,
an Engine primitive, a debug draw call or Canvas text. That gap is good news:
Levels 0–3 should have a very high visual return per hour. It is also the
reason the level order below is not revision 1's.

## Six source findings that reorder the plan

These were found by reading the build and each one moves work.

**1. There is no project map and no authored `/Game` asset tree.** Not "the
world has no coherent art direction" — there are no project assets to carry
one. Materials, fonts and UMG widgets can be authored without a custom map, but
Landscape, level PCG, HLOD, water bodies and authored set dressing cannot.
Creating the content folders, creating a checked-in map, and deciding what
lives in it versus what stays code-spawned are Level 0 work. The folder shell
blocks the first project material; the map blocks the editor-authored world
work in Levels 3, 4 and 9.

**2. Terrain and canopy are unlit, so the sun does not reach the ground.** The
project has a deterministic diurnal cycle, a `SkyAtmosphere`, a directional sun
and volumetric clouds, and none of them light the two objects that fill the
screen. `ParapentingMaterials.h` has the two-node material recipe written down
and has had it for some time; the asset was never authored because materials
cannot be compiled at runtime and nobody opened the editor. This is the single
highest-value hour in the whole plan.

Two traps come with it. First, `ParapentingTerrain.cpp:158` bakes a stand-in key
light into terrain vertex colours *conditionally on the material being unlit*,
so the terrain de-bakes itself correctly the moment the asset lands — but every
terrain colour on screen changes at once, and the before/after capture must be
taken across that flip, not after it. Second, the canopy bakes nothing
(`ParagliderPawn.cpp:1445` is flat stripe colour), so the wing gains real form
for free — and the material must be **two-sided**, because the canopy is a
single-thickness skin and the recipe already says so.

**3. In a Shipping build the wing probably has no lines.** Suspension visuals
are implemented exclusively with `DrawDebug*`; they are part of 29 debug-draw
call sites in `ParagliderPawn`, with the remainder serving airflow and geometry
inspection. `ENABLE_DRAW_DEBUG` is normally disabled for Shipping and Test
targets, so the suspension calls are expected to become no-ops. Promoting the
suspension graph to real geometry is therefore not Level 2 polish, it is a
shippability gate — and Level 0 should package a Shipping build and photograph
the result rather than treat the expectation as proof.

**4. A string-only runtime `LoadObject` path does not create an asset dependency
for cooking.** The trees,
buildings, roads and water in `ParapentingGameMode.cpp` are fetched by path
string at runtime. `/Engine/BasicShapes` is cooked as engine content, so this
happens to work; `/Game/Materials/M_VertexLit`, loaded exactly the same way,
has no discoverable project reference and should be expected to be omitted from
a cook. The one existing `DirectoriesToAlwaysCook` entry in `DefaultGame.ini`
is `/Engine/EngineSky/VolumetricClouds`, which is evidence that cloud cooking
has already needed explicit treatment. Every new `/Game` asset loaded only by
path needs an explicit cook path: preferably an asset/property reference or
Asset Manager rule, with a narrowly scoped `DirectoriesToAlwaysCook` entry as a
fallback. The packaged build is the only end-to-end proof.

**5. The renderer cannot see the geometry-driven physics, and will not soon.**
`docs/PHYSICS_TODO.md` item 7: `ParagliderPawn` flies `ParagliderDynamics`, a
six-degree-of-freedom body with a fitted polar. The coupled solver — cell
pressure, membrane fold, collapse state, per-line tension, section separation —
**is not run by the game at all**. Item 17 (removing the legacy path) is blocked
on item 11 (the pitch axis), which is open.

Revision 1's Level 2 asked for cell openings, fold state and line tension driven
"from geometry/state rather than painted fake folds". Today there is no such
state at runtime. Level 2 must therefore be written against what the legacy
telemetry actually exposes, with the richer inputs named as a later, explicitly
blocked, upgrade. Assuming otherwise would have produced a level that could not
close.

**6. Several contracts this plan proposes to build already exist.** Revision 1
described the presentation seam as future work. In `Source/Parapenting/Physics/`
there are already engine-independent, headless-testable:
`CameraFeedback` (position/rotation/FOV offsets from filtered acceleration,
already accessibility-scaled), `PilotPose`, `AccessibilityProfile`
(FullMotion / Comfort / MinimalMotion, with `inertialCameraScale`,
`rotorBuffetScale`, `hapticScale`), `GraphicsProfile` (Low/Medium/High/Epic with
`resolutionScale`), `WeatherSnapshot`, `AudioFeedback`, `HapticFeedback`,
`WindsockModel`, `LandingCircuitModel`, `PreflightBriefing`, `FlightDebrief`,
`FlightNavigation`, `CanopyGeometry`, `SuspensionGraph`, `CanopyLoadPose`,
`TerrainRenderLayout`. `Config/DefaultScalability.ini` already carries four
authored tiers for view distance, shadows, effects and foliage.

The visual track's job is to consume and extend these, not to invent parallel
ones. Where a level below says "define X", check this list first.

## Ambition and the summit ladder

| Camp | Levels | A valid stopping point gives us | Ambition |
|---|---:|---|---|
| **Base camp** | 0–2 | A measured baseline, a lit world and a glider worth looking at | Practical |
| **Camp I** | 3–5 | A recognisable, textured and atmospheric Bernese Oberland | Production engineering |
| **Camp II** | 6–8 | A coherent game UI and weather-responsive living world | High-end indie |
| **Camp III** | 9–11 | Hero-region fidelity, strong cinematography and scalable production | Small AAA team |
| **Death zone** | 12–15 | Microclimate optics, continental detail and a perceptual digital twin | Research to impossible |

Levels 12–15 are not promises. They make the long-term constraints visible so
the practical levels do not build dead ends. Reaching Level 8 with stable Mac
performance would be an excellent outcome. Reaching Level 11 would be
extraordinary for this project.

## Non-negotiable rules

1. **Every level is independently shippable.** No level may leave the main
   build visually broken while waiting for the next one.
2. **Physics owns truth.** Graphics interpolate simulation snapshots and may
   add cosmetic secondary motion only when it cannot alter flight, scoring,
   collision or replay determinism.
3. **One coordinate contract.** Terrain, landmarks, particles, audio and UI
   consume the surveyed route frame; no visual-only offsets become site data.
4. **One canopy contract.** The production canopy, lines and debug views derive
   from the authoritative geometry, attachment and deformation state rather
   than parallel hand-authored rigs.
5. **Art direction precedes asset volume.** A small coherent library beats a
   large mixture of incompatible assets.
6. **No detail without a distance.** Every asset and effect declares the camera
   range at which it matters, then gets an LOD, impostor, HLOD or cull policy.
7. **Scalability is authored, not hoped for.** Each feature declares Low,
   Medium, High and Epic behaviour before it lands, in the existing
   `DefaultScalability.ini` groups.
8. **Apple Silicon is measured continuously.** Windows may gain an optional
   ceiling, but Mac is never left with a non-functional fallback.
9. **Stable frame time outranks peak frame rate.** Traversal, route switches,
   weather transitions and UI openings are part of the benchmark.
10. **Readable beats spectacular.** Air hazards, horizon, terrain clearance,
    glider attitude, brake state and landing references must survive visual
    polish.
11. **Debug views remain available.** Beauty work must not remove airflow,
    geometry, collision, overdraw, LOD, streaming or scalability inspection.
12. **Provenance and licensing travel with assets.** Swiss source data,
    purchased packs, scans, fonts, logos and reference photography each have a
    recorded source and permitted use.
13. **Do not bind production identity to Engine sample assets.** Engine content
    is acceptable for fallback; shipped art direction lives under `/Game`.
    Every current art-asset dependency is an Engine primitive or fallback
    material, which is the measure of Levels 1–4 rather than a defect to fix
    separately.
14. **Transitions are features.** Time, weather, LOD, exposure and UI state
    change smoothly and replay consistently.
15. **Nothing visible may depend on `ENABLE_DRAW_DEBUG`.** Debug draw is a
    developer tool that vanishes in Shipping. If the player is supposed to see
    it, it is geometry.
16. **Every `/Game` asset loaded only by path needs an explicit cook path.**
    Prefer an asset/property reference or Asset Manager rule; use a narrow
    `DirectoriesToAlwaysCook` entry when those are impractical. Verify by
    packaging, not by reading.
17. **No presentation change may affect simulation inputs or stepping.**
    `ParagliderSolverClock` fixes the step and replays store one control input
    per physics step (`ParagliderPawn.h:437`). Rendering should therefore be
    irrelevant to the track. If a visual change alters the replayed state hash
    or trajectory, it has crossed the presentation boundary or exposed a clock
    defect and must not ship.

## Cross-agent boundary

The physics agent may extend engine-independent presentation snapshots. The
visual track may request fields but must not change coefficients or solver
behaviour to make an effect look better.

Recommended presentation seam:

```text
fixed simulation -> immutable presentation snapshot -> render interpolation
                 -> telemetry/event stream       -> UI and transient VFX
```

Much of this exists; see finding 6. What the snapshot still lacks, and what the
visual track should *request* rather than infer: canopy surface nodes with
normals and UVs at render resolution, coupled-solver cell pressure and fold
state, suspension nodes with cable-local tension and slack, a stable apparent-
wind sample for VFX, and typed incident/landing events. The legacy runtime does
already expose aggregate canopy pressure, collapse/cravat state, apparent wind
and four line-group tensions per side; those are enough for the first visual
tier but are not the geometry-driven state. Cosmetic systems keep their own
non-authoritative state. If presentation snapshots or events are later added to
the replay format, record those source values rather than individual particles;
today's replay files record setup plus control inputs only.

**The dependency that matters.** Everything in that "still lacks" list except
render-resolution canopy nodes lives in the coupled solver, which the game does
not run. The visual track's ceiling is set by `PHYSICS_TODO` items 7, 11 and 17,
in that order of consequence. Plan for two tiers of every canopy and line
effect: one that works against `ParagliderDynamics` telemetry today, one that
turns on when the geometry-driven stack flies. Do not schedule the second tier
against a date.

## Quality gates used at every level

Each level closes only when all applicable gates pass:

- **Before/after capture:** identical route, replay, time, weather, camera and
  graphics profile; representative launch, cruise, close canopy, cloud,
  landing and incident shots.
- **Golden replays:** Amisbühl–Lehn morning, thermal-day cruise, Grindelwald
  ridge, collapse recovery and landing flare.
- **Performance:** frame-time capture reports CPU game/render, GPU, memory,
  draw calls, primitives, Niagara and streaming; averages alone are rejected.
- **Image stability:** no material fallback, black mesh, terrain seam, LOD pop,
  shadow eruption, exposure pumping, translucent sorting failure or UI cutoff.
- **Gameplay legibility:** horizon, target field, canopy state and incident
  warnings remain readable at the supported FOVs and colour-vision profiles.
- **Packaging:** development **and Shipping** builds on Mac, opened and
  photographed, not just cooked; Windows checked at defined milestones. A
  Shipping build is the only thing that catches rules 15 and 16.
- **Determinism boundary:** existing headless physics and replay checks remain
  unchanged by presentation work. `Tools/check-build.sh` builds the module and
  runs every suite in about a minute; it is the gate, and there is no build
  quota (see `PHYSICS_LEARNINGS.md` §17).
- **Replay reproducibility:** a golden replay recorded before the change must
  produce the same state hash or trajectory within the existing deterministic
  tolerance. A presentation-only change should be incapable of changing it.

Target budgets must be established at Level 0 rather than invented here. As a
starting envelope, measure 30 fps Low on the oldest supported Mac, 60 fps
Medium on the reference Mac, and an optional higher-fidelity 60 fps profile on
the reference Windows GPU. Adjust only after recording hardware and resolution.

## Level 0 — Baseline, project shell and measurement spine

**Outcome:** the current build is reproducibly measured, it has somewhere to put
content, and every later visual change has a target, owner and comparison image.

### Bite-sized work

- [ ] Record supported hardware, resolutions, frame-rate targets and memory
  ceilings for Mac and Windows.
- [ ] **Create a checked-in level asset and authored `/Game` folder structure**,
  and decide
  per system what moves into it and what stays code-spawned. The world is
  currently built entirely in `InitGame`; the sun, sky, fog and clouds are
  reasonable candidates to author, the terrain tiles are not.
- [ ] Define asset naming, folders, material instances, texture channel packing,
  texel density and source/licence metadata — before the first asset, not after
  the fortieth.
- [ ] Create five deterministic golden replays and fixed camera bookmarks, and
  record what each one is for.
- [ ] Capture the baseline image set and Unreal Insights / GPU profiles,
  including the **procedural terrain rebuild spike**: a route change across
  regions rebuilds up to 64 `UProceduralMeshComponent`s synchronously
  (`ParapentingTerrain.cpp`). Measure it now so Level 3 has a number to beat.
- [ ] **Package and open a Shipping build on Mac.** Photograph the wing.
  Confirm or refute fact 3, and inventory anything else that vanishes.
- [ ] Inventory every visible runtime object, material, Engine dependency and
  procedural spawn path, and mark which are `/Engine` sample content (rule 13).
- [ ] Write a one-page art bible: naturalistic alpine realism, colour palette,
  atmospheric depth, contrast hierarchy, UI typography and prohibited looks.
- [ ] Extend, do not replace, the four `DefaultScalability.ini` tiers, and give
  `GraphicsProfile.h` ownership of the mapping.
- [ ] Add a visual QA checklist and an unattended screenshot comparison path.
  The project has no automated PIE harness today. A true headless/NullRHI run
  cannot validate rendered pixels, so choose a rendered packaged executable,
  offscreen-capable automation run or controlled editor session, and build the
  cheapest reliable option.

### Exit gate

The same five scenes can be replayed and captured on demand; performance and
memory baselines are stored including the terrain rebuild spike; a Shipping
package exists and its differences from Development are written down; the art
bible makes two artists likely to produce compatible work; no runtime
appearance has regressed.

## Level 1 — Light reaches the world

**Implementation status (2026-08-03): complete for the production path.** The
lit canopy and terrain materials, diagnostic material, calibrated swatches,
fixed-exposure/skylight policy, reproducible asset generation, terrain macro,
rock and snow response, and explicit Shipping cook path are implemented. The
historical unlit before-frame was not preserved, so the requested de-bake pair
cannot honestly be recreated after the fact; current fixed-time captures and
zero-warning Metal cooks are the continuing regression baseline. Further
surface detail belongs to the terrain and weather levels rather than keeping
the project indefinitely in Level 1.

**Outcome:** the sun, the sky and the diurnal cycle that already exist in the
simulation become visible on the two objects that fill the screen.

This level is first because it is the cheapest large change in the plan and
because every later judgement about colour, contrast and material is unreadable
until it lands.

### Bite-sized work

- [x] **Author `/Game/Materials/M_VertexLit` to the recipe in
  `ParapentingMaterials.h`**: Default Lit, two-sided, vertex colour to base
  colour, roughness 0.92, metallic 0. Give it an explicit cook path (rule 16) and
  confirm `bVertexColourMaterialIsLit` is true in a packaged build.
- [ ] Capture terrain **across** the de-bake flip. `ParapentingTerrain.cpp:158`
  drops its baked key light the moment the material is lit, so every ground
  colour changes at once; the before/after pair is the evidence that the new
  palette is better, not just different.
  A Shipping-compatible fixed-time capture harness now exists; the fallback
  baseline and reviewed comparison images remain to be produced.
- [x] Add a deliberate magenta error material, and make its appearance a test
  failure rather than a thing people learn to ignore. `M_VisualError` now
  exists, is explicitly cooked, and any missing/invalid production material
  fails the zero-warning cook gate rather than silently passing review.
- [ ] Build shared material functions for macro variation, triplanar rock,
  distance blend, detail normals, wetness, snow, wind and debug overrides.
  `M_SurfaceMaster` establishes the parameter contract and implements the first
  wetness/roughness path. `M_TerrainLit` now implements stable world-space macro
  variation without contaminating the moving canopy, plus a parameterized
  2.5–4.5 km distance fade to prevent shimmer. Steep rock now has a
  projection-free 3D breakup layer and distinct roughness, avoiding cliff UV
  stretch without texture memory. The existing altitude/aspect snow blend is
  now carried in terrain vertex alpha and drives a dedicated snow roughness,
  keeping the CPU palette and shader response on one mask. Detail normals,
  snow accumulation breakup, wind and debug overrides remain.
- [x] Establish a physically coherent exposure, white balance, sun/sky/fog and
  tone-mapping baseline for morning, midday and evening. Note that
  `r.DefaultFeature.AutoExposure` and `Bloom` are **off project-wide**; decide
  deliberately whether to turn exposure on with a bounded compensation range or
  keep it off and author fixed exposure per weather preset. Either is
  defensible; drifting between them is not. The fixed-exposure policy is now
  recorded in `docs/VISUAL_QA.md`; the first diurnal skylight-fill calibration
  is implemented, while reviewed morning/midday/evening captures remain open.
- [x] Create a small calibrated PBR swatch library: grass, soil, limestone,
  snow, water, ripstop nylon, webbing, metal and skin/clothing.
- [x] Add shader complexity and texture-density debug modes to the QA flow.
- [x] Eliminate startup shader/material warnings in a packaged build — the
  `M_VertexLit` fallback warning in `ParapentingMaterials.h` is the first one to
  go, and it should go by the asset existing.

### Exit gate

Terrain and canopy respond to sun and cloud shadow; the diurnal cycle is
visible on the ground; no visible object relies on the unlit debug material;
morning, noon and evening hold detail without clipped snow or crushed forests;
the packaged build loads the lit material; Low remains faster than the baseline
or the cost is explicitly accepted.

## Level 2 — Production glider, lines and pilot readability

**Implementation status (2026-08-03): complete for the procedural production
path.** The load-responsive suspension fan is opaque packaged geometry, not
`DrawDebugLine`; it includes risers, brake fan-out, maillons and harness
webbing. Canopy rendering independently samples 47 spanwise stations, has
stable manufactured UVs and an original panel colourway, and exposes a real
pressure-responsive intake gap with a dark interior per cell. A dedicated
two-sided transmitted-light fabric material replaces the generic lit surface.
The current articulated pilot remains a deliberately modest original fallback:
replacing it with a hero skeletal character and adding fibre-scale scans are
asset-production tasks, not blockers for this procedural Level 2 exit.

**Outcome:** the aircraft — the object closest to the camera — looks credible in
normal flight, survives a Shipping build, and visibly communicates control and
load within the limits of what the game's solver actually knows.

### Bite-sized work

- [x] **Promote the suspension graph from `DrawDebugLine` to real geometry.**
  Risers, mains, brakes and their terminations, with thickness, row
  colour/material, tension/slack response and distance LOD. This closes rule 15
  and is the level's blocking item.
- [x] **Decouple render resolution from solver resolution.** The canopy was 21 ×
  9 stations; the EPIC 2 ML has 45 cells and the EPSILON DLS 28 has 47
  (`Data/Wings/`).
  The real cell cadence, openings, ribs and seams cannot be represented
  geometrically by only 21 spanwise samples. The renderer now samples 47
  spanwise stations over `CanopyGeometry`, while the solver remains unchanged.
- [ ] Freeze a render-data adapter for canopy nodes, normals, UVs, attachment
  points and interpolation; retain the current mesh as fallback.
- [x] Give the canopy stable manufactured UVs and a generic, non-infringing
  colourway to replace the two flat stripe colours at
  `ParagliderPawn.cpp:1445`.
- [ ] Add fabric weave normal, transmission/subsurface response, roughness,
  panel variation and restrained edge wear at correct world scale. Transmission
  is the effect that reads first on a paraglider and it needs the lit material
  from Level 1.
- [ ] Render panel seams, rib shadows, cell openings, reinforcement bands and
  trailing-edge gathers from the authoritative geometry rather than painted
  fake folds. Pressure-responsive leading-edge gaps and dark cell interiors
  are implemented at the 47-cell cadence; raised seams, internal rib shadows
  and trailing-edge gather geometry remain a future close-camera asset pass.
- [x] Model risers, maillons, brake handles and brake fan-out; verify every line
  terminates at a real anchor through the whole brake range and in all poses.
- [ ] Replace the eight `/Engine/BasicShapes/Cylinder` limbs with a licensed or
  original skeletal presentation rig driven by `PilotPose`.
- [ ] Add canopy self-shadow, close-camera bias controls and a photo/replay mode
  that never changes simulation.
- [x] **Tier the state-driven effects.** Tier one drives fabric, brake fan-out
  and line load from `ParagliderDynamics` telemetry, which is what flies. Tier
  two — cell-local pressure, membrane fold geometry, cable-local tension and
  cravat contact — gets an engine-independent adapter contract and headless
  contract tests, then remains switched off until `PHYSICS_TODO` item 17 lands.
  Its rendered result cannot be accepted until the state exists in-game. The
  legacy telemetry tier is live; the coupled local tier remains intentionally
  disabled and does not block this level.
- [ ] Add collapse, pressure-loss, line-unload and ground-deflation visual tests
  at whichever tier is live.

### Exit gate

At chase distance the canopy reads as fabric cells rather than a coloured
surface; at close range lines connect correctly, are present in a Shipping
package, and do not shimmer excessively; brake travel, weight shift, unloading
and collapses are visually directional; the old glider can be selected as a
fallback while comparison captures prove the replacement is better; tier two is
green in headless tests without being enabled.

## Level 3 — Terrain surface vertical slice

**Implementation status (2026-08-03): in progress.** The measured architecture
decision retains the surveyed procedural tiles: they already match source
resolution, preserve physics coordinates, remain watertight and cull per tile.
`M_TerrainLit` now has explicit near, mid and far frequency bands; the new
contact band is snow-masked and fades from 350–800 m to prevent shimmer. Its
vegetation massing layer now keeps the summer-green read under blue aerial
perspective while retaining vertex-authored fields, rock and snow variation.

**Outcome:** one complete Amisbühl–Lehn corridor has believable ground from
altitude through flare, using the existing surveyed terrain as geometric truth.

### Bite-sized work

- [x] Decide by measured prototype whether the visual terrain migrates from
  procedural mesh tiles to Landscape, or retains tiles with an equivalent
  material/streaming path. Do not assume Nanite improves source resolution.
  Weigh the migration against what the tiles already give: watertight shared
  edges, per-tile bounds for frustum and occlusion culling, no collision
  cooking, and a topology contract enforced by headless tests
  (`TerrainRenderLayout.h`, `docs/TERRAIN_RENDERING.md`).
- [x] Fix or budget the synchronous rebuild measured at Level 0 — 64 procedural
  meshes on the game thread at a route change — before adding anything that
  makes it heavier. Builds now emit measured wall time, tile and vertex counts
  against an explicit 250 ms route-switch budget; same-region resets no-op.
- [ ] Import or derive land-cover masks for meadow, forest, settlement, rock,
  water, agriculture and snow with provenance. `TerrainColour` already
  classifies meadow, pasture, scree, exposed rock, snow retention and strata
  procedurally; the masks should replace or constrain that, not run beside it.
- [x] Build a slope/elevation/aspect/curvature-aware layered terrain material
  with near, mid and far frequency bands.
- [ ] Add mesh decals/overlays for exposed rock, erosion, paths, field edges and
  landing-field wear. The always-on landing debug sphere is now restricted to
  geometry-debug mode, so production captures must prove the field palette and
  approach references are sufficient; authored wear/edge overlays remain.
- [ ] Add contact-scale grass and stones only inside a camera-centred budget.
- [ ] Rebuild Lake Thun, Lake Brienz and the Aare with water materials, shoreline
  transition and altitude-correct placement. The Amisbühl slice now replaces
  Lake Thun's 3.1 × 1.2 km rectangular Engine plane with a tapered procedural
  shoreline polygon at the existing surveyed datum and dedicated Fresnel-lit
  `M_WaterSurface`; shoreline transition, Brienz and the final Aare surface
  remain.
- [ ] Preserve physics height queries and collision unchanged. Landing and
  ground clearance use `TerrainModel::HeightM` directly and the visual mesh has
  no cooked collision; document any render-only microdisplacement.
- [ ] Test terrain seams, grazing angles, snow clipping and landing visibility
  across all quality levels.

### Exit gate

The vertical slice holds up at 500 m AGL, tree-top height and landing height;
the landing field is readable without a giant primitive marker; terrain and
physics agree at every tested point; a route switch does not hitch beyond the
Level 0 budget.

## Level 4 — Alpine biome and human landscape

**Outcome:** Interlaken reads as a lived-in Swiss landscape, not coloured
terrain populated by the five Engine basic shapes.

### Bite-sized work

- [ ] Build a coherent conifer/deciduous/shrub/grass/rock asset kit with season,
  altitude, slope and aspect variants.
- [ ] Replace the hand-coded HISM distribution in `ParapentingGameMode.cpp` with
  an editor-baked PCG/biome pipeline or an equivalently inspectable
  deterministic tool. This needs the map from Level 0.
- [ ] Author forest edges, clearings, tree lines and density gradients; avoid
  uniform scatter.
- [ ] Create a modular low-cost Swiss building, roof and farm kit with distant
  HLOD/impostor treatment.
- [ ] Build roads, rail, rivers, fences, power lines, paths and field boundaries
  from sourced vectors where licensing permits.
- [ ] Replace landing/launch primitives and windsocks with production assets
  driven by `WindsockModel` and `LandingCircuitModel`.
- [ ] Author the minimum navigation landmark set visible from each route, chosen
  against `FlightNavigation`'s waypoints rather than by eye.
- [ ] Measure overdraw, shadow cost, WPO wind cost, instance count and streaming
  spikes before increasing density.

### Exit gate

A still image from the Amisbühl route is recognisably the Interlaken landscape
without HUD labels; forests have natural structure at three distance bands;
launch and landing approach are free of gameplay-obscuring clutter; traversal
meets the recorded frame-time and memory envelope.

## Level 5 — Air made visible

**Outcome:** atmosphere and particles make the same deterministic air model
perceptible without turning invisible airflow into misleading magic.

### Bite-sized work

- [ ] **Enable Niagara** and add the required plugin/module dependencies to the
  `.uproject` and `Parapenting.Build.cs`. Measure clean and incremental module
  build/link time before and after. `bUseUnity=false` is deliberate here (name
  collisions in `Physics/`), but do not assume the dependency cost before
  measuring it.
- [ ] Define an event/field adapter from wind, gust, thermal lifecycle, rotor,
  cloud, ground contact and canopy state into Niagara parameters, sourced from
  the presentation snapshot and not from solver internals.
- [ ] Add restrained near-camera dust, pollen, insects, leaf litter and snow
  motes whose motion samples apparent wind.
- [ ] Add launch dust/grass disturbance, footfall, line snap, fabric flutter and
  touchdown puffs with event budgets.
- [ ] Replace debug-only thermal/rotor presentation with optional educational
  layers visually distinct from normal scenic VFX. The two hard-coded debug
  bools in `ParagliderPawn` (airflow, geometry) are the current state of this,
  and `PHYSICS_TODO` item 16 owns generalising them.
- [ ] Give cloud coverage, base, thickness, drift and shadows smooth,
  replay-consistent transitions. Note the current `VolumetricCloud` uses
  `/Engine/EngineSky` material and is already cook-listed.
- [ ] Tune aerial perspective, valley haze and cloud contrast for terrain
  judgement rather than postcard saturation.
- [ ] Add Niagara significance, fixed bounds, pooling, culling and per-tier
  spawn budgets; prefer particles/flipbooks over fluid grids for gameplay.
- [ ] Coordinate the thermal-entry cue with the audio track's `AudioFeedback`
  and with `CameraFeedback`, and gate on the bundle rather than the particles.

### Exit gate

The player can infer wind direction near the surface and recognise entering a
thermal through a coherent bundle of subtle visual, audio and motion cues;
particles never imply a hazard absent from physics; disabling VFX changes no
flight result, leaves golden replays identical, and substantially reduces cost
on Low.

## Level 6 — Responsive UI, front end and visual language

**Outcome:** the flight lab becomes a coherent, resolution-independent game
without losing its engineering instrumentation — and gains the screens a game
needs and this one does not have.

Revision 1 scoped this level to the in-flight HUD, briefing and debrief. The
build has no main menu, no route selection screen, no graphics or accessibility
settings screen and no controller calibration screen, though
`InputBindingProfile`, `GraphicsProfile` and `AccessibilityProfile` all exist
behind keybinds. `MASTER_PLAN.md`'s backlog has wanted the calibration screen
since V0.

### Bite-sized work

- [ ] **Add `UMG`, `Slate` and `SlateCore` to `Parapenting.Build.cs`**, and
  decide on Common UI at the same time — it is a plugin, not a module, and its
  layered input routing is worth it for a front end and not for a vario.
- [ ] Acquire and check in a real font with the glyph coverage the target
  locales need. `GEngine->GetSmallFont()` is a stopgap and will not survive
  localisation.
- [ ] Define information hierarchy for scenic flight, training, incident,
  briefing, tuning, front end and debrief contexts, extending
  `docs/HUD_INFORMATION_MODEL.md` rather than restating it.
- [ ] Move compact and minimal presentation to UMG; preserve the Canvas
  expanded HUD as a developer fallback until parity is proven. The expanded HUD
  is a fixed 560×440 rect with hard-coded offsets
  (`ParapentingHUD.cpp:307`) and is the reason "resolution-independent" is a
  requirement rather than a nicety.
- [ ] Build shared typography, colour, spacing, icon and motion tokens with safe
  zones and DPI scaling.
- [ ] Implement instrument widgets for vario, air/ground speed, altitude/AGL,
  brakes, waypoint/reachability and winds; avoid dashboard duplication.
- [ ] Create persistent incident priority rules so collapse, stall, overload
  and ground warnings pre-empt lower-priority coaching. This is already the
  documented behaviour; make it structural rather than draw-order.
- [ ] **Build the front end**: title, route selection, weather preset, wing and
  pilot setup, graphics profile, accessibility profile, input binding and
  controller calibration. Each one binds to an existing engine-independent
  model; none of them should introduce new state.
- [ ] Add controller/keyboard glyph switching, focus states and full gamepad
  navigation.
- [ ] Rebuild briefing and debrief as layered screens with comparison, map and
  concise coaching views, over `PreflightBriefing` and `FlightDebrief`.
- [ ] Add localization-safe layout, font fallback, subtitle/caption hooks,
  colour-vision checks and scalable text.
- [ ] Surface `AccessibilityProfile` properly: camera motion, rotor buffet and
  haptic scaling are already implemented and currently invisible to the player.
- [ ] Test 16:9, 16:10, ultrawide and reference Retina resolutions.

### Exit gate

Every non-debug task can be completed with keyboard/mouse or controller,
including starting a flight from a cold boot without touching a keybind; no
critical value clips or overlaps at supported resolutions; minimal HUD is
genuinely scenic; expanded diagnostics retain all current information; UI
open/close does not create a perceptible frame spike.

## Level 7 — Weather, surface response and seasonal coherence

**Outcome:** weather presets change the whole scene coherently rather than only
wind numbers and a cloud layer.

### Bite-sized work

- [ ] Define visual weather snapshots derived from the existing deterministic
  atmosphere and diurnal state, extending `WeatherSnapshot` rather than
  duplicating it.
- [ ] Add humidity-aware haze, cloud type/shape variation and orographic cloud
  placement where the simulation supports it.
- [ ] Drive wetness, puddles, rock darkening, leaf response, snow retention and
  surface sparkle through bounded material parameter collections.
- [ ] Add scalable rain, snow and virga presentation with camera and ground
  response; label any scenic-only precipitation state as scenic-only in the
  code, not just in a comment.
- [ ] Couple tree, grass, windsock, line and loose-particle motion to the same
  local wind samples with deliberately different response bands.
- [ ] Author morning, valley-breeze, thermal-day, foehn and evening visual
  identities without LUT gimmicks that destroy measurement readability.
- [ ] Stress-test transitions over accelerated local time and replay seek.

### Exit gate

Preset identity is recognisable with the HUD hidden; wind-responsive objects
agree on direction and plausible inertia; wet/snow states do not appear where
their declared environmental conditions are false; transitions are smooth and
bounded on all profiles.

## Level 8 — Camera, replay and final-pixel polish

**Outcome:** ordinary flying feels authored and recordings look intentional,
while camera feedback remains subordinate to spatial judgement.

### Bite-sized work

- [ ] Tune chase, pilot, wing and scenic camera rigs against the deterministic
  `CameraFeedback` contract, keeping its accessibility scaling intact.
- [ ] Add collision-aware framing, horizon protection and low-speed/landing
  composition; never hide a collapse or flare cue.
- [ ] Establish motion blur, depth of field, lens flare, bloom, exposure and
  sharpening policy per camera and quality tier, consistent with the Level 1
  exposure decision.
- [ ] Add replay timeline, camera cuts, orbit/free camera and deterministic
  shot bookmarks over the existing control-input replay
  (`ParagliderPawn.h:437`). Note it replays inputs, not state, so scrubbing
  backwards means re-simulating from a keyframe — design for that rather than
  discovering it.
- [ ] Add photo mode with quality overrides isolated from live gameplay.
- [ ] Implement tasteful event emphasis for launch commitment, thermal entry,
  collapse and touchdown using coordinated camera, VFX, UI and audio cues.
- [ ] Perform a complete aliasing, ghosting, translucency, shadow and LOD-pop
  polish pass on golden replays.

### Exit gate

Five golden replays can produce trailer-usable shots without external tools;
gameplay cameras preserve horizon and landing judgement; visual emphasis never
changes simulation timing; the same replay file still produces the same track;
temporal artifacts are catalogued and below agreed thresholds.

## Level 9 — Grindelwald hero region and production world pipeline

**Outcome:** a second visually distinct, high-fidelity region proves the world
pipeline scales beyond one hand-polished corridor.

### Bite-sized work

- [ ] Apply Levels 3–8 to Grindelwald using reusable tools before adding
  one-off fixes. The region already exists as surveyed ground with its own
  `TerrainRenderLayout` and both routes on it.
- [ ] Build region parameter sets for geology, vegetation, settlement,
  agriculture, snow and atmospheric depth.
- [ ] Author First, Eiger/Mönch/Jungfrau silhouettes and route-critical
  landmarks from licensed sources at appropriate fidelity.
- [ ] Add validation overlays comparing surveyed anchors, source imagery,
  rendered placement and physics terrain. `PHYSICS_TODO` item 9 has a live
  example: Grindelwald First's published anchor sits 50 m above its surveyed
  ground, and the overlay should make that visible rather than hide it.
- [ ] Automate height/land-cover/vector import, PCG bake, HLOD build, asset audit
  and packaged route smoke test.
- [ ] Document how a third region is added without editing core flight code.

### Exit gate

Both regions meet the same capture and performance gates; a region switch is
safe and bounded; at least 80% of the Grindelwald result comes through reusable
pipeline/assets; the third-region procedure is demonstrably repeatable.

## Level 10 — Production canopy and character ecosystem

**Outcome:** equipment presentation supports variety, close inspection and
future partnerships without cloning the render rig or shader stack.

### Bite-sized work

- [ ] Define data-driven canopy colourways, fabric families, reinforcement,
  riser and line-material packages independent of handling data. The wing data
  packages in `Data/Wings/` are the model for how this should look.
- [ ] Support multiple authoritative canopy topologies through one renderer —
  the two research wings already differ in cell count, span and aspect ratio.
- [ ] Add pilot body/clothing/harness variants with fit and clipping tests.
- [ ] Add facial/head-look and hand-contact polish only after flight poses are
  correct.
- [ ] Implement damage/dirt/wetness as cosmetic history with reset and replay
  rules.
- [ ] Create automated close-up render tests for each equipment combination.
- [ ] Establish manufacturer asset/legal review gates before any branded model.

### Exit gate

Every supported wing/harness combination renders through shared systems,
anchors correctly, survives collapse/launch/landing poses and respects licence
status; adding a colourway requires data/assets rather than C++ changes.

## Level 11 — Hero realism at flight scale

**Outcome:** selected routes approach contemporary high-end outdoor-game
fidelity from ground contact to long alpine vistas.

### Bite-sized work

- [ ] Acquire higher-resolution licensed terrain/imagery for limited hero
  corridors and preserve the surveyed physics surface separately.
- [ ] Add close-range rock/soil/snow geometry with scan-derived materials and
  render-only displacement constrained around collision truth.
- [ ] Build photogrammetry-quality landmarks and vegetation assemblies where
  they dominate perception.
- [ ] Add robust HLOD, virtual-texture/streaming and shader precompile strategy.
- [ ] Introduce optional Nanite/Lumen/virtual-shadow paths only where profiling
  proves platform value; maintain Mac-compatible alternatives. The project
  already targets `SF_METAL_SM6` only, so the shader model is not the
  obstacle — measured cost on the reference Mac is.
- [ ] Run external art-direction, pilot-recognition and accessibility reviews,
  using `docs/PILOT_REVIEW_PROTOCOL.md` where it applies.

### Exit gate

Hero routes withstand ground, chase and telephoto cameras; optional high-end
features improve measured image quality rather than merely raising settings;
reference Mac performance remains within the agreed envelope; external viewers
consistently recognise location and flight state.

## Level 12 — Four-dimensional atmosphere

**Outcome:** cloud and optical phenomena become a spatial, evolving volume
coherent with terrain, sun and the modeled air instead of layered effects.

### Bite-sized work

- [ ] Define a sparse, streamable 4D atmospheric presentation field derived
  from—not substituted for—the physics atmosphere.
- [ ] Generate source-local cumulus with growth, decay, shadow and advection
  tied to deterministic thermal lifecycles.
- [ ] Model orographic caps, cloud streets, rotor clouds and valley haze where
  source state can justify them.
- [ ] Add multi-scattering approximations, silver lining, cloud self-shadow and
  terrain-scale light transport with temporal stability.
- [ ] Build GPU budgets, async update, clipmaps and lower-tier impostors.
- [ ] Validate cloud cues with meteorologists and pilots as presentation, not
  operational forecasting.

### Exit gate

Cloud development is spatially traceable to the atmosphere state, stable in
replay and readable over tens of kilometres; the reference platform holds the
budget during the worst authored weather case. This is a research-grade goal.

## Level 13 — Living Switzerland at continental scale

**Outcome:** multiple complete regions join into a continuous, streamed world
with believable local identity at paraglider speed and altitude.

### Bite-sized work

- [ ] Build licensed national terrain, imagery, land-cover, building, transport,
  water, vegetation and obstacle ingestion with update provenance.
- [ ] Move to world partitioning/streaming cells with region-local origin and
  precision policy proven against flight coordinates. The current
  one-region-at-a-time rebuild is the deliberate intermediate step and its
  limits are documented in `docs/TERRAIN_RENDERING.md`.
- [ ] Generate settlements and biomes with authored exceptions for every
  visually dominant corridor.
- [ ] Add traffic, cable cars, trains, boats, wildlife and other ambient life as
  aggressively budgeted presentation systems.
- [ ] Build distributed asset processing, HLOD, cook and visual regression.
- [ ] Staff ongoing art, geodata, licensing and QA operations.

### Exit gate

A player can fly between major regions without a loading break or coordinate
failure; streamed detail is coherent from altitude; national builds remain
reproducible and provenance-complete. This is a studio-scale programme, not a
normal project milestone.

## Level 14 — Perceptual alpine digital twin

**Outcome:** selected sites match reality across season, time, vegetation,
surface condition, atmosphere and human landscape closely enough for blinded
image/location studies.

### Bite-sized work

- [ ] Commission repeat seasonal aerial and ground capture with calibrated
  cameras, LiDAR and material references.
- [ ] Reconstruct vegetation species/age, buildings, infrastructure and terrain
  change with explicit capture dates.
- [ ] Solve inverse lighting/material calibration against reference panoramas.
- [ ] Assimilate archived cloud and aerosol observations into deterministic
  visual scenarios.
- [ ] Quantify perceptual error with blinded local-expert studies rather than
  screenshot preference alone.
- [ ] Maintain legal, privacy, storage and update pipelines for living data.

### Exit gate

Local experts reliably identify site, season and broad time/weather class from
unlabelled renders while being unable to point to systematic geometric or
material errors in hero corridors. Achieving and maintaining this is likely
more expensive than the rest of the game.

## Level 15 — The impossible summit: indistinguishable embodied flight

**Outcome:** in motion, at human visual acuity and latency, the simulation is
perceptually indistinguishable from looking through a camera—or a future
headset—during real alpine flight.

### Bite-sized research fronts

- [ ] Real-time, many-kilometre, temporally stable global illumination through
  terrain, vegetation, canopy fabric, aerosols and evolving clouds.
- [ ] Fibre-, seam- and wrinkle-scale canopy appearance driven by the same
  two-way structural state that governs flight.
- [ ] Resolved optical turbulence, humidity, ice, droplets, dust, insects and
  wake interaction from ground scale to cloud scale.
- [ ] Foveated, gaze-aware rendering at retina-class angular resolution with
  end-to-end motion-to-photon latency below perceptual thresholds.
- [ ] Neural reconstruction that is deterministic, controllable, licensed,
  artifact-free and physically constrained across unseen viewpoints.
- [ ] Global seasonal world updates with no manual cleanup and verified factual
  provenance.
- [ ] Blinded expert validation across displays/headsets, sites and conditions.

### Summit condition and honest assessment

Qualified pilots cannot distinguish recorded real flight from simulated flight
above chance across a preregistered, adversarial test covering close canopy,
terrain, vegetation, settlements, clouds, launch, active air and landing—while
the system still runs interactively on consumer hardware.

This combines unsolved real-time rendering, world reconstruction, atmosphere,
deformable appearance, content rights, display and validation problems. It is
not a credible production commitment. It is useful because it defines the
direction of travel: coherent truth across scales matters more than adding one
more isolated effect.

## Recommended implementation order

Do not start several levels at once. Within a level, keep changes vertically
sliceable and reversible:

1. Capture a golden replay and identify one visible deficiency.
2. Add the narrow data/asset contract needed for that deficiency.
3. Ship the smallest improved fallback-safe version.
4. Profile Mac and Windows, then author scalability.
5. Capture before/after evidence and close the checklist item.
6. Remove the old path only after parity and rollback have been demonstrated.

The recommended first production sequence is:

1. **Level 0's project shell first**: an authored `/Game` folder structure,
   naming rules, a minimal checked-in map, and a packaged Shipping build to see
   what is actually missing. The map enables later world authoring; it is not a
   prerequisite for standalone materials or widgets.
2. **`M_VertexLit`.** One small material graph, and the sun starts reaching the
   ground and the wing. Capture across the terrain de-bake flip.
3. Level 1's exposure decision and PBR swatches.
4. Level 2's suspension geometry — it is the rule 15 fix, not polish — then
   canopy UVs and fabric, then the pilot rig.
5. Level 3, one terrain vertical slice.
6. Reassess visual return, performance and asset budget before Level 4.

This order puts the free lighting win first, then the closest and most
frequently viewed object, then the surface beneath it, and postpones mass
content generation until the material, distance and scalability rules are
proven.

## Rough effort shape

These are comparative planning bands, not estimates or commitments. One unit
means one focused, reviewable chunk such as a material function, a canopy UV
pass, one instrument widget or one PCG biome rule—not an entire level.

| Level | Work units | Main constraint |
|---|---:|---|
| 0 | 10–16 | Discipline, reproducible capture, and creating a project shell that does not exist yet |
| 1 | 10–16 | Technical art and cross-platform shaders. The first unit is one asset and worth more than the other fifteen. |
| 2 | 20–35 | Canopy/pilot assets, line geometry and deformation integration |
| 3 | 18–30 | Geodata, terrain shading and near-ground quality |
| 4 | 25–50 | Asset library, licensing and world composition |
| 5 | 15–25 | VFX restraint and field integration |
| 6 | 30–50 | UX design, widget migration, a front end that does not exist, and accessibility |
| 7 | 20–35 | Coherent transitions and surface response |
| 8 | 15–25 | Cinematography and temporal artifact cleanup |
| 9 | 35–60 | Proving the regional pipeline |
| 10 | 30–60 | Equipment content and character rigging |
| 11 | 60–120 | Hero assets, scans and high-end rendering |
| 12 | 100–250 | Atmospheric rendering research |
| 13 | 300–1000+ | National data/content operations |
| 14 | 1000+ | Repeated real-world capture and calibration |
| 15 | Unbounded | Multiple unsolved research problems |

## Decisions deliberately deferred

- **Authored map versus code-spawned world.** The world is entirely spawned in
  `InitGame` today, which has kept it testable and diff-able and has cost it
  every editor authoring tool. The likely answer is a hybrid — authored
  lighting, atmosphere and hero set dressing; code-spawned terrain tiles and
  procedural distribution — but it should be decided at Level 0 with the
  reasons written down, because Levels 3, 4, 6 and 9 all depend on it.
- Landscape versus retained procedural terrain tiles: prototype and profile at
  Level 3. Landscape gives mature authoring/PCG integration; migration also has
  coordinate, cook and memory costs, and the tiles already give watertight
  edges, per-tile culling bounds and a test-enforced topology contract. Nanite
  Landscape streams both Nanite and non-Nanite data and does not add source
  detail by itself.
- Lumen and Nanite as defaults: decide per platform from captures, not prestige.
- Auto-exposure on or off: currently off project-wide. Level 1 decides, and the
  decision constrains Level 7's weather identities and Level 8's camera policy.
- Runtime PCG: prefer editor-baked deterministic results until a real runtime
  need justifies its complexity. GPU PCG is currently documented as Beta.
- Niagara Fluids: reserve for exceptional high-end/cinematic cases. Epic warns
  that fluid simulations are heavy; ordinary wind cues should use cheaper
  particles, materials and flipbooks.
- Common UI everywhere: use it for the front end, layered menus, controller
  navigation and glyph routing; simple in-flight instruments may remain
  ordinary UMG widgets.
- Branded real equipment: no logo, exact livery or claimed likeness without
  permission.

## Research basis

Primary Unreal references reviewed for this revision:

- [Using Nanite with Landscapes](https://dev.epicgames.com/documentation/en-us/unreal-engine/using-nanite-with-landscapes-in-unreal-engine) — source geometry is
  unchanged, Landscape keeps non-Nanite data for systems such as runtime
  virtual textures and water, and both representations consume memory.
- [Creating Landscapes](https://dev.epicgames.com/documentation/en-us/unreal-engine/creating-landscapes-in-unreal-engine) — Landscape supports imported
  heightmaps and is Unreal's mature large-terrain authoring path.
- [Procedural Content Generation Framework](https://dev.epicgames.com/documentation/en-us/unreal-engine/procedural-content-generation-framework-in-unreal-engine) — PCG is designed for iterative tools ranging from asset utilities to
  biome/world generation.
- [Using PCG with GPU Processing](https://dev.epicgames.com/documentation/en-us/unreal-engine/using-pcg-with-gpu-processing-in-unreal-engine) — GPU execution can
  accelerate supported point/spawn work but is documented as Beta.
- [Niagara Overview](https://dev.epicgames.com/documentation/en-us/unreal-engine/overview-of-niagara-effects-for-unreal-engine) — reusable systems,
  emitters, modules and parameters are the intended VFX architecture, and each
  layer has a resource cost.
- [Niagara Fluids](https://dev.epicgames.com/documentation/en-us/unreal-engine/niagara-fluids-in-unreal-engine) — fluids are graphically intensive;
  Epic recommends considering baked flipbooks for cheaper use.
- [Common UI overview](https://dev.epicgames.com/documentation/en-us/unreal-engine/overview-of-advanced-multiplatform-user-interfaces-with-common-ui-for-unreal-engine) and [quickstart](https://dev.epicgames.com/documentation/en-us/unreal-engine/common-ui-quickstart-guide-for-unreal-engine) — Common UI provides
  layered input routing, shared style assets, gamepad navigation and
  platform-specific input glyphs.

Project sources read for revision 2: `Source/Parapenting/*.{h,cpp}`,
`Source/Parapenting/Physics/` headers, `Config/Default{Engine,Game,Scalability}.ini`,
`Parapenting.uproject`, `Parapenting.Build.cs`, `Content/`,
`docs/TERRAIN_RENDERING.md`, `docs/HUD_INFORMATION_MODEL.md`,
`docs/PHYSICS_TODO.md`.

## First restart point

When implementation begins, start only Level 0, and inside Level 0 start with
the project shell: an authored `/Game` folder structure, a minimal checked-in
map, the naming and provenance rules, and a packaged Shipping build on Mac that
somebody actually opens and photographs. The map is the anchor for later
Landscape/PCG/HLOD work; creating it does not require moving the working
code-spawned world yet. The Shipping capture is the cheapest way to find out how
much of the current game is developer-only rendering.

The second concrete artifact is the golden-capture matrix: named replay, route,
time, weather, camera, graphics profile, resolution, expected focal subject and
performance counters.

Then author `M_VertexLit`. It is a tiny material graph, the recipe has been
sitting in `ParapentingMaterials.h` waiting for someone to open the editor, and
it is the largest visual change per unit of work available anywhere in this
document.

Do not buy assets or migrate terrain before those three exist.
