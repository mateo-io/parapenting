# Regional terrain rendering

> **`docs/TERRAIN.md` is the authoritative description of the frame, the
> surveyed regions and the traps.** This file covers rendering topology and
> surface shading only.

Two surveyed swissALTI3D regions — Interlaken and Grindelwald — share one
physics coordinate frame and feed atmosphere, landing logic and visible terrain
alike. The renderer draws **one region at a time**, rebuilding when a route
change crosses between them, and divides it into independently cullable tiles
without changing physics coordinates.

## Layout

Per region, from `LayoutFor(x, y)`, with tile counts chosen to keep vertex
spacing near the 20 m source cells:

| region | rendered extent | tiles | spacing |
|---|---|---|---|
| `interlaken` | x [-1800, 6100], y [-3500, 4500] | 8 × 8 | 19.8 × 20.0 m |
| `grindelwald` | x [4500, 11500], y [-18500, -14000] | 7 × 5 | 20.0 × 18.0 m |

- 50 × 50 quads per tile, 51 × 51 vertices;
- at most 166,464 submitted vertices and 320,000 triangles for a region;
- no procedural collision cooking.

Shared tile-edge positions are evaluated from identical world coordinates, so
adjacent meshes remain watertight. Landing and ground clearance use
`TerrainModel::HeightM` directly; disabling visual-mesh collision therefore
removes redundant work without changing flight behavior.

Terrain triangles are submitted in Unreal's clockwise front-face order. This
is part of the render contract: an earlier reverse order let a one-sided
material cull entire distant slopes, which read as transparent blue mountains
against the sky. The terrain material remains one-sided; correct winding keeps
the normal and lighting path honest without doubling raster work.

The game mode loads the heightfield during `InitGame`, before Unreal spawns the
default pawn. The pawn's first route reset therefore uses the surveyed launch
elevation; it cannot be embedded later by replacing an analytic fallback
surface during `BeginPlay`.

Each surveyed heightfield is authoritative only inside its own bounds. Outside
every region the analytic Interlaken proxy takes over; it is a shape, not a
place, and nothing terrain-dependent should be validated on it. Grindelwald used
to be exactly that — a hand-shaped analytic lane at an invented offset — and is
now its own surveyed region at the sites' true projected positions.

The topology contract lives in the engine-independent
`TerrainRenderLayout.h`. Headless tests enforce geographic coverage, sub-25 m
sampling and conservative vertex/triangle budgets.

## Level 3 renderer decision

The visual terrain retains the procedural tile path. A Landscape migration was
rejected for this vertical slice because the current mesh already samples the
20 m source grid directly, has watertight shared edges and per-tile culling,
uses no cooked collision, and feeds the same coordinates as physics. Landscape
or Nanite would not recover detail absent from the heightfield and would retain
additional non-Nanite data for systems that require it. The retained path will
spend its budget on material frequency bands, overlays and camera-budgeted
contact detail instead. Reconsider only if an authored-painting or streaming
requirement cannot be met on tiles, with an equivalent measured prototype.

Every regional build logs wall time, tile count and vertex count against a
250 ms route-switch budget. Repeated resets inside one region remain a no-op;
only an Interlaken/Grindelwald switch may pay the synchronous build. If Apple
Silicon captures exceed the budget, tile-data generation is the first candidate
for worker-thread preparation while component creation stays on the game
thread.

## Alpine surface classification

Vertex shading combines surveyed elevation and normal with:

- meadow, agricultural parcel and forest-floor zones;
- slope-dependent exposed rock;
- elevation and north-aspect snow retention;
- subtle rock strata;
- broad, detail and micro spatial variation;
- local heightfield curvature for gully occlusion and ridge definition;
- terrain exposure consistent with the authored alpine sun direction.

Surface bands convert the simulation's local Z back to metres above sea level
using the 565 m Lehn datum before applying alpine and snow thresholds. This is
important on the high Grindelwald shelf: applying a sea-level snow line directly
to local Z silently prevents snow from ever appearing.

The Amisbühl vertical slice uses a tapered procedural Lake Thun footprint at
the established -6.8 m local datum. It replaces the former rectangular Engine
plane, whose straight boundary read as a false horizon and occluded terrain far
beyond the intended shoreline. This polygon is render-only and has no collision;
flight and clearance continue to query `TerrainModel`. Its dedicated
`M_WaterSurface` material uses a Fresnel-weighted deep/grazing colour response
with restrained world-space broad and fine breakup; weather changes its
roughness and specular response through a dynamic material instance;
the Aare reuses it through a darker, rougher dynamic instance on a continuous
terrain-following ribbon. Lake Thun's water edge is softened by an 18 m
render-only wet-bank strip: its inner edge follows the water datum and its outer
edge samples `TerrainModel`, without changing collision or flight height
queries. Brienz and less uniform authored shoreline variation remain later
Level 3 work.

The fixed midday capture showed that the broad straight blue division remains
after removing the rectangular lake proxy; it is therefore the atmospheric
horizon/aerial-perspective boundary, not lake geometry. This is retained as
evidence against repeatedly “fixing” the water for an atmospheric feature.

Every tile uses its own render bounds, allowing Unreal frustum and occlusion
culling to reject distant/off-screen portions of the drawn region. Both landing
fields and their approaches sit inside the Interlaken region; both Grindelwald
routes sit inside the Grindelwald one. Region-at-a-time drawing is the
intermediate scalable path — orthophoto authoring, virtual textures and a true
World Partition content build remain future production work.
