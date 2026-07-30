# Regional terrain rendering

The Interlaken play area uses one authoritative surveyed heightfield for
atmosphere, landing logic and visible terrain. A sparse analytic Grindelwald
lane extends the same physics coordinate frame to First, Grund and Bodmi while
licensed regional elevation data is still pending. Rendering is divided into
independently cullable tiles without changing physics coordinates.

## Layout

- rendered extent: `x -1800…6100 m`, `y -4500…10000 m`;
- 8 × 16 render tiles;
- 40 × 40 quads per tile;
- approximately 24.7 × 22.7 m sample spacing;
- 215,168 submitted vertices including seam duplication;
- 409,600 triangles;
- no procedural collision cooking.

Shared tile-edge positions are evaluated from identical world coordinates, so
adjacent meshes remain watertight. Landing and ground clearance use
`TerrainModel::HeightM` directly; disabling visual-mesh collision therefore
removes redundant work without changing flight behavior.

The game mode loads the heightfield during `InitGame`, before Unreal spawns the
default pawn. The pawn's first route reset therefore uses the surveyed launch
elevation; it cannot be embedded later by replacing an analytic fallback
surface during `BeginPlay`.

The surveyed Interlaken heightfield is authoritative only inside its imported
coverage. The Grindelwald lane uses a common 565 m world datum and geographic
horizontal offsets from First, preserving world axes and meteorological wind
direction. Its terrain is deliberately a broad simulation proxy: route anchors
match published elevations, but slopes and landforms must not be read as
surveyed ground.

The topology contract lives in the engine-independent
`TerrainRenderLayout.h`. Headless tests enforce geographic coverage, sub-25 m
sampling and conservative vertex/triangle budgets.

## Alpine surface classification

Vertex shading combines surveyed elevation and normal with:

- meadow, agricultural parcel and forest-floor zones;
- slope-dependent exposed rock;
- elevation and north-aspect snow retention;
- subtle rock strata;
- broad, detail and micro spatial variation;
- local heightfield curvature for gully occlusion and ridge definition;
- terrain exposure consistent with the authored alpine sun direction.

Every tile uses its own render bounds, allowing Unreal frustum and occlusion
culling to reject distant/off-screen portions of the expanded tiled region.
The eastern edge fully contains Höhematte and its approach, while the northern
sparse lane contains both Grindelwald routes. This is an intermediate scalable
terrain path. Licensed Grindelwald elevation, orthophoto authoring, virtual
textures and a true World Partition content build remain future production
work.
