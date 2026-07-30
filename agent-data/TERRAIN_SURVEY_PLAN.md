# Terrain Survey Plan

Status of the surveyed terrain under the simulator, what it actually covers,
and what it would take to cover the rest. Written because the render layout
currently claims roughly twice the ground the survey provides, and the physics
work in [GEOMETRY_DRIVEN_PARAGLIDER_MASTER_PLAN.md](GEOMETRY_DRIVEN_PARAGLIDER_MASTER_PLAN.md)
assumes it is flying over real ground.

## The local frame

Everything below is in the route-aligned local frame defined by
`Tools/Terrain/build_interlaken_heightfield.py`. It is worth stating
explicitly because it is currently recorded **only** in that script — no C++
source and no field in `interlaken.provenance.json` carries the transform, so
nothing in the engine can verify a sample against its geographic origin.

| Property | Value |
|---|---|
| Origin | Amisbühl oben launch, LV95 `2629258.04, 1172293.04` |
| `+X` | along the Amisbühl→Lehn route, bearing **176.7°** (very nearly due south) |
| `+Y` | route-left, bearing **86.7°** (very nearly due east) |
| `Z` | metres relative to the Lehn landing field, 565.0 m LN02 |
| CRS | LV95 / LN02, EPSG:2056 |

Forward is `(landing − launch)` normalised; left is `(−forward_n, forward_e)`.
Route length launch→landing is **2398.06 m**, so Lehn sits at local
`x = 2398, y = 0`.

To convert:

```
E = 2629258.04 + x·0.057859 + y·0.998325
N = 1172293.04 − x·0.998325 + y·0.057859
Z = elevation_LN02 − 565.0
```

## What has been done

Source is **swissALTI3D 2025**, the official swisstopo height model, fetched
from the live asset catalogue by the generator, hash-verified, rotated into the
route frame, and downsampled from 2 m to 20 m.

| | |
|---|---|
| Tiles held | **73** (1 km × 1 km COGs) |
| Tile extent | E 2624–2632, N 1165–1174 |
| Source resolution | 2.0 m |
| Output resolution | 20.0 m |
| Grid | 396 × 351 samples, ~1.0 MB ESRI ASCII |
| Local bounds | x −1800 → 6100, y −4500 → **2500** |
| Covered area | 7.9 km × 7.0 km |
| Provenance | per-asset URL + SHA-256 in `interlaken.provenance.json` |

The 73 tiles are not a filled rectangle — the generator intersects each tile
against the rotated bounds polygon and skips the 17 corner tiles the rotated
rect never touches. That is correct behaviour, not missing data.

This covers the Interlaken basin, the Beatenberg/Amisbühl ridge, the Lehn
landing field, and the Aare valley floor. Every route whose launch and landing
lie inside these bounds is flying over surveyed ground.

The same bilinear heightfield feeds the visible mesh, ground collision, AGL,
slope normals, ridge lift and the lee-rotor model, so coverage is a physics
question, not only a visual one.

## What is missing

The renderer draws more ground than the survey covers.

| | X | Y |
|---|---|---|
| `interlaken.asc` | −1800 → 6100 | −4500 → **2500** |
| `TerrainRenderLayout` | −1800 → 6100 | −4500 → **10000** |

X agrees to the metre. Y does not: the survey stops at 2500 and the layout runs
to 10000. **7.5 km of the 14.5 km Y span — 51.7% of the rendered area — has no
surveyed data.** Since `+Y` is route-left, that is the whole eastward extension
toward Grindelwald.

Three consequences, in descending severity:

1. **A hard discontinuity at y = 2500.** `HeightfieldGrid::Sample` returns
   false outside its bounds and `TerrainModel::HeightM` drops to the analytic
   proxy with no blend. This is the vertical wall visible on the right of the
   viewport. Ridge lift and rotor sampled across that seam are reading a step
   change in terrain that does not exist.
2. **A dead band, y 2500 → 5000.** Past the survey but before the Grindelwald
   lane, which only engages at `y > 5000`. Roughly 2.5 km of ground served by
   neither the survey nor the regional proxy.
3. **The Grindelwald lane is geographically fictitious.** Grindelwald is ~17 km
   east of the Interlaken frame origin; the lane is deliberately translated to
   sit inside the same local frame rather than placed at its true offset, so
   that verified route geometry could coexist without stretching one mesh
   across the separation. Its terrain is analytic and its position is invented.
   Nothing that depends on real terrain should be validated there.

## What to do

### S1 — Record the frame where the engine can see it *(hours)*

Add the origin, forward/left basis and landing datum to
`interlaken.provenance.json`, and assert them in `TerrainModel` at load. Right
now the transform lives only in the generator, so a regenerated grid with a
different origin would load silently and move the whole world. Cheapest item
here and a prerequisite for trusting anything else.

### S2 — Make the coverage boundary explicit *(hours)*

Have `TerrainModel` expose whether a sample came from survey or fallback, and
surface it: a debug overlay, and a hard assertion that no route waypoint,
launch, landing or still-air test volume lies outside surveyed bounds. Failing
loudly beats a silent step in the terrain.

This is the item the physics plan actually depends on. Level 0's "freeze a
known-good still-air baseline" is only meaningful over surveyed ground, and
right now nothing checks that.

### S3 — Clamp the render layout to the survey *(hours)*

Set `TerrainRenderLayout::yMaxM = 2500` so the visible mesh stops where the
data stops. Removes the wall and the dead band immediately, at the cost of the
Grindelwald proxy lane. Do this if S4 is not going to happen soon — drawing
invented terrain is worse than drawing none.

### S4 — Extend the survey east *(1–2 days)*

Cover the gap band properly. Computed against the actual rotated bounds:

| | |
|---|---|
| Tiles touched by the gap band | 79 |
| Already held | 10 |
| **New tiles to fetch** | **69** |
| New columns | E 2632 → 2639 |
| Resulting grid | 396 × 726 samples (~2× current, ~2 MB) |

The generator already does the whole job — catalogue query, intersection,
download, hash, rotate, downsample. Raising `Y_MAX` should be close to the only
change. Verify the tiles exist in the catalogue first; coverage at the eastern
edge is worth confirming before committing to the approach.

Note this still does **not** reach Grindelwald. Local `y = 10000` lands near
E 2639600, about 6 km short of the village. Extending the grid removes the seam
and the dead band, but the Grindelwald lane stays a proxy.

### S5 — Decide what Grindelwald actually is *(design, then 1–2 days)*

Two honest options; the current state is neither.

- **Second framed heightfield.** Own origin, own route frame, own provenance,
  loaded as a separate region. Geographically true. Costs a region-switching
  concept the codebase does not have.
- **Drop it.** Remove the analytic lane and ship one truthful region until
  there is a reason to do otherwise.

Extending one grid 17 km to swallow both is the option to avoid: it would be
~250 additional tiles, most of them covering ground nobody flies over.

### S6 — Resolution review *(deferred)*

20 m from a 2 m source is a 10× throwaway. Fine for glide and ridge lift,
marginal for the terrain-driven rotor and venturi modelling in Level 13 of the
physics plan. Revisit when that level is approached, not before — and if it is
raised, raise it in a band around the flown routes rather than globally.

## Suggested order

S1 and S2 first: they are cheap, and they convert a silent problem into a loud
one. Then S3 as a stopgap, or go straight to S4 if the eastward extension is
wanted anyway. S5 is a decision, not a task, and should be made before S4 so
the extension is not sized against a region that later gets its own frame.

## Boundary

This is simulator content, not a navigation product. swissALTI3D is
authoritative for elevation; everything derived from it here — the rotation,
the 20 m downsample, the analytic fallback, the Grindelwald lane — is not.
