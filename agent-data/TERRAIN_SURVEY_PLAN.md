# Terrain Survey Plan

Status of the surveyed terrain under the simulator, what it covers, and what
is left. The physics work in
[GEOMETRY_DRIVEN_PARAGLIDER_MASTER_PLAN.md](GEOMETRY_DRIVEN_PARAGLIDER_MASTER_PLAN.md)
assumes it is flying over real ground, so this tracks how true that is.

**Resolved so far:** the frame handedness (S0), the 163 m georeferencing error
(S0b), the frame's absence from engine code (S1), the silent survey/fallback
boundary (S2), the render extent overrun (S3), and coverage of every Interlaken
route endpoint (S4). Remaining: Grindelwald (S5) and resolution (S6).

## The local frame

Everything below is in the route-aligned local frame, defined in
`Tools/Terrain/build_interlaken_heightfield.py` and mirrored in
`Source/Parapenting/Physics/RouteFrame.h`, with `TerrainSurveyTests`
cross-checking the two against `interlaken.provenance.json`.

The frame is **forward/right/up** — left-handed, matching the flight frame and
Unreal. See `ParagliderCoordinateSystem.h`.

| Property | Value |
|---|---|
| Origin | Amisbühl oben launch, LV95 `2629334.17, 1172436.78` (projected from WGS84) |
| `+X` | along the Amisbühl→Lehn route, bearing **176.7°** (very nearly due south) |
| `+Y` | route-right, bearing **266.7°** (very nearly due west) |
| `Z` | metres relative to the Lehn landing field, 565.0 m LN02 |
| CRS | LV95 / LN02, EPSG:2056 |

Forward is `(landing − launch)` normalised; right is `(forward_n, −forward_e)`.
Route length launch→landing is **2398.35 m**, so Lehn sits at local
`x = 2398, y = 0`. Lake Thun, being west, sits at **positive** Y.

To convert:

```
E = 2629334.17 + x·0.057859 − y·0.998325
N = 1172436.78 − x·0.998325 − y·0.057859
Z = elevation_LN02 − 565.0
```

The LV95 anchors are projected from the WGS84 site coordinates, not stored.
An earlier build hardcoded a pair 163 m away from where they actually project,
which shifted the whole grid relative to the sites.

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
| Local bounds | x −1800 → 6100, y −3500 → **4500** |
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

## Resolved

### S0 — Frame handedness *(done)*

The terrain frame ran `+Y` = route-left while the flight frame and the Unreal
boundary both ran forward/right/up. Nothing converted between them, so the
surveyed landscape was **mirrored about the route axis** relative to the
flight: Lake Thun rendered on the pilot's left when the real lake is west, and
pulling the left brake tracked route-right across the ground.

Invisible in play, because the mirroring was uniform — pull left, bank left,
world turns left. Only the relationship to real geography was wrong, which is
precisely what ridge lift, lee rotor, wind bearings and landing circuits are
built on.

Resolved by flipping the terrain frame to `+Y` = route-right, so all three
agree. `TerrainSurveyTests` now carries the Level 0 exit-gate check: pilot-left
input must turn toward −Y, symmetrically, *and* Lake Thun must sit at +Y.

### S0b — Georeferencing *(done)*

The generator hardcoded an LV95 anchor pair sitting (+76.1, +143.6) m from
where the WGS84 site anchors actually project — a 163 m shift of the whole
grid. It surfaced as elevation error on sloped launches (Bergbo 42 m low,
Amisbühl 12 m low) while flat valley sites looked fine, which is why it
survived. Both the generator and `RouteFrame` now project LV95 from the WGS84
anchors rather than carrying a second transcribed copy.

Mean elevation error against stated site heights, same four sites:
**14.5 m → 1.1 m**. Across all six sites: **3.0 m**.

### S1 — Frame recorded in engine code *(done)*

`RouteFrame.h` carries origin, basis, datum and surveyed bounds, with the basis
*derived* from the anchors rather than transcribed — a hand-written basis
cannot be held orthonormal, and a slightly non-orthonormal basis bends every
coordinate. `TerrainSurveyTests` checks it against the provenance file, and
separately checks that `RouteCatalogue`'s independently derived lat/lon frame
agrees: they currently differ by **0.85 m** at the landing, now a tracked
number rather than an unknown.

### S2 — Coverage boundary made explicit *(done)*

`TerrainModel::ProvenanceAt` reports `Surveyed`, `AnalyticInterlaken` or
`AnalyticGrindelwald` for any position, and `TerrainSurveyTests` prints the
provenance of every route endpoint on each run.

### S3 — Render extent clamped *(done)*

`TerrainRenderLayout` drew Y to 10000 against a survey ending at 2500, so more
than half the visible ground was analytic proxy joined by a hard step — the
vertical wall in the viewport, and terrain that ridge lift and rotor sampled as
if real. Bounds now match the survey exactly. Tiles retuned to 8×8×50 so sample
spacing is ~20 m on both axes, matching the heightfield: fewer vertices than
before and better matched to the data.

### S4 — Interlaken coverage completed *(done)*

The Höhematte landing field sat at local y = −2685 against a boundary at 2500,
so four of the shipped routes were touching down on analytic terrain. Bounds
extended to cover it with landing-circuit margin.

| | |
|---|---|
| Tiles held | **81** |
| Local bounds | x −1800 → 6100, y −3500 → 4500 |
| Grid | 396 × 401 samples, ~1.1 MB |
| Routes on surveyed ground | **8 of 10** (both exceptions are Grindelwald) |

## Remaining

### S5 — Grindelwald as a second framed region *(decided; 1–2 days)*

**Decision taken: give Grindelwald its own frame.** Own origin, own route
frame, own provenance, loaded as a separate region, so both regions are
geographically true. This needs a region concept the codebase does not have.

Current state: the two Grindelwald routes sit at local y ≈ −8500, translated
into the Interlaken frame rather than placed at their true ~17 km offset. Their
terrain is analytic and their position is invented. Since S3 clamped the render
extent, that lane is also no longer drawn, so those routes currently have no
visible ground.

Anchors, projected from the catalogue's WGS84 points:

| Site | LV95 | Tile |
|---|---|---|
| First (launch) | 2647187, 1167542 | 2647-1167 |
| Grund (landing) | 2645223, 1163396 | 2645-1163 |
| Bodmi (landing) | 2646308, 1164346 | 2646-1164 |

All well outside the current tile set (E 2624–2633). First→Grund is 4588 m on a
bearing of 205°.

Work: parameterise the generator by region config; generalise `RouteFrame` into
a per-region frame; give `TerrainModel` multiple regions with one active;
map each route to its region in `RouteCatalogue`; have the terrain actor build
the active region's bounds.

Do **not** extend the Interlaken grid east to swallow Grindelwald. Besides the
~250 wasted tiles, `HeightM` checks the survey before the analytic lane, so a
grid reaching y = −10000 would make the lane dead code and put the Grindelwald
routes on real terrain of the *wrong place* — near Habkern, ~6 km short of the
village. Plausible-looking and false is worse than an honest proxy.

### S6 — Resolution review *(deferred)*

20 m from a 2 m source is a 10× throwaway. Fine for glide and ridge lift,
marginal for the terrain-driven rotor and venturi modelling in Level 13 of the
physics plan. Revisit when that level is approached, not before — and if it is
raised, raise it in a band around the flown routes rather than globally.

## Suggested order

S5 is the only substantial item left. Until it lands, the two Grindelwald
routes have no surveyed or drawn ground; hiding them from route selection is a
reasonable stopgap.

## Boundary

This is simulator content, not a navigation product. swissALTI3D is
authoritative for elevation; everything derived from it here — the rotation,
the 20 m downsample, the analytic fallback, the Grindelwald lane — is not.
