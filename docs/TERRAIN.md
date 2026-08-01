# How terrain works

The authoritative description of the coordinate frame, the surveyed data, and
the traps. Read this before touching anything that has an `x` and a `y` in it.

`TERRAIN_AND_MAC.md` covers the geodata sources and Mac delivery;
`TERRAIN_RENDERING.md` covers the vertex/shading detail. This file is the one
that says what a coordinate *means*.

## The frame, in one place

One frame for the whole simulator, defined in
`Source/Parapenting/Physics/RouteFrame.h` and mirrored exactly by
`Tools/Terrain/build_heightfield.py`:

```
origin  Amisbuehl oben launch (46.702500 N, 7.822200 E)
+X      along the Amisbuehl -> Lehn route, bearing ~176.7 deg (~due south)
+Y      route-RIGHT, bearing ~266.7 deg (~due west)
+Z      metres relative to the Lehn landing field at 565 m MSL
CRS     LV95 / LN02, EPSG:2056
units   metres, everywhere, without exception
```

Three consequences that have each cost real debugging time:

1. **+Y is route-right, which makes the frame left-handed in ENU** — the same
   handedness as Unreal and as the flight frame's forward/right/up. The terrain
   frame ran route-*left* for a long time against a flight frame running
   forward/right/up, which mirrored the entire surveyed landscape about the
   route axis. It was invisible in play, because the mirroring was uniform:
   pull left, bank left, watch the world turn left. Only the relationship to
   real geography was wrong — which is exactly what ridge lift, lee rotor, wind
   bearings and landing circuits are built on.

2. **Z is not altitude.** Local z is metres relative to the Lehn field at
   565 m, not MSL and not above ground. A "fixed altitude" sample compares two
   places only if both are in open air at that height, and in mountains they
   usually are not. An early measurement of lee-rotor asymmetry sampled
   `z = 260` on both sides of the valley: open air at one, thirty metres inside
   a hillside at the other. The 0.82-against-0.00 difference that produced was
   read as evidence of a frame bug and carried in three documents for 25
   commits. **When comparing two places, sample at a height above ground.**

3. **Unreal is metres × 100.** The engine layer scales at the boundary and
   nowhere else. No handedness conversion happens there, and none is needed.

Every surveyed region shares this frame, so a coordinate means one thing
everywhere regardless of which grid answers for it.

## Surveyed regions

| region | file | bounds (local m) | grid | tiles |
|---|---|---|---|---|
| `interlaken` | `Content/Terrain/interlaken.asc` | x [-1800, 6100], y [-3500, 4500] | 396 × 401 | 81 |
| `grindelwald` | `Content/Terrain/grindelwald.asc` | x [4500, 11500], y [-18500, -14000] | 351 × 226 | 50 |

**Why two grids and not one.** The valleys are 20 km apart. A single grid
covering both would carry roughly 250 km² of terrain nobody ever flies over,
for the sake of the empty ground between them. Regions are the unit of scale
here, and the same shape World Partition will want later.

`RouteFrame::regions` is the C++ table. `REGIONS` in the generator is the Python
one. `terrain_survey_tests` holds them against each other through the
`<region>.provenance.json` files, so a grid generated against different bounds
cannot load silently and move that region's world.

## Data pipeline

```sh
python3 Tools/Terrain/build_heightfield.py interlaken
python3 Tools/Terrain/build_heightfield.py grindelwald
```

The generator queries swisstopo's official current-asset catalogue, downloads
the intersecting 2 m swissALTI3D COG tiles, rotates them into the route frame,
downsamples to 20 m, and writes the grid plus a provenance file recording every
source URL and SHA-256.

Notes worth keeping:

- The catalogue **silently returns nothing** for a polygon much bigger than a
  few square kilometres. It does not report a limit. The generator quarters
  every region's bounds for this reason.
- Tile completeness is decided by whether the file **decodes at 500 × 500**, not
  by a byte count. Tiles over flat ground and water compress well — one
  legitimate Grindelwald tile is 77 kB — so a size floor rejects real data while
  still accepting a truncated large tile.
- Fetches have a timeout and retries. `urlretrieve` has neither, and a fifty-tile
  region once died silently part-way through because of it.
- Regenerating a region must reproduce its `.asc` **byte for byte** if nothing
  but metadata changed. This is a cheap, real check on generator refactors.

## Reading terrain in C++

`TerrainModel` is a static façade over the loaded regions.

```cpp
TerrainModel::LoadHeightfieldAscii(path);   // additive, one call per region
TerrainModel::LoadedRegionCount();
TerrainModel::HeightM(x, y);                // metres in the Lehn datum
TerrainModel::Normal(x, y);
TerrainModel::IsSurveyed(x, y);
TerrainModel::ProvenanceAt(x, y);           // Surveyed | Analytic
TerrainModel::RidgeExposure(x, y);
TerrainModel::LeeRotorPotential(x, y, wind);
```

Loading is additive and order-independent: regions do not overlap, and the
first one covering a sample answers for it. The game mode loads both during
`InitGame`, before Unreal spawns the default pawn — the pawn's first route reset
must see surveyed launch elevation, and a surface swapped in during `BeginPlay`
is too late.

**Outside every region, `HeightM` returns the analytic Interlaken proxy.** It is
a shape, not a place. It says nothing about real geography and nothing
terrain-dependent should be validated on it. It exists so the world has a floor,
not so the world has ground truth. (There used to be a second, hand-shaped
"Grindelwald lane" proxy; it is gone, and so is the `AnalyticGrindelwald`
provenance value.)

## Routes

`RouteCatalogue` projects every site straight from its WGS84 anchor through
`GeoPointInPrimaryFrameM`. There are no special cases, and there should never be
another one: the Grindelwald pair used to be translated as a group onto an
invented lane at `y = -8500`, which kept their intra-valley geometry right and
put them 20 km from the real valley on ground that read 4683 m where the
published landing field is 950 m.

All ten routes launch and land on surveyed ground. `terrain_survey_tests` prints
the coverage table and fails if that stops being true.

## Weather is anchored per region

Terrain is not the only thing with a position. Two mechanisms in
`AtmosphereModel`:

- **Thermal triggers** — a small table per region, in local coordinates,
  choosing where convection starts. This was a single Interlaken set plus a
  `y > 5000 ? 7500 : 0` lane offset, so anywhere that offset did not reach was
  dead air with no thermals at any time of day.
- **Authored weather volumes** — every preset places its thermal/rotor/sink
  volumes in *Interlaken* coordinates, relative to the Amisbuehl launch. Each
  region carries a three-axis offset that moves them onto its own corridor. All
  three axes matter: offsetting only Y left every authored volume at
  x = 760…2520 while Grindelwald starts at x = 4500, and skipping Z left them
  underground, because Grindelwald's valley floor is 800 m higher. A foehn day
  out there peaked at 0.03 rotor until both were fixed; it is 0.66 now.

If you add anything else with a hardcoded position, it needs the same treatment.

## Rendering

`TerrainRenderLayout` is the engine-independent topology contract, and
`LayoutFor(x, y)` returns the layout of the region containing a position
(Interlaken for anything outside every region). Tile counts are chosen per
region to keep vertex spacing near the 20 m source cells.

The renderer draws **one region at a time**. `AParapentingTerrain::BuildForRegionAt`
rebuilds the tile meshes when the region changes and no-ops when it does not;
`AParagliderPawn::ResetFlight` calls it, which is the single point every route
change funnels through.

Bounds must match the surveyed rectangle exactly. They once ran to y = 10000
while the survey stopped at 2500, so more than half the visible ground was
analytic proxy joined by a hard step — the step read as a vertical wall, and
ridge lift and rotor sampled across it saw terrain that does not exist.

## Checks that exist, and what they would catch

`terrain_survey_tests`:

- the route frame in `RouteFrame.h` still matches the generator's output
  recorded in each provenance file;
- `RouteCatalogue`'s independently-derived lat/lon frame agrees with the LV95
  one — the frame is defined twice and nothing else checks that;
- **the handedness gate**: left weight shift and left brake each turn the wing
  toward -Y and mirror their right-hand counterparts, *and* Lake Thun — really
  west, therefore route-right of the southbound line — reads 557.7 m MSL at
  y = +2500 on the wing's right. Both halves in one place, which is what was
  missing while the frames disagreed;
- every route endpoint on surveyed ground;
- **published site elevation against surveyed ground**, per site.

That last one is an external check rather than a golden value, and it earns its
keep. A 163 m georeferencing error once put Bergbo 42 m below its surveyed
height and was found exactly this way. It currently reads:

| site | error | site | error |
|---|---|---|---|
| Lehn | +0.4 m | Amisbuehl oben | +2.2 m |
| Bergbo | -0.3 m | Grindelwald Bodmi | +10.7 m |
| Niederhorn south | -1.4 m | Hohwald | -11.5 m |
| Hoehematte | -1.7 m | **Grindelwald First** | **-50.3 m** |
| Grindelwald Grund | -1.7 m | | |

Grindelwald First is the one open anchor gap: 2123 m is the top station, while
its WGS84 pair is on the launch slope below, which the survey puts at 2073 m.
The terrain is the measurement and the anchor is the estimate, so it is recorded
with a named tolerance rather than fitted away by moving the terrain.

## Adding a region

1. Add bounds to `REGIONS` in `Tools/Terrain/build_heightfield.py` and to
   `RouteFrame::regions`. They must agree; the survey test enforces it.
2. Run the generator for the new region.
3. Load it in `AParapentingGameMode::InitGame` and in any test that needs it.
4. Give it thermal triggers and an authored-volume offset in `AtmosphereModel`,
   or it is dead air.
5. Pass its provenance path in `Tools/check-build.sh`.
6. Check the coverage and elevation tables in `terrain_survey_tests`.

## Rules of thumb

- **Never** add a coordinate special case for one route or one site. Project
  from the anchor. The last special case cost 20 km of position error.
- Compare places at a height **above ground**, never at a fixed local z.
- The surveyed grid is the measurement; published anchors are estimates. When
  they disagree, record the gap — do not move the terrain to fit.
- Anything with a hardcoded x/y/z in the physics layer is Interlaken-relative
  until proven otherwise, and needs a per-region offset before another region
  can use it.
