# Surveyed terrain

Two route-aligned ESRI ASCII grids, one per surveyed region:

```sh
/path/to/python3 Tools/Terrain/build_heightfield.py interlaken
/path/to/python3 Tools/Terrain/build_heightfield.py grindelwald
```

| region | file | bounds (local m) | grid | tiles |
|---|---|---|---|---|
| `interlaken` | `interlaken.asc` | x [-1800, 6100], y [-3500, 4500] | 396 × 401 | 81 |
| `grindelwald` | `grindelwald.asc` | x [4500, 11500], y [-18500, -14000] | 351 × 226 | 50 |

The generator queries swisstopo's official current-asset catalogue, downloads
only intersecting 2 m swissALTI3D COG tiles, verifies each one decodes at
500 × 500, rotates them into the Amisbühl–Lehn frame, downsamples to 20 m and
writes source URLs and SHA-256 hashes to `<region>.provenance.json`.

Both regions share **one** route frame: local `+X` along the Amisbühl–Lehn
route, local `+Y` to route-**right**, elevations in metres relative to the Lehn
landing field at 565 m. A coordinate therefore means one thing everywhere,
whichever grid answers for it. The frame is forward/right/up to match the flight
frame and Unreal's handedness; see
`Source/Parapenting/Physics/ParagliderCoordinateSystem.h`.

Two grids rather than one covering both: the valleys are 20 km apart and the
ground between them is never flown, so a single grid would carry 250 km² of
terrain nobody sees. `TerrainModel` loads regions additively and the first
region covering a sample answers for it; the renderer draws one region at a time
and rebuilds when a route change crosses between them.

At startup the same bilinear heightfield is used by the visible procedural mesh,
ground collision, AGL, slope normals, ridge lift and lee-rotor model. Outside
every region the analytic Interlaken proxy takes over — it is a shape, not a
place, and nothing terrain-dependent should be validated on it.

`RouteFrame::regions` carries the same rectangles in C++, and
`terrain_survey_tests` holds the two against each other so a grid generated
against different bounds cannot load silently and move that region's world.

The generated terrain is simulation content, not a navigation product.
