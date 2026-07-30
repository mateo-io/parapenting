# Surveyed terrain hook

Generate the route-aligned ESRI ASCII grid with:

```sh
/path/to/python3 Tools/Terrain/build_interlaken_heightfield.py
```

The generator queries swisstopo's official current-asset catalogue, downloads
only intersecting 2 m swissALTI3D COG tiles, verifies their shape, rotates them
into the Amisbühl–Lehn frame, downsamples to 20 m and writes source URLs and
SHA-256 hashes to `interlaken.provenance.json`.

The resulting file is `Content/Terrain/interlaken.asc`.
The grid must use metres, with local `+X` along the active route, local `+Y`
to route-left, and elevations relative to the route landing field.

At startup the same bilinear heightfield is used by the visible procedural
mesh, ground collision, AGL, slope normals, ridge lift and lee-rotor model.
When the file is absent or a sample is outside its bounds, the analytic
Interlaken proxy remains active.

The generated terrain is simulation content, not a navigation product.
