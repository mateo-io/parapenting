# Terrain and macOS delivery

## Terrain source

Use official swisstopo Open Government Data:

- **swissALTI3D 2 m COG** for the v0 landscape heightfield
- **SWISSIMAGE** for alignment/reference and, where appropriate, a base texture
- **swissTLM3D** for water, roads, forest boundaries, names, and landmark layout

The source datasets use Swiss LV95/LN02 coordinates. Keep that coordinate system
through preprocessing and define one fixed local origin near the midpoint of the
route. Store the LV95 origin in metadata; convert to Unreal centimetres only at
the final import boundary. Never treat latitude/longitude as planar metres.

Recommended v0 extent: an 8–12 km square around the route. Start at 2 m terrain
resolution, then downsample to an Unreal-compatible tiled heightfield. Preserve
the source raster and transformation metadata outside Unreal so terrain can be
rebuilt automatically.

Required attribution for shipped assets should be confirmed against the
downloaded dataset's current terms. The expected credit is:

> Source: Federal Office of Topography swisstopo

Do not ship swisstopo *sample* files; those are for testing only. Download the
actual Open Government Data product tiles.

## Import pipeline

1. Select intersecting swissALTI3D tiles in LV95.
2. Mosaic and crop to the chosen square extent.
3. Fill/inspect nodata and record min/max elevation.
4. Resample to tiled 16-bit heightmaps compatible with Unreal Landscape.
5. Generate matching reference imagery tiles.
6. Import with World Partition and a georeferenced local origin.
7. Compare known launch/landing elevations and at least five surveyed landmarks.
8. Add collision, Lehn landing polygon, windsock, and navigation landmarks.

The conversion should become a versioned command-line tool once the final
extent and download files are selected; generated terrain should not be edited
by hand.

## Graphics strategy

- Landscape, atmosphere, clouds, and lighting: Unreal-native systems.
- Scenery: scalable foliage and HLOD/World Partition.
- Canopy: separate low-resolution physics state and high-resolution render mesh.
- Mac presets: avoid assuming Windows feature parity; profile Metal on the
  baseline Apple Silicon machine throughout development.
- Keep gameplay and flight dynamics independent from Nanite, Lumen, cloth, and
  frame rate.

## macOS build gates

1. Confirm the installed Unreal release supports the installed Xcode version.
2. Build the editor target natively on `arm64`.
3. Run headless physics tests before packaging.
4. Package a Development `.app` and smoke-test input, audio, terrain streaming,
   save data, and controller hot-plug.
5. Profile Low/Medium/High presets on the baseline Mac.
6. Create a Developer ID Application certificate.
7. Sign hardened runtime, notarize, staple, and test on a clean Mac.
8. Add a Windows build only after the Mac path is continuously reproducible.

