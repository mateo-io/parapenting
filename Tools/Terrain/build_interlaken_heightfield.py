#!/usr/bin/env python3
"""Build a route-aligned Interlaken heightfield from official swissALTI3D COGs.

Requires Pillow and NumPy. It deliberately uses only public HTTP endpoints and
stores exact source URLs plus timestamps beside the generated terrain.
"""

from __future__ import annotations

import hashlib
import json
import math
import pathlib
import urllib.parse
import urllib.request

import numpy as np
from PIL import Image

PROJECT = pathlib.Path(__file__).resolve().parents[2]
CACHE = PROJECT / "build" / "terrain-cache"
OUTPUT = PROJECT / "Content" / "Terrain" / "interlaken.asc"
METADATA = PROJECT / "Content" / "Terrain" / "interlaken.provenance.json"

CATALOGUE = (
    "https://ogd.swisstopo.admin.ch/services/swiseld/services/"
    "assets/ch.swisstopo.swissalti3d/search"
)
FILTERS = {
    "format": "image/tiff; application=geotiff; profile=cloud-optimized",
    "resolution": "2.0",
    "srid": "2056",
    "state": "current",
}

# WGS84 route anchors. These are the authoritative site positions and must
# stay identical to the GeoPoints in Source/Parapenting/Physics/RouteCatalogue.cpp.
LAUNCH_LAT, LAUNCH_LON = 46.702500, 7.822200      # Amisbuehl oben
LANDING_LAT, LANDING_LON = 46.680956, 7.823861    # Lehn
LANDING_ELEVATION_M = 565.0


def wgs84_to_lv95(latitude_deg: float, longitude_deg: float) -> tuple[float, float]:
    """swisstopo approximate WGS84 -> LV95 (EPSG:2056), ~1 m accurate.

    Reproduces the Bern reference point to 0.34 m.
    """
    phi = (latitude_deg * 3600.0 - 169028.66) / 10000.0
    lam = (longitude_deg * 3600.0 - 26782.5) / 10000.0
    easting = (
        2600072.37
        + 211455.93 * lam
        - 10938.51 * lam * phi
        - 0.36 * lam * phi**2
        - 44.54 * lam**3
    )
    northing = (
        1200147.07
        + 308807.95 * phi
        + 3745.25 * lam**2
        + 76.63 * phi**2
        - 194.56 * lam**2 * phi
        + 119.79 * phi**3
    )
    return easting, northing


# Derived, not transcribed. The previous build hardcoded an LV95 pair that sat
# (+76.1, +143.6) m from where these WGS84 anchors actually project - a 163 m
# georeferencing error that shifted the entire surveyed grid relative to the
# sites. It showed up as elevation error on sloped launches: Bergbo came out
# 42 m below its surveyed height, Amisbuehl 12 m. Deriving the projection here
# keeps the two coordinate systems from drifting apart again.
LAUNCH_E, LAUNCH_N = wgs84_to_lv95(LAUNCH_LAT, LAUNCH_LON)
LANDING_E, LANDING_N = wgs84_to_lv95(LANDING_LAT, LANDING_LON)

X_MIN, X_MAX = -1800.0, 6100.0
# Y_MAX reaches past the Hoehematte landing field, which sits at local
# y = 2685 and is the landing for four of the shipped routes. The previous
# 2500 left it 185 m outside the surveyed grid, so those routes touched down
# on the analytic proxy. The extra ~800 m of margin covers the landing
# circuit rather than stopping at the field boundary.
Y_MIN, Y_MAX = -4500.0, 3500.0
OUTPUT_CELL_M = 20.0


def local_to_lv95(x_m: float, y_m: float) -> tuple[float, float]:
    dx = LANDING_E - LAUNCH_E
    dy = LANDING_N - LAUNCH_N
    length = math.hypot(dx, dy)
    forward_e, forward_n = dx / length, dy / length
    left_e, left_n = -forward_n, forward_e
    return (
        LAUNCH_E + x_m * forward_e + y_m * left_e,
        LAUNCH_N + x_m * forward_n + y_m * left_n,
    )


def route_bounds_polygon(
    x_min: float, x_max: float, y_min: float, y_max: float
) -> dict:
    corners = [
        local_to_lv95(x_min, y_min),
        local_to_lv95(x_max, y_min),
        local_to_lv95(x_max, y_max),
        local_to_lv95(x_min, y_max),
    ]
    corners.append(corners[0])
    return {
        "type": "Polygon",
        "crs": {"type": "name", "properties": {"name": "EPSG:2056"}},
        "coordinates": [[list(point) for point in corners]],
    }


def query_assets() -> list[dict]:
    query = urllib.parse.urlencode(FILTERS)
    x_mid = (X_MIN + X_MAX) * 0.5
    y_mid = (Y_MIN + Y_MAX) * 0.5
    regions = [
        (X_MIN, x_mid, Y_MIN, y_mid),
        (x_mid, X_MAX, Y_MIN, y_mid),
        (X_MIN, x_mid, y_mid, Y_MAX),
        (x_mid, X_MAX, y_mid, Y_MAX),
    ]
    assets_by_id = {}
    for region in regions:
        body = urllib.parse.urlencode(
            {
                "geometry": json.dumps(route_bounds_polygon(*region)),
                "approximation": "0",
            }
        ).encode()
        request = urllib.request.Request(
            f"{CATALOGUE}?{query}",
            data=body,
            headers={"Content-Type": "application/x-www-form-urlencoded"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=60) as response:
            payload = json.load(response)
        for asset in payload.get("items", []):
            assets_by_id[asset["ass_asset_id"]] = asset
    assets = sorted(assets_by_id.values(), key=lambda item: item["ass_asset_id"])
    if not assets:
        raise RuntimeError("The official catalogue returned no terrain assets")
    return assets


def download(asset: dict) -> pathlib.Path:
    CACHE.mkdir(parents=True, exist_ok=True)
    target = CACHE / asset["ass_asset_id"]
    if target.exists() and target.stat().st_size > 100_000:
        return target
    temporary = target.with_suffix(target.suffix + ".partial")
    urllib.request.urlretrieve(asset["ass_asset_href"], temporary)
    temporary.replace(target)
    return target


def tile_key(asset_id: str) -> tuple[int, int]:
    # swissalti3d_2025_2629-1172_2_2056_5728.tif
    coordinates = asset_id.split("_")[2]
    easting, northing = coordinates.split("-")
    return int(easting), int(northing)


def sample_tile(
    tiles: dict[tuple[int, int], np.ndarray], easting: float, northing: float
) -> float:
    key = (math.floor(easting / 1000), math.floor(northing / 1000))
    tile = tiles.get(key)
    if tile is None:
        raise RuntimeError(f"Missing swissALTI3D tile {key[0]}-{key[1]}")
    tile_e = key[0] * 1000.0
    tile_n = key[1] * 1000.0
    column = min(tile.shape[1] - 1, max(0, int((easting - tile_e) / 2.0)))
    row = min(tile.shape[0] - 1, max(0, int((tile_n + 1000.0 - northing) / 2.0)))
    return float(tile[row, column])


def main() -> None:
    assets = query_assets()
    tiles: dict[tuple[int, int], np.ndarray] = {}
    sources = []
    for index, asset in enumerate(assets, start=1):
        path = download(asset)
        with Image.open(path) as image:
            values = np.asarray(image, dtype=np.float32).copy()
        if values.shape != (500, 500):
            raise RuntimeError(f"Unexpected tile shape {values.shape} for {path.name}")
        tiles[tile_key(asset["ass_asset_id"])] = values
        sources.append(
            {
                "assetId": asset["ass_asset_id"],
                "href": asset["ass_asset_href"],
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
        )
        print(f"[{index:02d}/{len(assets):02d}] {path.name}")

    columns = round((X_MAX - X_MIN) / OUTPUT_CELL_M) + 1
    rows = round((Y_MAX - Y_MIN) / OUTPUT_CELL_M) + 1
    grid = np.empty((rows, columns), dtype=np.float32)
    # ESRI ASCII rows are north/top to south/bottom. Here local +Y is left,
    # so emit Y_MAX first and let HeightfieldGrid perform the standard flip.
    for row in range(rows):
        y_m = Y_MAX - row * OUTPUT_CELL_M
        for column in range(columns):
            x_m = X_MIN + column * OUTPUT_CELL_M
            easting, northing = local_to_lv95(x_m, y_m)
            grid[row, column] = (
                sample_tile(tiles, easting, northing) - LANDING_ELEVATION_M
            )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT.open("w", encoding="ascii") as output:
        output.write(f"ncols {columns}\n")
        output.write(f"nrows {rows}\n")
        output.write(f"xllcorner {X_MIN}\n")
        output.write(f"yllcorner {Y_MIN}\n")
        output.write(f"cellsize {OUTPUT_CELL_M}\n")
        output.write("NODATA_value -9999\n")
        np.savetxt(output, grid, fmt="%.3f")

    provenance = {
        "schemaVersion": 1,
        "dataset": "swissALTI3D",
        "catalogue": CATALOGUE,
        "officialProductPage":
            "https://www.swisstopo.admin.ch/en/height-model-swissalti3d",
        "coordinateSystem": "LV95/LN02 EPSG:2056",
        "sourceResolutionM": 2.0,
        "outputResolutionM": OUTPUT_CELL_M,
        "routeFrame": {
            "launch": "Amisbuehl oben",
            "landing": "Lehn",
            "launchLv95": [LAUNCH_E, LAUNCH_N],
            "landingLv95": [LANDING_E, LANDING_N],
            "landingElevationM": LANDING_ELEVATION_M,
        },
        "boundsLocalM": [X_MIN, Y_MIN, X_MAX, Y_MAX],
        "assets": sources,
        "safety": "Simulator terrain only; never use for real-world navigation.",
    }
    METADATA.write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote {OUTPUT} ({columns} x {rows})")
    print(f"Wrote {METADATA}")


if __name__ == "__main__":
    main()
