#!/usr/bin/env python3
"""Build route-aligned heightfields from official swissALTI3D COGs.

    python3 build_heightfield.py interlaken
    python3 build_heightfield.py grindelwald

Every region is expressed in the SAME route frame - origin at Amisbuehl oben,
+X along the Amisbuehl -> Lehn route, +Y route-right - so a coordinate means
one thing everywhere in the simulator regardless of which grid answers for it.
Grindelwald is a second grid rather than an extension of the first because the
two valleys are 20 km apart and the ground between them is not flown.

Requires Pillow and NumPy. It deliberately uses only public HTTP endpoints and
stores exact source URLs plus hashes beside the generated terrain.
"""

from __future__ import annotations

import argparse
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
TERRAIN = PROJECT / "Content" / "Terrain"

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

OUTPUT_CELL_M = 20.0

# Surveyed regions, all in the one route frame. Bounds are local metres.
REGIONS = {
    # +Y is route-RIGHT. Y_MIN reaches past the Hoehematte landing field, which
    # sits at local y = -2685 and is the landing for four of the shipped
    # routes. The margin beyond it covers the landing circuit rather than
    # stopping at the field boundary. Y_MAX covers the Niederhorn side at
    # y = +3323.
    "interlaken": {
        "bounds": (-1800.0, 6100.0, -3500.0, 4500.0),
        "note": "Eight Interlaken routes: Amisbuehl, Bergbo, Hohwald, "
                "Niederhorn south, landing at Lehn and Hoehematte.",
    },
    # Grindelwald First (x 5941, y -17481) down to Grund (x 9973, y -15085)
    # and Bodmi (x 9074, y -16614), with margin for the landing circuits and
    # the ridges that make the local air. These are the sites' true projected
    # positions; they were previously translated onto an invented lane at
    # y = -8500 because there was no terrain out here to put them on.
    "grindelwald": {
        "bounds": (4500.0, 11500.0, -18500.0, -14000.0),
        "note": "Grindelwald First to Grund and Bodmi.",
    },
}


def local_to_lv95(x_m: float, y_m: float) -> tuple[float, float]:
    """Route frame -> LV95.

    +X runs launch -> landing, +Y is route-RIGHT, matching the flight frame's
    forward/right/up convention and Unreal's handedness. The frame used to be
    route-left, which mirrored the entire surveyed landscape about the route
    axis relative to the flight: Lake Thun rendered on the pilot's left when
    the real lake is west, and pulling the left brake tracked route-right
    across the ground. Nothing converted between the two, and because the
    mirroring was uniform it was invisible in play - only the relationship to
    real geography was wrong, which is what ridge lift, lee rotor, wind
    bearings and landing circuits are built on.
    """
    dx = LANDING_E - LAUNCH_E
    dy = LANDING_N - LAUNCH_N
    length = math.hypot(dx, dy)
    forward_e, forward_n = dx / length, dy / length
    right_e, right_n = forward_n, -forward_e
    return (
        LAUNCH_E + x_m * forward_e + y_m * right_e,
        LAUNCH_N + x_m * forward_n + y_m * right_n,
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


def query_assets(bounds: tuple[float, float, float, float]) -> list[dict]:
    x_min, x_max, y_min, y_max = bounds
    query = urllib.parse.urlencode(FILTERS)
    x_mid = (x_min + x_max) * 0.5
    y_mid = (y_min + y_max) * 0.5
    # Quartered: the catalogue silently returns nothing for a polygon much
    # larger than a few square kilometres rather than reporting a limit.
    quarters = [
        (x_min, x_mid, y_min, y_mid),
        (x_mid, x_max, y_min, y_mid),
        (x_min, x_mid, y_mid, y_max),
        (x_mid, x_max, y_mid, y_max),
    ]
    assets_by_id = {}
    for region in quarters:
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


def readable_tile(path: pathlib.Path) -> bool:
    """True if this file decodes as a complete 500x500 swissALTI3D tile."""
    try:
        with Image.open(path) as image:
            return image.size == (500, 500)
    except Exception:  # noqa: BLE001 - unreadable is unreadable
        return False


def download(asset: dict, attempts: int = 4) -> pathlib.Path:
    """Fetch one tile, with a timeout and retries.

    urlretrieve has no timeout, so a stalled connection hangs the whole build
    indefinitely rather than failing - a fifty-tile region died silently
    part-way through exactly that way. Partial files are never promoted, so a
    resumed run re-fetches only what did not finish.
    """
    CACHE.mkdir(parents=True, exist_ok=True)
    target = CACHE / asset["ass_asset_id"]
    if target.exists() and readable_tile(target):
        return target
    temporary = target.with_suffix(target.suffix + ".partial")
    for attempt in range(1, attempts + 1):
        try:
            request = urllib.request.Request(asset["ass_asset_href"])
            with urllib.request.urlopen(request, timeout=60) as response, \
                    temporary.open("wb") as stream:
                while True:
                    chunk = response.read(1 << 16)
                    if not chunk:
                        break
                    stream.write(chunk)
            # Completeness is decided by whether it decodes at the expected
            # shape, not by a byte count. Tiles over flat ground and water
            # compress well - one legitimate tile here is 77 kB - so a size
            # floor rejects real data and accepts a truncated large one.
            if not readable_tile(temporary):
                raise RuntimeError("tile did not decode at 500x500")
            temporary.replace(target)
            return target
        except Exception as error:  # noqa: BLE001 - retry anything transient
            temporary.unlink(missing_ok=True)
            if attempt == attempts:
                raise
            print(f"    retry {attempt}/{attempts - 1} "
                  f"{asset['ass_asset_id']}: {error}")
    raise RuntimeError("unreachable")


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


def build(region_name: str) -> None:
    region = REGIONS[region_name]
    x_min, x_max, y_min, y_max = region["bounds"]
    output = TERRAIN / f"{region_name}.asc"
    metadata = TERRAIN / f"{region_name}.provenance.json"

    assets = query_assets(region["bounds"])
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

    columns = round((x_max - x_min) / OUTPUT_CELL_M) + 1
    rows = round((y_max - y_min) / OUTPUT_CELL_M) + 1
    grid = np.empty((rows, columns), dtype=np.float32)
    # ESRI ASCII rows run top to bottom in descending Y, so emit y_max first
    # and let HeightfieldGrid perform the standard flip on read.
    for row in range(rows):
        y_m = y_max - row * OUTPUT_CELL_M
        for column in range(columns):
            x_m = x_min + column * OUTPUT_CELL_M
            easting, northing = local_to_lv95(x_m, y_m)
            grid[row, column] = (
                sample_tile(tiles, easting, northing) - LANDING_ELEVATION_M
            )

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="ascii") as stream:
        stream.write(f"ncols {columns}\n")
        stream.write(f"nrows {rows}\n")
        stream.write(f"xllcorner {x_min}\n")
        stream.write(f"yllcorner {y_min}\n")
        stream.write(f"cellsize {OUTPUT_CELL_M}\n")
        stream.write("NODATA_value -9999\n")
        np.savetxt(stream, grid, fmt="%.3f")

    provenance = {
        "schemaVersion": 1,
        "region": region_name,
        "regionNote": region["note"],
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
        "boundsLocalM": [x_min, y_min, x_max, y_max],
        "assets": sources,
        "safety": "Simulator terrain only; never use for real-world navigation.",
    }
    metadata.write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote {output} ({columns} x {rows})")
    print(f"Wrote {metadata}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("region", choices=sorted(REGIONS), nargs="?",
                        default="interlaken")
    build(parser.parse_args().region)


if __name__ == "__main__":
    main()
