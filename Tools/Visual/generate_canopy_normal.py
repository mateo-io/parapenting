"""Derive a subtle, tile-safe normal map from the authored ripstop albedo.

This is deliberately a shallow weave normal, not a simulated cloth wrinkle:
the procedural canopy geometry remains responsible for pressure, brake and
collapse deformation. The normal only gives the reinforcement grid a grazing
light response in hero captures.
"""

from pathlib import Path

from PIL import Image, ImageFilter, ImageOps


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "Content/ArtSource/Canopy/T_CanopyRipstop_Albedo_Source_v2.png"
OUTPUT = ROOT / "Content/ArtSource/Canopy/T_CanopyRipstop_Normal_Source_v1.png"


def main():
    image = Image.open(SOURCE).convert("L")
    # Keep broad weave variation but suppress colour/pixel noise. Central
    # differences wrap at the tile boundary, so repeating UVs cannot show a
    # visible lighting seam.
    height = image.filter(ImageFilter.GaussianBlur(radius=0.65))
    width, height_px = height.size
    pixels = height.load()
    normal = Image.new("RGB", (width, height_px))
    out = normal.load()
    strength = 0.72
    for y in range(height_px):
        for x in range(width):
            dx = (pixels[(x + 1) % width, y] - pixels[(x - 1) % width, y]) / 255.0
            dy = (pixels[x, (y + 1) % height_px] - pixels[x, (y - 1) % height_px]) / 255.0
            nx = -dx * strength
            ny = -dy * strength
            nz = 1.0
            length = (nx * nx + ny * ny + nz * nz) ** 0.5
            out[x, y] = (
                round((nx / length * 0.5 + 0.5) * 255),
                round((ny / length * 0.5 + 0.5) * 255),
                round((nz / length * 0.5 + 0.5) * 255),
            )
    normal.save(OUTPUT, optimize=True)
    print(OUTPUT)


if __name__ == "__main__":
    main()
