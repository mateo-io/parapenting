#pragma once

#include "RouteFrame.h"

#include <cstddef>

namespace Parapenting::Physics
{
// Engine-independent rendering contract for the surveyed regions.
// Keeping topology outside Unreal makes tile coverage and budgets testable.
//
// One layout per surveyed region. The renderer draws the region the active
// route flies in - the two are 20 km apart, and nothing on the ground between
// them is ever seen, so drawing both at once would spend the whole vertex
// budget on empty valley floor.
struct TerrainRenderLayout
{
    // Bounds match a surveyed heightfield exactly. They previously ran to
    // y = 10000 while the survey stopped at 2500, so more than half the
    // visible ground was analytic proxy with a hard step at the boundary -
    // the step read as a vertical wall, and ridge lift and rotor sampled
    // across it saw terrain that does not exist.
    double xMinM = -1800.0;
    double xMaxM = 6100.0;
    double yMinM = -3500.0;
    double yMaxM = 4500.0;
    int tileCountX = 8;
    int tileCountY = 8;
    // 51x51 vertices per tile puts sample spacing near the heightfield's own
    // 20 m cells. The old layout oversampled Y at 12.5 m and undersampled X at
    // 24.7 m, spending vertices where there was no more data to resolve. Tile
    // counts are chosen per region to keep that match; see LayoutFor.
    int cellsPerTile = 50;

    constexpr int TileCount() const { return tileCountX * tileCountY; }
    constexpr int VerticesPerTile() const
        { return (cellsPerTile + 1) * (cellsPerTile + 1); }
    constexpr int TrianglesPerTile() const
        { return cellsPerTile * cellsPerTile * 2; }
    constexpr int TotalVertices() const
        { return TileCount() * VerticesPerTile(); }
    constexpr int TotalTriangles() const
        { return TileCount() * TrianglesPerTile(); }
    constexpr double SampleSpacingXM() const
        { return (xMaxM - xMinM) / (tileCountX * cellsPerTile); }
    constexpr double SampleSpacingYM() const
        { return (yMaxM - yMinM) / (tileCountY * cellsPerTile); }

    // The Interlaken region, which is also what a caller with no position
    // gets. Kept as named constants so the eight Interlaken routes render
    // exactly as they did before Grindelwald existed.
    static constexpr double xMinInterlakenM = -1800.0;
    static constexpr double xMaxInterlakenM = 6100.0;
    static constexpr double yMinInterlakenM = -3500.0;
    static constexpr double yMaxInterlakenM = 4500.0;
};

// The layout for the region containing this position, or the Interlaken
// layout if none does. Tile counts hold sample spacing within a couple of
// metres of the 20 m source cells on both regions: Interlaken 7900 m over 400
// cells, Grindelwald 7000 m over 350 and 4500 m over 250.
constexpr TerrainRenderLayout LayoutFor(double xM, double yM)
{
    const RouteFrame::SurveyedRegion* region = RouteFrame::RegionAt(xM, yM);
    if (region == nullptr) return TerrainRenderLayout{};
    TerrainRenderLayout layout;
    layout.xMinM = region->xMinM;
    layout.xMaxM = region->xMaxM;
    layout.yMinM = region->yMinM;
    layout.yMaxM = region->yMaxM;
    // 20 m cells, rounded up to whole tiles of 50.
    const auto tiles = [](double extentM)
    {
        const int cells = static_cast<int>((extentM / 20.0) + 0.999);
        const int count = (cells + 49) / 50;
        return count < 1 ? 1 : count;
    };
    layout.tileCountX = tiles(layout.xMaxM - layout.xMinM);
    layout.tileCountY = tiles(layout.yMaxM - layout.yMinM);
    return layout;
}
}
