#pragma once

#include <cstddef>

namespace Parapenting::Physics
{
// Engine-independent rendering contract for the surveyed Interlaken region
// and sparse Grindelwald proxy lane.
// Keeping topology outside Unreal makes tile coverage and budgets testable.
struct TerrainRenderLayout
{
    static constexpr int tileCountX = 8;
    static constexpr int tileCountY = 16;
    static constexpr int cellsPerTile = 40;
    static constexpr double xMinM = -1800.0;
    static constexpr double xMaxM = 6100.0;
    static constexpr double yMinM = -4500.0;
    static constexpr double yMaxM = 10000.0;

    static constexpr int TileCount() { return tileCountX * tileCountY; }
    static constexpr int VerticesPerTile()
        { return (cellsPerTile + 1) * (cellsPerTile + 1); }
    static constexpr int TrianglesPerTile()
        { return cellsPerTile * cellsPerTile * 2; }
    static constexpr int TotalVertices()
        { return TileCount() * VerticesPerTile(); }
    static constexpr int TotalTriangles()
        { return TileCount() * TrianglesPerTile(); }
    static constexpr double SampleSpacingXM()
        { return (xMaxM - xMinM) / (tileCountX * cellsPerTile); }
    static constexpr double SampleSpacingYM()
        { return (yMaxM - yMinM) / (tileCountY * cellsPerTile); }
};
}
