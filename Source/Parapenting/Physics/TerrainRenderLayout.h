#pragma once

#include <cstddef>

namespace Parapenting::Physics
{
// Engine-independent rendering contract for the surveyed Interlaken region
// and sparse Grindelwald proxy lane.
// Keeping topology outside Unreal makes tile coverage and budgets testable.
struct TerrainRenderLayout
{
    // Bounds match the surveyed heightfield exactly. They previously ran to
    // y = 10000 while the survey stopped at 2500, so more than half the
    // visible ground was analytic proxy with a hard step at the boundary -
    // the step read as a vertical wall, and ridge lift and rotor sampled
    // across it saw terrain that does not exist.
    static constexpr int tileCountX = 8;
    static constexpr int tileCountY = 8;
    // 51x51 vertices per tile puts sample spacing at ~19.8 m in X and 20.0 m
    // in Y, matching the heightfield's own 20 m cells. The old layout
    // oversampled Y at 12.5 m and undersampled X at 24.7 m, spending vertices
    // where there was no more data to resolve.
    static constexpr int cellsPerTile = 50;
    static constexpr double xMinM = -1800.0;
    static constexpr double xMaxM = 6100.0;
    static constexpr double yMinM = -4500.0;
    static constexpr double yMaxM = 3500.0;

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
