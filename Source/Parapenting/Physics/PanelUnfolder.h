#pragma once

#include "BillowRelaxation.h"
#include "CanopyGeometry.h"

#include <vector>

namespace Parapenting::Physics
{
// Level 1: turning the inflated 3D cell surface back into the flat panel a
// sailmaker would cut.
//
// The surface between two ribs is not developable - it has curvature in both
// directions once billow is sewn in - so no exact flat pattern exists. The
// method here does not pretend otherwise. It triangulates the panel and
// unrolls one triangle at a time, placing each new vertex by circle-circle
// intersection from two already-placed vertices using the true 3D edge
// lengths. Every triangle is therefore *exactly* isometric: no edge is
// stretched anywhere, and no fabric is scaled.
//
// The non-developability does not vanish. It is relocated: it accumulates
// entirely in the quad diagonals that were not used to triangulate, which in
// the real sail appears as micro-puckering along the panel/rib seam. That is
// what chord-cut-billow construction exists to absorb. Bounded and measured is
// the achievable goal; compensated is not.
//
// This is the same family of method as Rhino's Squish/Smash.

struct Vec2
{
    double x = 0.0;
    double y = 0.0;
};

struct UnfoldResidual
{
    // Relative error in the unused quad diagonals: the length unrolling could
    // not preserve, over its 3D length.
    //
    // Read this as a closure error, not as pucker. The panel is cut with the
    // developed width - straight rib-to-rib distance times (1 + billow) - so
    // the flat diagonal is longer than the straight-line 3D diagonal by
    // roughly the billow allowance, which is intended bulge rather than
    // wasted fabric. Isolating the part that is genuinely non-developable
    // needs the diagonal measured across the bulged surface rather than
    // through it, which this does not yet do. Consequently these numbers are
    // NOT comparable to flat-pattern residuals quoted by sailmaking tools,
    // which report only the unrecoverable part; expect these to read several
    // times larger.
    //
    // What the numbers are good for as they stand: they are exactly zero for a
    // developable panel, they grow monotonically with sewn-in billow, and they
    // are identical for mirrored cells. Those are the properties the tests
    // assert.
    double maxFraction = 0.0;
    double rmsFraction = 0.0;
    // Area of the panel in 3D versus laid flat. Isometric triangles preserve
    // area exactly per triangle, so any difference is bookkeeping error.
    double developedAreaM2 = 0.0;
    double flatAreaM2 = 0.0;
};

struct UnfoldedPanel
{
    // Flat pattern, row-major: chordStations columns by spanStations rows.
    std::vector<Vec2> flatVertices;
    // The corresponding 3D surface points, same ordering.
    std::vector<Vec3> surfaceVertices;
    int chordStations = 0;
    int spanStations = 0;
    UnfoldResidual residual;
};

// Samples the bulged surface of one cell and unrolls it.
//
//   cellIndex     0 .. cellCount-1, left to right
//   upper         which skin
//   billowFraction extra spanwise length sewn in, relative to the straight
//                 rib-to-rib distance. 0 gives a taut ruled surface.
UnfoldedPanel UnfoldCell(
    const CanopyGeometry& geometry, int cellIndex, bool upper,
    double billowFraction, int chordStations = 24, int spanStations = 9);

// Unfolds every cell of one skin and reports the worst and RMS residual.
UnfoldResidual UnfoldSkin(
    const CanopyGeometry& geometry, bool upper, double billowFraction,
    int chordStations = 24, int spanStations = 9);
}
