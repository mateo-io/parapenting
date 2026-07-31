#include "PanelUnfolder.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
// Chord-cut billow: the sewn-in extra length is not uniform along the chord.
// It runs out to zero where the panel meets the leading and trailing edges,
// which is what keeps those seams flat and is the whole point of the
// construction - BGD's own chord-cut-billow patterning. Published practice
// puts the transition at roughly the first 10% of chord and the last 20%.
// Uniform billow instead forces curvature right into the edge seams and
// overstates how much of the panel cannot be flattened.
double BillowAtChordImpl(double chordFraction, double peakBillow)
{
    constexpr double LeadingEdgeRunout = 0.10;
    constexpr double TrailingEdgeRunout = 0.20;
    const double t = std::clamp(chordFraction, 0.0, 1.0);
    double taper = 1.0;
    if (t < LeadingEdgeRunout) taper = t / LeadingEdgeRunout;
    else if (t > 1.0 - TrailingEdgeRunout)
        taper = (1.0 - t) / TrailingEdgeRunout;
    // Smooth the corners so curvature does not step at the runout boundary.
    taper = taper * taper * (3.0 - 2.0 * taper);
    return peakBillow * taper;
}

// Half-angle of the circular arc whose length exceeds its chord by `billow`.
double HalfAngleForBillow(double billow)
{
    if (billow <= 1e-12) return 0.0;
    const double target = 1.0 / (1.0 + billow);
    double low = 1e-9;
    double high = 3.0;
    for (int i = 0; i < 200; ++i)
    {
        const double mid = 0.5 * (low + high);
        if (std::sin(mid) / mid > target) low = mid;
        else high = mid;
    }
    return 0.5 * (low + high);
}

double Distance3(const Vec3& a, const Vec3& b)
{
    return Length(b - a);
}

double Distance2(const Vec2& a, const Vec2& b)
{
    return std::hypot(b.x - a.x, b.y - a.y);
}

// Places a point at known distances from two already-placed points. This is
// the whole unrolling primitive: it is what makes every triangle exactly
// isometric, because the new vertex is positioned only by true edge lengths.
//
// Two solutions exist, mirrored across the line ab; `sideSign` picks the one
// that keeps the panel's winding consistent, so the strip does not fold back
// on itself.
Vec2 PlaceByDistances(
    const Vec2& a, const Vec2& b, double radiusA, double radiusB,
    double sideSign)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double d = std::hypot(dx, dy);
    if (d < 1e-12) return a;

    // Degenerate or near-degenerate triangles: clamp rather than produce NaN.
    // A triangle inequality violation here would mean the 3D mesh itself is
    // broken, so clamping keeps the unroll going and leaves the discrepancy
    // visible in the residual instead of poisoning every later vertex.
    double along = (d * d + radiusA * radiusA - radiusB * radiusB) / (2.0 * d);
    along = std::clamp(along, -radiusA, radiusA);
    const double offsetSquared = radiusA * radiusA - along * along;
    const double offset = offsetSquared > 0.0 ? std::sqrt(offsetSquared) : 0.0;

    const double ux = dx / d;
    const double uy = dy / d;
    return {
        a.x + ux * along - uy * offset * sideSign,
        a.y + uy * along + ux * offset * sideSign};
}

// The cross diagonal of the strip quad, in 3D.
double DiagonalLength(const Vec3& a, const Vec3& b) { return Length(b - a); }

double TriangleArea3(const Vec3& a, const Vec3& b, const Vec3& c)
{
    return 0.5 * Length(Cross(b - a, c - a));
}

double TriangleArea2(const Vec2& a, const Vec2& b, const Vec2& c)
{
    return 0.5 * std::fabs(
        (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
}
}

double ChordCutBillowAt(double chordFraction, double peakBillow)
{
    return BillowAtChordImpl(chordFraction, peakBillow);
}

UnfoldedPanel UnfoldCell(
    const CanopyGeometry& geometry, int cellIndex, bool upper,
    double billowFraction, int chordStations, int spanStations)
{
    UnfoldedPanel panel;
    panel.chordStations = std::max(2, chordStations);
    panel.spanStations = std::max(2, spanStations);

    const auto& ribs = geometry.Ribs();
    const int cells = static_cast<int>(ribs.size()) - 1;
    const int cell = std::clamp(cellIndex, 0, cells - 1);
    const double fractionA = ribs[static_cast<std::size_t>(cell)].spanFraction;
    const double fractionB =
        ribs[static_cast<std::size_t>(cell) + 1].spanFraction;

    // Sample the bulged cell surface.
    //
    // Billow is extra spanwise length sewn into the panel relative to the
    // straight rib-to-rib distance, so under internal pressure the fabric
    // bows out into a circular arc whose chord is that straight distance and
    // whose arc length is (1 + billow) times it. Solving the half-angle from
    // sin(t)/t = 1/(1+billow) gives the arc, and the bulge is applied along
    // the local surface normal. With zero billow the arc collapses to the
    // chord and the panel is the taut ruled surface between the ribs.

    // The panel is a strip between two rib curves, sampled along the chord.
    // Two rows, not a grid: a 2-row strip is a triangle strip, which is a
    // spanning tree of the mesh, so every vertex can be placed from an edge it
    // genuinely shares and the unroll is exact everywhere. A wider grid has
    // loops, and no isometric flattening of a loop exists on a non-developable
    // surface - attempting it silently violates the triangle inequality and
    // starts stretching edges, which is the one thing this method must never
    // do.
    //
    // Billow lives in the developed width: the spanwise edge is the arc length
    // the fabric actually has between the ribs, (1 + billow) times the straight
    // rib-to-rib distance, which is the length a sailmaker cuts.
    panel.spanStations = 2;
    panel.surfaceVertices.resize(
        static_cast<std::size_t>(panel.chordStations * 2));
    std::vector<double> developedWidth(
        static_cast<std::size_t>(panel.chordStations), 0.0);

    for (int j = 0; j < panel.chordStations; ++j)
    {
        const double chordFraction = static_cast<double>(j)
            / static_cast<double>(panel.chordStations - 1);
        const Vec3 edgeA =
            geometry.SurfacePointM(fractionA, chordFraction, upper);
        const Vec3 edgeB =
            geometry.SurfacePointM(fractionB, chordFraction, upper);
        panel.surfaceVertices[static_cast<std::size_t>(j * 2)] = edgeA;
        panel.surfaceVertices[static_cast<std::size_t>(j * 2 + 1)] = edgeB;
        developedWidth[static_cast<std::size_t>(j)] =
            Distance3(edgeA, edgeB)
            * (1.0 + ChordCutBillowAt(chordFraction, billowFraction));
    }

    // Unroll the strip. Each vertex is placed from the two endpoints of the
    // edge its triangle shares with the previously placed triangle.
    panel.flatVertices.assign(panel.surfaceVertices.size(), Vec2{});
    const auto At = [&](int j, int i)
    { return static_cast<std::size_t>(j * 2 + i); };

    panel.flatVertices[At(0, 0)] = {0.0, 0.0};
    panel.flatVertices[At(0, 1)] = {0.0, developedWidth[0]};
    for (int j = 1; j < panel.chordStations; ++j)
    {
        const double alongA = Distance3(panel.surfaceVertices[At(j - 1, 0)],
                                        panel.surfaceVertices[At(j, 0)]);
        const double alongB = Distance3(panel.surfaceVertices[At(j - 1, 1)],
                                        panel.surfaceVertices[At(j, 1)]);
        // Triangle {A(j-1), B(j-1), A(j)}: shared edge is the previous rung.
        panel.flatVertices[At(j, 0)] = PlaceByDistances(
            panel.flatVertices[At(j - 1, 0)], panel.flatVertices[At(j - 1, 1)],
            alongA, DiagonalLength(panel.surfaceVertices[At(j - 1, 1)],
                                   panel.surfaceVertices[At(j, 0)]),
            -1.0);
        // Triangle {B(j-1), A(j), B(j)}: shared edge is the diagonal just laid.
        panel.flatVertices[At(j, 1)] = PlaceByDistances(
            panel.flatVertices[At(j, 0)], panel.flatVertices[At(j - 1, 1)],
            developedWidth[static_cast<std::size_t>(j)], alongB, -1.0);
    }

    // Residual.
    //
    // The comparison has to be made against the surface the fabric actually
    // takes, not the straight rib-to-rib line. Both corners of a cross
    // diagonal sit on ribs, where the bulge is zero, but the diagonal itself
    // passes over the bulge - so measuring it as a straight line in 3D
    // understates it by roughly the billow allowance and makes intended bulge
    // look like unrecoverable error. Walking the bulged surface instead
    // isolates the part that is genuinely non-developable.
    //
    // The bulge is the circular arc the sewn-in extra length forms between the
    // ribs: chord equal to the straight distance, arc length (1 + billow)
    // times it. Sampling along the diagonal's parameter line and summing
    // segments approximates the surface path.
    // Continuous point on the bulged surface. `station` is a fractional chord
    // index and `v` the spanwise parameter, so a diagonal can be walked
    // smoothly across the panel. Sampling at rounded integer stations instead
    // makes the path zigzag between two rungs, which reports a large residual
    // even on a flat panel.
    const auto BulgedPoint = [&](double station, double v)
    {
        const double clamped = std::clamp(
            station, 0.0, static_cast<double>(panel.chordStations - 1));
        const int lower = static_cast<int>(std::floor(clamped));
        const int upper2 = std::min(lower + 1, panel.chordStations - 1);
        const double frac = clamped - static_cast<double>(lower);

        const Vec3 a = panel.surfaceVertices[At(lower, 0)]
            + (panel.surfaceVertices[At(upper2, 0)]
               - panel.surfaceVertices[At(lower, 0)]) * frac;
        const Vec3 b = panel.surfaceVertices[At(lower, 1)]
            + (panel.surfaceVertices[At(upper2, 1)]
               - panel.surfaceVertices[At(lower, 1)]) * frac;
        const Vec3 span = b - a;
        Vec3 point = a + span * v;
        const double localBillow = ChordCutBillowAt(
            clamped / static_cast<double>(panel.chordStations - 1),
            billowFraction);
        if (localBillow <= 1e-12) return point;
        const double localHalfAngle = HalfAngleForBillow(localBillow);
        if (localHalfAngle <= 1e-9) return point;

        const double straight = Length(span);
        const double radius = straight / (2.0 * std::sin(localHalfAngle));
        const double angle = (2.0 * v - 1.0) * localHalfAngle;
        const double sagitta =
            radius * (std::cos(angle) - std::cos(localHalfAngle));
        const Vec3 chordwise = Normalized(
            panel.surfaceVertices[At(upper2, 0)]
            - panel.surfaceVertices[At(lower, 0)]);
        Vec3 outward = Normalized(Cross(Normalized(span), chordwise));
        if (!upper) outward = -outward;
        return point + outward * sagitta;
    };

    double sumSquares = 0.0;
    int quadCount = 0;
    constexpr int GeodesicSamples = 96;
    for (int j = 1; j < panel.chordStations; ++j)
    {
        // Path across the bulged surface from A(j-1) to B(j).
        double surfacePath = 0.0;
        Vec3 previous = BulgedPoint(static_cast<double>(j - 1), 0.0);
        for (int k = 1; k <= GeodesicSamples; ++k)
        {
            const double t = static_cast<double>(k)
                / static_cast<double>(GeodesicSamples);
            // Interpolate chord station and spanwise parameter together.
            const Vec3 point =
                BulgedPoint(static_cast<double>(j - 1) + t, t);
            surfacePath += Length(point - previous);
            previous = point;
        }
        const double flat = Distance2(panel.flatVertices[At(j - 1, 0)],
                                      panel.flatVertices[At(j, 1)]);
        if (surfacePath < 1e-12) continue;
        const double fraction = std::fabs(flat - surfacePath) / surfacePath;
        panel.residual.maxFraction =
            std::max(panel.residual.maxFraction, fraction);
        sumSquares += fraction * fraction;
        ++quadCount;
    }
    panel.residual.rmsFraction = quadCount > 0
        ? std::sqrt(sumSquares / static_cast<double>(quadCount)) : 0.0;

    for (int j = 1; j < panel.chordStations; ++j)
    {
        panel.residual.developedAreaM2 +=
            TriangleArea3(panel.surfaceVertices[At(j - 1, 0)],
                          panel.surfaceVertices[At(j - 1, 1)],
                          panel.surfaceVertices[At(j, 0)])
            + TriangleArea3(panel.surfaceVertices[At(j - 1, 1)],
                            panel.surfaceVertices[At(j, 1)],
                            panel.surfaceVertices[At(j, 0)]);
        panel.residual.flatAreaM2 +=
            TriangleArea2(panel.flatVertices[At(j - 1, 0)],
                          panel.flatVertices[At(j - 1, 1)],
                          panel.flatVertices[At(j, 0)])
            + TriangleArea2(panel.flatVertices[At(j - 1, 1)],
                            panel.flatVertices[At(j, 1)],
                            panel.flatVertices[At(j, 0)]);
    }
    return panel;
}

UnfoldResidual UnfoldSkin(
    const CanopyGeometry& geometry, bool upper, double billowFraction,
    int chordStations, int spanStations)
{
    UnfoldResidual total;
    const int cells = static_cast<int>(geometry.Ribs().size()) - 1;
    double sumSquares = 0.0;
    for (int cell = 0; cell < cells; ++cell)
    {
        const UnfoldedPanel panel = UnfoldCell(
            geometry, cell, upper, billowFraction, chordStations, spanStations);
        total.maxFraction =
            std::max(total.maxFraction, panel.residual.maxFraction);
        sumSquares += panel.residual.rmsFraction * panel.residual.rmsFraction;
        total.developedAreaM2 += panel.residual.developedAreaM2;
        total.flatAreaM2 += panel.residual.flatAreaM2;
    }
    total.rmsFraction = cells > 0
        ? std::sqrt(sumSquares / static_cast<double>(cells)) : 0.0;
    return total;
}
}
