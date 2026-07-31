// Level 1: the canopy is manufactured, not sculpted.
//
// Checks that the parametric geometry reproduces the published BGD EPIC 2 ML
// specification, and that the properties claimed to be exact by construction
// really are exact rather than merely close.
#include "CanopyGeometry.h"
#include "PanelUnfolder.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace Parapenting::Physics;

namespace
{
int Failures = 0;

void Check(bool condition, const std::string& what)
{
    if (!condition)
    {
        std::printf("  FAIL  %s\n", what.c_str());
        ++Failures;
    }
}

void CheckWithin(double actual, double expected, double percent,
                 const std::string& what)
{
    const double error = 100.0 * (actual - expected) / expected;
    if (!(std::fabs(error) <= percent))
    {
        std::printf("  FAIL  %s: %.4f vs published %.4f (%+.3f%%, allowed %g%%)\n",
            what.c_str(), actual, expected, error, percent);
        ++Failures;
    }
}
}

int main()
{
    const CanopyGeometrySpec spec;
    const CanopyGeometry geometry(spec);

    std::printf("EPIC 2 ML geometry vs published specification\n");
    std::printf("  %-22s %10s %10s %9s\n", "", "published", "derived", "error");
    const auto Row = [](const char* name, double published, double derived)
    {
        std::printf("  %-22s %10.4f %10.4f %+8.3f%%\n", name, published,
            derived, 100.0 * (derived - published) / published);
    };
    Row("flat span (m)", spec.flatSpanM, geometry.DevelopedSpanM());
    Row("flat area (m^2)", spec.flatAreaM2, geometry.DevelopedAreaM2());
    Row("root chord (m)", spec.rootChordM, geometry.StationAt(0.0).chordM);
    Row("projected span (m)", spec.projectedSpanM, geometry.ProjectedSpanM());
    Row("projected area (m^2)", spec.projectedAreaM2,
        geometry.ProjectedAreaM2());
    Row("flat aspect ratio", spec.flatAspectRatio, geometry.FlatAspectRatio());
    Row("projected aspect", spec.projectedAspectRatio,
        geometry.ProjectedAspectRatio());

    // Exact by construction. The arc is walked along its own arc length, so
    // developed span is the published flat span to machine precision; taper is
    // solved against the same trapezoid that measures area, so flat area is
    // exact too; and the tip arc angle is bisected until projected span lands.
    // These are tight because they are constructions, not fits.
    CheckWithin(geometry.DevelopedSpanM(), spec.flatSpanM, 1e-9,
                "developed span is exact");
    CheckWithin(geometry.DevelopedAreaM2(), spec.flatAreaM2, 1e-6,
                "developed area is exact");
    // Solved by bisection on a 2048-step quadrature, then measured across the
    // 45-cell rib path, so the two integrations differ slightly. Tight, but
    // not machine-precision tight the way span and area are.
    CheckWithin(geometry.ProjectedSpanM(), spec.projectedSpanM, 0.01,
                "projected span matches the solve");
    CheckWithin(geometry.StationAt(0.0).chordM, spec.rootChordM, 0.05,
                "root chord matches published");

    // Derived checks. Nothing forces these, so agreement is evidence the arc
    // and planform shapes are approximately right.
    //
    // The aspect ratios are published to two significant figures and are
    // simply span^2/area: 11.8^2/27 = 5.157 and 9.3^2/22.8 = 3.794, so the
    // published 5.2 and 3.8 are rounded and the residual here is theirs, not
    // the model's. Projected area at -0.8% is the one genuine residual, and it
    // reflects the assumed arc curvature distribution.
    CheckWithin(geometry.ProjectedAreaM2(), spec.projectedAreaM2, 1.5,
                "projected area is close");
    CheckWithin(geometry.FlatAspectRatio(), spec.flatAspectRatio, 1.5,
                "flat aspect ratio is close");
    CheckWithin(geometry.ProjectedAspectRatio(), spec.projectedAspectRatio, 1.5,
                "projected aspect ratio is close");

    std::printf("\n  tip arc angle       %8.2f deg\n",
        geometry.TipArcAngleRad() * 180.0 / 3.14159265358979);
    std::printf("  solved tip chord    %8.4f m (%.3f of root)\n",
        geometry.Ribs().front().chordM, geometry.SolvedTipChordFraction());
    std::printf("  arc height          %8.4f m\n",
        geometry.Ribs().front().positionM.z
            - geometry.StationAt(0.0).positionM.z);
    std::printf("  cells / ribs        %8d / %zu\n",
        spec.cellCount, geometry.Ribs().size());

    std::printf("\nStructure\n");
    {
        Check(geometry.Ribs().size()
                  == static_cast<std::size_t>(spec.cellCount) + 1,
              "one more rib than cells");

        // A tip chord that solves to something no wing has means the planform
        // is being forced by the wrong constraint. Solving taper from
        // projected area rather than root chord produced 10.7 cm here.
        const double tipChord = geometry.Ribs().front().chordM;
        Check(tipChord > 0.25 && tipChord < 0.90,
              "tip chord is physically plausible");

        // Symmetry about the centre rib. This is the geometry half of the
        // Level 0 exit gate: left and right must be mirror images.
        for (double fraction : {0.15, 0.4, 0.65, 0.9, 1.0})
        {
            const RibStation left = geometry.StationAt(-fraction);
            const RibStation right = geometry.StationAt(fraction);
            Check(std::fabs(left.chordM - right.chordM) < 1e-9,
                  "chord is symmetric at " + std::to_string(fraction));
            Check(std::fabs(left.positionM.y + right.positionM.y) < 1e-9,
                  "span position is mirrored at " + std::to_string(fraction));
            Check(std::fabs(left.positionM.z - right.positionM.z) < 1e-9,
                  "arc height is symmetric at " + std::to_string(fraction));
        }

        // Chord must fall monotonically from root to tip.
        double previous = geometry.StationAt(0.0).chordM;
        for (int i = 1; i <= 20; ++i)
        {
            const double chord =
                geometry.StationAt(static_cast<double>(i) / 20.0).chordM;
            Check(chord <= previous + 1e-12, "chord tapers monotonically");
            previous = chord;
        }

        // Arc must rise monotonically from centre to tip.
        double previousZ = geometry.StationAt(0.0).positionM.z;
        for (int i = 1; i <= 20; ++i)
        {
            const double z =
                geometry.StationAt(static_cast<double>(i) / 20.0).positionM.z;
            Check(z >= previousZ - 1e-12, "arc rises monotonically");
            previousZ = z;
        }

        // The centreline sits at the origin. Note 45 cells is odd, so there
        // are 46 ribs and none of them lands on the centreline - a cell
        // straddles it, which is how the wing is actually built. StationAt(0)
        // therefore interpolates across that centre cell, and since the arc is
        // convex there it reads a fraction of a millimetre above the true
        // minimum. That is chord-versus-arc over one cell, not an error.
        const RibStation centre = geometry.StationAt(0.0);
        Check(std::fabs(centre.positionM.y) < 1e-9, "centreline at y = 0");
        Check(std::fabs(centre.positionM.z) < 2e-3,
              "centreline within a millimetre of z = 0");
        Check(std::fabs(centre.arcAngleRad) < 1e-12, "centreline is level");
        std::printf("  centre cell straddles the axis; interpolated z = %.6f m\n",
            centre.positionM.z);

        // The arc minimum is at the centre, not off to one side.
        double lowest = centre.positionM.z;
        double lowestAt = 0.0;
        for (int i = -40; i <= 40; ++i)
        {
            const double fraction = static_cast<double>(i) / 40.0;
            const double z = geometry.StationAt(fraction).positionM.z;
            if (z < lowest) { lowest = z; lowestAt = fraction; }
        }
        Check(std::fabs(lowestAt) < 0.03, "the arc minimum is at the centre");
    }

    std::printf("\nSurface\n");
    {
        // Upper and lower surfaces must be distinct everywhere but the edges.
        for (double chord : {0.1, 0.3, 0.5, 0.8})
        {
            const Vec3 upper = geometry.SurfacePointM(0.0, chord, true);
            const Vec3 lower = geometry.SurfacePointM(0.0, chord, false);
            Check(upper.z > lower.z,
                  "upper surface is above lower at chord "
                      + std::to_string(chord));
        }
        // Chord runs leading edge forward to trailing edge aft.
        const Vec3 leadingEdge = geometry.SurfacePointM(0.0, 0.0, true);
        const Vec3 trailingEdge = geometry.SurfacePointM(0.0, 1.0, true);
        Check(leadingEdge.x > trailingEdge.x,
              "leading edge is forward of trailing edge");
        const double chordLength = leadingEdge.x - trailingEdge.x;
        CheckWithin(chordLength, geometry.StationAt(0.0).chordM, 1e-6,
                    "surface chord length matches the station");
    }

    // The physics currently hardcodes an aspect ratio of 5.2 to derive half
    // span, independently of the render geometry. That duplication is what
    // guiding rule 1 forbids, and this is the number it should be reading.
    std::printf("\n  physics half-span from hardcoded AR 5.2: %.4f m\n",
        0.5 * std::sqrt(27.0 * 5.2));
    std::printf("  geometry half-span (projected):          %.4f m\n",
        0.5 * geometry.ProjectedSpanM());
    std::printf("  geometry half-span (developed):          %.4f m\n",
        0.5 * geometry.DevelopedSpanM());

    std::printf("\nIsometric panel unfold\n");
    {
        // The construction guarantee: no edge of the flat pattern is ever
        // stretched. Chordwise seams carry their true 3D length; the rungs
        // carry the developed width, which is the straight rib-to-rib distance
        // times (1 + billow) - the length a sailmaker actually cuts.
        const double billow = geometry.Spec().billowFraction;
        const UnfoldedPanel panel = UnfoldCell(geometry, 20, true, billow);
        const auto At = [&](int j, int i)
        { return static_cast<std::size_t>(j * 2 + i); };
        const auto Flat = [&](std::size_t a, std::size_t b)
        {
            return std::hypot(
                panel.flatVertices[a].x - panel.flatVertices[b].x,
                panel.flatVertices[a].y - panel.flatVertices[b].y);
        };

        double worstSeam = 0.0;
        for (int j = 1; j < panel.chordStations; ++j)
        {
            for (int i = 0; i < 2; ++i)
            {
                const double solid = Length(panel.surfaceVertices[At(j, i)]
                    - panel.surfaceVertices[At(j - 1, i)]);
                worstSeam = std::max(worstSeam,
                    std::fabs(Flat(At(j, i), At(j - 1, i)) - solid) / solid);
            }
        }
        double worstRung = 0.0;
        for (int j = 0; j < panel.chordStations; ++j)
        {
            const double straight = Length(panel.surfaceVertices[At(j, 1)]
                - panel.surfaceVertices[At(j, 0)]);
            const double chordFraction = static_cast<double>(j)
                / static_cast<double>(panel.chordStations - 1);
            const double developed = straight
                * (1.0 + ChordCutBillowAt(chordFraction, billow));
            worstRung = std::max(worstRung,
                std::fabs(Flat(At(j, 0), At(j, 1)) - developed) / developed);
        }
        std::printf("  worst seam edge error  %.3e\n", worstSeam);
        std::printf("  worst rung edge error  %.3e\n", worstRung);
        Check(worstSeam < 1e-12, "chordwise seams are exactly isometric");
        Check(worstRung < 1e-12, "rungs carry exactly the developed width");

        // Chord-cut billow must run out at both edges, so the leading and
        // trailing seams are cut flat.
        Check(ChordCutBillowAt(0.0, billow) == 0.0,
              "no billow at the leading edge");
        Check(ChordCutBillowAt(1.0, billow) == 0.0,
              "no billow at the trailing edge");
        Check(std::fabs(ChordCutBillowAt(0.5, billow) - billow) < 1e-12,
              "full billow at mid chord");

        // Zero billow leaves the taut ruled surface between the ribs, which
        // is very nearly developable but not exactly: adjacent rib sections
        // differ in chord and sit at different arc angles, so the strip
        // between them carries a little Gaussian curvature of its own. 0.05%
        // rms is that, and it is converged - raising the path sampling from
        // 16 to 96 does not move it. It is the floor the construction cannot
        // go below, not a discretisation artefact.
        const UnfoldResidual taut = UnfoldSkin(geometry, true, 0.0);
        std::printf("  residual at zero billow %.4f%% rms\n",
            taut.rmsFraction * 100.0);
        Check(taut.rmsFraction < 1e-3,
              "zero billow is near-developable");

        // Residual must grow monotonically with sewn-in billow: more fabric
        // between the ribs is more curvature, and more curvature is more of
        // the shape that no flat pattern can hold.
        double previous = -1.0;
        std::printf("  %10s %12s %12s\n", "billow", "rms", "max");
        for (double sewn : {0.0, 0.013, 0.026, 0.05, 0.08})
        {
            const UnfoldResidual r = UnfoldSkin(geometry, true, sewn);
            std::printf("  %9.1f%% %11.4f%% %11.4f%%\n",
                sewn * 100.0, r.rmsFraction * 100.0, r.maxFraction * 100.0);
            Check(r.rmsFraction >= previous,
                  "residual grows with billow");
            previous = r.rmsFraction;
        }

        // Left and right must unfold to congruent panels.
        const int cells = static_cast<int>(geometry.Ribs().size()) - 1;
        const UnfoldedPanel left = UnfoldCell(geometry, 3, true, billow);
        const UnfoldedPanel right =
            UnfoldCell(geometry, cells - 4, true, billow);
        double worstMirror = 0.0;
        for (int j = 0; j < left.chordStations; ++j)
        {
            const double a = Flat(At(j, 0), At(j, 1));
            (void)a;
            const double leftRung = std::hypot(
                left.flatVertices[At(j, 0)].x - left.flatVertices[At(j, 1)].x,
                left.flatVertices[At(j, 0)].y - left.flatVertices[At(j, 1)].y);
            const double rightRung = std::hypot(
                right.flatVertices[At(j, 0)].x - right.flatVertices[At(j, 1)].x,
                right.flatVertices[At(j, 0)].y - right.flatVertices[At(j, 1)].y);
            worstMirror = std::max(worstMirror,
                std::fabs(leftRung - rightRung) / leftRung);
        }
        Check(worstMirror < 1e-9, "mirrored cells unfold congruently");
    }

    if (Failures)
    {
        std::printf("\n%d geometry check(s) failed.\n", Failures);
        return 1;
    }
    std::printf("\nAll geometry checks passed.\n");
    return 0;
}
