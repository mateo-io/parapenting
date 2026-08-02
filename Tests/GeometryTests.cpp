// Level 1: the canopy is manufactured, not sculpted.
//
// Checks that the parametric geometry reproduces the published BGD EPIC 2 ML
// specification, and that the properties claimed to be exact by construction
// really are exact rather than merely close.
#include "CanopyGeometry.h"
#include "CanopyLoadPose.h"
#include "BillowRelaxation.h"
#include "SuspensionSystem.h"
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

int main(int argc, char** argv)
{
    const std::string dataPath = argc > 1
        ? argv[1] : "Data/Wings/bgd-epic-2-ml-geometry.json";
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
    std::printf("  tip hangs below apex %7.4f m\n",
        geometry.StationAt(0.0).positionM.z
            - geometry.Ribs().front().positionM.z);
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
            Check(right.positionM.z < 1e-9,
                  "tips hang below the centre at "
                      + std::to_string(fraction));
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

        // The canopy is a dome over the pilot, so height falls monotonically
        // from the centre out to each tip.
        double previousZ = geometry.StationAt(0.0).positionM.z;
        for (int i = 1; i <= 20; ++i)
        {
            const double z =
                geometry.StationAt(static_cast<double>(i) / 20.0).positionM.z;
            Check(z <= previousZ + 1e-12, "arc falls toward the tips");
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

        // The apex is at the centre, not off to one side.
        double highest = centre.positionM.z;
        double highestAt = 0.0;
        for (int i = -40; i <= 40; ++i)
        {
            const double fraction = static_cast<double>(i) / 40.0;
            const double z = geometry.StationAt(fraction).positionM.z;
            if (z > highest) { highest = z; highestAt = fraction; }
        }
        Check(std::fabs(highestAt) < 0.03, "the arc apex is at the centre");
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

    std::printf("\nBillow emerges from the pattern\n");
    {
        // Level 1's exit gate: changing a seam allowance must change the
        // inflated cross-section, with no 3D surface edited by hand.
        const auto SectionFor = [](double allowance, double pressurePa)
        {
            CanopyGeometrySpec spec;
            spec.billowFraction = allowance;
            spec.internalPressurePa = pressurePa;
            return CanopyGeometry(spec).InflatedSectionAt(0.45);
        };

        const CellInflation narrow = SectionFor(0.013, 65.0);
        const CellInflation nominal = SectionFor(0.026, 65.0);
        const CellInflation wide = SectionFor(0.050, 65.0);
        std::printf("  allowance 1.3%%  sagitta %6.2f mm  hoop %5.1f N/m\n",
            narrow.sagittaM * 1000.0, narrow.hoopTensionNPerM);
        std::printf("  allowance 2.6%%  sagitta %6.2f mm  hoop %5.1f N/m\n",
            nominal.sagittaM * 1000.0, nominal.hoopTensionNPerM);
        std::printf("  allowance 5.0%%  sagitta %6.2f mm  hoop %5.1f N/m\n",
            wide.sagittaM * 1000.0, wide.hoopTensionNPerM);

        Check(nominal.sagittaM > narrow.sagittaM
                  && wide.sagittaM > nominal.sagittaM,
              "more seam allowance gives a deeper section");

        // Hoop tension must FALL as allowance rises. A flatter panel has a
        // larger radius, and Laplace makes tension proportional to radius, so
        // cutting a panel flat is what loads it hardest. This is the reason
        // chord-cut billow exists, and it should come out of the solve rather
        // than being asserted anywhere.
        Check(nominal.hoopTensionNPerM < narrow.hoopTensionNPerM
                  && wide.hoopTensionNPerM < nominal.hoopTensionNPerM,
              "hoop tension falls as seam allowance rises");

        // Ovalization: the inflated cell is narrower than the panel it was cut
        // from, by the amount the pattern added.
        Check(std::fabs(nominal.ovalizationFraction - 0.026 / 1.026) < 1e-6,
              "ovalization matches the cut allowance");

        // Pressure holds the section. With none, the cell cannot carry hoop
        // tension and does not hold shape.
        const CellInflation unpressurised = SectionFor(0.026, 0.0);
        Check(!unpressurised.holdsSection, "no pressure means no section");
        Check(nominal.holdsSection, "trim pressure holds the section");

        // Softer cloth stretches further and bulges deeper at the same cut.
        CanopyGeometrySpec soft;
        soft.fabric.membraneStiffnessNPerM = 4000.0;
        const CellInflation softSection =
            CanopyGeometry(soft).InflatedSectionAt(0.45);
        Check(softSection.sagittaM > nominal.sagittaM,
              "softer cloth gives a deeper section at the same cut");

        // A panel cut flat still bulges, because a flat membrane cannot carry
        // a normal pressure - it stretches until it can. It is the most
        // heavily loaded case, which is the wrinkle problem in one number.
        const CellInflation flatCut = SectionFor(0.0, 65.0);
        Check(flatCut.sagittaM > 0.0, "a flat-cut panel still bulges");
        Check(flatCut.hoopTensionNPerM > 3.0 * wide.hoopTensionNPerM,
              "a flat-cut panel is the most heavily loaded");
        std::printf("  flat cut       sagitta %6.2f mm  hoop %5.1f N/m"
                    "  strain %.3f%%\n",
            flatCut.sagittaM * 1000.0, flatCut.hoopTensionNPerM,
            flatCut.fabricStrain * 100.0);

        // The chord-cut profile must leave the edge seams flat.
        const CanopyGeometry nominalGeometry{CanopyGeometrySpec{}};
        Check(nominalGeometry.InflatedSectionAt(0.5).sagittaM
                  > nominalGeometry.InflatedSectionAt(0.0).sagittaM,
              "mid chord bulges more than the leading edge");
        Check(nominalGeometry.InflatedSectionAt(0.5).sagittaM
                  > nominalGeometry.InflatedSectionAt(1.0).sagittaM,
              "mid chord bulges more than the trailing edge");

        // And the inflated surface must actually differ from the rib profile.
        const Vec3 ribProfile =
            nominalGeometry.SurfacePointM(0.3, 0.45, true);
        const Vec3 midCell =
            nominalGeometry.InflatedSurfacePointM(0.3, 0.45, 0.5, true);
        const Vec3 atRib =
            nominalGeometry.InflatedSurfacePointM(0.3, 0.45, 0.0, true);
        Check(midCell.z > ribProfile.z + 1e-4,
              "the cell bulges above the rib profile");
        Check(std::fabs(atRib.z - ribProfile.z) < 1e-9,
              "the surface meets the rib exactly");
    }

    std::printf("\nGeometry data file\n");
    {
        // The compiled defaults and the data file must agree. Same contract as
        // RouteFrame against the terrain provenance: the file is the record,
        // the struct is what runs, and nothing else stops them drifting.
        CanopyGeometrySpec loaded;
        const bool ok = LoadCanopyGeometrySpec(dataPath, loaded);
        std::printf("  %s: %s\n", dataPath.c_str(),
            ok ? "loaded" : "NOT FOUND");
        Check(ok, "geometry data file loads");
        if (ok)
        {
            Check(loaded.cellCount == spec.cellCount, "cell count matches");
            CheckWithin(loaded.flatSpanM, spec.flatSpanM, 1e-9,
                        "flat span matches");
            CheckWithin(loaded.flatAreaM2, spec.flatAreaM2, 1e-9,
                        "flat area matches");
            CheckWithin(loaded.rootChordM, spec.rootChordM, 1e-9,
                        "root chord matches");
            CheckWithin(loaded.projectedSpanM, spec.projectedSpanM, 1e-9,
                        "projected span matches");
            CheckWithin(loaded.projectedAreaM2, spec.projectedAreaM2, 1e-9,
                        "projected area matches");
            CheckWithin(loaded.arcExponent, spec.arcExponent, 1e-9,
                        "arc exponent matches");
            CheckWithin(loaded.billowFraction, spec.billowFraction, 1e-9,
                        "billow fraction matches");
            CheckWithin(loaded.internalPressurePa, spec.internalPressurePa,
                        1e-9, "internal pressure matches");
            CheckWithin(loaded.fabric.membraneStiffnessNPerM,
                        spec.fabric.membraneStiffnessNPerM, 1e-9,
                        "membrane stiffness matches");

            // A wing built from the file must be the wing built from defaults.
            const CanopyGeometry fromFile(loaded);
            CheckWithin(fromFile.DevelopedSpanM(), geometry.DevelopedSpanM(),
                        1e-9, "file-built geometry is identical");
            CheckWithin(fromFile.ProjectedAreaM2(), geometry.ProjectedAreaM2(),
                        1e-9, "file-built projected area is identical");
        }

        // A missing file must leave the spec untouched rather than half-apply.
        CanopyGeometrySpec untouched;
        untouched.flatSpanM = 999.0;
        Check(!LoadCanopyGeometrySpec("no/such/file.json", untouched),
              "a missing file reports failure");
        Check(untouched.flatSpanM == 999.0,
              "a failed load does not modify the spec");
    }

    std::printf("\nSuspension attachments on the canopy\n");
    {
        // Level 1 exit gate: every line must terminate on an actual point of
        // the rendered canopy. Both the mesh and the line endpoints now read
        // their span, chord and arch from this geometry, so the test is that
        // every attachment resolves to a station the surface actually has.
        // The populated EPIC 2 ML plan, not the struct's zeroed default.
        const SuspensionGeometry& suspension = Epic2MlSuspensionGeometry();
        int left = 0;
        int right = 0;
        int groupCount[5] = {0, 0, 0, 0, 0};
        for (const SuspensionAttachment& attachment : suspension.attachments)
        {
            Check(attachment.spanFraction >= -1.0
                      && attachment.spanFraction <= 1.0,
                  "attachment span fraction is on the wing");
            Check(attachment.chordFraction >= 0.0
                      && attachment.chordFraction <= 1.0,
                  "attachment chord fraction is on the section");

            // Resolving through the geometry must land on the lower surface,
            // below the rib profile and inside the span.
            const Vec3 point = geometry.SurfacePointM(
                attachment.spanFraction, attachment.chordFraction, false);
            const RibStation station =
                geometry.StationAt(attachment.spanFraction);
            Check(std::fabs(point.y) <= 0.5 * geometry.ProjectedSpanM() + 1e-9,
                  "attachment is inside the projected span");
            Check(point.z <= station.positionM.z + 1e-9,
                  "attachment is on the lower surface");

            if (attachment.spanFraction < 0.0) ++left;
            else if (attachment.spanFraction > 0.0) ++right;
            ++groupCount[static_cast<int>(attachment.group)];
        }
        std::printf("  %d attachments: %d left, %d right\n",
            static_cast<int>(suspension.attachments.size()), left, right);
        std::printf("  A %d  A' %d  B %d  C %d  brake %d  (per wing: "
                    "%d/%d/%d)\n",
            groupCount[0], groupCount[1], groupCount[2], groupCount[3],
            groupCount[4], groupCount[0] / 2 + groupCount[1] / 2,
            groupCount[2] / 2, groupCount[3] / 2);
        Check(left == right, "attachments are balanced left and right");

        // Observation, not a gate. BGD publish the main-line plan as 3/4/3
        // (A/B/C) per wing; this table gives 4/3/3 counting A' with A. Those
        // are not the same quantity - published mains are the lower lines at
        // the riser, and each cascades into several canopy attachments - so
        // the counts need not match. But having more A attachments than B is
        // the reverse of the usual arrangement, and it is worth resolving
        // against a digitised line plan rather than leaving it to coincidence.
        std::printf("  published main lines 3/4/3 (A/B/C) per wing;"
                    " attachments here are 4/3/3\n");

        // Mirror symmetry, which is the geometry half of the Level 0 gate
        // applied to the suspension.
        for (const SuspensionAttachment& attachment : suspension.attachments)
        {
            if (attachment.spanFraction >= 0.0) continue;
            bool mirrored = false;
            for (const SuspensionAttachment& other : suspension.attachments)
            {
                if (other.group == attachment.group
                    && std::fabs(other.spanFraction + attachment.spanFraction)
                           < 1e-9
                    && std::fabs(other.chordFraction - attachment.chordFraction)
                           < 1e-9)
                    mirrored = true;
            }
            Check(mirrored, "every left attachment has a right mirror");
        }

        // Row ordering along the chord: A ahead of B ahead of C.
        double lastA = 0.0;
        double firstC = 1.0;
        double firstB = 1.0;
        double lastB = 0.0;
        for (const SuspensionAttachment& attachment : suspension.attachments)
        {
            if (attachment.group == SuspensionGroup::A)
                lastA = std::max(lastA, attachment.chordFraction);
            if (attachment.group == SuspensionGroup::B)
            {
                firstB = std::min(firstB, attachment.chordFraction);
                lastB = std::max(lastB, attachment.chordFraction);
            }
            if (attachment.group == SuspensionGroup::C)
                firstC = std::min(firstC, attachment.chordFraction);
        }
        Check(lastA < firstB, "the A row sits ahead of the B row");
        Check(lastB < firstC, "the B row sits ahead of the C row");
    }

    // -- the canopy's swing on its lines ----------------------------------
    //
    // A sign test, and it exists because the renderer used to infer this
    // direction from Unreal's rotator handedness. That inference cannot be
    // checked by any suite here, and a wing that surges backwards looks
    // exactly as convincing as one that surges forwards until a pilot flies
    // it. An explicit displacement can be checked, so the displacement is what
    // the renderer now consumes.
    {
        constexpr double LinesM = 7.3;
        const CanopySwingOffset level = EvaluateCanopySwingOffset(0.0, LinesM);
        Check(std::fabs(level.forwardM) < 1.0e-12
              && std::fabs(level.riseM) < 1.0e-12,
              "a wing that has not swung sits straight above the pilot");

        // Brake pushes the wing back behind the pilot: canopyRelativePitchRad
        // positive, so the canopy must move BACKWARD.
        const CanopySwingOffset aft = EvaluateCanopySwingOffset(0.30, LinesM);
        std::printf("Canopy swung +0.30 rad: %+.2f m along track, %+.2f m "
                    "vertically\n", aft.forwardM, aft.riseM);
        Check(aft.forwardM < -0.5,
              "a positive swing puts the canopy behind the pilot, which is "
              "what brake does");
        Check(aft.riseM < 0.0,
              "and lower, because it is travelling on an arc rather than "
              "sliding along a shelf");

        const CanopySwingOffset forward =
            EvaluateCanopySwingOffset(-0.30, LinesM);
        Check(forward.forwardM > 0.5,
              "and a negative swing puts it ahead - the surge");
        Check(std::fabs(forward.forwardM + aft.forwardM) < 1.0e-12
              && std::fabs(forward.riseM - aft.riseM) < 1.0e-12,
              "the arc is symmetric about straight overhead");

        Check(std::fabs(aft.forwardM + LinesM * std::sin(0.30)) < 1.0e-12,
              "and the displacement is L sin(q) exactly - the renderer is "
              "reading a length off the suspension, not a tuned offset");
    }

    if (Failures)
    {
        std::printf("\n%d geometry check(s) failed.\n", Failures);
        return 1;
    }
    std::printf("\nAll geometry checks passed.\n");
    return 0;
}
