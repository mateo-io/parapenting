#pragma once

#include "BillowRelaxation.h"
#include "ParagliderDynamics.h"

#include <string>
#include <vector>

namespace Parapenting::Physics
{
// Level 1 of the master plan: the canopy is manufactured, not sculpted.
//
// Today the inflated shape is authored directly as a 3D surface from tuned
// constants in the render path - half span 465 cm, arch 650 cm, local chord
// 280 cm * sqrt(1 - 0.72 s^2) - while the physics separately assumes an
// aspect ratio of 5.2. The two are unrelated numbers describing the same wing,
// which is the duplication guiding rule 1 forbids, and it means billow is
// drawn rather than produced.
//
// This replaces both with a parametric definition built from published
// figures, from which the 3D surface is derived. Two properties are exact by
// construction rather than fitted:
//
//   * flat span. The arc is integrated numerically along its own arc length,
//     so the developed span is the published flat span to machine precision
//     and the projected span falls out of the arc rather than being imposed.
//   * flat area. The chord distribution is scaled so its integral over the
//     developed span equals the published flat area.
//
// Everything downstream - attachment nodes, panels, the render mesh - reads
// from here, so there is one geometry rather than two.

// Published manufacturer figures. These are measurements, not tuning.
struct CanopyGeometrySpec
{
    // BGD EPIC 2 ML, from the manufacturer's published specification.
    int cellCount = 45;
    // Developed (flat) figures.
    double flatSpanM = 11.8;
    double flatAreaM2 = 27.0;
    // Root chord is published, which pins the planform taper directly and is
    // a far better constraint than inferring taper from an area ratio.
    double rootChordM = 2.8;
    // Projected figures. The arc is solved to reproduce projected span;
    // projected area and both aspect ratios are then derived checks.
    double projectedSpanM = 9.3;
    double projectedAreaM2 = 22.8;
    // Published aspect ratios, used as checks only.
    double flatAspectRatio = 5.2;
    double projectedAspectRatio = 3.8;
    // Shapes how quickly the arc curves toward the tips. 1 is a circular arc
    // of uniform curvature; higher values keep the centre flatter and bend the
    // tips more, which is what a modern wing does.
    //
    // Only the arc's span ratio is pinned by published data - projectedSpanM
    // over flatSpanM fixes how much span the arc eats, and the tip angle is
    // solved to hit it exactly. How that curvature is *distributed* is not
    // published, so this exponent is an assumption, and the resulting arc
    // height is a consequence of it: 1.0 gives 64.7 deg at the tip and 2.94 m
    // of arc, 1.9 gives 83.2 deg and 2.49 m. 1.3 sits near the middle with a
    // plausible tip angle. Replace when the rib plan is digitised.
    double arcExponent = 1.3;
    // Peak seam allowance sewn into each panel relative to the straight
    // rib-to-rib distance, tapering to zero at the leading and trailing edges.
    // This is a manufacturing input, not a shape: the inflated section is
    // solved from it, so changing it changes the canopy without any surface
    // being edited by hand.
    double billowFraction = 0.026;
    // Internal cell pressure above ambient. Trim dynamic pressure for this
    // wing is about 65 Pa.
    double internalPressurePa = 65.0;
    FabricProperties fabric{};
};

// A rib station on the developed wing.
struct RibStation
{
    // Signed span fraction, -1 at the left tip to +1 at the right tip,
    // matching ParagliderCoordinateSystem.h.
    double spanFraction = 0.0;
    // Distance along the developed arc from the centre rib, metres. Signed.
    double developedOffsetM = 0.0;
    // Position of the rib's quarter-chord in body axes, metres.
    Vec3 positionM{};
    // Chord length at this station, metres.
    double chordM = 0.0;
    // Local arc tangent angle from horizontal, radians. Zero at the centre.
    double arcAngleRad = 0.0;
};

// Reads a wing geometry from Data/Wings/*.json. The Physics layer is
// engine-independent, so this is a deliberately small key-lookup reader rather
// than a JSON library: the schema is flat and fully known, and adding a
// dependency to read six numbers would not be a trade worth making.
//
// Returns false and leaves `spec` untouched if the file is missing or a
// required key is absent - a partially applied spec would be worse than the
// compiled default, because it would look like it had loaded.
bool LoadCanopyGeometrySpec(
    const std::string& filePath, CanopyGeometrySpec& spec);

class CanopyGeometry
{
public:
    explicit CanopyGeometry(const CanopyGeometrySpec& spec = {});

    const CanopyGeometrySpec& Spec() const { return SpecValue; }
    const std::vector<RibStation>& Ribs() const { return RibStations; }

    // Derived, and exact by construction.
    double DevelopedSpanM() const { return DevelopedSpan; }
    double DevelopedAreaM2() const { return DevelopedArea; }
    // Derived, and checked against the published values.
    double ProjectedSpanM() const { return ProjectedSpan; }
    double ProjectedAreaM2() const { return ProjectedArea; }
    double FlatAspectRatio() const
    {
        return DevelopedSpan * DevelopedSpan / DevelopedArea;
    }
    double ProjectedAspectRatio() const
    {
        return ProjectedSpan * ProjectedSpan / ProjectedArea;
    }
    // Solved so the arc reproduces the published projected span.
    double TipArcAngleRad() const { return TipArcAngle; }
    // Solved so the planform reproduces the published projected area.
    double SolvedTipChordFraction() const { return SolvedTipChord; }
    double RootChordMetres() const { return RootChordM; }

    // Rib-profile surface point in body axes: the section the ribs hold,
    // with no cell bulge. spanFraction in [-1, 1], chordFraction in [0, 1]
    // from leading to trailing edge.
    Vec3 SurfacePointM(double spanFraction, double chordFraction,
                       bool upper) const;

    // The inflated section between two ribs, solved from the seam allowance
    // and internal pressure. Nothing here is authored: the bulge is the
    // equilibrium of a pressurised membrane whose length is what the pattern
    // was cut to.
    CellInflation InflatedSectionAt(double chordFraction) const;

    // Surface point including the solved bulge. `acrossCell` runs 0 to 1 from
    // one rib to the next.
    Vec3 InflatedSurfacePointM(double spanFraction, double chordFraction,
                               double acrossCell, bool upper) const;

    // Mean rib-to-rib spacing on the developed wing.
    double CellSpacingM() const;

    // Interpolated rib station at an arbitrary span fraction.
    RibStation StationAt(double spanFraction) const;

private:
    CanopyGeometrySpec SpecValue;
    std::vector<RibStation> RibStations;
    double DevelopedSpan = 0.0;
    double DevelopedArea = 0.0;
    double ProjectedSpan = 0.0;
    double ProjectedArea = 0.0;
    double TipArcAngle = 0.0;
    double SolvedTipChord = 0.0;
    double RootChordM = 0.0;
};
}
