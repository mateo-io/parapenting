#include "CanopyGeometry.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
// Chord distribution over the developed half-span, normalised to 1 at the
// root. Elliptical-ish with a taper floor, which is what a modern low-B
// planform approximates.
double NormalisedChord(double absSpanFraction, double tipFraction)
{
    const double t = std::clamp(absSpanFraction, 0.0, 1.0);
    const double elliptical = std::sqrt(std::max(0.0, 1.0 - t * t));
    // Blend toward the published tip fraction so the tip does not collapse to
    // zero chord the way a pure ellipse would.
    return tipFraction + (1.0 - tipFraction) * elliptical;
}

// Arc tangent angle at a developed offset. Zero at the centre rib, rising to
// tipAngle at the tips. The exponent keeps the centre section flat.
double ArcAngleAt(double signedSpanFraction, double tipAngle, double exponent)
{
    const double t = std::clamp(std::fabs(signedSpanFraction), 0.0, 1.0);
    const double magnitude = tipAngle * std::pow(t, exponent);
    return signedSpanFraction < 0.0 ? -magnitude : magnitude;
}

// Integrates the arc from centre to tip and returns the projected half-span.
// Walking arc length, not horizontal distance, is what keeps developed span
// exact: each step advances a fixed distance along the curve, so the total
// length is the flat half-span by construction and the horizontal extent is
// whatever the curvature leaves.
double ProjectedHalfSpan(
    double developedHalfSpan, double tipAngle, double exponent, int steps)
{
    const double ds = developedHalfSpan / static_cast<double>(steps);
    double y = 0.0;
    for (int i = 0; i < steps; ++i)
    {
        // Midpoint rule: second-order for the same number of samples.
        const double s = (static_cast<double>(i) + 0.5) * ds;
        const double angle =
            ArcAngleAt(s / developedHalfSpan, tipAngle, exponent);
        y += std::cos(angle) * ds;
    }
    return y;
}
}

CanopyGeometry::CanopyGeometry(const CanopyGeometrySpec& spec)
    : SpecValue(spec)
{
    const double developedHalfSpan = 0.5 * SpecValue.flatSpanM;
    constexpr int IntegrationSteps = 2048;

    // Solve the tip arc angle that reproduces the published projected span.
    // Projected span decreases monotonically with tip angle, so bisection is
    // both sufficient and robust; there is no need for a derivative.
    const double targetProjectedHalfSpan = 0.5 * SpecValue.projectedSpanM;
    double low = 0.0;
    double high = 1.5;  // ~86 deg at the tip, far beyond any real wing
    for (int iteration = 0; iteration < 200; ++iteration)
    {
        const double mid = 0.5 * (low + high);
        const double projected = ProjectedHalfSpan(
            developedHalfSpan, mid, SpecValue.arcExponent, IntegrationSteps);
        if (projected > targetProjectedHalfSpan) low = mid;
        else high = mid;
    }
    TipArcAngle = 0.5 * (low + high);

    // Taper is solved from the published root chord.
    //
    // Flat area alone fixes only the chord scale, leaving taper free. The
    // published root chord pins it directly: given flat area and root chord,
    // the mean normalised chord is determined, and taper follows.
    //
    // An earlier attempt solved taper from projected area instead. That is a
    // weaker constraint - it depends on the arc shape as well as the planform,
    // and forcing it drove tip chord to 10.7 cm, which no wing has. Projected
    // area is a better check than it is an input.
    const int ribCountForArea = SpecValue.cellCount + 1;
    const auto NormalisedAreaFor = [&](double tipFraction)
    {
        double integral = 0.0;
        for (int rib = 1; rib < ribCountForArea; ++rib)
        {
            const double fractionA = -1.0 + 2.0 * static_cast<double>(rib - 1)
                / static_cast<double>(ribCountForArea - 1);
            const double fractionB = -1.0 + 2.0 * static_cast<double>(rib)
                / static_cast<double>(ribCountForArea - 1);
            const double meanChord = 0.5 * (
                NormalisedChord(std::fabs(fractionA), tipFraction)
                + NormalisedChord(std::fabs(fractionB), tipFraction));
            integral += meanChord
                * std::fabs(fractionB - fractionA) * developedHalfSpan;
        }
        return integral;
    };
    // Flat area rises monotonically with taper at fixed root chord.
    {
        double lowTip = 0.0;
        double highTip = 1.0;
        for (int iteration = 0; iteration < 200; ++iteration)
        {
            const double mid = 0.5 * (lowTip + highTip);
            if (NormalisedAreaFor(mid) * SpecValue.rootChordM
                    < SpecValue.flatAreaM2)
                lowTip = mid;
            else
                highTip = mid;
        }
        SolvedTipChord = 0.5 * (lowTip + highTip);
    }
    RootChordM = SpecValue.rootChordM;

    // Lay out rib stations at equal developed offsets. Cell count gives cells,
    // so there is one more rib than cells.
    const int ribCount = SpecValue.cellCount + 1;
    RibStations.resize(static_cast<std::size_t>(ribCount));

    // Integrate the arc once from the left tip to the right tip, recording
    // position at every rib.
    constexpr int SubStepsPerCell = 64;
    const int totalSteps = SpecValue.cellCount * SubStepsPerCell;
    const double ds = SpecValue.flatSpanM / static_cast<double>(totalSteps);

    double y = 0.0;
    double z = 0.0;
    std::vector<Vec3> path(static_cast<std::size_t>(totalSteps) + 1);
    path[0] = {0.0, -0.5 * SpecValue.flatSpanM, 0.0};
    for (int i = 0; i < totalSteps; ++i)
    {
        const double s = -0.5 * SpecValue.flatSpanM
            + (static_cast<double>(i) + 0.5) * ds;
        const double angle = ArcAngleAt(
            s / developedHalfSpan, TipArcAngle, SpecValue.arcExponent);
        // The canopy arches over the pilot: seen from behind it is a dome,
        // centre highest and tips hanging lower. In body axes with +Z up that
        // means height must FALL toward both tips.
        //
        // The vertical rate also has to change sign at the centre rib -
        // walking left to right, height rises until the centre and falls after
        // it. Accumulating one sign unconditionally gives a monotonic ramp
        // across the whole span, one tip high and the other low, which is an
        // asymmetric wing rather than an arc.
        y += std::cos(angle) * ds;
        z -= std::copysign(std::fabs(std::sin(angle)), s) * ds;
        path[static_cast<std::size_t>(i) + 1] =
            {0.0, -0.5 * SpecValue.flatSpanM + y, z};
    }

    // Re-centre so the centre rib sits at the origin and the tips are level.
    const double centreZ = path[static_cast<std::size_t>(totalSteps) / 2].z;
    const double centreY = 0.5 * (path.front().y + path.back().y);

    for (int rib = 0; rib < ribCount; ++rib)
    {
        const int pathIndex = rib * SubStepsPerCell;
        const double spanFraction =
            -1.0 + 2.0 * static_cast<double>(rib)
                / static_cast<double>(ribCount - 1);
        RibStation station;
        station.spanFraction = spanFraction;
        station.developedOffsetM = spanFraction * developedHalfSpan;
        station.arcAngleRad = ArcAngleAt(
            spanFraction, TipArcAngle, SpecValue.arcExponent);
        station.chordM = RootChordM
            * NormalisedChord(std::fabs(spanFraction),
                              SolvedTipChord);
        station.positionM = {
            0.0,
            path[static_cast<std::size_t>(pathIndex)].y - centreY,
            path[static_cast<std::size_t>(pathIndex)].z - centreZ};
        RibStations[static_cast<std::size_t>(rib)] = station;
    }

    // Developed span is the sum of rib-to-rib arc lengths, which is the flat
    // span by construction. Compute it rather than assert it, so the test can
    // check the construction actually holds.
    DevelopedSpan = 0.0;
    for (std::size_t i = 1; i < RibStations.size(); ++i)
    {
        DevelopedSpan += std::fabs(RibStations[i].developedOffsetM
            - RibStations[i - 1].developedOffsetM);
    }

    // Developed and projected area by trapezoid over the rib chords.
    DevelopedArea = 0.0;
    ProjectedArea = 0.0;
    for (std::size_t i = 1; i < RibStations.size(); ++i)
    {
        const RibStation& a = RibStations[i - 1];
        const RibStation& b = RibStations[i];
        const double meanChord = 0.5 * (a.chordM + b.chordM);
        DevelopedArea += meanChord
            * std::fabs(b.developedOffsetM - a.developedOffsetM);
        ProjectedArea += meanChord * std::fabs(b.positionM.y - a.positionM.y);
    }
    ProjectedSpan = std::fabs(
        RibStations.back().positionM.y - RibStations.front().positionM.y);
}

RibStation CanopyGeometry::StationAt(double spanFraction) const
{
    const double clamped = std::clamp(spanFraction, -1.0, 1.0);
    const double position = (clamped + 1.0) * 0.5
        * static_cast<double>(RibStations.size() - 1);
    const auto lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = std::min(lower + 1, RibStations.size() - 1);
    const double t = position - static_cast<double>(lower);

    const RibStation& a = RibStations[lower];
    const RibStation& b = RibStations[upper];
    RibStation result;
    result.spanFraction = clamped;
    result.developedOffsetM =
        a.developedOffsetM + (b.developedOffsetM - a.developedOffsetM) * t;
    result.chordM = a.chordM + (b.chordM - a.chordM) * t;
    result.arcAngleRad = a.arcAngleRad + (b.arcAngleRad - a.arcAngleRad) * t;
    result.positionM = a.positionM + (b.positionM - a.positionM) * t;
    return result;
}

Vec3 CanopyGeometry::SurfacePointM(
    double spanFraction, double chordFraction, bool upper) const
{
    const RibStation station = StationAt(spanFraction);
    const double chord = std::clamp(chordFraction, 0.0, 1.0);

    // Section thickness, as a fraction of chord. A single-skin approximation
    // of the profile until Level 1's airfoil digitisation lands; the shape
    // matters here only so upper and lower surfaces are distinguishable.
    constexpr double MaxThicknessFraction = 0.155;
    constexpr double MaxThicknessChordPosition = 0.28;
    const double t = chord < MaxThicknessChordPosition
        ? chord / MaxThicknessChordPosition
        : (1.0 - chord) / (1.0 - MaxThicknessChordPosition);
    const double thickness = MaxThicknessFraction * station.chordM
        * std::sqrt(std::max(0.0, t)) * (upper ? 1.0 : -0.35);

    // Quarter-chord sits at the rib position; +X is forward, so the leading
    // edge is ahead of it.
    const double xFromQuarterChord = (0.25 - chord) * station.chordM;

    return {
        station.positionM.x + xFromQuarterChord,
        station.positionM.y,
        station.positionM.z + thickness};
}

double CanopyGeometry::CellSpacingM() const
{
    return SpecValue.cellCount > 0
        ? DevelopedSpan / static_cast<double>(SpecValue.cellCount) : 0.0;
}

double CanopyGeometry::CellVolumeM3(double spanFraction) const
{
    const double spacing = CellSpacingM();
    if (spacing <= 0.0) return 0.0;
    const RibStation station = StationAt(spanFraction);

    // Trapezoid over the chord. The cross-sectional area at a chord station is
    // the rib-profile thickness times the rib spacing, plus the two bulges the
    // pressurised skin makes between the ribs. A circular arc of sagitta h
    // over a chord w encloses about two thirds of w times h.
    constexpr int Steps = 64;
    double volume = 0.0;
    double previousArea = 0.0;
    for (int step = 0; step <= Steps; ++step)
    {
        const double chordFraction =
            static_cast<double>(step) / static_cast<double>(Steps);
        const Vec3 upper = SurfacePointM(spanFraction, chordFraction, true);
        const Vec3 lower = SurfacePointM(spanFraction, chordFraction, false);
        const double thickness = std::fabs(upper.z - lower.z);
        const CellInflation section = InflatedSectionAt(chordFraction);
        const double bulge = section.holdsSection
            ? 2.0 * (2.0 / 3.0) * spacing * section.sagittaM : 0.0;
        const double area = thickness * spacing + bulge;
        if (step > 0)
        {
            volume += 0.5 * (area + previousArea)
                * station.chordM / static_cast<double>(Steps);
        }
        previousArea = area;
    }
    return volume;
}

CellInflation CanopyGeometry::InflatedSectionAt(double chordFraction) const
{
    const double spacing = CellSpacingM();
    const double allowance =
        ChordCutBillowAt(chordFraction, SpecValue.billowFraction);
    return RelaxCell(spacing, CutWidthForBillow(spacing, allowance),
                     SpecValue.internalPressurePa, SpecValue.fabric);
}

Vec3 CanopyGeometry::InflatedSurfacePointM(
    double spanFraction, double chordFraction, double acrossCell,
    bool upper) const
{
    const Vec3 base = SurfacePointM(spanFraction, chordFraction, upper);
    const CellInflation section = InflatedSectionAt(chordFraction);
    if (!section.holdsSection || section.radiusM <= 1e-9) return base;

    // Height of the solved arc above its chord at this position across the
    // cell. Zero at both ribs, maximum at mid-cell.
    const double v = std::clamp(acrossCell, 0.0, 1.0);
    const double angle = (2.0 * v - 1.0) * section.halfAngleRad;
    const double height = section.radiusM
        * (std::cos(angle) - std::cos(section.halfAngleRad));

    // Bulge outward from the skin: up on the upper surface, down on the lower.
    return {base.x, base.y, base.z + (upper ? height : -height)};
}

// Finds "key" and reads the next number after it. Skips the "_comment" and
// "why" string fields the schema uses for provenance, because those are for
// humans and must never be parsed as data.
bool ReadJsonNumber(const std::string& text, const std::string& key, double& out)
{
    const std::string quoted = "\"" + key + "\"";
    const std::size_t at = text.find(quoted);
    if (at == std::string::npos) return false;
    std::size_t i = at + quoted.size();
    // Step over a nested object opening, so {"value": 1.3, "why": "..."} works.
    while (i < text.size() && (text[i] == ':' || text[i] == ' '
                               || text[i] == '\n' || text[i] == '\r'
                               || text[i] == '\t'))
        ++i;
    if (i < text.size() && text[i] == '{')
    {
        const std::size_t valueAt = text.find("\"value\"", i);
        if (valueAt == std::string::npos) return false;
        i = valueAt + 7;
        while (i < text.size() && (text[i] == ':' || text[i] == ' ')) ++i;
    }
    if (i >= text.size()) return false;
    if (!(std::isdigit(static_cast<unsigned char>(text[i]))
          || text[i] == '-' || text[i] == '+')) return false;
    try
    {
        std::size_t used = 0;
        out = std::stod(text.substr(i), &used);
        return used > 0;
    }
    catch (...)
    {
        return false;
    }
}

namespace
{
bool ReadNumber(const std::string& text, const std::string& key, double& out)
{
    return ReadJsonNumber(text, key, out);
}
}

bool LoadCanopyGeometrySpec(
    const std::string& filePath, CanopyGeometrySpec& spec)
{
    std::ifstream file(filePath);
    if (!file) return false;
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string text = buffer.str();

    CanopyGeometrySpec loaded;
    double value = 0.0;
    if (!ReadNumber(text, "cellCount", value)) return false;
    loaded.cellCount = static_cast<int>(value);
    if (!ReadNumber(text, "flatSpanM", loaded.flatSpanM)) return false;
    if (!ReadNumber(text, "flatAreaM2", loaded.flatAreaM2)) return false;
    if (!ReadNumber(text, "rootChordM", loaded.rootChordM)) return false;
    if (!ReadNumber(text, "projectedSpanM", loaded.projectedSpanM)) return false;
    if (!ReadNumber(text, "projectedAreaM2", loaded.projectedAreaM2))
        return false;
    if (!ReadNumber(text, "flatAspectRatio", loaded.flatAspectRatio))
        return false;
    if (!ReadNumber(text, "projectedAspectRatio", loaded.projectedAspectRatio))
        return false;
    if (!ReadNumber(text, "arcExponent", loaded.arcExponent)) return false;
    if (!ReadNumber(text, "billowFraction", loaded.billowFraction)) return false;
    if (!ReadNumber(text, "internalPressurePa", loaded.internalPressurePa))
        return false;
    if (!ReadNumber(text, "membraneStiffnessNPerM",
                    loaded.fabric.membraneStiffnessNPerM))
        return false;

    spec = loaded;
    return true;
}
}
