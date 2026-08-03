#include "SectionProfile.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double Pi = 3.14159265358979323846;

// NACA four-digit thickness distribution, with the trailing-edge coefficient
// closed (-0.1036 rather than -0.1015) so the contour ends on a point and the
// panel solver has no gap to bridge. Maximum at s = 0.30, nose radius
// 1.1019 t^2 - a real radius, which is the whole reason for using a
// distribution rather than a thin plate.
double ThicknessOrdinate(double s, double thicknessFraction)
{
    const double c = std::clamp(s, 0.0, 1.0);
    return 5.0 * thicknessFraction
        * (0.2969 * std::sqrt(c) - 0.1260 * c - 0.3516 * c * c
           + 0.2843 * c * c * c - 0.1036 * c * c * c * c);
}

// Moves the maximum-thickness station without changing the family: a monotone
// map from the section's own chord to the distribution's, pinned at 0, at the
// requested position, and at 1. It scales the nose radius by
// (0.30 / position), which is the correct first-order consequence of pulling
// the thickness forward.
double ThicknessStation(double x, double position)
{
    const double p = std::clamp(position, 0.05, 0.95);
    const double c = std::clamp(x, 0.0, 1.0);
    if (c <= p) return 0.30 * c / p;
    return 0.30 + 0.70 * (c - p) / (1.0 - p);
}

struct MeanLine
{
    double z = 0.0;
    double slope = 0.0;
};

// NACA four-digit mean line. Two parabolas meeting at the camber station with
// a common tangent.
MeanLine CamberLine(double x, double camberFraction, double camberPosition)
{
    const double m = camberFraction;
    const double p = std::clamp(camberPosition, 0.05, 0.95);
    const double c = std::clamp(x, 0.0, 1.0);
    MeanLine line;
    if (m == 0.0) return line;
    if (c < p)
    {
        line.z = m / (p * p) * (2.0 * p * c - c * c);
        line.slope = 2.0 * m / (p * p) * (p - c);
    }
    else
    {
        const double q = 1.0 - p;
        line.z = m / (q * q) * ((1.0 - 2.0 * p) + 2.0 * p * c - c * c);
        line.slope = 2.0 * m / (q * q) * (p - c);
    }
    return line;
}

// How much of the brake deflection has been taken up by a given chord
// station. Zero ahead of the blend, one at the attachment and aft of it, with
// a smoothstep between, so the bent camber line has a continuous slope and the
// panel solver sees no corner to put a spurious suction peak on.
double BrakeTakeUp(double x, const SectionProfileSpec& spec)
{
    const double hinge = std::clamp(spec.brakeChordFraction, 0.05, 0.99);
    const double width = std::max(1.0e-3, spec.brakeBlendChordFraction);
    const double start = std::max(0.0, hinge - width);
    if (x <= start) return 0.0;
    if (x >= hinge) return 1.0;
    const double t = (x - start) / (hinge - start);
    return t * t * (3.0 - 2.0 * t);
}

// The bend the brake puts into the camber line, as an ordinate. The line's
// local angle below its unbraked direction is the take-up times the full
// deflection; integrating its tangent along the chord is the displacement
// that produces. This is the only place brake enters the geometry - there is
// no separate flap-effectiveness term anywhere, because the panel solver reads
// the bent shape directly.
constexpr std::size_t BendSamples = 512;
using BendTable = std::array<double, BendSamples + 1>;

BendTable BrakeBendTable(const SectionProfileSpec& spec, double brake)
{
    BendTable table{};
    const double deflection =
        std::clamp(brake, 0.0, 1.0) * spec.fullBrakeDeflectionRad;
    table[0] = 0.0;
    double sum = 0.0;
    double previous = 0.0;
    for (std::size_t i = 1; i <= BendSamples; ++i)
    {
        const double x = static_cast<double>(i)
            / static_cast<double>(BendSamples);
        const double angle = deflection * BrakeTakeUp(x, spec);
        const double current = std::tan(angle);
        sum += 0.5 * (current + previous) / static_cast<double>(BendSamples);
        previous = current;
        table[i] = -sum;
    }
    return table;
}

double SampleBend(const BendTable& table, double x)
{
    const double c = std::clamp(x, 0.0, 1.0) * static_cast<double>(BendSamples);
    const auto low = static_cast<std::size_t>(c);
    const std::size_t high = std::min(low + 1, BendSamples);
    const double t = c - static_cast<double>(low);
    return table[low] + (table[high] - table[low]) * t;
}
}

double SectionCamberOrdinate(
    const SectionProfileSpec& spec, double brake, double chordFraction)
{
    const auto bend = BrakeBendTable(spec, brake);
    const MeanLine line = CamberLine(
        chordFraction, spec.maxCamberFraction, spec.maxCamberPosition);
    return line.z + SampleBend(bend, chordFraction);
}

double SectionBrakeTrailingEdgeDrop(
    const SectionProfileSpec& spec, double brake)
{
    return -SampleBend(BrakeBendTable(spec, brake), 1.0);
}

namespace
{
// Surface point at a chord station: the half-thickness laid off normal to the
// bent camber line, which is what keeps the section's own thickness when the
// trailing edge is pulled down.
SectionPoint SurfaceAt(
    const SectionProfileSpec& spec, const BendTable& bend, double deflection,
    double x, bool upper)
{
    const MeanLine line = CamberLine(
        x, spec.maxCamberFraction, spec.maxCamberPosition);
    const double z = line.z + SampleBend(bend, x);
    const double slope =
        line.slope - std::tan(deflection * BrakeTakeUp(x, spec));
    const double phi = std::atan(slope);
    // ThicknessOrdinate is already the half thickness - the NACA polynomial
    // is written for one surface, and its maximum is t/2. Halving it again
    // draws the section at half the thickness it was specified with, which
    // reads as a plausible aerofoil right up until the enclosed volume is
    // integrated.
    const double halfThickness = ThicknessOrdinate(
        ThicknessStation(x, spec.maxThicknessPosition),
        spec.maxThicknessFraction);
    const double sign = upper ? 1.0 : -1.0;
    SectionPoint point;
    point.x = x - sign * halfThickness * std::sin(phi);
    point.z = z + sign * halfThickness * std::cos(phi);
    return point;
}
}

SectionPoint SectionSurfacePoint(
    const SectionProfileSpec& spec, double brake, double chordFraction,
    bool upper)
{
    const double deflection =
        std::clamp(brake, 0.0, 1.0) * spec.fullBrakeDeflectionRad;
    return SurfaceAt(spec, BrakeBendTable(spec, brake), deflection,
                     std::clamp(chordFraction, 0.0, 1.0), upper);
}

SectionProfile BuildSectionProfile(
    const SectionProfileSpec& spec, double brake)
{
    SectionProfile profile;
    const std::size_t panels = std::max<std::size_t>(20, spec.panelCount);
    const std::size_t half = panels / 2;
    const auto bend = BrakeBendTable(spec, brake);
    const double deflection =
        std::clamp(brake, 0.0, 1.0) * spec.fullBrakeDeflectionRad;

    const auto surface = [&](double x, bool upper)
    {
        return SurfaceAt(spec, bend, deflection, x, upper);
    };

    // Cosine spacing, so panels crowd where the curvature is: the nose, and
    // the trailing edge where the brake bend lives.
    const auto station = [half](std::size_t i)
    {
        const double beta = Pi * static_cast<double>(i)
            / static_cast<double>(half);
        return 0.5 * (1.0 - std::cos(beta));
    };

    profile.nodes.reserve(panels + 1);
    // Trailing edge forward along the lower surface to the nose.
    for (std::size_t i = half; i > 0; --i)
    {
        profile.nodes.push_back(surface(station(i), false));
    }
    // The nose itself, on the camber line, shared by both surfaces.
    profile.nodes.push_back(surface(0.0, true));
    // And aft along the upper surface.
    for (std::size_t i = 1; i <= half; ++i)
    {
        profile.nodes.push_back(surface(station(i), true));
    }

    profile.trailingEdgeDropFraction = -SampleBend(bend, 1.0);
    profile.inletChordFraction = spec.inletChordFraction;
    return profile;
}
}
