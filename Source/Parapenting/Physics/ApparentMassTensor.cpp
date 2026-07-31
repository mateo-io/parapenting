#include "ApparentMassTensor.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double Pi = 3.14159265358979323846;
}

ApparentMassTensor LissamanBrownApparentMass(
    double spanM, double meanChordM, double thicknessFraction,
    double airDensityKgM3)
{
    const double span = std::max(0.1, spanM);
    const double chord = std::max(0.1, meanChordM);
    const double thickness =
        std::clamp(thicknessFraction, 0.01, 0.5) * chord;
    const double aspectRatio = span / chord;
    const double density = std::max(0.1, airDensityKgM3);

    // Lissaman and Brown's correction factors. Each is the finite-aspect-ratio
    // reduction on the corresponding two-dimensional ideal, and each tends to
    // one as the wing becomes slender in the relevant sense.
    const double A = 0.666 * (1.0 + 8.0 / 3.0
        * (thickness / chord) * (thickness / chord));
    const double B = 0.267;
    const double C = 0.785 * std::sqrt(1.0 + 2.0
        * (thickness / chord) * (thickness / chord) * (1.0 - thickness / chord))
        * aspectRatio / (1.0 + aspectRatio);

    ApparentMassTensor tensor;
    // Along the chord: the air pushed ahead of a thin plate is small, which is
    // why this term is the least of the three.
    tensor.massKg.x = A * density * thickness * thickness * span;
    // Along the span: edgewise motion moves little air.
    tensor.massKg.y = B * density * chord * chord * span;
    // Normal to the wing: the large one. A flat plate accelerating broadside
    // carries a cylinder of air with it, and for this wing that is comparable
    // to the mass of the pilot.
    tensor.massKg.z = C * density * Pi * 0.25 * chord * chord * span;

    // Rotational terms, from the same idealisation. Roll is about the chord,
    // pitch about the span, yaw about the normal.
    const double P = 0.055 * aspectRatio / (1.0 + aspectRatio);
    const double Q = 0.0308 * aspectRatio / (1.0 + aspectRatio);
    const double R = 0.0555;
    tensor.inertiaKgM2.x = P * density * chord * chord * span * span * span;
    tensor.inertiaKgM2.y = Q * density * chord * chord * chord * chord * span
        * (1.0 + Pi / 6.0 * (1.0 + aspectRatio) * aspectRatio
           * (thickness / chord) * (thickness / chord));
    tensor.inertiaKgM2.z = R * density * thickness * thickness * span * span
        * span;
    return tensor;
}

ApparentMassTensor CanopyApparentMass(
    const CanopyGeometry& geometry, double thicknessFraction,
    double airDensityKgM3)
{
    // The wing that flies is the projected one, and its mean chord is the
    // projected area over the projected span.
    const double span = geometry.ProjectedSpanM();
    const double meanChord = geometry.ProjectedAreaM2()
        / std::max(0.1, geometry.ProjectedSpanM());
    return LissamanBrownApparentMass(
        span, meanChord, thicknessFraction, airDensityKgM3);
}
}
