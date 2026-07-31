#include "BillowRelaxation.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
double ChordCutBillowAt(double chordFraction, double peakBillow)
{
    constexpr double LeadingEdgeRunout = 0.10;
    constexpr double TrailingEdgeRunout = 0.20;
    const double t = std::clamp(chordFraction, 0.0, 1.0);
    double taper = 1.0;
    if (t < LeadingEdgeRunout) taper = t / LeadingEdgeRunout;
    else if (t > 1.0 - TrailingEdgeRunout)
        taper = (1.0 - t) / TrailingEdgeRunout;
    taper = taper * taper * (3.0 - 2.0 * taper);
    return peakBillow * taper;
}

double CutWidthForBillow(double ribSpacingM, double billowFraction)
{
    return ribSpacingM * (1.0 + std::max(0.0, billowFraction));
}

CellInflation RelaxCell(
    double ribSpacingM, double cutWidthM, double pressurePa,
    const FabricProperties& fabric)
{
    CellInflation result;
    if (ribSpacingM <= 1e-9) return result;

    const double cutWidth = std::max(cutWidthM, ribSpacingM);
    result.ovalizationFraction = cutWidth > 1e-12
        ? (cutWidth - ribSpacingM) / cutWidth : 0.0;

    // A panel cut exactly to the rib spacing is not a special case and must
    // not be short-circuited to "no bulge, no tension". A flat membrane cannot
    // carry a normal pressure at all - tension would have to be infinite - so
    // what actually happens is that the cloth stretches until it has enough
    // length to bow. The solve below finds that equilibrium on its own: as the
    // half-angle goes to zero the radius, and therefore the strain, diverges,
    // so the residual is unbounded below and a root always exists.

    const double stiffness = std::max(1.0, fabric.membraneStiffnessNPerM);
    const double pressure = std::max(0.0, pressurePa);

    // Residual of the arc-length condition at a given half-angle.
    //
    //   R      = c / (2 sin(theta))
    //   T      = p R
    //   strain = T / stiffness
    //   want:  2 R theta  ==  cutWidth * (1 + strain)
    //
    // The left side rises faster than the right, so the residual is monotone
    // and bisection is safe.
    const auto Residual = [&](double theta)
    {
        const double sine = std::sin(theta);
        if (sine < 1e-12) return -cutWidth;
        const double radius = ribSpacingM / (2.0 * sine);
        const double tension = pressure * radius;
        const double strain = tension / stiffness;
        return 2.0 * radius * theta - cutWidth * (1.0 + strain);
    };

    // theta in (0, pi): 0 is flat, pi is a full semicircle. The arc length
    // ratio theta/sin(theta) is monotone increasing over this range, so there
    // is exactly one root for any cut width greater than the rib spacing.
    double low = 1e-7;
    double high = 3.14159265358979 - 1e-7;
    for (int iteration = 0; iteration < 200; ++iteration)
    {
        const double mid = 0.5 * (low + high);
        if (Residual(mid) < 0.0) low = mid;
        else high = mid;
    }
    result.halfAngleRad = 0.5 * (low + high);

    const double sine = std::sin(result.halfAngleRad);
    result.radiusM = sine > 1e-12
        ? ribSpacingM / (2.0 * sine) : 0.0;
    result.hoopTensionNPerM = pressure * result.radiusM;
    result.fabricStrain = result.hoopTensionNPerM / stiffness;
    result.developedWidthM = 2.0 * result.radiusM * result.halfAngleRad;
    result.sagittaM =
        result.radiusM * (1.0 - std::cos(result.halfAngleRad));
    result.holdsSection = result.hoopTensionNPerM >= fabric.minimumTensionNPerM;
    return result;
}
}
