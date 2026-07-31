#pragma once

#include "CanopyGeometry.h"
#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
// Level 4: the air the canopy has to drag around with it.
//
// A paraglider is a very lightly loaded wing - about 4 kg per square metre -
// and the air it accelerates is not a small correction to that. The classical
// treatment is Lissaman and Brown (AIAA 1993-1236), which idealises the canopy
// as a section of an ellipsoid and gives closed-form coefficients from span,
// chord and thickness alone. Those are the coefficients here, so this needs no
// data beyond the Level 1 geometry.
//
// The literature is unambiguous that this is not optional: apparent mass
// couples the linear and rotational degrees of freedom and materially changes
// how a parafoil pitches and turns. The plan replaces these closed forms with
// coefficients integrated from the actual panel geometry once the solver is
// stable.
//
// Confidence differs between the two halves, and saying so is the point:
//
//   * the LINEAR terms follow the standard ellipsoid idealisation, and the
//     normal one comes out at 33.6 kg against the 31 kg this model already
//     carried as an independent estimate. Two routes to within a tenth is a
//     real cross-check.
//   * the ROTATIONAL terms are dimensionally consistent and scale correctly,
//     but their leading coefficients could not be checked against the source
//     paper here, and the roll term lands well above the estimate already in
//     WingParameters. They are registered Disputed for that reason, the tests
//     assert only their ordering and scaling, and nothing should use their
//     magnitudes until someone reads the paper against them.

struct ApparentMassTensor
{
    // Air accelerated with the canopy, kilograms, in body axes.
    // x is along the chord, y along the span, z normal to the wing.
    Vec3 massKg{};
    // Air rotated with it, kg m^2, about the same axes.
    Vec3 inertiaKgM2{};
};

// Lissaman and Brown's ellipsoid coefficients, evaluated on the wing's own
// dimensions. `thicknessFraction` is section thickness over chord.
ApparentMassTensor LissamanBrownApparentMass(
    double spanM, double meanChordM, double thicknessFraction,
    double airDensityKgM3 = 1.225);

// The same, taken from the manufactured geometry: projected span, mean chord
// from projected area, and the section thickness the polars were built with.
// One wing, so one place these dimensions come from.
ApparentMassTensor CanopyApparentMass(
    const CanopyGeometry& geometry, double thicknessFraction = 0.155,
    double airDensityKgM3 = 1.225);
}
