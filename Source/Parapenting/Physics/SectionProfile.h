#pragma once

#include <cstddef>
#include <vector>

namespace Parapenting::Physics
{
// The section the ribs are cut to, as coordinates.
//
// Everything above this file used to describe the profile by two numbers -
// thickness and camber - handed to thin-airfoil theory. Thin-airfoil theory
// has no nose radius, so it cannot say where the suction peak runs out, which
// is the only thing that decides where a section stalls. That is why the stall
// angle had to be a stated constant (`stallMarginRad`), and why maximum lift
// came out the same at every brake setting: the model had no geometry to read
// it from.
//
// This is the geometry. A closed contour, generated from the design
// parameters the wing is actually drawn with, panelled with cosine spacing so
// the nose is resolved. Brake bends the camber line rather than hinging a
// flat plate, because a brake line pulls fabric and fabric curves.
//
// Nothing here is fitted to a published flight number. The profile is the
// design; the aerodynamics are whatever the design turns out to have.

struct SectionProfileSpec
{
    // Maximum thickness and where it sits, as chord fractions. 15.5% at 30%
    // chord is what CanopyGeometry cuts the ribs to.
    double maxThicknessFraction = 0.155;
    double maxThicknessPosition = 0.30;
    // Maximum camber and its position. A paraglider profile is strongly
    // cambered and carries that camber well forward.
    double maxCamberFraction = 0.035;
    double maxCamberPosition = 0.38;
    // Where the brake starts taking the trailing edge, as a chord fraction,
    // and how far the trailing edge is bent at full travel.
    double brakeChordFraction = 0.78;
    double fullBrakeDeflectionRad = 0.42;
    // How far ahead of the brake attachment the fabric starts to curve. A
    // hinge is a discontinuity and a real trailing edge is not one; the bend
    // is distributed over this much chord ahead of the attachment.
    double brakeBlendChordFraction = 0.14;
    // The cell opening. A paraglider section is not closed: there is a hole
    // in the nose, and the air that keeps the wing inflated goes through it.
    // The contour is still panelled closed - potential flow does not care
    // about a hole at the stagnation point, and the pressure solver already
    // measures the cell recovering Cp 0.97 through it - but the boundary
    // layer does care, because the flow that crosses the nose crosses the
    // opening first and arrives on the upper surface as a reattaching shear
    // layer rather than as a fresh laminar one.
    //
    // Where the mouth sits is not published for this wing. Openings on wings
    // of this class sit just below the leading edge, at the stagnation point
    // at trim - which is where the pressure solver independently measures it,
    // 9.7 degrees below the chord line. Only the POSITION appears here: the
    // opening's height would set how much momentum the shear layer off its lip
    // carries, and that is the piece deliberately left out - see
    // SectionViscousSolver.cpp and PHYSICS_TODO item 12.
    double inletChordFraction = 0.015;
    // Panels around the contour. Even, so the leading edge lands on a node.
    std::size_t panelCount = 200;

    bool operator==(const SectionProfileSpec&) const = default;
};

struct SectionPoint
{
    double x = 0.0;
    double z = 0.0;
};

// A panelled section: `nodes` runs from the trailing edge forward along the
// LOWER surface, round the nose, and aft along the UPPER surface back to the
// trailing edge, with the first and last node coincident. That ordering is
// what the panel solver's Kutta condition assumes.
struct SectionProfile
{
    std::vector<SectionPoint> nodes;
    // The cell opening, carried through because the boundary layer needs it
    // even though the panelled contour is closed.
    double inletChordFraction = 0.0;
    // Camber line ordinate and slope, sampled on the same chord stations the
    // caller asked for. Kept because the moment reference and the brake bend
    // are both defined on it.
    double trailingEdgeDropFraction = 0.0;

    std::size_t PanelCount() const
    {
        return nodes.empty() ? 0 : nodes.size() - 1;
    }
};

// Builds the contour at a given brake setting, 0 to 1.
SectionProfile BuildSectionProfile(const SectionProfileSpec& spec, double brake);

// One surface point, on the unit chord: x runs aft from the leading edge and
// z is up. This is the same shape `BuildSectionProfile` panels, and it is what
// the rib stations are drawn from, so the section the polars are solved on and
// the section the canopy encloses cannot drift apart.
SectionPoint SectionSurfacePoint(
    const SectionProfileSpec& spec, double brake, double chordFraction,
    bool upper);

// The camber line alone, for callers that want the mean line rather than the
// surface: ordinate at a chord fraction, including the brake bend.
double SectionCamberOrdinate(
    const SectionProfileSpec& spec, double brake, double chordFraction);
}
