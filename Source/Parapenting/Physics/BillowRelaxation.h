#pragma once

namespace Parapenting::Physics
{
// Level 1: billow emerges, it is not applied.
//
// Until now the inflated cross-section was produced by *applying* a bulge: a
// circular arc was drawn between the ribs with whatever extra length the
// pattern specified. That gets the right shape for the wrong reason - the
// shape was authored, so changing a seam allowance changed a drawing rather
// than changing an equilibrium.
//
// This solves it instead. A pressurised membrane strip with fixed edges, no
// bending stiffness and uniform internal pressure takes a circular arc,
// because constant tension against constant pressure means constant curvature.
// What is *not* fixed is its length: hoop tension stretches the cloth, and the
// tension depends on the radius, which depends on the length. So the arc has
// to be solved self-consistently:
//
//   Laplace         T = p * R                     (N/m)
//   Fabric          strain = T / membraneStiffness
//   Arc length      s = cutWidth * (1 + strain)
//   Geometry        s = 2 * R * theta
//                   c = 2 * R * sin(theta)        c is the rib spacing
//
// Eliminating R gives one equation in theta, solved by bisection.
//
// Ovalization falls out of this rather than being asserted: the fabric between
// two ribs is longer than the gap it spans, so the inflated cell is narrower
// than the panel it was cut from. Design practice quotes 5-6%, and the model
// reproduces it from the cut width alone.

// Chord-cut billow profile: the seam allowance sewn in at a given chord
// position, given the peak. It runs out to zero at the leading and trailing
// edges so those seams are cut flat, which is the whole point of chord-cut
// patterning. This is a property of the pattern, so it lives with the pattern.
double ChordCutBillowAt(double chordFraction, double peakBillow);

struct FabricProperties
{
    // Membrane stiffness, force per unit width per unit strain (N/m). This is
    // Young's modulus times thickness for the cloth; for the coated ripstop
    // used in paraglider skins it is of order tens of kN/m.
    double membraneStiffnessNPerM = 35000.0;
    // Below this the cloth is treated as unable to carry hoop tension, so the
    // cell cannot hold its section. Not a physical constant - a numerical
    // floor that keeps the solve finite as pressure goes to zero.
    double minimumTensionNPerM = 0.05;
};

struct CellInflation
{
    // Solved.
    double halfAngleRad = 0.0;
    double radiusM = 0.0;
    // Bulge height above the straight rib-to-rib line.
    double sagittaM = 0.0;
    double hoopTensionNPerM = 0.0;
    double fabricStrain = 0.0;
    // Arc length the fabric actually has after stretching.
    double developedWidthM = 0.0;
    // (cut width - rib spacing) / cut width. This is the ovalization the
    // sailmaking literature quotes.
    double ovalizationFraction = 0.0;
    // False when the pressure cannot hold the section: the cell is slack.
    bool holdsSection = false;
};

// ribSpacingM   distance between the two rib attachment lines, from geometry
// cutWidthM     width the flat panel was cut to, always >= ribSpacingM
// pressurePa    internal pressure above ambient
CellInflation RelaxCell(
    double ribSpacingM, double cutWidthM, double pressurePa,
    const FabricProperties& fabric = {});

// Convenience: the cut width that a given billow allowance implies.
double CutWidthForBillow(double ribSpacingM, double billowFraction);
}
