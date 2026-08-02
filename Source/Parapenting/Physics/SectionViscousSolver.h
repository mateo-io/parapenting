#pragma once

#include <cstddef>
#include <vector>

#include "SectionProfile.h"

namespace Parapenting::Physics
{
// A viscous section solve on the real contour: potential flow by panels, the
// boundary layer by integral methods, and the two coupled through the
// separation point.
//
// Why this exists. Every flight number in the geometry-driven stack rests on
// 2-D section data, and until now that data was thin-airfoil theory with a
// stated stall angle. A stated stall angle has two consequences that a pilot
// feels. Maximum lift cannot change with brake, because brake only slid the
// whole curve along the incidence axis while the stall stayed a fixed number
// of degrees from the zero-lift angle - so pulling brake bought no lift at
// all, and 40% brake, an ordinary input, walked the wing off the top of a
// curve that never rose. And the pitching moment was a constant, so the
// section had no aerodynamic-centre movement and the pitch loop gain had
// nothing in it that could stabilise the wing at speed.
//
// Both come from the same missing thing: nothing in thin-airfoil theory knows
// the shape of the nose, and the nose is what decides when the flow lets go.
//
// The method here is the classical one, and it predates XFOIL by decades:
//
//   * Hess-Smith panels - constant source on each panel plus one vortex
//     strength shared by all of them, with the Kutta condition closing the
//     system. Exact potential flow on the given contour, no thin-airfoil
//     approximation anywhere.
//   * Thwaites' laminar momentum-integral method from the stagnation point.
//   * Michel's transition criterion, with a laminar separation bubble
//     treated as immediate transition - which at this wing's Reynolds
//     numbers, 0.5 to 3 million, is what actually happens.
//   * Head's entrainment method with the Ludwieg-Tillmann skin friction for
//     the turbulent run, separating at a shape factor of 2.4.
//   * Squire-Young at the trailing edge for profile drag.
//   * A Kirchhoff constant-pressure dead-air region aft of separation, which
//     is what makes the coupling a fixed point rather than a march: the
//     separated wake unloads the section, the unloaded section has a weaker
//     suction peak, and the separation point moves back until the two agree.
//
// Maximum lift is then not a number in a struct. It is where the growth of
// circulation with incidence stops outrunning the loss of loaded chord, and
// it moves when the geometry moves - which is exactly what brake does.
//
// What this is not: it is not XFOIL. There is no e^N amplification model, no
// full inverse boundary-layer formulation, and no wake. It will be optimistic
// about thin-airfoil-type leading-edge stall and it has nothing to say about
// compressibility, neither of which this wing does. The registry records it
// as computed from geometry, not measured.

struct SectionAerodynamics
{
    // Wind axes, per unit span, referenced to the chord.
    double liftCoefficient = 0.0;
    double dragCoefficient = 0.0;
    // Quarter chord, nose-up positive.
    double momentCoefficient = 0.0;

    // The same section with the boundary layer removed: potential flow on
    // this contour. This is the attached branch the solver above blends
    // against, and it is what the lift curve would be if the flow never let
    // go.
    double attachedLiftCoefficient = 0.0;
    double attachedMomentCoefficient = 0.0;

    // Fraction of chord over which the flow has separated, measured on
    // whichever surface has let go furthest. 0 is attached to the trailing
    // edge.
    double separatedChordFraction = 0.0;
    // The chord fraction each surface stays attached over, kept separately so
    // a sweep can hand them back in as the starting point for the next
    // incidence. That is what keeps the solve on the branch it is already on
    // rather than falling into the fully separated one, which is also a fixed
    // point of the same map and is the deep stall.
    double lowerAttachedFraction = 1.0;
    double upperAttachedFraction = 1.0;
    // Where transition sits on each surface, as a chord fraction. Reported
    // because it is the one thing here a wind tunnel could check cheaply.
    double upperTransitionChordFraction = 1.0;
    double lowerTransitionChordFraction = 1.0;

    bool converged = false;
};

class SectionViscousSolver
{
public:
    // Builds and factorises the panel system for this contour. The
    // factorisation does not depend on incidence, and the whole system is
    // linear in (cos alpha, sin alpha), so an incidence sweep costs two
    // solves in total rather than two per angle.
    SectionViscousSolver(const SectionProfile& profile, double reynoldsNumber);

    // The separation state the iteration starts from. Left at 1 it starts
    // attached; handed the previous incidence's answer it continues along the
    // branch the section is on.
    SectionAerodynamics Solve(
        double alphaRad,
        double initialLowerAttached = 1.0,
        double initialUpperAttached = 1.0) const;

    bool Valid() const { return PanelCount > 0; }
    std::size_t Panels() const { return PanelCount; }

    // Surface pressure coefficient at each panel for a given incidence, with
    // no boundary layer. Exposed for validation against published pressure
    // distributions.
    std::vector<double> InviscidPressure(double alphaRad) const;

private:
    struct Panel
    {
        double midX = 0.0;
        double midZ = 0.0;
        double tangentX = 0.0;
        double tangentZ = 0.0;
        double normalX = 0.0;
        double normalZ = 0.0;
        double length = 0.0;
    };

    std::vector<Panel> Geometry;
    std::size_t PanelCount = 0;
    double Reynolds = 1.0e6;
    double Chord = 1.0;
    // Tangential surface velocity for unit freestream along x and along z.
    // Any incidence is a combination of the two.
    std::vector<double> TangentialAlongX;
    std::vector<double> TangentialAlongZ;
    // The circulation mode: unit vortex strength with no freestream and the
    // no-penetration condition still satisfied. Adding a multiple of it is
    // how the separated solve carries less circulation than the Kutta
    // condition would give it.
    std::vector<double> TangentialCirculation;
    double VortexAlongX = 0.0;
    double VortexAlongZ = 0.0;
    // Where the cell opening sits, as a chord fraction. The surface the flow
    // reaches by crossing it has no laminar run.
    double InletChordFraction = 0.0;

    std::vector<double> TangentialVelocity(double alphaRad) const;
    SectionAerodynamics IntegrateSurface(
        const std::vector<double>& pressure, double alphaRad) const;
};
}
