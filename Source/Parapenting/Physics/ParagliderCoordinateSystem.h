#pragma once

// Level 0 of the geometry-driven master plan: define the conventions once, in
// one place, so every later solver level can be checked against them instead
// of against scattered comments.
//
// Nothing here is a new decision. These are the conventions the existing
// dynamics, terrain and route code already follow; writing them down makes
// them testable and gives later levels something to violate loudly.

namespace Parapenting::Physics
{
// ---------------------------------------------------------------------------
// World frame
// ---------------------------------------------------------------------------
// Right-handed, metres, route-aligned. See RouteFrame.h for the geographic
// anchoring; this describes only the axis meaning.
//
//   +X  along the active route, launch -> landing
//   +Y  route-left
//   +Z  up, zero at the route landing field
//
// Note this is NOT the Unreal world frame. The engine layer scales by 100
// (metres -> centimetres) at the boundary and nowhere else.
namespace WorldAxes
{
constexpr double MetresToUnrealUnits = 100.0;
constexpr double UnrealUnitsToMetres = 0.01;
}

// ---------------------------------------------------------------------------
// Body frame
// ---------------------------------------------------------------------------
// Right-handed, fixed to the canopy.
//
//   +X  forward, along the chord toward the leading edge
//   +Y  right, toward the right wing tip
//   +Z  up, toward the upper surface
//
// Attitude is the body-to-world rotation: FlightState::attitude.Rotate(v)
// takes a body vector to world.

// ---------------------------------------------------------------------------
// Rotations
// ---------------------------------------------------------------------------
// MEASURED, not assumed. This section previously stated all three the other
// way round, and the cost of that was real: the weight-shift model banked the
// wing one way and flew it the other for as long as it existed, because each
// author who found a sign wrong fixed it downstream against this description
// instead of against the wing.
//
// The flight frame is forward/right/up, which is left-handed, while the
// quaternion algebra in ParagliderDynamics is right-handed. So a positive
// rotation about each body axis is:
//
//   roll  (+X)  right wing UP
//   pitch (+Y)  nose DOWN
//   yaw   (+Z)  nose RIGHT
//
// Verified by integrating the attitude directly: a body rate of +0.5 rad/s
// about +Z for one second puts the nose at y = +0.479, and a +0.30 rad
// rotation about +X puts the span vector at z = +0.30, right tip high. A wing
// left in that attitude then curves LEFT, which is the correct physics for a
// right-tip-high bank and confirms the lift coupling.
//
// Because of this, code that needs a physical sign should read it off the
// rotated basis vectors rather than from an Euler-angle formula:
//
//   incidence, nose-up positive :  asin(attitude.Rotate({1,0,0}).z)
//   bank, right tip down positive: asin(-attitude.Rotate({0,1,0}).z)
//
// FlightState::angularVelocityBodyRadps carries these in body axes, rad/s.
enum class BodyAxis
{
    Roll = 0,   // +X
    Pitch = 1,  // +Y
    Yaw = 2,    // +Z
};

// ---------------------------------------------------------------------------
// Left and right  -- CONTESTED, see UNRESOLVED below
// ---------------------------------------------------------------------------
// Measured behaviour of the shipped build, not an aspiration:
//
//   pilot left  (weightShift -1, leftBrake)  turns toward -Y
//   pilot right (weightShift +1, rightBrake) turns toward +Y
//
// So the flight frame is forward/right/up: +Y is the RIGHT wing. That is
// left-handed, and it matches Unreal, which the engine boundary relies on -
// ParagliderPawn copies position and attitude straight through with no
// handedness conversion.
namespace Span
{
// Signed span fraction: -1 at the left tip, 0 at centre, +1 at the right tip.
constexpr double LeftTip = -1.0;
constexpr double Centre = 0.0;
constexpr double RightTip = 1.0;

constexpr bool IsLeft(double spanFraction) { return spanFraction < 0.0; }
constexpr bool IsRight(double spanFraction) { return spanFraction > 0.0; }
}

// ---------------------------------------------------------------------------
// RESOLVED: the terrain frame agrees with the flight frame
// ---------------------------------------------------------------------------
// This block recorded the project's oldest defect: RouteFrame, the heightfield
// generator, RouteCatalogue and the content placement in ParapentingGameMode
// all defined +Y as route-LEFT while the flight frame has +Y as right, with
// nothing converting, so the surveyed landscape was mirrored about the route
// axis relative to the flight.
//
// Fixed by flipping the terrain side to match the flight frame - the flight
// frame is the one Unreal's handedness and the whole dynamics stack depend on,
// so it was cheaper and safer to move the terrain to it. The heightfield was
// regenerated in the same change rather than reinterpreted, so the data and
// the transform have never disagreed.
//
// Gated end to end by TerrainSurveyTests' handedness section, which asserts
// both halves in one place and would fail if either drifted:
//
//   * left weight shift and left brake each turn the wing toward -Y, and
//     mirror their right-hand counterparts to 0.05 rad over 40 s;
//   * Lake Thun - which is really west, and therefore route-right of the
//     southbound Amisbuehl line - reads 557.7 m MSL at local y = +2500, on the
//     wing's right, within 6 m of its true surface.
//
// Note when re-measuring anything sided: local z is metres relative to the
// Lehn landing field at 565 m, so a "fixed altitude" sample can sit inside a
// hillside on one flank and in open air on the other. An early measurement of
// lee rotor asymmetry was exactly that mistake. Sample at a height above
// ground when comparing two places.

// ---------------------------------------------------------------------------
// Chord
// ---------------------------------------------------------------------------
// Chord fraction runs 0 at the leading edge to 1 at the trailing edge.
namespace Chord
{
constexpr double LeadingEdge = 0.0;
constexpr double TrailingEdge = 1.0;
}

// ---------------------------------------------------------------------------
// Control sign conventions
// ---------------------------------------------------------------------------
// ControlInput uses these; they are stated here so the "no direct
// control-to-moment" rule can be checked against a fixed reference.
//
//   leftBrake, rightBrake  0 = hands up, 1 = full travel
//   weightShift            -1 = fully left, +1 = fully right
//   accelerator            0 = trim, 1 = full bar
//
// weightShift is deliberately opposite in sign to +Y: it names the direction
// the pilot's mass moves, which is toward the RIGHT carabiner at +1. Level 3
// replaces it with an actual payload CG offset and this sign disappears.
namespace Controls
{
constexpr double WeightShiftLeft = -1.0;
constexpr double WeightShiftRight = 1.0;
constexpr double HandsUp = 0.0;
constexpr double FullBrake = 1.0;
constexpr double Trim = 0.0;
constexpr double FullBar = 1.0;
}

// ---------------------------------------------------------------------------
// Units
// ---------------------------------------------------------------------------
// SI throughout the physics layer, without exception:
//   length m, mass kg, time s, angle rad, force N, pressure Pa, energy J.
// Any non-SI value belongs at the presentation boundary, never in a solver.
// Fields carry the unit in their name (…M, …Kg, …Radps, …N, …Pa, …J) so a
// mismatch is visible at the call site.
}
