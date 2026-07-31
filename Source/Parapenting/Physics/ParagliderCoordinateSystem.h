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
// UNRESOLVED: the terrain frame disagrees with the flight frame
// ---------------------------------------------------------------------------
// RouteFrame, the heightfield generator, RouteCatalogue and the content
// placement in ParapentingGameMode all define +Y as route-LEFT. The flight
// frame above has +Y as right. Nothing converts between them, so the surveyed
// landscape is mirrored about the route axis relative to the flight.
//
// Verified rather than inferred: Lake Thun sits at 557.7 m in the heightfield
// at local y = -2500, and the lake mesh is placed at y = -1450 with the
// comment "route-right/west of the southbound Amisbuehl line". Under the
// terrain's own convention that is correct - the real lake is west, which is
// route-right of a southbound track. But the renderer treats -Y as the
// pilot's left, so Lake Thun appears on the wrong side, and pulling the left
// brake tracks route-right across the ground.
//
// It is invisible in normal play because the mirroring is consistent: pull
// left, bank left, see the world turn left. Only the relationship to real
// geography is wrong - which is exactly what ridge lift, lee rotor, wind
// bearings and landing circuits depend on.
//
// This blocks the Level 0 exit gate ("left input, left attachment, left line,
// left canopy half, and left world trajectory agree"). Resolve before any
// solver level is built on top of it.

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
