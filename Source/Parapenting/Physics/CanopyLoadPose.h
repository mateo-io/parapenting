#pragma once

namespace Parapenting::Physics
{
// Engine-independent presentation contract for unresolved flexible-canopy
// deformation. Units are explicit so Unreal and future renderers share the
// same response without feeding presentation back into flight dynamics.
struct CanopyLoadPose
{
    double spanScale = 1.0;
    double chordScale = 1.0;
    double camberScale = 1.0;
    double extraArchDropCm = 0.0;
    double lineStretchCm = 0.0;
    double rippleAmplitudeCm = 0.0;
};

CanopyLoadPose EvaluateCanopyLoadPose(
    double highLoadDeformation, double loadFactor);

// Where the canopy sits when it has swung fore or aft on its lines, relative
// to hanging straight above the pilot. Metres, in the pilot's frame: forward
// is +X and up is +Z, the same axes the flight model uses.
//
// This exists so the renderer does not have to infer the direction from a
// rotation convention. Swinging a canopy by rotating it about a pivot 7.3 m
// below itself does produce the right arc, but only if the sign of the
// rotation is right, and that sign depends on the engine's rotator handedness
// rather than on any physics - it cannot be checked by the physics suite, and
// this project has had two convention errors that survived review. An explicit
// displacement can be checked, and is, in the geometry suite.
struct CanopySwingOffset
{
    double forwardM = 0.0;
    double riseM = 0.0;
};

// `swingRad` is the canopy's rotation on its lines, positive AFT - the same
// sense and the same state as FlightState::canopyRelativePitchRad, so a wing
// that has been pushed back behind the pilot under brake has a positive value
// and a negative `forwardM`.
CanopySwingOffset EvaluateCanopySwingOffset(
    double swingRad, double suspensionLengthM);
}
