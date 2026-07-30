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
}
