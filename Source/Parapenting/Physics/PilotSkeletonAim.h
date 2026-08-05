#pragma once

#include "ParagliderDynamics.h"

namespace Parapenting::Physics
{
// Aiming a bone at the next joint down the chain.
//
// The rig publishes joint positions. A skinned mesh needs joint rotations too:
// moving a bone's origin without turning it drags the skin along the segment
// but never rotates it, so the mesh shears at the shoulder and the wrist and
// the limb reads as bent tubing rather than an arm. These functions turn the
// positions the rig already solves into the rotations the skin needs.

// The shortest rotation taking restDirection onto targetDirection.
//
// Antiparallel inputs have no shortest rotation - every half turn about an axis
// perpendicular to the pair is equally valid - so one perpendicular axis is
// chosen deterministically rather than left to whatever the cross product's
// rounding produces. Zero-length inputs return identity: a bone with no length
// and a joint that has collapsed onto its parent both have no direction to
// point, and inventing one would spin the limb.
Quaternion AimRotation(const Vec3& restDirection, const Vec3& targetDirection);

// The same aim, but with the roll about the aim axis pinned by a reference
// direction. Aiming alone leaves a limb free to spin about its own length,
// which is exactly the twist a wrist needs and a forearm must not have.
//
// restUp and targetUp need not be perpendicular to their directions; the
// component along the aim axis is removed. When an up vector is degenerate -
// zero length, or parallel to the aim - the result falls back to the plain
// aim, because there is no roll information to apply.
Quaternion AimRotationWithRoll(const Vec3& restDirection, const Vec3& restUp,
    const Vec3& targetDirection, const Vec3& targetUp);
}
