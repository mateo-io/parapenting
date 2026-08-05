#include "PilotSkeletonAim.h"

#include <cmath>

namespace Parapenting::Physics
{
namespace
{
// Any unit vector perpendicular to the argument. Picking the smallest
// component to cross against keeps the result well conditioned: crossing
// against an axis the input nearly lies along would give a near-zero vector
// and amplify its rounding into the chosen axis.
Vec3 AnyPerpendicular(const Vec3& value)
{
    const double ax = std::fabs(value.x);
    const double ay = std::fabs(value.y);
    const double az = std::fabs(value.z);
    const Vec3 axis = (ax <= ay && ax <= az) ? Vec3{1.0, 0.0, 0.0}
        : (ay <= az ? Vec3{0.0, 1.0, 0.0} : Vec3{0.0, 0.0, 1.0});
    return Normalized(Cross(value, axis));
}
}

Quaternion AimRotation(const Vec3& restDirection, const Vec3& targetDirection)
{
    const double restLength = Length(restDirection);
    const double targetLength = Length(targetDirection);
    if (restLength < 1e-12 || targetLength < 1e-12) return {};

    const Vec3 from = restDirection * (1.0 / restLength);
    const Vec3 to = targetDirection * (1.0 / targetLength);
    const double dot = Dot(from, to);

    // Half turn: no shortest rotation exists, so choose an axis rather than
    // let the cross product's rounding choose one.
    if (dot < -1.0 + 1e-12)
    {
        const Vec3 axis = AnyPerpendicular(from);
        return Quaternion{0.0, axis.x, axis.y, axis.z}.Normalized();
    }

    // Half-angle form: w = 1 + cos, xyz = cross. Building the quaternion this
    // way avoids an acos and stays accurate for the small rotations that
    // dominate here, where the bone barely moves between frames.
    const Vec3 axis = Cross(from, to);
    return Quaternion{1.0 + dot, axis.x, axis.y, axis.z}.Normalized();
}

Quaternion AimRotationWithRoll(const Vec3& restDirection, const Vec3& restUp,
    const Vec3& targetDirection, const Vec3& targetUp)
{
    const Quaternion aim = AimRotation(restDirection, targetDirection);

    const double targetLength = Length(targetDirection);
    if (targetLength < 1e-12) return aim;
    const Vec3 axis = targetDirection * (1.0 / targetLength);

    // Both up vectors, flattened into the plane the aim axis is normal to.
    // What is left is pure roll about the bone's own length.
    const Vec3 rotatedUp = aim.Rotate(restUp);
    const Vec3 flatFrom = rotatedUp - axis * Dot(rotatedUp, axis);
    const Vec3 flatTo = targetUp - axis * Dot(targetUp, axis);
    if (Length(flatFrom) < 1e-9 || Length(flatTo) < 1e-9) return aim;

    // Roll is applied after the aim, so it composes on the left.
    return (AimRotation(flatFrom, flatTo) * aim).Normalized();
}
}
