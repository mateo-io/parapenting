#pragma once

#include <array>
#include <cstddef>

namespace Parapenting::Physics
{
enum class AccessibilityProfileId
{
    FullMotion,
    Comfort,
    MinimalMotion
};

struct AccessibilityProfile
{
    AccessibilityProfileId id;
    const char* displayName;
    double inertialCameraScale;
    double rotorBuffetScale;
    double hapticScale;
};

constexpr std::size_t AccessibilityProfileCount = 3;
const std::array<AccessibilityProfile, AccessibilityProfileCount>&
GetAccessibilityProfiles();
const AccessibilityProfile& GetAccessibilityProfile(
    AccessibilityProfileId id);
}
