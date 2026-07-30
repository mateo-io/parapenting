#include "AccessibilityProfile.h"

namespace Parapenting::Physics
{
namespace
{
constexpr std::array<AccessibilityProfile, AccessibilityProfileCount> Profiles{{
    {AccessibilityProfileId::FullMotion, "FULL MOTION", 1.0, 1.0, 1.0},
    {AccessibilityProfileId::Comfort, "COMFORT", 0.42, 0.32, 0.72},
    {AccessibilityProfileId::MinimalMotion, "MINIMAL MOTION", 0.06, 0.0, 0.48}
}};
}

const std::array<AccessibilityProfile, AccessibilityProfileCount>&
GetAccessibilityProfiles()
{
    return Profiles;
}

const AccessibilityProfile& GetAccessibilityProfile(
    AccessibilityProfileId id)
{
    for (const auto& profile : Profiles)
        if (profile.id == id) return profile;
    return Profiles[0];
}
}
