#include "InputBindingProfile.h"

#include <algorithm>

namespace Parapenting::Physics
{
namespace
{
constexpr std::size_t Index(FlightBindingAction action)
{
    return static_cast<std::size_t>(action);
}
}

InputBindingProfile InputBindingProfile::Standard()
{
    return {{{"A", "D", "Left", "Right", "Down", "Up"}}};
}

InputBindingProfile InputBindingProfile::Compact()
{
    // Avoid every non-flight action in DefaultInput.ini. Earlier Q/E/A/D/S/W
    // presets also changed wing and speedbar state.
    return {{{"E", "N", "A", "D", "Down", "Up"}}};
}

InputBindingProfile InputBindingProfile::RightHand()
{
    return {{{"N", "Slash", "Home", "End", "PageDown", "PageUp"}}};
}

RebindResult InputBindingProfile::Rebind(
    FlightBindingAction action, const std::string& keyName)
{
    if (Index(action) >= FlightBindingActionCount || keyName.empty())
        return RebindResult::RejectedInvalid;
    if (IsProtectedBindingKey(keyName))
        return RebindResult::RejectedProtected;
    const std::size_t target = Index(action);
    const auto conflict = std::find(
        keyNames.begin(), keyNames.end(), keyName);
    if (conflict != keyNames.end())
    {
        const std::size_t other = static_cast<std::size_t>(
            std::distance(keyNames.begin(), conflict));
        if (other == target) return RebindResult::Changed;
        std::swap(keyNames[target], keyNames[other]);
        return RebindResult::Swapped;
    }
    keyNames[target] = keyName;
    return RebindResult::Changed;
}

const std::string& InputBindingProfile::Key(
    FlightBindingAction action) const
{
    return keyNames[Index(action) % FlightBindingActionCount];
}

const char* FlightBindingActionDisplayName(FlightBindingAction action)
{
    switch (action)
    {
        case FlightBindingAction::WeightShiftLeft: return "WEIGHT SHIFT LEFT";
        case FlightBindingAction::WeightShiftRight: return "WEIGHT SHIFT RIGHT";
        case FlightBindingAction::LeftBrake: return "LEFT BRAKE";
        case FlightBindingAction::RightBrake: return "RIGHT BRAKE";
        case FlightBindingAction::BothBrakesMore: return "BOTH BRAKES MORE";
        case FlightBindingAction::BrakesRelease: return "BRAKES RELEASE";
        default: return "UNKNOWN";
    }
}

bool IsProtectedBindingKey(const std::string& keyName)
{
    static constexpr std::array<const char*, 10> Protected{{
        "Escape", "Tab", "F6", "F7", "F8",
        "F9", "F10", "F11", "F12", "R"
    }};
    return std::any_of(Protected.begin(), Protected.end(),
        [&keyName](const char* value) { return keyName == value; });
}
}
