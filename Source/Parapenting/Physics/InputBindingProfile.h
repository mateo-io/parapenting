#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace Parapenting::Physics
{
enum class FlightBindingAction : std::size_t
{
    WeightShiftLeft,
    WeightShiftRight,
    LeftBrake,
    RightBrake,
    BothBrakesMore,
    BrakesRelease,
    Count
};

constexpr std::size_t FlightBindingActionCount =
    static_cast<std::size_t>(FlightBindingAction::Count);

enum class RebindResult
{
    Changed,
    Swapped,
    RejectedInvalid,
    RejectedProtected
};

struct InputBindingProfile
{
    std::array<std::string, FlightBindingActionCount> keyNames;

    static InputBindingProfile Standard();
    static InputBindingProfile Compact();
    static InputBindingProfile RightHand();

    RebindResult Rebind(FlightBindingAction action,
                        const std::string& keyName);
    const std::string& Key(FlightBindingAction action) const;
};

const char* FlightBindingActionDisplayName(FlightBindingAction action);
bool IsProtectedBindingKey(const std::string& keyName);
}
