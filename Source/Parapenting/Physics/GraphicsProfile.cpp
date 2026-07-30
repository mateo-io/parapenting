#include "GraphicsProfile.h"

#include <algorithm>

namespace Parapenting::Physics
{
namespace
{
constexpr std::array<GraphicsProfile, GraphicsProfileCount> Profiles{{
    {GraphicsProfileId::Low, "LOW", 0, 72.0},
    {GraphicsProfileId::Medium, "MEDIUM", 1, 85.0},
    {GraphicsProfileId::High, "HIGH", 2, 100.0},
    {GraphicsProfileId::Epic, "EPIC", 3, 100.0}
}};
}

const std::array<GraphicsProfile, GraphicsProfileCount>& GetGraphicsProfiles()
{
    return Profiles;
}

const GraphicsProfile& GetGraphicsProfile(GraphicsProfileId id)
{
    const auto index = std::clamp(
        static_cast<std::size_t>(id),
        std::size_t{0}, GraphicsProfileCount - 1);
    return Profiles[index];
}

} // namespace Parapenting::Physics
