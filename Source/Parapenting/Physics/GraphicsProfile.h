#pragma once

#include <array>
#include <cstddef>

namespace Parapenting::Physics
{

enum class GraphicsProfileId
{
    Low = 0,
    Medium,
    High,
    Epic
};

struct GraphicsProfile
{
    GraphicsProfileId id;
    const char* displayName;
    int qualityLevel;
    double resolutionScale;
};

constexpr std::size_t GraphicsProfileCount = 4;
const std::array<GraphicsProfile, GraphicsProfileCount>& GetGraphicsProfiles();
const GraphicsProfile& GetGraphicsProfile(GraphicsProfileId id);

} // namespace Parapenting::Physics
