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

// The rate the simulation steps at. The frame caps below are stated against
// it because it is what makes them derivable rather than chosen: a frame can
// carry no physics newer than one step, and the controls are sampled once per
// frame, so the step rate is the point where rendering harder stops buying
// anything the pilot can act on. See `PERFORMANCE_PLAN.md` Level 1.
constexpr double SimulationRateHz = 120.0;

struct GraphicsProfile
{
    GraphicsProfileId id;
    const char* displayName;
    int qualityLevel;
    double resolutionScale;
    // Frames per second the renderer is allowed to produce. 0 would mean
    // uncapped, which is what this project shipped with and is not offered
    // here: uncapped costs the whole difference between the cap and whatever
    // the GPU can manage, in power, for frames that carry no new simulated
    // state.
    //
    // MEASURED, NOT PREFERRED. `determinism_tests` samples the controls the
    // way the pawn does - once per frame, held across that frame's steps - and
    // compares the 30 s flight against sampling every step:
    //
    //   240 Hz  bit-identical      144 Hz  bit-identical
    //   120 Hz  0.0007 m            60 Hz  0.0776 m
    //    30 Hz  0.2292 m            20 Hz  0.3792 m
    //
    // So a cap at or above the step rate costs nothing a pilot could feel -
    // 0.7 mm over 30 s, four orders below the 8 m a step covers - and a cap
    // below it is a handling change. That is why the two upper tiers sit at
    // the step rate and the lower two go under it deliberately, on machines
    // where the alternative is not holding any rate at all.
    double frameRateCapHz;
    // Whether to also synchronise to the display. Vsync and the cap are not
    // alternatives: the cap bounds the work, vsync removes the tearing, and
    // whichever is lower governs. Off on the lowest tier, where a machine that
    // cannot hold 30 should not also be waiting for a scan-out.
    bool verticalSync;
};

constexpr std::size_t GraphicsProfileCount = 4;
const std::array<GraphicsProfile, GraphicsProfileCount>& GetGraphicsProfiles();
const GraphicsProfile& GetGraphicsProfile(GraphicsProfileId id);

} // namespace Parapenting::Physics
