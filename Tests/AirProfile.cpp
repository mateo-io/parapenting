// Performance plan L4: what the air costs.
//
// `SOLVER_PROFILE.md` measured the flight solver and `FRAME_PROFILE.md` the
// frame. Between them they missed one stage entirely: the atmosphere sample.
// `Tests/SolverProfile.cpp` constructs a `CoupledAtmosphere` and never calls
// it, and the frame profile sees the sample only inside the pawn's fixed-step
// spine, which L3 measured at 0.013 ms for everything it contains.
//
// So this stage has never had a number, and it is the one the wind and lift
// work grows. A budget nobody has measured is not headroom, it is a hope.
//
// Nothing here asserts, for the same reason the solver profile does not: wall
// clock is a property of the machine, and gating on it would turn a busy
// laptop into a red suite. It prints, and the numbers go into
// docs/AIR_PROFILE.md with the machine they came off.
#include "AtmosphereModel.h"
#include "TerrainModel.h"

#include <chrono>
#include <cstdio>
#include <vector>

using namespace Parapenting::Physics;

namespace
{
constexpr double StepSeconds = 1.0 / 120.0;
constexpr double StepBudgetUs = 1.0e6 * StepSeconds;   // 8333 us at 120 Hz

// A position is walked between iterations rather than held, so nothing is
// measured from a branch predictor that has seen the same coordinates a
// million times. The offsets are tiny compared with the terrain features.
template <typename Fn>
double NanosecondsPer(int iterations, Fn&& fn)
{
    const auto start = std::chrono::steady_clock::now();
    double sink = 0.0;
    for (int i = 0; i < iterations; ++i)
        sink += fn(static_cast<double>(i) * 0.001);
    const auto end = std::chrono::steady_clock::now();
    // Consume the accumulator so the loop cannot be optimised away.
    if (sink == 1.2345e-300) std::printf(" ");
    return std::chrono::duration<double, std::nano>(end - start).count()
        / static_cast<double>(iterations);
}

int ActiveVolumes(const AtmosphereModel& air)
{
    int active = 0;
    for (const WeatherVolume& volume : air.GetVolumes())
        if (volume.radiusM > 0.0 && volume.heightM > 0.0) ++active;
    return active;
}
}

int main()
{
    constexpr int Iterations = 200000;
    // Somewhere on the Interlaken flight frame, in the air, which is where the
    // aircraft spends its time.
    const Vec3 Cruise{1200.0, -800.0, 1650.0};

    std::printf("\nAtmosphere sample cost\n");
    std::printf("  %d iterations per row, %.0f us step budget at 120 Hz\n\n",
                Iterations, StepBudgetUs);

    // -- by weather preset ------------------------------------------------
    std::printf("%-18s %8s %12s %14s %12s\n", "preset", "volumes",
                "ns/sample", "us/step (x3)", "% of step");
    for (const WeatherPreset& preset : GetWeatherPresets())
    {
        AtmosphereModel air;
        air.SetPreset(preset.id);
        const double ns = NanosecondsPer(Iterations, [&](double drift)
        {
            return air.Sample({Cruise.x + drift, Cruise.y, Cruise.z}, 12.5)
                .windWorldMps.z;
        });
        // The pawn takes ONE SampleCanopy per fixed step, and SampleCanopy is
        // three Samples - centre and both wing tips.
        const double perStepUs = 3.0 * ns / 1000.0;
        std::printf("%-18s %8d %12.1f %14.3f %11.2f%%\n",
                    preset.displayName, ActiveVolumes(air), ns, perStepUs,
                    100.0 * perStepUs / StepBudgetUs);
    }

    // -- by where the aircraft is -----------------------------------------
    //
    // The sample reads the terrain, so its cost is a function of position and
    // not only of weather. A row that is cheap in the valley and expensive on
    // a ridge would price the route rather than the model.
    std::printf("\n%-26s %12s\n", "position", "ns/sample");
    AtmosphereModel air;
    air.SetPreset(WeatherPresetId::ThermalDay);
    struct Place { const char* name; Vec3 at; };
    for (const Place& place : {
        Place{"cruise, mid valley", Cruise},
        Place{"low, near the ground", {1200.0, -800.0, 620.0}},
        Place{"high, above thermal top", {1200.0, -800.0, 3200.0}},
        Place{"far off the terrain grid", {90000.0, 90000.0, 1650.0}}})
    {
        const Vec3 at = place.at;
        const double ns = NanosecondsPer(Iterations, [&](double drift)
        {
            return air.Sample({at.x + drift, at.y, at.z}, 12.5).turbulence;
        });
        std::printf("%-26s %12.1f\n", place.name, ns);
    }

    // -- what inside the sample costs --------------------------------------
    //
    // THE QUESTION THIS EXISTS TO ANSWER. If the sample is mostly terrain
    // queries, then richer WIND is nearly free and more SAMPLE POINTS is not -
    // and the wind work should be planned the opposite way round from a model
    // where the cost is the wind field itself.
    std::printf("\n%-30s %12s\n", "component", "ns/call");
    const double heightNs = NanosecondsPer(Iterations, [&](double drift)
        { return TerrainModel::HeightM(Cruise.x + drift, Cruise.y); });
    const double normalNs = NanosecondsPer(Iterations, [&](double drift)
        { return TerrainModel::Normal(Cruise.x + drift, Cruise.y).z; });
    const double ridgeNs = NanosecondsPer(Iterations, [&](double drift)
        { return TerrainModel::RidgeExposure(Cruise.x + drift, Cruise.y); });
    const double rotorNs = NanosecondsPer(Iterations, [&](double drift)
        { return TerrainModel::LeeRotorPotential(
              Cruise.x + drift, Cruise.y, {-1.5, 0.0, 0.0}); });
    std::printf("%-30s %12.1f\n", "TerrainModel::HeightM", heightNs);
    std::printf("%-30s %12.1f\n", "TerrainModel::Normal", normalNs);
    std::printf("%-30s %12.1f\n", "TerrainModel::RidgeExposure", ridgeNs);
    std::printf("%-30s %12.1f\n", "TerrainModel::LeeRotorPotential", rotorNs);

    const double sampleNs = NanosecondsPer(Iterations, [&](double drift)
        { return air.Sample({Cruise.x + drift, Cruise.y, Cruise.z}, 12.5)
              .windWorldMps.z; });
    // Sample reads the ground twice through two different helpers, the normal
    // once, ridge exposure once, and the lee rotor once in the rotor modes.
    const double terrainNs =
        2.0 * heightNs + normalNs + ridgeNs + rotorNs;
    std::printf("%-30s %12.1f  (%.0f%% of the sample)\n",
                "= terrain queries in one sample", terrainNs,
                100.0 * terrainNs / sampleNs);
    std::printf("%-30s %12.1f\n", "one whole Sample", sampleNs);

    // -- the budget ---------------------------------------------------------
    std::printf("\nPer fixed step at 120 Hz, against %.0f us:\n", StepBudgetUs);
    std::printf("  atmosphere (1 SampleCanopy = 3 Samples)   %8.3f us  %5.2f%%\n",
                3.0 * sampleNs / 1000.0,
                100.0 * (3.0 * sampleNs / 1000.0) / StepBudgetUs);
    std::printf("  flight model the game runs (measured in-engine, L3)"
                "  ~13 us   0.16%%\n");
    std::printf("  coupled solver, when it becomes the flight model"
                "     540 us   6.48%%\n");

    // -- what more air would cost ------------------------------------------
    //
    // The forward-looking rows, and the reason this profile is a plan level
    // rather than a curiosity. The FIRST version of this block priced "one
    // more piece of weather" by regressing sample cost against the number of
    // active volumes across presets, and reported 204 ns per volume. THAT
    // NUMBER WAS WRONG AND ITS OWN TABLE SAYS SO: the still-air preset carries
    // zero volumes at 63 ns, the evening-drainage preset carries zero at 821,
    // and the four-volume thermal day is CHEAPER than the two-volume valley
    // breeze. Volume count does not order the rows; weather MODE does, because
    // the mode decides which terrain queries run.
    //
    // So the honest pair of questions is: does more weather structure cost
    // anything (no), and what does cost - more SAMPLE POINTS (yes, linearly).
    std::printf("\nDoes more weather structure cost anything?\n");
    std::printf("%-22s %8s %12s\n", "preset", "volumes", "ns/sample");
    for (const WeatherPreset& preset : GetWeatherPresets())
    {
        AtmosphereModel priced;
        priced.SetPreset(preset.id);
        const double ns = NanosecondsPer(Iterations / 4, [&](double drift)
        {
            return priced.Sample({Cruise.x + drift, Cruise.y, Cruise.z}, 12.5)
                .windWorldMps.z;
        });
        std::printf("%-22s %8d %12.1f\n", preset.displayName,
                    ActiveVolumes(priced), ns);
    }
    std::printf("  Volumes do not order these rows. The sample is %.0f%% "
                "terrain query, and\n  which of those run is set by the "
                "weather MODE, not by how much weather there is.\n",
                100.0 * terrainNs / sampleNs);

    // AND WHAT DOES SCALE: sample points. The wing is sampled at three today -
    // centre and both tips. A per-section wind field for the 45-section VSM
    // would be 45 of them, and that is the axis the wind work actually moves
    // along.
    std::printf("\nWhat sample POINTS cost, at %.0f ns each:\n", sampleNs);
    std::printf("%-42s %10s %10s\n", "per step", "us/step", "% of step");
    for (const auto& row : {
        std::pair<const char*, int>{"3  - centre and tips, what ships today", 3},
        {"16 - one per spanwise strip", 16},
        {"45 - one per VSM section", 45},
        {"135 - one per VSM section, with tips", 135}})
    {
        const double us = row.second * sampleNs / 1000.0;
        std::printf("%-42s %10.2f %9.2f%%\n", row.first, us,
                    100.0 * us / StepBudgetUs);
    }
    std::printf("\n");
    return 0;
}
