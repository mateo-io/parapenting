#pragma once

#include "ParagliderDynamics.h"

#include <cstdint>

namespace Parapenting::Physics
{
// Deterministic hash of solver state, for replay comparison.
//
// Level 0 requires that the same inputs reproduce the same state regardless of
// render rate. Comparing whole states field by field is unwieldy and gets
// stale as FlightState grows, so runs are compared by hash and only
// investigated in detail when the hashes differ.
//
// Properties that matter:
//   * bit-exact. Hashing the IEEE-754 representation, not a rounded decimal,
//     so a 1-ULP divergence is caught rather than smoothed over.
//   * -0.0 and +0.0 hash alike, and any NaN hashes alike. Without this a run
//     that produced -0.0 where another produced +0.0 would look divergent
//     while being numerically identical.
//   * order-dependent, so a field swap changes the hash.
//
// This is FNV-1a over the state's doubles. It is not cryptographic and does
// not need to be; it needs to be stable across builds and platforms, which a
// fixed integer mix is and a floating-point reduction is not.
class SolverStateHash
{
public:
    static constexpr std::uint64_t OffsetBasis = 1469598103934665603ull;
    static constexpr std::uint64_t Prime = 1099511628211ull;

    SolverStateHash() = default;

    void MixBits(std::uint64_t bits);
    void Mix(double value);
    void Mix(const Vec3& value);
    void Mix(const Quaternion& value);

    std::uint64_t Value() const { return Accumulator; }

private:
    std::uint64_t Accumulator = OffsetBasis;
};

// Hash of everything in FlightState that the solver integrates. Any field
// added to FlightState should be added here too; HashFlightState is checked
// against the struct's size so a silently unhashed field is caught.
std::uint64_t HashFlightState(const FlightState& state);
}
