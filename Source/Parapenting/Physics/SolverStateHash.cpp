#include "SolverStateHash.h"

#include <cmath>
#include <cstring>

namespace Parapenting::Physics
{
void SolverStateHash::MixBits(std::uint64_t bits)
{
    for (int byte = 0; byte < 8; ++byte)
    {
        Accumulator ^= (bits >> (byte * 8)) & 0xffull;
        Accumulator *= Prime;
    }
}

void SolverStateHash::Mix(double value)
{
    // Canonicalise before hashing. -0.0 == +0.0 numerically but has a
    // different bit pattern, and NaN has 2^52 representations; without this a
    // run could differ in hash while being numerically identical.
    if (value == 0.0) value = 0.0;
    if (std::isnan(value))
    {
        MixBits(0x7ff8000000000000ull);
        return;
    }
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    MixBits(bits);
}

void SolverStateHash::Mix(const Vec3& value)
{
    Mix(value.x);
    Mix(value.y);
    Mix(value.z);
}

void SolverStateHash::Mix(const Quaternion& value)
{
    Mix(value.w);
    Mix(value.x);
    Mix(value.y);
    Mix(value.z);
}

// FlightState is all doubles and fixed double arrays, so its size is exactly
// the count of values HashFlightState covers. If a field is added without
// being hashed, replays stop being able to detect divergence in it - silently.
// This turns that into a build failure.
constexpr int HashedValueCount = 74;
static_assert(sizeof(FlightState) == HashedValueCount * sizeof(double),
    "FlightState changed size: add the new field(s) to HashFlightState and "
    "update HashedValueCount.");

std::uint64_t HashFlightState(const FlightState& state)
{
    SolverStateHash hash;
    hash.Mix(state.positionWorldM);
    hash.Mix(state.velocityWorldMps);
    hash.Mix(state.attitude);
    hash.Mix(state.angularVelocityBodyRadps);
    hash.Mix(state.leftCollapse);
    hash.Mix(state.rightCollapse);
    hash.Mix(state.frontalCollapse);
    hash.Mix(state.canopyPressure);
    hash.Mix(state.harnessRollRad);
    hash.Mix(state.harnessRollRateRadps);
    hash.Mix(state.harnessPitchRad);
    hash.Mix(state.harnessPitchRateRadps);
    hash.Mix(state.previousLongitudinalAccelerationMps2);
    hash.Mix(state.previousLoadFactor);
    // Level 3 payload body.
    hash.Mix(state.payload.rollRad);
    hash.Mix(state.payload.rollRateRadps);
    hash.Mix(state.payload.pitchRad);
    hash.Mix(state.payload.pitchRateRadps);
    hash.Mix(state.previousSymmetricBrake);
    hash.Mix(state.previousLeftBrake);
    hash.Mix(state.previousRightBrake);
    hash.Mix(state.leftBrakeTravel);
    hash.Mix(state.rightBrakeTravel);
    hash.Mix(state.leftPumpEnergy);
    hash.Mix(state.rightPumpEnergy);
    hash.Mix(state.previousMeanCollapse);
    hash.Mix(state.recoverySurge);
    hash.Mix(state.recoverySurgeRate);
    hash.Mix(state.deepBrakeTime);
    hash.Mix(state.deepStall);
    hash.Mix(state.spin);
    hash.Mix(state.spiralDevelopment);
    hash.Mix(state.brakeZoomEnergy);
    hash.Mix(state.brakeZoomEnergyJ);
    hash.Mix(state.canopyRelativePitchRad);
    hash.Mix(state.canopyRelativePitchRateRadps);
    hash.Mix(state.canopyRelativeRollRad);
    hash.Mix(state.canopyRelativeRollRateRadps);
    hash.Mix(state.leftSeparatedSpan);
    hash.Mix(state.rightSeparatedSpan);
    for (double value : state.filteredLeftLineTensionN) hash.Mix(value);
    for (double value : state.filteredRightLineTensionN) hash.Mix(value);
    for (double value : state.filteredLeftLineSlack) hash.Mix(value);
    for (double value : state.filteredRightLineSlack) hash.Mix(value);
    hash.Mix(state.leftCravat);
    hash.Mix(state.rightCravat);
    hash.Mix(state.acceleratorTravel);
    hash.Mix(state.acceleratorRate);
    hash.Mix(state.previousDynamicPressurePa);
    hash.Mix(state.previousAngleOfAttackRad);
    hash.Mix(state.flareEnergy);
    hash.Mix(state.flareLift);
    hash.Mix(state.previousMechanicalEnergyJ);
    return hash.Value();
}
}
