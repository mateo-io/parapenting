#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace Parapenting::Physics
{
struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    Vec3 operator/(double scalar) const { return {x / scalar, y / scalar, z / scalar}; }
    Vec3& operator+=(const Vec3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
};

inline double Dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3 Cross(const Vec3& a, const Vec3& b)
{
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
inline double Length(const Vec3& value) { return std::sqrt(Dot(value, value)); }
inline Vec3 Normalized(const Vec3& value)
{
    const double length = Length(value);
    return length > 1e-9 ? value / length : Vec3{};
}

struct Quaternion
{
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Quaternion operator*(const Quaternion& rhs) const;
    Quaternion Normalized() const;
    Vec3 Rotate(const Vec3& value) const;
    Vec3 InverseRotate(const Vec3& value) const;
};

struct WingParameters
{
    double allUpMassKg = 105.0;
    double canopyMassKg = 5.1;
    double areaM2 = 27.0;
    double airDensityKgM3 = 1.12;
    double trimCl = 0.58;
    double trimAngleOfAttackRad = -0.113446401;
    // Canopy-only polar. Harness and projected suspension drag are added
    // separately by the installed-system model below.
    double zeroLiftDrag = 0.018;
    double inducedDragFactor = 0.080;
    double brakeLiftGain = 0.42;
    double brakeDragGain = 0.48;
    std::array<double, 5> brakeLiftCurve{
        0.0, 0.095, 0.235, 0.355, 0.42};
    std::array<double, 5> brakeDragCurve{
        0.0, 0.005, 0.070, 0.245, 0.60};
    double maxLiftCoefficient = 1.35;
    double stallAngleRad = 16.0 * 3.141592653589793 / 180.0;
    double rollInertiaKgM2 = 95.0;
    double pitchInertiaKgM2 = 120.0;
    double yawInertiaKgM2 = 150.0;
    // Air accelerated with the inflated canopy. These research estimates
    // form a diagonal apparent-mass tensor in canopy body axes.
    Vec3 apparentMassKg{7.5, 24.0, 31.0};
    Vec3 apparentRotationalInertiaKgM2{18.0, 34.0, 42.0};
    double rollDamping = 42.0;
    double pitchDamping = 220.0;
    double yawDamping = 165.0;
    double brakeRollMoment = 230.0;
    double brakeYawMoment = 145.0;
    double weightShiftRollMoment = 110.0;
    // Coupled canopy/payload approximation. Brake first yaws through drag;
    // the suspended mass then banks the system toward a coordinated turn.
    // These replace the old unbounded direct brake-to-roll torque.
    double coordinatedRollStiffness = 820.0;
    double coordinatedRollDamping = 145.0;
    double yawToBankGain = 0.72;
    double weightShiftBankRad = 0.48;
    double maximumSustainedBankRad = 0.92;
    double coordinatedYawStiffness = 210.0;
    // Aerodynamic/pendular restoring moment toward the configured trim
    // incidence. A zero value permits a nonphysical persistent post-spiral
    // dive even after roll and yaw rates have decayed.
    double pitchStiffness = 165.0;
    double acceleratorLiftReduction = 0.16;
    double acceleratorDragReduction = 0.006;
    double acceleratorPitchMoment = 42.0;
    double brakeTravelMm = 620.0;
    double brakeFreePlayFraction = 0.10;
    double brakePressureExponent = 1.75;
    double brakeForceAtFullTravelN = 52.0;
    // Flexible-canopy load softening is distinct from the 8 g EN structural
    // qualification boundary. Values are research envelopes pending measured
    // maneuver data.
    double loadSofteningOnsetG = 3.6;
    double operationalLiftLimitG = 4.8;
    double overspeedDragOnsetMps = 18.0;
    double overspeedDragQuadratic = 0.0011;
    // Research stability/recovery envelope. A value above one for collapse
    // resistance reduces identical disturbance-driven folding. Recovery
    // rates are per-second coefficients, not certification claims.
    double collapseResistance = 1.0;
    double passiveReinflationRate = 0.17;
    double brakeReinflationGain = 0.16;
    double pumpReinflationGain = 1.05;
    double frontalReinflationRate = 0.31;
    double cravatSusceptibility = 1.0;
    double recoverySurgeGain = 2.4;
};

struct ControlInput
{
    double leftBrake = 0.0;
    double rightBrake = 0.0;
    double weightShift = 0.0;
    double accelerator = 0.0;
};

struct Atmosphere
{
    Vec3 windWorldMps{};
    double turbulence = 0.0;
    double rotorStrength = 0.0;
    double lateralGust = 0.0;
    double lowFrequencyGustMps = 0.0;
    double highFrequencyGustMps = 0.0;
    double gustEnergyMps = 0.0;
    double groundClearanceM = 10000.0;
    double thermalLiftMps = 0.0;
    double slopeFlowMps = 0.0;
    double sinkRingMps = 0.0;
    // Dominant coherent plume at this sample. These are exposed separately
    // from vertical velocity so coaching, audio and replay analysis can
    // distinguish a thermal's lifecycle from generic rising air.
    double thermalCoreStrength = 0.0;
    double thermalLifecycle = 0.0;
    double cloudBaseClearanceM = 10000.0;
    // Air sampled at the physical canopy halves relative to the centre
    // sample. These spatial deltas let a thermal edge or rotor unload one
    // side without inventing an arbitrary left/right event.
    Vec3 leftWingWindDeltaMps{};
    Vec3 rightWingWindDeltaMps{};
    double spanwiseAirflowShearMps = 0.0;
};

struct HarnessParameters
{
    double dragAreaM2 = 0.32;
    double rollStiffness = 7.5;
    double rollDamping = 3.2;
    double pitchStiffness = 5.8;
    double pitchDamping = 2.8;
    double weightShiftAuthority = 1.0;
};

struct FlightState
{
    Vec3 positionWorldM{};
    Vec3 velocityWorldMps{10.5, 0.0, -1.2};
    Quaternion attitude{};
    Vec3 angularVelocityBodyRadps{};
    double leftCollapse = 0.0;
    double rightCollapse = 0.0;
    double frontalCollapse = 0.0;
    double canopyPressure = 1.0;
    double harnessRollRad = 0.0;
    double harnessRollRateRadps = 0.0;
    double harnessPitchRad = 0.0;
    double harnessPitchRateRadps = 0.0;
    double previousLongitudinalAccelerationMps2 = 0.0;
    double previousSymmetricBrake = 0.0;
    double previousLeftBrake = 0.0;
    double previousRightBrake = 0.0;
    double leftBrakeTravel = 0.0;
    double rightBrakeTravel = 0.0;
    double leftPumpEnergy = 0.0;
    double rightPumpEnergy = 0.0;
    double previousMeanCollapse = 0.0;
    double recoverySurge = 0.0;
    double recoverySurgeRate = 0.0;
    double deepBrakeTime = 0.0;
    double deepStall = 0.0;
    double spin = 0.0;
    double spiralDevelopment = 0.0;
    double brakeZoomEnergy = 0.0;
    double brakeZoomEnergyJ = 0.0;
    double canopyRelativePitchRad = 0.0;
    double canopyRelativePitchRateRadps = 0.0;
    double canopyRelativeRollRad = 0.0;
    double canopyRelativeRollRateRadps = 0.0;
    double leftSeparatedSpan = 0.0;
    double rightSeparatedSpan = 0.0;
    std::array<double, 4> filteredLeftLineTensionN{};
    std::array<double, 4> filteredRightLineTensionN{};
    std::array<double, 4> filteredLeftLineSlack{};
    std::array<double, 4> filteredRightLineSlack{};
    double leftCravat = 0.0;
    double rightCravat = 0.0;
    double acceleratorTravel = 0.0;
    double acceleratorRate = 0.0;
    double previousDynamicPressurePa = 0.0;
    double previousAngleOfAttackRad = 0.0;
    double flareEnergy = 0.8;
    double flareLift = 0.0;
    double previousMechanicalEnergyJ = 0.0;
};

struct Telemetry
{
    double airspeedMps = 0.0;
    double angleOfAttackRad = 0.0;
    double liftCoefficient = 0.0;
    double dragCoefficient = 0.0;
    double loadFactor = 0.0;
    double highLoadDeformation = 0.0;
    double turbulence = 0.0;
    double rotorStrength = 0.0;
    double lowFrequencyGustMps = 0.0;
    double highFrequencyGustMps = 0.0;
    double gustEnergyMps = 0.0;
    double leftCollapse = 0.0;
    double rightCollapse = 0.0;
    double frontalCollapse = 0.0;
    double canopyPressure = 1.0;
    double harnessRollRad = 0.0;
    double harnessPitchRad = 0.0;
    double flareBoost = 0.0;
    double groundEffect = 0.0;
    double flareEnergy = 0.0;
    double flareAuthority = 0.0;
    double leftBrakePressure = 0.0;
    double rightBrakePressure = 0.0;
    double leftBrakeForceN = 0.0;
    double rightBrakeForceN = 0.0;
    double thermalLiftMps = 0.0;
    double thermalCoreStrength = 0.0;
    double thermalLifecycle = 0.0;
    double cloudBaseClearanceM = 10000.0;
    double recoverySurge = 0.0;
    double deepStall = 0.0;
    double spin = 0.0;
    double leftCravat = 0.0;
    double rightCravat = 0.0;
    double spanwiseLoadAsymmetry = 0.0;
    double accelerator = 0.0;
    double aRiserLoad = 0.0;
    double bRiserLoad = 0.0;
    double cRiserLoad = 0.0;
    double dRiserLoad = 0.0;
    double lineLoadTotalN = 0.0;
    double lineDragN = 0.0;
    double suspensionPitchMomentNm = 0.0;
    double brakeLineLoad = 0.0;
    double bankAngleRad = 0.0;
    double coordinatedBankTargetRad = 0.0;
    double spiralDevelopment = 0.0;
    std::array<double, 4> leftLineTensionN{};
    std::array<double, 4> rightLineTensionN{};
    std::array<double, 4> leftLineSlack{};
    std::array<double, 4> rightLineSlack{};
    double lateralLineLoadImbalance = 0.0;
    double leftStalledSpan = 0.0;
    double rightStalledSpan = 0.0;
    double brakeRollAuthority = 1.0;
    double effectiveLeftBrake = 0.0;
    double effectiveRightBrake = 0.0;
    double brakeZoomEnergy = 0.0;
    double brakeZoomLiftCoefficient = 0.0;
    double coordinatedYawTargetRadps = 0.0;
    double brakeZoomEnergyJ = 0.0;
    double brakeZoomAvailableKineticEnergyJ = 0.0;
    double canopyRelativePitchRad = 0.0;
    double canopyRelativePitchRateRadps = 0.0;
    double canopyRelativeRollRad = 0.0;
    double canopyRelativeRollRateRadps = 0.0;
    double suspensionControlTransmission = 1.0;
    double leftSeparatedSpanState = 0.0;
    double rightSeparatedSpanState = 0.0;
    double canopyMassKg = 0.0;
    double payloadMassKg = 0.0;
    Vec3 apparentMassKg{};
    Vec3 effectiveTranslationalMassKg{};
    Vec3 suspensionMomentNm{};
    Vec3 payloadReactionMomentNm{};
    Vec3 aerodynamicForceWorldN{};
    Vec3 gravityForceWorldN{};
    Vec3 accelerationWorldMps2{};
    double mechanicalEnergyJ = 0.0;
    double aerodynamicPowerW = 0.0;
    double energyResidualW = 0.0;
    double leftCarabinerLateralCm = -18.0;
    double rightCarabinerLateralCm = 18.0;
    double carabinerVerticalCm = 34.0;
    double allUpMassKg = 0.0;
    double wingLoadingKgM2 = 0.0;
    double harnessDragN = 0.0;
    double brakeTravelLeftMm = 0.0;
    double brakeTravelRightMm = 0.0;
    double leftReinflationAuthority = 0.0;
    double rightReinflationAuthority = 0.0;
    double leftReinflationRatePerS = 0.0;
    double rightReinflationRatePerS = 0.0;
    double frontalReinflationRatePerS = 0.0;
    double surgeContainment = 0.0;
    double aerodynamicUnloading = 0.0;
    double dynamicPressureDropPaPerS = 0.0;
    double spanwiseAirflowShearMps = 0.0;
    double leftAirflowDisturbance = 0.0;
    double rightAirflowDisturbance = 0.0;
    double collapseResistance = 1.0;
    double passiveReinflationRate = 0.0;
    double frontalReinflationRate = 0.0;
    double cravatSusceptibility = 1.0;
    bool stalled = false;
};

class ParagliderDynamics
{
public:
    explicit ParagliderDynamics(WingParameters parameters = {});

    void Step(FlightState& state, const ControlInput& controls,
              const Atmosphere& atmosphere, double deltaSeconds);

    const WingParameters& Parameters() const { return Params; }
    void SetParameters(const WingParameters& parameters) { Params = parameters; }
    void SetHarnessParameters(const HarnessParameters& parameters)
        { Harness = parameters; }
    const Telemetry& LastTelemetry() const { return TelemetryState; }

private:
    WingParameters Params;
    HarnessParameters Harness;
    Telemetry TelemetryState;
};
}
