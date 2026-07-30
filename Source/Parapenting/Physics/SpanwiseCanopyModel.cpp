#include "SpanwiseCanopyModel.h"
#include "AerodynamicPolar.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
SpanwiseAeroResult EvaluateSpanwiseCanopy(
    const WingParameters& p, const FlightState& state,
    const ControlInput& controls, double baseCl, double flareBoost,
    double dynamicPressure, double airspeedMps,
    double globalAngleOfAttackRad, bool stalled, double inducedDragReduction,
    const Atmosphere& atmosphere)
{
    constexpr int PanelCount = 16;
    constexpr double AspectRatio = 5.2;
    const double spanM = std::sqrt(p.areaM2 * AspectRatio);
    double weightSum = 0.0;
    double leftLoad = 0.0;
    double rightLoad = 0.0;
    double leftPanelWeight = 0.0;
    double rightPanelWeight = 0.0;
    SpanwiseAeroResult result;
    const double commonBrake =
        std::min(controls.leftBrake, controls.rightBrake);
    const PolarSample commonBrakePolar = SampleBrakePolar(
        p.brakeLiftCurve, p.brakeDragCurve, commonBrake);

    for (int panel = 0; panel < PanelCount; ++panel)
    {
        const double span01 =
            -1.0 + 2.0 * (static_cast<double>(panel) + 0.5) / PanelCount;
        const double weight = std::sqrt(std::max(0.05, 1.0 - span01 * span01));
        const double tipBlend = std::clamp(
            (std::abs(span01) - 0.35) / 0.65, 0.0, 1.0);
        const bool left = span01 < 0.0;
        const double brake = left ? controls.leftBrake : controls.rightBrake;
        const double collapse =
            (left ? state.leftCollapse : state.rightCollapse) * tipBlend;
        const double cravat =
            (left ? state.leftCravat : state.rightCravat) * tipBlend;
        const double armM = span01 * spanM * 0.5;
        // Body roll creates opposite vertical velocities at the tips while
        // yaw changes their local tangential speed. Both alter panel airflow.
        const double verticalPanelVelocity =
            state.angularVelocityBodyRadps.x * armM;
        const double tangentialPanelVelocity =
            -state.angularVelocityBodyRadps.z * armM;
        // The atmosphere provides centre-relative wind at each outer
        // half-span. Linearly distribute that measured gradient through the
        // sixteen strips. This preserves the centre sample used by the
        // translational solver while giving every strip its own incidence and
        // dynamic pressure at thermal/rotor boundaries.
        const Vec3 tipDeltaWorld = left
            ? atmosphere.leftWingWindDeltaMps
            : atmosphere.rightWingWindDeltaMps;
        const Vec3 panelWindDeltaBody = state.attitude.InverseRotate(
            tipDeltaWorld * std::abs(span01));
        const double localForwardSpeed = std::max(
            0.5, airspeedMps + tangentialPanelVelocity
                - panelWindDeltaBody.x);
        const double localVerticalFlow =
            verticalPanelVelocity + panelWindDeltaBody.z;
        const double localAirspeed = std::max(
            0.5, std::sqrt(localForwardSpeed * localForwardSpeed
                + localVerticalFlow * localVerticalFlow
                + panelWindDeltaBody.y * panelWindDeltaBody.y));
        const double localDynamicPressure = dynamicPressure
            * (localAirspeed * localAirspeed)
            / std::max(0.25, airspeedMps * airspeedMps);
        // A panel moving upward meets the relative airflow at a lower local
        // incidence. Keeping this sign opposing roll rate provides the
        // aerodynamic roll damping of a real span instead of feeding energy
        // back into the roll mode.
        const double localAngleOfAttack = globalAngleOfAttackRad
            - std::atan2(localVerticalFlow, localForwardSpeed);
        const PolarSample brakePolar = SampleBrakePolar(
            p.brakeLiftCurve, p.brakeDragCurve, brake);

        double sectionCl = baseCl
            + 2.2 * (localAngleOfAttack - globalAngleOfAttackRad)
            + commonBrakePolar.liftDelta + flareBoost
            - 0.11 * controls.weightShift * span01;
        // Deep unilateral brake does not keep creating lift indefinitely.
        // Separation starts earlier on a slow/high-incidence panel and moves
        // outward through the braked half, producing spin rather than an
        // unlimited roll command.
        const double speedStallBias = std::clamp(
            (8.2 - localAirspeed) / 3.2, 0.0, 0.24);
        const double localStallStart = 0.78 - speedStallBias
            - 0.08 * std::clamp(
                (localAngleOfAttack - p.trimAngleOfAttackRad) / 0.22,
                0.0, 1.0);
        const double localBrakeStall = std::clamp(
            (brake - localStallStart) / 0.18, 0.0, 1.0);
        const double separationMemory = left
            ? state.leftSeparatedSpan : state.rightSeparatedSpan;
        const double effectiveSeparation = std::max(
            localBrakeStall, separationMemory * (0.72 + 0.28 * tipBlend));
        sectionCl *= 1.0 - 0.72 * effectiveSeparation;
        if (stalled)
            sectionCl *= 0.42 + 0.30 * (1.0 - brake);
        sectionCl *= 1.0 - 0.78 * collapse;
        sectionCl *= 1.0 - 0.84 * cravat;
        sectionCl *= 1.0 - 0.62 * state.frontalCollapse;
        sectionCl *= 1.0 - 0.68 * state.deepStall;
        sectionCl = std::clamp(sectionCl, -0.65, p.maxLiftCoefficient);

        const double sectionCd =
            p.zeroLiftDrag
            + p.inducedDragFactor * (1.0 - inducedDragReduction)
                * sectionCl * sectionCl
            + brakePolar.dragDelta
            + 0.58 * collapse + 0.76 * cravat
            + 0.68 * state.frontalCollapse
            + 0.92 * state.deepStall
            + 0.38 * std::abs(state.spin)
            + 0.46 * effectiveSeparation
            + (stalled ? 0.34 : 0.0);

        const double pressureRatio =
            localDynamicPressure / std::max(0.01, dynamicPressure);
        result.liftCoefficient += sectionCl * weight * pressureRatio;
        result.dragCoefficient += sectionCd * weight * pressureRatio;
        const double panelArea = p.areaM2 * weight / PanelCount;
        result.rollMomentNm +=
            localDynamicPressure * panelArea * sectionCl * armM;
        result.yawMomentNm +=
            localDynamicPressure * panelArea * sectionCd * armM * 0.10;
        if (left) leftLoad += sectionCl * weight * pressureRatio;
        else rightLoad += sectionCl * weight * pressureRatio;
        if (left)
        {
            result.leftStalledFraction += effectiveSeparation * weight;
            leftPanelWeight += weight;
        }
        else
        {
            result.rightStalledFraction += effectiveSeparation * weight;
            rightPanelWeight += weight;
        }
        weightSum += weight;
    }

    result.liftCoefficient /= weightSum;
    result.dragCoefficient /= weightSum;
    const double totalLoad = std::max(0.05, std::abs(leftLoad) + std::abs(rightLoad));
    result.loadAsymmetry = std::clamp(
        (rightLoad - leftLoad) / totalLoad, -1.0, 1.0);
    result.leftStalledFraction /= std::max(0.01, leftPanelWeight);
    result.rightStalledFraction /= std::max(0.01, rightPanelWeight);
    return result;
}
}
