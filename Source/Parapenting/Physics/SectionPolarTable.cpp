#include "SectionPolarTable.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double Pi = 3.14159265358979323846;

// Table resolution. Fine enough that linear interpolation between samples is
// well under the uncertainty of the analytic model that produced them.
constexpr std::size_t AlphaSamples = 361;   // 0.5 deg over -90..90
constexpr std::size_t BrakeSamples = 21;
constexpr double AlphaMin = -0.5 * Pi;
constexpr double AlphaMax = 0.5 * Pi;

// Thin-airfoil lift-curve slope, with the usual thickness correction. The
// 2 pi of thin-airfoil theory is for a zero-thickness section; a real one is
// slightly stiffer.
double LiftSlope(double thicknessFraction)
{
    return 2.0 * Pi * (1.0 + 0.77 * thicknessFraction);
}

// Zero-lift angle of a circular-arc camber line. Exact under thin-airfoil
// theory: alpha_L0 = -2 h/c.
double BaseZeroLiftAngle(double camberFraction)
{
    return -2.0 * camberFraction;
}
}

double ThinAirfoilFlapEffectiveness(double flapChordFraction)
{
    // The camber line maps to its angular coordinate as x/c = (1 - cos t)/2,
    // so the hinge sits at t = acos(1 - 2 x/c). Writing that as acos(2 x/c - 1)
    // reflects it about pi/2 and turns a 22% flap from 57% effective into 95%
    // effective - which would make brake very nearly as powerful as pitching
    // the whole wing, and stalls the canopy at half brake.
    const double ratio = std::clamp(flapChordFraction, 0.0, 1.0);
    const double hinge = std::acos(std::clamp(1.0 - 2.0 * ratio, -1.0, 1.0));
    return 1.0 - (hinge - std::sin(hinge)) / Pi;
}

SectionPolarTable SectionPolarTable::Analytic(const AnalyticPolarSpec& spec)
{
    SectionPolarTable table;
    table.SpecValue = spec;
    table.Source = PolarProvenance::Analytic;
    table.AlphaCount = AlphaSamples;
    table.BrakeCount = BrakeSamples;
    table.AlphaMinRad = AlphaMin;
    table.AlphaMaxRad = AlphaMax;
    table.Samples.resize(AlphaSamples * BrakeSamples);
    table.Attached.resize(AlphaSamples * BrakeSamples);
    table.Separated.resize(AlphaSamples * BrakeSamples);
    table.SeparationCurve.resize(AlphaSamples * BrakeSamples);

    const double slope = LiftSlope(spec.thicknessFraction);
    const double flapEffectiveness =
        ThinAirfoilFlapEffectiveness(spec.flapChordFraction);

    // Viterna-Corrigan post-stall. CDmax for a finite wing, from the 1982
    // paper: 1.11 + 0.018 AR.
    const double dragMax = 1.11 + 0.018 * spec.aspectRatioForPostStall;

    for (std::size_t brakeIndex = 0; brakeIndex < BrakeSamples; ++brakeIndex)
    {
        const double brake = static_cast<double>(brakeIndex)
            / static_cast<double>(BrakeSamples - 1);
        // Brake deflects the trailing edge, which shifts the zero-lift angle.
        // The section flies at more camber, not more incidence.
        const double deflection = brake * spec.fullBrakeDeflectionRad;
        const double zeroLift = BaseZeroLiftAngle(spec.camberFraction)
            - flapEffectiveness * deflection;
        // A deflected trailing edge stalls earlier: the suction peak it adds
        // is what runs out first.
        const double stallMargin = spec.stallMarginRad
            * (1.0 - spec.stallMarginBrakeLoss * brake);
        const double stallAlpha = zeroLift + stallMargin;
        const double stallCl = slope * stallMargin;
        const double stallCd = spec.minimumDragCoefficient
            + spec.dragRiseFactor * stallCl * stallCl
            // Flap drag: a deflected trailing edge is a bluff element.
            + 0.30 * deflection * deflection;

        // Viterna coefficients, evaluated at the stall point so the two
        // branches meet.
        const double sinStall = std::sin(std::max(0.05, stallAlpha));
        const double cosStall = std::cos(stallAlpha);
        const double a1 = 0.5 * dragMax;
        const double b1 = dragMax;
        const double a2 = (stallCl - dragMax * sinStall * cosStall)
            * sinStall / std::max(1e-3, cosStall * cosStall);
        const double b2 = (stallCd - dragMax * sinStall * sinStall)
            / std::max(1e-3, cosStall);

        for (std::size_t alphaIndex = 0; alphaIndex < AlphaSamples;
             ++alphaIndex)
        {
            const double alpha = AlphaMin
                + (AlphaMax - AlphaMin) * static_cast<double>(alphaIndex)
                    / static_cast<double>(AlphaSamples - 1);

            const double attachedCl = slope * (alpha - zeroLift);
            const double attachedCd = spec.minimumDragCoefficient
                + spec.dragRiseFactor * attachedCl * attachedCl
                + 0.30 * deflection * deflection;

            const double sinAlpha = std::sin(alpha);
            const double cosAlpha = std::cos(alpha);
            const double separatedCl = a1 * std::sin(2.0 * alpha)
                + a2 * cosAlpha * cosAlpha
                    / (std::fabs(sinAlpha) < 0.05
                        ? std::copysign(0.05, sinAlpha == 0.0 ? 1.0 : sinAlpha)
                        : sinAlpha);
            const double separatedCd = b1 * sinAlpha * sinAlpha
                + b2 * cosAlpha;

            // Blend from attached to separated across the stall. Exactly zero
            // below it, exactly one beyond the blend width, smooth between:
            // a section below its stall angle must carry none of the
            // post-stall branch at all. That a smooth blend stands in for
            // real stall hysteresis remains a modelling choice - this table
            // has no memory, and the attached/separated state machine is
            // still owed.
            const double past = std::fabs(alpha - zeroLift) - stallMargin;
            const double blendWidth = std::max(1.0e-3,
                spec.stallBlendWidthRad);
            const double t = std::clamp(past / blendWidth, 0.0, 1.0);
            const double separation = t * t * (3.0 - 2.0 * t);

            SectionPolarSample sample;
            sample.liftCoefficient =
                attachedCl * (1.0 - separation) + separatedCl * separation;
            sample.dragCoefficient = std::max(
                spec.minimumDragCoefficient,
                attachedCd * (1.0 - separation) + separatedCd * separation);
            SectionPolarSample attachedBranch;
            attachedBranch.liftCoefficient = attachedCl;
            attachedBranch.dragCoefficient = attachedCd;
            SectionPolarSample separatedBranch;
            separatedBranch.liftCoefficient = separatedCl;
            separatedBranch.dragCoefficient = std::max(
                spec.minimumDragCoefficient, separatedCd);
            // Quarter-chord moment of a cambered section, plus the flap
            // increment. Thin-airfoil theory puts the flap contribution well
            // aft, which is why brake pitches the section nose-down.
            sample.momentCoefficient =
                -0.25 * Pi * spec.camberFraction
                - 0.60 * flapEffectiveness * deflection;

            const std::size_t at = brakeIndex * AlphaSamples + alphaIndex;
            attachedBranch.momentCoefficient = sample.momentCoefficient;
            separatedBranch.momentCoefficient = sample.momentCoefficient;
            table.Samples[at] = sample;
            table.Attached[at] = attachedBranch;
            table.Separated[at] = separatedBranch;
            table.SeparationCurve[at] = separation;
        }
    }
    return table;
}

SectionPolarSample SectionPolarTable::Sample(
    double alphaRad, double brake) const
{
    if (Samples.empty()) return {};

    const double clampedAlpha = std::clamp(alphaRad, AlphaMinRad, AlphaMaxRad);
    const double alphaPosition =
        (clampedAlpha - AlphaMinRad) / (AlphaMaxRad - AlphaMinRad)
            * static_cast<double>(AlphaCount - 1);
    const auto alphaLow = static_cast<std::size_t>(alphaPosition);
    const std::size_t alphaHigh = std::min(alphaLow + 1, AlphaCount - 1);
    const double alphaT = alphaPosition - static_cast<double>(alphaLow);

    const double brakePosition = std::clamp(brake, 0.0, 1.0)
        * static_cast<double>(BrakeCount - 1);
    const auto brakeLow = static_cast<std::size_t>(brakePosition);
    const std::size_t brakeHigh = std::min(brakeLow + 1, BrakeCount - 1);
    const double brakeT = brakePosition - static_cast<double>(brakeLow);

    const auto at = [this](std::size_t brakeIndex, std::size_t alphaIndex)
    {
        return Samples[brakeIndex * AlphaCount + alphaIndex];
    };
    const auto blend = [](const SectionPolarSample& a,
                          const SectionPolarSample& b, double t)
    {
        SectionPolarSample result;
        result.liftCoefficient = a.liftCoefficient
            + (b.liftCoefficient - a.liftCoefficient) * t;
        result.dragCoefficient = a.dragCoefficient
            + (b.dragCoefficient - a.dragCoefficient) * t;
        result.momentCoefficient = a.momentCoefficient
            + (b.momentCoefficient - a.momentCoefficient) * t;
        return result;
    };

    return blend(
        blend(at(brakeLow, alphaLow), at(brakeLow, alphaHigh), alphaT),
        blend(at(brakeHigh, alphaLow), at(brakeHigh, alphaHigh), alphaT),
        brakeT);
}

SectionPolarSample SectionPolarTable::SampleAtSeparation(
    double alphaRad, double brake, double separation) const
{
    if (Samples.empty()) return {};
    const double blend = std::clamp(separation, 0.0, 1.0);

    const double clampedAlpha = std::clamp(alphaRad, AlphaMinRad, AlphaMaxRad);
    const double alphaPosition =
        (clampedAlpha - AlphaMinRad) / (AlphaMaxRad - AlphaMinRad)
            * static_cast<double>(AlphaCount - 1);
    const auto alphaLow = static_cast<std::size_t>(alphaPosition);
    const std::size_t alphaHigh = std::min(alphaLow + 1, AlphaCount - 1);
    const double alphaT = alphaPosition - static_cast<double>(alphaLow);

    const double brakePosition = std::clamp(brake, 0.0, 1.0)
        * static_cast<double>(BrakeCount - 1);
    const auto brakeLow = static_cast<std::size_t>(brakePosition);
    const std::size_t brakeHigh = std::min(brakeLow + 1, BrakeCount - 1);
    const double brakeT = brakePosition - static_cast<double>(brakeLow);

    const auto pick = [&](const std::vector<SectionPolarSample>& table,
                          std::size_t brakeIndex, std::size_t alphaIndex)
    {
        return table[brakeIndex * AlphaCount + alphaIndex];
    };
    const auto mix = [](const SectionPolarSample& a,
                        const SectionPolarSample& b, double t)
    {
        SectionPolarSample result;
        result.liftCoefficient = a.liftCoefficient
            + (b.liftCoefficient - a.liftCoefficient) * t;
        result.dragCoefficient = a.dragCoefficient
            + (b.dragCoefficient - a.dragCoefficient) * t;
        result.momentCoefficient = a.momentCoefficient
            + (b.momentCoefficient - a.momentCoefficient) * t;
        return result;
    };
    const auto bilinear = [&](const std::vector<SectionPolarSample>& table)
    {
        return mix(
            mix(pick(table, brakeLow, alphaLow),
                pick(table, brakeLow, alphaHigh), alphaT),
            mix(pick(table, brakeHigh, alphaLow),
                pick(table, brakeHigh, alphaHigh), alphaT),
            brakeT);
    };

    return mix(bilinear(Attached), bilinear(Separated), blend);
}

double SectionPolarTable::SeparationEquilibrium(
    double alphaRad, double brake, double currentSeparation) const
{
    const double stall = StallAngleRad(brake);
    const double zeroLift = ZeroLiftAngleRad(brake);
    const double margin = stall - zeroLift;
    const double excess = std::fabs(alphaRad - zeroLift) - margin;

    // Reattachment happens lower than separation did. Which curve applies
    // depends on which way the section is already going, and that is the
    // whole of the hysteresis: a stalled section stays stalled through the
    // band, an attached one stays attached.
    const double shift = currentSeparation > 0.5
        ? -SpecValue.reattachmentHysteresisRad : 0.0;
    const double width = std::max(1.0e-3, SpecValue.stallBlendWidthRad);
    const double t = std::clamp((excess - shift) / width, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double SectionPolarTable::LiftCurveSlopePerRad(double) const
{
    return LiftSlope(SpecValue.thicknessFraction);
}

double SectionPolarTable::ZeroLiftAngleRad(double brake) const
{
    const double deflection =
        std::clamp(brake, 0.0, 1.0) * SpecValue.fullBrakeDeflectionRad;
    return BaseZeroLiftAngle(SpecValue.camberFraction)
        - ThinAirfoilFlapEffectiveness(SpecValue.flapChordFraction)
            * deflection;
}

double SectionPolarTable::StallAngleRad(double brake) const
{
    return ZeroLiftAngleRad(brake)
        + SpecValue.stallMarginRad
            * (1.0 - SpecValue.stallMarginBrakeLoss
                   * std::clamp(brake, 0.0, 1.0));
}
}
