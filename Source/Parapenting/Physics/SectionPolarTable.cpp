#include "SectionPolarTable.h"

#include "SectionPolarCache.h"

#include "SectionViscousSolver.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <utility>

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
    table.ZeroLiftByBrake.resize(BrakeSamples);
    table.SlopeByBrake.resize(BrakeSamples);
    table.StallByBrake.resize(BrakeSamples);
    table.MaximumLiftByBrake.resize(BrakeSamples);
    table.BlendWidthRad = spec.stallBlendWidthRad;
    table.ReattachmentRad = spec.reattachmentHysteresisRad;

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

        table.ZeroLiftByBrake[brakeIndex] = zeroLift;
        table.SlopeByBrake[brakeIndex] = slope;
        table.StallByBrake[brakeIndex] = stallAlpha;
        table.MaximumLiftByBrake[brakeIndex] = stallCl;

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
            //
            // For a circular-arc (parabolic) camber line z/c = 4 (h/c) x/c
            // (1 - x/c), the Fourier coefficients of the camber slope are
            // A1 = 4 h/c and A2 = 0, and
            //
            //     Cm_c/4 = (pi/4) (A2 - A1) = -pi (h/c)
            //
            // which is -0.110 at this section's 3.5% camber. It is the same
            // camber line that gives BaseZeroLiftAngle its -2 (h/c) above, so
            // the two must be derived from the same A1 or they describe
            // different sections.
            //
            // This read -(pi/4)(h/c) = -0.0275, four times too small, which is
            // what taking A1 as h/c rather than 4 h/c gives - the factor of
            // four in the Fourier coefficient dropped, while the (pi/4)
            // prefactor was kept. Nothing caught it because nothing balanced
            // this moment against anything until the wing and the pilot became
            // two bodies: with the canopy pinned, the wing's own pitching
            // moment had no way to set its incidence, so being four times too
            // small changed no number anyone read.
            sample.momentCoefficient =
                -Pi * spec.camberFraction
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

namespace
{
// Viterna-Corrigan (1982), anchored wherever the section actually stalls. The
// coefficients are fixed by requiring the post-stall branch to meet the
// attached one at the stall point, so moving the stall point moves the whole
// branch with it - which is what has to happen once the stall point is a
// solved quantity rather than a stated one.
struct ViternaBranch
{
    double a1 = 0.0;
    double b1 = 0.0;
    double a2 = 0.0;
    double b2 = 0.0;

    SectionPolarSample At(double alpha) const
    {
        const double sinAlpha = std::sin(alpha);
        const double cosAlpha = std::cos(alpha);
        const double guardedSin = std::fabs(sinAlpha) < 0.05
            ? std::copysign(0.05, sinAlpha == 0.0 ? 1.0 : sinAlpha)
            : sinAlpha;
        SectionPolarSample sample;
        sample.liftCoefficient = a1 * std::sin(2.0 * alpha)
            + a2 * cosAlpha * cosAlpha / guardedSin;
        sample.dragCoefficient = b1 * sinAlpha * sinAlpha + b2 * cosAlpha;
        return sample;
    }
};

ViternaBranch MakeViterna(double stallAlpha, double stallCl, double stallCd,
                          double aspectRatio)
{
    const double dragMax = 1.11 + 0.018 * aspectRatio;
    const double sinStall = std::sin(std::max(0.05, stallAlpha));
    const double cosStall = std::cos(stallAlpha);
    ViternaBranch branch;
    branch.a1 = 0.5 * dragMax;
    branch.b1 = dragMax;
    branch.a2 = (stallCl - dragMax * sinStall * cosStall)
        * sinStall / std::max(1e-3, cosStall * cosStall);
    // Viterna's b2 goes negative whenever the section stalls at a drag
    // coefficient below CDmax sin^2(stall), which a clean section at a
    // million Reynolds always does - and a negative b2 gives the post-stall
    // branch a drag that falls as it separates further. Clamped at zero,
    // which costs the exact join in drag at the stall point and keeps the
    // branch monotone, and the join in lift is untouched.
    branch.b2 = std::max(0.0,
        (stallCd - dragMax * sinStall * sinStall) / std::max(1e-3, cosStall));
    return branch;
}

double InterpolateByBrake(const std::vector<double>& values, double brake)
{
    if (values.empty()) return 0.0;
    if (values.size() == 1) return values.front();
    const double position = std::clamp(brake, 0.0, 1.0)
        * static_cast<double>(values.size() - 1);
    const auto low = static_cast<std::size_t>(position);
    const std::size_t high = std::min(low + 1, values.size() - 1);
    const double t = position - static_cast<double>(low);
    return values[low] + (values[high] - values[low]) * t;
}
}

SectionPolarTable SectionPolarTable::Computed(const ComputedPolarSpec& spec)
{
    SectionPolarTable table;
    table.ComputedValue = spec;
    table.Source = PolarProvenance::Computed;
    const std::size_t brakeCount = std::max<std::size_t>(2, spec.brakeSamples);
    table.AlphaCount = AlphaSamples;
    table.BrakeCount = brakeCount;
    table.AlphaMinRad = AlphaMin;
    table.AlphaMaxRad = AlphaMax;
    table.Samples.resize(AlphaSamples * brakeCount);
    table.Attached.resize(AlphaSamples * brakeCount);
    table.Separated.resize(AlphaSamples * brakeCount);
    table.SeparationCurve.resize(AlphaSamples * brakeCount);
    table.ZeroLiftByBrake.resize(brakeCount);
    table.SlopeByBrake.resize(brakeCount);
    table.StallByBrake.resize(brakeCount);
    table.MaximumLiftByBrake.resize(brakeCount);
    table.BlendWidthRad = spec.stallBlendWidthRad;
    table.ReattachmentRad = spec.reattachmentHysteresisRad;

    // The analytic spec is still carried, because the registry and the
    // aerodynamics suite both describe the section by thickness, camber and
    // brake travel and those are the same numbers the profile is drawn from.
    table.SpecValue.thicknessFraction = spec.section.maxThicknessFraction;
    table.SpecValue.camberFraction = spec.section.maxCamberFraction;
    table.SpecValue.flapChordFraction = spec.section.brakeChordFraction;
    table.SpecValue.fullBrakeDeflectionRad = spec.section.fullBrakeDeflectionRad;
    table.SpecValue.aspectRatioForPostStall = spec.aspectRatioForPostStall;
    table.SpecValue.stallBlendWidthRad = spec.stallBlendWidthRad;
    table.SpecValue.reattachmentHysteresisRad = spec.reattachmentHysteresisRad;

    // Incidences actually solved on the contour. One degree is finer than the
    // curvature of the lift curve anywhere it matters, and the table
    // interpolates between them at half that.
    constexpr double SweepStepRad = 0.0174532925199433;   // 1 deg

    for (std::size_t brakeIndex = 0; brakeIndex < brakeCount; ++brakeIndex)
    {
        const double brake = static_cast<double>(brakeIndex)
            / static_cast<double>(brakeCount - 1);
        const SectionProfile profile =
            BuildSectionProfile(spec.section, brake);
        const SectionViscousSolver solver(profile, spec.reynoldsNumber);

        struct Solved
        {
            double alpha = 0.0;
            SectionAerodynamics flow{};
        };
        std::vector<Solved> sweep;

        // Swept outward from zero in both directions, carrying the separation
        // state along, so each solve starts on the branch the section is
        // already flying and only leaves it where that branch stops existing.
        // Started cold at every angle instead, the deeply separated state -
        // which is also a fixed point, and is the deep stall - captures the
        // sweep the moment it is reachable, and the polar has no peak at all.
        const auto run = [&](double from, double to, double step)
        {
            std::vector<Solved> branch;
            double lowerAttached = 1.0;
            double upperAttached = 1.0;
            for (double alpha = from;
                 step > 0.0 ? alpha <= to + 1e-9 : alpha >= to - 1e-9;
                 alpha += step)
            {
                Solved solved;
                solved.alpha = alpha;
                solved.flow = solver.Solve(alpha, lowerAttached, upperAttached);
                lowerAttached = solved.flow.lowerAttachedFraction;
                upperAttached = solved.flow.upperAttachedFraction;
                branch.push_back(solved);
            }
            return branch;
        };

        const std::vector<Solved> downward =
            run(0.0, spec.sweepLowRad, -SweepStepRad);
        const std::vector<Solved> upward =
            run(SweepStepRad, spec.sweepHighRad, SweepStepRad);
        for (std::size_t i = downward.size(); i-- > 0;)
        {
            sweep.push_back(downward[i]);
        }
        sweep.insert(sweep.end(), upward.begin(), upward.end());

        // Where the section stalls: the incidence at which the solved lift
        // stops rising, taken off the upward branch before it collapses into
        // the fully separated state. Nothing states it.
        double stallAlpha = spec.sweepHighRad;
        double stallCl = 0.0;
        double stallCd = 0.05;
        double stallCm = -0.1;
        {
            // The branch ends where the solve falls into the fully separated
            // state, which is the deep stall and is a different flow. The
            // peak is the largest lift on this side of that.
            //
            // Reading it as "the first angle whose lift did not exceed the
            // last" instead - which is what this did - makes the stall angle
            // jump around between neighbouring brake settings, because a
            // single station where the Kirchhoff iteration lands in a cycle
            // rather than on a point is enough to end the search three
            // degrees early. Measured across the brake axis it gave 10, 11,
            // 7, 12, 3 and 13 degrees, which is noise with a stall angle
            // written on it.
            std::size_t last = 0;
            for (std::size_t i = 0; i < upward.size(); ++i)
            {
                if (upward[i].flow.separatedChordFraction > 0.6) break;
                last = i;
            }
            for (std::size_t i = 0; i <= last && i < upward.size(); ++i)
            {
                if (upward[i].flow.liftCoefficient <= stallCl) continue;
                stallAlpha = upward[i].alpha;
                stallCl = upward[i].flow.liftCoefficient;
                stallCd = upward[i].flow.dragCoefficient;
                stallCm = upward[i].flow.momentCoefficient;
            }
        }
        const ViternaBranch viterna = MakeViterna(
            stallAlpha, stallCl, stallCd, spec.aspectRatioForPostStall);

        // The attached branch, as a straight line through the part of the
        // solved curve that is straight. Fitted over the four degrees either
        // side of zero, where the section is unquestionably attached.
        double zeroLiftAlpha = 0.0;
        double slope = 2.0 * Pi;
        {
            const auto liftAt = [&](double alpha)
            {
                double best = 0.0;
                double bestDistance = 1e30;
                for (const Solved& solved : sweep)
                {
                    const double distance = std::fabs(solved.alpha - alpha);
                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        best = solved.flow.attachedLiftCoefficient;
                    }
                }
                return best;
            };
            const double low = liftAt(-4.0 * SweepStepRad);
            const double high = liftAt(4.0 * SweepStepRad);
            slope = (high - low) / (8.0 * SweepStepRad);
            const double atZero = liftAt(0.0);
            if (std::fabs(slope) > 1e-6) zeroLiftAlpha = -atZero / slope;
        }

        table.ZeroLiftByBrake[brakeIndex] = zeroLiftAlpha;
        table.SlopeByBrake[brakeIndex] = slope;
        table.StallByBrake[brakeIndex] = stallAlpha;
        table.MaximumLiftByBrake[brakeIndex] = stallCl;

        // Interpolates the solved sweep, and continues it linearly outside
        // the solved range. Past the stall the attached branch is what the
        // section would carry if the flow never let go, which is exactly the
        // branch the caller blends against with its own separation state.
        const auto solvedAt = [&](double alpha)
        {
            SectionPolarSample sample;
            if (sweep.empty()) return sample;
            const double lowAlpha = sweep.front().alpha;
            const double highAlpha = sweep.back().alpha;
            if (alpha <= lowAlpha || alpha >= highAlpha)
            {
                const Solved& edge = alpha <= lowAlpha
                    ? sweep.front() : sweep.back();
                sample.liftCoefficient = edge.flow.attachedLiftCoefficient
                    + slope * (alpha - edge.alpha);
                sample.dragCoefficient = edge.flow.dragCoefficient;
                sample.momentCoefficient = edge.flow.attachedMomentCoefficient;
                return sample;
            }
            const double position = (alpha - lowAlpha) / SweepStepRad;
            const auto low = static_cast<std::size_t>(position);
            const std::size_t high = std::min(low + 1, sweep.size() - 1);
            const double t = position - static_cast<double>(low);
            const SectionAerodynamics& a = sweep[low].flow;
            const SectionAerodynamics& b = sweep[high].flow;
            sample.liftCoefficient = a.liftCoefficient
                + (b.liftCoefficient - a.liftCoefficient) * t;
            sample.dragCoefficient = a.dragCoefficient
                + (b.dragCoefficient - a.dragCoefficient) * t;
            sample.momentCoefficient = a.momentCoefficient
                + (b.momentCoefficient - a.momentCoefficient) * t;
            return sample;
        };

        for (std::size_t alphaIndex = 0; alphaIndex < AlphaSamples;
             ++alphaIndex)
        {
            const double alpha = AlphaMin
                + (AlphaMax - AlphaMin) * static_cast<double>(alphaIndex)
                    / static_cast<double>(AlphaSamples - 1);

            SectionPolarSample attachedBranch = solvedAt(alpha);
            // Past the solved peak the attached branch continues along the
            // line it was on. A branch that turned over here would give the
            // caller's separation state a negative lift slope to blend
            // against, which is the thing limitation 6 is about.
            const double mirroredStall = -stallAlpha + 2.0 * zeroLiftAlpha;
            if (alpha > stallAlpha)
            {
                attachedBranch.liftCoefficient =
                    stallCl + slope * (alpha - stallAlpha);
                attachedBranch.momentCoefficient = stallCm;
            }
            else if (alpha < mirroredStall)
            {
                attachedBranch.liftCoefficient = -stallCl
                    + slope * (alpha - mirroredStall);
                attachedBranch.momentCoefficient = stallCm;
            }

            SectionPolarSample separatedBranch = viterna.At(alpha);
            // A fully separated section carries its resultant at mid chord,
            // so about the quarter chord the moment is a quarter of the
            // normal force, nose down. This is the companion to Viterna's
            // lift and drag and it is why a deeply stalled wing pitches down
            // rather than holding the section moment it had while flying -
            // which is what taking the attached branch's moment here would
            // have said.
            const double normalForce =
                separatedBranch.liftCoefficient * std::cos(alpha)
                + separatedBranch.dragCoefficient * std::sin(alpha);
            separatedBranch.momentCoefficient = -0.25 * normalForce;

            // Where the section is on the two branches. Measured from the
            // solved stall, symmetric about the zero-lift angle so a section
            // pushed to negative incidence stalls too.
            const double past = alpha > zeroLiftAlpha
                ? alpha - stallAlpha
                : (2.0 * zeroLiftAlpha - stallAlpha) - alpha;
            const double width = std::max(1.0e-3, spec.stallBlendWidthRad);
            const double t = std::clamp(past / width, 0.0, 1.0);
            const double separation = t * t * (3.0 - 2.0 * t);

            SectionPolarSample sample;
            sample.liftCoefficient =
                attachedBranch.liftCoefficient * (1.0 - separation)
                + separatedBranch.liftCoefficient * separation;
            sample.dragCoefficient = std::max(0.0,
                attachedBranch.dragCoefficient * (1.0 - separation)
                + separatedBranch.dragCoefficient * separation);
            sample.momentCoefficient =
                attachedBranch.momentCoefficient * (1.0 - separation)
                + separatedBranch.momentCoefficient * separation;

            const std::size_t at = brakeIndex * AlphaSamples + alphaIndex;
            separatedBranch.dragCoefficient =
                std::max(0.0, separatedBranch.dragCoefficient);
            table.Samples[at] = sample;
            table.Attached[at] = attachedBranch;
            table.Separated[at] = separatedBranch;
            table.SeparationCurve[at] = separation;
        }
    }
    return table;
}

const SectionPolarTable& SectionPolarTable::Default()
{
    return ForSection(SectionProfileSpec{});
}

const SectionPolarTable& SectionPolarTable::ForSection(
    const SectionProfileSpec& section)
{
    // A deque, not a vector: callers hold the reference this returns, and a
    // deque keeps references valid when a later section is appended.
    static std::mutex guard;
    static std::deque<std::pair<SectionProfileSpec, SectionPolarTable>> built;
    const std::lock_guard<std::mutex> lock(guard);
    for (const auto& entry : built)
    {
        if (entry.first == section) return entry.second;
    }
    ComputedPolarSpec spec;
    spec.section = section;

    // Try the cache before solving. A miss for ANY reason - no file, a changed
    // spec, or a witness that no longer reproduces - means solve, so the slow
    // path is always available and is what a disagreement falls back to. See
    // SectionPolarCache.h for why the witness is the check that matters.
    SectionPolarTable table;
    if (!LoadSectionPolarTable(spec, table).hit)
    {
        table = Computed(spec);
        SaveSectionPolarTable(spec, table);
    }
    built.emplace_back(section, std::move(table));
    return built.back().second;
}

double SectionPolarTable::MaximumLiftCoefficient(double brake) const
{
    return InterpolateByBrake(MaximumLiftByBrake, brake);
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

namespace
{
// Below this internal pressure coefficient the section is considered to have
// lost its shape entirely. A cell at trim runs near 0.95, so the working
// range is narrow and the loss is steep once it starts.
constexpr double ShapeHoldingPressureCoefficient = 0.55;

double PressureDeficit(double internalPressureCoefficient)
{
    const double held = std::clamp(
        internalPressureCoefficient / ShapeHoldingPressureCoefficient,
        0.0, 1.0);
    return 1.0 - held;
}
}

double SectionPolarTable::PressureLiftFactor(
    double internalPressureCoefficient)
{
    const double deficit = PressureDeficit(internalPressureCoefficient);
    return std::clamp(1.0 - 0.85 * deficit * deficit, 0.0, 1.0);
}

double SectionPolarTable::PressureDragPenalty(
    double internalPressureCoefficient)
{
    const double deficit = PressureDeficit(internalPressureCoefficient);
    return 0.45 * deficit * deficit;
}

SectionPolarSample SectionPolarTable::SampleAtSeparation(
    double alphaRad, double brake, double separation,
    double internalPressureCoefficient) const
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

    SectionPolarSample sample = mix(
        bilinear(Attached), bilinear(Separated), blend);

    // A cell that has lost its pressure has lost its section.
    sample.liftCoefficient *=
        PressureLiftFactor(internalPressureCoefficient);
    sample.dragCoefficient +=
        PressureDragPenalty(internalPressureCoefficient);
    return sample;
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

SectionPolarSample SectionPolarTable::SampleAtEquilibrium(
    double alphaRad, double brake, double internalPressureCoefficient) const
{
    SectionPolarSample sample = Sample(alphaRad, brake);
    sample.liftCoefficient *=
        PressureLiftFactor(internalPressureCoefficient);
    sample.dragCoefficient +=
        PressureDragPenalty(internalPressureCoefficient);
    return sample;
}

// All three are read off the table's own per-brake summaries. The analytic
// generator fills them from its closed forms and the computed one from the
// solved curve, so a caller cannot tell which it has - which is the point.
double SectionPolarTable::LiftCurveSlopePerRad(double brake) const
{
    if (SlopeByBrake.empty()) return LiftSlope(SpecValue.thicknessFraction);
    return InterpolateByBrake(SlopeByBrake, brake);
}

double SectionPolarTable::ZeroLiftAngleRad(double brake) const
{
    if (ZeroLiftByBrake.empty())
    {
        const double deflection =
            std::clamp(brake, 0.0, 1.0) * SpecValue.fullBrakeDeflectionRad;
        return BaseZeroLiftAngle(SpecValue.camberFraction)
            - ThinAirfoilFlapEffectiveness(SpecValue.flapChordFraction)
                * deflection;
    }
    return InterpolateByBrake(ZeroLiftByBrake, brake);
}

double SectionPolarTable::StallAngleRad(double brake) const
{
    if (StallByBrake.empty())
    {
        return ZeroLiftAngleRad(brake)
            + SpecValue.stallMarginRad
                * (1.0 - SpecValue.stallMarginBrakeLoss
                       * std::clamp(brake, 0.0, 1.0));
    }
    return InterpolateByBrake(StallByBrake, brake);
}
}
