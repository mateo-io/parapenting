#pragma once

#include <cstddef>
#include <vector>

namespace Parapenting::Physics
{
// Level 4 of the master plan: 2D section data, tabulated.
//
// The plan asks for XFOIL runs over the digitised EPIC 2 profiles across the
// Reynolds envelope, extended post-stall and swept across brake deflection.
// That is a data-acquisition job and it has not been done. What is here
// instead is an analytic generator built from published theory, producing the
// same table shape so the solver above it never needs to know which it got:
//
//   * pre-stall lift from thin-airfoil theory, with the zero-lift angle of a
//     circular-arc camber line (alpha_L0 = -2 h/c, exact under that theory);
//   * brake as a trailing-edge flap, using the thin-airfoil flap
//     effectiveness tau = 1 - (theta_f - sin theta_f)/pi, which is derived
//     rather than fitted;
//   * post-stall from the Viterna-Corrigan extension (1982), the standard
//     published treatment for wings that operate past stall.
//
// Every one of those is theory, not measurement. The table is marked
// Analytic, the registry records it as unvalidated, and Level 9 is where it
// gets replaced by real polars. Nothing here should be read as EPIC 2 section
// data - it is a placeholder with the right shape and known failure modes,
// which is worth more than a fitted curve with none.

enum class PolarProvenance
{
    // Generated from theory by AnalyticSectionPolar. Not measurement.
    Analytic,
    // Loaded from a viscous 2D solver or wind tunnel run.
    Measured,
};

struct SectionPolarSample
{
    double liftCoefficient = 0.0;
    double dragCoefficient = 0.0;
    // About the quarter chord, nose-up positive.
    double momentCoefficient = 0.0;
};

struct AnalyticPolarSpec
{
    // Section thickness and camber as chord fractions. A paraglider profile
    // is thick and strongly cambered by aircraft standards.
    double thicknessFraction = 0.155;
    double camberFraction = 0.035;
    // Where the brake starts, as a chord fraction. This is what makes brake a
    // camber change rather than an incidence change.
    double flapChordFraction = 0.78;
    // Trailing edge deflection at full brake, radians. With the flap
    // effectiveness above this is what decides where the wing stalls on the
    // brake travel, so it is the natural calibration hook for stall onset and
    // it is registered as such.
    double fullBrakeDeflectionRad = 0.42;
    // Profile drag at the zero-lift angle, and the quadratic drag rise.
    double minimumDragCoefficient = 0.0125;
    double dragRiseFactor = 0.016;
    // Where the section stalls, measured from its own zero-lift angle.
    double stallMarginRad = 0.244;   // 14 deg
    // Angular width of the stall transition, past the stall angle. The blend
    // is exactly zero below stall and exactly one beyond this, so attached
    // flow is genuinely attached.
    //
    // It replaced a logistic blend in the stall margin, which never reached
    // zero: at trim, a section 7.7 deg below its stall angle still carried a
    // fifth of the post-stall branch, and that branch has a NEGATIVE lift
    // slope. Softening the knee mixed in more of it, so the solve became less
    // stable the gentler the stall was made - the opposite of what a stall
    // model should do.
    double stallBlendWidthRad = 0.10;
    // How much a deflected trailing edge brings the stall forward, as a
    // fraction of the clean margin at full brake. A flap raises maximum lift
    // and lowers the angle it happens at; most of that lowering is already
    // carried by the zero-lift shift, so what is left here is small. It was
    // 0.42, which double-counted the shift and stalled the wing under any
    // meaningful brake.
    double stallMarginBrakeLoss = 0.15;
    // Aspect ratio the post-stall flat-plate drag is built for. Viterna's
    // CDmax depends on it.
    double aspectRatioForPostStall = 5.2;
    // How far below the stall angle the flow reattaches, radians. Separation
    // and reattachment do not happen at the same incidence - a stalled
    // section has to be brought well back down before it flies again, which
    // is why a paraglider recovers from a stall later than it entered one.
    double reattachmentHysteresisRad = 0.09;
};

// A table over angle of attack and brake deflection. Reynolds number is a
// single declared value rather than an axis: the operating range of this wing
// is 0.5-3 x 10^6 and the analytic generator has no Reynolds dependence to
// interpolate. The axis is where it belongs when real polars arrive.
class SectionPolarTable
{
public:
    SectionPolarTable() = default;

    static SectionPolarTable Analytic(const AnalyticPolarSpec& spec = {});

    // alphaRad is measured from the chord line, brake runs 0 to 1. Samples
    // the section at its equilibrium separation, which is what a steady solve
    // with no memory wants.
    SectionPolarSample Sample(double alphaRad, double brake) const;

    // Samples with the separation held at a given value instead. This is what
    // makes the solve well posed near stall: with separation an input rather
    // than an instantaneous function of incidence, the lift curve the solver
    // sees is single valued and smooth, and the branch-swapping that stops a
    // deeply braked case ever settling cannot happen.
    SectionPolarSample SampleAtSeparation(
        double alphaRad, double brake, double separation) const;

    // Where separation settles for this incidence, 0 attached to 1 fully
    // separated. Separating and reattaching follow different curves.
    double SeparationEquilibrium(
        double alphaRad, double brake, double currentSeparation) const;

    // Lift-curve slope per radian in the linear range, and the zero-lift
    // angle, both at the given brake setting. Reported rather than assumed so
    // the VSM validation can check them against thin-airfoil theory.
    double LiftCurveSlopePerRad(double brake) const;
    double ZeroLiftAngleRad(double brake) const;
    double StallAngleRad(double brake) const;

    PolarProvenance Provenance() const { return Source; }
    const AnalyticPolarSpec& Spec() const { return SpecValue; }
    std::size_t AlphaSampleCount() const { return AlphaCount; }

private:
    AnalyticPolarSpec SpecValue{};
    PolarProvenance Source = PolarProvenance::Analytic;
    std::size_t AlphaCount = 0;
    std::size_t BrakeCount = 0;
    double AlphaMinRad = 0.0;
    double AlphaMaxRad = 0.0;
    std::vector<SectionPolarSample> Samples;
    // The two branches, kept apart so a caller can blend them by a separation
    // state that has memory rather than by incidence alone.
    std::vector<SectionPolarSample> Attached;
    std::vector<SectionPolarSample> Separated;
    std::vector<double> SeparationCurve;
};

// Thin-airfoil flap effectiveness for a trailing edge hinged at
// flapChordFraction: the fraction of the deflection that appears as an
// effective incidence change. Exposed because the brake model rests on it.
double ThinAirfoilFlapEffectiveness(double flapChordFraction);
}
