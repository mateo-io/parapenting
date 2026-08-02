#pragma once

#include <cstddef>
#include <vector>

#include "SectionProfile.h"

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
    // Solved on the section's own coordinates by SectionViscousSolver:
    // potential flow by panels, the boundary layer by integral methods, and
    // the stall where the two stop agreeing. Still not measurement - but the
    // stall angle, the maximum lift coefficient and the pitching moment are
    // consequences of the shape rather than numbers in a struct, which is the
    // whole difference between this and Analytic.
    Computed,
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

// What the computed generator needs that the analytic one did not: the
// section's coordinates, and the Reynolds number to solve the boundary layer
// at.
struct ComputedPolarSpec
{
    SectionProfileSpec section{};
    // Chord times airspeed over kinematic viscosity, at trim: 1.4 m of mean
    // chord at 11 m/s in 1.46e-5 m2/s of air. The wing runs 0.5 to 3 x 10^6
    // across its span and its speed range, and the section polars change
    // slowly enough across that to make one value defensible - which is why
    // Reynolds is still not an axis.
    double reynoldsNumber = 1.05e6;
    // Aspect ratio the post-stall flat-plate branch is built for.
    double aspectRatioForPostStall = 5.2;
    // Angular width over which the solve hands off to the post-stall branch,
    // past the incidence where the computed lift peaks. Narrower than the
    // analytic table's 0.10, because there the stall angle was a stated
    // number with a stated margin of error around it and the blend absorbed
    // some of that; here the peak is solved, and a blend wide enough to carry
    // the attached branch several degrees past it would put the equilibrium
    // curve's maximum above the maximum lift the section was found to have.
    double stallBlendWidthRad = 0.06;
    // How far below the stall the flow reattaches. Separation and
    // reattachment do not happen at the same incidence.
    double reattachmentHysteresisRad = 0.09;
    // Incidence range solved on the real contour, either side of zero. Past
    // it the section is fully separated and the post-stall branch is what
    // applies, so solving further buys nothing.
    double sweepLowRad = -0.35;   // -20 deg
    double sweepHighRad = 0.44;   // 25 deg
    std::size_t brakeSamples = 21;
};

// A table over angle of attack and brake deflection. Reynolds number is a
// single declared value rather than an axis: the operating range of this wing
// is 0.5-3 x 10^6, and the section data changes slowly enough across it that
// one value is defensible. The axis is where it belongs when the wing is
// solved span station by span station.
class SectionPolarTable
{
public:
    SectionPolarTable() = default;

    static SectionPolarTable Analytic(const AnalyticPolarSpec& spec = {});

    // Solved on the section's own coordinates. This is what the solver above
    // gets by default; `Analytic` is kept because the two are compared in
    // `aerodynamics_tests`, and because a disagreement between them is the
    // cheapest check that either is right.
    //
    // Building one costs a panel factorisation and an incidence sweep per
    // brake station, so the default spec's table is built once and shared.
    static const SectionPolarTable& Default();
    static SectionPolarTable Computed(const ComputedPolarSpec& spec);

    // The table for a given rib profile, built once and kept. A canopy hands
    // in its own section here, so changing the profile changes the polars the
    // wing flies on without anything else being touched - and building the
    // same wing twice does not solve it twice.
    static const SectionPolarTable& ForSection(const SectionProfileSpec& spec);

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
        double alphaRad, double brake, double separation,
        double internalPressureCoefficient = 1.0) const;

    // The steady sample, with the cell's pressure applied. Uses the table's
    // own baked-in separation blend so a caller with no pressure model gets
    // exactly what Sample() has always returned.
    SectionPolarSample SampleAtEquilibrium(
        double alphaRad, double brake,
        double internalPressureCoefficient) const;

    // How much a section loses when its cell is under-pressurised.
    //
    // A soft cell does not hold the profile it was cut to: the nose goes
    // round, the section loses effective camber and picks up form drag, and
    // at the limit it stops being an aerofoil at all. The plan's proper
    // answer is a polar family generated at reduced pressure, which is part
    // of the data acquisition Level 4 still owes. This is the stand-in for
    // it, and it is deliberately a single documented curve rather than
    // something tuned per-effect: lift falls and drag rises with the square
    // of the pressure deficit.
    static double PressureLiftFactor(double internalPressureCoefficient);
    static double PressureDragPenalty(double internalPressureCoefficient);

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

    // Maximum lift coefficient at a brake setting, and where it happens. On
    // the analytic table these were a restatement of `stallMarginRad`; on the
    // computed one they are read off the solved curve, and they move with
    // brake because the section does.
    double MaximumLiftCoefficient(double brake) const;

    PolarProvenance Provenance() const { return Source; }
    const AnalyticPolarSpec& Spec() const { return SpecValue; }
    const ComputedPolarSpec& ComputedSpec() const { return ComputedValue; }
    std::size_t AlphaSampleCount() const { return AlphaCount; }

private:
    AnalyticPolarSpec SpecValue{};
    ComputedPolarSpec ComputedValue{};
    PolarProvenance Source = PolarProvenance::Analytic;
    std::size_t AlphaCount = 0;
    std::size_t BrakeCount = 0;
    double AlphaMinRad = 0.0;
    double AlphaMaxRad = 0.0;
    // Per-brake summaries. On the analytic table these are closed forms; on
    // the computed one they are measured off the solved curve, which is why
    // they became arrays.
    std::vector<double> ZeroLiftByBrake;
    std::vector<double> SlopeByBrake;
    std::vector<double> StallByBrake;
    std::vector<double> MaximumLiftByBrake;
    double BlendWidthRad = 0.10;
    double ReattachmentRad = 0.09;
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
