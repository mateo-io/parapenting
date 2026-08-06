#include "CoupledParagliderSolver.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double GravityMps2 = 9.80665;

// Adds its lifetime to a counter, or does nothing. `enabled` is a member read
// once per scope; with profiling off the clock is never touched.
class StageTimer
{
public:
    StageTimer(bool enabled, long long& sink)
        : Sink(enabled ? &sink : nullptr),
          Start(enabled ? std::chrono::steady_clock::now()
                        : std::chrono::steady_clock::time_point{})
    {
    }

    ~StageTimer()
    {
        if (Sink == nullptr) return;
        *Sink += std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - Start).count();
    }

    StageTimer(const StageTimer&) = delete;
    StageTimer& operator=(const StageTimer&) = delete;

private:
    long long* Sink;
    std::chrono::steady_clock::time_point Start;
};

std::vector<double> SectionSpanFractions(
    const std::vector<VsmSection>& sections)
{
    std::vector<double> spans;
    spans.reserve(sections.size());
    for (const VsmSection& section : sections)
        spans.push_back(section.spanFraction);
    return spans;
}

Quaternion IntegrateAttitude(
    const Quaternion& q, const Vec3& bodyRate, double dt)
{
    const Vec3 worldRate = q.Rotate(bodyRate);
    const Quaternion omega{0.0, worldRate.x, worldRate.y, worldRate.z};
    const Quaternion derivative = omega * q;
    return Quaternion{
        q.w + 0.5 * dt * derivative.w,
        q.x + 0.5 * dt * derivative.x,
        q.y + 0.5 * dt * derivative.y,
        q.z + 0.5 * dt * derivative.z}.Normalized();
}
}

CoupledParagliderSolver::CoupledParagliderSolver(
    const CanopyGeometry& geometry, const LinePlanSpec& linePlan,
    const CoupledSchedule& schedule, const PayloadMassProperties& payload,
    const ConstructionProbe& probe)
    : ConstructionProbeSettings(probe),
      ScheduleValue(schedule),
      Aerodynamics(geometry,
                   SectionPolarTable::ForSection(geometry.Spec().section),
                   geometry.Spec().cellCount),
      Pressure(geometry.Spec().cellCount),
      Membrane(geometry.CellSpacingM()),
      Lines(BuildSuspensionGraph(geometry, linePlan)),
      Collapse(SectionSpanFractions(Aerodynamics.Sections())),
      Polars(SectionPolarTable::ForSection(geometry.Spec().section)),
      ApparentMass(CanopyApparentMass(geometry)),
      PayloadMass(payload)
{
    // How far a fold at each aerodynamic station has to reach before it is
    // past the line beside it. Measured off the graph that was just built, so
    // moving a line moves the cravat criterion with it.
    SectionLineGapM.reserve(Aerodynamics.Sections().size());
    for (const VsmSection& section : Aerodynamics.Sections())
        SectionLineGapM.push_back(LineFoldGapM(Lines, section.spanFraction));

    // The chord the brake bends, measured where the brake fan lands rather
    // than averaged over a span it does not reach. On this wing the fan takes
    // the trailing edge between 22% and 86% of the half span, where the chord
    // is well above the tip's 0.445 m, so a mean-chord shortcut understates
    // it. The drop the section reports is a fraction of the WHOLE chord, so
    // this is the whole chord and not the part aft of the attachment.
    BrakeSection = geometry.Spec().section;
    double brakeChordSum = 0.0;
    int brakeChordCount = 0;
    for (const SuspensionNode& node : Lines.nodes)
    {
        if (node.kind != SuspensionNodeKind::CanopyAttachment) continue;
        if (node.row != LineRow::Brake) continue;
        brakeChordSum += geometry.StationAt(node.spanFraction).chordM;
        ++brakeChordCount;
    }
    BrakeStationChordM = brakeChordCount > 0
        ? brakeChordSum / static_cast<double>(brakeChordCount) : 0.0;

    CanopyMassKg = 5.1;
    SystemMassKg = PayloadMass.TotalKg() + CanopyMassKg;
    ReferenceAreaM2 = Aerodynamics.ReferenceAreaM2();
    ReferenceSpanM = geometry.DevelopedSpanM();
    PendulumLengthM = SuspensionPendulumLengthM(Lines);
    Harness = Lines.plan.harness;

    SolveTrimLoadDistribution();
    MeasureLineStiffness();
}

CoupledParagliderSolver::LineStiffness
CoupledParagliderSolver::LineStiffnessAt(double loadN) const
{
    if (StiffnessCurve.empty()) return LineStiffness{};
    const double load = std::max(0.0, loadN);
    // Below the lowest sample the spring is scaled down in proportion, because
    // that is what an unloaded geometric spring does: no tension, no restoring
    // moment. Above the highest it is held, which understates a wing at 5 g
    // rather than extrapolating a curve that was never measured there.
    const StiffnessSample& first = StiffnessCurve.front();
    if (load <= first.loadN)
    {
        const double scale = first.loadN > 1.0e-6 ? load / first.loadN : 0.0;
        LineStiffness scaled = first.stiffness;
        scaled.pitchNmPerRad *= scale;
        scaled.rollNmPerRad *= scale;
        return scaled;
    }
    for (std::size_t i = 1; i < StiffnessCurve.size(); ++i)
    {
        const StiffnessSample& lo = StiffnessCurve[i - 1];
        const StiffnessSample& hi = StiffnessCurve[i];
        if (load > hi.loadN) continue;
        const double span = std::max(1.0e-6, hi.loadN - lo.loadN);
        const double t = std::clamp((load - lo.loadN) / span, 0.0, 1.0);
        const auto mix = [t](double a, double b) { return a + (b - a) * t; };
        LineStiffness blended;
        blended.pitchNmPerRad =
            mix(lo.stiffness.pitchNmPerRad, hi.stiffness.pitchNmPerRad);
        blended.rollNmPerRad =
            mix(lo.stiffness.rollNmPerRad, hi.stiffness.rollNmPerRad);
        blended.hangIncidenceRad =
            mix(lo.stiffness.hangIncidenceRad, hi.stiffness.hangIncidenceRad);
        blended.pitchHingeArmM =
            mix(lo.stiffness.pitchHingeArmM, hi.stiffness.pitchHingeArmM);
        blended.rollHingeArmM =
            mix(lo.stiffness.rollHingeArmM, hi.stiffness.rollHingeArmM);
        return blended;
    }
    return StiffnessCurve.back().stiffness;
}

CoupledSchedule FullFidelitySchedule()
{
    return CoupledSchedule{};
}

CoupledSchedule ReducedFidelitySchedule()
{
    CoupledSchedule schedule;
    schedule.suspensionIterations = 40;
    schedule.dampingProbeInterval = 3;
    return schedule;
}

void CoupledParagliderSolver::MeasureLineStiffness()
{
    // What holds a paraglider's wing at its incidence is not a pendulum on the
    // pilot's mass. A mass hanging from a single point has no pitch stiffness
    // at all - it is free to rotate about the attachment - and this solver's
    // lumped "pendulum moment" was standing in for something else entirely:
    // the A, B and C rows attach at different stations along the chord, so
    // rotating the canopy lengthens one row and shortens another and the lines
    // pull it back. That is the real spring, it is in the graph, and it can be
    // measured rather than written down.
    //
    // Rotate the canopy a small angle either side of its free equilibrium with
    // the network solved at each, and take the slope of the moment. Then do it
    // again at four loads, because the answer depends on load and pretending
    // otherwise cost this model its pitch stability - see the header.
    SuspensionSolveInput probe;
    probe.canopyWeightN = CanopyMassKg * GravityMps2;

    SuspensionSolverSettings settings;
    // 4000 iterations at 0.995 velocity retention, and BOTH numbers moved
    // together for one reason: the fictitious dynamics was under-damped, so
    // most of those 12000 iterations were spent ringing rather than
    // converging. Held at 0.02 rad the probe swings +177%, -176%, +27%, -13%
    // of its converged value at 500, 1000, 2000 and 4000 iterations - that is
    // not a solve creeping up on an answer, it is one oscillating about it.
    //
    // Measured against 48000 iterations at the old settings, which is the
    // reference this has to reproduce and does not share a path with:
    //
    //   0.999 (was), 12000 iterations   6246.3 Nm/rad   -0.69%   19.3 ms
    //   0.995,        4000 iterations   6298.8 Nm/rad   +0.15%    6.4 ms
    //   0.995,       12000 iterations   6289.8 Nm/rad    0.00%   19.4 ms
    //   reference: 0.999, 48000         6289.5 Nm/rad             79 ms
    //
    // So the new setting is three times faster AND four times closer to the
    // answer. The equilibrium is not a function of the fictitious damping -
    // only the path to it is - which is what makes this a numerical change
    // rather than a tuning: two different paths land on 6289 to 0.005%.
    //
    // Retention below this is worse, and monotonically: 0.99 needs 12000 to
    // reach the same place, 0.98 is 6.2% out there, 0.95 is 114% out. It is a
    // damping optimum and it is not sharp on the low side of 0.999 only.
    // `cableDampingRatio` barely matters at all - 0.06 to 0.70 moves the
    // answer by 0.2% - because it damps line-axial motion and what rings here
    // is the network's shape.
    //
    // A warm-started in-flight solve still cannot answer this question, which
    // is why it is asked once, here. Warm-starting the probes themselves was
    // tried first and does NOT help: the held solve imposes an attitude 0.02
    // rad away from the free pose, so what a warm start supplies is the answer
    // to a different question, and it converged no faster and less accurately
    // (0.37 N of node residual against 0.011).
    settings.iterations = ConstructionProbeSettings.freeIterations;
    settings.velocityRetention = ConstructionProbeSettings.freeRetention;
    // The held probes get their own, faster settings - see `ConstructionProbe`.
    SuspensionSolverSettings heldSettings = settings;
    heldSettings.iterations = ConstructionProbeSettings.heldIterations;
    heldSettings.velocityRetention = ConstructionProbeSettings.heldRetention;

    constexpr double ProbeAngleRad = 0.02;
    const double weightN = SystemMassKg * GravityMps2;

    const auto measureAt = [&](double loadN)
    {
        SuspensionSolveInput loaded = probe;
        loaded.aeroForceN = Vec3{0.0, 0.0, loadN};
        // The pose the wing hangs at on its own, which is the zero of the
        // spring.
        const SuspensionSolution free =
            SolveSuspension(Lines, loaded, settings);

        // Centred, for the same reason Level 7's damping probe is: a one-sided
        // difference is not odd in the angle, so nose-up and nose-down would
        // measure two different springs on a wing that has one.
        const auto pitchMomentAt = [&](double offset)
        {
            SuspensionSolveInput held = loaded;
            held.holdCanopyAttitude = true;
            held.imposedCanopyAttitude =
                NoseUpAttitude(free.canopyPitchRad + offset);
            return SolveSuspension(Lines, held, heldSettings)
                .canopyMomentBodyNm.y;
        };
        // Sign: a positive right-hand rotation about +Y tips the nose DOWN
        // (SuspensionGraph.h says so, and it is the trap that convention has
        // set twice before), so the nose-down moment answering a nose-up
        // displacement comes back POSITIVE. Measured over +-0.06 rad the curve
        // is straight to within 3% and passes through zero at the free
        // equilibrium, which is what a spring is.
        //
        // Refuse a measurement of the wrong sign rather than feeding
        // excitation back into the pitch axis.
        const double pitchSlope =
            (pitchMomentAt(ProbeAngleRad) - pitchMomentAt(-ProbeAngleRad))
                / (2.0 * ProbeAngleRad);

        // How far the canopy's origin MOVES when it rotates. This wing does
        // not pivot about itself: held 0.02 rad nose-up its origin shifts
        // 0.1325 m, and 0.04 rad shifts it 0.2648 - a constant 6.62 m arm, so
        // the canopy is swinging about a virtual hinge two thirds of the way
        // down its own lines. That is geometry off the built graph, and it is
        // the difference between a wing that rotates and a wing that swings.
        const auto pitchArm = [&](double offset)
        {
            SuspensionSolveInput held = loaded;
            held.holdCanopyAttitude = true;
            held.imposedCanopyAttitude =
                NoseUpAttitude(free.canopyPitchRad + offset);
            const SuspensionSolution s =
                SolveSuspension(Lines, held, heldSettings);
            return Length(s.canopyOriginM - free.canopyOriginM)
                / std::fabs(offset);
        };

        // The same probe in the roll plane, and it is needed for the same
        // reason. With the payload on a free link the canopy has no gravity
        // spring in roll either - a banked wing whose pilot has swung out
        // under it is in equilibrium, which is exactly what a coordinated turn
        // is - so what resists the canopy rolling relative to the pilot is the
        // lines, and it has to be measured rather than left as the W L sin
        // term that used to stand in for it.
        //
        // There is no NoseUp-style flip on this axis: a positive right-hand
        // rotation about +X lifts the right tip, so a restoring moment is
        // NEGATIVE and the slope comes back negative. Negating here keeps
        // every stiffness in this file a positive number.
        const auto rollSolve = [&](double offset)
        {
            SuspensionSolveInput held = loaded;
            held.holdCanopyAttitude = true;
            const Quaternion roll{
                std::cos(0.5 * offset), std::sin(0.5 * offset), 0.0, 0.0};
            held.imposedCanopyAttitude =
                (NoseUpAttitude(free.canopyPitchRad) * roll).Normalized();
            return SolveSuspension(Lines, held, heldSettings);
        };
        const SuspensionSolution rolledUp = rollSolve(ProbeAngleRad);
        const SuspensionSolution rolledDown = rollSolve(-ProbeAngleRad);
        const double rollSlope =
            (rolledUp.canopyMomentBodyNm.x - rolledDown.canopyMomentBodyNm.x)
                / (2.0 * ProbeAngleRad);

        StiffnessSample sample;
        sample.loadN = loadN;
        sample.stiffness.pitchNmPerRad = pitchSlope > 0.0 ? pitchSlope : 0.0;
        sample.stiffness.rollNmPerRad = rollSlope < 0.0 ? -rollSlope : 0.0;
        sample.stiffness.hangIncidenceRad = free.canopyPitchRad;
        sample.stiffness.pitchHingeArmM =
            0.5 * (pitchArm(ProbeAngleRad) + pitchArm(-ProbeAngleRad));
        sample.stiffness.rollHingeArmM = 0.5 * (
            Length(rolledUp.canopyOriginM - free.canopyOriginM)
            + Length(rolledDown.canopyOriginM - free.canopyOriginM))
                / ProbeAngleRad;
        return sample;
    };

    // Half a g to four. Below half a g the wing is unloaded and the spring is
    // scaled down in proportion; above four it is held. A paraglider spends
    // its life between them and a spiral reaches the top of the range.
    StiffnessCurve.clear();
    for (const double g : {0.5, 1.0, 2.0, 4.0})
        StiffnessCurve.push_back(measureAt(g * weightN));

    // The hinge arms are geometry, not load: measured across half a g to four
    // they move by under 3%, so the one-g pair is used throughout rather than
    // interpolated as if it were a load-dependent quantity.
    const LineStiffness atWeight = LineStiffnessAt(weightN);
    PitchHingeArmM = atWeight.pitchHingeArmM;
    RollHingeArmM = atWeight.rollHingeArmM;

    TrimLineIncidenceRad = atWeight.hangIncidenceRad;
    // The swing angle at which the lines are unstressed in pitch. With the
    // canopy nose-up by t, the world-down line direction sits at -t in the
    // canopy's own axes, so the two coordinates are the same one negated.
    TrimSwingRad = -TrimLineIncidenceRad;

    // And the same pose on full bar. This is the entire mechanism of the
    // accelerator and it was doing nothing at all: bar shortens the A and B
    // risers by 120 and 80 mm, which rotates the wing nose-down and drops its
    // incidence, and that is why a wing on bar flies faster and is
    // collapse-prone. The line network has always modelled it - the shortened
    // risers are in its geometry - but the flight model never read the pitch
    // it produced, so pulling full bar changed the airspeed by nothing
    // whatsoever.
    SuspensionSolveInput accelerated = probe;
    accelerated.aeroForceN = Vec3{0.0, 0.0, weightN};
    accelerated.accelerator = 1.0;
    const SuspensionSolution onBar =
        SolveSuspension(Lines, accelerated, settings);
    AcceleratedSwingRad = -onBar.canopyPitchRad;

    // And the same for BRAKE, which had exactly the bug the accelerator had
    // before Level 7 found it, in the other direction.
    //
    // Pulling brake shortens the brake lines, which pulls the trailing edge
    // down, which rotates the whole canopy nose-UP on its suspension. The line
    // network has always modelled that - the shortened rest lengths are in its
    // geometry - and the flight model never read it. What the flight model DID
    // read was the other half of the same input: the section camber change,
    // which a trailing-edge deflection turns into a large nose-DOWN pitching
    // couple. So brake arrived at the wing as a pitching moment with the
    // rotation that answers it missing, and the harder it was pulled the more
    // one-sided that got. Measured: 40% brake pitched the wing down until it
    // was descending at 7.8 m/s in a fully separated stall, which is the
    // opposite of what brake does.
    //
    // Sampled rather than assumed linear, because the brake line has sewn-in
    // slack: the first fifth of the travel moves nothing at all, so a straight
    // line between the endpoints would be wrong exactly where a pilot spends
    // most of their time.
    // The stations are NOT evenly spaced, and that is the whole point. The
    // slack ends at 0.12 m of a 0.62 m travel - 19.4% - and an evenly spaced
    // curve straddles it, so interpolating from hands-up to the first station
    // leaks rotation into travel where no line is pulling. Measured: 15% of
    // travel, which transmits nothing, moved a held collapse from 0.829 to
    // 0.217. Guiding rule 3 says a slack line transmits NOTHING, not a little.
    //
    // So the slack point is a station, and everything below it interpolates
    // between two samples the network puts at zero.
    const double slackFraction = std::clamp(
        Lines.plan.brakeSlackM / std::max(1.0e-6, Lines.plan.brakeTravelM),
        0.0, 0.95);
    std::vector<double> stations{0.0, slackFraction};
    for (int index = 1; index <= 4; ++index)
        stations.push_back(slackFraction
            + (1.0 - slackFraction) * 0.25 * index);

    BrakeSwingCurve.clear();
    for (const double travel : stations)
    {
        SuspensionSolveInput braked = probe;
        braked.aeroForceN = Vec3{0.0, 0.0, weightN};
        braked.leftBrake = travel;
        braked.rightBrake = travel;
        braked.leftBrakeFlapTakeUpM = BrakeFlapTakeUpM(travel);
        braked.rightBrakeFlapTakeUpM = braked.leftBrakeFlapTakeUpM;
        const SuspensionSolution pulled =
            SolveSuspension(Lines, braked, settings);
        // Exactly zero inside the slack, not merely small. The network is flat
        // to six decimal places below the slack point - 0.072436 rad at 0,
        // 0.05, 0.10 and 0.15 of travel - and returns 0.072710 AT it, so the
        // 2.7e-4 difference is relaxation noise rather than the wing moving.
        // Passing it through is not conservative: a collapse is a threshold
        // process, and a hundredth of a degree of incidence was worth 0.83
        // against 0.30 of fold on a wing already at the edge of one.
        BrakeSwingCurve.push_back({travel,
            travel <= slackFraction
                ? 0.0 : -pulled.canopyPitchRad - TrimSwingRad});
    }
}

double CoupledParagliderSolver::EngagedBrake(double handTravel) const
{
    // What the hands do to the trailing edge, which is not what the hands do.
    // There is slack sewn into the brake line at hands-up - 120 mm of a 620 mm
    // travel on this wing - and a slack line transmits nothing (guiding rule
    // 3). The line network has always known this, because the slack is in its
    // rest lengths; the aerodynamics did not, and took the handle position as
    // a camber change directly, so the first fifth of the travel deflected a
    // trailing edge that no line was pulling on.
    const double travelM = Lines.plan.brakeTravelM;
    const double slackM = Lines.plan.brakeSlackM;
    return std::clamp(
        (std::clamp(handTravel, 0.0, 1.0) * travelM - slackM)
            / std::max(1.0e-6, travelM - slackM),
        0.0, 1.0);
}

double CoupledParagliderSolver::BrakeFlapTakeUpM(double handTravel) const
{
    // The deflection the section is being flown at, turned back into the
    // length of brake line it cost. This is the half of the pull the canopy
    // does NOT rotate with, and leaving it out was worth about a factor of two
    // in brake authority: the fabric bent for free and the lines rotated the
    // whole canopy as if it had not.
    return BrakeStationChordM * SectionBrakeTrailingEdgeDrop(
        BrakeSection, EngagedBrake(handTravel));
}

double CoupledParagliderSolver::BrakeSwingOffsetRad(double travel) const
{
    if (BrakeSwingCurve.empty()) return 0.0;
    const double wanted = std::clamp(travel, 0.0, 1.0);
    if (wanted <= BrakeSwingCurve.front().travel)
        return BrakeSwingCurve.front().offsetRad;
    for (std::size_t i = 1; i < BrakeSwingCurve.size(); ++i)
    {
        const BrakeSwingSample& lo = BrakeSwingCurve[i - 1];
        const BrakeSwingSample& hi = BrakeSwingCurve[i];
        if (wanted > hi.travel) continue;
        const double span = std::max(1.0e-9, hi.travel - lo.travel);
        const double fraction = (wanted - lo.travel) / span;
        return lo.offsetRad + (hi.offsetRad - lo.offsetRad) * fraction;
    }
    return BrakeSwingCurve.back().offsetRad;
}

void CoupledParagliderSolver::SolveTrimLoadDistribution()
{
    // The wing in clean hands-up flight, at whatever speed carries its own
    // weight. Two solves: one at a starting guess, then one at the speed that
    // first solve says balances the weight. The speed only has to be close -
    // what is wanted is the SHAPE of the span loading, and that is set by the
    // planform and the arc rather than by the airspeed.
    constexpr double DensityKgM3 = 1.12;
    double airspeed = 11.0;
    VsmSettings settings;
    settings.maxIterations = 600;
    VsmSolution solved;
    for (int pass = 0; pass < 2; ++pass)
    {
        VsmSolveInput trim;
        trim.airspeedBodyMps = Vec3{airspeed, 0.0, 0.0};
        trim.airDensityKgM3 = DensityKgM3;
        solved = Aerodynamics.Solve(trim, settings);
        const double lift = solved.forceBodyN.z;
        if (lift > 1.0) airspeed *= std::sqrt(
            SystemMassKg * GravityMps2 / lift);
        airspeed = std::clamp(airspeed, 5.0, 25.0);
    }

    SectionTrimLoadN.assign(Aerodynamics.Sections().size(), 0.0);
    for (std::size_t i = 0;
         i < Aerodynamics.Sections().size() && i < solved.sections.size(); ++i)
    {
        SectionTrimLoadN[i] = std::fabs(Dot(
            solved.sections[i].forceBodyN, Aerodynamics.Sections()[i].normal));
    }
}

void CoupledParagliderSolver::Step(
    CoupledState& state, const CoupledControls& controls,
    const CoupledAtmosphere& atmosphere)
{
    const double dt = ScheduleValue.timeStepS;
    CoupledDiagnostics diagnostics;
    const StageTimer stepTimer(Profiling, ProfileValue.totalNs);
    if (Profiling) ++ProfileValue.steps;

    // What the hands do to the trailing edge, which is not what the hands do.
    // See EngagedBrake: the sewn-in slack comes off first, and the SAME
    // engaged fraction drives the section's camber and the line the fabric
    // bends with, so brake reaches the wing once.
    const double leftBrakeAtWing = EngagedBrake(controls.leftBrake);
    const double rightBrakeAtWing = EngagedBrake(controls.rightBrake);

    // A simulation that starts mid-flight starts with an inflated canopy.
    // Leaving the cells packed while the wing is already doing 10 m/s made
    // the pressure model report Cp near zero, which the section polars
    // correctly turned into 15% of the lift and a large drag penalty - so the
    // wing stalled on its first aerodynamic re-solve and never recovered. The
    // solvers were all right; the initial condition was not a wing.
    if (!state.initialised) Pressure.SeedInflated(state.pressure);

    // -- 1. controls -------------------------------------------------------
    // Nothing to integrate: brake and bar are rest-length changes, and they
    // reach the wing through the line network in step 6 and through the
    // section camber in step 3. Neither commands a moment (guiding rules 4
    // and 5), which is why there is no code here.

    // -- 2. atmosphere -----------------------------------------------------
    const Vec3 airVelocityWorld = state.velocityWorldMps - atmosphere.windWorldMps;
    // NOT adjusted for the canopy swinging on its virtual hinge, and that is a
    // decision rather than an omission. The canopy really does travel through
    // an arc when it rotates against the lines, and the sections really do
    // meet that air - but adding the arc velocity to the relative wind while
    // the moments are still summed about the canopy's ORIGIN is not a
    // half-measure, it is the wrong sign. Measured: the extra forward speed
    // raises dynamic pressure and lowers incidence, both of which increase the
    // nose-down moment that produced the rotation, so the term acts as
    // NEGATIVE damping and the wing left the envelope inside twenty seconds at
    // 250 m/s.
    //
    // Getting it right means summing moments about the hinge instead, and
    // there the aerodynamic force's arm is cancelled by the line tension's
    // through the canopy's own force balance - so the swing is not, in the
    // end, strongly damped by the air at all. What the arc DOES change is the
    // inertia, which is applied below and is what this measurement was for.
    const Vec3 airspeedBody = state.attitude.InverseRotate(airVelocityWorld);
    const double airspeed = Length(airspeedBody);
    const double dynamicPressure =
        0.5 * atmosphere.densityKgM3 * airspeed * airspeed;
    diagnostics.airspeedMps = airspeed;
    diagnostics.angleOfAttackRad = airspeed > 1.0e-6
        ? std::atan2(-airspeedBody.z, airspeedBody.x) : 0.0;

    // The staggered loop. Aero, then the structure it loads, then back again
    // with the load relaxed - which is what keeps a light structure in dense
    // flow from oscillating itself apart.
    Vec3 exchangedForceBody = state.heldAeroForceBodyN;
    Vec3 exchangedMomentBody = state.heldAeroMomentBodyNm;
    const bool solveAerodynamics =
        !state.initialised
        || state.stepsSinceAerodynamics >= ScheduleValue.aerodynamicsInterval;

    std::vector<double>& cellPressureCoefficient = state.heldPressureCoefficient;
    SuspensionSolution lineSolution;
    double previousExchangedMagnitude = Length(exchangedForceBody);

    Vec3 aeroTargetForce = exchangedForceBody;
    Vec3 aeroTargetMoment = exchangedMomentBody;

    // -- 3, 4, 5. aerodynamics, pressure, membrane -------------------------
    // These run once per aerodynamic interval, not once per coupling
    // iteration. What the staggered loop below iterates is the exchange
    // between that load and the structure it acts on, which is where the
    // added-mass instability lives; re-solving the whole aerodynamic side
    // three times a step would cost three times as much to relax the same
    // number.
    {
        if (solveAerodynamics)
        {
            VsmSolveInput aero;
            aero.airspeedBodyMps = airspeedBody;
            aero.angularVelocityBodyRadps = state.angularVelocityBodyRadps;
            aero.airDensityKgM3 = atmosphere.densityKgM3;
            aero.leftBrake = leftBrakeAtWing;
            aero.rightBrake = rightBrakeAtWing;
            // The pressure the sections actually fly on. A folded cell is not
            // a cell that lost some pressure - it is skin lying against skin
            // with nothing inside it - so the collapse state takes its cell's
            // pressure out on the way to the aerodynamics. This is the whole
            // feedback path from Level 8 to Level 4, and it needs no new
            // aerodynamic term: the section polars already turn a cell with no
            // pressure into lost lift and a drag penalty, which is what makes
            // a fold cost lift where it is rather than everywhere.
            std::vector<double> flyingPressure = cellPressureCoefficient;
            for (std::size_t i = 0; i < flyingPressure.size(); ++i)
            {
                const double folded = i < state.collapse.sections.size()
                    ? state.collapse.sections[i].collapse : 0.0;
                flyingPressure[i] *= std::clamp(1.0 - folded, 0.0, 1.0);
            }
            aero.internalPressureCoefficient = flyingPressure;

            // The gust, in the wing's own axes, at the sections it covers.
            if (Length(atmosphere.gustWorldMps) > 1.0e-9)
            {
                const Vec3 gustBody =
                    state.attitude.InverseRotate(atmosphere.gustWorldMps);
                const double from = std::min(
                    atmosphere.gustSpanFrom, atmosphere.gustSpanTo);
                const double to = std::max(
                    atmosphere.gustSpanFrom, atmosphere.gustSpanTo);
                aero.sectionGustBodyMps.assign(
                    Aerodynamics.Sections().size(), Vec3{});
                for (std::size_t i = 0; i < Aerodynamics.Sections().size(); ++i)
                {
                    const double span =
                        Aerodynamics.Sections()[i].spanFraction;
                    if (span >= from && span <= to)
                        aero.sectionGustBodyMps[i] = gustBody;
                }
            }

            VsmSettings settings;
            // Warm-started by the separation state, so a handful of
            // iterations is enough once the first solve has landed.
            settings.maxIterations = state.initialised ? 40 : 600;
            // Zero unless a test set it, and it is carried into the frozen and
            // still probes below with the rest of the settings - a damping
            // derivative measured on a wing with different drag from the one
            // flying would be a difference between two wings.
            settings.sectionDragOffset = SectionDragOffsetValue;
            if (Profiling) ++ProfileValue.aeroTicks;
            const VsmSolution solved = [&]
            {
                const StageTimer t(Profiling, ProfileValue.vsmUnsteadyNs);
                return Aerodynamics.SolveUnsteady(
                    aero, state.separation, dt, settings);
            }();

            // Reject a solve that has not converged to anything usable. Deep
            // stall genuinely has no steady state - the separated branch has a
            // negative lift slope, which inverts the downwash feedback between
            // sections - so the VSM will not settle there and must not be
            // allowed to hand a diverged load to the structure. Holding the
            // previous load is wrong, but it is bounded and it is reported.
            const auto allFinite = [](const Vec3& v)
            {
                return std::isfinite(v.x) && std::isfinite(v.y)
                    && std::isfinite(v.z);
            };
            // The moment needs its own bound, and did not have one. A single
            // step near stall returned a converged solve carrying 34 kNm of
            // yaw - against a q S b of 14 kNm, so a moment coefficient of
            // nearly 2.5 - which was accepted, and every step after it was
            // rejected and held that number. Ten seconds of a held 26 kNm on
            // an inertia of 150 is a turn rate of 100 rad/s, which is where
            // the asymmetric-brake NaN came from.
            const double momentScaleNm = std::max(
                1.0, dynamicPressure * ReferenceAreaM2 * ReferenceSpanM);
            const bool usable =
                allFinite(solved.forceBodyN)
                && allFinite(solved.momentBodyNm)
                && Length(solved.forceBodyN) < 50.0 * SystemMassKg * GravityMps2
                && Length(solved.momentBodyNm) < 2.0 * momentScaleNm;
            if (!usable)
            {
                diagnostics.aerodynamicsRejected = true;
                diagnostics.vsmResidual = solved.residual;
                diagnostics.vsmConverged = false;
                diagnostics.meanPressureCoefficient =
                    state.heldPressureCoefficientMean;
                goto structureSolve;
            }

            aeroTargetForce = solved.forceBodyN;

            // Measure how much of the moment is the wing's rotation being
            // damped, by solving again with the rotation removed. The
            // difference over the rate is a damping derivative that can be
            // applied at the live rate every step while the rest is held.
            // The probes ask about the wing the unsteady solve just found, so
            // they hold its separation state and continue its circulation
            // rather than starting cold at the equilibrium separation for
            // whatever incidence they land on. Cold, capped at 40 iterations
            // where a cold solve needs about ninety, they were not converged
            // and not even the same wing - which is why the coefficient they
            // returned moved 10% between one interval and the next, and why
            // two mirror-image flights measured different damping.
            VsmSolveInput still = aero;
            still.angularVelocityBodyRadps = Vec3{};
            VsmSettings stillSettings = settings;
            stillSettings.maxIterations =
                std::max(1, ScheduleValue.frozenSolveIterations);
            const VsmSolution stationary = [&]
            {
                const StageTimer t(Profiling, ProfileValue.vsmStationaryNs);
                return Aerodynamics.SolveFrozen(
                    still, state.separation, state.stationaryCirculation,
                    stillSettings);
            }();

            aeroTargetMoment = stationary.momentBodyNm;
            const bool probeUsable = allFinite(stationary.momentBodyNm)
                && Length(stationary.momentBodyNm) < 2.0 * momentScaleNm;
            if (!probeUsable)
            {
                // The rotation-free probe is a second solve and can fail on
                // its own. Falling back to the previous derivative keeps the
                // damping bounded rather than letting a NaN through it.
                diagnostics.aerodynamicsRejected = true;
                aeroTargetMoment = exchangedMomentBody;
            }

            // The damping derivative, measured against a rate the solver
            // chooses rather than the one the wing happens to have.
            //
            // Dividing the live moment difference by the live rate is an
            // ill-posed estimator: near zero rate it is noise over nothing, and
            // the guard that suppressed it below 1e-3 rad/s made the whole
            // damping law discontinuous in the state. Two mirror-image flights
            // took different branches of that guard within four seconds and
            // stopped being mirror images, and the coefficient itself swung
            // between -2.3e4 and +2.5e3 Nm per rad/s between one aerodynamic
            // interval and the next.
            //
            // A fixed perturbation removes both problems. One axis is probed
            // per aerodynamic interval - each axis refreshed every 0.3 s, far
            // slower than the coefficient changes with airspeed - at a rate
            // typical of a real turn, so the difference is always well above
            // the solve's own noise and never divided by a small number.
            //
            // The probe is centred, +p and -p rather than +p against zero.
            // A one-sided probe is not odd in the rate, so the same wing
            // braked left and braked right measures two different damping
            // coefficients, and two flights that should be mirror images of
            // each other stop being so in the fourth second.
            const int probeInterval =
                std::max(1, ScheduleValue.dampingProbeInterval);
            if (probeUsable && state.aeroTicksSinceDampingProbe + 1
                                   >= probeInterval)
            {
                state.aeroTicksSinceDampingProbe = 0;
                constexpr double ProbeRateRadps = 0.3;
                const int axis = state.dampingProbeAxis % 3;
                const auto component = [](const Vec3& v, int which)
                {
                    return which == 0 ? v.x : which == 1 ? v.y : v.z;
                };
                const auto setComponent = [](Vec3& v, int which, double value)
                {
                    if (which == 0) v.x = value;
                    else if (which == 1) v.y = value;
                    else v.z = value;
                };
                const auto probeAt = [&](double rate,
                                         std::vector<double>& warm)
                {
                    const StageTimer t(
                        Profiling, ProfileValue.vsmDampingProbeNs);
                    VsmSolveInput probe = still;
                    Vec3 probeRate{};
                    setComponent(probeRate, axis, rate);
                    probe.angularVelocityBodyRadps = probeRate;
                    return Aerodynamics.SolveFrozen(
                        probe, state.separation, warm, stillSettings);
                };
                const VsmSolution forward =
                    probeAt(ProbeRateRadps, state.forwardProbeCirculation);
                const VsmSolution backward =
                    probeAt(-ProbeRateRadps, state.backwardProbeCirculation);
                if (allFinite(forward.momentBodyNm)
                    && allFinite(backward.momentBodyNm)
                    && Length(forward.momentBodyNm) < 2.0 * momentScaleNm
                    && Length(backward.momentBodyNm) < 2.0 * momentScaleNm)
                {
                    const double delta = component(forward.momentBodyNm, axis)
                        - component(backward.momentBodyNm, axis);
                    setComponent(state.rotationalDampingNmPerRadps, axis,
                                 delta / (2.0 * ProbeRateRadps));
                }
                state.dampingProbeAxis = (axis + 1) % 3;
            }
            else
            {
                ++state.aeroTicksSinceDampingProbe;
            }

            diagnostics.vsmResidual = solved.residual;
            diagnostics.vsmConverged = solved.converged;
            diagnostics.aerodynamicsSolvedThisStep = true;

            // -- 4. pressure ----------------------------------------------
            CellPressureInput cells;
            cells.dynamicPressurePa.resize(solved.sections.size());
            cells.angleOfAttackRad.resize(solved.sections.size());
            for (std::size_t i = 0; i < solved.sections.size(); ++i)
            {
                cells.dynamicPressurePa[i] = dynamicPressure;
                cells.angleOfAttackRad[i] = solved.sections[i].angleOfAttackRad;
            }
            const CellPressureResult pressureResult = [&]
            {
                const StageTimer t(Profiling, ProfileValue.pressureNs);
                return Pressure.Step(state.pressure, cells, dt);
            }();
            cellPressureCoefficient = pressureResult.pressureCoefficient;
            state.heldPressureCoefficientMean =
                pressureResult.meanPressureCoefficient;
            diagnostics.meanPressureCoefficient =
                pressureResult.meanPressureCoefficient;

            // -- 5. membrane ----------------------------------------------
            // One representative station stands for the chord. The skin's own
            // dynamics are far faster than the flight's, so what matters here
            // is the shape it settles to under the pressure just solved.
            MembraneLoad skin;
            const std::vector<double>& gauge = pressureResult.gaugePressurePa;
            const double medianGaugePa = gauge.empty()
                ? 0.0 : gauge[gauge.size() / 2];
            skin.internalPressurePa = medianGaugePa;
            skin.brakeLineForceN = 40.0
                * (leftBrakeAtWing + rightBrakeAtWing);
            const MembraneResult skinResult = [&]
            {
                const StageTimer t(Profiling, ProfileValue.membraneNs);
                return Membrane.Step(skin, dt);
            }();
            diagnostics.membraneStrain = skinResult.maximumStrain;

            // A second station, at whichever cell is worst fed. The skin's
            // shape is a function of the pressure holding it out, and the
            // representative station above is by construction not the one that
            // is folding - so a wing whose tip has emptied would report the
            // taut mid-span skin as the shape of the whole canopy. Two solves
            // bracket the wing and each section is placed between them by its
            // own gauge pressure, which is a stated interpolation rather than
            // forty membrane solves per aerodynamic interval.
            const double lowestGaugePa = gauge.empty()
                ? 0.0 : *std::min_element(gauge.begin(), gauge.end());
            MembraneResult worstSkin = skinResult;
            if (lowestGaugePa < medianGaugePa - 1.0)
            {
                MembraneLoad worst = skin;
                worst.internalPressurePa = lowestGaugePa;
                const StageTimer t(Profiling, ProfileValue.membraneNs);
                worstSkin = Membrane.Step(worst, dt);
            }

            // -- 6. collapse ----------------------------------------------
            // Everything the pressure balance needs was solved above. None of
            // it is written down here a second time.
            state.collapseInput.assign(
                solved.sections.size(), SectionCollapseInput{});
            for (std::size_t i = 0; i < solved.sections.size(); ++i)
            {
                const VsmSection& geometrySection = Aerodynamics.Sections()[i];
                const VsmSectionResult& aeroSection = solved.sections[i];
                SectionCollapseInput& fold = state.collapseInput[i];

                fold.internalPressureCoefficient =
                    i < cellPressureCoefficient.size()
                        ? cellPressureCoefficient[i] : 1.0;
                fold.angleOfAttackRad = aeroSection.angleOfAttackRad;
                fold.separation = aeroSection.separation;
                // A panel straddling the centreline is braked partly by each
                // hand, which is the same weighting the aerodynamics uses.
                const double right = std::clamp(
                    geometrySection.rightSideFraction, 0.0, 1.0);
                fold.brake = rightBrakeAtWing * right
                    + leftBrakeAtWing * (1.0 - right);
                // Already the engaged brake rather than the handle's
                // travel, so a pump inside the sewn-in slack does nothing to
                // a fold - which is the plan's exit gate for brake pumping.
                fold.zeroLiftAngleRad = Polars.ZeroLiftAngleRad(fold.brake);

                // How much load this section is carrying, against what the
                // same section carries in clean trim. Lines carry what the
                // fabric hands them, so a section making no lift has slack
                // lines under it - which is the mechanism behind a turbulence
                // collapse, and it is measured here rather than asserted.
                const double trimLoadN = i < SectionTrimLoadN.size()
                    ? SectionTrimLoadN[i] : 0.0;
                fold.loadFraction = trimLoadN > 1.0e-6
                    ? std::clamp(
                          Dot(aeroSection.forceBodyN, geometrySection.normal)
                              / trimLoadN,
                          0.0, 1.0)
                    : 1.0;

                // The skin at this section, between the two membrane solves.
                const double gaugeHere = i < gauge.size()
                    ? gauge[i] : medianGaugePa;
                const double span = medianGaugePa - lowestGaugePa;
                const double toWorst = span > 1.0
                    ? std::clamp((medianGaugePa - gaugeHere) / span, 0.0, 1.0)
                    : 0.0;
                fold.skinSlackFraction = skinResult.slackFraction
                    + (worstSkin.slackFraction - skinResult.slackFraction)
                        * toWorst;
                fold.foldDepthM = skinResult.foldDepthM
                    + (worstSkin.foldDepthM - skinResult.foldDepthM) * toWorst;
                fold.lineGapM = i < SectionLineGapM.size()
                    ? SectionLineGapM[i] : 1.0e9;
            }
        }
    }

structureSolve:
    // The pressure coefficient is only recomputed when the aerodynamics run,
    // so it is carried rather than reported as zero on the steps between.
    if (!diagnostics.aerodynamicsSolvedThisStep)
        diagnostics.meanPressureCoefficient =
            state.heldPressureCoefficientMean;

    // The collapse runs every physics step, not once per aerodynamic interval.
    // A nose folds in about a tenth of a second and the aerodynamic interval
    // is a tenth of a second, so a collapse solved at 10 Hz would be one step
    // wide - the fold rate is a real time constant and it has to be resolved.
    // Its inputs are held between aerodynamic solves, like the loads are.
    if (!state.collapseInput.empty())
    {
        const StageTimer t(Profiling, ProfileValue.collapseNs);
        diagnostics.collapseState =
            Collapse.Step(state.collapse, state.collapseInput, dt);
    }
    // A half wing that has folded is not carrying its share. Positive means
    // the right half carries more, so a folded left half is positive. There is
    // no coefficient here: a fully folded half carries nothing, which is an
    // asymmetry of one.
    diagnostics.collapseLoadAsymmetry = std::clamp(
        diagnostics.collapseState.leftCollapse
            - diagnostics.collapseState.rightCollapse,
        -1.0, 1.0);

    const int couplingIterations = std::max(1, ScheduleValue.couplingIterations);
    for (int coupling = 0; coupling < couplingIterations; ++coupling)
    {
        // Relax the load handed to the structure rather than applying it
        // whole. This is the damping the ram-air FSI literature calls for:
        // without it a light structure in dense flow drives itself unstable
        // through added mass.
        const double relax = std::clamp(
            ScheduleValue.couplingRelaxation, 0.05, 1.0);
        exchangedForceBody = exchangedForceBody
            + (aeroTargetForce - exchangedForceBody) * relax;
        exchangedMomentBody = exchangedMomentBody
            + (aeroTargetMoment - exchangedMomentBody) * relax;

        // -- 6. lines -------------------------------------------------------
        SuspensionSolveInput suspension;
        // The aerodynamic resultant reaches the lines in the payload frame,
        // which is where the network is solved.
        suspension.aeroForceN = state.attitude.Rotate(exchangedForceBody);
        suspension.canopyWeightN = CanopyMassKg * GravityMps2;
        suspension.accelerator = controls.accelerator;
        suspension.leftBrake = controls.leftBrake;
        suspension.rightBrake = controls.rightBrake;
        suspension.leftBrakeFlapTakeUpM = BrakeFlapTakeUpM(controls.leftBrake);
        suspension.rightBrakeFlapTakeUpM =
            BrakeFlapTakeUpM(controls.rightBrake);
        suspension.weightShift = controls.weightShift;
        suspension.spanwiseLoadAsymmetry = diagnostics.collapseLoadAsymmetry;

        SuspensionSolverSettings lineSettings;
        lineSettings.iterations = state.initialised
            ? std::max(1, ScheduleValue.suspensionIterations) : 12000;
        {
            const StageTimer t(Profiling, ProfileValue.suspensionNs);
            lineSolution = SolveSuspension(
                Lines, suspension, lineSettings, &state.suspension);
        }
        diagnostics.suspensionResidualN = lineSolution.canopyForceResidualN;

        const double magnitude = Length(exchangedForceBody);
        diagnostics.couplingResidual = previousExchangedMagnitude > 1.0e-9
            ? std::fabs(magnitude - previousExchangedMagnitude)
                / previousExchangedMagnitude
            : 0.0;
        previousExchangedMagnitude = magnitude;
        diagnostics.couplingIterationsUsed = coupling + 1;
    }

    state.stepsSinceAerodynamics =
        solveAerodynamics ? 1 : state.stepsSinceAerodynamics + 1;
    state.heldAeroForceBodyN = exchangedForceBody;
    state.heldAeroMomentBodyNm = exchangedMomentBody;

    // -- 7. rigid motion ---------------------------------------------------
    // Runs to the end of the step, so this timer's scope is the rest of the
    // function and it is deliberately declared last.
    const StageTimer motionTimer(Profiling, ProfileValue.rigidMotionNs);

    // The payload, on its own pendulum under the carabiners.
    PayloadInput payloadInput;
    payloadInput.weightShift = controls.weightShift;
    payloadInput.suspendedLoadN =
        std::max(1.0, PayloadMass.TotalKg() * GravityMps2);
    payloadInput.loadFactor = 1.0;
    const PayloadLoads payloadLoads = StepPayload(
        state.payload, PayloadMass, Harness, payloadInput, dt);
    diagnostics.leftCarabinerLoadN = payloadLoads.leftCarabinerN;
    diagnostics.rightCarabinerLoadN = payloadLoads.rightCarabinerN;

    // Everything hanging below the wing. Lines, risers, harness and pilot are
    // 47% of the canopy's own drag on this aircraft, so leaving them out of
    // the coupled solve makes it glide at 14 where the wing glides at 9.5.
    const InstalledDragResult installed = EvaluateInstalledDrag(
        InstalledDrag, airspeedBody, atmosphere.densityKgM3);

    // THE PILOT'S OWN AIRFLOW, which is not the aircraft's when the pilot is
    // swinging. Item 11's estimate of the swing damping it has been replacing
    // with a coefficient is "the pilot's drag on an 8 m arm, plus the lines
    // sweeping", and neither term was ever in this solver: the harness drag
    // below is built from the AIRCRAFT's airspeed, so it is a force along the
    // flight path that cannot oppose a swing, and the lines' drag becomes a
    // moment on the canopy that never reaches the bob. Nothing here was
    // proportional to the swing rate except `swingDampingRatio` itself.
    // `PHYSICS_LEARNINGS` §59.
    //
    // The bob's velocity through the air is the aircraft's plus its own
    // rotation about the hinge, w x r, with r the arm from the canopy down to
    // the pilot. That is geometry this solver already has, not a new number.
    //
    // WHY THE ARGUMENT AT `airVelocityWorld` DOES NOT COVER THIS, since it ends
    // with "the swing is not, in the end, strongly damped by the air at all":
    // that paragraph is about the CANOPY's arc velocity feeding the sections'
    // relative wind, and the cancellation it describes is between the
    // aerodynamic force's arm and the line tension's, about the hinge. Line
    // tension acts ALONG the line and therefore has no moment about the hinge
    // to cancel anything on the pilot's side. A bluff body at the end of the
    // arm, moving through air, makes an uncancelled damping torque. The two
    // claims are about different bodies and this one does not overturn that one.
    const Vec3 pilotAirVelocityWorld = [&]
    {
        if (HarnessDragReferenceValue == HarnessDragReference::Aircraft
            || !state.initialised)
            return airVelocityWorld;
        const Vec3 arm = Normalized(state.payloadDirWorld) * PendulumLengthM;
        return airVelocityWorld + Cross(state.linkRateWorldRadps, arm);
    }();
    const double pilotAirspeed = Length(pilotAirVelocityWorld);
    const bool pilotReferenced =
        HarnessDragReferenceValue == HarnessDragReference::Pilot
        && state.initialised;

    // The harness force is computed ONCE, from whichever airflow is in force,
    // and used both here in the system's force balance and below in the
    // pendulum's tangential equation. Computing it twice from two different
    // airflows is how the same force ends up acting with two magnitudes, which
    // is the mistake the note below this one records having already made.
    //
    // Drag is 0.5 rho Cd A |v| v, and `harnessDragN` is that evaluated at the
    // AIRCRAFT's dynamic pressure - so referred to the pilot's own airflow the
    // magnitude scales by (pilotAirspeed / airspeed)^2 and the direction is the
    // pilot's. Written as one product to keep the |v| v shape visible.
    const Vec3 harnessDragWorldForce =
        (pilotReferenced && airspeed > 1.0e-6)
            ? pilotAirVelocityWorld
                * (-installed.harnessDragN * pilotAirspeed
                   / (airspeed * airspeed))
            : Vec3{};

    // The default path is the original statement, character for character. A
    // reference that defaults to off has to be bit-identical when it is off,
    // and this one is a rotation round-trip away from not being - see the note
    // in `EvaluateInstalledDrag` about the coupled check that caught the last
    // one of these.
    const Vec3 installedDragBody = pilotReferenced
        ? (airspeed > 1.0e-6
               ? airspeedBody * (-installed.lineDragN / airspeed) : Vec3{})
            + state.attitude.InverseRotate(harnessDragWorldForce)
        : (airspeed > 1.0e-6
               ? airspeedBody * (-installed.totalDragN / airspeed) : Vec3{});
    exchangedForceBody += installedDragBody;
    // Only the LINES' drag moment. The harness drag acts on the pilot, eight
    // metres below, and the path by which it pitches the wing is the pendulum
    // and the line spring - which now exist, and which apply it below. Adding
    // it here as well pitched the wing with the same force twice.
    exchangedMomentBody += installed.lineMomentBodyNm;

    // Forces on the system. The line network is internal - it appears once on
    // the canopy and once on the payload, and cancels - so what accelerates
    // the system is the aerodynamic force and gravity, and nothing else.
    const Vec3 aeroWorld = state.attitude.Rotate(exchangedForceBody);
    const Vec3 weightWorld{0.0, 0.0, -SystemMassKg * GravityMps2};
    const Vec3 netForceWorld = aeroWorld + weightWorld;

    diagnostics.aeroForceBodyN = exchangedForceBody;
    diagnostics.lineForceBodyN = state.attitude.InverseRotate(
        lineSolution.leftCarabinerForceN + lineSolution.rightCarabinerForceN);
    diagnostics.weightForceWorldN = weightWorld;
    diagnostics.netForceWorldN = netForceWorld;

    // Internal closure. Every line tension acts on two things, so summing all
    // of them over both ends must give zero. The suspension solver measures
    // this itself; carrying it here is what makes it a system property rather
    // than one solver's private check.
    diagnostics.internalForceClosureN = Length(lineSolution.endpointForceSumN);
    diagnostics.internalMomentClosureNm = lineSolution.canopyMomentResidualNm;

    // Apparent mass. The air the canopy accelerates with it is a third of the
    // aircraft, so it belongs in the denominator - leaving it out overstates
    // every acceleration the wing makes.
    const Vec3 effectiveMass{
        SystemMassKg + ApparentMass.massKg.x,
        SystemMassKg + ApparentMass.massKg.y,
        SystemMassKg + ApparentMass.massKg.z};
    const Vec3 netForceBody = state.attitude.InverseRotate(netForceWorld);
    const Vec3 accelerationBody{
        netForceBody.x / effectiveMass.x,
        netForceBody.y / effectiveMass.y,
        netForceBody.z / effectiveMass.z};
    const Vec3 accelerationWorld = state.attitude.Rotate(accelerationBody);

    // The pendulum's own energy is NOT in these books, and that is a stated
    // limitation rather than an oversight. It was tried: the pilot swinging
    // fore and aft carries real kinetic energy - 900 J at the top of a surge -
    // and its height under the canopy is real potential energy, so both belong
    // in an energy audit of a paraglider.
    //
    // They do not belong in an energy audit of THIS model, because the rigid
    // motion below still integrates one lumped body with all its mass at the
    // canopy. The payload's height changes in the bookkeeping and not in the
    // dynamics, so adding the term makes the books disagree with the solver
    // they are auditing - measured, and it took hands-off trim from 4 W of
    // residual to 19. The energy that the swing carries therefore shows up as
    // residual during a pitch transient, 154 W at the peak of a surge, and
    // that is reported by the Level 9 manoeuvres as a known gap.
    //
    // It closes with PHYSICS_TODO item 10, which is the same rewrite: two
    // bodies with their own states, at which point the payload has a height
    // the dynamics uses and the books can audit it.
    const double kineticBefore = 0.5 * SystemMassKg
        * Dot(state.velocityWorldMps, state.velocityWorldMps);
    const double potentialBefore =
        SystemMassKg * GravityMps2 * state.positionWorldM.z;

    state.velocityWorldMps += accelerationWorld * dt;
    state.positionWorldM += state.velocityWorldMps * dt;

    // Moments: the aerodynamic moment the VSM integrated, plus the payload's
    // weight acting off-centre through the carabiners. Neither is a control
    // term - there is no brake-to-yaw or shift-to-roll coefficient anywhere
    // in this file.
    // The held moment, plus the rotational damping evaluated at the rate the
    // wing actually has right now, plus the payload's weight acting through
    // the carabiners. No control term appears here.
    // The damping coefficients, as positive resistances. Only a moment that
    // opposes the rotation is damping; a measured derivative of the other sign
    // is the aerodynamic solve being asked a question it cannot answer, and it
    // is dropped rather than fed back as excitation.
    const Vec3 dampingNmPerRadps{
        -std::min(0.0, state.rotationalDampingNmPerRadps.x),
        -std::min(0.0, state.rotationalDampingNmPerRadps.y),
        -std::min(0.0, state.rotationalDampingNmPerRadps.z)};
    // The pendulum. The payload's weight does not act at the canopy - it acts
    // at a centre of mass eight metres below it, on the end of the lines. Any
    // rotation of the canopy away from having that mass directly beneath it
    // is resisted by W L sin(theta), which for this wing is about 7000 Nm per
    // radian and is far and away the dominant pitch and roll stiffness.
    //
    // Leaving it out is not a small omission. Without it the only pitch
    // moment is the wing's own, nothing restores attitude, and the canopy
    // pitches up until it is descending vertically at 7.5 m/s with no forward
    // speed at all - which is what it did.
    // WHERE the payload is hanging is a state, not a constant. The pilot is a
    // 95 kg bob on the end of a 7 m line whose pivot - the wing - is being
    // accelerated by the air. Pinning the bob straight below the pivot removes
    // the single most important thing in a paraglider's pitch behaviour: the
    // wing and the pilot are not at the same place along track, the angle
    // between them is what a brake input actually changes first, and the surge
    // is that angle running back through zero.
    //
    // The bob equation, with the pivot accelerating. With e the unit vector
    // from canopy to payload and t the tangential direction:
    //
    //     e = ( sin q, 0, -cos q )      t = ( cos q, 0, sin q )
    //     bob acceleration = a_pivot + L q'' t - L q'^2 e
    //     tangential balance:  L q'' = (g - a_pivot) . t  +  drag term
    //
    // so it is driven by the difference between what the wing is doing and
    // what gravity is doing, in body axes, and nothing else. Brake decelerates
    // the wing, the pilot keeps going and swings ahead; release accelerates
    // the wing and it swings back past the pilot, which is the dive. Neither
    // is written down anywhere - they are the same equation with the sign of
    // a_pivot reversed.
    // The harness hangs in the airflow and the pilot is most of the aircraft's
    // parasitic drag, so the bob is pushed aft by its own drag - which is why
    // a pilot hangs slightly behind the wing in trim rather than exactly
    // under it. Level 4's installed drag already knows the number.
    const Vec3 harnessDragBody = airspeed > 1.0e-6
        ? airspeedBody * (-installed.harnessDragN / airspeed) : Vec3{};
    // Damping on the pilot's swing. The lines are not a frictionless hinge and
    // the harness is a bluff body sweeping through air, so the swing bleeds
    // out over a few periods rather than ringing forever.
    //
    // THIS IS THE MODEL'S LEAST DEFENSIBLE NUMBER AND THE MOST LOAD-BEARING.
    // It is stated, not derived, and hands-off stability depends on it: at
    // 0.20, which is what a wing settling in three swings implies and what
    // this file used to say, the aircraft's pitch diverges and it is in a
    // fully separated stall inside a minute. At 0.35 it converges to trim.
    //
    // What the damper is really doing at 0.35 is not damping, it is TRACKING.
    // The pendulum has to follow apparent gravity - in a pull-up the resultant
    // swings round with the flight path and the pilot swings with it, and that
    // is what holds a paraglider's incidence steady through a phugoid. A
    // lightly damped pendulum follows it late. Measured at 0.20 the link
    // tracked 10.7 degrees of a 14.6 degree flight-path change, and the
    // missing 3.9 degrees went into incidence.
    //
    // That matters here and not on a normal aircraft because this wing's pitch
    // feedback has a loop gain of a c Cm / (k CL^2) - measured off its own
    // aerodynamics and its own suspension - which is 0.32 at trim but passes
    // ONE at CL 0.35, and the wing's own full-bar CL is 0.31. So an incidence
    // error below about 2 degrees of incidence does not decay, it runs.
    //
    // Estimated honestly from what physically damps the swing - the pilot's
    // own drag on an 8 m arm, plus the lines sweeping - the ratio is nearer
    // 0.06 than 0.35. So this coefficient is standing in for a stabilising
    // mechanism the model does not have, rather than for friction it does.
    // Registered Tuned and Unvalidated, bounded in `calibration_tests`, and
    // written up in PHYSICS_TODO as the largest known weakness in the pitch
    // axis. It should be retired by finding the missing mechanism, not by
    // being measured more precisely.
    // Item 11's one genuinely free number. A member rather than a constant so
    // `pitch_axis_trace` can sweep it - it is registered Tuned/Unvalidated and
    // the registry says it stands in for a stabilising mechanism the model does
    // not have, so being able to ask what it is worth is the point.
    const double SwingDampingRatio = SwingDampingRatioValue;
    const double swingFrequency =
        std::sqrt(GravityMps2 / std::max(0.5, PendulumLengthM));
    // Where the lines are unstressed, which the accelerator moves. Linear in
    // the pedal because the riser shortening is: the plan lists a trim and a
    // full-bar length for each riser and bar interpolates between them.
    // Brake moves it the other way, and by the amount the network says. Both
    // hands, because this is the symmetric coordinate; the asymmetric part of
    // a brake input reaches the wing through the aerodynamics and the roll
    // spring, not through here.
    const double brakeCommandedSwing = BrakeSwingOffsetRad(
        0.5 * (controls.leftBrake + controls.rightBrake));
    // Reported because it is one half of item 11's open question. This is the
    // nose-up rotation the SHORTENED BRAKE LINE commands geometrically, before
    // the section's nose-down couple has argued with it. What the wing ends up
    // at is `payloadSwingRad`, and the difference between the two is how much
    // of the command the aerodynamics took back.
    diagnostics.brakeCommandedSwingRad = brakeCommandedSwing;
    const double unstressedSwing = TrimSwingRad
        + (AcceleratedSwingRad - TrimSwingRad)
            * std::clamp(controls.accelerator, 0.0, 1.0)
        + brakeCommandedSwing;

    // The link starts hanging where the lines are unstressed, in world axes,
    // and the canopy starts pointing where that link puts it. Starting both at
    // zero was worth about a tenth of a radian of pitch error, which a spring
    // this stiff turns into a violent first second.
    if (!state.initialised)
    {
        state.payloadDirWorld = Normalized(state.attitude.Rotate(Vec3{
            std::sin(unstressedSwing), 0.0, -std::cos(unstressedSwing)}));
        state.linkRateWorldRadps = Vec3{};
    }
    Vec3 linkDirWorld = Normalized(state.payloadDirWorld);

    // What the lines see: the link's direction in the canopy's own axes. Both
    // angles are relative coordinates - the difference between where the pilot
    // hangs and where the canopy points - and they are the ONLY coordinates
    // the springs act on. Absolute attitude does not appear, which is the
    // whole of item 10: a wing banked at 45 degrees with its pilot swung out
    // under it has no line stress and no restoring moment, and that is what a
    // coordinated turn is.
    const auto relativeAngles = [&](const Vec3& dirWorld)
    {
        const Vec3 body = state.attitude.InverseRotate(dirWorld);
        return Vec3{
            std::asin(std::clamp(body.x, -1.0, 1.0)),
            std::asin(std::clamp(body.y, -1.0, 1.0)),
            0.0};
    };
    Vec3 angles = relativeAngles(linkDirWorld);

    const double payloadArmInertiaKgM2 = std::max(1.0,
        PayloadMass.TotalKg() * PendulumLengthM * PendulumLengthM);

    // The stiffnesses at the load the wing is carrying right now. The lines
    // transmit the aerodynamic resultant less the canopy's own small weight,
    // so its magnitude is the load, and it is a live quantity: a wing at 2 g
    // in a spiral has twice the spring a wing at trim has, and a wing that has
    // just been unloaded by a gust has almost none.
    const double lineLoadN = std::max(
        0.0, Length(exchangedForceBody) - CanopyMassKg * GravityMps2);
    const LineStiffness stiffness = LineStiffnessAt(lineLoadN);
    diagnostics.linePitchStiffnessNmPerRad = stiffness.pitchNmPerRad;
    diagnostics.lineRollStiffnessNmPerRad = stiffness.rollNmPerRad;

    // The link's equation of motion, in world axes, as a vector so that no
    // angle convention can get it wrong:
    //
    //   a_rel = g - a_pivot + drag/m           (what the bob is pulled by)
    //   a_t   = a_rel - e (e . a_rel)          (the part it can swing under)
    //   alpha = (e x a_t) / L
    //
    // NOTE this damper is against the WORLD, and that was tested against the
    // alternative rather than assumed. Damping the link's rate relative to the
    // CANOPY is where the friction physically sits, but it leaves the pendulum
    // free to be dragged around by the wing with nothing resisting, and the
    // wing left the envelope inside twenty seconds. Against the world the link
    // lags apparent gravity by its own time constant instead, which is a cost
    // paid knowingly: see the damping ratio below.
    //
    // The line spring does NOT appear here. A moment is not something you can
    // apply to a bob on a string: the lines' reaction to the canopy's spring
    // lands on the harness, whose inertia about its own centre of mass is
    // 5 kg m^2, and not on the swing coordinate, whose inertia is m L^2 =
    // 6200. Feeding it into the swing instead - which the first version of
    // this did - put a 660 Nm couple on a 6200 kg m^2 arm, hung the pilot five
    // degrees behind apparent vertical in steady glide, and took the wing's
    // incidence down with it. The harness rotation that really absorbs it is
    // Level 3's own pendulum, stepped above, which reports its pitch moment
    // separately.
    const Vec3 gravityWorld{0.0, 0.0, -GravityMps2};
    // The SAME force computed above, not a second evaluation of it. With the
    // pilot reference off this is the aircraft-referenced vector rotated to
    // world exactly as before; with it on it is the pilot's own drag, which is
    // the term that opposes the swing and the one item 11's estimate names.
    const Vec3 harnessDragWorld = pilotReferenced
        ? harnessDragWorldForce : state.attitude.Rotate(harnessDragBody);
    const Vec3 relativeAccelWorld = gravityWorld - accelerationWorld
        + harnessDragWorld / std::max(1.0, PayloadMass.TotalKg());
    const Vec3 tangentialAccelWorld = relativeAccelWorld
        - linkDirWorld * Dot(linkDirWorld, relativeAccelWorld);
    const Vec3 linkAngularAccel =
        Cross(linkDirWorld, tangentialAccelWorld)
            / std::max(0.5, PendulumLengthM);

    // Symplectic, and with the damping taken implicitly for the same reason
    // the canopy's is.
    //
    // THE DAMPER IS AGAINST THE WORLD BY DEFAULT, and the paragraph that used
    // to sit here said the opposite - it described the CANOPY-referenced
    // version and the case for it, next to a line that has always damped the
    // world rate. The two were written a level apart and the comment was never
    // brought back when the change was reverted. It is corrected rather than
    // deleted because the argument in it is real and is still the case against
    // this line: damped against the world the link cannot follow apparent
    // gravity without a lag of its own time constant, and following apparent
    // gravity is exactly its job - in a pull-up the resultant swings round
    // with the flight path, the pendulum swings with it, and that is what
    // holds a paraglider's incidence steady through a phugoid. Measured, the
    // link tracked 10.7 degrees of lean against the 14.6 the flight path
    // turned through, and the missing 3.9 went into incidence.
    //
    // What answers that argument is no longer an opinion. The lag it predicts
    // would show as a moving PHASE between the link and surge inside the 16 s
    // mode; that phase holds within 1.9 degrees across the whole damping sweep
    // while the mode's growth rate crosses zero. `PHYSICS_LEARNINGS` §41.
    //
    // The alternative is now reachable rather than remembered - see
    // `SetLinkDampingReference`. Against the canopy the damper resists the
    // link and the wing moving with respect to each other, which is where the
    // friction physically is, and it leaves the pendulum free to be dragged
    // bodily by the wing with nothing resisting.
    //
    // The pilot's own air damping is not lost either way: it is already in the
    // equation above, as the harness drag term, where it belongs.
    Vec3 linkRate = state.linkRateWorldRadps + linkAngularAccel * dt;
    // What the damper is measured against, kept for the energy accounting
    // below: the rate it acts on is NOT the link's world rate when the
    // reference is the canopy, and the dissipation would be wrong by exactly
    // the shared rotation if it read the world rate in both cases. Zero in the
    // world case, which leaves that arithmetic untouched.
    Vec3 damperReferenceRate{};
    const double dampingFactor =
        1.0 + 2.0 * SwingDampingRatio * swingFrequency * dt;
    if (LinkDampingReferenceValue == LinkDampingReference::Canopy)
    {
        // Damp only the part of the link's rate that the CANOPY does not
        // already share. A wing and a pilot rotating together at the same rate
        // have nothing sliding at the maillons, so that component is not
        // damped at all.
        const Vec3 canopyRateWorld =
            state.attitude.Rotate(state.angularVelocityBodyRadps);
        linkRate = canopyRateWorld + (linkRate - canopyRateWorld)
            / dampingFactor;
        damperReferenceRate = canopyRateWorld;
    }
    else
    {
        linkRate = linkRate / dampingFactor;
    }
    // A rotation about the link's own axis is not a degree of freedom of a
    // line - it is a spin of a point mass - so it is projected out rather than
    // integrated into a direction it cannot move.
    linkRate = linkRate - linkDirWorld * Dot(linkDirWorld, linkRate);
    linkDirWorld = Normalized(linkDirWorld + Cross(linkRate, linkDirWorld) * dt);

    // The lines cannot push, so the pilot cannot swing above the wing's own
    // level. Well outside anything short of an SIV manoeuvre, and it is a
    // geometric limit rather than a handling number.
    constexpr double SwingLimitRad = 1.4;
    angles = relativeAngles(linkDirWorld);
    if (std::fabs(angles.x) > SwingLimitRad
        || std::fabs(angles.y) > SwingLimitRad)
    {
        const double clampedX =
            std::clamp(angles.x, -SwingLimitRad, SwingLimitRad);
        const double clampedY =
            std::clamp(angles.y, -SwingLimitRad, SwingLimitRad);
        const double vertical = std::sqrt(std::max(0.0,
            1.0 - std::sin(clampedX) * std::sin(clampedX)
                - std::sin(clampedY) * std::sin(clampedY)));
        linkDirWorld = Normalized(state.attitude.Rotate(Vec3{
            std::sin(clampedX), std::sin(clampedY), -vertical}));
        linkRate = Vec3{};
        angles = relativeAngles(linkDirWorld);
    }
    state.payloadDirWorld = linkDirWorld;
    state.linkRateWorldRadps = linkRate;

    const double swing = angles.x;
    const double lateralSwing = angles.y;
    const double swingFromTrim = swing - unstressedSwing;

    const Vec3 payloadOffsetBody =
        state.attitude.InverseRotate(linkDirWorld) * PendulumLengthM;
    const Vec3 payloadWeightWorld{
        0.0, 0.0, -PayloadMass.TotalKg() * GravityMps2};
    // The payload's weight about the canopy reference, REPORTED AND NOT
    // APPLIED. This used to be the lumped body's gravity moment, and it was
    // the second copy of a restoring torque the swing equation already had:
    // one physical hinge, two springs, twice the pitch stiffness the lines
    // actually provide. With the payload on a link of its own, its weight is
    // carried along that link and reaches the canopy only as line tension
    // through the attachments - which is the measured spring below, and which
    // is applied once.
    const Vec3 pendulumMoment = Cross(
        payloadOffsetBody, state.attitude.InverseRotate(payloadWeightWorld));

    // The canopy's end of the line spring, at the stiffness this load buys.
    // Rotating the canopy nose-up moves the link aft in its own frame, so the
    // swing angle and the canopy's nose-up angle are the same coordinate with
    // opposite signs; the roll pair works the same way about +X.
    const Vec3 lineMomentBody{
        stiffness.rollNmPerRad * lateralSwing,
        -stiffness.pitchNmPerRad * swingFromTrim,
        0.0};
    diagnostics.linePitchMomentNm = lineMomentBody.y;
    diagnostics.lineRollMomentNm = lineMomentBody.x;
    diagnostics.pendulumWeightMomentNm = pendulumMoment.y;
    diagnostics.aeroPitchMomentNm = exchangedMomentBody.y;
    diagnostics.payloadSwingRad = swing;
    diagnostics.payloadSwingLateralRad = lateralSwing;
    // The rate a pilot feels as the surge is the fore-aft one, and it is read
    // off the link's angular velocity rather than carried as its own scalar.
    diagnostics.payloadSwingRateRadps =
        -state.attitude.InverseRotate(linkRate).y;
    // How far ahead of the pilot the canopy is ALONG TRACK, which is what this
    // has always claimed to be and what a pilot can see. Measured in the
    // world, against the direction the aircraft is going.
    //
    // It used to be read off the body-axis offset, and that quietly mixed in
    // the canopy's own attitude: rotate the canopy nose-up and the number
    // moves without the wing or the pilot going anywhere. That was harmless
    // while brake did nothing to the canopy's pitch and stopped being harmless
    // the moment brake started rotating it on its lines, which is the fix
    // above - the pilot swung forward under brake and the canopy's nose-up
    // rotation cancelled it exactly, so the surge read as a flat line.
    const Vec3 trackWorld{
        state.velocityWorldMps.x, state.velocityWorldMps.y, 0.0};
    const double trackSpeed = Length(trackWorld);
    diagnostics.canopyLeadM = trackSpeed > 0.1
        ? -PendulumLengthM * Dot(linkDirWorld, trackWorld / trackSpeed)
        : -payloadOffsetBody.x;

    const Vec3 undampedMomentBody{
        exchangedMomentBody.x + lineMomentBody.x + payloadLoads.rollMomentNm,
        exchangedMomentBody.y + lineMomentBody.y + payloadLoads.pitchMomentNm,
        exchangedMomentBody.z};
    // The CANOPY's inertia, and only the canopy's. The payload's m L^2 used to
    // be added here, which is right for one lumped body rotating about the
    // pilot and wrong for this one: the pilot is not rigidly attached to the
    // wing's rotation any more, they hang on a link with its own state, and
    // their arm inertia is in that link's equation above. Carrying it in both
    // places made the wing 66 times harder to roll than a 5 kg canopy is -
    // 6300 kg m^2 against 95 - which is most of why a brake input took twenty
    // seconds to do what a pilot expects in one.
    // Plus what that swing has to drag with it. Rotating the canopy about a
    // hinge 6.6 m below accelerates the canopy's own 5.1 kg through the arc,
    // and the air it carries with it - the apparent mass, which is larger than
    // the canopy - so the inertia resisting a pitch rotation is not the
    // canopy's own 120 but 120 + (m + m_apparent) h^2. Pitch swings the canopy
    // fore and aft so it drags the chordwise apparent mass; roll swings it
    // sideways and drags the spanwise one.
    const Vec3 inertia{
        95.0 + (CanopyMassKg + ApparentMass.massKg.y)
            * RollHingeArmM * RollHingeArmM,
        120.0 + (CanopyMassKg + ApparentMass.massKg.x)
            * PitchHingeArmM * PitchHingeArmM,
        150.0};
    // Damping AND the line spring are integrated implicitly, the rest
    // explicitly. The measured yaw damping runs to 2e5 Nm per rad/s against an
    // inertia of 150, which is a time constant of under a millisecond - an
    // explicit c*omega at 120 Hz overshoots it by a factor of eleven,
    // alternates sign and doubles every step. That is what sent asymmetric
    // brake to an infinite turn rate in fourteen seconds.
    //
    // The spring needs the same treatment now and did not before. Against the
    // old lumped inertia of 6300 the pitch mode ran at 1.5 rad/s; against the
    // canopy's own 120 it runs at 7, and in a 2 g spiral where the stiffness
    // doubles it runs at 14. Explicit integration of a spring is only
    // conditionally stable and this one is no longer comfortably inside the
    // condition. Taking the restoring moment at the END of the step costs one
    // extra term in the same divide:
    //   omega' = (omega + M/I dt) / (1 + c/I dt + k/I dt^2)
    // and is unconditionally stable at any stiffness.
    const Vec3 springNmPerRad{
        stiffness.rollNmPerRad, stiffness.pitchNmPerRad, 0.0};
    const Vec3 angularAcceleration{
        undampedMomentBody.x / inertia.x,
        undampedMomentBody.y / inertia.y,
        undampedMomentBody.z / inertia.z};
    const Vec3 unrelaxedRate =
        state.angularVelocityBodyRadps + angularAcceleration * dt;
    state.angularVelocityBodyRadps = Vec3{
        unrelaxedRate.x / (1.0 + dampingNmPerRadps.x * dt / inertia.x
                           + springNmPerRad.x * dt * dt / inertia.x),
        unrelaxedRate.y / (1.0 + dampingNmPerRadps.y * dt / inertia.y
                           + springNmPerRad.y * dt * dt / inertia.y),
        unrelaxedRate.z / (1.0 + dampingNmPerRadps.z * dt / inertia.z
                           + springNmPerRad.z * dt * dt / inertia.z)};
    // The damping moment actually applied, at the rate it was applied to, so
    // the reported net moment is the one the state integrated.
    const Vec3 damped{
        -dampingNmPerRadps.x * state.angularVelocityBodyRadps.x,
        -dampingNmPerRadps.y * state.angularVelocityBodyRadps.y,
        -dampingNmPerRadps.z * state.angularVelocityBodyRadps.z};
    const Vec3 momentBody = undampedMomentBody + damped;
    // Structural damping: the wing is not a rigid body and its rotations are
    // resisted by the lines it hangs on. Without this the pendulum rings.
    const double damping = std::clamp(1.0 - 1.6 * dt, 0.0, 1.0);
    state.angularVelocityBodyRadps = state.angularVelocityBodyRadps * damping;
    state.attitude = IntegrateAttitude(
        state.attitude, state.angularVelocityBodyRadps, dt);

    diagnostics.netMomentBodyNm = momentBody;

    // Energy accounting. The work the aerodynamic force does on the system in
    // this step, against the change in kinetic plus potential energy. A
    // subsystem that creates energy cannot hide from this.
    const double kineticAfter = 0.5 * SystemMassKg
        * Dot(state.velocityWorldMps, state.velocityWorldMps);
    const double potentialAfter =
        SystemMassKg * GravityMps2 * state.positionWorldM.z;
    const double swingDampingPowerW =
        2.0 * SwingDampingRatio * swingFrequency * payloadArmInertiaKgM2
            * Dot(linkRate - damperReferenceRate,
                  linkRate - damperReferenceRate);
    diagnostics.swingDampingPowerW = swingDampingPowerW;
    const double workDone = Dot(aeroWorld, state.velocityWorldMps * dt);
    diagnostics.kineticEnergyJ = kineticAfter;
    diagnostics.potentialEnergyJ = potentialAfter;
    diagnostics.energyResidualW =
        ((kineticAfter - kineticBefore) + (potentialAfter - potentialBefore)
         - workDone) / std::max(1.0e-9, dt);

    // A simulation that starts mid-flight starts TRIMMED, for exactly the
    // reason it starts with an inflated canopy: the initial condition has to
    // be a wing, not a set of zeros.
    //
    // The canopy's pitch equilibrium is not its hang pose. The wing carries a
    // 327 Nm nose-down camber couple, so it sits about 3.3 degrees below the
    // pose the lines alone would hold it at, and starting it at the hang pose
    // is a 3.3 degree step input into a spring with a damping ratio near 0.14.
    // That rings to twice the offset, which takes incidence from 6 degrees to
    // 0.3, which takes the LOAD off the lines - and the line spring is a
    // geometric one, so an unloaded wing has almost no pitch stiffness and
    // pitches further. Measured: 976 N and 5727 Nm/rad at a tenth of a second,
    // 207 N and 989 Nm/rad two seconds later, and the wing never recovered.
    //
    // None of that is the trim being wrong - the wing settles at 11.4 m/s
    // against a published 10.8 either side of the excursion. It is a startup
    // transient with enough energy to knock the aircraft out of its own
    // envelope, and the fix is to not apply it.
    if (!state.initialised)
    {
        const double balanceRad = stiffness.pitchNmPerRad > 1.0
            ? exchangedMomentBody.y / stiffness.pitchNmPerRad : 0.0;
        state.attitude = NoseUpAttitude(TrimLineIncidenceRad - balanceRad);
        state.angularVelocityBodyRadps = Vec3{};
        state.payloadDirWorld = Vec3{0.0, 0.0, -1.0};
        state.linkRateWorldRadps = Vec3{};
    }

    const Vec3 span = state.attitude.Rotate({0.0, 1.0, 0.0});
    diagnostics.bankRad = std::asin(std::clamp(-span.z, -1.0, 1.0));
    diagnostics.turnRateRadps = state.angularVelocityBodyRadps.z;

    state.initialised = true;
    LastDiagnostics = diagnostics;
}
}
