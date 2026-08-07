// Level 2: the suspension is a graph, not a load table.
//
// These are the master plan's Level 2 exit gates, written as checks:
// slack transmits exactly zero, endpoint reactions are equal and opposite,
// weight shift moves carabiner load without a roll moment, brake loads only
// the brake cascade, and full bar changes incidence through riser geometry
// alone.
#include "CanopyGeometry.h"
#include "CoupledParagliderSolver.h"
#include "SuspensionGraph.h"
#include "TensionCableSolver.h"

#include <algorithm>
#include <cmath>
#include <vector>
#include <cstdio>
#include <string>

using namespace Parapenting::Physics;

namespace
{
int Failures = 0;

void Check(bool condition, const std::string& what)
{
    if (!condition)
    {
        std::printf("  FAIL  %s\n", what.c_str());
        ++Failures;
    }
}

// All-up weight of a mid-range EPIC 2 ML pilot, as a steady-flight load.
constexpr double AllUpMassKg = 100.0;
constexpr double TrimLoadN = AllUpMassKg * 9.80665;

SuspensionSolveInput TrimInput()
{
    SuspensionSolveInput input;
    // Steady flight: the aerodynamic resultant carries the whole system.
    input.aeroForceN = {0.0, 0.0, TrimLoadN};
    input.canopyWeightN = 5.1 * 9.80665;
    return input;
}

double RowFraction(const SuspensionSolution& solution, LineRow row)
{
    return solution.rowLoadFraction[static_cast<std::size_t>(row)];
}

double RowTension(const SuspensionSolution& solution, LineRow row)
{
    return solution.rowTensionN[static_cast<std::size_t>(row)];
}
}

int main(int argc, char** argv)
{
    const std::string dataPath = argc > 1
        ? argv[1] : "Data/Wings/bgd-epic-2-ml-lineplan.json";
    const CanopyGeometry geometry;
    const SuspensionGraph graph =
        BuildSuspensionGraph(geometry, Epic2MlLinePlan());

    std::printf("Suspension graph\n");
    std::printf("  nodes %zu, cables %zu, manufactured line %.1f m\n",
                graph.nodes.size(), graph.elements.size(),
                TotalLineLengthM(graph));

    // -- the drag the lines actually present -------------------------------
    //
    // `InstalledDragSpec::lineProjectedFraction` is 0.35, and its comment says
    // what the number bundles: "manufactured line length is not all normal to
    // the flow: cascades overlap, upper galleries are inclined, lower lines
    // shield one another". Three effects, one stated number, never derived -
    // and installed drag is 47% of the canopy's own, so this is not a
    // correction term. `PHYSICS_TODO` item 12 asks for a glide that lands
    // "without a coefficient chosen to put it there", and this is one of the
    // coefficients.
    //
    // Two of the three effects are GEOMETRY, and the graph has the geometry. A
    // cylinder in crossflow presents `L d sin(theta)` to the flow, theta being
    // the angle between the line and the wind, so the inclination part of that
    // fraction is a sum over the elements this file has already built. What it
    // cannot see is SHIELDING - a line in another line's wake - because that is
    // a flow question, not a geometric one.
    //
    // So this does not replace the dial. It splits it into a part that is
    // measured and a residual that is named, which is the difference between a
    // number nobody can check and a number with one honest unknown left in it.
    {
        const Vec3 flow{1.0, 0.0, 0.0};
        double totalLength = 0.0;
        double frontalAreaM2 = 0.0;
        double projectedAreaM2 = 0.0;
        double weightedDiameter = 0.0;
        for (const CableElement& cable : graph.elements)
        {
            if (cable.nodeA < 0 || cable.nodeB < 0) continue;
            const Vec3 a = graph.nodes[cable.nodeA].designM;
            const Vec3 b = graph.nodes[cable.nodeB].designM;
            const Vec3 span{b.x - a.x, b.y - a.y, b.z - a.z};
            const double length = std::sqrt(
                span.x * span.x + span.y * span.y + span.z * span.z);
            if (length < 1.0e-9) continue;
            const Vec3 dir{span.x / length, span.y / length, span.z / length};
            // |d x f| for unit vectors is sin of the angle between them.
            const Vec3 cross{
                dir.y * flow.z - dir.z * flow.y,
                dir.z * flow.x - dir.x * flow.z,
                dir.x * flow.y - dir.y * flow.x};
            const double sine = std::sqrt(
                cross.x * cross.x + cross.y * cross.y + cross.z * cross.z);
            totalLength += length;
            weightedDiameter += length * cable.diameterM;
            frontalAreaM2 += length * cable.diameterM;
            projectedAreaM2 += length * cable.diameterM * sine;
        }
        const double meanDiameter =
            totalLength > 0.0 ? weightedDiameter / totalLength : 0.0;
        const double geometricFraction =
            frontalAreaM2 > 0.0 ? projectedAreaM2 / frontalAreaM2 : 0.0;
        const InstalledDragSpec shipped;
        std::printf("\n  Line drag area, measured off the built graph "
                    "(PHYSICS_LEARNINGS section 62)\n");
        std::printf("    geometric line length      %8.1f m   (spec says "
                    "%.1f)\n", totalLength, shipped.lineTotalLengthM);
        std::printf("    length-weighted diameter   %8.5f m   (spec says "
                    "%.5f)\n", meanDiameter, shipped.lineMeanDiameterM);
        std::printf("    projected fraction, incl.  %8.3f     (spec says "
                    "%.3f)\n", geometricFraction,
                    shipped.lineProjectedFraction);
        std::printf("    implied shielding residual %8.3f\n",
                    geometricFraction > 0.0
                        ? shipped.lineProjectedFraction / geometricFraction
                        : 0.0);
        std::printf("    projected area             %8.5f m2\n",
                    projectedAreaM2);
        // Bounds rather than a fit. Inclination alone cannot take a bundle of
        // mostly-spanwise-inclined lines below a half, and it cannot exceed one
        // by definition - a fraction outside that says the design pose or the
        // flow direction is wrong, not that the wing is unusual.
        Check(geometricFraction > 0.4 && geometricFraction < 1.0,
              "geometric projected fraction is a fraction, and inclination "
              "alone does not halve the bundle");
        Check(totalLength > 200.0 && totalLength < 320.0,
              "graph line length is near the manufactured 254 m");
    }

    // -- is 60% shielding even geometrically possible? ---------------------
    //
    // §62 isolated `lineShieldingFactor` as the one stated number left in line
    // drag: 0.394, meaning the cascade is asserted to hide 60% of its own
    // frontal area from the flow. That is a large claim and it was never
    // checked against the one thing that decides it - HOW FAR APART THE LINES
    // ARE.
    //
    // Wake interference between cylinders is a function of spacing in
    // DIAMETERS. A cylinder sitting a few diameters behind another is in a
    // genuine velocity deficit; by twenty or thirty diameters the wake has
    // spread and slowed and the deficit is small; at hundreds it is nothing.
    // Paraglider lines are about a millimetre thick and spaced in centimetres
    // to metres, so the ratio is the whole question and it is pure geometry.
    //
    // This measures the nearest-neighbour distance for every cable, at the
    // design pose, in units of its own diameter. It does not compute a
    // shielding factor - that needs a wake model - but it bounds one: if the
    // typical line has no neighbour within tens of diameters, no wake argument
    // reaches 60%.
    {
        std::vector<double> spacingInDiameters;
        const auto midpoint = [&](const CableElement& c)
        {
            const Vec3 a = graph.nodes[c.nodeA].designM;
            const Vec3 b = graph.nodes[c.nodeB].designM;
            return Vec3{0.5 * (a.x + b.x), 0.5 * (a.y + b.y),
                        0.5 * (a.z + b.z)};
        };
        for (std::size_t i = 0; i < graph.elements.size(); ++i)
        {
            const CableElement& ci = graph.elements[i];
            if (ci.nodeA < 0 || ci.nodeB < 0 || ci.diameterM <= 0.0) continue;
            const Vec3 mi = midpoint(ci);
            double nearest = 1.0e30;
            for (std::size_t j = 0; j < graph.elements.size(); ++j)
            {
                if (i == j) continue;
                const CableElement& cj = graph.elements[j];
                if (cj.nodeA < 0 || cj.nodeB < 0) continue;
                const Vec3 mj = midpoint(cj);
                const Vec3 d{mj.x - mi.x, mj.y - mi.y, mj.z - mi.z};
                const double distance =
                    std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
                if (distance > 1.0e-9) nearest = std::min(nearest, distance);
            }
            if (nearest < 1.0e29)
                spacingInDiameters.push_back(nearest / ci.diameterM);
        }
        std::sort(spacingInDiameters.begin(), spacingInDiameters.end());
        const auto quantile = [&](double q)
        {
            if (spacingInDiameters.empty()) return 0.0;
            const std::size_t index = static_cast<std::size_t>(
                q * static_cast<double>(spacingInDiameters.size() - 1));
            return spacingInDiameters[index];
        };
        std::printf("\n  Line-to-line spacing at the design pose, in "
                    "DIAMETERS (section 64)\n");
        std::printf("    closest pair            %8.0f d\n", quantile(0.0));
        std::printf("    lower quartile          %8.0f d\n", quantile(0.25));
        std::printf("    median                  %8.0f d\n", quantile(0.50));
        std::printf("    upper quartile          %8.0f d\n", quantile(0.75));
        std::printf("    shielding factor stated %8.3f  (so 60%% of the "
                    "frontal area\n                                      is "
                    "asserted to be hidden)\n",
                    InstalledDragSpec{}.lineShieldingFactor);
        // A bound, not a fit. Cylinder wake deficits are a near-field effect;
        // a cascade whose CLOSEST pair is already tens of diameters apart
        // cannot hide most of itself, whatever wake model is used.
        Check(quantile(0.0) > 5.0,
              "even the closest pair of lines is clear of the near wake");
        Check(quantile(0.5) > 50.0,
              "the median line has no neighbour within fifty diameters");
    }

    // -- topology ---------------------------------------------------------
    {
        Check(graph.leftCarabiner >= 0 && graph.rightCarabiner >= 0,
              "both carabiners exist");

        int attachments = 0;
        int junctions = 0;
        for (const SuspensionNode& node : graph.nodes)
        {
            if (node.kind == SuspensionNodeKind::CanopyAttachment) ++attachments;
            if (node.kind == SuspensionNodeKind::CascadeJunction) ++junctions;
        }
        Check(attachments == 52, "26 canopy attachments per side");
        Check(junctions == 26, "one cascade junction per main line");

        // Every cable must terminate on real nodes, and no cable may be
        // degenerate: this is the Level 1 gate carried forward to the graph.
        for (const CableElement& cable : graph.elements)
        {
            Check(cable.nodeA >= 0
                  && cable.nodeA < static_cast<int>(graph.nodes.size()),
                  "cable start is a real node");
            Check(cable.nodeB >= 0
                  && cable.nodeB < static_cast<int>(graph.nodes.size()),
                  "cable end is a real node");
            Check(cable.restLengthM > 0.05, "cable has a positive rest length");
            Check(cable.axialStiffnessN > 1000.0, "cable has real stiffness");
        }

        // Mirrored pairs are structural, not data, so this checks the build
        // rather than the file.
        double leftLength = 0.0;
        double rightLength = 0.0;
        for (const CableElement& cable : graph.elements)
        {
            if (graph.nodes[static_cast<std::size_t>(cable.nodeA)].side < 0.0)
                leftLength += cable.restLengthM;
            else
                rightLength += cable.restLengthM;
        }
        Check(std::fabs(leftLength - rightLength) < 1e-9,
              "left and right halves are identical lengths");

        // Published total line length is 254 m. The rest lengths are derived
        // from the geometry, so agreement is a check on the cascade split and
        // the design pose, not something that was fitted.
        const double total = TotalLineLengthM(graph);
        Check(total > 190.0 && total < 320.0,
              "manufactured line length is near the published 254 m");

        // The pendulum length the flight model uses is measured here, not
        // written down twice.
        const double pendulum = SuspensionPendulumLengthM(graph);
        std::printf("  pendulum length %.2f m\n", pendulum);
        Check(pendulum > graph.plan.canopyToRiserM,
              "the pendulum reaches below the risers to the pilot's CG");
        Check(pendulum < graph.plan.canopyToRiserM + 1.5,
              "and not by much - risers and the seat, not another wingspan");
    }

    // -- hands-up trim ----------------------------------------------------
    const SuspensionSolution trim = SolveSuspension(graph, TrimInput());
    {
        std::printf("Trim: A %.0f%%  A' %.0f%%  B %.0f%%  C %.0f%%  brake %.0f%%\n",
                    100.0 * RowFraction(trim, LineRow::A),
                    100.0 * RowFraction(trim, LineRow::ABaby),
                    100.0 * RowFraction(trim, LineRow::B),
                    100.0 * RowFraction(trim, LineRow::C),
                    100.0 * RowFraction(trim, LineRow::Brake));
        std::printf("  carabiner %.0f / %.0f N, stretch %.1f cm, "
                    "incidence %+.2f deg\n",
                    trim.leftCarabinerLoadN, trim.rightCarabinerLoadN,
                    100.0 * trim.lineStretchM,
                    trim.incidenceChangeRad * 180.0 / 3.14159265358979);
        std::printf("  residual: canopy %.2f N / %.2f Nm, node %.3f N, "
                    "slack cables %d\n",
                    trim.canopyForceResidualN, trim.canopyMomentResidualNm,
                    trim.maxNodeResidualN, trim.slackCableCount);

        Check(trim.canopyForceResidualN < 0.02 * TrimLoadN,
              "canopy force residual is a fraction of a percent of the load");
        Check(trim.canopyMomentResidualNm < 20.0,
              "canopy moment residual is small");
        Check(trim.maxNodeResidualN < 2.0,
              "cascade junctions are in force balance");

        // Equal and opposite, exactly: every tension appears twice with
        // opposite sign.
        Check(Length(trim.endpointForceSumN) < 1e-9,
              "endpoint reactions sum to zero");

        // The lines carry the system. Carabiner reaction must match the
        // applied load, or force is appearing from nowhere.
        const double carried =
            trim.leftCarabinerLoadN + trim.rightCarabinerLoadN;
        Check(carried > 0.9 * TrimLoadN && carried < 1.1 * TrimLoadN,
              "carabiner load matches the applied aerodynamic load");

        // Slack transmits exactly zero. Not small - zero.
        for (const CableState& cable : trim.cables)
        {
            if (!cable.slack) continue;
            Check(cable.tensionN == 0.0,
                  "a slack cable transmits exactly zero force");
        }
        for (const CableState& cable : trim.cables)
            Check(cable.tensionN >= 0.0, "no cable is in compression");

        // Hands up, the brake line is slack by construction of its sewn
        // slack, so the trailing edge is unloaded.
        Check(RowTension(trim, LineRow::Brake) == 0.0,
              "hands-up brake lines carry nothing");

        // Row order emerges from where the lines attach and where the
        // resultant acts. It is not asserted anywhere in the solver.
        Check(RowTension(trim, LineRow::A) > RowTension(trim, LineRow::C),
              "the A row carries more than the C row at trim");
        Check(RowTension(trim, LineRow::B) > 0.0, "the B row is loaded");

        Check(trim.meanMainStrain > 0.0005 && trim.meanMainStrain < 0.03,
              "main lines stretch by a fraction of a percent");
        Check(trim.lineStretchM > 0.005 && trim.lineStretchM < 0.20,
              "line stretch is centimetres, not millimetres or metres");

        Check(std::fabs(trim.lateralLoadImbalance) < 1e-6,
              "symmetric flight loads both carabiners equally");
        Check(std::fabs(trim.canopyRollRad) < 1e-6,
              "symmetric flight produces no roll");
    }

    // -- determinism ------------------------------------------------------
    {
        const SuspensionSolution again = SolveSuspension(graph, TrimInput());
        bool identical = again.cables.size() == trim.cables.size();
        for (std::size_t i = 0; identical && i < trim.cables.size(); ++i)
            identical = again.cables[i].tensionN == trim.cables[i].tensionN;
        Check(identical, "the same input produces bit-identical tensions");
    }

    // -- accelerator ------------------------------------------------------
    {
        SuspensionSolveInput input = TrimInput();
        input.accelerator = 1.0;
        const SuspensionSolution bar = SolveSuspension(graph, input);
        std::printf("Full bar: incidence %+.2f deg (trim %+.2f), "
                    "A %.0f%% C %.0f%%\n",
                    bar.incidenceChangeRad * 180.0 / 3.14159265358979,
                    trim.incidenceChangeRad * 180.0 / 3.14159265358979,
                    100.0 * RowFraction(bar, LineRow::A),
                    100.0 * RowFraction(bar, LineRow::C));

        // Shortening the front risers must pitch the canopy nose-down. No
        // code path other than riser geometry is involved.
        Check(bar.incidenceChangeRad < trim.incidenceChangeRad - 0.01,
              "full bar reduces canopy incidence");
        // The C row unloads on bar, which is why rear-riser steering works
        // there. Where the shed load goes between A and B depends on the
        // direction of the aerodynamic resultant, and at this level that
        // vector is an input rather than a solved one - Level 4 supplies it
        // with drag included, and this check gets sharper then.
        Check(RowFraction(bar, LineRow::C) < RowFraction(trim, LineRow::C),
              "bar unloads the C risers");
        Check(std::fabs(bar.canopyRollRad) < 1e-6,
              "bar is symmetric and rolls nothing");
    }

    // -- brake ------------------------------------------------------------
    {
        SuspensionSolveInput input = TrimInput();
        input.rightBrake = 0.7;
        const SuspensionSolution brake = SolveSuspension(graph, input);
        const std::size_t brakeRow = static_cast<std::size_t>(LineRow::Brake);
        std::printf("Right brake 0.7: brake row L %.0f N / R %.0f N, "
                    "roll %+.2f deg\n",
                    brake.leftRowTensionN[brakeRow],
                    brake.rightRowTensionN[brakeRow],
                    brake.canopyRollRad * 180.0 / 3.14159265358979);

        Check(brake.rightRowTensionN[brakeRow] > 10.0,
              "right brake loads the right brake cascade");
        Check(brake.leftRowTensionN[brakeRow] == 0.0,
              "right brake leaves the left brake line slack");

        // Brake must reach the canopy only through the trailing edge. Every
        // loaded brake cable ends on an attachment aft of mid-chord.
        // Only cables carrying real load are checked: a released brake line
        // still hangs from the trailing edge under its own weight, and that
        // fraction of a newton is physics, not a control input.
        for (const CableState& cable : brake.cables)
        {
            if (cable.row != LineRow::Brake || cable.tensionN < 1.0) continue;
            const SuspensionNode& end =
                graph.nodes[static_cast<std::size_t>(cable.nodeB)];
            if (end.kind != SuspensionNodeKind::CanopyAttachment) continue;
            Check(end.chordFraction > 0.9,
                  "brake load lands on the trailing edge");
            Check(end.spanFraction > 0.0,
                  "right brake loads only the right half");
        }

        // The turn is a consequence, not a command: pulling one brake
        // deforms the load path and the canopy rolls toward that side.
        Check(brake.canopyRollRad > 0.0,
              "right brake rolls the canopy right through the lines");
    }

    // -- weight shift -----------------------------------------------------
    {
        SuspensionSolveInput right = TrimInput();
        right.weightShift = 1.0;
        const SuspensionSolution shifted = SolveSuspension(graph, right);
        std::printf("Weight shift right: carabiner %.0f / %.0f N, "
                    "roll %+.2f deg\n",
                    shifted.leftCarabinerLoadN, shifted.rightCarabinerLoadN,
                    shifted.canopyRollRad * 180.0 / 3.14159265358979);

        Check(shifted.rightCarabinerLoadN > shifted.leftCarabinerLoadN,
              "weight shift right loads the right carabiner more");
        Check(shifted.lateralLoadImbalance > 0.0,
              "the imbalance has the sign of the shift");
        Check(shifted.canopyRollRad > 0.0,
              "the canopy rolls right without any direct roll moment");

        SuspensionSolveInput left = TrimInput();
        left.weightShift = -1.0;
        const SuspensionSolution mirrored = SolveSuspension(graph, left);
        // The graph is mirror-exact by construction (checked above), so any
        // asymmetry here is the relaxation's remaining residual, not the wing.
        // It tightens with iteration count, which the convergence check below
        // relies on.
        Check(std::fabs(mirrored.canopyRollRad + shifted.canopyRollRad)
                  < 1e-4 * std::fabs(shifted.canopyRollRad),
              "mirrored weight shift mirrors the roll");
        Check(std::fabs(mirrored.rightCarabinerLoadN
                        - shifted.leftCarabinerLoadN)
                  < 1e-3 * shifted.rightCarabinerLoadN,
              "mirrored weight shift mirrors the carabiner loads");
    }

    // -- convergence ------------------------------------------------------
    {
        // The Level 7 gate in miniature: an answer that changes when the
        // iteration budget changes is a tuned answer, not a solved one.
        SuspensionSolverSettings longer;
        longer.iterations = 30000;
        const SuspensionSolution settled =
            SolveSuspension(graph, TrimInput(), longer);
        Check(settled.canopyForceResidualN < trim.canopyForceResidualN
              || settled.canopyForceResidualN < 0.01,
              "more iterations do not increase the residual");
        Check(std::fabs(settled.totalTensionN - trim.totalTensionN)
                  < 0.005 * trim.totalTensionN,
              "line tensions are converged, not iteration-dependent");
        Check(std::fabs(settled.incidenceChangeRad - trim.incidenceChangeRad)
                  < 0.002,
              "trim incidence is converged, not iteration-dependent");
    }

    // -- asymmetric aerodynamic load --------------------------------------
    {
        SuspensionSolveInput input = TrimInput();
        input.spanwiseLoadAsymmetry = 0.3;
        const SuspensionSolution asymmetric = SolveSuspension(graph, input);
        Check(asymmetric.rightCarabinerLoadN > asymmetric.leftCarabinerLoadN,
              "a right-heavy aerodynamic load loads the right carabiner more");
        Check(asymmetric.canopyRollRad < 0.0,
              "the more heavily loaded half rises");
    }

    // -- load scaling -----------------------------------------------------
    {
        SuspensionSolveInput input = TrimInput();
        input.aeroForceN = {0.0, 0.0, 2.0 * TrimLoadN};
        const SuspensionSolution loaded = SolveSuspension(graph, input);
        const double ratio = loaded.totalTensionN / trim.totalTensionN;
        Check(ratio > 1.9 && ratio < 2.1,
              "doubling the load doubles line tension");
        Check(loaded.meanMainStrain > trim.meanMainStrain,
              "more load means more stretch");
        Check(std::fabs(loaded.incidenceChangeRad - trim.incidenceChangeRad)
                  > 1.0e-4,
              "stretch under load shifts trim, and the shift is recorded");
    }

    // -- line plan data file ----------------------------------------------
    {
        LinePlanSpec loaded = Epic2MlLinePlan();
        const bool read = LoadLinePlanSpec(dataPath, loaded);
        Check(read, "the line plan data file loads");
        if (read)
        {
            const LinePlanSpec& compiled = Epic2MlLinePlan();
            Check(loaded.canopyToRiserM == compiled.canopyToRiserM,
                  "data file and compiled default agree on canopy-to-riser");
            Check(loaded.lineModulusPa == compiled.lineModulusPa,
                  "data file and compiled default agree on line modulus");
            Check(loaded.brakeTravelM == compiled.brakeTravelM,
                  "data file and compiled default agree on brake travel");
            Check(loaded.designIncidenceRad == compiled.designIncidenceRad,
                  "data file and compiled default agree on design incidence");
        }

        LinePlanSpec untouched = Epic2MlLinePlan();
        untouched.brakeTravelM = 1.234;
        Check(!LoadLinePlanSpec("does-not-exist.json", untouched),
              "a missing file fails to load");
        Check(untouched.brakeTravelM == 1.234,
              "a failed load leaves the spec untouched");
    }

    // -- riser set -----------------------------------------------------------
    {
        LinePlanSpec plan = Epic2MlLinePlan();
        Check(plan.riserCount == 4, "the Epic is a three-liner: four risers");
        Check(plan.riserRow[0] == LineRow::A
                  && plan.riserRow[1] == LineRow::ABaby
                  && plan.riserRow[2] == LineRow::B
                  && plan.riserRow[3] == LineRow::C,
              "its risers are A, A', B, C front to back");

        MakeTwoLinerRiserSet(plan);
        Check(plan.riserCount == 2, "a two-liner has two risers");
        Check(plan.riserRow[0] == LineRow::A && plan.riserRow[1] == LineRow::B,
              "and they are the A and the B");
        Check(plan.riserForeAftM[0] > plan.riserForeAftM[1],
              "the A sits forward of the B on the plate");
        Check(plan.acceleratedRiserLengthM[0] < plan.trimRiserLengthM[0],
              "the bar shortens the A");
        Check(plan.acceleratedRiserLengthM[1] == plan.trimRiserLengthM[1],
              "and leaves the rear riser alone, which is what it pitches "
              "about");
        // The rest of the plan is untouched: this changes the riser set, not
        // which rows exist above it.
        Check(plan.halfWingMains.size()
                  == Epic2MlLinePlan().halfWingMains.size(),
              "reconfiguring risers does not rewrite the line plan");
    }

    // -- brake station influence ------------------------------------------
    {
        const std::vector<double> stations = BrakeStationSpans(graph);
        Check(!stations.empty(), "the graph publishes brake stations");
        Check(std::is_sorted(stations.begin(), stations.end()),
              "brake stations come back ascending");
        const double reach = BrakeStationReach(stations);
        Check(reach > 0.0 && reach <= 0.35, "station reach is bounded");

        // A station pulls its own trailing edge at full travel.
        for (const double station : stations)
            Check(std::fabs(BrakeStationInfluence(station, stations, reach)
                      - 1.0) < 1e-9,
                  "a brake station sees the whole travel");

        // Never more than the travel that was actually pulled, anywhere.
        double peak = 0.0;
        for (int step = -200; step <= 200; ++step)
            peak = std::max(peak,
                BrakeStationInfluence(step / 200.0, stations, reach));
        Check(peak <= 1.0 + 1e-12, "influence never exceeds the travel");

        // The point of the change: one side's brake must not pull the other
        // side's trailing edge.
        std::vector<double> leftOnly;
        for (const double station : stations)
            if (station < 0.0) leftOnly.push_back(station);
        Check(!leftOnly.empty(), "the left fan has stations");
        for (int step = 1; step <= 200; ++step)
            Check(BrakeStationInfluence(step / 200.0, leftOnly, reach) == 0.0,
                  "a left-side fan leaves the right trailing edge alone");

        // And it must fall off between stations rather than acting as one
        // rigid flap across the half span.
        const double outermost = stations.front();
        Check(BrakeStationInfluence(outermost - 4.0 * reach, stations, reach)
                  < 0.05,
              "influence decays away from the outermost station");
        Check(BrakeStationInfluence(0.0, {-0.9}, 0.1) < 0.05,
              "an outboard station does not pull the centre");

        // Degenerate inputs keep the old whole-span behaviour rather than
        // silently flattening the wing.
        Check(BrakeStationInfluence(0.3, {}, reach) == 1.0,
              "no stations means undiminished travel");
        Check(BrakeStationReach({}) > 0.0, "an empty fan still has a reach");
        Check(BrakeStationReach({-0.5, 0.5}) > 0.0,
              "a centreline-only gap does not collapse the reach");
    }

    // -- the construction probes converge on an answer, and it is not the one
    //    they ship ------------------------------------------------------
    //
    // The stiffness curve is measured by relaxing this network 24 times with
    // the canopy held either side of its hang pose. That relaxation is
    // UNDER-DAMPED at the shipped settings: it rings past its answer rather
    // than creeping up on it, and 12000 iterations stops it somewhere on the
    // way.
    //
    // Two things are asserted, and only the first is a property of the model:
    //
    //   1. THE CONTROL. Equilibrium cannot depend on the fictitious damping -
    //      it is a numerical device with no physics in it - so two very
    //      different relaxation paths, run long enough, must land on the same
    //      spring. That is what makes the reference below a reference rather
    //      than just another setting.
    //   2. THE SHIPPED ERROR, bounded where it is rather than where anyone
    //      would like it, so it cannot quietly grow. It is roughly 1.7% on the
    //      roll spring at one g. Closing it is `PHYSICS_TODO` item 14 and is
    //      blocked on a modelling decision, not on this measurement:
    //      `PHYSICS_LEARNINGS` §52.
    {
        std::printf("\nConstruction probes: shipped against converged\n");
        const CanopyGeometry canopy;
        const LinePlanSpec linePlan = Epic2MlLinePlan();
        const auto build = [&](int held, double retention)
        {
            ConstructionProbe probe;
            probe.heldIterations = held;
            probe.heldRetention = retention;
            return CoupledParagliderSolver(canopy, linePlan, CoupledSchedule{},
                                           PayloadMassProperties{}, probe);
        };
        const CoupledParagliderSolver shipped = build(12000, 0.999);
        const CoupledParagliderSolver reference = build(48000, 0.999);
        const CoupledParagliderSolver otherPath = build(48000, 0.995);
        const double weightN = shipped.AllUpMassKg() * 9.80665;
        const auto at = [&](const CoupledParagliderSolver& solver, double g)
            { return solver.LineStiffnessAt(g * weightN); };

        double worstPathGap = 0.0;
        double worstShippedGap = 0.0;
        for (const double g : {0.5, 1.0, 2.0, 4.0})
        {
            const auto r = at(reference, g);
            const auto o = at(otherPath, g);
            const auto s = at(shipped, g);
            const auto gap = [](double a, double b)
                { return std::fabs(a - b) / std::max(1.0, std::fabs(b)); };
            worstPathGap = std::max({worstPathGap,
                gap(o.pitchNmPerRad, r.pitchNmPerRad),
                gap(o.rollNmPerRad, r.rollNmPerRad)});
            worstShippedGap = std::max({worstShippedGap,
                gap(s.pitchNmPerRad, r.pitchNmPerRad),
                gap(s.rollNmPerRad, r.rollNmPerRad)});
            std::printf("  %.1f g: reference %.0f/%.0f, shipped %.0f/%.0f\n",
                        g, r.pitchNmPerRad, r.rollNmPerRad,
                        s.pitchNmPerRad, s.rollNmPerRad);
        }
        std::printf("  worst gap: converged paths %.3f%%, shipped %.3f%%\n",
                    100.0 * worstPathGap, 100.0 * worstShippedGap);
        Check(worstPathGap < 0.005,
              "two relaxation paths converge on the same spring - the "
              "equilibrium does not depend on the fictitious damping, which "
              "is what makes the reference a reference");
        Check(worstShippedGap < 0.025,
              "KNOWN: the shipped construction probes stop while the "
              "relaxation is still ringing, about 1.7% out on the roll spring "
              "at one g. Bounded so it cannot grow; item 14");
    }

    if (Failures == 0) std::printf("All suspension checks passed.\n");
    else std::printf("%d suspension check(s) failed.\n", Failures);
    return Failures == 0 ? 0 : 1;
}
