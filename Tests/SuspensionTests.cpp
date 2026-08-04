// Level 2: the suspension is a graph, not a load table.
//
// These are the master plan's Level 2 exit gates, written as checks:
// slack transmits exactly zero, endpoint reactions are equal and opposite,
// weight shift moves carabiner load without a roll moment, brake loads only
// the brake cascade, and full bar changes incidence through riser geometry
// alone.
#include "CanopyGeometry.h"
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

    if (Failures == 0) std::printf("All suspension checks passed.\n");
    else std::printf("%d suspension check(s) failed.\n", Failures);
    return Failures == 0 ? 0 : 1;
}
