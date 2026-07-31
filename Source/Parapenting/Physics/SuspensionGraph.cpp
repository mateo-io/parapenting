#include "SuspensionGraph.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace Parapenting::Physics
{
namespace
{
constexpr double Pi = 3.14159265358979323846;

int RiserIndexFor(LineRow row)
{
    switch (row)
    {
    case LineRow::A: return 0;
    case LineRow::ABaby: return 1;
    case LineRow::B: return 2;
    case LineRow::C: return 3;
    case LineRow::Brake: return 3;
    }
    return 0;
}

}

Quaternion NoseUpAttitude(double radians)
{
    return {std::cos(0.5 * radians), 0.0, std::sin(-0.5 * radians), 0.0};
}

const char* LineRowName(LineRow row)
{
    switch (row)
    {
    case LineRow::A: return "A";
    case LineRow::ABaby: return "A'";
    case LineRow::B: return "B";
    case LineRow::C: return "C";
    case LineRow::Brake: return "brake";
    }
    return "?";
}

const LinePlanSpec& Epic2MlLinePlan()
{
    static const LinePlanSpec Plan = []
    {
        LinePlanSpec plan;
        // Right half-wing, tip-ward order within each row. The published line
        // count fixes the number of mains; each main cascades into two upper
        // lines except the baby-A, which is a tip line pair, and the row's
        // chord station comes from the Level 1 attachment digitisation.
        plan.halfWingMains = {
            {LineRow::A, {{0.14, 0.12}, {0.31, 0.12}}},
            {LineRow::A, {{0.45, 0.12}, {0.60, 0.12}}},
            {LineRow::A, {{0.72, 0.12}, {0.85, 0.12}}},
            {LineRow::ABaby, {{0.92, 0.16}, {0.975, 0.16}}},
            {LineRow::B, {{0.10, 0.38}, {0.24, 0.38}}},
            {LineRow::B, {{0.36, 0.38}, {0.48, 0.38}}},
            {LineRow::B, {{0.58, 0.38}, {0.70, 0.38}}},
            {LineRow::B, {{0.80, 0.40}, {0.93, 0.42}}},
            {LineRow::C, {{0.16, 0.66}, {0.33, 0.66}}},
            {LineRow::C, {{0.45, 0.66}, {0.60, 0.68}}},
            {LineRow::C, {{0.72, 0.70}, {0.88, 0.74}}},
            {LineRow::Brake, {{0.22, 0.98}, {0.42, 0.98}}},
            {LineRow::Brake, {{0.62, 0.98}, {0.86, 0.98}}},
        };
        return plan;
    }();
    return Plan;
}

bool LoadLinePlanSpec(const std::string& filePath, LinePlanSpec& spec)
{
    std::ifstream file(filePath);
    if (!file) return false;
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string text = buffer.str();

    LinePlanSpec loaded = Epic2MlLinePlan();
    const auto read = [&text](const char* key, double& out)
    {
        return ReadJsonNumber(text, key, out);
    };
    if (!read("canopyToRiserM", loaded.canopyToRiserM)) return false;
    if (!read("carabinerSeparationM", loaded.harness.carabinerSeparationM))
        return false;
    if (!read("hipTravelM", loaded.harness.hipTravelM)) return false;
    if (!read("chestStrapM", loaded.harness.chestStrapM)) return false;
    if (!read("brakeSlackM", loaded.brakeSlackM)) return false;
    if (!read("brakeTravelM", loaded.brakeTravelM)) return false;
    if (!read("cascadeSplitFraction", loaded.cascadeSplitFraction)) return false;
    if (!read("brakeCascadeSplitFraction", loaded.brakeCascadeSplitFraction))
        return false;
    if (!read("mainLineDiameterM", loaded.mainLineDiameterM)) return false;
    if (!read("upperLineDiameterM", loaded.upperLineDiameterM)) return false;
    if (!read("brakeLineDiameterM", loaded.brakeLineDiameterM)) return false;
    if (!read("lineModulusPa", loaded.lineModulusPa)) return false;
    if (!read("lineDensityKgM3", loaded.lineDensityKgM3)) return false;
    if (!read("designIncidenceRad", loaded.designIncidenceRad)) return false;

    spec = loaded;
    return true;
}

SuspensionGraph BuildSuspensionGraph(
    const CanopyGeometry& geometry, const LinePlanSpec& plan)
{
    SuspensionGraph graph;
    graph.plan = plan;
    graph.designIncidenceRad = plan.designIncidenceRad;

    double meanRiserLengthM = 0.0;
    for (double length : plan.trimRiserLengthM) meanRiserLengthM += 0.25 * length;
    graph.canopyDesignOriginM = {
        0.0, 0.0, meanRiserLengthM + plan.canopyToRiserM};
    const Quaternion designAttitude = NoseUpAttitude(plan.designIncidenceRad);

    const auto addNode = [&graph](const SuspensionNode& node)
    {
        graph.nodes.push_back(node);
        return static_cast<int>(graph.nodes.size()) - 1;
    };

    const auto addCable = [&graph, &plan](
        int a, int b, LineRow row, int level, double extraRestM)
    {
        const double diameter = row == LineRow::Brake
            ? plan.brakeLineDiameterM
            : (level == 0 ? plan.mainLineDiameterM : plan.upperLineDiameterM);
        const double area = 0.25 * Pi * diameter * diameter;
        CableElement cable;
        cable.nodeA = a;
        cable.nodeB = b;
        cable.row = row;
        cable.cascadeLevel = level;
        cable.restLengthM = Length(graph.nodes[static_cast<std::size_t>(b)].designM
            - graph.nodes[static_cast<std::size_t>(a)].designM) + extraRestM;
        cable.axialStiffnessN = plan.lineModulusPa * area;
        cable.diameterM = diameter;
        cable.massKg = plan.lineDensityKgM3 * area * cable.restLengthM;
        graph.elements.push_back(cable);
    };

    for (int sideIndex = 0; sideIndex < 2; ++sideIndex)
    {
        const double side = sideIndex == 0 ? -1.0 : 1.0;
        const double carabinerY = side * 0.5 * plan.harness.carabinerSeparationM;

        SuspensionNode carabiner;
        carabiner.kind = SuspensionNodeKind::Carabiner;
        carabiner.side = side;
        carabiner.payloadLocalM = {0.0, carabinerY, 0.0};
        carabiner.designM = carabiner.payloadLocalM;
        const int carabinerIndex = addNode(carabiner);
        if (side < 0.0) graph.leftCarabiner = carabinerIndex;
        else graph.rightCarabiner = carabinerIndex;

        int riserNode[4]{-1, -1, -1, -1};
        for (int riser = 0; riser < 4; ++riser)
        {
            SuspensionNode top;
            top.kind = SuspensionNodeKind::RiserTop;
            top.side = side;
            top.row = riser == 0 ? LineRow::A
                : riser == 1 ? LineRow::ABaby
                : riser == 2 ? LineRow::B : LineRow::C;
            top.payloadLocalM = {
                plan.riserForeAftM[riser],
                carabinerY,
                plan.trimRiserLengthM[riser]};
            top.designM = top.payloadLocalM;
            riserNode[riser] = addNode(top);
        }

        SuspensionNode handle;
        handle.kind = SuspensionNodeKind::BrakeHandle;
        handle.side = side;
        handle.row = LineRow::Brake;
        handle.payloadLocalM = {
            plan.brakeHandleLocalM.x,
            side * plan.brakeHandleLocalM.y,
            plan.brakeHandleLocalM.z};
        handle.designM = handle.payloadLocalM;
        const int handleIndex = addNode(handle);

        for (const LinePlanMain& main : plan.halfWingMains)
        {
            const int lowerIndex = main.row == LineRow::Brake
                ? handleIndex : riserNode[RiserIndexFor(main.row)];

            // Attachment nodes first: the junction is placed on the run
            // between the riser top and where the uppers actually go, so the
            // cascade geometry follows the canopy rather than the reverse.
            std::vector<int> attachmentIndices;
            Vec3 attachmentMean{};
            for (const LinePlanUpper& upper : main.uppers)
            {
                SuspensionNode attachment;
                attachment.kind = SuspensionNodeKind::CanopyAttachment;
                attachment.row = main.row;
                attachment.side = side;
                attachment.spanFraction = side * upper.spanFraction;
                attachment.chordFraction = upper.chordFraction;
                attachment.canopyLocalM = geometry.SurfacePointM(
                    attachment.spanFraction, upper.chordFraction, false);
                attachment.designM = graph.canopyDesignOriginM
                    + designAttitude.Rotate(attachment.canopyLocalM);
                attachmentIndices.push_back(addNode(attachment));
                attachmentMean += attachment.designM;
            }
            attachmentMean =
                attachmentMean / static_cast<double>(main.uppers.size());

            const double brakeExtra =
                main.row == LineRow::Brake ? plan.brakeSlackM : 0.0;
            if (main.uppers.size() == 1)
            {
                addCable(lowerIndex, attachmentIndices[0], main.row, 0,
                         brakeExtra);
                continue;
            }

            SuspensionNode junction;
            junction.kind = SuspensionNodeKind::CascadeJunction;
            junction.row = main.row;
            junction.side = side;
            const double split = main.row == LineRow::Brake
                ? plan.brakeCascadeSplitFraction : plan.cascadeSplitFraction;
            const Vec3 lower = graph.nodes[
                static_cast<std::size_t>(lowerIndex)].designM;
            junction.designM = lower + (attachmentMean - lower) * split;
            const int junctionIndex = addNode(junction);

            // The whole brake slack goes into the main run, so the fan itself
            // stays taut and the handle carries the slack, as it does on a
            // real wing.
            addCable(lowerIndex, junctionIndex, main.row, 0, brakeExtra);
            for (int attachment : attachmentIndices)
                addCable(junctionIndex, attachment, main.row, 1, 0.0);
        }
    }

    constexpr int SpanSampleCount = 41;
    graph.spanSamples.resize(SpanSampleCount);
    for (int i = 0; i < SpanSampleCount; ++i)
    {
        const double spanFraction = -1.0 + 2.0 * static_cast<double>(i)
            / static_cast<double>(SpanSampleCount - 1);
        const RibStation station = geometry.StationAt(spanFraction);
        graph.spanSamples[static_cast<std::size_t>(i)] = {
            spanFraction, station.positionM, station.chordM};
    }

    return graph;
}

Vec3 CanopyPointLocalM(
    const SuspensionGraph& graph, double spanFraction, double chordFraction)
{
    if (graph.spanSamples.empty()) return {};
    const double clamped = spanFraction < -1.0 ? -1.0
        : (spanFraction > 1.0 ? 1.0 : spanFraction);
    const double position = (clamped + 1.0) * 0.5
        * static_cast<double>(graph.spanSamples.size() - 1);
    const auto lower = static_cast<std::size_t>(position);
    const std::size_t upper =
        lower + 1 < graph.spanSamples.size() ? lower + 1 : lower;
    const double t = position - static_cast<double>(lower);
    const CanopySpanSample& a = graph.spanSamples[lower];
    const CanopySpanSample& b = graph.spanSamples[upper];
    const Vec3 quarterChord =
        a.quarterChordLocalM + (b.quarterChordLocalM - a.quarterChordLocalM) * t;
    const double chord = a.chordM + (b.chordM - a.chordM) * t;
    return {quarterChord.x + (0.25 - chordFraction) * chord,
            quarterChord.y, quarterChord.z};
}

double TotalLineLengthM(const SuspensionGraph& graph)
{
    double total = 0.0;
    for (const CableElement& cable : graph.elements) total += cable.restLengthM;
    return total;
}
}
