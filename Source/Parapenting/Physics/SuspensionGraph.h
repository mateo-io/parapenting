#pragma once

#include "CanopyGeometry.h"
#include "HarnessGeometry.h"
#include "ParagliderDynamics.h"

#include <string>
#include <vector>

namespace Parapenting::Physics
{
// Level 2 of the master plan: the suspension is a graph, not a load table.
//
// SuspensionSystem.h models the risers as four load fractions produced by a
// fitted polynomial in accelerator and brake. Those fractions are the answer to
// the question this level asks, written down in advance: the A riser carries
// 36% because a constant says so, not because the A lines run from the riser to
// the leading edge and the canopy hangs where the tensions balance.
//
// This replaces the fractions with the thing that produces them - carabiners,
// riser tops, main lines, cascade junctions, upper galleries, canopy
// attachments, a brake fan and brake handles - and lets TensionCableSolver find
// the equilibrium. Row shares, carabiner loads, the incidence change on bar and
// the pitch response to brake all come out of the geometry.
//
// Two things are deliberately not modelled here and belong to later levels:
// the payload is a fixed anchor rather than a rigid body (Level 3), and the
// canopy is rigid rather than a membrane (Level 6).

enum class LineRow
{
    A,
    ABaby,
    B,
    C,
    Brake,
};

constexpr int LineRowCount = 5;
const char* LineRowName(LineRow row);

enum class SuspensionNodeKind
{
    // Fixed to the payload. The anchor the whole network reacts against until
    // Level 3 gives the payload its own six degrees of freedom.
    Carabiner,
    // Riser maillon. Rigidly offset from its carabiner; the accelerator
    // shortens that offset, which is the only way bar reaches the canopy.
    RiserTop,
    // Free node where a main line splits into upper lines. Its position is
    // solved, not authored.
    CascadeJunction,
    // Fixed in the canopy body frame, on the lower surface, from Level 1
    // geometry.
    CanopyAttachment,
    // Fixed to the payload. Brake input changes the brake line's rest length
    // rather than moving the handle, so slack behaves correctly.
    BrakeHandle,
};

struct SuspensionNode
{
    SuspensionNodeKind kind = SuspensionNodeKind::CascadeJunction;
    LineRow row = LineRow::A;
    // -1 left half, +1 right half.
    double side = 1.0;
    // Position in the payload frame, for nodes anchored to the payload.
    Vec3 payloadLocalM{};
    // Position in the canopy body frame, for canopy attachments.
    Vec3 canopyLocalM{};
    // Design-pose position in the payload frame, for every node. Rest lengths
    // are measured here, so the unloaded network is exactly taut.
    Vec3 designM{};
    // Canopy attachments only.
    double spanFraction = 0.0;
    double chordFraction = 0.0;
};

struct CableElement
{
    int nodeA = -1;
    int nodeB = -1;
    LineRow row = LineRow::A;
    // 0 = main line (riser top to cascade junction, or brake handle to fan),
    // 1 = upper gallery (junction to canopy attachment).
    int cascadeLevel = 0;
    double restLengthM = 0.0;
    // EA, newtons. Tension is EA times strain, so this is the whole material
    // model: Dyneema is linear to well past any flight load.
    double axialStiffnessN = 0.0;
    double diameterM = 0.0;
    double massKg = 0.0;
};

struct LinePlanUpper
{
    double spanFraction = 0.0;
    double chordFraction = 0.0;
};

// One main line and the upper lines it cascades into. A main with a single
// upper is a straight run with no junction, which is what the tip lines are.
struct LinePlanMain
{
    LineRow row = LineRow::A;
    std::vector<LinePlanUpper> uppers;
};

struct LinePlanSpec
{
    // Published: central canopy-to-riser line length.
    double canopyToRiserM = 7.3;
    // Riser lengths at trim and on full bar, indexed A, A', B, C. The forward
    // risers shorten by 120/120/80/0 mm, which is the published bar travel.
    double trimRiserLengthM[4]{0.50, 0.50, 0.50, 0.50};
    double acceleratedRiserLengthM[4]{0.38, 0.38, 0.42, 0.50};
    // Fore-aft separation of the riser maillons on the plate, +X forward.
    double riserForeAftM[4]{0.035, 0.035, 0.0, -0.035};
    // Level 3: the harness itself, which is where carabiner separation and
    // everything weight shift does now come from. There is no separate travel
    // or tilt stand-in any more - the pilot's CG moves and the harness rolls.
    HarnessGeometry harness{};
    // Brake handle position in the payload frame, right side; mirrored.
    Vec3 brakeHandleLocalM{0.10, 0.34, -0.10};
    // Slack sewn into the brake line at hands-up, and handle travel at full
    // brake. Hands-up must transmit nothing, which is what the slack buys.
    double brakeSlackM = 0.12;
    double brakeTravelM = 0.62;
    // Where along the riser-to-canopy run the cascade junction sits.
    double cascadeSplitFraction = 0.62;
    double brakeCascadeSplitFraction = 0.55;
    // Line diameters by cascade level and row.
    double mainLineDiameterM = 0.00125;
    double upperLineDiameterM = 0.00070;
    double brakeLineDiameterM = 0.00090;
    // Effective axial modulus of sheathed Dyneema-class line. Fibre UHMWPE is
    // near 100 GPa; a sheathed, spliced, bedded-in line behaves softer.
    double lineModulusPa = 55.0e9;
    double lineDensityKgM3 = 970.0;
    // Root chord incidence at the design pose, radians, nose-up positive.
    double designIncidenceRad = 0.0873;
    // Right half-wing. The left half is mirrored, so symmetry is structural
    // rather than something a data file can get wrong.
    std::vector<LinePlanMain> halfWingMains;
};

// EPIC 2 ML: three A mains, one baby-A, four B, three C per wing, matching the
// published line count, each cascading into the upper gallery.
const LinePlanSpec& Epic2MlLinePlan();

// Reads the scalar part of the plan from Data/Wings/*-lineplan.json. The main
// and upper topology stays in code: it is structure, not measurement, and a
// half-written topology would produce a wing that solved but was not this wing.
// Returns false and leaves `spec` untouched on any missing key.
bool LoadLinePlanSpec(const std::string& filePath, LinePlanSpec& spec);

// The canopy's quarter-chord line, sampled at build time so the solver can
// place an aerodynamic resultant anywhere on the wing without carrying the
// geometry around. Chordwise is +X, so a point at any chord fraction is the
// quarter chord plus an offset along the chord.
struct CanopySpanSample
{
    double spanFraction = 0.0;
    Vec3 quarterChordLocalM{};
    double chordM = 0.0;
};

struct SuspensionGraph
{
    std::vector<SuspensionNode> nodes;
    std::vector<CableElement> elements;
    std::vector<CanopySpanSample> spanSamples;
    int leftCarabiner = -1;
    int rightCarabiner = -1;
    // Canopy body origin in the payload frame at the design pose.
    Vec3 canopyDesignOriginM{};
    double designIncidenceRad = 0.0;
    LinePlanSpec plan;
};

// Builds the graph from Level 1 geometry. Attachment positions come from
// CanopyGeometry::SurfacePointM, so there is still one wing: move a rib and
// the lines move with it.
SuspensionGraph BuildSuspensionGraph(
    const CanopyGeometry& geometry, const LinePlanSpec& plan);

// Attitude that pitches the canopy nose-up by `radians`.
//
// The quaternion algebra is right-handed and the flight frame is not
// (ParagliderCoordinateSystem.h: +X forward, +Y right, +Z up), so a positive
// right-hand rotation about +Y tips the nose DOWN. Rather than carry that trap
// around, this names the intent and the solver reads incidence and bank back
// off the rotated basis vectors, where the sign is whatever geometry says.
Quaternion NoseUpAttitude(double radians);

// Point on the canopy in body axes, from the sampled quarter-chord line.
Vec3 CanopyPointLocalM(
    const SuspensionGraph& graph, double spanFraction, double chordFraction);

// Total manufactured line length, for comparison against the published 254 m.
double TotalLineLengthM(const SuspensionGraph& graph);
}
