#include "ResearchCoefficientRegistry.h"
#include "BillowRelaxation.h"
#include "CanopyGeometry.h"
#include "HarnessGeometry.h"
#include "PayloadRigidBody.h"
#include "ApparentMassTensor.h"
#include "CanopyMembraneSolver.h"
#include "CanopyPressureSolver.h"
#include "SectionPolarTable.h"
#include "SuspensionGraph.h"
#include "ParagliderDynamics.h"

#include <iterator>
#include <cstring>

namespace Parapenting::Physics
{
namespace
{
constexpr WingParameters Wing{};
constexpr HarnessParameters Harness{};
const CanopyGeometrySpec Canopy{};
const LinePlanSpec& Lines = Epic2MlLinePlan();
constexpr HarnessGeometry Harness3{};
constexpr PayloadMassProperties Payload{};
constexpr AnalyticPolarSpec Polar{};
constexpr CellPressureSpec Cell{};
constexpr MembraneSpec Membrane{};

using S = CoefficientSource;
using C = CalibrationStatus;

// Values are read from the defaults rather than retyped, so the registry
// cannot drift from what the solver actually uses.
// Size deduced: entries are added often enough that a hand-kept count
// would only ever be wrong.
const CoefficientRecord Records[] = {
    // --- Mass and geometry ------------------------------------------------
    {"allUpMassKg", "kg", Wing.allUpMassKg, 55.0, 140.0, S::Published, C::Validated,
     "EPIC 2 ML certified weight range, mid-band pilot plus equipment.", 3},
    {"canopyMassKg", "kg", Wing.canopyMassKg, 3.0, 8.0, S::Published, C::Validated,
     "Published EPIC 2 ML canopy mass.", 6},
    {"areaM2", "m^2", Wing.areaM2, 18.0, 32.0, S::Published, C::Validated,
     "Published EPIC 2 ML flat area.", 1},
    {"airDensityKgM3", "kg/m^3", Wing.airDensityKgM3, 0.9, 1.30, S::Physical, C::Validated,
     "ISA density near typical Bernese Oberland flying altitudes.", 4},

    // --- Trim and polar ---------------------------------------------------
    {"trimCl", "1", Wing.trimCl, 0.35, 0.85, S::Estimated, C::Unvalidated,
     "Back-solved from published trim speed and wing loading. Replaced when "
     "section polars arrive.", 4},
    {"trimAngleOfAttackRad", "rad", Wing.trimAngleOfAttackRad, -0.30, 0.05,
     S::Estimated, C::Unvalidated,
     "Chosen with trimCl to land on published trim speed.", 4},
    {"zeroLiftDrag", "1", Wing.zeroLiftDrag, 0.008, 0.040, S::Literature, C::Unvalidated,
     "Canopy-only profile drag, ram-air parafoil range.", 4},
    {"inducedDragFactor", "1", Wing.inducedDragFactor, 0.03, 0.20, S::Literature, C::Unvalidated,
     "Induced drag factor for the published aspect ratio.", 4},
    {"maxLiftCoefficient", "1", Wing.maxLiftCoefficient, 1.0, 1.8, S::Literature, C::Unvalidated,
     "Typical parafoil CLmax. No section data behind it yet.", 4},
    {"stallAngleRad", "rad", Wing.stallAngleRad, 0.17, 0.42, S::Literature, C::Unvalidated,
     "16 deg. Section polars will supersede this.", 4},

    // --- Brake ------------------------------------------------------------
    {"brakeLiftGain", "1", Wing.brakeLiftGain, 0.1, 0.9, S::Tuned, C::Unvalidated,
     "Camber change with brake. No measurement.", 4},
    {"brakeDragGain", "1", Wing.brakeDragGain, 0.1, 0.9, S::Tuned, C::Unvalidated,
     "Drag rise with brake. No measurement.", 4},
    {"brakeTravelMm", "mm", Wing.brakeTravelMm, 400.0, 900.0, S::Published, C::Validated,
     "EPIC 2 ML brake line travel to stall.", 2},
    {"brakeFreePlayFraction", "1", Wing.brakeFreePlayFraction, 0.0, 0.30,
     S::Estimated, C::Unvalidated, "Slack before the trailing edge loads.", 2},
    {"brakePressureExponent", "1", Wing.brakePressureExponent, 1.0, 2.5,
     S::Tuned, C::Unvalidated, "Shapes the felt pressure build.", 2},
    {"brakeForceAtFullTravelN", "N", Wing.brakeForceAtFullTravelN, 20.0, 90.0,
     S::Estimated, C::Unvalidated, "Brake pressure at full travel.", 2},

    // --- Inertia and apparent mass ---------------------------------------
    {"rollInertiaKgM2", "kg*m^2", Wing.rollInertiaKgM2, 40.0, 200.0, S::Estimated, C::Unvalidated,
     "Estimated from span and mass distribution.", 3},
    {"pitchInertiaKgM2", "kg*m^2", Wing.pitchInertiaKgM2, 50.0, 250.0, S::Estimated, C::Unvalidated,
     "Estimated from chord and suspension length.", 3},
    {"yawInertiaKgM2", "kg*m^2", Wing.yawInertiaKgM2, 60.0, 300.0, S::Estimated, C::Unvalidated,
     "Estimated from span and mass distribution.", 3},
    {"apparentMassKgX", "kg", Wing.apparentMassKg.x, 1.0, 40.0, S::Literature, C::Unvalidated,
     "Lissaman & Brown ellipsoid apparent mass, forward axis.", 4},
    {"apparentMassKgY", "kg", Wing.apparentMassKg.y, 5.0, 80.0, S::Literature, C::Unvalidated,
     "Lissaman & Brown ellipsoid apparent mass, lateral axis.", 4},
    {"apparentMassKgZ", "kg", Wing.apparentMassKg.z, 5.0, 90.0, S::Literature, C::Unvalidated,
     "Lissaman & Brown ellipsoid apparent mass, normal axis.", 4},
    {"apparentInertiaX", "kg*m^2", Wing.apparentRotationalInertiaKgM2.x, 2.0, 60.0,
     S::Literature, C::Unvalidated, "Lissaman & Brown apparent inertia, roll.", 4},
    {"apparentInertiaY", "kg*m^2", Wing.apparentRotationalInertiaKgM2.y, 5.0, 90.0,
     S::Literature, C::Unvalidated, "Lissaman & Brown apparent inertia, pitch.", 4},
    {"apparentInertiaZ", "kg*m^2", Wing.apparentRotationalInertiaKgM2.z, 5.0, 110.0,
     S::Literature, C::Unvalidated, "Lissaman & Brown apparent inertia, yaw.", 4},

    // --- Damping ----------------------------------------------------------
    {"rollDamping", "N*m*s/rad", Wing.rollDamping, 10.0, 120.0, S::Tuned, C::Unvalidated,
     "Lumped roll damping. Emerges from panel forces at Level 4.", 4},
    {"pitchDamping", "N*m*s/rad", Wing.pitchDamping, 60.0, 500.0, S::Tuned, C::Unvalidated,
     "Lumped pitch damping. Emerges from panel forces at Level 4.", 4},
    {"yawDamping", "N*m*s/rad", Wing.yawDamping, 50.0, 400.0, S::Tuned, C::Unvalidated,
     "Lumped yaw damping. Emerges from panel forces at Level 4.", 4},
    {"pitchStiffness", "N*m/rad", Wing.pitchStiffness, 40.0, 400.0, S::Tuned, C::Unvalidated,
     "Pendular restoring toward trim incidence. Emerges once payload and "
     "lines are separate bodies.", 3},

    // --- Direct control moments: all superseded by rule 2 -----------------
    {"brakeRollMoment", "N*m", Wing.brakeRollMoment, 0.0, 500.0, S::Tuned, C::Provisional,
     "Direct control-to-moment. Guiding rule 2 removes this entirely.", 8},
    {"brakeYawMoment", "N*m", Wing.brakeYawMoment, 0.0, 400.0, S::Tuned, C::Provisional,
     "Direct control-to-moment. Guiding rule 2 removes this entirely.", 8},
    {"coordinatedRollStiffness", "N*m/rad", Wing.coordinatedRollStiffness, 200.0, 1600.0,
     S::Tuned, C::Provisional, "Coupled-turn approximation.", 7},
    {"coordinatedRollDamping", "N*m*s/rad", Wing.coordinatedRollDamping, 40.0, 400.0,
     S::Tuned, C::Provisional, "Coupled-turn approximation.", 7},
    {"yawToBankGain", "1", Wing.yawToBankGain, 0.1, 1.5, S::Tuned, C::Provisional,
     "Coupled-turn approximation.", 7},
    {"weightShiftBankRad", "rad", Wing.weightShiftBankRad, 0.1, 1.0, S::Tuned, C::Provisional,
     "Bank reached at full weight shift. Emerges from payload CG at Level 3.", 3},
    {"maximumSustainedBankRad", "rad", Wing.maximumSustainedBankRad, 0.4, 1.4,
     S::Tuned, C::Provisional, "Envelope limit, not a flight characteristic.", 7},
    {"coordinatedYawStiffness", "N*m/rad", Wing.coordinatedYawStiffness, 50.0, 500.0,
     S::Tuned, C::Provisional, "Coupled-turn approximation.", 7},

    // --- Accelerator ------------------------------------------------------
    {"acceleratorLiftReduction", "1", Wing.acceleratorLiftReduction, 0.0, 0.4,
     S::Estimated, C::Unvalidated, "Incidence change at full bar.", 2},
    {"acceleratorDragReduction", "1", Wing.acceleratorDragReduction, 0.0, 0.05,
     S::Estimated, C::Unvalidated, "Profile drag change at full bar.", 2},
    {"acceleratorPitchMoment", "N*m", Wing.acceleratorPitchMoment, 0.0, 150.0,
     S::Tuned, C::Provisional, "Direct moment; riser geometry replaces it.", 2},

    // --- Load envelope ----------------------------------------------------
    {"loadSofteningOnsetG", "1", Wing.loadSofteningOnsetG, 2.0, 6.0, S::Estimated, C::Unvalidated,
     "Flexible-canopy softening onset. Distinct from the 8 g EN structural "
     "qualification boundary.", 6},
    {"operationalLiftLimitG", "1", Wing.operationalLiftLimitG, 3.0, 7.0,
     S::Estimated, C::Unvalidated, "Research envelope, not a certification claim.", 6},
    {"overspeedDragOnsetMps", "m/s", Wing.overspeedDragOnsetMps, 12.0, 26.0,
     S::Tuned, C::Unvalidated, "Numerical envelope guard.", 4},
    {"overspeedDragQuadratic", "1", Wing.overspeedDragQuadratic, 0.0, 0.01,
     S::Tuned, C::Unvalidated, "Numerical envelope guard.", 4},

    // --- Collapse and recovery: all superseded by rule 6 ------------------
    {"collapseResistance", "1", Wing.collapseResistance, 0.5, 2.0, S::Tuned, C::Provisional,
     "Scripted collapse threshold. Rule 6 makes collapse emergent.", 8},
    {"passiveReinflationRate", "1/s", Wing.passiveReinflationRate, 0.05, 0.6,
     S::Tuned, C::Provisional, "Scripted recovery rate. Level 5 pressure replaces it.", 8},
    {"brakeReinflationGain", "1", Wing.brakeReinflationGain, 0.0, 0.6, S::Tuned, C::Provisional,
     "Scripted recovery gain.", 8},
    {"pumpReinflationGain", "1", Wing.pumpReinflationGain, 0.3, 2.5, S::Tuned, C::Provisional,
     "Scripted recovery gain.", 8},
    {"frontalReinflationRate", "1/s", Wing.frontalReinflationRate, 0.1, 0.8,
     S::Tuned, C::Provisional, "Scripted recovery rate.", 8},
    {"cravatSusceptibility", "1", Wing.cravatSusceptibility, 0.3, 2.0, S::Tuned, C::Provisional,
     "Scripted cravat likelihood. Level 8 derives it from tip geometry.", 8},
    {"recoverySurgeGain", "1", Wing.recoverySurgeGain, 0.5, 5.0, S::Tuned, C::Provisional,
     "Scripted surge magnitude.", 8},

    // --- Harness ----------------------------------------------------------
    {"harnessDragAreaM2", "m^2", Harness.dragAreaM2, 0.15, 0.60, S::Literature, C::Unvalidated,
     "Seated pilot plus harness drag area.", 4},

    // --- Level 1 canopy geometry -----------------------------------------
    {"flatSpanM", "m", Canopy.flatSpanM, 10.0, 13.0, S::Published, C::Validated,
     "BGD EPIC 2 ML published flat span.", 0},
    {"flatAreaM2", "m^2", Canopy.flatAreaM2, 20.0, 32.0, S::Published, C::Validated,
     "BGD EPIC 2 ML published flat area.", 0},
    {"rootChordM", "m", Canopy.rootChordM, 2.0, 3.5, S::Published, C::Validated,
     "BGD EPIC 2 ML published root chord. Pins the planform taper.", 0},
    {"projectedSpanM", "m", Canopy.projectedSpanM, 8.0, 11.0, S::Published, C::Validated,
     "BGD EPIC 2 ML published projected span. Solves the arc.", 0},
    {"cellCount", "1", static_cast<double>(Canopy.cellCount), 30.0, 70.0,
     S::Published, C::Validated, "BGD EPIC 2 ML published cell count.", 0},
    {"arcExponent", "1", Canopy.arcExponent, 1.0, 3.0, S::Estimated, C::Unvalidated,
     "Distribution of arc curvature toward the tips. Only the span ratio is "
     "published; the shape is assumed pending a digitised rib plan.", 1},
    {"billowFraction", "1", Canopy.billowFraction, 0.0, 0.12, S::Estimated, C::Unvalidated,
     "Peak chord-cut seam allowance. Sets the inflated section through the "
     "membrane relaxation rather than being drawn.", 6},
    {"internalPressurePa", "Pa", Canopy.internalPressurePa, 0.0, 400.0,
     S::Estimated, C::Provisional,
     "Cell pressure above ambient, taken as trim dynamic pressure. Level 5 "
     "solves it from inlet flow instead.", 5},
    {"membraneStiffnessNPerM", "N/m", Canopy.fabric.membraneStiffnessNPerM,
     5000.0, 100000.0, S::Literature, C::Unvalidated,
     "Coated ripstop membrane stiffness, E times thickness. Drives how much "
     "the cloth stretches under hoop tension.", 6},

    // --- Level 2 suspension graph -----------------------------------------
    {"lineModulusPa", "Pa", Lines.lineModulusPa, 20.0e9, 120.0e9,
     S::Literature, C::Unvalidated,
     "Effective axial modulus of sheathed Dyneema-class line. UHMWPE fibre is "
     "near 100 GPa; sheathed, spliced and bedded-in line behaves softer. Sets "
     "line stretch and therefore the trim shift under load. A pull test on a "
     "real line replaces it.", 9},
    {"designIncidenceRad", "rad", Lines.designIncidenceRad, 0.0, 0.20,
     S::Published, C::Validated,
     "Root chord incidence at the unloaded design pose, where the line rest "
     "lengths are cut. IDENTIFIED at Level 9 against the published 39 km/h "
     "hands-up trim at the published 105 kg all-up, which is the fit this "
     "entry always called for. Residual 0.4 km/h. Three numbers it was NOT "
     "fitted to follow: sink 1.15 m/s against 1.14, glide 9.43 against 9.5, "
     "and incidence 5.02 deg against the 5.30 the published trim CL of 0.580 "
     "needs. Was 0.0873, a round 5 degrees, worth 2.4 km/h. The "
     "manufacturer's rigging angle would still be better.", 4},
    {"cascadeSplitFraction", "1", Lines.cascadeSplitFraction, 0.3, 0.9,
     S::Estimated, C::Unvalidated,
     "Height of the cascade junctions along the riser-to-canopy run. Sets how "
     "much of the load path is main line rather than upper gallery, and so how "
     "much the suspension stretches. A digitised line plan replaces it.", 0},
    {"brakeSlackM", "m", Lines.brakeSlackM, 0.0, 0.35, S::Estimated, C::Unvalidated,
     "Slack sewn into the brake line at hands-up. Without it the trailing edge "
     "loads at trim.", 0},

    {"swingDampingRatio", "1", 0.35, 0.0, 1.0, S::Tuned, C::Unvalidated,
     "Damping ratio on the pilot's swing about the canopy. THE MODEL'S LEAST "
     "DEFENSIBLE NUMBER AND THE MOST LOAD-BEARING: hands-off pitch stability "
     "depends on it, and at 0.20 - which is what a wing settling in three "
     "swings implies, and what this solver used to use - the aircraft "
     "diverges and is fully separated inside a minute. Estimated from what "
     "physically damps the swing, the pilot's own drag on an 8 m arm plus the "
     "lines sweeping, it should be nearer 0.06. So it is standing in for a "
     "stabilising mechanism the model does not have rather than for friction "
     "it does: the pendulum has to TRACK apparent gravity through a phugoid "
     "and a lightly damped one follows late, which matters because this "
     "wing's pitch loop gain passes one below CL 0.35. Retire it by finding "
     "the missing mechanism, not by measuring it better. PHYSICS_TODO item "
     "11.", 11},
    {"linePitchStiffnessSpecificM", "m", 6.13, 3.0, 12.0, S::Physical,
     C::Validated,
     "Line pitch stiffness per newton of load, measured off the built "
     "suspension graph by holding the canopy either side of its free "
     "equilibrium at four loads: 3306, 6317, 11512 and 15393 Nm/rad at half a "
     "g, one, two and four. Proportional to load because the spring is "
     "GEOMETRIC - the lines stretch 0.2% while the canopy origin moves "
     "0.13 m, so the wing pivots about a virtual hinge 6.6 m below itself. "
     "Not a coefficient so much as a property of the graph; listed because "
     "freezing it at its one-g value is what made the pitch axis diverge.", 0},
    {"pitchHingeArmM", "m", 6.62, 3.0, 12.0, S::Physical, C::Validated,
     "How far the canopy's origin travels per radian it rotates, off the same "
     "probe: 0.1325 m at 0.02 rad and 0.2648 at 0.04, so a constant arm. Sets "
     "the inertia that resists a pitch rotation, which is the canopy's own "
     "plus the apparent mass it drags through the arc.", 0},

    // --- Level 3 payload and harness --------------------------------------
    {"hipTravelM", "m", Harness3.hipTravelM, 0.03, 0.14, S::Estimated,
     C::Unvalidated,
     "How far a seated pilot can move their CG sideways at full weight shift. "
     "The whole of weight-shift authority now rests on this and the chest "
     "strap - there is no authority multiplier left to hide behind. Measure "
     "it on a simulator rig to replace it.", 9},
    {"chestStrapM", "m", Harness3.chestStrapM, 0.34, 0.56, S::Estimated,
     C::Unvalidated,
     "Chest strap setting between riser attachment points. Narrowing it "
     "increases weight-shift authority through geometry alone.", 9},
    {"carabinerAboveCgM", "m", Harness3.carabinerAboveCgM, 0.15, 0.45,
     S::Estimated, C::Unvalidated,
     "Carabiners above the seated pilot's CG. Sets the payload's own pendulum "
     "period, which is the settle the pilot feels under them, and converts CG "
     "offset into harness roll.", 9},
    {"pilotKg", "kg", Payload.pilotKg, 45.0, 120.0, S::Published, C::Validated,
     "Pilot mass. Part of the certified weight range, not a handling "
     "parameter.", 0},
    {"suspensionLengthM", "m", 8.08, 4.0, 12.0, S::Estimated, C::Unvalidated,
     "Payload CG to canopy: carabiner arm, riser and line run. Sets every "
     "pendulum period in the model, and is measured on the built suspension "
     "graph rather than written down beside it.", 7},
    {"mobilityLossPerG", "1", 0.35, 0.0, 1.0, S::Estimated, C::Unvalidated,
     "How much of a pilot's weight-shift reach is lost per g. Their own body "
     "is the mass being moved, so the effort scales with load while the "
     "strength available does not. No measurement behind the rate.", 9},
    // --- Level 4 section polars -------------------------------------------
    // Every one of these is theory, not measurement. The whole table is
    // Provisional: XFOIL runs over the digitised profiles replace it wholesale.
    {"sectionCamberFraction", "1", Polar.camberFraction, 0.0, 0.08,
     S::Estimated, C::Provisional,
     "Section camber. Sets the zero-lift angle through thin-airfoil theory, "
     "so it sets trim incidence. Assumed from the profile family, not "
     "digitised.", 9},
    {"sectionThicknessFraction", "1", Polar.thicknessFraction, 0.05, 0.25,
     S::Estimated, C::Provisional,
     "Section thickness. Only enters as the lift-slope correction here.", 9},
    {"flapChordFraction", "1", Polar.flapChordFraction, 0.5, 0.95,
     S::Estimated, C::Provisional,
     "Where the brake starts. Feeds thin-airfoil flap effectiveness, which is "
     "derived rather than fitted - this fraction is the only assumption in "
     "the brake model.", 9},
    {"sectionStallMarginRad", "rad", Polar.stallMarginRad, 0.12, 0.40,
     S::Estimated, C::Provisional,
     "Where the section stalls above its zero-lift angle. 14 deg is typical "
     "for a thick cambered section; nothing here measures it.", 9},
    {"stallBlendWidthRad", "rad", Polar.stallBlendWidthRad, 0.02, 0.30,
     S::Estimated, C::Provisional,
     "Angular width of the stall transition. Zero below stall and one beyond, "
     "so attached flow carries none of the post-stall branch. Real polars "
     "make this a measurement rather than a shape.", 9},
    {"fullBrakeDeflectionRad", "rad", Polar.fullBrakeDeflectionRad, 0.15, 0.80,
     S::Estimated, C::Provisional,
     "Trailing edge deflection at full brake. With the derived flap "
     "effectiveness this is what sets where on the brake travel the wing "
     "stalls, so it is the calibration hook for stall onset.", 9},
    {"lineProjectedFraction", "1", 0.35, 0.15, 0.60, S::Estimated,
     C::Unvalidated,
     "How much of the manufactured line length is normal to the flow. "
     "Cascades overlap and lower lines shield one another, so it is well "
     "under one. Together with harness area this puts whole-aircraft glide at "
     "9.46 against the published 9.5.", 9},
    {"harnessAreaM2", "m^2", 0.32, 0.15, 0.60, S::Literature, C::Unvalidated,
     "Frontal area of a seated pilot and harness. The largest single drag "
     "contributor on the aircraft, larger than the whole line set.", 9},
    {"apparentMassNormalKg", "kg", 33.6, 10.0, 60.0, S::Literature,
     C::Unvalidated,
     "Air accelerated with the canopy normal to the wing, from the ellipsoid "
     "idealisation on the projected geometry. A third of the aircraft's mass, "
     "which is why it cannot be left out. Agrees to within a tenth with the "
     "independent estimate already in WingParameters.", 6},
    {"apparentRollInertiaKgM2", "kg*m^2", 254.0, 5.0, 400.0, S::Literature,
     C::Disputed,
     "Air rotated with the canopy in roll. Dimensionally consistent and scales "
     "correctly with span, but the leading coefficient could not be checked "
     "against the source paper and it lands well above the 18 kg m^2 estimate "
     "already carried. Disputed deliberately: nothing uses its magnitude until "
     "someone reads Lissaman and Brown against it.", 4},
    // --- Level 5 cell pressure --------------------------------------------
    {"inletAngularPositionRad", "rad", Cell.inletAngularPositionRad, 0.0, 0.60,
     S::Estimated, C::Unvalidated,
     "Where the leading-edge opening is cut, as an angle below the chord line "
     "on the nose. Everything about how well the wing pressurises follows "
     "from this against the stagnation point, so it is the first thing to "
     "digitise from a real canopy.", 0},
    {"inletAreaM2", "m^2", Cell.inletAreaM2, 0.002, 0.05, S::Estimated,
     C::Unvalidated,
     "Open area of one cell's inlet. Sets how fast a cell fills and, with the "
     "cell volume, the whole inflation timescale.", 0},
    {"cellVolumeM3", "m^3", Cell.cellVolumeM3, 0.05, 0.60, S::Estimated,
     C::Unvalidated,
     "Default cell volume, used when a caller has no geometry to hand. "
     "CanopyGeometry::CellVolumeM3 now integrates it from the solved inflated "
     "section instead: 0.31 m^3 at the root, 0.09 at the tip, 9.9 m^3 for the "
     "canopy. Change the seam allowance and the inflation time moves with "
     "it.", 0},
    {"shapeHoldingPressureCoefficient", "1", 0.55, 0.2, 0.9, S::Estimated,
     C::Provisional,
     "Internal pressure coefficient below which a section is treated as "
     "having lost its shape entirely. Stands in for the reduced-pressure "
     "polar family Level 4 owes; a single documented curve rather than a "
     "per-effect tuning, and it is what makes a soft cell group stop making "
     "lift where it is.", 9},
    {"crossportAreaM2", "m^2", Cell.crossportAreaM2, 0.0, 0.02, S::Estimated,
     C::Unvalidated,
     "Rib crossport area. Far smaller than an inlet, so a cell fed only "
     "through its ribs fills in tens of seconds rather than seconds - which "
     "is why blocked inlets across a group matter.", 0},
    {"fabricPorosityM3PerM2sPerPa", "m^3/(m^2*s*Pa)",
     Cell.porosityM3PerM2sPerPa, 0.0, 1.0e-4, S::Literature, C::Unvalidated,
     "Leakage through coated ripstop. Small enough that it is not what "
     "empties a cell; the inlet running backwards is.", 0},
    {"collapseRiskPressureCoefficient", "1",
     Cell.collapseRiskPressureCoefficient, 0.1, 0.8, S::Estimated,
     C::Unvalidated,
     "Internal pressure coefficient below which a cell is reported at risk. "
     "A reporting threshold only - nothing in the model behaves differently "
     "either side of it, and Level 8 is where collapse itself is decided.", 8},
    // --- Level 6 membrane -------------------------------------------------
    {"warpStiffnessNPerM", "N/m", Membrane.fabric.warpStiffnessNPerM,
     5000.0, 100000.0, S::Literature, C::Unvalidated,
     "Membrane stiffness along the warp threads. With the hoop tension it "
     "sets how far the skin stretches: 0.06% at flight pressure, which is "
     "why the inflated shape is essentially the pattern's.", 9},
    {"biasStiffnessNPerM", "N/m", Membrane.fabric.biasStiffnessNPerM,
     500.0, 30000.0, S::Literature, C::Unvalidated,
     "Stiffness on the bias, where the weave shears rather than the threads "
     "stretching. Eight times softer here, and it is what governs how a "
     "canopy wrinkles and where it folds.", 9},
    {"membraneSolverMassScale", "1", Membrane.solverMassScale, 1.0, 1.0e6,
     S::Estimated, C::Unvalidated,
     "Fictitious node mass, as a multiple of the fabric's own. Numerical, "
     "not physical: ripstop is 2.7e6 N/m stiff against 0.46 g a node, which "
     "puts its elastic waves at 12 kHz, and resolving those is what the "
     "solve was spending itself on. Physical forces are computed from the "
     "real mass, and the tests check the shape is unchanged at ten times "
     "this value.", 12},
    {"vsmFilamentCoreFraction", "1", 0.5, 0.05, 2.0, S::Estimated,
     C::Unvalidated,
     "Trailing-filament core radius as a fraction of panel width. Numerical, "
     "not physical: it exists to bound the singularity a control point sees "
     "from its own trailing legs. Must never be used to shape handling, and "
     "the panel-convergence test is what keeps it honest.", 11},

    {"payloadRollRadiusOfGyrationM", "m", 0.24, 0.15, 0.40, S::Literature,
     C::Unvalidated,
     "Seated human plus harness in roll. Anthropometric tables put a seated "
     "adult near this; it sets how fast the harness settles.", 9},
};
constexpr std::size_t RecordCount = std::size(Records);
}

const CoefficientRecord* CoefficientRecords() { return Records; }
std::size_t CoefficientRecordCount() { return RecordCount; }

const CoefficientRecord* FindCoefficient(const char* name)
{
    if (!name) return nullptr;
    for (const auto& record : Records)
        if (std::strcmp(record.name, name) == 0) return &record;
    return nullptr;
}

const char* CoefficientSourceName(CoefficientSource source)
{
    switch (source)
    {
    case CoefficientSource::Physical: return "physical";
    case CoefficientSource::Published: return "published";
    case CoefficientSource::Literature: return "literature";
    case CoefficientSource::Estimated: return "estimated";
    case CoefficientSource::Tuned: return "tuned";
    }
    return "unknown";
}

const char* CalibrationStatusName(CalibrationStatus status)
{
    switch (status)
    {
    case CalibrationStatus::Validated: return "validated";
    case CalibrationStatus::Unvalidated: return "unvalidated";
    case CalibrationStatus::Disputed: return "disputed";
    case CalibrationStatus::Provisional: return "provisional";
    }
    return "unknown";
}

CoefficientAudit AuditCoefficients()
{
    CoefficientAudit audit;
    audit.total = RecordCount;
    for (const auto& record : Records)
    {
        if (record.source == CoefficientSource::Tuned) ++audit.tuned;
        if (record.status != CalibrationStatus::Validated) ++audit.unvalidated;
        if (!(record.value >= record.validMin && record.value <= record.validMax))
            ++audit.outOfRange;
    }
    return audit;
}
}
