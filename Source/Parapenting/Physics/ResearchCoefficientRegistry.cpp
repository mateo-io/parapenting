#include "ResearchCoefficientRegistry.h"
#include "BillowRelaxation.h"
#include "CanopyGeometry.h"
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
    {"weightShiftRollMoment", "N*m", Wing.weightShiftRollMoment, 0.0, 300.0,
     S::Tuned, C::Provisional,
     "Direct control-to-moment. Guiding rule 5 removes this entirely.", 3},
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
