#pragma once

#include <cstddef>

namespace Parapenting::Physics
{
// Level 0, guiding rule 9: every coefficient must have a unit, a source, a
// stated estimate and a valid range.
//
// WingParameters and HarnessParameters currently carry around fifty tuned
// constants with no provenance. Some are physical and well founded, some are
// digitised from published EPIC 2 figures, and a good number were chosen to
// make the handling feel right. Nothing distinguishes them, so there is no way
// to know which numbers a later solver level is allowed to overwrite and which
// are load-bearing measurements.
//
// This registry records that distinction. It deliberately does not hold the
// values - WingParameters remains the single source of those - it annotates
// them, and the tests check the annotation still matches reality.

enum class CoefficientSource
{
    // Physical constant or a directly derived quantity. Not tunable.
    Physical,
    // Digitised from published manufacturer or certification data.
    Published,
    // Taken from the parafoil/ram-air literature, possibly adapted.
    Literature,
    // Engineering estimate from geometry or first principles.
    Estimated,
    // Chosen to produce desired behaviour. No external justification.
    Tuned,
};

enum class CalibrationStatus
{
    // Compared against an external reference and within tolerance.
    Validated,
    // Plausible, self-consistent, but never checked against anything.
    Unvalidated,
    // Known to disagree with the reference, retained deliberately.
    Disputed,
    // Expected to be replaced wholesale by a later solver level.
    Provisional,
};

struct CoefficientRecord
{
    const char* name;
    const char* unit;          // SI, or "1" for dimensionless
    double value;
    double validMin;
    double validMax;
    CoefficientSource source;
    CalibrationStatus status;
    // Where the number came from, or what it would take to justify it.
    const char* provenance;
    // Solver level of the master plan that is expected to supersede it.
    // 0 means nothing currently planned replaces it.
    int supersededByLevel;
};

const CoefficientRecord* CoefficientRecords();
std::size_t CoefficientRecordCount();
// Null when the name is not registered.
const CoefficientRecord* FindCoefficient(const char* name);

const char* CoefficientSourceName(CoefficientSource source);
const char* CalibrationStatusName(CalibrationStatus status);

// How much of the model rests on numbers with no external justification.
struct CoefficientAudit
{
    std::size_t total = 0;
    std::size_t tuned = 0;
    std::size_t unvalidated = 0;
    std::size_t outOfRange = 0;
};

CoefficientAudit AuditCoefficients();
}
