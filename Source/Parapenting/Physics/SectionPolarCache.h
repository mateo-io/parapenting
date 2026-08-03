#pragma once

#include "SectionPolarTable.h"

#include <string>

namespace Parapenting::Physics
{
// Solving the section polar table costs about a second: 21 brake stations,
// each a panel factorisation and an incidence sweep. That is 6.5% of one core
// per step against a full second to swap a wing, so it is the only measured
// cost in the solver that a pilot would ever notice - PHYSICS_TODO item 14.
//
// This caches the solved table. The whole difficulty is that a cache is a file
// which can disagree with the geometry that produced it, and a section polar
// that silently disagrees with its own section is precisely the failure this
// level was built away from: the analytic table's STATED stall angle is what
// item 1 replaced, and a stale cache would put a stated number back with none
// of the honesty of having declared it.
//
// So the cache is validated twice, and the second check is the one that
// matters:
//
//   1. The full `ComputedPolarSpec` is written into the file and compared
//      against the requested spec on load. That catches a changed section,
//      Reynolds number, sweep range or brake resolution.
//
//   2. A WITNESS is stored: the lift, drag and moment of one canonical cold
//      solve - the section at zero brake, zero incidence, fully attached,
//      which is by construction the first solve of the sweep and so is exactly
//      reproducible. On load it is re-solved and compared. That catches what
//      the spec cannot: a change to the viscous solver, to the profile
//      geometry, or to anything else in the physics that leaves the inputs
//      identical and the answer different.
//
// Check 2 is what makes this safe without a version constant somebody has to
// remember to bump. It costs one section solve - milliseconds against the
// second it is protecting - and it fails toward re-solving, never toward
// flying stale numbers. A cache that cannot be read, parsed, or validated is
// not an error: it means solve.
struct SectionPolarCacheResult
{
    bool hit = false;
    // Why a load did not produce a table. Reported rather than swallowed,
    // because "the cache silently never hits" and "the cache always hits" are
    // both bugs and they look identical from the outside.
    enum class Miss
    {
        None,
        NoFile,
        Unreadable,
        BadFormat,
        SpecMismatch,
        WitnessMismatch,
    };
    Miss miss = Miss::NoFile;
};

const char* SectionPolarCacheMissName(SectionPolarCacheResult::Miss miss);

// Where cached tables live. Overridden by PARAPENTING_POLAR_CACHE, so a test
// can point at a scratch directory and the game can point at its Saved tree.
// Empty disables the cache entirely, which is what a build that wants to prove
// it can still solve from cold sets.
std::string SectionPolarCacheDirectory();

// One file per spec. The name carries a digest of the spec so two wings do not
// fight over one file; the CONTENT is what is trusted, never the name.
std::string SectionPolarCachePath(const ComputedPolarSpec& spec);

SectionPolarCacheResult LoadSectionPolarTable(
    const ComputedPolarSpec& spec, SectionPolarTable& table);

// Best effort. A cache that cannot be written is a slow start, not a failure,
// so this reports rather than throws.
bool SaveSectionPolarTable(
    const ComputedPolarSpec& spec, const SectionPolarTable& table);
}
