#include "SectionPolarCache.h"

#include "SectionProfile.h"
#include "SectionViscousSolver.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace Parapenting::Physics
{
namespace
{
constexpr char Magic[8] = {'P', 'P', 'P', 'O', 'L', 'A', 'R', 'C'};
constexpr std::uint32_t FormatVersion = 1;

// Adding a field to either spec must break this build, because a field that is
// not serialised is a field the cache cannot tell apart - which is exactly the
// silent drift this file exists to prevent. If one of these fires, add the new
// field to Write/Read below and then update the size.
static_assert(sizeof(SectionProfileSpec) == 72,
              "SectionProfileSpec changed: serialise the new field in "
              "SectionPolarCache.cpp before updating this size");
static_assert(sizeof(ComputedPolarSpec) == 128,
              "ComputedPolarSpec changed: serialise the new field in "
              "SectionPolarCache.cpp before updating this size");
static_assert(sizeof(AnalyticPolarSpec) == 88,
              "AnalyticPolarSpec changed: serialise the new field in "
              "SectionPolarCache.cpp before updating this size");

template <typename T>
void WritePod(std::ostream& out, const T& value)
{
    static_assert(std::is_trivially_copyable_v<T>);
    out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

template <typename T>
bool ReadPod(std::istream& in, T& value)
{
    static_assert(std::is_trivially_copyable_v<T>);
    in.read(reinterpret_cast<char*>(&value), sizeof(T));
    return static_cast<bool>(in);
}

void WriteDoubles(std::ostream& out, const std::vector<double>& values)
{
    WritePod(out, static_cast<std::uint64_t>(values.size()));
    if (!values.empty())
    {
        out.write(reinterpret_cast<const char*>(values.data()),
                  static_cast<std::streamsize>(
                      values.size() * sizeof(double)));
    }
}

bool ReadDoubles(std::istream& in, std::vector<double>& values)
{
    std::uint64_t count = 0;
    if (!ReadPod(in, count)) return false;
    // A length read out of a corrupted file must not become an allocation.
    if (count > (1u << 24)) return false;
    values.assign(static_cast<std::size_t>(count), 0.0);
    if (count == 0) return true;
    in.read(reinterpret_cast<char*>(values.data()),
            static_cast<std::streamsize>(count * sizeof(double)));
    return static_cast<bool>(in);
}

void WriteSamples(std::ostream& out,
                  const std::vector<SectionPolarSample>& values)
{
    WritePod(out, static_cast<std::uint64_t>(values.size()));
    for (const SectionPolarSample& sample : values)
    {
        WritePod(out, sample.liftCoefficient);
        WritePod(out, sample.dragCoefficient);
        WritePod(out, sample.momentCoefficient);
    }
}

bool ReadSamples(std::istream& in, std::vector<SectionPolarSample>& values)
{
    std::uint64_t count = 0;
    if (!ReadPod(in, count)) return false;
    if (count > (1u << 24)) return false;
    values.assign(static_cast<std::size_t>(count), SectionPolarSample{});
    for (SectionPolarSample& sample : values)
    {
        if (!ReadPod(in, sample.liftCoefficient)) return false;
        if (!ReadPod(in, sample.dragCoefficient)) return false;
        if (!ReadPod(in, sample.momentCoefficient)) return false;
    }
    return true;
}

void WriteSection(std::ostream& out, const SectionProfileSpec& s)
{
    WritePod(out, s.maxThicknessFraction);
    WritePod(out, s.maxThicknessPosition);
    WritePod(out, s.maxCamberFraction);
    WritePod(out, s.maxCamberPosition);
    WritePod(out, s.brakeChordFraction);
    WritePod(out, s.fullBrakeDeflectionRad);
    WritePod(out, s.brakeBlendChordFraction);
    WritePod(out, s.inletChordFraction);
    WritePod(out, static_cast<std::uint64_t>(s.panelCount));
}

bool ReadSection(std::istream& in, SectionProfileSpec& s)
{
    std::uint64_t panels = 0;
    const bool ok = ReadPod(in, s.maxThicknessFraction)
        && ReadPod(in, s.maxThicknessPosition)
        && ReadPod(in, s.maxCamberFraction)
        && ReadPod(in, s.maxCamberPosition)
        && ReadPod(in, s.brakeChordFraction)
        && ReadPod(in, s.fullBrakeDeflectionRad)
        && ReadPod(in, s.brakeBlendChordFraction)
        && ReadPod(in, s.inletChordFraction)
        && ReadPod(in, panels);
    s.panelCount = static_cast<std::size_t>(panels);
    return ok;
}

void WriteComputed(std::ostream& out, const ComputedPolarSpec& s)
{
    WriteSection(out, s.section);
    WritePod(out, s.reynoldsNumber);
    WritePod(out, s.aspectRatioForPostStall);
    WritePod(out, s.stallBlendWidthRad);
    WritePod(out, s.reattachmentHysteresisRad);
    WritePod(out, s.sweepLowRad);
    WritePod(out, s.sweepHighRad);
    WritePod(out, static_cast<std::uint64_t>(s.brakeSamples));
}

bool ReadComputed(std::istream& in, ComputedPolarSpec& s)
{
    std::uint64_t brakes = 0;
    const bool ok = ReadSection(in, s.section)
        && ReadPod(in, s.reynoldsNumber)
        && ReadPod(in, s.aspectRatioForPostStall)
        && ReadPod(in, s.stallBlendWidthRad)
        && ReadPod(in, s.reattachmentHysteresisRad)
        && ReadPod(in, s.sweepLowRad)
        && ReadPod(in, s.sweepHighRad)
        && ReadPod(in, brakes);
    s.brakeSamples = static_cast<std::size_t>(brakes);
    return ok;
}

void WriteAnalytic(std::ostream& out, const AnalyticPolarSpec& s)
{
    WritePod(out, s.thicknessFraction);
    WritePod(out, s.camberFraction);
    WritePod(out, s.flapChordFraction);
    WritePod(out, s.fullBrakeDeflectionRad);
    WritePod(out, s.minimumDragCoefficient);
    WritePod(out, s.dragRiseFactor);
    WritePod(out, s.stallMarginRad);
    WritePod(out, s.stallBlendWidthRad);
    WritePod(out, s.stallMarginBrakeLoss);
    WritePod(out, s.aspectRatioForPostStall);
    WritePod(out, s.reattachmentHysteresisRad);
}

bool ReadAnalytic(std::istream& in, AnalyticPolarSpec& s)
{
    return ReadPod(in, s.thicknessFraction)
        && ReadPod(in, s.camberFraction)
        && ReadPod(in, s.flapChordFraction)
        && ReadPod(in, s.fullBrakeDeflectionRad)
        && ReadPod(in, s.minimumDragCoefficient)
        && ReadPod(in, s.dragRiseFactor)
        && ReadPod(in, s.stallMarginRad)
        && ReadPod(in, s.stallBlendWidthRad)
        && ReadPod(in, s.stallMarginBrakeLoss)
        && ReadPod(in, s.aspectRatioForPostStall)
        && ReadPod(in, s.reattachmentHysteresisRad);
}

// The canonical cold solve. Zero brake, zero incidence, both surfaces fully
// attached - which is exactly how `SectionPolarTable::Computed` seeds the
// downward branch of its sweep, so this is reproducible rather than
// approximately reproducible.
//
// It is the check that does not depend on anybody remembering anything. If the
// viscous solver, the boundary layer, the profile geometry or the panelling
// changes, this number changes, and the cache is refused.
struct Witness
{
    double lift = 0.0;
    double drag = 0.0;
    double moment = 0.0;
};

Witness SolveWitness(const ComputedPolarSpec& spec)
{
    const SectionProfile profile = BuildSectionProfile(spec.section, 0.0);
    const SectionViscousSolver solver(profile, spec.reynoldsNumber);
    const SectionAerodynamics flow = solver.Solve(0.0, 1.0, 1.0);
    return Witness{flow.liftCoefficient, flow.dragCoefficient,
                   flow.momentCoefficient};
}

bool WitnessAgrees(const Witness& a, const Witness& b)
{
    // Tight, because this is a deterministic solve of the same inputs on the
    // same machine and it should reproduce to the last few bits. Loose enough
    // not to trip on a different optimiser reassociating a sum.
    constexpr double Tolerance = 1.0e-9;
    const auto close = [](double x, double y)
    {
        return std::fabs(x - y) <= Tolerance * (1.0 + std::fabs(x));
    };
    return close(a.lift, b.lift) && close(a.drag, b.drag)
        && close(a.moment, b.moment);
}

// Not a security digest - it names a file. The CONTENT decides whether the
// file is trusted, so a collision here costs a spec comparison, not a wrong
// answer.
std::uint64_t SpecDigest(const ComputedPolarSpec& spec)
{
    std::uint64_t hash = 1469598103934665603ull;
    const auto mix = [&hash](double value)
    {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        for (int byte = 0; byte < 8; ++byte)
        {
            hash ^= (bits >> (byte * 8)) & 0xffull;
            hash *= 1099511628211ull;
        }
    };
    mix(spec.section.maxThicknessFraction);
    mix(spec.section.maxThicknessPosition);
    mix(spec.section.maxCamberFraction);
    mix(spec.section.maxCamberPosition);
    mix(spec.section.brakeChordFraction);
    mix(spec.section.fullBrakeDeflectionRad);
    mix(spec.section.brakeBlendChordFraction);
    mix(spec.section.inletChordFraction);
    mix(static_cast<double>(spec.section.panelCount));
    mix(spec.reynoldsNumber);
    mix(spec.aspectRatioForPostStall);
    mix(spec.stallBlendWidthRad);
    mix(spec.reattachmentHysteresisRad);
    mix(spec.sweepLowRad);
    mix(spec.sweepHighRad);
    mix(static_cast<double>(spec.brakeSamples));
    return hash;
}
}

// The one place allowed inside a polar table's arrays, named in the table's
// own friend declaration. Everything above is format; this is the only code
// that touches the table itself.
struct SectionPolarCacheAccess
{
    static void Write(std::ostream& out, const SectionPolarTable& table)
    {
        WritePod(out, static_cast<std::uint32_t>(table.Source));
        WritePod(out, static_cast<std::uint64_t>(table.AlphaCount));
        WritePod(out, static_cast<std::uint64_t>(table.BrakeCount));
        WritePod(out, table.AlphaMinRad);
        WritePod(out, table.AlphaMaxRad);
        WritePod(out, table.BlendWidthRad);
        WritePod(out, table.ReattachmentRad);
        WriteAnalytic(out, table.SpecValue);
        WriteDoubles(out, table.ZeroLiftByBrake);
        WriteDoubles(out, table.SlopeByBrake);
        WriteDoubles(out, table.StallByBrake);
        WriteDoubles(out, table.MaximumLiftByBrake);
        WriteSamples(out, table.Samples);
        WriteSamples(out, table.Attached);
        WriteSamples(out, table.Separated);
        WriteDoubles(out, table.SeparationCurve);
    }

    // Returns false for a file that cannot be parsed OR that parses into a
    // table whose arrays do not agree with its own declared shape - a
    // truncation that happens to land on a valid length must not become a
    // table with the wrong dimensions.
    static bool Read(std::istream& in, const ComputedPolarSpec& spec,
                     SectionPolarTable& table)
    {
        SectionPolarTable loaded;
        std::uint32_t source = 0;
        std::uint64_t alphaCount = 0;
        std::uint64_t brakeCount = 0;
        if (!ReadPod(in, source) || !ReadPod(in, alphaCount)
            || !ReadPod(in, brakeCount)
            || !ReadPod(in, loaded.AlphaMinRad)
            || !ReadPod(in, loaded.AlphaMaxRad)
            || !ReadPod(in, loaded.BlendWidthRad)
            || !ReadPod(in, loaded.ReattachmentRad)
            || !ReadAnalytic(in, loaded.SpecValue)
            || !ReadDoubles(in, loaded.ZeroLiftByBrake)
            || !ReadDoubles(in, loaded.SlopeByBrake)
            || !ReadDoubles(in, loaded.StallByBrake)
            || !ReadDoubles(in, loaded.MaximumLiftByBrake)
            || !ReadSamples(in, loaded.Samples)
            || !ReadSamples(in, loaded.Attached)
            || !ReadSamples(in, loaded.Separated)
            || !ReadDoubles(in, loaded.SeparationCurve))
        {
            return false;
        }

        loaded.ComputedValue = spec;
        loaded.Source = static_cast<PolarProvenance>(source);
        loaded.AlphaCount = static_cast<std::size_t>(alphaCount);
        loaded.BrakeCount = static_cast<std::size_t>(brakeCount);

        const std::size_t expected = loaded.AlphaCount * loaded.BrakeCount;
        if (expected == 0 || loaded.Samples.size() != expected
            || loaded.Attached.size() != expected
            || loaded.Separated.size() != expected
            || loaded.SeparationCurve.size() != expected
            || loaded.ZeroLiftByBrake.size() != loaded.BrakeCount
            || loaded.SlopeByBrake.size() != loaded.BrakeCount
            || loaded.StallByBrake.size() != loaded.BrakeCount
            || loaded.MaximumLiftByBrake.size() != loaded.BrakeCount)
        {
            return false;
        }

        table = std::move(loaded);
        return true;
    }
};

const char* SectionPolarCacheMissName(SectionPolarCacheResult::Miss miss)
{
    switch (miss)
    {
    case SectionPolarCacheResult::Miss::None: return "hit";
    case SectionPolarCacheResult::Miss::NoFile: return "no file";
    case SectionPolarCacheResult::Miss::Unreadable: return "unreadable";
    case SectionPolarCacheResult::Miss::BadFormat: return "bad format";
    case SectionPolarCacheResult::Miss::SpecMismatch: return "spec mismatch";
    case SectionPolarCacheResult::Miss::WitnessMismatch:
        return "witness mismatch - the solver changed, not the spec";
    }
    return "unknown";
}

std::string SectionPolarCacheDirectory()
{
    if (const char* override = std::getenv("PARAPENTING_POLAR_CACHE"))
        return std::string(override);
    return "Intermediate/PolarCache";
}

std::string SectionPolarCachePath(const ComputedPolarSpec& spec)
{
    const std::string directory = SectionPolarCacheDirectory();
    if (directory.empty()) return {};
    char name[64];
    std::snprintf(name, sizeof(name), "/section-%016llx.polar",
                  static_cast<unsigned long long>(SpecDigest(spec)));
    return directory + name;
}

SectionPolarCacheResult LoadSectionPolarTable(
    const ComputedPolarSpec& spec, SectionPolarTable& table)
{
    SectionPolarCacheResult result;
    const std::string path = SectionPolarCachePath(spec);
    if (path.empty())
    {
        result.miss = SectionPolarCacheResult::Miss::NoFile;
        return result;
    }

    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        result.miss = SectionPolarCacheResult::Miss::NoFile;
        return result;
    }

    char magic[8] = {};
    std::uint32_t version = 0;
    if (!ReadPod(in, magic) || std::memcmp(magic, Magic, sizeof(Magic)) != 0
        || !ReadPod(in, version) || version != FormatVersion)
    {
        result.miss = SectionPolarCacheResult::Miss::BadFormat;
        return result;
    }

    ComputedPolarSpec stored;
    if (!ReadComputed(in, stored))
    {
        result.miss = SectionPolarCacheResult::Miss::Unreadable;
        return result;
    }
    if (!(stored == spec))
    {
        result.miss = SectionPolarCacheResult::Miss::SpecMismatch;
        return result;
    }

    Witness storedWitness;
    if (!ReadPod(in, storedWitness.lift) || !ReadPod(in, storedWitness.drag)
        || !ReadPod(in, storedWitness.moment))
    {
        result.miss = SectionPolarCacheResult::Miss::Unreadable;
        return result;
    }
    // The check that costs a section solve and buys the whole design. Done
    // BEFORE the bulk read, so a stale cache is cheap to reject.
    if (!WitnessAgrees(storedWitness, SolveWitness(spec)))
    {
        result.miss = SectionPolarCacheResult::Miss::WitnessMismatch;
        return result;
    }

    if (!SectionPolarCacheAccess::Read(in, stored, table))
    {
        result.miss = SectionPolarCacheResult::Miss::BadFormat;
        return result;
    }

    result.hit = true;
    result.miss = SectionPolarCacheResult::Miss::None;
    return result;
}

bool SaveSectionPolarTable(
    const ComputedPolarSpec& spec, const SectionPolarTable& table)
{
    const std::string path = SectionPolarCachePath(spec);
    if (path.empty()) return false;

    std::error_code ec;
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path(), ec);

    // Written to a temporary and renamed, so a process killed mid-write leaves
    // the old cache rather than a half file that parses.
    const std::string temporary = path + ".partial";
    {
        std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
        if (!out) return false;

        out.write(Magic, sizeof(Magic));
        WritePod(out, FormatVersion);
        WriteComputed(out, spec);

        const Witness witness = SolveWitness(spec);
        WritePod(out, witness.lift);
        WritePod(out, witness.drag);
        WritePod(out, witness.moment);

        SectionPolarCacheAccess::Write(out, table);
        if (!out) return false;
    }

    std::filesystem::rename(temporary, path, ec);
    return !ec;
}
}
