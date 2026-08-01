#!/usr/bin/env bash
# Compile the Unreal module and run the engine-independent physics suite.
#
# The CMake suite in Tests/ builds each Physics/*.cpp as its own translation
# unit. That is exactly the configuration in which a unity-build name
# collision is invisible, so it once let the module stay broken for hours
# while the tests stayed green. Always run both.
#
#   Tools/check-build.sh            # module + tests
#   Tools/check-build.sh --module   # module only (fast)
#   Tools/check-build.sh --tests    # tests only
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT="$ROOT/Parapenting.uproject"
BUILD_DIR="${PARAPENTING_BUILD_DIR:-$ROOT/Intermediate/PhysicsTests}"

ENGINE="${UE_ROOT:-/Users/Shared/Epic Games/UE_5.8}"
BUILD_SH="$ENGINE/Engine/Build/BatchFiles/Mac/Build.sh"

run_module() {
    if [[ ! -x "$BUILD_SH" ]]; then
        echo "error: engine not found at $ENGINE" >&2
        echo "       set UE_ROOT to your Unreal install" >&2
        return 1
    fi
    echo "==> building ParapentingEditor"
    "$BUILD_SH" ParapentingEditor Mac Development \
        -Project="$PROJECT" -waitmutex
}

run_tests() {
    echo "==> building and running physics tests"
    cmake -S "$ROOT/Tests" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build "$BUILD_DIR" -j"$(sysctl -n hw.ncpu)"
    "$BUILD_DIR/parapenting_physics_tests"
    "$BUILD_DIR/parapenting_determinism_tests"
    "$BUILD_DIR/parapenting_payload_tests"
    "$BUILD_DIR/parapenting_aerodynamics_tests"
    "$BUILD_DIR/parapenting_pressure_tests"
    "$BUILD_DIR/parapenting_membrane_tests"
    "$BUILD_DIR/parapenting_collapse_tests"
    # Level 7. Ten minutes of flight at 120 Hz plus a brake sweep, so this one
    # takes minutes rather than seconds - it is the whole stack running.
    "$BUILD_DIR/parapenting_coupled_tests"
    (cd "$ROOT" && "$BUILD_DIR/parapenting_geometry_tests" \
        Data/Wings/bgd-epic-2-ml-geometry.json)
    (cd "$ROOT" && "$BUILD_DIR/parapenting_suspension_tests" \
        Data/Wings/bgd-epic-2-ml-lineplan.json)
    # Run from the project root: the survey test reads the provenance files
    # and both surveyed heightfields. One argument per region.
    (cd "$ROOT" && "$BUILD_DIR/parapenting_terrain_survey_tests" \
        Content/Terrain/interlaken.provenance.json \
        Content/Terrain/grindelwald.provenance.json)
}

case "${1:-all}" in
    --module) run_module ;;
    --tests)  run_tests ;;
    all)      run_module && run_tests ;;
    *) echo "usage: $0 [--module|--tests]" >&2; exit 2 ;;
esac

echo "==> ok"
