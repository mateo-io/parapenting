# Parapenting current-state handoff

Last updated: 2026-07-30, Europe/Amsterdam.

This is the restart point for future work. Treat the worktree and test output
as authoritative if this document ever disagrees with source.

## Product direction

Parapenting is an Unreal Engine 5.8 paraglider simulator targeting Apple
Silicon macOS first, with scalable architecture for a universal Mac and later
Windows release. Physics fidelity has priority over final art. Switzerland is
the first region, centered on Interlaken and Grindelwald.

The default equipment intent is:

- 85 kg pilot;
- seated GH/ABS-style harness;
- low-B wing sized by all-up takeoff mass, not body mass alone;
- keyboard and PlayStation-style controller support from the start.

The simulator must never imply that a manufacturer wing, weather mode, site,
or safety assessment is certified or operationally valid without external
validation.

## Controls

Keyboard flight controls use five persistent steps:

- `A` / `D`: weight shift one step left/right;
- `Left` / `Right`: add one step of corresponding brake;
- `Down`: add one step to both brakes;
- `Up`: release both brakes one step;
- a side arrow held with `Up`: release only that side;
- `Q`: cycle all wings;
- `5`: Training A;
- `6`: BGD EPIC 2 ML research profile;
- `7`: Sport B;
- `8`: ADVANCE EPSILON DLS 28 research profile;
- `1`–`4`: chill, ridge, localized rotor, rotor everywhere;
- `R`: reset;
- `N`: prepare forward ground launch;
- `M`: prepare reverse ground launch;
- `Space`: launch run;
- `Z` / `X` / `C`: left asymmetric, frontal, right asymmetric training event.

Controller support:

- left stick X: proportional weight shift;
- L2 / R2: proportional left/right brake;
- right stick forward: speedbar;
- bottom face button: reset;
- top face button: cycle wing.

See `README.md` and `Config/DefaultInput.ini` for the complete command set.

## Implemented and verified

### Core flight

- deterministic 120 Hz engine-independent dynamics;
- 16-panel canopy with local airspeed, incidence, lift, drag and loading;
- pilot pendulum and three harness profiles;
- compliant accelerator and A/B/C/D riser load transfer;
- five-point per-wing brake polars;
- airspeed-, loading-, pressure- and collapse-dependent brake force;
- flexible high-load deformation, load softening and overspeed drag;
- pitch/surge behavior, deep stall, spin, asymmetric/frontal collapse,
  cravat, pumping, over-brake inhibition and pressure-gated surge checking;
- stateful ground effect, finite flare energy, touchdown and run-out/fall/
  settle dynamics.

### Wings and equipment

Six profiles currently ship:

1. Training A 28;
2. Alpine A+ 27 research envelope;
3. BGD EPIC 2 ML research profile;
4. XC B 26 research envelope;
5. Sport B 26;
6. ADVANCE EPSILON DLS 28 research profile.

Existing enum values were preserved and the EPSILON was appended, maintaining
old replay compatibility. BGD and ADVANCE published facts are separated from
simulator-estimated handling in:

- `Data/Wings/bgd-epic-2-ml-research.json`;
- `Data/Wings/advance-epsilon-dls-28-research.json`.

Per-wing research parameters now include collapse resistance, passive/brake/
pump reinflation, frontal reopening, cravat susceptibility and recovery-surge
energy. The training A is deliberately most forgiving; Sport B is most
dynamic. These are not manufacturer measurements.

### Atmosphere and weather

- chill, ridge, localized rotor and global rotor modes;
- five named weather presets;
- finite deterministic thermal lifecycles;
- expanding, drifting, meandering cores with convergence and sink rings;
- cloud-base/inversion caps;
- ridge lift, boundary layer and lee rotor;
- terrain-normal anabatic/katabatic circulation;
- replay-safe diurnal solar/heating/cloud cycle;
- deterministic three-band turbulence;
- centre/left/right canopy airflow sampling;
- optional Open-Meteo model-wind ingestion with source age and offline cache.

Real weather is contextual simulation input, not a flight forecast.

### Terrain and Swiss routes

- surveyed Interlaken terrain path and reproducible swissALTI3D pipeline;
- tiled procedural terrain rendering, lakes, rivers, roads, fields, forests,
  settlements, snow and rock classification;
- Amisbühl, Bergbo, Hohwald and Niederhorn routes to Lehn/Höhematte;
- source-backed Grindelwald First routes to Grund and Bodmi;
- landing windsocks and wind-oriented circuits;
- route provenance, hazards, wind envelopes and pre-flight suitability.

Important limitation: the sparse Grindelwald lane remains an anchor-matched
analytic proxy. Licensed regional elevation/orthophoto authoring, virtual
textures and World Partition remain unfinished.

### Presentation and game systems

- pressure-responsive double-surface canopy with closed tips, cell thickness,
  collapse deformation, suspension lines and load shading;
- articulated seated pilot/harness;
- procedural alpine atmosphere, volumetric cloud field, sunlight and shadows;
- Low/Medium/High/Epic Metal graphics profiles;
- three cameras with deterministic inertial, rotor, collapse and surge motion;
- stereo procedural wind, vario, fabric, brake-line, thermal and surge audio;
- controller haptics and motion-reduction accessibility profiles;
- expanded/compact/minimal HUD;
- eight repeatable training exercises with scoring and coaching;
- replay library, ghost, CSV export, debrief and persistent progression.

The detailed checklist is `docs/V1_V4_ROADMAP.md`.

## Latest verified test state

Before the sensory-feedback work described below:

```sh
cmake -S Tests -B build/tests
cmake --build build/tests -j 8
./build/tests/parapenting_physics_tests
./build/tests/parapenting_regression_matrix
```

Results:

- all physics tests passed;
- all 60 ten-minute deterministic matrix runs passed;
- matrix scope is 6 wings × 5 weather cases × 2 identical repetitions;
- no run spent time above 5 g;
- all state remained finite and repeated runs matched exactly.

## Work in progress at interruption

The current unfinished pass is improving the sensory moment when collapsed
cells reopen.

Already edited but **not yet rebuilt or regression-tested**:

- `Telemetry` gained left/right/frontal reinflation rates per second;
- `ParagliderDynamics.cpp` computes those rates from each 120 Hz step;
- `CameraFeedback.cpp` uses reinflation rate for a short vertical/pitch/FOV
  kick and side-specific roll/yaw;
- `HapticFeedback.cpp` adds side-specific reopening texture and a frontal cue.

Still required to finish this pass:

1. Add reinflation-rate inputs/outputs to `AudioFeedback`.
2. Route them through `UParaglidingAudioComponent` and add a short localized
   fabric-opening envelope.
3. Add reinflation-rate columns to telemetry CSV export.
4. Add deterministic tests proving:
   - no reopening cue during stable flight;
   - left reopening produces stronger left haptics/audio;
   - frontal reopening produces a symmetric cue;
   - reduced-motion accessibility scales camera kick;
   - repeated inputs return identical cues.
5. Rebuild headless tests and rerun the full 60-flight matrix.
6. Update audio/camera/collapse documentation.

Do not assume this interrupted sensory pass compiles until those checks pass.

## Mac build state

Unreal Engine path:

```text
/Users/Shared/Epic Games/UE_5.8
```

Editor build command:

```sh
"/Users/Shared/Epic Games/UE_5.8/Engine/Build/BatchFiles/Mac/Build.sh" \
  ParapentingEditor Mac Development \
  "/Users/pachosky/projects/parapenting/Parapenting.uproject" \
  -WaitMutex -NoHotReloadFromIDE
```

An older app exists at:

```text
build/package/Mac/Parapenting.app
```

Its directory timestamp is 2026-07-30 15:28:31. It predates the latest wing,
landing-rollout and collapse/recovery changes and must not be described as the
latest build.

**We have stopped attempting module builds for now**, deliberately. The quota
below makes every attempt a waste of time, so engine-side work is being
committed unverified and the debt is tracked in one place:
[`docs/PHYSICS_TODO.md`](PHYSICS_TODO.md), under "UNCOMPILED ENGINE CHANGES",
which lists each affected commit and what to smoke-test once a build is
possible. Do not add engine changes without adding them to that table.

External Unreal build/cook/package execution was quota-blocked until roughly
2026-08-05 13:55. When available:

1. compile the editor target;
2. launch and smoke-test the Amisbühl → Lehn route;
3. test keyboard and a PlayStation-style controller;
4. cook/package the latest Mac app;
5. inspect logs for Metal/shader errors;
6. verify packaged startup on Apple Silicon;
7. later complete universal signing/notarization and Windows QA.

## Remaining release gates

The v1–v4 goal is not complete until these are addressed:

- external SIV/pilot validation of pitch, brake pressure, collapses and recovery;
- manufacturer/importer cooperation for licensed real-wing identity, geometry,
  polar and maneuver data;
- licensed regional terrain/orthophoto integration and scalable World Partition;
- runtime visual QA of the newest source in Unreal on Mac;
- performance profiling across representative Apple Silicon machines;
- current standalone Mac package;
- signed/notarized universal Mac release;
- Windows build and platform QA.

## Recommended next order

1. Finish and validate the interrupted reinflation sensory pass.
2. Accept the user's new feedback and translate it into concrete physics/UI/
   world tasks.
3. Improve terrain fidelity and streaming around the selected routes.
4. Perform the latest Unreal compile and Mac runtime smoke test.
5. Iterate handling from actual pilot feedback and recorded telemetry.
6. Produce release-quality Mac and Windows builds only after validation.

