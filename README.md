# Parapenting

Physics-first paraglider simulation for macOS and Windows, centered on
Interlaken and Grindelwald in the Bernese Oberland.

The authoritative implementation summary, verified commands, known gaps and
restart point are in
[`docs/CURRENT_STATE_HANDOFF.md`](docs/CURRENT_STATE_HANDOFF.md).

## Current playable flight lab

- Unreal Engine 5.8 C++ project, compiling natively on Apple Silicon
- Engine-independent C++20 flight-dynamics core
- Fixed 120 Hz simulation stepping
- Symmetric/asymmetric brake and weight-shift controls
- Aerodynamic lift, induced/profile/brake drag, stall degradation and moments
- Stateful, technique-sensitive flare reserve and span-aware ground effect
- Continuous post-touchdown runout, fall and canopy-deflation simulation
- Headless CMake tests
- Ten source-backed route pairings across Interlaken and Grindelwald
- Surveyed Interlaken terrain plus an anchor-matched Grindelwald proxy, with
  shared terrain collision and airflow
- Per-landing windsocks driven by each field's local boundary-layer wind,
  gust and terrain-circulation sample
- Wind-oriented downwind/base/final landing circuits with phase coaching,
  stabilized-approach detection and approach-aware touchdown scoring
- Optional terrain-contact ground launch with apparent-wind inflation,
  overhead check, running transition, crosswind loss and brake aborts
- Phase-aware flight debrief with distinct safety, efficiency, thermal and
  landing ratings, incident counting and automatic CSV/replay analytics
- `F5` pre-flight briefing derived from launch/cruise/landing airflow, thermal
  top, cloud margin, turbulence, rotor zones and route-specific wind limits
- Polar-aware flight computer with wind-corrected crab, waypoint arrival
  height, reachability, required/available glide and speed-to-fly guidance
- Coherent gusts, ridge lift, structured thermals, anabatic/katabatic slope
  circulation, localized lee rotor and collapses
- Continuously deforming curved canopy mesh and suspended-pilot movement
- Ten selectable route profiles with researched geographic anchors
- Stateful frontal/tip collapses, recovery surges, cravats, spins and deep stall
- Procedural vario, sink, wind and fabric audio
- Optional 10 Hz CSV telemetry for repeatable tuning and flight analysis
- Eight deterministic training scenarios (`Y`), including spiral-energy
  management and a standardized flare/run-out final, plus independent
  controller haptics
- Sixteen-strip spanwise canopy aerodynamics with panel-local airflow and
  aerodynamic roll damping
- Provenance-tracked 2025 swissALTI3D terrain shared by graphics, collision
  and airflow across the georeferenced Interlaken route frame
- Higher-density alpine land-cover rendering, mixed forest and valley
  settlement layers with Metal-scalable fog, clouds, shadows and view distance

This is a playable physics-first simulator prototype with a surveyed
swissALTI3D terrain base. Its wing models remain research calibrations rather
than manufacturer-validated certified models.

## Run the physics tests

```sh
cmake -S Tests -B build/tests
cmake --build build/tests
./build/tests/parapenting_physics_tests
./build/tests/parapenting_regression_matrix
```

## Controls

Controls use five persistent steps:

- `A` / `D`: one weight-shift step left/right
- `←` / `→`: add one brake step on that side
- `↓`: add one step to both brakes
- `↑`: release one step from both brakes
- `↑` + `←` / `→`: release one step from only that brake
- `↓` + `←` / `→`: same as the side arrow alone
- `R`: reset the flight
- `N`: prepare a full ground launch at the selected takeoff
- `Shift+N` / controller D-pad up: prepare a reverse launch
- `F5`: toggle the route and weather pre-flight briefing
- Hold `Space` / controller right face button: inflate and keep running;
  release before commitment to abort
- `[` / `]`: previous/next route (restarts at the selected launch)
- `T`: start/stop a timestamped CSV recording in `Saved/Telemetry`
- `U`: start/stop deterministic 120 Hz replay recording in `Saved/Replays`
- `P`: reset and play the last replay with its captured flight setup
- `Y`: cycle deterministic training scenarios
- `-` / `=`: decrease/increase wind by 0.5 m/s
- `,` / `.`: rotate meteorological wind direction by 15 degrees
- `W` / `S`: apply/release speedbar one step
- `H`: cycle seated, pod and lightweight harnesses
- `K` / `L`: decrease/increase pilot mass by 5 kg
- `B`: cycle 0/5/10 kg ballast
- `V`: cycle S/M/L wing size
- `J`: cycle short/standard/long brake-travel calibration
- `Tab`: cycle compact, expanded-help and minimal HUD modes
- `F9`: cycle standard, compact, right-hand and custom keyboard layouts
- `F6`: choose the flight-control action to rebind
- `F7`: capture the next free keyboard key for that action (`Esc` cancels)

Controller support is present from v0:

- Left stick X: proportional weight shift
- L2 / R2: proportional left/right brakes
- Right stick forward: proportional speedbar
- Bottom face button: reset

Air modes:

- `1`: chill
- `2`: ridge lift
- `3`: localized rotor (default)
- `4`: rotor everywhere
- `O`: cycle named morning, valley-breeze, thermal-day, foehn and
  evening-drainage presets
- `F11`: advance the scenario start time by three local hours

Wing comparison:

- `Q` / controller top face button: cycle all six research wing profiles,
  including BGD EPIC 2 ML and ADVANCE EPSILON DLS 28 calibrations
- `5`: forgiving training A
- `6`: BGD EPIC 2 ML research calibration (default)
- `7`: more dynamic sport B
- `8`: ADVANCE EPSILON DLS 28 research calibration

Deterministic recovery training:

- `Z`: left asymmetric collapse
- `X`: frontal collapse
- `C`: right asymmetric collapse

## Open on macOS

1. Install Unreal Engine 5.8 and the current Xcode/Metal toolchain.
2. Open `Parapenting.uproject`; allow Unreal to rebuild if requested.
3. Press Play. The route, sky, terrain, pawn and landing marker are generated
   by the C++ game mode, so no hand-authored level is required.
4. Package with `Platforms > Mac > Package Project`.
5. Sign and notarize public builds with an Apple Developer ID.

See [docs/V0_PLAN.md](docs/V0_PLAN.md) for scope and acceptance criteria and
[docs/TERRAIN_AND_MAC.md](docs/TERRAIN_AND_MAC.md) for the geodata and Mac
delivery pipeline. The long-term physics and Switzerland roadmap is in
[docs/MASTER_PLAN.md](docs/MASTER_PLAN.md).

Working on the physics or the world? Start with these four:

- [docs/PHYSICS_ENGINE.md](docs/PHYSICS_ENGINE.md) — what is built and what it
  is checked against
- [docs/PHYSICS_TODO.md](docs/PHYSICS_TODO.md) — every open item, what blocks
  it, what done looks like
- [docs/PHYSICS_LEARNINGS.md](docs/PHYSICS_LEARNINGS.md) — what it cost, as
  rules rather than anecdotes
- [docs/TERRAIN.md](docs/TERRAIN.md) — the coordinate frame, the surveyed
  regions, and the traps that have cost real time
