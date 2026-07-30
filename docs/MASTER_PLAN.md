# Parapenting master plan

Last reviewed: 2026-07-30

## North star

Build the most convincing consumer paraglider simulation: a native macOS and
Windows game where wing loading, harness, brake travel, terrain, wind, thermals,
rotor, collapses, launch, landing, and pilot decisions form one coherent
physical system.

The game is entertainment and simulation, not flight instruction or a source
of live aviation safety information.

## Product principles

1. Physics is measured and regression-tested before it is made spectacular.
2. The authoritative simulator is independent of Unreal rendering.
3. Every real-world wing is a versioned data package with provenance.
4. Weather presets are reproducible. Live weather is clearly time-stamped and
   never presented as an operational forecast.
5. Terrain airflow is spatial, time-varying, and debuggable—not random shaking.
6. macOS is a first-class target; Windows remains continuously buildable.
7. Graphics scale independently from physics quality.

## Default equipment profile

Initial candidate: **BGD EPIC 2**, a low EN-B wing with unusually useful public
documentation: current manual, line layout, per-size line lengths, flight
reports, certificates, geometry, trim behaviour, and brake setup guidance.

The initial player profile is:

- Pilot body mass: 85 kg
- Seated ABS/GH harness with seatboard
- Harness attachment spacing: 46 cm for a pilot above 80 kg
- Harness + reserve + helmet/clothing/instruments: provisional 12–16 kg
- Wing: approximately 5 kg depending on size
- Expected all-up mass: approximately 102–106 kg

That all-up mass likely requires an ML-class wing rather than selecting a size
from body mass alone. Final size must be chosen from the model's certified and
ideal *takeoff-weight* ranges.

The current research profile is stored in
`Data/Wings/bgd-epic-2-ml-research.json`. The ML public envelope covers
85–110 kg all-up, so the default 105 kg setup sits near the upper part of its
certified range. Published targets are 39 km/h trim, 1.0 m/s minimum sink,
9.5 best glide, 720 mm brake range at maximum all-up mass, and 70 mm brake
free play. The solver now uses a five-point research brake polar and validates
its quasi-steady trim/glide envelope; the full curve remains unverified without
manufacturer or instrumented-flight data.

A second current comparison profile is stored in
`Data/Wings/advance-epsilon-dls-28-research.json`. ADVANCE publishes 27.6 m²
flat area, 23.3 m² projected area, 11.92 m span, 5.14 aspect ratio, 47 cells,
4.35 kg wing mass, a 91–118 kg certified range and 99–113 kg ideal range for
the EPSILON DLS 28. The default 85 kg pilot setup is approximately 103.35 kg
all-up and therefore inside both ranges. ADVANCE does not publish the complete
polar or brake-force curve on the cited product data page, so all performance,
brake-feel, collapse and recovery calibration remains clearly labelled as a
simulator research approximation.

No manufacturer name, logo, exact appearance, or claim of exact handling ships
without permission. Public manuals are a starting dataset, not proof that the
simulation matches the product. A manufacturer partnership and instrumented
flight-validation data are the preferred path.

## Input architecture

### Keyboard

Five persistent steps:

- `A` / `D`: weight shift left/right
- Left/right arrows: increase the corresponding brake
- Down: increase both brakes
- Up: release both brakes
- Up + side: release only that brake

### Controller

Designed in from v0:

- Left stick X: proportional weight shift
- L2/R2: proportional left/right brake travel
- Dead-zone and response curves stored per controller profile
- PlayStation, Xbox, and generic SDL/GameInput-compatible devices
- Calibration screen records trigger minimum/maximum and stick centre
- Keyboard and controller feed the same normalized pilot-control model

Later hardware support can include two physical brake handles, load cells, VR,
and motion rigs without changing flight dynamics.

## Simulation architecture

Run authoritative physics at a fixed 120–240 Hz. Render and cloth interpolate
from snapshots.

### Wing and harness

- Canopy rigid-body translation and rotation
- Pilot/harness relative pitch and roll: coupled 8-DOF baseline
- Suspension-line geometry, tension, slack, elasticity, and drag
- Seatboard and carabiner-spacing effects on weight shift
- Spanwise aerodynamic panels using lifting-line/horseshoe-vortex methods
- Viscous/profile drag and line/harness drag
- Brake reefing mapped from physical travel to spanwise trailing-edge geometry
- Speed-system geometry
- Unsteady gust response and aerodynamic damping
- Reduced-order canopy pressure and deformation modes
- Progressive tip softness, collapses, cravats, reinflation, stalls and spins

Chaos Cloth is a visual consumer of simulation state, not the flight model.

### Atmosphere

The atmospheric field is layered:

1. Synoptic/base wind varying with altitude
2. Boundary-layer shear and surface roughness
3. Terrain deflection, acceleration, ridge lift and lee sink
4. Rotor and separation zones
5. Thermals, streets, convergence and valley flows
6. Deterministic band-limited inertial-range approximation at sub-grid scales
7. Local wakes from trees, buildings, cliffs and other wings where affordable

Terrain-flow fidelity stages:

- Analytic slope-normal flow and lee heuristics
- Precomputed multi-direction CFD/vector-field tiles
- Runtime blending by wind direction, speed and atmospheric stability
- Selected high-value sites with higher-resolution CFD and validation

Rotor must conserve plausible energy and respond to terrain/wind. It must not
be implemented as arbitrary camera shake.

## Weather modes

- **Chill:** smooth air, mild wind, broad forgiving thermals
- **Thermal day:** selectable season/time, realistic cycles and drift
- **Ridge:** laminar ridge lift with adjustable wind strength
- **Rotor everywhere:** deliberately educational/debug scenario with visualized
  turbulent zones
- **Localized rotor:** realistic lee zones only
- **Strong day:** gusts, shear, stronger cycles and active piloting
- **Recorded day:** deterministic reconstruction from archived observations
- **Live-inspired:** current observations initialize a scenario, clearly
  timestamped and frozen when the session begins
- **Custom:** wind profile, stability, thermal strength, cloud base, turbulence,
  humidity and time controls

Real weather requires licensed, documented sources and caching. It must never
suggest that the game is suitable for deciding whether to fly in real life.

## Switzerland world

### Data foundation

- swissALTI3D for terrain
- SWISSIMAGE for reference/ground texturing
- swissTLM3D and building datasets for landscape features
- SHV/FSVL-verified airspace and public flying-area information where licensed
- Official federal obstacle data plus permitted 3D enrichments
- Local club/site rules stored with source and review date

Every launch and landing record contains:

- Stable internal ID and source provenance
- Coordinates, elevation, polygon and surface slope
- Allowed orientations and usable wind sector
- Difficulty and hazards
- Landing circuit and windsocks
- Seasonal/local restrictions
- Airspace and obstacle references
- Source timestamp and last manual review

Start with Amisbühl–Lehn. Expand by complete, well-validated regions rather than
scattering low-quality sites across the country.

Candidate progression:

1. Interlaken/Beatenberg
2. Mürren–Lauterbrunnen
3. Grindelwald
4. Central Switzerland
5. Valais
6. Graubünden/Ticino

## Physics validation

Each wing package must include target envelopes rather than one magic tune:

- Trim/top speed versus all-up mass and altitude
- Complete polar curve
- Minimum sink and best glide
- Brake free-play, effective range and stall travel
- Roll, pitch and yaw natural periods/damping
- Turn rate and bank angle versus brake/weight shift
- Spiral entry and recovery
- Speed-bar travel and force
- Line loads
- Certified manoeuvre/collapse recovery envelopes

Validation sources, in increasing quality:

1. Manual, certification report, geometry and line plan
2. Manufacturer-provided design/polar data
3. Controlled calm-air flight tracks
4. Differential pressure airspeed, IMU, brake travel and line-load logging
5. Wind-tunnel or manufacturer CFD comparisons
6. Blind evaluation by multiple qualified pilots

All calibration runs become automated regression fixtures. Physics changes may
not silently improve one manoeuvre while breaking another.

## Delivery stages

### V0 — flight laboratory

- Stable Play with no crash
- Primitive visible canopy/pilot and landing marker
- Keyboard and controller control paths
- Fixed-step deterministic flight core
- HUD and telemetry
- Reset, landing detection and result
- Signed Apple Silicon development build

Exit gate: ten consecutive complete sessions without crash; automated startup,
physics and input tests pass.

### V0.1 — credible glider

- 8-DOF wing/harness dynamics
- Data-driven generic low-B profile at 102–106 kg all-up
- Calibrated polar, brake travel, pitch/roll response
- CSV/JSON telemetry and deterministic replay
- Ground collision and flare

### V0.2 — Amisbühl–Lehn

- Licensed real terrain and local metric georeferencing
- Verified launch/landing polygons
- Visual landmarks, trees, windsocks and landing circuit
- Calm and light-wind presets
- Performance presets for baseline Macs

### V0.3 — living air

- Boundary layer, ridge lift, thermals, gusts and localized rotor
- Weather-selection screen and debug airflow visualization
- Vario audio
- Scenario seeds and recorded-day playback

### V0.4 — real-wing candidate

- Manufacturer permission/partnership
- Wing-specific geometry, line and brake model
- Instrumented validation campaign
- Seated harness model and selectable all-up mass
- Speed bar and active piloting

### V0.5 — canopy incidents

- Reduced-order pressure/deformation model
- Asymmetric/frontal collapses, stalls, spins and reinflation
- Certification-inspired test suite
- Visual canopy driven by physical modes

### V1 — Switzerland early access

- Several fully reviewed Swiss regions
- Launch, landing and flight modes
- Saved flights, replay and scoring
- Mac/Windows packaging, controller calibration, accessibility and graphics
  presets
- Clear licensing, attribution, safety language and data-update process

## Immediate backlog

1. Confirm Play crash is gone on the user's interactive editor.
2. Add automated runtime test that reaches `BeginPlay`.
3. Add reset and landing/touchdown detection.
4. Add input-unit tests for five keyboard steps and controller response curves.
5. Add controller connection/calibration UI and PS controller smoke test.
6. Replace primitive coefficient model with tabulated generic low-B polar.
7. Implement separate canopy and harness states.
8. Add telemetry recording and deterministic replay.
9. Obtain exact Amisbühl/Lehn polygons and terrain tiles.
10. Approach BGD and ADVANCE about simulation data and trademark/visual use.

## Context and project tracking

Keep durable decisions in this file, route data in `Data/Sites`, wing data in a
future `Data/Wings`, and short active work in `docs/V0_PLAN.md`. Chat context is
not the source of truth. At each milestone, update these files, tests, and data
provenance so future work can resume from the repository without replaying the
entire conversation.
