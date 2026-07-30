# Parapenting v1–v4 roadmap

The simulator grows in validated vertical slices. Every version must retain a
launchable Mac build, deterministic physics tests and the v0 control scheme.

Status legend: `[x]` implemented and regression-tested, `[~]` working prototype,
`[ ]` planned.

## v1 — Air and wing feel

- [x] Structured thermals: finite source cycles, altitude-expanding/drifting
  and coherently meandering plumes, low-level convergence, sink rings,
  alternating edge circulation and weather-driven cloud-base/inversion caps
- [x] Toggleable sampled 3D airflow field showing wind, lift, sink and rotor
- [x] Terrain boundary layer, ridge lift and localized lee rotor
- [x] Terrain-normal-driven anabatic and katabatic slope circulation with
  signed surface heating, altitude decay and a playable evening-drainage preset
- [x] Replay-safe accelerated local-time cycle coupling solar angle, surface
  heating, thermal activity, cloud development, daylight and warm horizon light
- [x] Advected deterministic three-band turbulence spectrum with
  terrain-height decay, canopy-scale gradients and frequency-separated sensory
  feedback
- [x] Deterministic centre/left/right canopy airflow sampling, so thermal
  edges and rotor boundaries drive side-specific unloading and collapse onset
- [x] Six data-driven comparison wings spanning EN-A training, A+, two low-B
  research calibrations, mid-B XC and high-B sport envelopes, with published
  BGD/ADVANCE facts separated from simulator approximations
- [x] Five-point per-wing brake polars used by all spanwise panels, with
  steady-glide calibration checks for trim, sink, glide and deep-brake drag
- [x] Stateful span/height-aware ground effect and finite flare-energy reserve,
  with early-flare depletion, pressure/collapse authority loss and telemetry
- [x] Deterministic post-touchdown runout/fall/settle dynamics with live pilot
  motion, brake-sensitive balance, canopy deflation and delayed mastery scoring
- [x] Airspeed-, loading-, pressure- and wing-dependent nonlinear brake force
  in newtons, realistic 520–720 mm travel and collapse-side unloading
- [x] Aerodynamic asymmetric/frontal onset from incidence and dynamic-pressure
  loss, gust-sided folds, pressure loss, recovery surge and pumping
- [x] Per-wing collapse resistance, asymmetric/frontal reinflation, cravat
  susceptibility and recovery-surge envelopes with telemetry and regressions
- [x] Vario/wind/canopy audio, scenario selector and recorded telemetry

Exit: ten-minute flights remain finite; each wing has measurably distinct polar
and turn response; thermal and collapse exercises are repeatable by seed.

## v2 — Switzerland

- [x] Route catalogue backed by verified site metadata, source provenance,
  landing circuits, launch hazards and route-specific simulator wind envelopes
- [x] Optional physical ground-launch sequence with terrain slope, apparent
  wind, canopy inflation, overhead stabilization, crosswind drift, pilot run,
  brake abort and continuous transition into airborne dynamics
- [x] Explicit reverse-launch presentation and mechanics with pilot-facing
  separation, wing-relative brake continuity, centered-wing gate, neutralized
  turn input, finite turn-under-wing phase and premature-commit prevention
- [x] Playable selection for Amisbühl, Bergbo, Hohwald and Niederhorn to
  Lehn/Höhematte, plus source-backed Grindelwald First routes to Grund/Bodmi
- [x] Reproducible current swissALTI3D catalogue/download pipeline, shared
  surveyed heightfield and per-tile provenance
- [~] Regional terrain split into 128 independently culled sub-25 m tiles with
  curvature/aspect/strata alpine classification and testable geometry budgets;
  Interlaken is surveyed while the sparse Grindelwald lane is an anchor-matched
  analytic proxy; licensed regional elevation/orthophoto authoring, virtual
  textures and World Partition remain
- [x] Mixed procedural forests, settlements, all landing markers, windsock,
  parcel boundaries, kilometre-scale lake, river/road navigation cues, fields,
  alpine rock/snow classification and surveyed terrain mesh
- [x] Per-landing windsocks driven by locally sampled boundary-layer wind and
  gusts, with deterministic direction, inflation, droop and flutter
- [x] Wind-oriented landing circuits for Lehn and Höhematte, including
  downwind/base/final gates, surface-flow circuit reversal, stabilized-final
  coaching and approach-aware landing scores
- [x] Atmospheric perspective, volumetric alpine clouds, cloud shadows,
  softened sunlight and terrain exposure/detail shading; cloud base, depth,
  coverage, drift and shadow strength now follow deterministic thermal
  lifecycles
- [x] Pressure-responsive double-surface canopy geometry with lower skin,
  closed tips, dynamic cell thickness, collapse shading and suspension fan
- [x] Runtime-selectable, persistent Low/Medium/High/Epic Metal graphics
      profiles for Apple Silicon (`F10`), isolated from 120 Hz flight physics

Exit: geographic anchors and elevation are traceable; routes stream without
hitches; landing areas and major hazards are recognizable.

## v3 — Advanced canopy simulation

- [x] Sixteen spanwise aerodynamic panels with evolving panel-local airspeed,
  angle of attack, lift/drag and aerodynamic roll damping, now driven by
  interpolated centre-to-tip atmospheric velocity gradients
- [x] Compliant accelerator travel, A/B/C/D riser load transfer, harness inertia
  and pilot pendulum
- [x] Wing-class flexible high-load softening separated from the EN 926-1
  structural boundary, with deformation geometry/feedback, telemetry and
  load-duration regressions
- [x] Restored aerodynamic pitch stability and wing-class overspeed drag rise,
  preventing persistent post-spiral dives and rigid-wing-like 30+ m/s equilibria
- [~] Pitch/surge energy exchange, deep stall, spin, frontal, cravat and
  cascade, now with aerodynamic unloading onset, rate-sensitive pumping,
  over-brake inhibition and pressure-gated surge checks; external maneuver
  validation remains
- [x] Eight-exercise active-piloting curriculum, including load-aware spiral
  exit coaching, a local-wind-aligned flare/run-out final, deterministic event
  injection and exact 120 Hz control/setup replay
- [x] Independent meteorological wind controls, offline snapshot application,
  five named weather presets and authored thermal/sink/rotor volumes
- [x] On-demand Open-Meteo Interlaken model-wind ingestion with surveyed-frame
  direction conversion, source/age labeling, validation and offline cache;
  explicitly not an operational aviation forecast

Exit: maneuvers conserve plausible energy, recoveries are technique-sensitive,
and failure states are covered by regression envelopes.

## v4 — Validation and product

- Manufacturer/importer collaboration for licensed geometry and polar data
- [x] Pilot mass, three harness dynamics profiles, ballast, S/M/L wing
  geometry/loading and short/standard/long brake-travel setup
- [~] Deterministic independent left/right controller feedback for brake load,
  rotor texture, unloading, collapse onset, cravat and recovery surge;
  persistent full/comfort/minimal-motion accessibility profiles and three
  preset keyboard layouts plus persistent conflict-safe capture for all six
  primary flight controls; general command remapping and VR investigation
  remain
- [x] Physics-driven stereo wind/fabric/line audio with thermal breath,
  unloading hiss, brake-line tension, cravat localization, asymmetric
  collapse cues, vario and recovery-surge rush; deterministic multi-frequency rotor,
  unloading, load, collapse, deep-stall and surge camera response across
  three view modes and all accessibility profiles
- [x] Articulated low-poly seated pilot/harness presentation with independent
  brake-hand travel, force-responsive elbows, weight shift, pendulum motion
  and collapse/recovery response through a deterministic pose contract
- [x] Persistent compact/expanded/minimal flight HUD hierarchy with
  unobstructed default instrumentation, safety-critical incident overrides and
  chase framing that keeps pilot and canopy movement readable together
- [x] Deterministic flight-lab scoring and live coaching for thermal centering,
  rotor handling, collapse recovery and landing quality, with session bests
- [x] 120 Hz replay plus a 10 Hz in-world canopy ghost trajectory, exportable
  control/pose CSV analytics and persistent per-scenario personal bests
- [x] Engine-independent phase-aware flight debrief with de-duplicated
  incident events, energy/thermal/rotor/approach metrics, category ratings,
  focused practice feedback, touchdown export and replay analysis sidecars
- [x] Pre-flight route/weather briefing sampling the physical launch, cruise
  and landing air fields, with thermal/cloud/rotor context, traceable
  suitability score, risk level, priority recommendation and stale-metadata
  prevention for presets and manual wind
- [x] Configured-polar flight computer with virtual route gates,
  wind/crosswind/crab solution, arrival-height reachability,
  required/available glide, speed-to-fly cues, altitude-aware waypoint capture
  and in-world/HUD navigation presentation
- [x] Versioned multi-session replay library with local discovery, validated
  metadata, browsing, deterministic playback and ghost loading
- [x] Persistent skill-based career progression from scenario bests, with
  dual-gated pilot ranks, bronze/silver/gold mastery medals and rank progress
- Signed/notarized universal Mac release plus Windows build and performance QA

Exit: external pilots and manufacturers review handling; physics assumptions and
limits are documented; distributable builds pass platform release checklists.

## Non-negotiables

- A game profile is never presented as a certified real-wing model without
  manufacturer validation.
- Current real-world airspace/site status is never inferred from simulator data.
- Every large visual or content change must preserve headless physics tests and
  packaged-app startup checks.
