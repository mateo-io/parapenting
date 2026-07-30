# Parapenting v0 plan

## Product slice

One repeatable, solo flight in calm air from Amisbühl to the Lehn landing field.
The player starts airborne just after launch. The goal is to make a controlled
approach, flare, and stop inside the landing zone.

The v0 is a simulator prototype. It is not flight instruction and must not be
represented as a substitute for training, local knowledge, or current airspace
information.

## Acceptance criteria

### Flight

- Runs physics at a fixed 120 Hz regardless of render frame rate.
- Supports independent left/right brake and weight shift.
- Stable hands-up trim flight without numerical energy gain.
- Produces a tunable speed polar with measurable trim speed, sink rate, and glide.
- Symmetric braking reduces speed and increases sink near stall.
- Asymmetric braking creates coupled roll/yaw and a sustained turn.
- Landing detection reports touchdown speed, vertical speed, and distance from
  the target.
- Flight telemetry can be saved and replayed.

### Route

- Uses terrain covering Amisbühl, Lehn, and enough surrounding Interlaken
  landmarks to navigate visually.
- Launch and landing coordinates are verified before final terrain alignment.
- Lehn landing zone has a visible windsock and target boundary.
- Initial weather is deterministic: light southeast wind, no thermals.

### macOS

- Editor and packaged game run natively on Apple Silicon.
- A Development `.app` is smoke-tested on the oldest supported test Mac.
- Release packaging is signed and notarized.
- Rendering has scalable Low/Medium/High presets; v0 targets 60 fps at 1080p on
  the chosen baseline Mac.

## Physics maturity ladder

### P0 — current reduced-order core

Single rigid flight body, nonlinear lift/drag, brake and weight-shift moments,
stall drag, fixed-step integration. This establishes controls, test machinery,
telemetry, and engine separation.

### P1 — v0 playable target

- Calibrated polar tables rather than generic coefficients
- Separate wing and pilot/harness pitch/roll states (8-DOF reduced model)
- Pendular coupling and line geometry
- Brake-dependent spanwise aerodynamic sections
- Ground contact, flare scoring, gust input

### P2 — post-v0

- Horseshoe-vortex/lifting-line span model
- Nonlinear harness aerodynamics and explicit line drag
- Terrain-aware ridge lift, thermals, shear, and turbulence
- Reduced-order canopy deformation and pressure state
- Asymmetric/frontal collapses and reinflation
- Ground handling and inflation

The Unreal cloth mesh is visual. It must consume physics state rather than act
as the authoritative aerodynamic solver.

## Work packages

1. **Bootstrap:** install Unreal, generate Xcode files, compile the current
   project, create the empty route map.
2. **Calibration harness:** CSV telemetry, automated trim/polar/turn tests,
   parameter asset for a representative EN-B wing.
3. **8-DOF dynamics:** wing and harness states, suspension constraint, control
   geometry, deterministic integrator.
4. **Terrain:** obtain licensed Swiss elevation/imagery data, project to a local
   metric coordinate system, import World Partition landscape.
5. **Playable loop:** spawn, flight HUD, landing zone, touchdown and reset.
6. **Visual proxy:** canopy/pilot meshes, lines, simple deformation, camera,
   atmospheric lighting and scalable foliage.
7. **Mac release:** performance pass, controller QA, package, sign, notarize.

## Data still needed

- Exact Amisbühl launch polygon and Lehn landing polygon
- Representative wing model, size, all-up weight and published polar data
- Terrain/imagery license and desired map radius
- Baseline Mac model for the 60 fps target
- Real-flight IGC/IMU/airspeed data for calibration

## Definition of done

V0 is done when a new player can launch the signed Mac app, fly the complete
route with keyboard or controller, land at Lehn, see a result, and replay the
same deterministic conditions—while the headless physics regression suite and
Mac smoke test both pass.

