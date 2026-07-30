# Geometry-Driven Paraglider Simulation Master Plan

## Purpose

Replace the current collection of high-level handling approximations with a
geometry-driven, coupled canopy–suspension–payload simulation in which the
important behavior emerges from:

- inflated cell geometry and leading-edge openings;
- flexible upper and lower surfaces, ribs, and spanwise structural coupling;
- local aerodynamic forces and flow separation;
- cell pressure and pressure loss;
- tension-only lines, cascades, risers, and brake galleries;
- a rigid harness/pilot with movable mass;
- equal-and-opposite reactions between canopy, lines, and payload.

This is a roughly **100-hour engineering plan**. It targets a deterministic,
real-time reduced-order research simulator. It does not propose real-time
Navier–Stokes CFD or claim flight-training validation.

## Guiding rules

1. Geometry is authoritative. Rendering and physics consume the same nodes,
   attachments, deformation state, and transforms.
2. No direct control-to-bank, control-to-yaw, or control-to-climb impulses in
   the final architecture.
3. Lines carry tension only. Slack lines transmit no control force.
4. Brake input changes brake-line rest length; it does not directly command a
   turn.
5. Weight shift moves payload mass and carabiners; it does not directly
   command roll.
6. Collapses arise from local unloading, pressure loss, and membrane
   deformation rather than random scripted folding.
7. Every solver level must pass still-air tests before weather is introduced.
8. Every coefficient must have a unit, source, stated estimate, and valid
   range.
9. Determinism at the fixed 120 Hz physics rate is mandatory.
10. Existing approximations are removed only after their replacements pass
    behavioral and numerical tests.

## Target architecture

```text
Pilot input
   |
   v
Harness/pilot rigid body ---- movable pilot CG
   |
   v
Carabiners and risers
   |
   v
Tension-only line/cascade graph
   |
   v
Canopy attachment nodes
   |
   +---- flexible membrane and ribs
   |          |
   |          v
   |      cell volumes and pressure
   |          |
   |          v
   +---- local aerodynamic panels
              |
              v
       forces and moments
              |
              v
 Coupled canopy/line/payload integration
```

## Level 0 — Baseline, measurements, and solver contract

**Budget: 6 hours**

### Work

- Freeze a known-good still-air EPIC 2 ML baseline.
- Record current trim speed, sink rate, glide ratio, pitch attitude, line
  tension, and numerical residuals.
- Define coordinate conventions once:
  - body forward/right/up;
  - positive roll, pitch, and yaw;
  - left/right canopy indexing;
  - chord and span fractions.
- Create SI-unit geometry and solver contracts.
- Add deterministic state hashing for replay comparison.
- Create a coefficient registry containing source, value, units, confidence,
  and calibration status.

### Deliverables

- `ParagliderCoordinateSystem.h`
- `ResearchCoefficientRegistry.{h,cpp}`
- still-air baseline CSV and golden test
- solver-state hash test

### Exit gate

- Left input, left attachment, left line, left canopy half, and left world
  trajectory agree in an end-to-end test.
- Ten minutes of hands-up still air remain deterministic and finite.

## Level 1 — Authoritative EPIC 2 geometry

**Budget: 10 hours**

### Work

- Build an explicit parametric canopy geometry:
  - cell count;
  - rib stations;
  - flat and projected span;
  - chord distribution;
  - arc;
  - upper/lower surface profiles;
  - leading-edge openings;
  - trailing edge;
  - cell and half-cell boundaries.
- Represent every A, A′, B, C/upper-D, and brake attachment node.
- Import/digitize the published line plan into a versioned data file.
- Generate both physics panels and rendered mesh from the same geometry.
- Remove duplicated procedural endpoint formulas from rendering.

### Deliverables

- `CanopyGeometry.{h,cpp}`
- `CanopyGeometryData.h`
- `Data/Wings/bgd-epic-2-ml-geometry.json`
- geometry visualizer and attachment labels
- geometry consistency tests

### Exit gate

- Every rendered line terminates on an actual rendered attachment vertex.
- Mirrored attachment pairs are geometrically symmetric.
- Published area, span, chord, line length, and riser dimensions remain within
  declared digitization tolerances.

## Level 2 — Suspension and cascade graph

**Budget: 10 hours**

### Work

- Replace grouped decorative lines with a physics graph:
  - carabiners;
  - riser endpoints;
  - lower lines;
  - cascade junctions;
  - upper galleries;
  - canopy attachments;
  - brake handles and brake fan.
- Add tension-only cable elements with rest length, stiffness, damping,
  diameter, mass, and drag.
- Solve line stretch and slack state iteratively.
- Apply equal-and-opposite line forces at both endpoints.
- Make accelerator and brake inputs change riser/line rest lengths.
- Derive visual sag from solved nodes, not an invented curve.

### Deliverables

- `SuspensionGraph.{h,cpp}`
- `TensionCableSolver.{h,cpp}`
- line-plan data importer
- line tension/slack telemetry per element

### Exit gate

- Slack elements transmit zero compressive force.
- Total endpoint reaction is equal and opposite within tolerance.
- Weight shift changes carabiner loads and attachment geometry without any
  direct roll moment.
- Brake input loads only through the brake cascade and trailing edge.

## Level 3 — Pilot, harness, and payload rigid body

**Budget: 8 hours**

### Work

- Split payload and canopy into independent rigid/deformable systems.
- Define pilot, harness, reserve, ballast, and equipment masses.
- Define payload CG and inertia tensor.
- Move pilot CG laterally and longitudinally from weight shift and body pose.
- Connect left and right carabiners at physical harness positions.
- Integrate six payload degrees of freedom.
- Preserve suspension constraints without hiding payload movement in canopy
  orientation.

### Deliverables

- `PayloadRigidBody.{h,cpp}`
- `HarnessGeometry.{h,cpp}`
- mass-property calculator
- payload pose telemetry

### Exit gate

- Pendulum period scales with suspension length.
- Increasing pilot mass changes line load and acceleration but not geometry.
- Left weight shift produces left-biased carabiner loading and a left turn
  through line/canopy reactions.

## Level 4 — Aerodynamic panel model

**Budget: 12 hours**

### Work

- Divide the canopy into span/chord aerodynamic panels.
- Compute local relative airflow including:
  - canopy translation;
  - canopy rotation;
  - panel deformation velocity;
  - local wind;
  - induced-flow approximation.
- Assign section polars by local incidence, Reynolds envelope, brake
  deformation, and pressure.
- Calculate local lift, drag, and pitching moment.
- Apply forces at panel centers rather than as one global resultant.
- Add line, riser, harness, and pilot drag at their physical locations.
- Add attached-flow and separated-flow hysteresis.

### Deliverables

- `AerodynamicPanelSolver.{h,cpp}`
- section-polar data format
- force/moment debug visualization
- local incidence/separation telemetry

### Exit gate

- Hands-up trim converges without a speed controller.
- Integrated EPIC 2 targets are held in still air:
  - trim speed near published target;
  - glide ratio near the declared target;
  - stable sink rate;
  - bounded pitch mode.
- Mirrored inputs produce mirrored panel forces and trajectories.

## Level 5 — Cell openings and pressure model

**Budget: 10 hours**

### Work

- Represent each cell or cell group with:
  - volume;
  - inlet/opening area;
  - external stagnation pressure;
  - internal pressure;
  - leakage/porosity;
  - inter-cell flow.
- Compute mass flow through leading-edge openings.
- Couple cell volume changes to pressure.
- Reduce membrane stiffness and section performance as pressure falls.
- Model inlet closure or reverse flow at adverse incidence.

### Deliverables

- `CanopyPressureSolver.{h,cpp}`
- cell-pressure state and telemetry
- opening-flow visualization
- pressure conservation and decay tests

### Exit gate

- Pressure rises during inflation and stabilizes in trim.
- Local adverse incidence causes local pressure loss.
- Closing or unloading an inlet reduces the correct cell-group pressure.
- Pressure recovery follows airflow and opening state rather than a scripted
  timer.

## Level 6 — Flexible membrane and ribs

**Budget: 14 hours**

### Work

- Build a reduced membrane mesh shared with rendering.
- Use position-based dynamics, projective dynamics, or a small implicit
  mass-spring/finite-element approximation.
- Include:
  - warp/weft stretch;
  - shear;
  - bending;
  - rib constraints;
  - trailing-edge tension;
  - pressure forces;
  - attachment-node forces.
- Add substepping and constraint iterations independent of render frame rate.
- Add numerical energy monitoring and deformation limits.
- Support asymmetric deformation without mesh inversion.

### Deliverables

- `CanopyMembraneSolver.{h,cpp}`
- membrane material data
- shared simulation/render vertex buffer
- deformation and constraint-residual telemetry

### Exit gate

- Pressurized cells hold their shape without explosive energy growth.
- Attachment loads deform the canopy smoothly.
- Released deformation oscillates and damps at a controlled rate.
- Rendering cannot detach from physics geometry.

## Level 7 — Coupled solver and apparent mass

**Budget: 10 hours**

### Work

- Define the integration order for:
  1. pilot controls and rest-length changes;
  2. atmospheric sampling;
  3. local aerodynamics;
  4. pressure;
  5. membrane constraints;
  6. lines;
  7. canopy/payload rigid motion.
- Apply equal-and-opposite reactions consistently.
- Retain an apparent-mass tensor for air accelerated with the canopy.
- Add solver iterations where pressure, membrane, and line loads must
  converge.
- Record force, moment, energy, and constraint residuals.

### Deliverables

- `CoupledParagliderSolver.{h,cpp}`
- fixed-step solver schedule
- convergence diagnostics
- deterministic replay tests

### Exit gate

- No subsystem creates unaccounted net force or moment beyond tolerance.
- Still-air trim remains stable for ten minutes.
- Brake pulse exchanges speed, height, and pitch energy without creating
  energy.
- Weight shift and brake turns emerge without direct turn moments.

## Level 8 — Emergent stall, collapse, and reopening

**Budget: 10 hours**

### Work

- Remove remaining global brake/stall thresholds where local states exist.
- Derive collapse from:
  - local incidence;
  - panel unloading;
  - pressure loss;
  - line unloading;
  - membrane buckling.
- Allow collapse propagation through shared membrane and pressure states.
- Derive cravat risk from tip geometry and line/fabric contact approximation.
- Derive reopening from pressure, airflow, membrane tension, and brake-line
  loading.
- Keep a numerical safety envelope to prevent solver explosion, clearly
  separate from flight behavior.

### Deliverables

- local collapse-state telemetry
- collapse/reopening diagnostic views
- symmetric and asymmetric incident benchmarks

### Exit gate

- Deep symmetric brake progresses from flare to separation/stall.
- Asymmetric separation produces spin/spiral behavior, not barrel rolls.
- A collapse produces genuinely slack affected lines.
- Brake pumping only affects a collapse when brake-line tension reaches the
  relevant trailing-edge region.

## Level 9 — Calibration and pilot-facing validation

**Budget: 10 hours**

### Work

- Create repeatable still-air system-identification maneuvers:
  - hands-up trim;
  - brake steps;
  - brake pulse and release;
  - weight-shift step;
  - coordinated turn;
  - spiral entry and exit;
  - symmetric stall approach;
  - asymmetric stall approach.
- Export time series for airspeed, sink, pitch, roll, yaw, brake force, line
  tension, pressure, and energy.
- Fit only bounded, identified parameters.
- Compare results against published data where available.
- Prepare a structured expert-pilot review protocol.
- Label every unvalidated behavior and coefficient.

### Deliverables

- maneuver runner and CSV exporter
- calibration report template
- pilot review questionnaire
- coefficient confidence report

### Exit gate

- Still-air quantitative gates pass.
- Experienced pilots confirm direction, control progression, pitch timing,
  pendulum timing, and stall approach qualitatively.
- Known disagreements are documented rather than tuned invisibly.

## Level 10 — Performance, visualization, and retirement of legacy paths

**Budget: 10 hours**

### Work

- Profile CPU time and memory at 120 Hz.
- Introduce solver levels of detail without changing large-scale behavior.
- Move suitable panel/membrane work to parallel tasks or compute shaders.
- Add research visualization toggles:
  - cell pressure;
  - airflow/opening flow;
  - panel force vectors;
  - separation;
  - line tension/slack;
  - membrane strain;
  - solver residuals.
- Delete superseded direct moments, scripted visual deformation, and duplicate
  geometry.
- Run full regression and long-duration stability matrices.

### Deliverables

- performance budget report
- research visualization mode
- legacy-removal checklist
- final architecture document

### Exit gate

- Fixed 120 Hz physics meets the agreed target hardware budget.
- Physics and rendering use the same geometry and state.
- No legacy direct-control force remains active.
- All still-air and incident regression suites pass.

## Known risks and mitigations

- **Level 4 gates are provisional.** Trim speed and glide ratio are first
  validated on rigid geometry; Levels 5–6 change the effective airfoil shape,
  so the still-air baseline must be re-recorded and the Level 4 gates re-run
  after the membrane solver lands. Budget this into Level 7, not as a surprise.
- **Engine integration.** The fixed 120 Hz physics rate must be decoupled from
  the Unreal tick (accumulator with fixed substeps, interpolated render
  state). Decide this in Level 0 and encode it in the solver contract;
  retrofitting determinism onto a variable tick is far more expensive.
- **Parallel operation, not big-bang cutover.** Keep the legacy handling model
  behind a runtime flag until Level 10. Every level's exit gate should be
  runnable against both models so regressions are attributable.
- **Membrane solver is the schedule risk.** 14 hours for a stable,
  non-exploding membrane with asymmetric deformation is the most optimistic
  line in the budget. If it slips, cut scope to a coarser mesh with stronger
  rib constraints rather than borrowing hours from validation (Level 9).
- **Section polars are a data problem, not a code problem.** Level 4 assumes
  usable polars for a low-Reynolds, deformable parafoil section. Identify the
  source (XFOIL runs on the digitized profile, published parafoil data, or
  both) during Level 1 while the geometry is being digitized.

## Hour budget

| Level | Workstream | Hours |
|---:|---|---:|
| 0 | Baseline and contracts | 6 |
| 1 | Authoritative canopy geometry | 10 |
| 2 | Suspension/cascade graph | 10 |
| 3 | Pilot/harness rigid body | 8 |
| 4 | Aerodynamic panel solver | 12 |
| 5 | Cell openings and pressure | 10 |
| 6 | Flexible membrane and ribs | 14 |
| 7 | Coupled integration | 10 |
| 8 | Stall, collapse, reopening | 10 |
| 9 | Calibration and pilot validation | 10 |
| 10 | Performance and legacy removal | 10 |
|  | **Total** | **110** |

The estimate deliberately exceeds 100 hours. Levels 5–8 contain the greatest
uncertainty; attempting to force them into a smaller estimate would hide the
hardest work.

## Recommended implementation order

Implement levels sequentially. Do not begin flexible collapse work before the
geometry, line graph, local aerodynamics, and pressure state exist.

The first practical milestone is Levels 0–2: one authoritative EPIC 2
geometry, physically connected attachment points, and tension-only suspension
lines. That milestone removes the current visual/physical duplication and
creates the foundation needed by every later solver.

## Validation boundary

Completing this plan would produce a substantially more physical and
inspectable real-time model. It would still not establish that the simulator
is safe for independent pilot training. That requires instrumented flight
data, controlled calibration, expert review, documented limitations, and an
appropriate training and regulatory validation process.
