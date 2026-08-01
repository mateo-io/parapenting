# Geometry-Driven Paraglider Simulation Master Plan

> **Revision 2.** Grounded against the published ram-air wing literature,
> current real-time deformable-solver research, and paraglider design practice.
> Restructured as a summit ladder: the lower levels are engineering, the upper
> levels are research, and the final levels are deliberately beyond the
> published state of the art. See [Ambition and the summit ladder](#ambition-and-the-summit-ladder).

## Purpose

Replace the current collection of high-level handling approximations with a
geometry-driven, coupled canopy–suspension–payload simulation in which the
important behavior emerges from:

- flat panel patterns, billow, and the inflated cell geometry they produce;
- leading-edge openings and internal cell pressure;
- flexible upper and lower surfaces, ribs, mini-ribs, and diagonal ribs;
- local aerodynamic forces, viscous boundary layers, and flow separation;
- tension-only lines, cascades, risers, and brake galleries;
- a rigid harness/pilot with movable mass;
- equal-and-opposite reactions between canopy, lines, and payload.

The target is a deterministic, real-time, reduced-order research simulator of a
BGD EPIC 2 ML. It does not propose real-time Navier–Stokes CFD, and it does not
claim flight-training validation at any level.

## Ambition and the summit ladder

This plan is intentionally longer than it can be finished. It is organised as a
ladder with named base camps, so progress is meaningful even if the summit is
never reached:

| Camp | Levels | What you have | Status |
|---|---|---|---|
| **Base camp** | 0–2 | One authoritative geometry, real suspension graph, no duplicated meshes | Achievable |
| **Camp I** | 3–5 | Emergent trim and turns from local aero and cell pressure | Achievable, hard |
| **Camp II** | 6–8 | Emergent collapse, stall, and reopening from membrane mechanics | Research-grade |
| **Camp III** | 9–11 | Calibrated, unsteady-wake, validated against certification maneuvers | At the edge |
| **Death zone** | 12–15 | Resolved turbulence, real-time two-way FSI, instrumented-flight identification | Beyond current published real-time state of the art |

Levels 12–15 are not padding. They are written out in full because knowing the
shape of the unreachable work changes how the reachable work is designed. Do
not treat failure to reach them as project failure. Treat reaching Camp II as
an outstanding result.

## Build status

Updated at the end of the Level 7 work. The engine as built is documented in
`docs/PHYSICS_ENGINE.md`; what it cost to build is in
`docs/PHYSICS_LEARNINGS.md`.

| Level | Status | Evidence |
|---|---|---|
| 0 Baseline and determinism | **Done** | state hash identical at 30/60/144/uncapped; registry live |
| 1 Manufactured geometry | **Done** | unfold residual 2.4e-15; flat span, area, projected span exact |
| 2 Suspension graph | **Done** | 254.8 m against published 254; rows A 36 / A' 11 / B 37 / C 16 |
| 3 Payload and harness | **Done** | carabiner split exact to W(1/2 ± e/s); pendulum period 2π√(L/g) |
| 4 VSM and polars | **Done, with gaps** | CL_α 0.2%, CDi 3.6%, glide 9.46 vs published 9.5 |
| 5 Cell pressure | **Done** | stagnation 5.2/9.7/14.3 deg; inlet Cp 0.97 trim, 0.89 bar |
| 6 Membrane | **Core done** | sagitta 26.32 mm vs analytic 25.99; strain 0.060% vs 0.064% |
| 7 Coupled solver | **Done** | 38.5 km/h vs published 39; glide 9.5 vs 9.5; turns mirror to 2e-8; suite green |
| 8 Emergent collapse | Not started | — |
| 9 Calibration | Not started | — |
| 10 Performance and legacy removal | Not started | — |
| 11+ | Not started | — |

**Camp I is complete. Camp II is one level short**, with Level 7 passing every
gate and Level 6 delivered at a reduced scope — strips rather than a full mesh,
and no self-collision, which is what Level 8's cravats will need.

### Carried gaps, by priority

1. **The terrain frame disagrees with the flight frame.** `RouteFrame` and the
   heightfield define +Y as route-left; the flight frame has +Y as right, and
   nothing converts. Measured: foehn rotor is 0.82 route-left against 0.15
   route-right, so lee rotor sits on the wrong side of the ridge relative to the
   surveyed geography. This blocks the **Level 0** exit gate and is the oldest
   open defect in the project. Everything from Level 9 onward that depends on
   real terrain is built on it.
2. **Section polars are analytic.** Thin-airfoil plus Viterna, no XFOIL, no
   measurement. Every flight number in Level 4 rests on theory. The plan calls
   for this work to begin during Level 1; it has not begun.
3. **Both Grindelwald routes are off the surveyed heightfield and off the
   rendered extent.** Analytic terrain puts the Grund landing at 4683 m against
   a published 950 m, in air with no thermal field. Selectable content.
4. **Level 6 is one-dimensional.** Strips at chord stations, ribs as fixed
   endpoints, no self-collision. Level 8's cravats need the self-collision.
5. **Deep stall does not converge** in the VSM solved cold, and will not: the
   separated branch has a negative lift slope, which inverts the downwash
   feedback between sections. Level 11's unsteady wake is the honest treatment.
   Locked as a known-failure check. Inside the coupled solve, with the
   separation state carried between steps, the wing does reach a fully
   separated 46-degree stall at 4.65 m/s of sink without the solve failing.
6. **Apparent-mass rotational terms are disputed** — leading coefficients
   unverified against the source paper, and 14× the existing estimate in roll.
   Registered `Disputed`; nothing uses their magnitude.
7. **Nothing geometry-driven flies the wing.** The legacy polar still does, which
   is guiding rule 11 working as intended, but it means none of Levels 1-7 has
   been exercised by a pilot.

### Recommended next steps, in order

**Level 7 is closed.** Trim was fixed first — the pressure state started packed
while the flight state started flying, and installed drag was missing from the
coupled solve — and the last failing check is now fixed too. It was in the
solver, not the harness: rotational damping integrated explicitly at eleven
times its stability limit, a damping derivative measured by dividing by whatever
rate the wing happened to have, and an aerodynamic validity gate that bounded
the force it accepted but not the moment. `docs/PHYSICS_ENGINE.md` has the
diagnosis and `docs/PHYSICS_LEARNINGS.md` sections 7, 13 and 14 have what it
teaches.

1. **Resolve the terrain/flight frame disagreement.** It is a Level 0 gate, it
   is cheap relative to what it blocks, and every later level that touches
   terrain inherits it. It moves every route's geometry, so it wants a clear run
   and a careful re-validation of the terrain suite.
2. **Start the polar acquisition.** It is a data problem with a long lead time,
   it needs XFOIL or equivalent tooling, it can run in parallel with anything,
   and it is what turns Level 4's numbers from plausible into defensible.
3. **Then Level 8**, which now has its converged Level 7 and needs
   self-collision in Level 6 before collapse can be emergent rather than
   scripted.

Levels 9 and 10 should not start before the gaps above close: calibrating an
uncoupled model, or deleting the legacy path while the geometry-driven one
cannot fly, would both be premature.

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
7. **Shape is manufactured, not sculpted.** The inflated canopy is the
   equilibrium of flat panels, seam lengths, rib profiles, internal pressure,
   and line tension — never a directly authored 3D surface.
8. Every solver level must pass still-air tests before weather is introduced.
9. Every coefficient must have a unit, source, stated estimate, and valid
   range, recorded in the coefficient registry.
10. Determinism at the fixed 120 Hz physics rate is mandatory, and the physics
    rate is decoupled from the engine tick.
11. Existing approximations are removed only after their replacements pass
    behavioral and numerical tests, and both run side by side until then.
12. **Numerical safety and flight behavior are separate systems.** Anything
    that exists only to stop the solver exploding is logged, visible, and never
    silently shapes handling.
13. **Prefer a published method with known failure modes over a novel one.**
    Every solver choice in this plan names its literature source; deviating
    from it is allowed but must be recorded as a deviation.
14. Any behavior a pilot would recognise must be traceable to a physical state,
    and the debug view must be able to show that state.

## Method selection

The original plan said what to model but not which numerical method to use.
These are the recommended choices, with rationale. Each is a decision to
confirm or overturn deliberately, not a default to drift into.

### Aerodynamics: Vortex Step Method with viscous-inviscid coupling

Not a naive strip/panel model. The Vortex Step Method (VSM) is an enhanced
nonlinear lifting-line formulation developed specifically for ram-air and
leading-edge-inflatable kites at TU Delft, and it is the right altitude for
this project:

- It handles **low aspect ratio, strong anhedral/arc, and sweep**, which is
  exactly where classical lifting-line theory fails and where a paraglider
  lives.
- It couples to **2D airfoil polars** for viscous effects rather than assuming
  thin-airfoil linearity, so stall and post-stall come from section data.
- It has been validated against RANS CFD and wind-tunnel measurement for the
  TU Delft V3 kite.
- Open-source reference implementations exist in Python and Julia
  ([awegroup/Vortex-Step-Method](https://github.com/awegroup/Vortex-Step-Method),
  [OpenSourceAWE/VortexStepMethod.jl](https://github.com/OpenSourceAWE/VortexStepMethod.jl)),
  which means the C++ port has a golden reference to test against instead of
  being validated by vibes.

A steady VSM solve per physics step is likely too expensive at 120 Hz; the
practical route is a VSM solve at a lower rate (or on geometry change) with
cheap per-step interpolation, promoted to a genuinely unsteady formulation only
at Level 11.

### Structure: Vertex Block Descent, with XPBD as the fallback

The original plan listed "PBD, projective dynamics, or small implicit FEM" as
equivalent options. They are not.

- **Vertex Block Descent (VBD)** — Chen, Liu, Yang & Yuksel, ACM TOG 43(4),
  SIGGRAPH 2024 — is a Gauss–Seidel-style block-coordinate-descent FEM solver
  that is unconditionally stable, handles stiff materials, and parallelises
  well on GPU. Independent comparison work reports it outperforming PBD as node
  count rises. **Augmented VBD (AVBD)** adds an Augmented Lagrangian treatment
  of hard constraints, which is what you want for inextensible seams and
  tension-only line attachments.
- **XPBD** remains the fallback: better documented, easier to get running, but
  its stiffness is iteration-count-dependent, which is a determinism hazard.
  If XPBD is chosen, the multigrid-accelerated global variant (MGPBD) is the
  route to mesh-independent convergence.

Recommendation: prototype in XPBD at Level 6 because it is fast to stand up,
but design the constraint API so the solver is swappable, and plan the VBD/AVBD
migration into Level 12.

### Apparent mass: Lissaman & Brown, then geometry-derived

Start with the classical ellipsoid-based apparent mass and inertia coefficients
(Lissaman & Brown, AIAA 1993-1236), parameterised on span, chord and thickness.
The literature is unambiguous that apparent mass is **not optional** for a
lightly loaded wing: it couples linear and rotational dynamics and materially
changes turn and pitch response. Replace the ellipsoid closed form with
coefficients integrated from the actual panel geometry once Level 4 is stable.

### Pressure: inlet at stagnation, internal pressure coefficient, deflation threshold

Cells are pressurised through leading-edge openings positioned near the front
stagnation point, and internal pressure must stay above a threshold or the wing
deflates. This is measurable — wind-tunnel work on dynamically inflatable wing
cells has produced differential-pressure data usable as a calibration target,
and the same instrumentation concept has been proposed as an in-flight collapse
alert system. The internal pressure coefficient is the single most important
scalar for collapse risk, so it gets first-class telemetry from Level 5 onward.

### Validation: EN 926-2 maneuvers as the behavioral benchmark

EN 926-2 defines the flight-test maneuvers a certified wing is put through:
front collapse, large asymmetric collapse at trim and at accelerated speed,
deep/parachutal stall, developed full stall and recovery, gentle spiral
(3–5 m/s sink held for one turn), and spiral dive descent capability. These are
a ready-made, published, unambiguous behavioral test matrix. The EPIC 2 has a
published certification result; reproducing its *classification* is the summit
condition of this plan.

## Target architecture

```text
Flat panel patterns + seam lengths + rib profiles
   |                                (Level 1: manufactured, not sculpted)
   v
Inflated canopy geometry ---- billow / ovalization equilibrium
   |
   |   Pilot input
   |      |
   |      v
   |   Harness/pilot rigid body ---- movable pilot CG
   |      |
   |      v
   |   Carabiners and risers
   |      |
   |      v
   |   Tension-only line/cascade graph
   |      |
   |      v
   +-> Canopy attachment nodes
          |
          +---- flexible membrane, ribs, mini-ribs, diagonals
          |          |
          |          v
          |      cell volumes, inlet flow, internal pressure
          |          |
          |          v
          +---- VSM aerodynamics + 2D viscous polars
          |          |
          |          v
          |      separation state / hysteresis
          |          |
          |          v
          +---- apparent mass tensor
                     |
                     v
              forces and moments
                     |
                     v
        Coupled canopy/line/payload integration (fixed 120 Hz)
                     |
                     v
            Atmosphere: wind, thermals, turbulence
```

---

## Level 0 — Baseline, contracts, and the determinism spine

> **Status: done**, except the terrain/flight frame disagreement noted in
> the exit gate below, which remains open and is the project's oldest defect.

**Budget: 12 hours** *(was 6 — the engine-integration and dual-model work was
missing)*

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
- **Decouple physics from the Unreal tick**: fixed-step accumulator, fixed
  substeps, interpolated render state, no per-frame `DeltaTime` reaching any
  solver. This is the single most expensive thing to retrofit later.
- **Stand up the dual-model harness**: legacy handling model and new solver
  behind a runtime flag, both runnable against the same test suite, so every
  later regression is attributable.
- Add deterministic state hashing for replay comparison, including
  cross-platform float determinism policy (fixed FMA/fast-math settings,
  documented).
- Create a coefficient registry containing source, value, units, confidence,
  calibration status, and valid range.

### Deliverables

- `ParagliderCoordinateSystem.h`
- `ParagliderSolverClock.{h,cpp}` — fixed-step accumulator and interpolation
- `ResearchCoefficientRegistry.{h,cpp}`
- `ParagliderModelSelector` runtime flag and A/B test runner
- still-air baseline CSV and golden test
- solver-state hash test

### Exit gate

- Left input, left attachment, left line, left canopy half, and left world
  trajectory agree in an end-to-end test.
- Ten minutes of hands-up still air remain deterministic and finite.
- The same replay produces an identical state hash at 30, 60, 144, and
  uncapped render rates.

---

## Level 1 — Manufactured EPIC 2 geometry

> **Status: done.** Unfold residual 2.4e-15; flat span, flat area and
> projected span exact by construction; billow emergent from the seam allowance.

**Budget: 24 hours** *(was 10 — the original treated geometry as digitisation;
it is actually a small CAD problem)*

This is the level where the plan's thesis is won or lost. If the canopy shape
is authored as a 3D surface, every later level is decorating a lie.

### Work

- Build an explicit **parametric flat-pattern** definition:
  - cell count, rib stations, mini-rib and diagonal-rib topology;
  - flat and projected span;
  - chord distribution;
  - **arc integrated numerically so flat span is preserved exactly**, not
    approximated by a circular-arc closed form;
  - upper/lower surface profiles;
  - leading-edge openings;
  - trailing edge;
  - cell and half-cell boundaries;
  - **billow / chord-cut-billow allowance per panel**.
- Implement **isometric triangulated unfolding** (3D panel → flat cutting
  pattern): triangulate each cell strip between adjacent rib curves, unroll one
  triangle at a time by circle–circle intersection from two already-placed
  vertices using true 3D edge lengths. Every triangle is exactly isometric, so
  no edge length is stretched anywhere; the non-developability is pushed
  entirely into the choice of diagonal for each quad.
  - Measure and report the residual as a first-class output. Reference values
    from a working implementation of this method: **0.000% at zero billow,
    0.36% at 2.6% extra length, 1.08% at 8%**; ribs, mini-ribs and h-straps at
    0.000% (planar or lying on the skin), diagonal ribs at 0.24%.
  - Record the diagonal-choice policy explicitly. This is where the error goes,
    so it is a design parameter, not an implementation detail.
- Implement the **inverse**: flat pattern → inflated shape, by relaxing the
  panel mesh under internal pressure and seam-length constraints. Ovalization
  and billow must *emerge*. Published design-practice targets to check against:
  - inflated cell width ≈ **5–6% narrower** than the flat panel width;
  - panel width compensation ≈ **6% total (3% per side)**;
  - "virtual" in-flight airfoil **18–21.5% larger** than the projected flat
    pattern;
  - contour length difference ≈ **2.26% upper surface, 0.9% lower surface**;
  - linear transition zones ≈ 10% from LE both surfaces, 15% upper / 25% lower
    at the TE.
- Represent every A, A′, B, C/upper-D, and brake attachment node.
- Import/digitize the published line plan into a versioned data file.
- Generate both physics panels and rendered mesh from the same geometry.
- Remove duplicated procedural endpoint formulas from rendering.

### Deliverables

- `CanopyGeometry.{h,cpp}`
- `PanelUnfolder.{h,cpp}` — triangulated isometric unfold + residual report
- `BillowRelaxation.{h,cpp}` — flat pattern → inflated equilibrium
- `CanopyGeometryData.h`
- `Data/Wings/bgd-epic-2-ml-geometry.json`
- geometry visualizer with attachment labels, seam lines, and strain overlay
- geometry consistency tests

### Exit gate

- Every rendered line terminates on an actual rendered attachment vertex.
- Mirrored attachment pairs are geometrically symmetric.
- Published area, span, chord, line length, and riser dimensions remain within
  declared digitization tolerances.
- **Round trip**: 3D surface → flat pattern → re-inflated surface returns to
  within the declared unfold residual. No hidden scaling anywhere in the loop.
- **Billow is emergent**: changing a seam allowance changes the inflated cross
  section without any 3D surface being edited by hand.

> **Note on the unfolding method.** Triangulated isometric unrolling is the
> same family as Rhino's `Squish`/`Smash`. It does not eliminate
> non-developability, it relocates it: instead of stretching fabric, the error
> appears as a small discontinuity across the chosen diagonals, and in the real
> sail as micro-puckering along the panel/rib join — which is precisely what
> chord-cut-billow construction is designed to absorb. Bounded and measured is
> the correct goal here; compensated is not achievable and not necessary.

---

## Level 2 — Suspension and cascade graph

> **Status: done.** 90 nodes, 78 cables, 254.8 m manufactured against a
> published 254. Row shares emergent: A 36%, A' 11%, B 37%, C 16%.

**Budget: 14 hours** *(was 10)*

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
- Solve line stretch and slack state iteratively; use a complementarity or
  compliant-constraint formulation rather than a stiffness clamp, so slack is
  exact rather than "nearly zero".
- Apply equal-and-opposite line forces at both endpoints.
- Make accelerator and brake inputs change riser/line rest lengths.
- Model **line stretch under load** with real Dyneema-class properties, and
  record the trim-shift this causes; it is not negligible on a modern wing.
- Derive visual sag from solved nodes, not an invented curve.

### Deliverables

- `SuspensionGraph.{h,cpp}`
- `TensionCableSolver.{h,cpp}`
- line-plan data importer
- line tension/slack telemetry per element

### Exit gate

- Slack elements transmit zero compressive force — exactly zero, verified, not
  "small".
- Total endpoint reaction is equal and opposite within tolerance.
- Weight shift changes carabiner loads and attachment geometry without any
  direct roll moment.
- Brake input loads only through the brake cascade and trailing edge.
- Full-accelerator application changes canopy incidence through the riser
  geometry alone.

---

## Level 3 — Pilot, harness, and payload rigid body

> **Status: done**, except full six-degree-of-freedom payload integration,
> which is Level 7's two-body coupling rather than this level's.

**Budget: 10 hours** *(was 8)*

### Work

- Split payload and canopy into independent rigid/deformable systems.
- Define pilot, harness, reserve, ballast, and equipment masses.
- Define payload CG and inertia tensor.
- Move pilot CG laterally and longitudinally from weight shift and body pose.
- Model **harness geometry class** (seat plate vs. no plate, chest strap width)
  since it changes weight-shift authority and roll damping substantially.
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
- Narrowing the chest strap measurably increases weight-shift authority, with
  no code path other than harness geometry involved.

---

## Level 4 — Aerodynamics: VSM with viscous polars

> **Status: done, with two gaps.** Validated against lifting-line theory
> (CL_alpha 0.2%, CDi 3.6%) and published glide (9.46 vs 9.5). Section polars
> are analytic, not measured - the data acquisition this level depends on has
> not begun. Deep stall does not converge and is locked as a known failure.

**Budget: 28 hours** *(was 12 — this is a solver port plus a data-acquisition
problem, not one workstream)*

### Work

- Implement the Vortex Step Method over the spanwise geometry, with
  vortex-filament rings/horseshoes per section and a nonlinear circulation
  solve coupled to 2D section polars.
- **Acquire section polars.** This is a data problem that must start during
  Level 1:
  - run XFOIL / a viscous-inviscid 2D solver over the digitized profiles across
    the operating Reynolds envelope (roughly 0.5–3 × 10⁶);
  - extend to post-stall with a Viterna-type or published parafoil extension;
  - generate a family across brake deflection (trailing-edge deflection changes
    camber, not just incidence);
  - generate a reduced-pressure family for Level 5 coupling;
  - store as a versioned, interpolatable polar table with declared validity.
- Compute local relative airflow including canopy translation, canopy rotation,
  panel deformation velocity, local wind, and VSM-induced flow.
- Calculate local lift, drag, and pitching moment per section.
- Apply forces at panel centers rather than as one global resultant.
- Add line, riser, harness, and pilot drag at their physical locations — on a
  paraglider this is a **large** fraction of total drag, not a correction term.
- Add attached-flow and separated-flow hysteresis.
- Add the Lissaman & Brown apparent mass/inertia tensor.
- Validate the C++ VSM against the open-source Python/Julia reference on
  identical geometry before trusting a single flight number.

### Deliverables

- `VortexStepMethodSolver.{h,cpp}`
- `SectionPolarTable.{h,cpp}` + polar generation tooling
- `ApparentMassTensor.{h,cpp}`
- force/moment debug visualization
- local incidence/separation telemetry
- VSM cross-validation report against the reference implementation

### Exit gate

- C++ VSM matches the reference implementation on a rectangular wing and on the
  EPIC 2 planform within declared tolerance.
- Hands-up trim converges without a speed controller.
- Integrated EPIC 2 targets are held in still air: trim speed near published
  target, glide ratio near declared target, stable sink rate, bounded pitch
  mode.
- Mirrored inputs produce mirrored panel forces and trajectories.
- **Provisional-gate declaration:** these numbers are recorded as *provisional*
  and re-run after Level 6, because the membrane solver changes the effective
  airfoil. Do not treat Level 4 trim as final.

---

## Level 5 — Cell openings and pressure model

> **Status: done.** Stagnation point 5.2/9.7/14.3 degrees on bar/trim/brake;
> inlets recover Cp 0.97 at trim and 0.89 on bar; pressure feeds back into
> section performance.

**Budget: 16 hours** *(was 10)*

### Work

- Represent each cell or cell group with volume, inlet/opening area, external
  stagnation pressure, internal pressure, leakage/porosity, and inter-cell flow
  through rib crossports.
- Compute mass flow through leading-edge openings, with the inlet's position
  relative to the **moving stagnation point** as the governing variable.
- Couple cell volume changes to pressure.
- Reduce membrane stiffness and section performance as pressure falls — this is
  the polar family generated at Level 4.
- Model inlet closure or reverse flow at adverse incidence.
- Make **internal pressure coefficient** a first-class, always-available
  telemetry channel, per cell, with a declared collapse-risk threshold.

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
- The stagnation point migrates with incidence and accelerator, and inlet
  effectiveness follows it.

---

## Level 6 — Flexible membrane and ribs

> **Status: core done at reduced scope.** Sagitta 26.32 mm against an
> analytic 25.99, strain 0.060% against 0.064%, bias 8x softer than warp.
> Delivered as spanwise strips at chord stations, not a full two-dimensional
> mesh; ribs are fixed endpoints; no self-collision, which Level 8 needs.

**Budget: 40 hours** *(was 14 — this was by a wide margin the most optimistic
line in the original budget)*

### Work

- Build a reduced membrane mesh shared with rendering, derived from the Level 1
  flat patterns so seam lengths are the constraint rest lengths.
- Implement in XPBD first (fast to stand up), behind a swappable constraint
  API, with the VBD/AVBD migration scoped at Level 12.
- Include warp/weft stretch, shear, bending, rib constraints, mini-rib and
  diagonal-rib constraints, trailing-edge tension, pressure forces, and
  attachment-node forces.
- Model fabric anisotropy properly: ripstop nylon is far stiffer along warp and
  weft than on the bias, and bias behavior is what governs wrinkling and
  collapse folding.
- Add substepping and constraint iterations independent of render frame rate,
  with a **fixed** iteration count for determinism.
- Add numerical energy monitoring and deformation limits.
- Support asymmetric deformation without mesh inversion.

### Deliverables

- `CanopyMembraneSolver.{h,cpp}` with swappable constraint backend
- membrane material data (warp/weft/bias, measured or sourced)
- shared simulation/render vertex buffer
- deformation and constraint-residual telemetry

### Exit gate

- Pressurized cells hold their shape without explosive energy growth.
- Attachment loads deform the canopy smoothly.
- Released deformation oscillates and damps at a controlled rate.
- Rendering cannot detach from physics geometry.
- **The Level 4 aero gates are re-run and still pass** with the deformable
  canopy. Any drift is explained, not tuned away.

---

## Level 7 — Coupled solver and convergence

> **Status: done. Every exit gate passes and the suite runs with the rest.**
> Hands off it settles at 10.70 m/s - 38.5 km/h against a published 39 - with
> 1.12 m/s of sink and a glide of 9.5 against a published 9.5. Ten minutes holds
> within 0.15 m/s, internal closure is exact, energy residual stays under 4 W,
> and the coupling is converged rather than budgeted. Asymmetric brake turns
> without any control-to-yaw or control-to-bank term - 0.094 rad of bank and
> 0.030 rad/s after ten seconds - and its mirror image agrees to 2e-8 rad. Heavy
> brake walks the wing into a fully separated 46-degree stall at 4.65 m/s of
> sink, monotone in incidence, separation, sink and cell pressure, without the
> numerical safety envelope engaging at all.

**Budget: 22 hours** *(was 10)*

### Work

- Define the integration order for:
  1. pilot controls and rest-length changes;
  2. atmospheric sampling;
  3. aerodynamics (VSM, possibly at a reduced rate with interpolation);
  4. pressure;
  5. membrane constraints;
  6. lines;
  7. canopy/payload rigid motion.
- Apply equal-and-opposite reactions consistently.
- Implement a **staggered coupling scheme** between the aerodynamic and
  structural solvers, following the approach used in the ram-air kite FSI
  literature (potential-flow aero ↔ structural solve, iterated to a relaxed
  equilibrium). Add relaxation/damping on the coupling to prevent the classic
  added-mass instability of weakly coupled FSI.
- Add solver iterations where pressure, membrane, and line loads must converge,
  with a fixed iteration budget.
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
- Coupling iteration count can be reduced by one without a qualitative change
  in behavior — i.e. the solve is genuinely converged, not iteration-tuned.

---

## Level 8 — Emergent stall, collapse, and reopening

**Budget: 24 hours** *(was 10)*

### Work

- Remove remaining global brake/stall thresholds where local states exist.
- Derive collapse from local incidence, panel unloading, pressure loss, line
  unloading, and membrane buckling.
- Allow collapse propagation through shared membrane and pressure states.
- Derive cravat risk from tip geometry and line/fabric contact approximation —
  this requires actual self-collision on the canopy mesh, which is the hidden
  cost in this level.
- Derive reopening from pressure, airflow, membrane tension, and brake-line
  loading.
- Keep a numerical safety envelope to prevent solver explosion, clearly
  separate from flight behavior and logged whenever it engages.

### Deliverables

- local collapse-state telemetry
- canopy self-collision (tip/line/fabric)
- collapse/reopening diagnostic views
- symmetric and asymmetric incident benchmarks

### Exit gate

- Deep symmetric brake progresses from flare to separation/stall.
- Asymmetric separation produces spin/spiral behavior, not barrel rolls.
- A collapse produces genuinely slack affected lines.
- Brake pumping only affects a collapse when brake-line tension reaches the
  relevant trailing-edge region.
- The numerical safety envelope does not engage during any nominal maneuver.

---

## Level 9 — Calibration and pilot-facing validation

**Budget: 20 hours** *(was 10)*

### Work

- Create repeatable still-air system-identification maneuvers: hands-up trim,
  brake steps, brake pulse and release, weight-shift step, coordinated turn,
  spiral entry and exit, symmetric stall approach, asymmetric stall approach.
- Export time series for airspeed, sink, pitch, roll, yaw, brake force, line
  tension, pressure, and energy.
- Fit only bounded, identified parameters. Every fit must be recorded in the
  coefficient registry with its residual.
- Compare against published data where available.
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

---

## Level 10 — Performance, visualization, and retirement of legacy paths

**Budget: 20 hours** *(was 10)*

### Work

- Profile CPU time and memory at 120 Hz.
- Introduce solver levels of detail without changing large-scale behavior.
- Move suitable panel/membrane work to parallel tasks or compute shaders.
- Add research visualization toggles: cell pressure, airflow/opening flow,
  panel force vectors, separation, line tension/slack, membrane strain, solver
  residuals, unfold residual.
- Delete superseded direct moments, scripted visual deformation, and duplicate
  geometry. Remove the legacy model and the dual-model flag.
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

> **End of Camp II.** Everything above this line is an achievable, coherent
> simulator. Everything below is research.

---

## Level 11 — Unsteady wake

**Budget: 36 hours**

### Work

- Promote the steady VSM to an **unsteady vortex-lattice / vortex-particle**
  formulation with a free-deforming wake, following the nonlinear unsteady
  VLM–VPM approach used in rotorcraft aerodynamics.
- Shed wake vorticity from the trailing edge and convect it; allow the canopy
  to fly through its own wake.
- Add adaptive wake conversion (near-field lattice → far-field particles) so
  cost stays bounded.
- Add unsteady circulation lag (Wagner/Küssner-type) so pitch and brake
  transients have correct phase, rather than instantaneous polar lookup.

### Exit gate

- A spiral dive shows genuine wake re-encounter effects on the inner wing.
- Wingover and rapid pitch inputs show circulation lag, not instant response.
- A brake release produces the correct overshoot timing without a tuned delay.
- Wake cost stays inside the frame budget for 60 s of aggressive maneuvering.

---

## Level 12 — Solver replacement and GPU residency

**Budget: 40 hours**

### Work

- Migrate the membrane from XPBD to **VBD/AVBD**, using the Augmented
  Lagrangian formulation for hard seam and attachment constraints.
- Move the membrane, pressure, and wake solves fully onto GPU compute, with the
  canopy state never round-tripping to CPU during a step.
- Preserve bit-level determinism across the CPU→GPU move — this is genuinely
  hard and may require fixed-point accumulation or a deterministic reduction
  order.
- Raise the membrane mesh resolution to the point where fold and wrinkle
  structure is resolved rather than implied.

### Exit gate

- Identical state hashes between CPU reference and GPU path.
- Mesh resolution can be doubled without changing large-scale behavior.
- Collapse folds have visible, physically plausible wrinkle structure.

---

## Level 13 — Resolved atmosphere

**Budget: 60 hours**

### Work

- Replace scalar wind with a **spatially and temporally coherent turbulent
  field**: precomputed or procedurally synthesised turbulence with correct
  energy spectrum and length scales, sampled per-panel rather than per-wing.
- Model thermals as structures — core, edge shear, rotor, and the entry/exit
  gradients that actually cause collapses — not as vertical velocity offsets.
- Model terrain-induced flow: ridge lift, leeside rotor, venturi compression.
- Ensure the per-panel sampling is what drives asymmetric loading, so
  turbulence-induced collapse is emergent.

### Exit gate

- Flying into a strong thermal edge produces asymmetric loading and, at the
  limit, a collapse — with no collapse-triggering code involved.
- The same turbulence seed reproduces the same flight exactly.
- Wing behavior in rotor is qualitatively recognised by pilots who have flown
  the real thing.

---

## Level 14 — Real-time two-way FSI with boundary-layer separation

**Budget: 80 hours** *(research; may not be achievable at 120 Hz)*

### Work

- Replace polar-table lookup with an **integral boundary-layer solver coupled
  to the potential-flow solution**, following the paraglider/parachute FSI
  methodology in the literature — panel method plus integral BL plus an
  explicit separation model — so separation location is computed rather than
  interpolated.
- Two-way couple it to the membrane at every step, not staggered at a reduced
  rate.
- Handle the strongly-coupled regime near stall where the aero and structural
  solutions are mutually unstable if weakly coupled.

### Honest assessment

Published paraglider FSI of this class runs offline, not at 120 Hz. Reaching
this level in real time on consumer hardware would be a genuine contribution,
not an implementation task. **Expect not to reach it.** Attempt it only after
Level 11 is stable, and treat a reduced-rate or reduced-region version (BL
solver only near the tips and trailing edge) as the realistic outcome.

### Exit gate

- Separation location is computed, matches 2D reference solutions on a fixed
  section, and moves correctly with brake and pressure.
- The coupled solve is stable through a full stall and recovery.

---

## Level 15 — Instrumented flight identification and certification-grade validation

**Budget: 120 hours+, and mostly not software** *(the summit)*

### Work

- Instrument a real EPIC 2 ML: IMU at canopy and harness, pitot, barometric
  vario, GPS, **line tension load cells on at least the A and brake risers**,
  and differential pressure sensors in representative cells (the wind-tunnel
  literature shows this instrumentation is feasible and has been proposed as a
  collapse-alert system).
- Fly a structured identification programme with a qualified pilot in
  controlled conditions.
- Perform full system identification against the recorded data; correct the
  model rather than tuning it to match.
- Reproduce the **EN 926-2 maneuver set** — front collapse, large asymmetric
  collapse at trim and accelerated, deep/parachutal stall, developed full stall
  and recovery, gentle spiral at 3–5 m/s sink held for one turn, spiral dive
  descent — and compare the simulated outcome to the wing's published
  certification classification.

### Exit gate (the summit condition)

- The simulator, given only the EPIC 2 geometry and materials, **independently
  reproduces the wing's published EN 926-2 classification** across the maneuver
  set, without any per-maneuver tuning.

### Honest assessment

This requires airworthiness care, a test pilot, instrumentation engineering,
and a research budget. It is written down because it defines what "correct"
would actually mean. Nobody should expect to get here. If the model ever
reproduced a certification classification from first principles, that would be
a publishable result in its own right.

---

## Hour budget

| Camp | Level | Workstream | Hours | Confidence |
|---|---:|---|---:|---|
| Base | 0 | Baseline, contracts, determinism spine | 12 | High |
| Base | 1 | Manufactured canopy geometry | 24 | Medium |
| Base | 2 | Suspension/cascade graph | 14 | High |
| I | 3 | Pilot/harness rigid body | 10 | High |
| I | 4 | VSM aerodynamics + polars | 28 | Medium |
| I | 5 | Cell openings and pressure | 16 | Medium |
| II | 6 | Flexible membrane and ribs | 40 | Low |
| II | 7 | Coupled integration | 18 | Low |
| II | 8 | Stall, collapse, reopening | 24 | Low |
| III | 9 | Calibration and pilot validation | 20 | Medium |
| III | 10 | Performance and legacy removal | 20 | Medium |
| III | 11 | Unsteady wake | 36 | Low |
| Death zone | 12 | VBD migration and GPU residency | 40 | Very low |
| Death zone | 13 | Resolved atmosphere | 60 | Very low |
| Death zone | 14 | Real-time two-way FSI | 80 | Speculative |
| Death zone | 15 | Instrumented flight identification | 120+ | Speculative |
| | | **Camp II complete (Levels 0–10)** | **226** | |
| | | **Camp III complete (Levels 0–11)** | **262** | |
| | | **Full ladder** | **542+** | |

The Camp II figure is the number that matters. 226 hours produces a complete,
coherent, geometry-driven simulator with emergent collapse. The remaining
280+ hours buy fidelity that most of the field does not currently have in real
time.

Levels 6, 8, and 14 carry the greatest uncertainty. Level 6 alone decides
whether Camp II costs 226 hours or 350.

## Recommended implementation order

Implement levels sequentially. Do not begin flexible collapse work before the
geometry, line graph, local aerodynamics, and pressure state exist.

Two workstreams may run in parallel from the start, and should:

- **Polar acquisition** (Level 4 data) begins during Level 1, since it depends
  only on the digitized profiles.
- **Material characterisation** (Level 6 fabric data) begins during Level 2.

The first practical milestone is Levels 0–2: one authoritative EPIC 2 geometry
built from flat patterns, physically connected attachment points, and
tension-only suspension lines. That milestone removes the current
visual/physical duplication and creates the foundation every later solver
needs.

## Known risks and mitigations

- **Level 4 gates are provisional.** Trim speed and glide ratio are first
  validated on near-rigid geometry; Levels 5–6 change the effective airfoil
  shape, so the still-air baseline must be re-recorded and the Level 4 gates
  re-run after the membrane solver lands. This is budgeted into Level 6's exit
  gate, not left as a surprise.
- **Engine integration is a Level 0 decision.** The fixed 120 Hz physics rate
  must be decoupled from the Unreal tick from the first commit. Retrofitting
  determinism onto a variable tick costs more than building it.
- **Parallel operation, not big-bang cutover.** The legacy handling model stays
  behind a runtime flag until Level 10, and every exit gate runs against both.
- **The membrane solver is the schedule risk.** If Level 6 slips, cut mesh
  resolution and strengthen rib constraints. Do not borrow hours from Level 9;
  an uncalibrated high-fidelity model is worth less than a calibrated coarse
  one.
- **Section polars are a data problem, not a code problem.** Start them during
  Level 1.
- **Weakly-coupled FSI is unstable near stall.** This is a known failure mode,
  not a bug you will discover. Plan the relaxed/staggered coupling at Level 7
  rather than discovering divergence at Level 8.
- **Determinism and GPU are in tension.** Level 12 may have to choose. If it
  does, determinism wins — it is guiding rule 10, and without it none of the
  regression suites mean anything.
- **The unfold residual is a design parameter.** Level 1's diagonal-choice
  policy determines where non-developability goes. Fix it early and record it;
  changing it late invalidates every geometry regression.

## Validation boundary

Completing Levels 0–11 would produce a substantially more physical and
inspectable real-time model than anything this project has had. It would still
not establish that the simulator is safe for independent pilot training. That
requires instrumented flight data, controlled calibration, expert review,
documented limitations, and an appropriate training and regulatory validation
process — which is what Level 15 describes and why Level 15 is honest about
being out of reach.

No level of this plan authorises presenting the simulator as training-valid.
That claim requires the Level 15 evidence, and nothing less.

## Sources

- [A Real-Time 6DOF Computational Model to Simulate Ram-Air Parachute Dynamics](https://www.mecs-press.org/ijitcs/ijitcs-v9-n3/v9n3-3.html)
- [On the Development of a Six-Degree-of-Freedom Model of a Low-Aspect-Ratio Parafoil Delivery System (AIAA 2003-2105)](https://nps.edu/documents/106608270/107784480/Pegasus+-+Mortaloni+-+On+the+Development+of+a+Six-Degree-of-Freedom+Model+of+a+Low-Aspect-Ratio+Parafoil+Delivery+System.pdf/dba73a0e-806a-4ef5-a6b0-cb4555db5899)
- [Apparent Mass Effects on Parafoil Dynamics — Lissaman & Brown, AIAA 1993-1236](https://arc.aiaa.org/doi/abs/10.2514/6.1993-1236)
- [A Fluid Structure Interaction Methodology to design Paragliders and Parachutes (AIAA 2022-2752)](https://arc.aiaa.org/doi/10.2514/6.2022-2752)
- [Numerical Methods for Efficient Fluid–Structure Interaction Simulations of Paragliders](https://link.springer.com/article/10.1007/s42496-019-00017-2)
- [An Aero-Structural Model for Ram-Air Kite Simulations — Thedens & Schmehl](https://www.mdpi.com/1996-1073/16/6/2603)
- [Fluid-structure interaction simulation for performance prediction and design optimization of parafoils](https://www.tandfonline.com/doi/full/10.1080/19942060.2023.2194359)
- [Vortex Step Method — Python implementation](https://github.com/awegroup/Vortex-Step-Method)
- [VortexStepMethod.jl — Julia implementation](https://github.com/OpenSourceAWE/VortexStepMethod.jl)
- [Computational aerodynamics for soft-wing kite design (WES preprint)](https://wes.copernicus.org/preprints/wes-2026-46/)
- [A new non-linear vortex lattice method: Applications to wing aerodynamic optimizations](https://www.sciencedirect.com/science/article/pii/S1000936116300954)
- [Nonlinear Unsteady Vortex-Lattice Vortex-Particle Method with Adaptive Wake Conversion](https://arxiv.org/pdf/2511.11430)
- [Vertex Block Descent — Chen, Liu, Yang & Yuksel, ACM TOG 43(4)](https://www.researchgate.net/publication/382411275_Vertex_Block_Descent)
- [Going Further With Vertex Block Descent](https://onlinelibrary.wiley.com/doi/10.1002/cav.70039)
- [Performance Comparison of Vertex Block Descent and Position Based Dynamics Algorithms Using Cloth Simulation in Unity](https://www.mdpi.com/2076-3417/14/23/11072)
- [XPBD: Position-Based Simulation of Compliant Constrained Dynamics](https://dl.acm.org/doi/pdf/10.1145/2994258.2994272)
- [MGPBD: A Multigrid Accelerated Global XPBD Solver](https://arxiv.org/pdf/2505.13390)
- [Wind-Tunnel Measurement of Differential Pressure on the Surface of a Dynamically Inflatable Wing Cell](https://www.mdpi.com/2226-4310/8/2/34)
- [Wind Tunnel Test of a Paraglider (flexible) Wing Canopy](https://www.researchgate.net/publication/266493626_Wind_Tunnel_Test_of_a_Paraglider_flexible_Wing_Canopy)
- [Paraglider design handbook — Ovalization](https://www.laboratoridenvol.com/paragliderdesign/ovalization.html)
- [Bruce Goldsmith explains how Chord Cut Billow works](https://www.flybgd.com/en/paragliders/bruce-explains-how-cord-cut-billow-works--news-6161-688-0.html)
- [Draft EN 926-2](http://www.xcmag.com/wp-content/uploads/2012/08/DraftEN926-2.pdf)
- [EN 926-2 summary (Dudek)](https://dudek.eu/wp-content/uploads/2021/05/En926-2-EN-PG-test.pdf)
