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
| **Base camp** | 0–2 | One authoritative geometry, real suspension graph, no duplicated meshes | **Reached** |
| **Camp I** | 3–5 | Emergent trim and turns from local aero and cell pressure | **Reached for trim, not for turns** — see Level 7 |
| **Camp II** | 6–8 | Emergent collapse, stall, and reopening from membrane mechanics | **Reached, at reduced scope** — Level 6 is 1-D strips, no cravat in flight |
| **Camp III** | 9–11 | Calibrated, unsteady-wake, validated against certification maneuvers | **Camped on the approach.** 9 done bar pilot review and under-settled; 10 blocked on the envelope; 11 specified but unstarted |
| **Death zone** | 12–15 | Resolved turbulence, real-time two-way FSI, instrumented-flight identification | Beyond current published real-time state of the art |

Levels 12–15 are not padding. They are written out in full because knowing the
shape of the unreachable work changes how the reachable work is designed. Do
not treat failure to reach them as project failure. Treat reaching Camp II as
an outstanding result.

**That bar has been met: Camp II is reached.** What Camp III has taught so far
is that the remaining distance is not measured in levels. The three items
standing between here and a validated simulator — the envelope, the drag, and
the pilot review — are each *older* than the level that was supposed to contain
them, and none of them is the level's own work. That is the single most useful
thing to know before planning the next one.

## Build status

Updated at the end of the collapse-symmetry investigation (§64–§68), which is
where the physics work currently stands. The engine as built is documented in
`docs/PHYSICS_ENGINE.md`; what it cost to build is in
`docs/PHYSICS_LEARNINGS.md`; the live item list is `docs/PHYSICS_TODO.md`.

> **Naming, because it has already cost one reader a wrong conclusion.** Two
> different things have been called "Level 11". **Level 11 in this plan is the
> unsteady wake, and it is unstarted.** The *pitch-axis programme* — the
> eigenmode work in `PHYSICS_TODO` — was also called Level 11 while it ran, and
> it is finished. Where this document says Level 11 it always means the wake.

| Level | Status | Evidence |
|---|---|---|
| 0 Baseline and determinism | **Done** | state hash identical at 30/60/144/uncapped; registry live |
| 1 Manufactured geometry | **Done** | unfold residual 2.4e-15; flat span, area, projected span exact |
| 2 Suspension graph | **Done** | 254.8 m against published 254; mirror-symmetric to 6.2e-15 m (§65) |
| 3 Payload and harness | **Done** | carabiner split exact to W(1/2 ± e/s); pendulum period 2π√(L/g) |
| 4 VSM and polars | **Done** | CL_α 0.2%, CDi 3.6%; polars now solved on the section, not stated (item 1) |
| 5 Cell pressure | **Done** | stagnation 5.2/9.7/14.3 deg; inlet Cp 0.97 trim; mirror-exact to 0.000e+00 (§67) |
| 6 Membrane | **Core done, 1-D** | sagitta 26.32 mm vs analytic 25.99; strips, not a mesh — and **no torsion**, which is what the geometric channel now waits on (§72) |
| 7 Coupled solver | **REOPENED, blocked on canopy torsion** | books balance and mirrors to 2e-8, but its gate says weight-shift turns must *emerge* and they do not — 0.014 rad/s (§71). The channel's aerodynamic half is built and linear at 8543 N·m/rad; its structural half is identically zero because the canopy is rigid (§72). See "the geometric channel" |
| 8 Emergent collapse | **Done, with gaps** | fold from a pressure balance; no cravat in flight (item 4) |
| 9 Calibration | **Done bar pilot review** | trim 39.4 vs 39.0 km/h — but see item 18, every number under-settled |
| 10 Performance and legacy removal | **Strands 1–2 done, exit gate BLOCKED** | `SOLVER_PROFILE.md`, `SOLVER_LOD.md`; 15× real time full, 36× reduced; visualisation strand unstarted; legacy path still flies the game |
| 11 Unsteady wake | **Not started — and now specified by measurement** | §68: the separated solve is not short of iterations, it has nothing to converge to |
| 12+ | Not started | — |

**Camp II is built but not closed**, and the distinction is the whole of the
current position: Levels 6–8 pass their quantitative gates, Level 10's exit gate
does not, and **nothing geometry-driven flies the game** (item 7). Levels 1–8
have been exercised by their own suites and by nobody who flies.

**And Level 7 is reopened, which changes the shape of the ladder.** Its exit
gate reads *"weight shift and brake turns emerge without direct turn moments"*.
That has two halves. The absence of a direct turn moment was verified. The
emergence was not, and §71 measured it: full weight shift turns the wing at
**0.014 rad/s**. The cause is one interface — the suspension solve computes
every node's position and publishes a single root-chord scalar, so nothing
spanwise reaches the aerodynamics. **The geometry-driven stack is
geometry-driven everywhere except the interface where it matters most.** The
design is below, under "the geometric channel", and it is now the head of the
queue.

**Since built, and half of it landed** (§72). The aerodynamic channel exists
and is linear at 8543 N·m/rad; the per-station pose it was to be driven from is
identically zero, because the canopy is a rigid body in the line solve. What
the interface was missing turned out to be upstream of the interface: canopy
torsional compliance, which the stack does not have anywhere.

### The one structural finding this plan did not anticipate

The original ladder puts Level 10 before Level 11 and treats the wake as
Camp III polish. **The measured dependency runs the other way**, and four
sessions of work established it:

1. Level 10's exit gate is "no legacy direct-control force remains active"
   (item 17). It is blocked because the geometry-driven stack departs at 40%
   brake and is statically pitch-divergent at its published top speed — the
   usable envelope is hands-up to about a quarter brake (item 11).
2. The drag deficit (item 12, glide **10.96 against a published 9.5**) is the
   largest disagreement left, and closing it is expected to make the pitch axis
   *worse*, not better: the same drag applied at the canopy **costs** stability
   and only the moment arm reverses it (§55, §56). So items 11 and 12 have to
   be closed **together**. Closing 11 against a wing carrying a sixth too
   little drag would fix it against the wrong aircraft.
3. One candidate for item 12 — the line-drag shielding correction, which is
   measured rather than fitted and takes glide error from 19% to 4.3% — is
   blocked by a symmetry gate (§64).
4. That gate is blocked by the separated aerodynamic solve having no
   single-valued solution, **which is what Level 11 is** (§68). It is not a
   defect, not an instability, and not short of iterations: 40, 200 and 600
   iteration caps break on the same tick.

So on that route the wake sits upstream of Level 10's exit gate, upstream of
the largest calibration disagreement, and upstream of the envelope that decides
whether the stack can fly the game at all.

**That alternative has now been tested, and it failed (§69).** The hope was
that the deficit could be placed at the **harness** instead — §56 measured that
it lands glide, trim speed and sink together where a section-side offset lands
glide alone — and that harness drag, acting six metres under the canopy, would
not drive the canopy deeper into collapse. It does, through the pendulum: drag
at the pilot pitches the wing nose-down, which is the frontal direction.

Three wings at the same published glide, one symmetric frontal:

| | peak fold L / R | worst L−R | turn | safety envelope |
|---|---|---|---|---|
| shipped (glide 10.96) | 0.710 / 0.710 | 0.303 | 1.98 rad/s | idle |
| section +0.01035 | **0.958 / 0.703** | 0.667 | 1.93 rad/s | **ENGAGED** |
| harness +0.199 m² | 1.000 / 1.000 | 0.433 | **6.53 rad/s** | **ENGAGED** |

The section correction splits the halves like the line-drag one. The harness
correction only *looks* symmetric — two halves saturated at a full collapse is
saturation, not symmetry, and its path asymmetry is worse than shipped. **Both
engage the numerical safety envelope**, so by guiding rule 12 neither row is
flight behaviour, and the benchmark cannot adjudicate between them at all.

**So item 12 has no route around Level 11, measured for all three candidate
locations rather than argued for one.** The wake is on the critical path. What
the test also sharpened is *why*: the corrections fail differently but at the
same place — the partly separated regime §68 showed has no single-valued
solution. A wing that glides 9.5 instead of 10.96 arrives there harder whatever
slowed it down, so the blocker was never a property of line drag; line drag was
just the first correction to reach it.

### Carried gaps, by priority

1. **Nothing geometry-driven flies the wing (items 7, 17).** `ParagliderDynamics`
   — one 6-DOF body with a fitted polar — is still what the game flies. This is
   guiding rule 11 working as intended, but it means the entire ladder is
   validated by its own test suites and by no pilot. It is the largest
   validation gap in the project and it has been open since Level 1.
2. **The envelope is hands-up to a quarter brake (item 11).** 40% brake is
   unholdable and the pitch loop gain passes one at CL 0.35 where full bar is a
   CL 0.31 condition. This is what blocks gap 1. The eigenmode programme ran to
   the end of what linearisation can say; the live lead is the **moment arm**
   (§56), the first mechanism in three levels to move the boundary the right
   way.
3. **The largest calibration disagreement is open and its cause is contested
   (item 12).** Glide 10.96 against 9.5. Two candidates: a section-side deficit
   (the historical assumption, lands glide only) and an installed/harness-side
   deficit (lands all three). Likely a combination. Must be closed with gap 2,
   not before it.
4. **Every calibration number was measured after too short a settle (item 18).**
   The harness allows 90 s where hands-up needs 410 s and 25% brake needs 1080.
   This is not a small correction: glide read 11.33 unsettled and 10.96
   settled. **Every number Level 9 published, and every conclusion drawn from
   one, is provisional until re-measured.**
5. **The wing turns several times too slowly (item 0b).** 0.045 rad/s at 1.5°
   of bank on 35% brake, where an EN-B wing does about 0.3 rad/s at 20–30°.
   §54 found the disturbance-limit boundary is **nose-down**, the same low-CL
   loop-gain path as full bar and 40% brake, so this may share one cause with
   gap 2 rather than being its own problem.
6. **Level 6 is one-dimensional.** Strips at chord stations. It is why no
   cravat has formed in the coupled solve (item 4): the strip's fold depth
   stays short of the 0.178 m gap to the nearest line. A 2-D mesh is what would
   say whether that is the wing or the model.
7. **Coefficient registry: 96 coefficients, 26 tuned, 82 unvalidated** (item 8).
   The tuned ones are concentrated in the legacy model, so gap 1 is what
   retires them — with one loud exception inside the geometry-driven stack,
   `swingDampingRatio`, which is gap 2.
8. **Apparent-mass rotational terms are disputed** (item 2). Registered
   `Disputed`; nothing uses their magnitude. Blocked on source access.
9. **Grindelwald First's anchor is 50 m above its surveyed ground** (item 9).
   Recorded rather than fitted away — the terrain is the measurement.

### Recommended next steps, in order

The ordering principle has changed with the evidence. It used to be "close the
gaps below a level before starting it". Four sessions of chasing a symmetry
residual to the bottom of the stack suggest a better one: **re-establish the
validity of the numbers, then get the stack in front of a pilot, and only then
spend a research level.** A research level bought before either of those is
paid for with unvalidated numbers and no feedback.

1. ~~**Re-measure everything at a proper settle (item 18).**~~ **Done, and it
   holds.** `parapenting_calibration_settled` reproduces it: hands-up trim
   11.174 m/s, sink 1.015, glide **10.96**, incidence 4.95°, settling at 530 s
   against the fast suite's 90. What the same run also says is that **half the
   Level 9 manoeuvre set never reaches a comparable number** — the accelerator
   step departs, 25% brake does not settle even after its input, and deep brake
   and stall approach both engage the safety envelope. Those four rows are not
   disagreements with the manufacturer; they are not measurements.
2. ~~**Run the symmetry gate against a harness-side drag correction.**~~
   **Done, and it failed (§69).** All three candidate locations for the missing
   drag — lines, section, harness — break the symmetric frontal, and the two
   new ones engage the safety envelope, so the benchmark cannot adjudicate
   between them. **Item 12 has no route around Level 11.** The wake is on the
   critical path, now measured rather than assumed.
3. **Close items 11 and 12 together** — still the right shape, but the drag
   half is now blocked behind the wake rather than merely contested. What can
   proceed without it is the pitch half: the moment-arm lead (§56) is the only
   mechanism in three levels to move the boundary the right way, and it does
   not need the drag closed to be *investigated*, only to be finished.
4. ~~**Get it flying, in its measured envelope.**~~ **Measured, and it cannot
   proceed as written (§70).** The envelope was never a number, so
   `parapenting_model_agreement` made it one — both models, same wing, same
   105 kg, same air, side by side, which is guiding rule 11 taken literally.
   Bisected, the geometry-driven stack flies to **37% of brake** (confirming
   the documented 40%), **22% of speed bar** — half bar departs — and weight
   shift produces **0.01 rad/s**, which is nothing.

   **That is two of the four controls a paraglider has.** A stated envelope can
   route around a deep-brake departure; it cannot describe an aircraft with no
   bar and no weight shift. The stack can fly hands-up gliding to a third of
   brake: enough to compare against the legacy model, not enough to give a
   player. The seam itself turned out not to be the obstacle — the pawn makes
   17 calls into the legacy model and only one is the `Step`.

5. **Build the geometric channel — Level 7's reopened half, and now the head
   of the queue.** §71 diagnosed weight shift: the chain works as far as the
   lines, transferring a real 34% of load between the carabiners, and then
   stops, because `VsmSolveInput` has no channel from the suspension solve.
   The fix is not a coefficient — no value of `hipTravelM` closes a 0.6°
   geometric bank — it is to let the canopy's per-section pose be **read off
   the solved line geometry** instead of assumed. The full design is under
   "the geometric channel" above.

   It is the head of the queue for four reasons, in order of how much they
   matter:
   - it is the only item here that **removes** a guiding-rule violation rather
     than adding a capability — brake's scalar path into the aerodynamic solve
     is exactly the control-to-aero shortcut rule 4 forbids;
   - it is Level 7's own gate, unmet;
   - it is expected to reach item 0b (turn rate) by the same mechanism, so two
     items close on one channel;
   - and it is arithmetic on numbers the line solver **already computes and
     throws away**, which is a smaller change than its consequences suggest.

   **Half of it is now built, and the last reason above was wrong** (§72). The
   aerodynamic half exists and is gated: `sectionIncidenceOffsetRad` carries
   **8543 N·m/rad** of roll, linear to 0.03% over sixteen times the range,
   mirror-exact, and lift-preserving. The structural half is **identically
   zero** — every canopy attachment is placed as one rigid body, so the
   per-station pose is the root scalar again, bit-identical left to right at
   full weight shift while the A row carries 51 N one side and 349 N the
   other. The arithmetic was available; what was not available was anything
   for it to vary with.

   So this **leaves the head of the queue**. What replaces it is not a bigger
   version of it: because the gain is linear, the requirement divides out, and
   the question in front of the channel is structural — *does a canopy on its
   lines twist that far under a 300 N row-tension difference?*

   **And then that question turned out not to be the first one either** (§73).
   Imposing the twist and flying the aircraft gives **0.0272 rad/s per degree,
   linear**, so 0.20 rad/s wants about **seven degrees** — but **the aircraft
   spirals at four**, winding up to 3.48 rad/s with the safety envelope never
   engaging. A stable turn tops out near 0.09 rad/s, so a real wing's turn is
   on the far side of a departure and **perfect canopy torsion would not reach
   it**. Every stable turn also banks only 38-40% of its own coordinated bank,
   at a constant fraction across a sixfold input range.

   Both are new items (**25**), both are unowned, and both are measurable
   today without building anything. **They are in front of item 21**, because
   a channel whose output the aircraft cannot use is not worth a level.

6. **Level 11, the unsteady wake.** Confirmed on the critical path by step 2.
   It is specified by measurement rather than by ambition: the entry criterion
   is a separated solve that is single-valued, and `coupled_tests` already
   contains the gate that would show it — the symmetric frontal must not lose
   mirror symmetry on an aerodynamic tick, and a drag correction that lands the
   published glide must not engage the safety envelope.

**Levels 12–15 remain out of scope**, and the reason is now empirical rather
than budgetary: Level 11 alone has taken four sessions to *specify*.

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

## The geometric channel — Level 7's unfinished half

**Reopened, with evidence.** Level 7 was marked done and every exit gate was
recorded as passing. One of them is:

> Weight shift and brake turns **emerge** without direct turn moments.

That gate has two halves and only one was ever checked. *No direct turn moment
exists* — true, verified, and worth having. *A turn emerges* — false. Measured
(§71), full weight shift produces **0.014 rad/s**, against 0.20 in the legacy
model and something like 0.2–0.3 on a real EN-B. The absence of a shortcut was
confirmed; the presence of the physics that was supposed to replace it was not.

### What is actually missing

The suspension solve computes the 3-D position of every node in the line
network — `SuspensionSolution::nodePositionM`, one entry per attachment — and
then publishes this:

```cpp
// Change in root-chord incidence from the unloaded design pose. This is
// the only path bar and brake have to the canopy.
double incidenceChangeRad = 0.0;
```

**One scalar, at the root.** Everything spanwise in the solved geometry is
computed and discarded. `VsmSolveInput` correspondingly has no channel for it:
it carries airspeed, angular velocity, density, per-cell internal pressure,
left/right brake and a per-section gust, and nothing from the suspension at all.

So the consequences follow directly and all three are measured:

- **Weight shift does nothing** (item 21). Its only surviving path is
  translating the pilot's CG 7.1 cm on a 6.6 m hang, which is 0.6° of bank from
  geometry. The 34% riser load asymmetry it genuinely produces has nowhere to go.
- **Brake works, badly** (item 0b). Brake has a *second* path — it is passed
  into the aerodynamic solve as a scalar pair. That is why brake turns at all.
- **And that second path is the thing this plan exists to remove.** Guiding
  rule 4 says brake input changes brake-line rest length and does not directly
  command a turn. The rest length does change, and then a control scalar is
  handed to the aerodynamics anyway. The geometry-driven stack is
  geometry-driven everywhere except the one interface where it matters most.

### The design

**One idea: the canopy's per-section pose is read off the solved line geometry
rather than assumed.** That is guiding rule 1 applied to the interface that
currently violates it.

1. **The suspension solve publishes a per-station pose.** For each span station
   the VSM solves, read from the already-solved node positions:
   - the chord direction, from that station's front (A/A′) attachment to its
     rear (C) attachment, projected into the section plane;
   - hence a **per-section incidence offset** from the unloaded design pose;
   - the station's position and normal, giving the **deformed arc** — which is
     what converts sideslip into roll on a real wing.

   Nothing here is new physics or a new solve. It is arithmetic on numbers the
   line solver already produces and currently throws away.

2. **`VsmSolveInput` gains `sectionIncidenceOffsetRad`**, on exactly the
   pattern `internalPressureCoefficient` and `sectionGustBodyMps` already set:
   **empty means the design pose**, so every existing caller is unchanged and
   the change is additive rather than a migration.

3. **Brake's scalar path retires into it.** Pulling a brake shortens a line,
   which moves a trailing edge, which changes the camber and incidence of the
   stations that line reaches. Once the channel exists, `leftBrake`/`rightBrake`
   no longer need to be handed to an aerodynamic solver as controls. That closes
   guiding rule 4 for real, and it is the same mechanism that gives weight shift
   its authority — one channel, three controls.

4. **Weight shift then works without being given anything of its own**, which
   is the test of whether the design is right. If it needs its own term, the
   channel is wrong.

### What this is expected to fix, and what it is not

| | |
|---|---|
| item 21, weight shift authority | directly — this is its diagnosis |
| item 0b, turn rate several times too slow | expected, same mechanism, not assumed |
| guiding rule 4's control-to-aero shortcut | retires it |
| item 12, the drag deficit | **no** |
| the separated-regime blocker (Level 11) | **no** |

**Explicitly not claimed:** that this produces the right *magnitude*. The
design says the channel exists and carries the right sign; whether 34% of riser
asymmetry becomes 0.2 rad/s of turn is a measurement, and the first milestone
is that measurement rather than a finished turn rate.

### Risks, named rather than discovered later

- **It closes a feedback loop.** Aerodynamic load → line tension → canopy
  geometry → aerodynamic load. Level 7's staggered coupling already exists and
  this rides it, but the loop is new and may need more iterations or more
  relaxation. Level 7's own convergence gate — *one fewer iteration changes
  nothing qualitative* — is the test, and it must be re-run rather than
  inherited.
- **Level 6 is one-dimensional.** The membrane is strips at chord stations, so
  arc deformation is under-resolved. The incidence offset does not depend on
  the membrane and should land regardless; the arc/sideslip half may not.
- **Cost.** More coupling iterations against a solver that runs 15× real time
  at full fidelity — headroom exists and is measured (`SOLVER_PROFILE.md`).
- **It may not be enough.** If the channel carries the right sign and an order
  too little authority, the next suspect is the harness geometry the CG
  translation rests on, and that is a different item.

### What was built, and the risk none of the above named (§72)

**Step 2 is done.** `VsmSolveInput::sectionIncidenceOffsetRad` exists, empty is
the design pose bit for bit, and it is gated in `aerodynamics_tests`: the sign
is right, the gain is **8543 N·m/rad and linear to 0.03%** across sixteen times
the range, four degrees of twist changes lift by 0.7% so it is a couple, and
left and right mirror to 1e-9.

**Step 1 cannot be built, and the reason is upstream of it.** Every canopy
attachment in `TensionCableSolver` is placed as
`canopyOrigin + canopyAttitude.Rotate(node.canopyLocalM)` — one rigid body — so
the chord direction from A to C at any station differs from the design pose by
a single global rotation. The per-station pose *is* `incidenceChangeRad`, with
a longer derivation. Measured at full weight shift, the offsets are identical
to eight decimals across the span and bit-identical between left and right,
while the same solve puts the A row at 51 N left against 349 N right.

**The risk the list above missed is not in this design's mechanism but in its
premise:** it was derived from what the line solver *publishes*, and blocked by
what the line solver *represents*. Every node position is computed, so reading
a pose off them looked like arithmetic on available numbers — and it is, and it
returns a constant, because the rigidity sits upstream of the publishing.
Checking what a proposed input can actually *vary* is the cheap version of the
whole exercise, and it is a general check rather than this design's mistake.

**What it leaves is better posed than what it started with.** The missing
ingredient is canopy torsional compliance, which exists nowhere: Level 2's
canopy is rigid and Level 6's membrane is 1-D chordwise strips. And because the
gain is linear, the requirement divides out — **about 10° of antisymmetric
twist to match today's full-brake roll**, which is itself several times slow.
So the next question is structural, has a number attached, and settles item 21
and item 0b in whichever direction it falls: *does a canopy on its lines twist
several degrees under a 300 N row-tension difference?*

### Exit gate

- Weight shift produces a turn that **emerges from the riser asymmetry** — the
  gated 34% split drives it, and no weight-shift term appears anywhere in the
  aerodynamic solve.
- `leftBrake` and `rightBrake` are gone from `VsmSolveInput`, and brake turns
  are at least as good as they are today.
- The mirror-symmetry gates still hold to round-off (§65–§71).
- Level 7's convergence gate passes again, re-run rather than inherited.
- Turn rate against a pilot's number, not against a coefficient.

**And the gate itself is now behind the PITCH axis** (§73, corrected by §74 and
§75). *"Turn rate against a pilot's number"* cannot be met at all: the aircraft
departs at 0.19 rad/s, below the 0.2-0.3 a real EN-B holds as an ordinary turn.
Not caused by the channel and not fixed by it.

Two things §73 said about that are wrong and are corrected here, because both
pointed at work that does not need doing:

- The "60% bank deficit" is not a bank deficit. `bankRad` is the canopy's
  angle; the payload link, carrying 95 of the 105 kg, sits at the coordinated
  bank within 2%, and the measured sideslip is under 0.1°. The canopy sits
  inboard by exactly the line roll spring's deflection under the twist's steady
  roll moment. **There is no skid to fix** (§74).
- The departure is not a spiral. Walked to the edge, the turn envelope ends at
  CL 0.461 and the *accelerator* — same aircraft, same CL scale, no turn at all
  — ends at CL 0.425, indistinguishable at the resolution either sweep has.
  **It is item 11's pitch divergence reached through a turn** (§75).

So the blocker in front of this gate is **item 11**, and it was already the top
of the list. Nothing here waits on a roll, yaw or spiral mechanism.

**Split by §72, because the two halves now fail for different reasons.** The
aerodynamic gates are met: the channel exists, carries the right sign, is
linear, and mirrors to round-off. The rest are unmet and are all downstream of
the same missing thing — nothing can supply a twist, so weight shift cannot
emerge from riser asymmetry, brake's scalar path cannot retire, and there is
no turn rate to hold against a pilot's number. **Level 7 stays reopened, and
what it is now blocked on is canopy torsion rather than an interface.**

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
