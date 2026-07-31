# The physics engine as built

What exists, what it is checked against, and what it cannot yet do. Written
against the tree at the end of the Level 7 work; the master plan
(`agent-data/GEOMETRY_DRIVEN_PARAGLIDER_MASTER_PLAN.md`) is the specification
and carries the per-level status.

Nothing here is validated for flight training. Every number is either published
manufacturer data, classical theory, or a declared estimate, and the coefficient
registry says which.

## What runs today

Two models exist side by side, which is guiding rule 11.

**The legacy flight model** (`ParagliderDynamics`) is what actually flies the
wing in the game. It is a single six-degree-of-freedom body with a fitted
polar, now driven by a real payload body and real carabiner loads (Level 3) but
still using tuned coefficients for most of its handling.

**The geometry-driven stack** (Levels 1-7) is built, tested and not yet wired
into flight. It is exercised by its own suites and by the debug views.

```
CanopyGeometry ──┬─→ SuspensionGraph ──→ TensionCableSolver
   (Level 1)     │      (Level 2)            (Level 2)
                 │
                 ├─→ VortexStepMethodSolver ←── SectionPolarTable
                 │      (Level 4)                  (Level 4)
                 │              │
                 │              ↓ per-section angle of attack
                 ├─→ CanopyPressureSolver ──→ internal pressure coefficient
                 │      (Level 5)                  │
                 │                                 ↓ feeds back to the polar
                 └─→ CanopyMembraneSolver ←────────┘
                        (Level 6)

           HarnessGeometry → PayloadRigidBody   (Level 3)
           ApparentMassTensor                   (Level 4)

           CoupledParagliderSolver              (Level 7, trim unstable)
```

## Level 1 — geometry

`CanopyGeometry`, `PanelUnfolder`, `BillowRelaxation`,
`Data/Wings/bgd-epic-2-ml-geometry.json`.

The canopy is manufactured, not sculpted. A parametric flat pattern produces the
inflated shape; flat span, flat area and projected span are exact by
construction rather than fitted. Billow emerges from the seam allowance through
a membrane relaxation - change the allowance and the section changes with no 3D
surface being edited.

Checked against: published EPIC 2 ML figures; isometric unfolding residual
2.4e-15; round-trip surface → pattern → surface.

One geometry feeds physics, rendering and the line attachments. The four
duplicate descriptions of the wing that used to live in the render path are
gone.

## Level 2 — suspension

`SuspensionGraph`, `TensionCableSolver`,
`Data/Wings/bgd-epic-2-ml-lineplan.json`.

90 nodes, 78 cables: carabiners, riser tops, 26 main lines matching the
published 3/1/4/3 per wing, cascade junctions, upper galleries, a brake fan.
Rest lengths are measured from the geometry at the design pose, and the
manufactured total comes out at **254.8 m against a published 254 m** - a check,
not a fit.

Row shares at trim are emergent: **A 36%, A' 11%, B 37%, C 16%**. Slack cables
transmit exactly `0.0`, not a clamped small number. Full bar drops incidence
4.3 degrees and unloads the C row to nothing, through riser geometry alone.

## Level 3 — payload and harness

`HarnessGeometry`, `PayloadRigidBody`.

The pilot's CG moves by hip travel modified by chest strap and harness class.
The two carabiners take W(1/2 ± e/s) - statics, checked against the lever arm
rather than a previous run. Weight-shift authority comes only from harness
geometry, so the split-leg harness out-turns the seatboard because its geometry
lets the mass move further.

One suspension length (8.08 m) now sets every pendulum period in the model,
measured off the built graph. It replaced three separate hardcoded 7.3s.

## Level 4 — aerodynamics

`VortexStepMethodSolver`, `SectionPolarTable`, `ApparentMassTensor`.

A spanwise circulation solve coupled to 2D section polars. Validated against
classical lifting-line theory, which is exact and needs no external data:

| check | measured | analytic | error |
|---|---|---|---|
| elliptical CL_α | 5.2630 /rad | 2πAR/(AR+2) = 5.2519 | 0.2% |
| elliptical CDi | 0.00917 | CL²/πAR = 0.00951 | 3.6% |
| rectangular span efficiency | 0.976 | ~0.9-0.98 literature | — |
| panel convergence, 20 → 120 | 0.46727 / 0.45928 | — | 1.7% |

On the EPIC 2 it converges in 88 iterations, symmetric input gives exactly zero
roll and yaw, left and right mirror exactly, and roll damping falls out of the
spanwise incidence change with no damping coefficient anywhere.

**Section polars are analytic, not measured.** Thin-airfoil lift with the
circular-arc zero-lift angle, brake as a trailing-edge flap through thin-airfoil
flap effectiveness, Viterna-Corrigan post-stall. All derived, all registered
Provisional. XFOIL runs over digitised profiles replace them.

**Installed drag** - lines, risers, harness, pilot - is 47% of the canopy's own
drag, and with it whole-aircraft glide is **9.46 against a published 9.5**. That
comes from the line plan's own 254 m and a literature harness area, not a fitted
polar, and it is the only published performance figure this level can hold
itself to before real section data exists.

**Stall has memory.** Separation is a state carried between solves, spreading at
12/s and clearing at 4/s, reattaching about 5 degrees below where it began.
Ramping brake up and back down traces a loop: CL 0.93 going up at 0.6 brake,
0.51 coming back down.

**Apparent mass** gives 1.1 / 18.0 / 33.6 kg. The normal term is a third of the
aircraft, and agrees within a tenth with the model's independent estimate.

## Level 5 — cell pressure

`CanopyPressureSolver`.

Cells are plenums fed through leading-edge openings. Two standard results carry
it: the stagnation point sits about (α − α_L0) below the chord line, and
pressure round a rounded nose follows the cylinder distribution
Cp = 1 − 4 sin²θ. Everything else is a consequence.

Stagnation runs **5.2 degrees below the chord on bar, 9.7 at trim, 14.3 braked**.
Inlets are cut where trim puts it, so they recover **Cp 0.97 at trim and 0.89 on
bar** — accelerated flight pressurising less well comes from the geometry, not a
rule. Past 30 degrees of adverse incidence the inlet is in suction and the cell
empties through its own opening, which is the front collapse arriving the same
way.

Cell volume is integrated from the solved inflated section: 0.31 m³ at root,
0.09 at tip, 9.9 m³ for the canopy. Pressure feeds back into section
performance — depressurising the outer third takes CL from 0.479 to 0.360 and
puts 652 N·m of roll on the wing.

## Level 6 — membrane

`CanopyMembraneSolver`.

A spanwise strip from rib to rib, XPBD, tension-only, with warp/weft/bias
anisotropy. Checked against the analytic arc rather than against itself:

| | solved | analytic |
|---|---|---|
| sagitta | 26.32 mm | 25.99 mm |
| strain | 0.060% | 0.064% |

The solved bulge is slightly deeper because the fabric genuinely stretches.
Bias-cut fabric stretches 8× more than warp-cut (0.49% against 0.060%), which
is what decides how a canopy folds.

**Scope:** strips at chord stations rather than a full 2-D mesh, ribs as fixed
endpoints, no self-collision.

## Level 7 — coupled solver

`CoupledParagliderSolver`. **Built, deterministic, and not in the test suite:
trim is not stable.**

The schedule runs all six solvers in order at fixed 120 Hz with staggered
aero/structure coupling. Internal force closure is exact. The first 200 steps
are clean — 10.86 m/s, 973 N of lift against a 925 N weight, 1.15 m/s of sink.
Over the next four seconds the wing decelerates, α climbs through the 10-degree
section stall to 90, and it settles descending vertically at 7.5 m/s.

The deceleration leads and the stall follows, so the open question is where the
energy goes. Suspects: the held load being stale in body axes while the flight
path rotates beneath it, and the starting incidence not being the one this polar
trims at.

## Test suites

`Tools/check-build.sh` builds the Unreal module and runs nine suites. All green.

| suite | covers |
|---|---|
| `physics_tests` | the legacy model, 605 assertions |
| `determinism_tests` | state hashing, solver clock, coefficient registry |
| `geometry_tests` | Level 1 |
| `suspension_tests` | Level 2 |
| `payload_tests` | Level 3 |
| `aerodynamics_tests` | Level 4 |
| `pressure_tests` | Level 5 |
| `membrane_tests` | Level 6 |
| `terrain_survey_tests` | terrain georeferencing |

`coupled_tests` exists and is excluded, because Level 7 does not pass it.

## Coefficient registry

89 coefficients: 25 tuned, 78 unvalidated, 0 out of range. 28% tuned, down from
39% before this work — the geometry-driven levels replaced fitted numbers with
measured or derived ones.

Every coefficient carries a unit, source, valid range, calibration status and
the level expected to supersede it. Two are marked `Disputed`, deliberately:
the apparent-mass rotational terms, whose leading coefficients could not be
checked against the source paper and which disagree with the existing estimate
by a factor of fourteen.

## Known defects and gaps

Ordered by how much they matter.

1. **The terrain frame disagrees with the flight frame.** `RouteFrame`, the
   heightfield and the content placement define +Y as route-left; the flight
   frame has +Y as right. Nothing converts. Measured consequence: foehn rotor is
   0.82 route-left and 0.15 route-right, so lee rotor lands on the wrong side of
   the ridge relative to the surveyed geography. This blocks the Level 0 exit
   gate and is the oldest open defect in the project.
2. **Level 7 trim is unstable** (above).
3. **Both Grindelwald routes are off the map** — outside the surveyed
   heightfield *and* the rendered extent. The analytic fallback puts the Grund
   landing field at 4683 m against a published 950 m, in air with no thermal
   field. They are selectable content.
4. **Deep stall does not converge** in the VSM, and will not: the separated
   branch has a negative lift slope, which inverts the downwash feedback between
   sections, and a wing in deep stall has no stable steady state. Level 11's
   unsteady wake is the honest treatment. Locked as a known-failure check.
5. **Section polars are analytic.** No XFOIL runs, no measured data. Every
   flight number above rests on theory.
6. **The apparent-mass rotational terms are disputed** (above).
7. **None of the geometry-driven stack flies the wing.** The legacy polar still
   does.
