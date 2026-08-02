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

**The geometry-driven stack** (Levels 1-8) is built, tested and not yet wired
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

                 └─→ CanopyCollapseSolver ←─── pressure, incidence, load,
                        (Level 8)                skin slack, line geometry

           CoupledParagliderSolver              (Level 7, complete)
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
endpoints.

**Self-collision was built and then removed**, and the reason is a result
rather than a decision: a one-dimensional strip cannot self-intersect. The
strip was collapsed with the ribs drawn from full spacing down to a tenth of
it, and the segment-crossing count was zero at every step while the fold
deepened from 31 mm to 121 mm. Crumpling needs loads that reverse along the
skin, which needs the 2-D mesh this level does not have. Level 8's cravats do
not need it either, because a cravat is fabric caught on a *line*.

## Level 7 — coupled solver

`CoupledParagliderSolver`. **Complete.** Trim matches the published wing, turns
emerge, and the suite is green and runs with the rest.

The schedule runs all six solvers in order at fixed 120 Hz with staggered
aero/structure coupling, the VSM at a reduced rate with the rate-dependent part
of its load re-evaluated every step, and the line network warm-started.

Hands off, it settles at:

| | coupled solver | published EPIC 2 ML |
|---|---|---|
| trim speed | 10.70 m/s = **38.5 km/h** | 39 km/h |
| sink | **1.12 m/s** | 1.0 m/s min sink |
| glide | **9.5** | 9.5 |

Net force and moment go to zero. Ten minutes holds between 10.55 and 10.70 m/s.
Internal force closure is exact and the energy residual stays under 4 W on a
925 N aircraft. Taking a coupling iteration away changes airspeed by 0.002 m/s,
so the staggered solve is converged rather than tuned to its budget.

**Turns emerge and mirror.** Right brake at 0.35 held for ten seconds settles at
0.094 rad of bank and 0.030 rad/s of yaw, with no control-to-bank or
control-to-yaw term anywhere in the file. The same case braked left agrees with
it to 2×10⁻⁸ rad after ten seconds of coupled flight.

**Heavy brake stalls the wing rather than breaking the solve.** The brake sweep,
each case flown for eight seconds from trim:

| brake | alpha | separation | sink | Cp |
|---|---|---|---|---|
| 0.35 | 6.3° | 0.00 | 0.82 m/s | 0.90 |
| 0.60 | 14.9° | 0.38 | 1.56 m/s | 0.85 |
| 0.75 | 29.6° | 0.90 | 3.77 m/s | 0.67 |
| 0.90 | 46.1° | 1.00 | 4.65 m/s | 0.32 |

Monotone in every column, and the numerical safety envelope does not engage
anywhere in it. That envelope still exists — a non-finite or implausible
aerodynamic solve is rejected and the previous load held, reported separately in
the diagnostics under guiding rule 12 — but the coupled solve no longer needs it
to get through a stall. Level 4's separation memory is what makes the separated
branch well posed here; the VSM asked for a cold steady solve in deep stall
still has no answer, and that stays a locked known-failure check.

**What was wrong, and it was in this file rather than in the harness.** The NaN
turn rate reproduced at once in an unoptimized build, having hidden at -O2:

1. **Damping was integrated explicitly.** Measured roll damping is 3.7×10³ Nm
   per rad/s against an inertia of 95, and yaw damping 80 against 150. Explicit
   `c·omega` at 120 Hz needs `c·dt/I < 2`; yaw sat at eleven times that, so the
   damping term alternated sign and doubled every step. It is now backward Euler
   on the damping term alone — one divide, unconditionally stable at any
   coefficient.
2. **The damping derivative was ill-posed.** It divided the live moment
   difference by the live rotation rate, with a guard that zeroed it below
   10⁻³ rad/s. Near zero rate that is noise over nothing, and the guard made the
   damping law discontinuous in the state: two mirror-image flights took
   different branches of it within four seconds and stopped being mirror images.
   It is now a centred finite difference at a fixed ±0.3 rad/s, one axis per
   aerodynamic interval, each refreshed every 0.3 s. Centred matters — a
   one-sided probe is not odd in the rate, so left and right measure different
   coefficients.
3. **The probes solved a different wing.** They called the cold `Solve` capped
   at 40 iterations, where a cold solve needs about ninety, and got the
   equilibrium separation for whatever incidence they landed on rather than the
   separation state the wing actually had. The coefficient moved 10% between
   consecutive intervals. `SolveFrozen` now holds the live separation state and
   continues each probe's own circulation, which converges in a handful of
   iterations — the suite got roughly ten times faster as a side effect.
4. **The aerodynamic gate bounded force but not moment.** One step near stall
   returned a *converged* solve carrying 34 kNm of yaw against a `q·S·b` of
   14 kNm. It was accepted, every step after it was rejected, and the envelope
   dutifully held that number for ten seconds — 26 kNm on an inertia of 150 is
   a turn rate of 100 rad/s, and then infinity. The gate now bounds the moment
   at twice `q·S·b`, on the accepted solve and on both probes.

## Pitch: the wing and the pilot are two bodies

Added after Level 8, because Level 8's exit gate asks for a reopening surge and
there was nothing in the model that could surge.

The payload was pinned straight below the canopy in body axes. The wing could
pitch, but it could never *swing*, and the angle between wing and pilot — which
is most of what a pilot feels in pitch — did not exist as a quantity. Three
things followed from that, none of them visible until the degree of freedom was
added:

- **No surge.** A brake release could not send the wing out ahead of the pilot,
  because there was no "ahead".
- **The accelerator did nothing.** Bar shortens the A and B risers by 120 and
  80 mm, which rotates the wing nose-down on its lines. The line network
  modelled that correctly and always had; the flight model never read the pitch
  it produced, so pulling full bar changed the airspeed by *nothing at all*.
- **The wing had a fictitious pitch spring.** A lumped "pendulum moment"
  restored the canopy toward level. A mass hanging from a single point has no
  pitch stiffness whatsoever — it is free to rotate about the attachment.

What actually holds a paraglider at its incidence is the line geometry: A, B
and C attach at different stations along the chord, so rotating the canopy
lengthens one row and shortens another. That is measurable, and it is now
measured rather than asserted — `TensionCableSolver` gained a mode that holds
the canopy at an imposed attitude and reports the moment the lines exert there,
and the solver probes ±0.02 rad either side of the free equilibrium at
construction:

| | |
|---|---|
| line pitch stiffness | 5723 N·m/rad, linear to within 3% over ±0.06 rad |
| free hang angle | 4.75° nose-up |
| zero of the spring | at that hang angle, to 0.8 N·m |

The pilot is then a bob on a 7 m line whose pivot is being accelerated by the
air, with the spring acting once — written from the potential so the canopy's
moment and the pilot's reaction cannot double-count.

Measured, hands-off, brake to 0.7 for three seconds and release:

| | canopy ahead of pilot |
|---|---|
| trim | 0.77 m |
| under brake | 0.00 m (the wing comes back over the pilot) |
| top of the surge | 1.85 m, at 0.16 rad/s |

and the accelerator now does something: 8.21 m/s at 11.8° hands-up, 9.58 m/s at
6.9° on full bar. A gust that does nothing to the wing hands-up folds it on
bar, which is the collapse-proneness of accelerated flight arriving on its own.

**One defect fixed on the way in, one introduced and caught:** the harness drag
was being applied as a moment on the canopy *and* as a force on the pilot once
the pendulum existed — the same force pitching the wing twice. It acts on the
pilot and reaches the wing through the lines, so only the lines' own drag
moment stays on the canopy.

### The trim disagreement this exposed, and what was behind it

Trim is **8.86 m/s (31.9 km/h) against a published 39**, full bar 41.3 against
a published 53, glide 9.04 against 9.5.

The previous model agreed with the published 39 km/h almost exactly, and that
agreement was two errors cancelling: the canopy was pinned level, which forced
the wing to fly at 4.5° of incidence, and 4.5° is not what a paraglider flies
at.

**The first diagnosis of the remaining gap was wrong and is recorded as such.**
It blamed the analytic lift curve. Testing that against the published envelope
says otherwise:

| | |
|---|---|
| CL the published trim speed requires | 0.580 |
| incidence at which this model makes it | **5.30°** |
| CL the published top speed requires | 0.314 |
| incidence at which this model makes it | **0.54°** |
| incidence change the published speed range demands | 4.76° |
| incidence change the risers geometrically produce | **4.06°** |

The lift curve is close to right — it produces the published CLs at sensible
incidences, and the riser geometry spans most of the incidence range the
published speed range demands. The bar-to-trim speed ratio comes out 1.294
against a published 1.359. What was wrong was the wing's **pitching moment**,
and two defects were behind it:

1. **The section pitching moment was four times too small.** For a circular-arc
   camber line the Fourier coefficients of the camber slope are A1 = 4h/c and
   A2 = 0, giving `Cm_c/4 = (π/4)(A2 − A1) = −π h/c` — which is −0.110 at 3.5%
   camber. The code had `−(π/4)(h/c)` = −0.0275, which is what taking A1 = h/c
   gives: the factor of four in the Fourier coefficient dropped while the π/4
   prefactor was kept. The *same* A1 gives the zero-lift angle −2h/c, which was
   right, so one camber line was being described two ways.
2. **It was never applied to the wing at all.** The VSM's moment was only the
   cross product of the section forces about their quarter-chord positions. A
   section's own camber couple — which survives where the section makes no lift
   and is precisely what sets a wing's trim incidence — was computed by the
   polar table, stored, and discarded.

Neither could be caught before, and for the same reason as everything else on
this page: with the canopy pinned, its incidence was set by the pin. A wing
whose pitching moment is missing entirely flies exactly as well as one whose
pitching moment is right, as long as nothing is free to pitch.

Fixing both moved trim from 29.5 to 31.9 km/h and incidence from 11.8° to 9.1°,
and dropped the spurious turn in a symmetric frontal from 1.055 rad/s to 0.017.
`aerodynamics_tests` now checks the moment against the closed-form thin-airfoil
result and against the zero-lift angle it must share a camber line with.

**The remaining gap is most likely pitch stiffness, not aerodynamics.** The
wing still flies at 9.1° where the published CL implies 5.3°. The rigid motion
section integrates canopy and payload as one body whose centre of mass is eight
metres below the reference point, so gravity's restoring torque appears there
*and* in the swing degree of freedom — about 14000 N·m/rad of pitch stiffness
where the lines themselves provide 5723. Too stiff in pitch is exactly an
incidence that will not come down under an aerodynamic moment. Deleting the
lumped term is not the fix (measured: the wing goes to 157° of incidence,
because that term is load-bearing for the lumped body); giving the canopy and
the payload their own bodies is. Written up in `PHYSICS_TODO.md`.

## Level 8 — emergent collapse

`CanopyCollapseSolver`, and its integration into the coupled solve. **Built.**

The criterion is a pressure balance across the nose skin, which is what a
leading edge physically is:

```
margin = Cp_internal - Cp_external(fold station)
```

Both sides were already solved. The internal side is Level 5's cell pressure.
The external side is the same rounded-nose distribution Level 5 feeds the
inlets with, `Cp = 1 - 4 sin^2 t`, read where the nose skin is rather than
where the opening is. Local unloading (Level 2/4) and skin slack (Level 6)
erode the margin. Nothing in it is a threshold on a control input.

The behaviours fall out rather than being written in:

| | mechanism |
|---|---|
| bar is collapse-prone | incidence drops, so the inlets recover less *and* the shoulder loses the suction holding the skin out — both sides of the balance move the wrong way for the same reason |
| brake does not fold a wing | it raises incidence, which deepens that suction and feeds the inlets. Brake stalls a wing; it does not fold one |
| a tip goes first | smallest cells, least air, furthest from the crossports feeding them |
| turbulence folds by unloading | a section carrying no load has slack A lines and nothing holding its nose forward |

**Cravats** are a contact test, not a risk model: Level 6's fold depth against
the real gap to the nearest line, measured off the built suspension graph
(`LineFoldGapM`). Three things must be true at once and in order — folded, the
fold reaches past the line, and the line then reloads and traps it — which is
why a cravat is rarer than a collapse. It latches: a cravat holds its section
folded because the fabric is physically inside the wing, which is why one ends
in a spiral where a collapse ends in a surge.

**In the coupled solve** the fold takes its cell's pressure out on the way to
the aerodynamics, and the section polars turn that into lost lift and a drag
penalty where the fold is. There is no collapse-to-yaw term anywhere; the turn
comes from where the remaining lift is. The half that folded stops carrying its
share, and the imbalance is handed to the line network.

Measured, hands-off, with 4 m/s of descending air over the left half for one
second:

| | |
|---|---|
| worst fold, left half | 0.68 |
| worst fold, right half | 0.04 |
| turn while folded | -0.08 rad/s, toward the folded half |
| load asymmetry to the lines | 0.67 |
| recovered to | 0.00, in 13 s |
| numerical safety envelope | did not engage |

Measured on full bar, because that is when a wing folds — and nothing says so:
2 m/s of sink over half the span does nothing hands-up and folds the wing on
bar, because bar moves both sides of the pressure balance the wrong way at
once.

The same air over the whole span folds both halves to the same peak, 0.688. It
does not stay mirror-symmetric through the event: the two halves differ by up
to 0.15 part way through and the wing develops a turn. That is not the collapse
solver, which is mirror-exact to 0.0e+00 given mirrored input — `collapse_tests`
checks it directly, because that is the only place the claim can be isolated.
It is the circulation solve under it, which has no steady state to find when
the wing is this deeply separated.

**Limits, stated:** past about 5 m/s of gust the wing does not come back — it
pitches into full separation and descends vertically at 7.5 m/s, which is the
deep-stall attractor of item 5 below and Level 11's problem, not this level's.
Mirror symmetry in a frontal holds to 1e-15 through the fold and breaks in the
recovery, where the VSM passes through the same non-converging branch. The
reopening surge is not modelled: the wing recovers its lift smoothly as the
fold clears, where a real one dives forward and then pitches back.

**Three defects were found by the exit gates, all in levels below:**

1. **Crossport flow depended on which end of the wing the loop started at.**
   `CanopyPressureSolver` read its neighbours from the array it was writing, so
   every cell saw its left neighbour advanced and its right neighbour not. On a
   symmetric frontal that put 0.1 of difference in Cp between mirror cells
   within a second, on a wing whose sections agree to 1e-15, and the wing
   turned. Now read from the start-of-step state.
2. **Brake reached the trailing edge through slack line.** The line network
   always knew about the 120 mm of sewn-in slack, because it is in the rest
   lengths; the aerodynamics did not, and took the handle position as a camber
   change directly. The first 19% of the travel was deflecting a trailing edge
   no line was pulling on — a violation of guiding rule 3 in the one place it
   was not being checked.
3. **The load reference was the wing's weight spread evenly.** Loading on an
   arced, tapered wing falls away toward the tips by a factor of three, so an
   even share read every healthy tip as half unloaded and folded it. The
   reference is now each section's own load in clean trim, solved once at
   construction.

## Test suites

`Tools/check-build.sh` builds the Unreal module and runs eleven suites. All
green.

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
| `collapse_tests` | Level 8's criterion |
| `coupled_tests` | Level 7, and Level 8's incident benchmarks |
| `terrain_survey_tests` | terrain georeferencing |

`coupled_tests` is the slow one: ten minutes of flight at 120 Hz plus a brake
sweep, all six solvers running. Run it unoptimized as well as in Release
occasionally. The NaN it used to report was invisible at -O2 and reproduced on
the first try at -O0 — the divergence was there in both, but the optimized build
took a different enough path through the same marginal instability to survive
the ten seconds the test measures.

## Coefficient registry

89 coefficients: 25 tuned, 78 unvalidated, 0 out of range. 28% tuned, down from
39% before this work — the geometry-driven levels replaced fitted numbers with
measured or derived ones.

Every coefficient carries a unit, source, valid range, calibration status and
the level expected to supersede it. Two are marked `Disputed`, deliberately:
the apparent-mass rotational terms, whose leading coefficients could not be
checked against the source paper and which disagree with the existing estimate
by a factor of fourteen.

## Terrain

Two surveyed swissALTI3D regions, both in one route frame — origin at the
Amisbuehl launch, +X along the Amisbuehl → Lehn route, +Y route-right — so a
coordinate means one thing everywhere regardless of which grid answers for it.

| region | bounds (local m) | grid | tiles |
|---|---|---|---|
| `interlaken` | x [-1800, 6100], y [-3500, 4500] | 396 × 401 | 81 |
| `grindelwald` | x [4500, 11500], y [-18500, -14000] | 351 × 226 | 50 |

Two grids rather than one covering both: the valleys are 20 km apart and the
ground between them is never flown, so a single grid would carry 250 km² of
terrain nobody sees. `TerrainModel` loads regions additively and the first
covering a sample answers for it; the renderer draws one region at a time and
rebuilds when a route change crosses between them.

**All ten routes now launch and land on surveyed ground.** The two Grindelwald
routes previously sat on an invented lane at y = -8500 — their intra-valley
geometry was right and the whole group was 20 km from the real valley, on an
analytic proxy that put the Grund landing field at 4683 m against a published
950 m, in air with no thermal field. Surveyed ground puts it at 948 m.

Weather is anchored per region too. The thermal triggers were a single
Interlaken set plus a `y > 5000 ? 7500 : 0` lane offset, and every authored
weather volume in every preset was placed in Interlaken coordinates, so a foehn
day 20 km away was smooth air over dead ground. Each region now carries its own
triggers and an offset that moves authored volumes onto its own corridor in all
three axes — Grindelwald's valley floor is 800 m higher, so an unshifted volume
sat underground.

Published site elevation against surveyed ground, which is an external check
rather than a golden value:

| site | error |
|---|---|
| Lehn | +0.4 m |
| Bergbo | -0.3 m |
| Niederhorn south | -1.4 m |
| Hoehematte | -1.7 m |
| Grindelwald Grund | -1.7 m |
| Amisbuehl oben | +2.2 m |
| Grindelwald Bodmi | +10.7 m |
| Hohwald | -11.5 m |
| Grindelwald First | **-50.3 m** |

## Known defects and gaps

Ordered by how much they matter.

1. **Deep stall does not converge** in the VSM solved cold, and will not: the
   separated branch has a negative lift slope, which inverts the downwash
   feedback between sections, and a wing in deep stall has no stable steady
   state. Level 11's unsteady wake is the honest treatment. Locked as a
   known-failure check. Inside the coupled solve, where the separation state is
   carried between steps, the wing does walk into stall — see Level 7.
2. **Section polars are analytic.** No XFOIL runs, no measured data. Every
   flight number above rests on theory.
3. **The apparent-mass rotational terms are disputed** (above).
4. **Grindelwald First's anchor is 50 m off its surveyed ground.** Published
   2123 m is the top station; the WGS84 pair is on the launch slope below it,
   which the survey puts at 2073 m. Every other site agrees within 12 m. The
   terrain is the measurement and the anchor is the estimate, so it is recorded
   rather than fitted away. `terrain_survey_tests` prints the table.
5. **None of the geometry-driven stack flies the wing.** The legacy polar still
   does. Level 8's deliverable of a collapse/reopening *debug view* is blocked
   on this and nothing else: the pawn draws collapse from the legacy
   telemetry, because that is what is flying. The per-section state the view
   would draw — margin, external Cp, fold, propagation flag, fold reach past
   the line, cravat — is already reported by `SectionCollapseDiagnostics`.
6. **The reopening surge is not modelled.** A section recovers its lift as the
   fold clears, smoothly. A real wing dives forward as the nose catches air and
   then pitches back, and that needs the collapsed section's shape rather than
   only its state.
7. **A cravat has never formed in the coupled solve.** It forms in
   `collapse_tests`, from the built graph's real 0.178 m tip line gap against a
   fold deep enough to reach past it. In flight the strip's fold depth stays
   short of that gap, so the contact test correctly returns nothing. Whether
   that is the wing or the one-dimensional strip understating how far skin
   hangs is not yet known, and the 2-D mesh is what would answer it.
