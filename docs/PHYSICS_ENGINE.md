# The physics engine as built

What exists, what it is checked against, and what it cannot yet do. Written
against the tree at the end of the Level 9 work; the master plan
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

**The geometry-driven stack** (Levels 1-9) is built, tested and not yet wired
into flight. It is exercised by its own suites and by the debug views. It now
agrees with the published envelope on four independent numbers at the weight
they were published at, and has a measured and narrow envelope: hands-up to
about a quarter brake. Both are in `docs/CALIBRATION_REPORT.md`.

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
there was nothing in the model that could surge. Completed at Level 9, because
the first version of it was still counting one thing twice.

The payload was pinned straight below the canopy in body axes. The wing could
pitch, but it could never *swing*, and the angle between wing and pilot — which
is most of what a pilot feels in pitch — did not exist as a quantity. Three
things followed, none visible until the degree of freedom was added: no surge;
an accelerator that changed the airspeed by nothing at all; and a fictitious
pitch spring, a lumped "pendulum moment" restoring the canopy toward level,
when a mass hanging from a single point has no pitch stiffness whatsoever.

### The link, in world axes

The pilot is a link — a unit vector from canopy to payload, held in **world**
axes with its own angular velocity — driven by

```
a_rel = g − a_pivot + harness drag / m
a_t   = a_rel − e (e · a_rel)
alpha = (e × a_t) / L
```

and by nothing else. The line spring does not appear in it: a moment is not
something you can apply to a bob on a string, and the lines' reaction to the
canopy's spring lands on the harness, whose inertia about its own centre of
mass is 5 kg·m² rather than the swing coordinate's 6200.

Holding the link in world axes is the whole point. As a body-frame angle,
rotating the canopy carried the pilot with it, so gravity's restoring torque
appeared once in the swing equation and again as the lumped body's weight
moment — 14000 N·m/rad of pitch stiffness where the lines provide 6300. In
world axes the link knows nothing about the canopy except through the lines,
each restoring torque is written exactly once, and a wing banked at 45° with
its pilot swung out under it has no line stress and no restoring moment, which
is what a coordinated turn is.

### The spring is geometric, and scales with load

`TensionCableSolver` holds the canopy at an imposed attitude and reports the
moment the lines exert there. Probed ±0.02 rad either side of the free
equilibrium, at four loads:

| load | pitch stiffness | roll stiffness |
|---|---|---|
| 0.5 g | 3306 N·m/rad | |
| 1.0 g | **6317** | **8204** |
| 2.0 g | 11512 | |
| 4.0 g | 15393 | |

Proportional to load, because the spring is **geometric rather than elastic**:
the lines stretch 0.2% while the canopy's origin moves 0.13 m, so the wing is
pivoting about a virtual hinge **6.62 m below itself** and the restoring moment
is a tension times an arm.

Freezing it at its one-g value — which the first attempt at this rewrite did —
makes the pitch axis diverge, because the aerodynamic moment it answers scales
with dynamic pressure and a constant spring loses to it at speed. The wing
stalled at 35 s and stayed there.

Two consequences of the same hinge:

- **Inertia.** Rotating the canopy drags its own mass and its apparent mass
  through a 6.62 m arc, so pitch inertia is 120 + (m + m_apparent)·h², not 120.
- **Not the relative wind.** The canopy really does travel through that arc,
  but adding the arc velocity to the relative wind while moments are still
  summed about the canopy's *origin* is not a half-measure, it is the wrong
  sign — the extra speed raises dynamic pressure and lowers incidence, both of
  which increase the moment that produced the rotation. Measured: negative
  damping, and the wing left the envelope at 250 m/s. Summed about the hinge
  instead, the aerodynamic force's arm is cancelled by the line tension's.

The probe needs its 12000 iterations. Held at 0.02 rad it returns 19849 N·m/rad
at 120 iterations, 9228 at 2000 and 6371 at 48000 — which is why the warm
started in-flight solve cannot be asked this question, and why "just read the
live network's moment" returns noise.

### Brake and bar both rotate the wing on its lines

Bar shortens the A and B risers; brake pulls the trailing edge down and rotates
the canopy nose-**up**. Both move the spring's unstressed angle and both are
measured off the network rather than asserted.

Brake had exactly the bug the accelerator had before Level 7 found it, in the
other direction: the section camber change reached the aerodynamics as a large
nose-down couple while the rotation that answers it was missing, so 40% brake
pitched the wing down into a fully separated stall. The brake curve is sampled
at explicit stations rather than evenly, because the 120 mm of sewn-in slack in
a 620 mm travel puts a corner in it at 19% and an even spacing interpolates
straight across — worth 0.83 against 0.30 of fold on a wing at the edge of one.
Inside the slack the offset is **exactly** zero, not merely small.

### A simulation that starts mid-flight starts trimmed

The canopy's pitch equilibrium is not its hang pose: the wing carries a 327 N·m
nose-down camber couple, so it sits about 3.3° below where the lines alone
would hold it. Starting at the hang pose is a 3.3° step input into a spring
with a damping ratio near 0.14, which rings to twice the offset, which takes
incidence from 6° to 0.3°, which takes the **load** off the lines — and a
geometric spring with no load has almost no stiffness left. Measured: 976 N and
5727 N·m/rad at a tenth of a second, 207 N and 989 N·m/rad two seconds later.

Same reasoning as seeding an inflated canopy. An initial condition of zeros is
not a wing, and on a stiff lightly damped axis it is an impulse.

### What it produces

Hands-off, brake to 0.30 for three seconds and release, canopy ahead of the
pilot **along track** — measured in the world, because the body-axis version
mixed in the canopy's own attitude and cancelled the pilot's swing exactly once
brake started rotating the canopy:

| | |
|---|---|
| trim | 0.34 m |
| under brake | −0.53 m (the wing comes back over the pilot) |
| top of the surge | 1.16 m |
| surge period | 3.87 s, against 5.70 s for a simple pendulum on the same lines |

The aircraft has **two** pitch modes an order of magnitude apart — the wing
swinging against the pilot near four seconds, and a speed-and-incidence mode
near twenty — and any identification has to say which one it is measuring.

### One number holds this up, and it is stated rather than derived

`swingDampingRatio` is 0.35. At 0.20 — what a wing settling in three swings
implies — the pitch axis diverges. What it is really doing is not damping but
**tracking**: the pendulum has to follow apparent gravity, which in a pull-up
swings round with the flight path, and a lightly damped one follows late. At
0.20 the link tracked 10.7° of a 14.6° flight-path change and the missing 3.9°
went into incidence.

That matters because this wing's pitch feedback has a loop gain of
`a·c·Cm/(k·CL²)` — measured off its own polar and its own suspension — which is
0.32 at trim and passes **one** at CL 0.35. Estimated from what physically
damps the swing, the ratio should be nearer 0.06. Registered Tuned/Unvalidated;
`PHYSICS_TODO` item 11.

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

## Level 9 — calibration

`CalibrationManeuver`, and the `calibration_tests` suite. **Built.** The full
report is `docs/CALIBRATION_REPORT.md`; the half of the exit gate that needs
people is `docs/PILOT_REVIEW_PROTOCOL.md` and has not been run.

Eight repeatable still-air manoeuvres on the coupled solver, each settled 90 s
before its input and recorded for 45 s — long, because the aircraft's slow mode
has an eleven second period and a fifteen second settle left every manoeuvre
reporting NOT SETTLED, honestly. Each applies one input. The series exports as
CSV.

**Flown at the published 105 kg all-up**, made up by ballasting the default
payload. This is not a detail: trim speed goes as the square root of wing
loading, the EPIC 2 ML's envelope is a 105 kg figure against a 90–110 kg range,
and the solver's unballasted payload is 94.3 kg — 5.5% of speed built into the
comparison rather than into the model.

### Against the published envelope

| | model | published | |
|---|---|---|---|
| trim speed | **39.4 km/h** | 39.0 | +0.4 |
| sink at trim | 1.15 m/s | 1.14 | +0.01 |
| glide at trim | 9.43 | 9.5 | −0.07 |
| incidence | 5.02° | 5.30° for the published trim CL of 0.580 | −0.28° |

**One parameter was identified and three numbers were not.** The fitted one is
the line plan's design incidence — where the rest lengths are cut — which
`bgd-epic-2-ml-lineplan.json` has always named as the quantity to identify
("replace with the manufacturer's rigging angle, or by fitting Level 4 trim
speed"). It moved from a round 5.0° to 4.4°. Sink, glide and incidence follow
without being fitted, which is the test; trim speed on its own is a restatement
of the fit.

This number was 31.9 km/h for most of the project's life, and the 18% shortfall
was the doubled pitch stiffness above — not the lift curve, which the first
diagnosis blamed and which tests out close to right.

### The manoeuvre set

| manoeuvre | settled result |
|---|---|
| hands-up trim | 10.95 m/s, sink 1.15, glide 9.43, α 5.0° |
| accelerator step | **departs** — α 88° |
| brake step 25% | 8.82 m/s, sink 0.78, α 9.7° |
| deep brake step 40% | **departs** — α 95° |
| brake pulse and release | surge period 3.87 s, damping 0.06, 1.7 m of travel |
| weight shift step | 0.014 rad/s at 1.5° of bank |
| coordinated turn 35% | 0.045 rad/s at 1.5° of bank |
| stall approach | minimum 6.31 m/s, separated |

Direction, mirroring and ordering of the controls are correct and were checked
against **world vectors** rather than a sign convention: right brake turns the
ground track +1.217 rad toward +Y with the right tip 0.030 below the horizon,
and left brake mirrors it to four digits. The suite previously asserted that
turn rate and bank carry opposite signs; that was backwards, and it passed
because the old model banked the wrong way.

### Three disagreements, each bounded

1. **The wing turns several times too slowly.** 0.045 rad/s at 1.5° of bank on
   35% brake against an EN-B's 0.3 at 20–30°. Was 0.015; removing the payload's
   `m L²` from the canopy's roll inertia and the world-referenced roll spring
   tripled it. The largest one left.
2. **It cannot hold full bar.** The pitch loop gain passes one at CL 0.35 and
   full bar is a CL 0.31 condition, so the wing is statically pitch-divergent at
   its published top speed. Half bar departs too, more slowly. No amount of
   damping fixes a gain above one; the two levers are the section pitching
   moment and the specific stiffness of 6.13 m.
3. **The polar's lift ceiling costs the envelope.** Swept on the VSM the
   analytic polars peak at **CL 0.866 at 11°** where this wing's profile
   carries 1.32. Trim at 5.0° leaves barely six degrees of brake before the
   wing is past its own stall, and past it there is no steady state to return
   to. **The usable envelope is hands-up to about 25% brake.** This is item 1 —
   analytic thin-airfoil polars, blocked on XFOIL — arriving where a pilot
   would feel it, and the single most valuable thing real section data buys.

Energy: 1.1 W at trim and 1.2 W under weight shift on a 1030 N aircraft; up to
38.8 W through a pitch transient, attributed to the pendulum's own energy
sitting outside books that still track one lumped translation.

## Test suites

`Tools/check-build.sh` builds the Unreal module and runs twelve suites. All
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
| `calibration_tests` | Level 9's still-air manoeuvres |
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
2. **Section polars are analytic, and Level 9 measured what that costs.** No
   XFOIL runs, no measured data. Swept on the VSM they peak at CL 0.866 at 11°
   of incidence where this wing's profile carries 1.32, which puts the stall
   six degrees above trim and makes 40% brake — an ordinary EN-B input —
   unholdable. **The usable envelope is hands-up to about a quarter brake.**
   The same sweep gives Cm near 0.10 across the range, which is half of the
   pitch loop gain that costs the other end of the envelope.
3. **The wing is statically pitch-divergent below CL 0.35**, and full bar is a
   CL 0.31 condition. The loop gain `a·c·Cm/(k·CL²)` is 0.32 at trim. Neither
   input to it is tunable: both were measured, one off the VSM and one off the
   suspension graph.
4. **`swingDampingRatio` is stated, not derived, and hands-off stability
   depends on it.** 0.35 where the physics suggests 0.06. It is standing in for
   a stabilising mechanism the model does not have. `PHYSICS_TODO` item 11.
5. **The apparent-mass rotational terms are disputed** (above).
6. **Grindelwald First's anchor is 50 m off its surveyed ground.** Published
   2123 m is the top station; the WGS84 pair is on the launch slope below it,
   which the survey puts at 2073 m. Every other site agrees within 12 m. The
   terrain is the measurement and the anchor is the estimate, so it is recorded
   rather than fitted away. `terrain_survey_tests` prints the table.
7. **None of the geometry-driven stack flies the wing.** The legacy polar still
   does. Level 8's deliverable of a collapse/reopening *debug view* is blocked
   on this and nothing else: the pawn draws collapse from the legacy
   telemetry, because that is what is flying. The per-section state the view
   would draw — margin, external Cp, fold, propagation flag, fold reach past
   the line, cravat — is already reported by `SectionCollapseDiagnostics`.
8. **The LOCAL part of the reopening surge is not modelled.** The whole-wing
   part now is — the wing and the pilot are two bodies on a real spring, so a
   recovery swings the wing forward the way a brake release does. What is
   missing is the *shape*: a real frontal recovery has the nose catching air
   and scooping forward, which needs the membrane's fold geometry read back
   into the aerodynamics.
9. **A cravat has never formed in the coupled solve.** It forms in
   `collapse_tests`, from the built graph's real 0.178 m tip line gap against a
   fold deep enough to reach past it. In flight the strip's fold depth stays
   short of that gap, so the contact test correctly returns nothing. Whether
   that is the wing or the one-dimensional strip understating how far skin
   hangs is not yet known, and the 2-D mesh is what would answer it.
