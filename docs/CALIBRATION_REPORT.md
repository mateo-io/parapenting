# Level 9 calibration report

What the geometry-driven model says, measured off repeatable still-air
manoeuvres, set against the manufacturer's published envelope. Regenerate with:

```sh
Tools/check-build.sh
```

The numbers below are printed by `parapenting_calibration_tests` and every one
of them is read off an exported time series, so it is something an instrumented
flight could also produce.

## Configuration

| | |
|---|---|
| wing | BGD EPIC 2 ML (research profile) |
| all-up weight | **105.0 kg** — the weight the published envelope is quoted at |
| air density | 1.12 kg/m³ |
| solver | `CoupledParagliderSolver`, 120 Hz, aero at 10 Hz, 3 coupling iterations |
| settle / record | 90 s / 45 s |

The weight matters and is not a detail. Trim speed goes as the square root of
wing loading; the EPIC 2 ML's figures are 105 kg figures against a 90–110 kg
certified range, and this solver's unballasted payload comes to 94.3 kg.
Comparing those two directly builds a 5.5% error into the comparison rather
than into the model. The manoeuvre runner ballasts to the published weight.

## Hands-up trim, against the published envelope

| quantity | model | published | error |
|---|---|---|---|
| trim speed | 39.4 km/h | 39.0 | **+0.4 km/h** |
| sink at trim | 1.15 m/s | 1.14 | +0.01 |
| glide at trim | 9.43 | 9.5 | −0.07 |
| incidence | 5.02° | 5.30° needed for the published trim CL of 0.580 | −0.28° |

**One parameter was fitted to produce this and three were not.** The fitted one
is the line plan's design incidence — the root chord angle at the unloaded
design pose, where the rest lengths are cut — which
`Data/Wings/bgd-epic-2-ml-lineplan.json` has always named as the quantity to
identify: *"replace with the manufacturer's rigging angle, or by fitting Level 4
trim speed."* It moved from a round 5.0° to 4.4°.

Sink, glide and incidence were not fitted to anything and all three land. That
is the test; the trim speed on its own is a restatement of the fit.

This number was 31.9 km/h for most of the project's life. What closed it was
`PHYSICS_TODO` item 10 — see `PHYSICS_LEARNINGS.md` §21.

## The manoeuvre set

| manoeuvre | speed | sink | glide | incidence | settled |
|---|---|---|---|---|---|
| hands-up trim | 10.95 m/s | 1.15 | 9.43 | 5.0° | yes |
| accelerator step (full bar) | 7.81 | 7.81 | 0.00 | 88.4° | **departed** |
| brake step 25% | 8.82 | 0.78 | 11.32 | 9.7° | still oscillating |
| deep brake step 40% | 7.80 | 7.80 | 0.02 | 94.9° | **departed** |
| brake pulse and release | 9.79 | 1.27 | 7.66 | 7.4° | still oscillating |
| weight shift step | 10.97 | 1.16 | 9.39 | 5.0° | yes |
| coordinated turn 35% | 8.96 | 0.93 | 9.59 | 8.9° | still oscillating |
| stall approach | 7.74 | 7.73 | 0.05 | 102.0° | separated, as intended |

## Pitch, against a closed form

A brake pulse released leaves a free oscillation of the wing against the pilot.
Its period is bounded above by a simple pendulum on the same lines — the line
geometry adds stiffness a bob does not have.

| | |
|---|---|
| surge period | 3.87 s |
| damping ratio | 0.06 |
| simple pendulum on 8.08 m | 5.70 s |
| canopy travel, brake to surge | 0.34 m at trim → −0.53 m under brake → 1.16 m at the peak |

The aircraft has **two** pitch modes an order of magnitude apart: the wing
swinging against the pilot at about four seconds, and a slow speed-and-incidence
mode near twenty. Given the whole record the identifier locks onto the slow one
and reports a 20.4 s "pendulum", which is a true statement about the wrong mode,
so the identification window is bounded to the fast one.

## Turning

| | brake 35% | weight shift |
|---|---|---|
| turn rate | 0.045 rad/s | 0.014 |
| bank | 1.5° | 1.5° |

Direction, mirroring and ordering are all correct and were checked against
world-frame vectors rather than against a sign convention: right brake turns the
ground track +1.217 rad toward +Y with the right tip 0.030 below the horizon,
and left brake mirrors it to four digits. Brake outranks weight shift; both
right-hand inputs turn the same way; the wing banks into its turn.

Note the test previously asserted that turn rate and bank carry **opposite**
signs, on the stated grounds that "positive bank is right tip up, which is a
left turn". That is backwards and the code says so — `bankRad` is
`asin(-span.z)`, so a right tip below the horizon reads positive. Corrected.

## Known disagreements

Recorded rather than tuned away. Each is bounded in `calibration_tests` so that
closing it registers as a failure rather than passing silently.

### 1. The wing turns several times too slowly

0.045 rad/s at 1.5° of bank on 35% brake, where a real EN-B wing does about
0.3 rad/s at 20–30°. Roughly a seventh.

Narrowed, not closed. Item 10's rewrite removed two mechanisms that were
suppressing it — the payload's `m L²` sitting in the canopy's roll inertia,
which made a 5 kg canopy 66 times harder to roll than it is, and a gravity roll
spring referenced to the world vertical, which a coordinated turn should not
have at all — and the turn rate roughly tripled.

### 2. The wing cannot hold full bar

Full accelerator diverges in pitch. This has a closed form and measured inputs.

With the payload on its own link the canopy hangs at its line angle less
`M_aero/K`. The line spring is **geometric**, not elastic — the lines stretch
0.2% while the canopy's origin moves 0.13 m, so the wing pivots about a virtual
hinge 6.6 m below itself — which makes `K` proportional to load: measured 3306,
6317, 11512 and 15393 N·m/rad at ½, 1, 2 and 3 g. So the incidence offset is
`c·Cm/(k·CL)` and depends on lift coefficient alone, and the loop gain of
*steepen the path → lose incidence → lose CL → lose more incidence* is

    gain = a·c·Cm / (k·CL²)

Swept off the wing's own VSM polar:

| α | CL | Cm | offset | gain | |
|---|---|---|---|---|---|
| 0° | 0.283 | 0.102 | 7.70° | 1.62 | diverges |
| 1° | 0.341 | 0.100 | 6.26° | 1.07 | diverges |
| 2° | 0.399 | 0.098 | 5.24° | 0.75 | |
| 5° | 0.565 | 0.089 | 3.36° | 0.32 | trim |

The gain passes one at CL 0.35. Full bar is a CL 0.31 condition — so the wing is
statically pitch-divergent at exactly its published top speed, and no amount of
damping fixes a gain above one. Half bar departs too, more slowly.

The two candidates are the analytic section pitching moment (item 1) and the
specific stiffness of 6.13 m. Both are single measurable numbers.

### 3. The polar's lift ceiling, and the envelope it costs

Swept on the VSM the analytic section polars give the wing a maximum lift
coefficient of **0.866 at 11° of incidence**, where this wing's own profile
carries **1.32**. Trim sits at 5.0°, so there is barely six degrees of brake
before the wing is past its own stall — and past it the separated branch has no
steady state to return to (limitation 6), so a transient overshoot is permanent.

40% of brake travel is an ordinary EN-B input and this model cannot hold it.

**The usable envelope is therefore hands-up to about 25% brake.** That is
narrow, it is measured, and it is the single most valuable thing real section
data would buy. This is item 1 — analytic thin-airfoil polars, blocked on XFOIL
— arriving where a pilot would feel it.

### 4. Energy during pitch transients

| manoeuvre | worst residual |
|---|---|
| hands-up trim | 1.1 W |
| weight shift step | 1.2 W |
| brake step 25% | 20.9 W |
| coordinated turn | 28.6 W |
| brake pulse | 38.8 W |

Steady flight closes to about a watt on a 1030 N aircraft. What raises the
number is a pitch transient, and the reason is stated rather than absorbed: the
pendulum between wing and pilot carries real energy, and the rigid motion still
integrates one lumped translation, so the payload's height changes in the
bookkeeping and not in the dynamics.

## What Level 9 does not answer

The plan's exit gate also asks that experienced pilots confirm direction,
control progression, pitch timing, pendulum timing and stall approach
qualitatively. That cannot be done from here. `docs/PILOT_REVIEW_PROTOCOL.md` is
the structured protocol for it, and until it has been run the handling of this
model is **unvalidated** in the registry's sense: self-consistent, compared
against four published numbers, and never flown by anyone.
