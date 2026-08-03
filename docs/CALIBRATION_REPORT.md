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
| hands-up trim | 11.06 m/s | 0.97 | 11.33 | 5.1° | yes |
| accelerator step (full bar) | 7.83 | 7.83 | 0.01 | 85.8° | **departed** |
| brake step 25% | 10.92 | 0.94 | 11.63 | 4.4° | still oscillating |
| deep brake step 40% | 7.81 | 7.81 | 0.01 | 91.2° | **departed** |
| brake pulse and release | 11.56 | 1.36 | 8.45 | 4.3° | still oscillating |
| weight shift step | 11.09 | 0.98 | 11.32 | 5.1° | yes |
| coordinated turn 35% | 10.78 | 1.08 | 9.91 | 4.2° | still oscillating |
| stall approach | 7.80 | 7.80 | 0.02 | 94.6° | separated, as intended |

The brake rows read weaker than they did before the brake double count was
removed — 25% is worth 0.14 m/s here where it was worth 2.1 — and the incidence
column runs the wrong way under brake. Both are §3d and item 11.

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

### 3. The lift ceiling — CLOSED, and what took its place

This section used to read: the analytic section polars give the wing a maximum
lift coefficient of **0.866 at 11°**, where this wing's own profile carries
1.32, so 40% brake takes the wing past its own stall and there is no steady
state to return to.

**That reason is gone.** The polars are solved on the section's own coordinates
now — panels for the potential flow, an integral boundary layer over them, and
a Kirchhoff dead-air region aft of separation iterated to a fixed point.

| | analytic | computed |
|---|---|---|
| section CLmax, hands up | 0.87 | 1.81 at 12° |
| section CLmax, 25% brake | 0.87 | 2.05 at 11° |
| section CLmax, 40% brake | 0.87 | 2.10 at 9° |
| section Cm | −0.110, constant | −0.093 to −0.102 across the range |

A stated stall margin above a moving zero-lift angle means maximum lift that
cannot change with brake. A real deflected trailing edge raises it and lowers
the angle it happens at, and both now do.

### 3b. The section was stalling at its nose

Getting there took a second fix, and it was worth more than it looked. The
solved stall angle jumped around across the brake axis — 10, 11, 7, 12, 3, 13
degrees — which read as solver jitter. It was not. **A turbulent boundary layer
separating in the first 3% of chord was being read as the section stalling**:
one degree of incidence took the flow from separating at 94% of chord to 3%,
and the whole upper surface went at once. That is leading-edge stall, and a
15.5% section with a 2.65% nose radius does not do it.

Letting a leading-edge bubble reattach — the turbulent twin of the laminar
short bubble already in the code — fixed three things that had looked
unrelated:

- maximum lift and stall angle became monotone in brake;
- the 4 m/s asymmetric gust benchmark, which had stopped recovering, folds less
  (0.653 against 0.888) and clears completely;
- the symmetric frontal's halves peak at 0.710 and 0.710, from 11% apart.

### 3c. Brake does what brake does

Settled properly at each setting — forty seconds hands up, an eight-second
ramp, forty seconds held — brake slows the wing and costs glide, and a firm
input from trim trades the speed for height first:

| | hands up | 25% brake |
|---|---|---|
| airspeed | 10.48 m/s | 9.77 |
| glide | 10.87 | 10.34 |

and a firm input climbs at **1.15 m/s** before settling slower — the same
pendulum as the release surge with the sign of the wing's acceleration
reversed. All three are gated in `coupled_tests`.

An earlier version of this report said brake was making the wing *faster* over
the first fifth of its travel, and attributed it to the section's pitching
moment. That was measured off a ramp started before the wing had finished
settling: the phugoid here has a four-second period and a damping ratio near
0.09, so what was measured was the tail of the oscillation. See
`PHYSICS_LEARNINGS` §30.

### 3d. What is still short

**Brake reaches the wing with the wrong sign, and the wing departs at 40% of
travel** where an EN-B stalls at 65 to 80%. Full bar reaches 13.4 m/s before it
diverges. Both are item 11, the pitch axis.

The reading of this changed, and the change is worth more than the numbers. It
used to be that brake rotated the canopy 3.6° between 19% and 40% of travel and
ate a 5° stall margin. That rotation was **twice what the brake line has length
for**, because the pull was counted twice: the line network shortened the brake
run by the whole 0.62 m of handle travel and rotated a rigid canopy with it,
while the section polars spent the same travel again bending the trailing edge
into camber. A brake line ends at 98% chord, so the fabric it bends and the
canopy it rotates are pulled through one length, not two.

Counted once — the take-up measured off the geometry, 2.315 m of mean chord at
the four span stations the brake fan lands on — the 0.62 m divides as **0.120 m
of sewn-in slack, 0.298 m of fabric, 0.202 m of rotation**, and full brake
rotates the canopy 5.0° where it used to rotate it 12.4°.

What that exposed is that the double count had been propping up a pitch axis
that does not settle at all.

> **Retracted.** This section previously read the 25% brake row as brake
> *lowering* incidence — 4.4° against 5.14° at trim — and concluded that the
> suspension could not produce the right sign, leaving "one unmeasured number,
> the specific stiffness of 6.13 m". Both halves were wrong.
>
> The aircraft never reaches a steady state: hands-up in still air it swings
> **0.60°** of incidence after sixty seconds, and **2.26°** under 25% brake, so
> the 0.5–1.8° differences the sign was read from were smaller than the
> oscillation containing them. The `NOT SETTLED` flag beside that row in this
> report's own manoeuvre table was saying so.
>
> And 6.13 m is not a lever: it is registered *Validated*, measured off the
> built graph at four loads, and is not an input to the solver — the stiffness
> curve is measured and 6.13 is its slope written down afterwards.
>
> The real finding, with the evidence, is `PHYSICS_TODO` item 11 and
> `parapenting_pitch_axis_trace`. The section-side measurements below still
> stand: the pitching moment agrees with thin-airfoil flap theory to 10%, and
> the brake take-up comes off the geometry.

Bounded, not fitted. `calibration_tests` gates the direction and caps the
incidence drop at 1.7°; `coupled_tests` gates the direction and the glide and
loosens only the magnitude. Both carry the strict thresholds in comments so
restoring them is a revert rather than a rediscovery.

### 3e. Where the model stands against the published wing

At the published 105 kg all-up, with the design incidence set by a design rule
— hands up, the wing sits at its own best glide — rather than fitted to any of
these numbers:

| | published | model | |
|---|---|---|---|
| trim | 39.0 km/h | 39.8 | +2% |
| trim incidence | 5.30° (from published CL 0.580) | 5.14° | −3% |
| sink at trim | 1.14 m/s | 0.97 | −15% |
| best glide | 9.5 | 11.33 | +19% |

The lift side lands and the drag side does not, and it is one error rather than
two. The solved section runs 0.0157 of profile drag at trim where paraglider
sections are quoted at 0.018 to 0.025. The analytic polars agreed with the
published glide at 9.43, and that agreement rested on a stated minimum section
drag of 0.0125 — it did not survive the drag becoming a consequence. Item 12,
bounded in `calibration_tests` in the direction the model is wrong.

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
