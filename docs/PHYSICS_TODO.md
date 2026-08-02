# Physics: what is left

Every open physics item in one place, with what blocks it and what "done" looks
like. The specification and per-level detail live in
`agent-data/GEOMETRY_DRIVEN_PARAGLIDER_MASTER_PLAN.md`; what is built lives in
`docs/PHYSICS_ENGINE.md`; what it cost is in `docs/PHYSICS_LEARNINGS.md`.

Status as of Level 9 substantially complete. Levels 0-8 are done, Level 9's
manoeuvre runner, still-air gates, calibration report and pilot review protocol
are in, all twelve test suites green, and nothing geometry-driven flies the wing
yet.

**Item 10 is closed and it closed item 0 with it.** The rigid motion no longer
counts gravity's restoring torque twice. Hands-up trim is 39.4 km/h against a
published 39.0, sink 1.15 against 1.14, glide 9.43 against 9.5, and incidence
5.02 degrees against the 5.30 the published trim lift coefficient needs - one
parameter identified, three numbers that were not fitted following it. The
18% shortfall that survived two rounds of narrowing was a doubled pitch
stiffness, exactly as item 10 predicted. See `docs/CALIBRATION_REPORT.md`.

**What that exposed is a narrow envelope.** The wing flies hands-up to about a
quarter brake. Full accelerator and 40% brake both leave the envelope and do
not return, for two separately measured reasons - items 1 and 11 below - and
both are bounded in `calibration_tests` rather than hidden.

## Blocked on things this environment does not have

**0. CLOSED. Trim was 18% slow and the pitch model was the suspect - it was.**
31.9 km/h against a published 39. Now 39.4 against 39.0, at the published
105 kg all-up, with sink, glide and incidence all following.

- The first diagnosis, that the analytic lift curve was too high, was wrong and
  was recorded as wrong. The curve tests out close to right.
- Two real defects were behind part of the gap and were fixed earlier: the
  section pitching moment was four times too small and was never applied to the
  wing at all.
- **The rest was item 10's doubled pitch stiffness, and closing item 10 closed
  this.** One parameter was then identified against the published trim - the
  line plan's design incidence, 5.0 degrees to 4.4, which the line plan file
  has always named as the quantity to fit - and sink, glide and incidence were
  not fitted and all three land.
- Gated in `calibration_tests` as an agreement rather than a disagreement, so
  it cannot regress unnoticed.

**10. CLOSED. The rigid motion no longer lumps the canopy and the payload.**
The payload is a link with its own direction in WORLD axes
(`payloadDirWorld`, `linkRateWorldRadps`), driven by `(g - a_pivot + harness
drag/m)` and nothing else. The canopy carries its own inertia. Gravity's
restoring torque is written exactly once.

What it took, beyond the rewrite itself, was four measurements:

- **The line spring is GEOMETRIC, not elastic, and scales with load.** Measured
  off the built graph at four loads: 3306, 6317, 11512 and 15393 Nm/rad at half
  a g, one, two and four. The lines stretch 0.2% while the canopy's origin
  moves 0.13 m, so the wing pivots about a virtual hinge 6.62 m below itself.
  Freezing the spring at its one-g value - which the first attempt did - makes
  the pitch axis diverge, because the aerodynamic moment scales with dynamic
  pressure and a constant spring loses to it.
- **The probe needs 12000 iterations.** Held at 0.02 rad it returns 19849
  Nm/rad at 120 iterations, 9228 at 2000 and 6371 at 48000. A warm-started
  in-flight solve cannot answer this question, which is why the first attempt's
  "use the live network" idea produced garbage.
- **The hinge arm sets the inertia.** Rotating the canopy drags its own mass
  and the apparent mass through a 6.62 m arc, so pitch inertia is 120 plus
  `(m + m_apparent) h^2`, not 120.
- **A simulation that starts mid-flight must start TRIMMED.** The canopy's
  equilibrium is not its hang pose - it sits about 3.3 degrees below it under
  the camber couple - and starting at the hang pose is a step input into a
  lightly damped spring that rings to twice the offset, takes incidence to 0.3
  degrees, takes the load off the lines, and the geometric spring then has
  almost nothing left to restore with. Measured: 976 N and 5727 Nm/rad at a
  tenth of a second, 207 N and 989 Nm/rad two seconds later.

Roll got the same treatment: a line roll stiffness of 8204 Nm/rad measured by
the same probe, replacing a `W L sin` term referenced to the world vertical
that a coordinated turn should never have had.

**0b. The wing turns several times too slowly.** 0.045 rad/s at 1.5 degrees of
bank on 35% brake, where an EN-B wing does about 0.3 rad/s at 20-30 degrees.
Found by Level 9's coordinated-turn manoeuvre. Was 0.015 rad/s; item 10's
rewrite roughly tripled it and this is what is left.

- Not a calibration error at that magnitude - a mechanism is missing or
  overwhelmed. Direction and ordering are right: both right-hand inputs turn
  the same way, the wing banks into its turn, and brake outranks weight shift.
- Candidates: the doubled stiffness of item 10 applies to roll as well as
  pitch; the yaw damping the VSM measures by centred probe; and the only path
  from bank to turn is the sideslip the circulation solve sees.
- **The first candidate was measured, fixed, and was most of the answer.** The
  payload's `m L^2` was in the canopy's rotational inertia, making a 5.1 kg
  canopy 66 times harder to roll than it is, and the `W L sin` term gave it a
  roll spring referenced to the WORLD vertical - which a coordinated turn
  should not have at all, since the pilot swings out under the wing and
  apparent gravity lies along the lines. Both are gone. Turn rate tripled.
- What remains, untested: the yaw damping the VSM measures by centred probe,
  and the fact that the only path from bank to turn is the sideslip the
  circulation solve sees. Direction, mirroring and bank are all correct and
  checked against world vectors.
- Bounded in `calibration_tests` so closing it registers.
- This is the largest disagreement in the model and probably the one a pilot
  would notice first.

**1. Section polars are analytic.** Thin-airfoil lift with the circular-arc
zero-lift angle, brake as a trailing-edge flap, Viterna-Corrigan post-stall.
All derived, all registered `Provisional`. Every flight number in Level 4 rests
on theory rather than measurement.

- **Level 9 measured what this costs, and it is the binding constraint on the
  flight envelope.** Swept on the VSM the analytic polars give the wing a
  maximum lift coefficient of 0.866 at 11 degrees of incidence, where this
  wing's own profile carries 1.32. Trim sits at 5.0 degrees, so there is barely
  six degrees of brake before the wing is past its own stall - and past it the
  separated branch has no steady state to return to (item 6), so a transient
  overshoot is permanent. 40% of brake travel is an ordinary EN-B input and
  this model cannot hold it. The usable envelope is hands-up to about 25%
  brake.
- The same sweep gives Cm near 0.10 across the whole range, which is the other
  half of item 11's loop gain.
- Blocked by: no XFOIL, aerosandbox or neuralfoil available here. Checked.
- Needs: XFOIL (or equivalent) runs over digitised EPIC 2 ML profiles at the
  Reynolds numbers the sections actually see, swept over incidence and brake
  deflection, tabulated into `SectionPolarTable`.
- Done when: the registry's `Provisional` polar coefficients become `Measured`,
  and Level 4's lifting-line validation still holds with real section data.
- Can run in parallel with anything. Long lead time. Start it early.

**2. Apparent-mass rotational terms are disputed.** The leading coefficients
could not be checked against the source paper and disagree with the model's own
estimate by a factor of fourteen in roll.

- Blocked by: no access to the source paper.
- Needs: the Lissaman and Brown derivation, or an independent derivation.
- Done when: the two `Disputed` registry entries resolve to `Provisional` or
  better, with the disagreement explained rather than averaged.
- Nothing currently uses their magnitude, so this is not urgent — but it will be
  the moment rotational dynamics are calibrated.

## Level 8 — emergent collapse (built)

**3. The reopening surge is now a pendulum, not a shape.** The wing and the
pilot are two bodies on a 7 m line with the line geometry's own measured pitch
spring between them, so a collapse recovery swings the wing forward the way a
brake release does - 0.77 m ahead of the pilot at trim, 1.85 m at the top of a
surge.

- What is still missing is the local part: the surge is driven by the change in
  the whole wing's force, not by the *shape* of the reopening section. A real
  frontal recovery has the nose catching air and scooping forward, which needs
  the membrane's fold geometry read back into the aerodynamics.
- Done when: a recovered collapse's pitch excursion differs from the one the
  same force change produces symmetrically, and the energy accounting still
  closes across it.

**4. A cravat has never formed in the coupled solve.** It forms in
`collapse_tests`, from the built graph's real 0.178 m tip line gap against a
fold deep enough to reach past it, and it latches and clears the way one does.
In flight the strip's fold depth stays short of that gap, so the contact test
correctly returns nothing.

- Whether that is the wing or the one-dimensional strip understating how far
  skin hangs is not known. The 2-D mesh is what would answer it.
- Not a defect as it stands: the criterion is contact, and there is no contact.

**5. The collapse debug view is blocked on the integration debt** (item 7), and
on nothing else. The pawn draws collapse from the legacy telemetry because that
is what flies. Every quantity the view would draw already exists in
`SectionCollapseDiagnostics`: margin, external Cp, fold, whether it propagated
from a neighbour, fold reach past the line, cravat.

### What Level 8 closed

- Collapse from a pressure balance across the nose - Level 5's cell pressure
  against the same rounded-nose distribution read at the fold station - eroded
  by local unloading and skin slack. No threshold on a control input anywhere.
- Cravats as a contact test between Level 6's fold depth and the real line gap
  off the built suspension graph. A cravat latches and holds its section
  folded, which is why it ends in a spiral where a collapse ends in a surge.
- Wired into the coupled solve: a fold takes its cell's pressure out on the way
  to the aerodynamics, and the section polars do the rest. No collapse-to-yaw
  term exists.
- Incident benchmarks in `coupled_tests`, driven only by air arriving at part
  of the wing. Still air and a braked turn fold nothing; 4 m/s down over the
  left half folds it to 0.70 against 0.08 on the right, turns the wing toward
  the folded half, hands the line network a 0.68 load imbalance and recovers
  fully; the same air over the whole span folds both halves to 0.712 and does
  not turn it; brake inside the sewn-in slack does nothing to a fold and brake
  past it holds one in. The numerical safety envelope engages in none of them.
- Self-collision: built, measured to be incapable of firing on a 1-D strip
  (zero segment crossings with the ribs drawn to a tenth of their spacing), and
  removed. It is the 2-D mesh that would need it, and cravats do not.
- Three defects in the levels below, found by the exit gates: crossport flow
  that depended on which end of the wing the loop started at, brake reaching
  the trailing edge through slack line, and a load reference that read every
  healthy tip as half unloaded. `PHYSICS_ENGINE.md` §Level 8 has all three.

**Past about 5 m/s of gust the wing does not come back.** It pitches into full
separation and descends vertically at 7.5 m/s. That is the deep-stall attractor
below, not this level - a collapse is what puts the wing there and not what
keeps it there.

## Deliberate limitations, not bugs

**6. Deep stall does not converge in the VSM solved cold**, and will not. The
separated branch has a negative lift slope, which inverts the downwash feedback
between sections; a wing in deep stall has no stable steady state to find.

- Locked as a known-failure check: `Check(!converged, "KNOWN FAILURE: ...")`,
  which fails loudly if someone ever fixes it.
- The honest treatment is Level 11's unsteady wake.
- Note this is about the *cold* solve. Inside the coupled solver, with Level 4's
  separation state carried between steps, the wing walks into a fully separated
  46-degree stall at 4.65 m/s of sink without the solve failing at all.

## Integration debt

**7. Nothing geometry-driven flies the wing.** `ParagliderDynamics` — one
six-degree-of-freedom body with a fitted polar — is still what the game flies.
Levels 1-7 are exercised only by their own suites and the debug views.

- This is guiding rule 11 working as intended, not an accident.
- But it means no part of the geometry-driven stack has been felt by a pilot,
  and handling feedback cannot reach it.
- Level 10 removes the legacy path. It must not start before Level 9 calibration,
  and Level 9 must not start before real section polars (item 1).

**8. Coefficient registry: 96 coefficients, 26 tuned, 82 unvalidated.** 27%
tuned. The remaining tuned coefficients are concentrated in the legacy model,
and item 7 is what retires them — with one loud exception in the geometry-driven
stack, `swingDampingRatio`, which is item 11.

**11. The pendulum damping ratio is a stated number holding up the pitch axis.**
`swingDampingRatio` is 0.35. Hands-off stability depends on it: at 0.20 - what
a wing settling in three swings implies, and what this solver used to use - the
aircraft's pitch diverges and it is fully separated inside a minute.

- What it is really doing at 0.35 is not damping, it is TRACKING. The pendulum
  has to follow apparent gravity, which in a pull-up swings round with the
  flight path, and that is what holds a paraglider's incidence steady through a
  phugoid. Measured at 0.20 the link tracked 10.7 degrees of a 14.6 degree
  flight-path change and the missing 3.9 degrees went into incidence.
- That matters here because the wing's pitch feedback has a loop gain of
  `a c Cm / (k CL^2)` - measured off its own polar and its own suspension -
  which is 0.32 at trim but passes ONE at CL 0.35. Full bar is a CL 0.31
  condition, so the wing is statically pitch-divergent at its published top
  speed and no damping fixes that.
- Estimated honestly from what physically damps the swing - the pilot's drag on
  an 8 m arm, plus the lines sweeping - the ratio is nearer 0.06.
- Tried and rejected: damping the link against the CANOPY rather than the world,
  which is where the friction physically is. It leaves the pendulum free to be
  dragged by the wing and the aircraft left the envelope in twenty seconds.
- Done when: the wing holds full bar with the ratio at a value derived from
  pilot and line drag. The two levers on the loop gain are the section pitching
  moment (item 1) and the specific stiffness of 6.13 m, and both are single
  measurable numbers.
- Registered Tuned/Unvalidated, superseded-by Level 11, and bounded by the
  full-bar gate in `calibration_tests`.

## Data gaps

**9. Grindelwald First's anchor is 50 m above its surveyed ground.** Published
2123 m is the top station; its WGS84 pair is on the launch slope below, which
swissALTI3D puts at 2073 m. Every other site agrees within 12 m.

- Needs: a better coordinate for the actual launch, from a source.
- Recorded with a named tolerance in `terrain_survey_tests` rather than fitted
  away — the terrain is the measurement, the anchor is the estimate.

## Building the module: there is no quota, and it takes 35 seconds

`Tools/check-build.sh` builds `ParapentingEditor` and runs all eleven suites.
The whole thing is under a minute on this machine. **Run it. There is nothing
stopping you.**

```sh
Tools/check-build.sh          # module AND tests, in that order
```

This section previously said the opposite — that Unreal build/cook was
"quota-blocked until roughly 2026-08-05", so engine changes were being committed
unverified on purpose. That claim came from `CURRENT_STATE_HANDOFF.md`, was
inherited and repeated across several commits, and **was simply false**. Unreal
has no build quota. When finally tested, the module built clean in 35 seconds,
including every file that had been marked unverified. Nothing was ever broken;
the constraint was.

The reason it went unchallenged for so long is worth keeping: an environmental
constraint, written down once, is invisible in a way a wrong number is not.
Nobody re-derives "we can't build" — they route around it. See
`PHYSICS_LEARNINGS.md` §17.

**What is still true and matters:** the CMake suites do not compile a single line
of engine code. They build each `Physics/*.cpp` as its own translation unit —
exactly the configuration in which a unity-build name collision is invisible,
which has let the module stay broken for hours while the tests stayed green. So
suites passing still says nothing about `Source/Parapenting/*.cpp`. That is why
`check-build.sh` builds the module first, and why it should be the gate rather
than `--tests`.

After a change to the rig, the terrain regions or route placement, a runtime
smoke test is still owed, because those changes are visual and structural and
compiling proves neither:

1. fly the Amisbühl → Lehn route — risers should read as four separate webbing
   bands per side, mains should fan from the riser tops, and the brake lines
   should end in the pilot's fists through the whole brake range;
2. weight-shift hard both ways — the pilot, the carabiners and the lines must
   move as one object;
3. `[` / `]` to a Grindelwald route — the terrain mesh must rebuild for the new
   region rather than leaving the pilot over empty space, and back again.

Nothing else in this file should start before that, because every later engine
change stacks on top of an unverified one.

## Closed recently, for orientation

- **Level 8 emergent collapse.** A pressure balance across the nose, cravats as
  a contact test, wired into the coupled solve and gated by incident
  benchmarks that only ever do one thing to the wing: put air at part of it.
  Three defects fixed in the levels below, all found by the gates rather than
  by inspection.
- **Level 7 coupled solver.** Trim on the published wing, turns emerge and
  mirror to 2e-8 rad, suite green and running with the other nine. Four defects
  fixed: explicit damping integration at 11x its stability limit, a damping
  derivative divided by the live rate, probes solving a cold unconverged wing,
  and a validity gate that bounded force but not moment.
- **Terrain/flight frame disagreement.** Fixed 25 commits before anyone noticed;
  the evidence for it being open was a rotor sample taken thirty metres
  underground.
- **Both Grindelwald routes off the map.** Own swissALTI3D region at the sites'
  true projected positions. All ten routes on surveyed ground.
