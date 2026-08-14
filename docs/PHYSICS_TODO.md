# Physics: what is left

Every open physics item in one place, with what blocks it and what "done" looks
like. The specification and per-level detail live in
`agent-data/GEOMETRY_DRIVEN_PARAGLIDER_MASTER_PLAN.md`; what is built lives in
`docs/PHYSICS_ENGINE.md`; what it cost is in `docs/PHYSICS_LEARNINGS.md`.

Status as of the collapse-symmetry investigation closing (§64-§68). Levels 0-8
are done, Level 9 is done bar its pilot review, Level 10's first two strands are
done and its exit gate is blocked, Level 11 (the unsteady wake) is unstarted but
now **specified by measurement** rather than by ambition, all twelve test suites
are green, and nothing geometry-driven flies the wing yet.

**The ordering has changed and the master plan carries the reasoning.** Its
first two steps have now been run, and between them they close one question and
open none:

- **Item 18 is done and it holds.** `parapenting_calibration_settled`
  reproduces hands-up trim at 11.174 m/s, sink 1.015, glide **10.96**,
  incidence 4.95°, settling at **530 s** against the fast suite's 90. The same
  run says **half the Level 9 manoeuvre set never reaches a comparable
  number**: the accelerator step departs, 25% brake does not settle after its
  input, and deep brake and stall approach both engage the safety envelope.
  Those four are not disagreements with the manufacturer — they are not
  measurements.
- **The harness-side drag route is closed (§69).** All three candidate
  locations for the missing drag — lines, section, harness — break the
  symmetric frontal, and the two new ones engage the safety envelope, so the
  benchmark cannot adjudicate between them. **Item 12 has no route around
  Level 11.**

**The ordering changed again at §80, and this time item 19 goes first.** Item
11's pitch half has run out of candidate mechanisms — the last one was
eliminated in both directions — and its exit criterion has been rewritten
because the old one was unreachable rather than merely hard (see item 11). Item
12's drag half is blocked behind the wake. So the pair that used to lead this
list is one part re-scoped and one part blocked, and neither is advanced by a
fifth instrument.

**A REFERENCE IN THIS PARAGRAPH WAS WRONG AND IS CORRECTED.** It used to read
"get the stack flying across a stated envelope … (item 19)". Item 19 is not
that — it is *the legacy pitch axis has no gravity-referenced pendulum*. The
stated-envelope work is item 17's, and it is blocked. The two were conflated
because both are pilot-facing, and the mistake sent a session at the wrong item.

**THE ORDERING CHANGED AGAIN AT ITEM 30, AND WHAT MOVED TO THE FRONT IS A
DEFECT RATHER THAN A LEVEL.** Asking whether strand 2's lagged circulation
should ship measured three things and found two problems that are older and
wider than the flag:

- **The aerodynamic states have been advancing at one sixth of real time.**
  `SolveUnsteady` runs once per `aerodynamicsInterval` and was handed the
  simulation step, so the separation state — stall's memory — and strand 2's
  Wagner lag both run six times slow at the shipped schedule. Corrected behind
  a flag; **the correction reddens seven collapse gates, including strand 2's
  own**, which is why it defaults off.
- **And the implemented lag is not Wagner's** even with that corrected: 12% of
  a circulation step closed where the published Φ(0) is 0.5. Strand 1's
  verification of the component stands; nothing ever re-checked the composite.
  **Now located: the response is Wagner's and the TARGET is not.** One Jacobi
  pass across sections closes 0.233 of a step where the iterated solve closes
  1.000, and 0.508 × 0.233 = 0.118 against the measured 0.119. The
  cross-section coupling strand 2 dropped as "the weak one" carries three
  quarters of the answer.
- **The lag is not item 19's missing phugoid damping.** σ unchanged, period
  down 40%, and the pendulum's ζ nearly tripled — which is the ring item 19
  spent several rounds declining to spend.

So the front of the queue is now **item 30**, and two of its three parts are
done. The second lag is identified — in the target, not the response — and the
obvious repair has been measured and **ruled out**: iterating the target
converges at trim for the price the shipped solve already pays, and past the
stall it lands whole multiples of the step away with the answer depending on
the budget. What is left is genuinely the hard part, and it is now a narrow
question rather than an open one: **Wagner describes approach to a steady value,
and in the separated regime this wing has none a bounded solve can find.** The
three remaining routes are written out under item 30. **The collapse gates
should be re-derived first** — the frontal is a separated-flow event, so it is
the benchmark that decides between those routes, and it is currently
characterised against aerodynamic states running six times slow.

After that, and unchanged: **item 19 and item 24 together** — the legacy pitch
axis, which is what a pilot actually flies and where the only two handling
reports this project has ever received both land; then item 12's shielding
number on its own evidence; then Level 11, which unblocks the rest of item 12.
Item 17 (retire the legacy path) stays blocked behind the geometry stack's two
departures and its dead weight-shift control, and is not a route around either.

**Item 10 is closed and it closed item 0 with it.** The rigid motion no longer
counts gravity's restoring torque twice. Hands-up trim is 39.4 km/h against a
published 39.0, sink 1.15 against 1.14, glide 9.43 against 9.5, and incidence
5.02 degrees against the 5.30 the published trim lift coefficient needs - one
parameter identified, three numbers that were not fitted following it. The
18% shortfall that survived two rounds of narrowing was a doubled pitch
stiffness, exactly as item 10 predicted. See `docs/CALIBRATION_REPORT.md`.

**The envelope is still hands-up to about a quarter brake, but the reason has
changed and that is the useful part.** It used to be two reasons, items 1 and
11. Item 1 is closed - the section polars are solved on the section's own
coordinates now, maximum lift rises with brake as a real flap makes it, and the
lift ceiling that stopped 40% brake is gone. Both limits are now item 11, the
pitch axis, and both are bounded in `calibration_tests` rather than hidden.

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

**1. CLOSED. The section polars are solved on the section, not stated.**
Thin-airfoil theory has no nose radius, so it cannot say where the flow lets
go, so the stall angle had to be a constant in a struct (`stallMarginRad`) -
and a constant stall margin above a moving zero-lift angle means **maximum lift
that cannot change with brake at all**. That was the binding constraint on the
flight envelope, and it is gone.

What replaced it, in `SectionProfile` and `SectionViscousSolver`:

- the section as a closed contour generated from the design parameters the
  ribs are cut to, with a real nose radius, panelled with cosine spacing;
- brake as a **bend in the camber line** distributed over 14% of chord ahead of
  the attachment, not a hinged flat plate - so there is no flap-effectiveness
  term left anywhere, the panel solver reads the bent shape;
- Hess-Smith panels, one constant source per panel plus a shared vortex, with
  the Kutta condition. Exact potential flow on that contour;
- Thwaites laminar from the stagnation point, Michel transition, a laminar
  separation bubble treated as immediate transition, Head's entrainment method
  with Ludwieg-Tillmann friction for the turbulent run, Squire-Young at the
  trailing edge;
- a **Kirchhoff dead-air region** aft of separation, iterated to a fixed point:
  the wake unloads the section, the unloaded section has a weaker suction peak,
  the separation point moves back until the two agree. Maximum lift is where
  that stops having a solution. Nothing states it.

The whole thing is one `Computed` table, built once per section and shared. It
costs about a second to build and nothing to sample.

**Validated against sections somebody has measured**, in `aerodynamics_tests`.
NACA 2412 from its own coordinates: zero-lift angle **-2.12 degrees against a
published -2.1**, lift slope 0.121 per degree against 0.11, quarter-chord
moment **-0.055 against -0.05**, minimum drag **0.0062 against 0.006**, and
maximum lift 1.96 at 16 degrees against a published 1.6-1.7 at 16. The angle
lands; the value is 18% high, and that is the method's known direction of
error - with no inverse-mode boundary layer the branch runs until it ends
rather than being solved through the separation.

**What it bought this wing.** The section carries **1.59 hands up and 2.40 at
40% brake** where the analytic table gave 0.87 at every brake setting. Swept on
the VSM the wing's own lift coefficient now rises monotonically to 1.20 at 40%
brake, where the analytic polars peaked at 0.82 and then fell off a cliff. The
pitching moment is no longer a constant - it runs -0.093 at zero incidence to
-0.102 at the stall - so the wing finally has an aerodynamic centre that moves.

**And the open nose is in the model.** A paraglider section has a hole in it.
The contour is still panelled closed - potential flow does not care about a
hole at the stagnation point - but the surface the flow reaches by crossing the
mouth has no laminar run, and that is roughly half this section's profile drag.

**What it did NOT buy: 40% brake still departs, and now for a measured
reason.** See item 11. The lift ceiling is closed; the pitch axis is what is
left.

**Still not measurement.** The registry entries stay `Provisional`: the profile
is assumed from the family rather than digitised, and camber is the single most
influential number in the stack. XFOIL over a digitised EPIC 2 profile would
make these `Measured` and is still worth doing - but it is no longer what
blocks the envelope.

**12. The section drag is optimistic, and this is still the largest
disagreement in the model — but a fifth smaller than it was written up as.**

Settled to a criterion (item 18), glide at trim is **10.96 against a published
9.5** and sink is **1.015 against 1.14**. On the fast suite's 90-second clock
those read 11.33 and 0.97, so part of what was being called drag was an
unsettled measurement. Re-check any conclusion in this item that was drawn from
the older pair.

It is still one error rather than two, and the lift side still carries it: trim
speed is 40.2 km/h against a published 39.0 and incidence 4.95 degrees against
the 5.30 the published lift coefficient needs. Note both of those got slightly
WORSE with proper settling (from +2.1% and 0.16 degrees to +3.1% and 0.35), so
"trim speed and incidence both land" is a weaker statement than it was. The
solved section runs about **0.0157 at trim** where paraglider sections are
usually quoted at 0.018 to 0.025.

The analytic polars agreed with the published glide, at 9.43. That agreement
rested on `minimumDragCoefficient = 0.0125`, a stated number, and it did not
survive the drag becoming a consequence.

- **AND IT CANNOT BE SETTLED HERE, MEASURED (§69).** Three wings at the same
  published glide, flown into the same symmetric frontal: the section
  correction splits the halves 0.958/0.703, the harness correction saturates
  both at a full 1.000 with three times the rotation, and **both engage the
  numerical safety envelope** where the shipped wing does not. By guiding rule
  12 neither row is flight behaviour, so the collapse benchmark cannot say
  which correction is right. **This item is blocked behind Level 11 whichever
  location turns out to be correct** - the blocker was never a property of line
  drag, it is the partly separated regime (§68), and a wing that glides 9.5
  instead of 10.96 arrives there harder whatever slowed it down.
- **THE DEFICIT MAY NOT BE ON THE SECTION AT ALL (§56).** This item has assumed
  for four levels that the missing drag is a section term. Asked to land the
  published glide, a section offset gives 9.49 glide but takes the trim speed
  out with it — 9.93 m/s against 10.83 — while the same deficit applied at the
  **harness** lands **9.51 glide, 10.64 m/s and 1.113 m/s sink against a
  published 9.50, 10.83 and 1.140: all three at once.** The installed-drag
  inputs are stated dials of exactly the kind that hide a deficit —
  `harnessAreaM2` 0.32 and `lineProjectedFraction` 0.35, neither solved — and
  installed drag is already 47% of canopy drag, so it is not a correction term
  where an error would be small.
  - Not proof, and the temptation to read it as proof is the reason this bullet
    says so: the 0.199 m² of Cd·A needed is a 59% increase on the harness, which
    is a lot to attribute to an underestimate, and a **combination** of section
    and installed deficits is more likely than either alone. What is
    established is that a harness-side deficit is consistent with three
    published numbers and a section-side one is not.
  - Worth doing before more section work: solve the harness and line drag
    properly — a seated pilot's frontal area and drag coefficient, and the
    line-projected fraction — rather than sweeping the section for a term that
    may not be missing there.
- The largest single candidate ON THE SECTION is the momentum thickness the
  shear layer off the cell mouth carries onto the upper surface. It is certainly not zero.
  **It has been tried twice and left out both times.** Seeded at the lip with
  `theta = h/6` - the momentum thickness of a linear profile across an opening
  of height h - quadrupling h changes the section's drag by 2%, because the lip
  sits in the steepest favourable gradient on the section and theta decays
  there as `Ue^-(H+2)`. Seeded instead at reattachment, six opening heights
  downstream, the same derivation gives 0.036 at a 1% opening and 0.076 at a 3%
  one, against the 0.019 wanted - three to four times too much, because `h/6`
  measures the layer against the freestream while it forms where the local edge
  velocity is a fraction of it. So the derivation has a factor in it that is
  not geometry, the answer moves by five times across the plausible range of
  the opening height, and one value in that range lands on the published glide.
  That is a dial, and it stays out.
- Second: the skin between ribs is scalloped and seamed and the model's is
  smooth. Needs the 2-D mesh.
- Third, and checked off rather than left hanging: the VSM's induced drag is
  **not** the cause. It reports a span efficiency of 1.31 against the wing's
  projected span, which looks impossible until you remember the wake is not
  planar - an arched wing's trailing sheet beats the flat-wake limit for its
  projected span the way a winglet does. Measured against the same wing with
  the arc removed the solver gives 1.085, which IS above the planar limit and
  is the discretisation error the elliptical-wing validation already reports at
  3.6%. Worth 8% of the induced drag, not the 100% of profile drag needed.
- **THE LINE-DRAG DIAL IS 99% SHIELDING, AND THAT IS NOW ISOLATED (§62).**
  `lineProjectedFraction` is 0.35 and its comment attributes it to three
  effects: overlap, inclination, and shielding. Measured off the built
  suspension graph — the geometry is already there — **inclination is worth
  0.993, not a large share.** The lines hang canopy-to-pilot and fan out
  spanwise, both perpendicular to a horizontal wind, so only the A-to-C spread
  tilts any of them. The graph agrees with the spec on length (254.7 m against
  254.0) and puts the mean diameter 12% lower (0.00094 against 0.00105).
  - Consequence: the shipped model carries **39% of the line drag area its own
    geometry presents** — projected 0.093 m² against a geometric 0.237 — and
    the whole reduction rests on an interference allowance never justified in
    writing. A factor of three from shielding is aggressive against published
    practice.
  - **This is the "coefficient chosen to put it there" this item objects to,
    and it is now a single flow-physics question** rather than a lumped number:
    how much does a line cascade shield itself? Bounded and printed by
    `suspension_tests`.
  - **Wired in.** The solver measures the projected line area off its own graph
    at construction; length, diameter and inclination are consequences now, not
    statements. `lineShieldingFactor` is the only stated number left in line
    drag, carried over at 0.394 so that nothing about the flight changed —
    every headline number identical at printed precision, with a 0.14% residual
    in the area recorded rather than tuned away. **Justifying or replacing that
    one number is what is left of this part of item 12**, and it is now a
    question that can be answered from literature instead of from the glide it
    has to produce.
  - **THE SHIELDING NUMBER IS NOW MEASURED WRONG, AND THE FIX IS BLOCKED BY A
    SYMMETRY FAILURE (§64).** Line-to-line spacing at the design pose: closest
    pair **158 diameters**, median 534. Wake interference is a near-field
    effect and dies out by twenty or thirty diameters, so **no wake argument
    reaches the asserted 60% shielding.** Removing it moves every published
    number toward its reference — at shielding 1.0, glide 11.33 → **9.91**
    against 9.5, sink 0.97 → **1.100** against 1.14, incidence 5.14° → 5.35°
    against the 5.30 the published CL needs. **Glide error 19% → 4.3% by
    deleting a coefficient rather than fitting one**, which is this item's
    stated criterion.
  - **Blocked, and not by evidence against it.** At 1.0 the 4 m/s symmetric
    frontal folds L 0.779 R 0.903 where it folded 0.710/0.710, failing the gate
    *"symmetrically — the two halves are the same wing"*. That is a
    solver-integrity failure rather than a calibration disagreement, so
    "toward published" does not license it. It drives the collapse deeper into
    the regime the gate already documents as having no steady state to find —
    **the fix is Level 11**, as that gate's own text says.
  - **That explanation is now tested rather than asserted (§65).** The
    suspension graph is mirror-symmetric to **6.2e-15 m** — a relative 1e-15 on
    an eight-metre structure, with every off-centre node having a partner. The
    graph is eliminated as a seed, so the 0.124 fold difference is round-off
    amplified by order **1e14** across the event. **Level 11 is a stability
    problem, not a bug hunt**, which is worth knowing before a level is spent
    looking for a broken mirror. Untested downstream: the VSM's section
    ordering, the collapse solver and the pressure model — a sequentially
    updated loop reading partially updated neighbours would break symmetry
    systematically rather than by round-off.
  - **The VSM is eliminated too (§66):** worst relative left-right circulation
    difference **9.3e-15** on a symmetric wing at symmetric incidence, bounded
    in `aerodynamics_tests`. Two of the three downstream candidates are now
    ruled out individually, so a symmetric frontal *enters* the collapse regime
    symmetric to round-off and leaves it 0.124 apart. **The collapse solver and
    the pressure model are what remain**, and both take the same cheap probe.
  - **The list is empty, and the explanation has changed shape (§67).** The
    collapse solver was already gated at **0.00e+00** in `collapse_tests`; the
    pressure model measures **0.000e+00** in pressure and in fill through an
    inflation transient, now gated in `pressure_tests` — which is also the
    regression gate the Level 8 Gauss-Seidel fix never had. *All* candidates are
    closed.
  - **But "amplification of order 1e14" is the wrong picture, measured.** The
    symmetric frontal enters the fold at 2.33e-15 and peaks at 9.55e-01 through
    **two single-step discontinuities**, not a growth: at **t=1.400 s** the
    pressure *margin* field breaks 1.95e-14 → 4.91e-01 in one 8.3 ms step while
    every margin is still positive and the wing stays folded to 1e-15; at
    **t=1.500 s**, 100 ms later, that field crosses zero on one side and the
    fold breaks 2.29e-15 → 3.94e-02 in one step. The amplifier is a **sign test
    on a field that had already lost its symmetry silently**. So Level 11 is
    *not* "make the symmetric solution stable" — that mechanism is not present.
    It is **branch selection in the separated solve** (item 6's no-steady-state
    regime). Gated in `coupled_tests`: the margin must break no later than the
    fold, or the collapse solver would be making the asymmetry rather than
    revealing it. **Which upstream step produces the O(1) jump is the open
    question** — one 8.3 ms step, at a known time, on a reproducible run.
  - **Answered, and Level 11 cannot be bought with compute (§68).** The jump is
    one **aerodynamic tick**: the margin field is piecewise constant between
    ticks, and both its sides break on the same one — incidence 1.6e-14 →
    0.379, pressure 3.3e-15 → 0.112 — as the **VSM residual jumps four orders,
    1.14e-06 → 2.22e-02**, after falling monotonically for a second and a half.
    §67's 100 ms latency is exactly one aerodynamic interval (12 steps at
    120 Hz), not a drift.
  - **And iterations are not the lever, tested rather than argued.** Sweeping
    the flight solve's cap via the new `SetFlightSolveIterationCap` instrument:
    40 (shipped), 200 and 600 all break **on the same tick**, with the residual
    at the break no better and not even monotone (2.22e-02 / 6.95e-02 /
    2.05e-02) — while the residual *before* the break does improve (1.14e-06 →
    8.03e-07), so the extra work is real. **The separated solve is not short of
    iterations; it has nothing to converge to** — item 6's no-steady-state
    branch, now demonstrated on the aircraft. Level 11 is an **unsteady wake
    formulation whose separated solve is single-valued**, not a stabilised or
    better-iterated steady one. Gated in `coupled_tests`.
  - Still open, and much narrower: *why* single-valuedness is lost at that
    particular tick and not at any of the fourteen before it.
  - **Naming collision, now fixed (§68):** the "Level 11" these gates name is
    the **unsteady wake**, which is unstarted. The section below that used to
    be headed "What Level 11 closed" is the **pitch axis by linearisation**,
    which is done, and is now headed "What the pitch-axis programme closed" —
    following the gate to it no longer reads as "the fix already shipped".
    Level numbers themselves are unchanged.
  - Not tuned around: a shielding value picked as the largest that keeps the
    symmetry gate green would be the coefficient-chosen-to-put-it-there this
    item objects to. 0.394 stays, labelled measured-wrong.
  - **Lead, now supported rather than speculative:** §56 needed 0.199 m² of
    Cd·A at the harness end to land glide, speed and sink together. A defensible
    shielding allowance of 0.9 would restore (0.993−0.35)/0.993 × 0.237 × 1.05 ≈
    **0.16 m²**, at a moment arm half way down the lines. Same order, same end
    of the aircraft, from geometry rather than from the glide it has to produce
    — which is precisely why the shielding number must be settled on its own
    evidence before this is believed.

- **Glide alone is not a sufficient criterion, and §55 measured why.** Asked to
  land the published glide, `pitch_eigenmodes --drag` bisects a flat section
  offset to Δcd 0.01035 and gets glide 9.49 against 9.50 — while the trim speed
  leaves, 9.93 m/s against a published 10.83 where the clean wing's 10.60 was
  nearly right, and sink stays low at 1.040 against 1.140. A uniform offset
  therefore satisfies the "done when" below while making two other published
  numbers worse, so **the missing term has to land glide, speed and sink at
  once and a flat Δcd provably cannot.** That is a constraint on its shape,
  obtained without deciding what it is.
- **It is not free elsewhere, either.** The same pass found that restoring the
  drag costs pitch stability rather than buying it (§55, item 11): σ at ratio
  0.35 falls 21% in magnitude and 0.30 goes from unsettled to departing.
  Whoever closes this item should expect the pitch axis to get worse, and
  should not read that as a regression in their own change.
- Done when: glide lands inside the published figure without a coefficient
  chosen to put it there, and the trim speed and sink stay landed with it.
- Bounded in `calibration_tests` in the direction the model is wrong.

**13. MOSTLY CLOSED. The section was stalling at its nose.**

The solved stall angle used to jump around across the brake axis - 10, 11, 7,
12, 3 and 13 degrees at 0, 10, 25, 40, 60 and 100% brake - and it was not
cosmetic. It cost the wing its brake range and its collapse recovery.

The cause: **a turbulent boundary layer separating in the first few percent of
chord was being read as the section stalling.** Measured, the layer went from
separating at 94% of chord to separating at 3% of it for one degree more
incidence, and the wing lost its whole upper surface in a single step. That is
leading-edge stall, and it is not what a 15.5% section with a 2.65% nose radius
does - leading-edge stall belongs to thin sections with sharp noses. It was the
integral method being asked a question it cannot answer: just aft of the
suction peak the layer is a few thousandths of a chord thick, the gradient is
at its steepest, and Head's entrainment equation has no bubble in it.

The fix is the turbulent twin of the laminar short bubble the code already had:
a separation forward of 3% of chord reattaches, and the march continues. What
it bought, all measured:

- maximum lift and stall angle became **monotone in brake** - 1.81 at 12
  degrees hands up, 2.05 at 11, 2.10 at 9, 2.35 at 9 as brake goes on - which
  is what a flap does;
- the 4 m/s asymmetric gust benchmark, which had stopped recovering, folds less
  (0.653 against 0.888) and **clears completely** again;
- the symmetric frontal's two halves now peak at 0.710 and 0.710, identical to
  three decimals, where they had drifted 11% apart.

The 3% limit is stated rather than solved, and it is the one number in
`SectionViscousSolver` that is. Short bubbles run half a percent to two percent
of chord and long ones reach five to ten. **What replaces it is the inverse
boundary-layer formulation, which is the difference between this and XFOIL.**
Some jitter remains at 40 and 100% brake and is printed in
`aerodynamics_tests`.

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

**24. THE SURGE DOES NOT OSCILLATE, AND A PILOT REPORTED IT BEFORE ANY
INSTRUMENT DID.** Reported from flying the build: after a stall the recovery is
slow, it is a *single* forward movement, and it is the same single movement
however hard the stall was. A real recovery is a decaying pendulum — big surge,
pitch back, smaller surge, pitch back — over several cycles. That sequence is
absent.

**This is the first handling defect on this axis that came from a pilot rather
than from a sweep, and it arrived one session after §80 concluded that flight
data was the likelier source of item 11's missing mechanism.** It is evidence
about `swingDampingRatio` of a kind four instrument runs could not produce.

**THE FIRST DIAGNOSIS OF THIS ITEM WAS WRITTEN AGAINST THE WRONG MODEL, AND THE
CORRECTION IS THE MOST USEFUL THING IN IT.** The three dissipators listed below
are in `CoupledParagliderSolver`. **The pawn does not fly the coupled solver** —
`ParagliderPawn` holds a `ParagliderDynamics`, exactly as item 7 says in
writing, and item 7 is two screens above this one. A report from *flying* is a
report about the legacy lumped model, and the first pass spent its effort on a
solver the pilot has never felt. Guiding rule 11 is not a footnote: while item 7
is open, "what the pilot saw" and "what the physics stack does" are two
different aircraft, and every handling report has to be routed to the model that
produced it before a line of it is believed.

**A SECOND DIAGNOSIS WAS WRITTEN HERE FROM ARITHMETIC OFF THE SHIPPED CONSTANTS,
AND MEASURING IT REFUTED MOST OF IT.** It is kept in outline because the way it
failed is the reusable part. It claimed the pitch axis was nearly critically
damped (ζ ≈ 0.69 from `pitchStiffness` 165, `pitchDamping` 220, inertia 120+34),
that `canopyPitchDampingRatio` 0.55 left 1.6% per cycle, and that the
`recoverySurge` clamp at [−0.2, 0.45] rad plus `pitchRateLimit` produced the
amplitude-independence. Measured in `physics_tests`, full brake to a stall then
hands off:

- **THE RECOVERY RINGS. It does not creep.** Horizontal speed, which is what
  "surge" means to a pilot, oscillates peak-to-peak **5.25, 6.35, 4.61, 3.25
  m/s** at roughly a 4–5 s period, decaying over about 20 s. The
  canopy-relative angle rings too: 48°, −23, +15, −18, +18, −12, +6. **A
  second-order damping ratio computed from three constants predicted 0.25% per
  cycle and the model delivers a visible decaying sequence.** The constants do
  not compose the way that arithmetic assumed — the incidence loop feeds the
  axis and the ratio is not the closed-loop one.
- **NEITHER CLAMP EVER FIRES.** `recoverySurge` pinned for **0 steps** and
  `pitchRateLimit` for **0 steps** through the whole recovery. The
  amplitude-independence was attributed to two limiters that are not reached.
- **WHAT DOES SATURATE IS THE STALL ITSELF, AND THAT IS THE ANSWER.**
  `deepStallTarget = state.deepBrakeTime > 0.9 ? 1.0 : 0.0` — a **binary**
  target, relaxed toward at rate 1.7, so `deepStall` reaches 1.0 in one to two
  seconds and then cannot go further. Measured, holding full brake for 2, 3 and
  5 seconds gives stalled sink **5.44, 5.41, 5.37 m/s** — the same stall — and
  therefore the same recovery:

| full brake held | stalled sink | surge peak-to-peak, m/s |
|---|---|---|
| 1 s | 3.86 | 8.23, 3.62 |
| 2 s | 5.44 | 11.44, 5.09, 6.18, 4.40, 3.04 |
| 3 s | 5.41 | 12.04, 5.25, 6.35, 4.61, 3.25 |
| 5 s | 5.37 | 5.37, 6.46, 4.75, 3.38, 2.21 |

- **"No matter how hard the stall it just recovers in one movement" is the STALL
  being amplitude-independent, not the recovery.** Past about two seconds of
  deep brake every stall in this model is the identical stall, so every recovery
  is the identical recovery. That is a different defect from the one this item
  was opened on, it is one line rather than six constants, and no amount of
  retuning dampers would have touched it.
- **The fix is that stall severity has to be carried by a state that can hold
  it.** A binary target plus a first-order relaxation cannot represent "how far
  past the stall" — depth, duration and energy all collapse onto one saturating
  scalar. Done when a harder or longer stall produces a measurably bigger first
  surge.

**FIXED, AND THE AXIS IT WAS FIXED ON IS NOT THE ONE THIS ITEM NAMED.** The
diagnosis above is written entirely about DURATION — the 2/3/5 s table — and
duration turns out not to be the defect. Two corrections, both from measuring:

- **THE DURATION TABLE ABOVE IS A MEASUREMENT ARTEFACT.** It reads the surge as
  a peak-to-peak between successive extrema, which has no leading extremum to
  difference against when the stall is released at 0.1 m/s: the recovery rises
  smoothly from the release speed to its first peak, and the pairwise measure
  skips that rise entirely and reports the *second* swing. Scored as the
  excursion from the release speed — which is what a pilot feels — the shipped
  model's duration axis was **already monotone**: 8.15, 11.03, 12.02, 12.38,
  12.39 m/s at holds of 1, 2, 3, 5, 8 s. The 12.05 → 5.39 "inversion" was the
  metric, not the model. It flattens past 3 s because `vmin` is already 0.10
  m/s — the wing has no speed left to lose — and that is a floor rather than a
  clamp.
- **DEPTH WAS THE DEFECT, AND NOTHING ABOVE MEASURED IT.** `symmetricBrake >
  0.86` made the entire top eighth of the brake range one case. Shipped, a 3 s
  hold at 0.88 / 0.92 / 0.96 / 1.00 gave stalled sinks 5.58 / 5.52 / 5.50 /
  5.43 and first surges 11.80 / 11.86 / 12.00 / 12.02 — a **0.22 m/s spread,
  1.9%**, across a control range the pilot moves by hand. That is the "same
  stall however hard" the report describes, and it is a step function, not a
  saturating relaxation.

The change is in `ParagliderDynamics::Step`: `brakeStallDepth` is continuous in
brake past 0.78, it scales both the `deepBrakeTime` accrual and the `deepStall`
target, and full brake is unchanged at both ends (depth 1.0, target 1.0,
accrual 1.0/s) so every shipped full-brake number stays where it was. Measured
after:

| brake, 3 s hold | 0.88 | 0.92 | 0.96 | 1.00 |
|---|---|---|---|---|
| stalled sink, m/s | 7.26 | 5.98 | 5.65 | 5.43 |
| first surge, m/s | 9.88 | 11.31 | 11.87 | 11.98 |
| ringing cycles | 7 | 7 | 4 | 6 |

Spread 0.22 → **2.10 m/s**, monotone, and the duration axis stays monotone at
11.00 / 11.98 / 12.34 / 12.36. Ringing is not traded away for it.

**A SECOND TERM WAS BUILT, MEASURED, AND REMOVED, and it is the more useful
half of the session.** `canopyPitchTarget` follows `previousHangTiltRad`, which
is an ACCELERATION, so it decays to zero once the stalled descent steadies and
the canopy drifts forward while the wing is still fully stalled — the release
angle falls from 0.843 rad at a 3 s hold to 0.695 at 5 s. That looks exactly
like the defect, and a severity-integrating state driving an aft bias holds the
canopy back through the stall precisely as designed. **It bought nothing**: the
duration axis it was built to fix was already monotone under the honest metric,
and holding the wing aft into the recovery turned a dive-and-ring into one long
dive, dropping ringing from 6 cycles to 2. It was reverted, and with it the
`stallSeverity` state, which then had no consumer. The lesson is the one this
item already learned twice: **the metric was wrong before the model was**, and
a term that improves a number by degrading the criterion next to it is not a
fix. `deepStall` now carries severity itself; a second state was not needed.

**THE THIRD CLAMP, FOUND ON THE WAY.** This item records that `recoverySurge`
and `pitchRateLimit` pin for 0 steps. `canopyRelativePitchRad` pins at its
±0.85 bound for **over a second of every deep stall** — from 1.5 s into a full
brake hold until release. It is not currently causing the reported symptom, but
it is the one clamp on this axis that a pilot actually reaches, and the "no
clamp ever fires" line above should not be read as covering it.
- ~~**STILL OPEN, and it needs the pilot rather than the suite:** the model
  rings four to five times over twenty seconds and the report says one
  movement. Either the oscillation is not reaching the screen, or the in-game
  brake range cannot reach the stalls measured here.~~ **ANSWERED, AND BY
  NEITHER BRANCH.**

**THE BRAKE IS QUANTISED AND ONLY THE TOP RUNG STALLS.**
`AParagliderPawn::ControlStep` is **0.2**, and every brake action binds
`IE_Pressed` — one discrete step per key press, no analog hold. The keyboard
brake is therefore a six-rung ladder, {0.0, 0.2, 0.4, 0.6, 0.8, 1.0}, and
nothing between the rungs is reachable. Measured at exactly those rungs, 3 s
holds:

| rung | 0.0 | 0.2 | 0.4 | 0.6 | 0.8 | 1.0 |
|---|---|---|---|---|---|---|
| `deepStall` | 0 | 0 | 0 | 0 | **0** | 0.957 |
| stalled sink, m/s | 1.14 | 1.11 | 1.62 | 4.02 | 7.52 | 5.43 |
| first surge, m/s | 0.03 | 1.12 | 3.12 | 4.95 | 8.01 | 11.98 |

**`deepStall` is nonzero at exactly one rung.** 0.8 is not a stall at all — it
is a deep-braked mush at 7.52 m/s of sink. So a keyboard pilot has **one stall
in the game**, and it is full brake.

*"No matter how hard the stall it just recovers in one movement"* is therefore
literally true, and it was never a statement about the physics. Every stall the
pilot has ever flown is the same input: brake 1.0, `deepStall` 0.957, first
surge 11.98 m/s. They all feel identical because they **are** identical.
Amplitude-independence was in the INPUT.

- The old branch B — *"the brake range cannot reach the stalls"* — is false as
  stated. The range reaches the stall. It contains exactly one of them.
- The old branch A — *"the oscillation is not reaching the screen"* — was then
  checked on its own merits, and the render path is **intact**.
  `canopyRelativePitchRad` reaches `CanopyVisual` twice over: as a
  `SetRelativeRotation`, and as a translation onto the arc that
  `EvaluateCanopySwingOffset` computes in the physics layer. The camera is a
  chase cam 8.5 m behind and 2.6 m above the root with the canopy 7.3 m up and
  in frame, and its position/rotation smoothing (`VInterpTo` 2.8,
  `RInterpTo` 3.6, time constants ~0.36 s and ~0.28 s) is an order of
  magnitude faster than the 4 s pitch mode, so it tracks the swing rather than
  filtering it. A 44.5-degree first swing is on screen. **Caveat: this is the
  data path verified by reading, not by flying and looking** — but nothing in
  it now needs to carry the explanation.
- **The gamepad path is already continuous.** `SetControllerLeftBrake` /
  `SetControllerRightBrake` bind `BindAxis` and clamp an analog value to
  [0, 1], and `AppliedControls` takes the max of keyboard and controller. The
  depth axis fixed above is fully available on a stick and **invisible on a
  keyboard**, where it spans two rungs, one of which does not stall.

**FIXED: THE BRAKE IS NOW A NON-UNIFORM LADDER.** `ControlStep` used to serve
the brakes as well as weight shift and accelerator; the brakes now have their
own `BrakeLevels` table in `ParagliderPawn.h` and the other two keep
`ControlStep` unchanged. The spacing follows where the wing changes rather than
being uniform — nothing happens between 0 and 0.30 (sink 1.14 → 1.13) so those
rungs stay cheap, and the whole stall lives between 0.84 and 1.00 so that is
where it tightens:

| rung | 0.00 | 0.20 | 0.40 | 0.58 | 0.72 | 0.84 | 0.90 | 0.95 | 1.00 |
|---|---|---|---|---|---|---|---|---|---|
| `deepStall` | 0 | 0 | 0 | 0 | 0 | 0 | 0.614 | 0.822 | 0.957 |
| first surge, m/s | 0.03 | 1.12 | 3.12 | ~4.5 | ~7.0 | 8.45 | 10.85 | 11.83 | 11.98 |

**Three distinguishable stalls where there was one, and monotone.** The cost is
eight presses to full rather than five, which is why the bottom stays coarse:
there is no separate flare key, so a landing flare is repeated presses of the
same control, and ordinary flying plus the start of a flare are still two
rungs. Only the stall band is expensive to reach, which is the right way round
for a control that ends in a stall.

The stepping is verified standalone (8 presses up, 8 down, saturation a fixed
point, continuous controller values landing on real rungs and always moving in
the commanded direction), and `ParagliderPawn.cpp` compiles and links under
UE 5.8 — **the game module builds in this environment in about 50 s**, which
earlier sessions had assumed it did not.

**What a finer UNIFORM ladder would have bought, kept because it is what set
the spacing above:**

| brake | 0.80 | 0.85 | 0.90 | 0.95 | 1.00 |
|---|---|---|---|---|---|
| `deepStall` | 0 | 0 | 0.614 | 0.822 | 0.957 |
| first surge, m/s | 8.01 | 8.57 | 10.85 | 11.83 | 11.98 |

`ControlStep` 0.1 puts two distinguishable stalls in the pilot's hands (0.90 at
10.85 m/s against 1.00 at 11.98); 0.05 puts four there. **But `ControlStep` is
one constant used at all fourteen step-control sites in the pawn** — brakes,
weight shift, accelerator — so halving it doubles the presses to reach full
brake on every axis, not just this one. That is a control-feel judgement for
the pilot, exactly like the `canopyPitchDampingRatio` target, and the
alternative (a brake-only step, or a held-key ramp) is a design choice rather
than a tuning one. **Not changed here.**
- Done when, for the legacy model: a stall recovery shows more than one
  surge-pitch cycle, the excursion scales with the stall rather than saturating
  at a clamp, and whichever constants moved are justified against the
  pilot-visible criterion the `canopyPitchDampingRatio` note already states.
- **THIS IS ITEM 19'S AXIS, FROM THE SAME PILOT, AND THE TWO DIAGNOSES
  INTERLOCK.** Item 19 found that `pitchStiffness` is referenced to
  **incidence** rather than gravity — the weathercock and the pendulum folded
  into one spring — so nothing in the legacy pitch axis references gravity at
  all. **A pendulum needs two things to ring: a gravity-referenced restoring
  term, and damping low enough to let it.** Item 19 is the missing (a); this
  item is the violated (b), at ζ 0.69 with two clamps on top. Neither alone
  explains the report and neither alone fixes it — a gravity term added under
  ζ 0.69 would creep to the new attitude instead of swinging, and lowering the
  damping without a gravity term leaves nothing to swing *about*.
- **So the routing is: item 19 near-term, item 17 properly.** Item 19 is
  already diagnosed and is blocked on exactly one thing — *"the target numbers
  want a pilot's judgement rather than a plausible constant"*. That judgement is
  now available: the pilot has described the target behaviour twice, in
  pendulum terms both times. Item 17 (Level 10's exit gate) is the structural
  fix and is **blocked** — the geometry-driven stack departs at 37% brake and
  22% bar with weight shift at 0.01 rad/s, and item 19 already records that it
  "cannot fly a stall recovery either, so it is not a swap-in fix."
- **Do not wait for item 17 for this.** Four hand-tuned dampers and two clamps
  on one axis is what a lumped model costs, and the coupled solver carries the
  angle as a real degree of freedom — but that is a Level 10 exit gate behind
  two departures and a dead control, and this is a defect a pilot hits on every
  stall today.

**THE COUPLED SOLVER HAS ITS OWN VERSION OF THIS, FOUND ON THE WAY, AND IT IS
STILL WORTH HAVING** — it is what item 7 will migrate onto, so it will become
the pilot's aircraft. Diagnosis from reading the solver, with arithmetic — not
yet measured, and NOT the cause of the flight report above:

1. **`1.0 - 1.6 * dt` on the canopy's angular rate**, every step, all three
   axes (`CoupledParagliderSolver.cpp`, the `Structural damping` block). A
   0.62 s time constant. Over one 1.86 s pendulum period it takes the wing's
   rotation rate to `exp(-1.6 x 1.86)` — **5% left per swing**. Its own comment
   says *"Without this the pendulum rings"*, which is the reported symptom
   written down as an intention. **The literal `1.6` appears exactly once in
   the repository: no registry entry, no derivation, no test, no sweep.** Every
   other coefficient in this stack has been argued over for levels.
2. **The 1.4 rad swing clamp sets `linkRate` to zero**, discarding the whole
   swing kinetic energy in one step. Its comment says "well outside anything
   short of an SIV manoeuvre" — and a hard stall recovery IS one. **This is the
   only candidate that explains amplitude-independence:** a damper makes a
   harder stall surge proportionally harder for the same number of cycles,
   whereas a clamp makes every stall above threshold recover identically.
   Appears in no test.
3. **`swingDampingRatio` at 0.35**, item 11's known ~0.18. A mode at ζ 0.35
   keeps 10% per cycle — one movement. At the ~0.06 that pilot and line drag
   imply it keeps 69% — five to eight decaying swings. **The pilot's
   description is the ~0.06 behaviour, quantitatively**, which is the outside
   evidence item 11 has never had.

- **Why the eigenmode programme never saw it.** The fast mode is 1.86 s at
  ζ≈0.09 and is predominantly *link swing*; the post-stall surge is
  predominantly *canopy pitch rotation*, and (1) acts on the canopy rate. The
  instrument and the pilot are watching different coordinates of the same axis.
- **Worse, the tool was told to discard it.** Item 11's rule is *"treat
  anything with ζ > 0.5 from this tool as discretisation until it survives a
  change of T"*. A 1.86 s mode decaying at 1.6/s has an equivalent **ζ ≈ 0.47**,
  rising for anything slower. That rule was written for spurious rows at T=2 and
  may have been eating a real over-damped canopy-pitch mode ever since. It is
  now a testable question, not a worry: **zero the structural damping and the
  ζ > 0.5 rows either move or they do not.**
- **The energy audit has a blind spot exactly where all three live.**
  `energyResidualW` is translational KE + PE against aerodynamic work. Canopy
  *rotational* KE and pendulum *swing* KE are in neither term. Its comment —
  "a subsystem that creates energy cannot hide from this" — is true for
  creation and false for destruction: all three of these dissipate rotational
  energy invisibly. **That is why nine levels of energy accounting never
  flagged a term that removes 95% of the wing's rotational energy per swing.**
- Done when, for the coupled solver: a stall recovery shows **more than one**
  surge-pitch cycle, the cycle count and decay follow the stall's severity
  rather than being pinned by a clamp, and the rotational and swing energy are
  inside the audit so a dissipator on this axis cannot hide again.
- Bounded in `coupled_tests` by surge-peak count, so a regression to the single
  movement registers.
- **Instrument built, not yet run: `pitch_axis_trace --surge`.** Sweeps the
  three coupled-solver terms against three stall severities and reports the
  energy each removes in JOULES rather than the coefficient each carries, so
  which one dominates is measured rather than argued. It prints the safety
  envelope flag per row, because at `aerodynamicsInterval` 6 a nominal frontal
  now engages it (item 25) and a recovery measured through the envelope is not
  flight behaviour.
- **A LESSON ABOUT THIS FILE, not about the wing.** Item 7 states in writing
  that the pawn flies `ParagliderDynamics`, and the first pass at this item read
  three levels of pitch-axis history without checking which model the reporter
  was flying. §80 recorded the same shape of failure one session earlier —
  reaching for an instrument the file had already superseded — and called it a
  retrieval failure rather than a knowledge one. **Twice now, and both times the
  fact needed was in this file.** The cheap guard is a question, asked before any
  reading: *which model produced this observation?*

**25. THE SCHEDULE FIX IS NOT FREE, AND "ALL TWELVE SUITES GREEN" DID NOT HOLD
AT THE INTERVAL IT SHIPPED.** `aerodynamicsInterval` went 12 → 6 on §79/§80's
evidence. Bisected by running `coupled_tests` at both intervals with nothing
else different: **4 checks fail at 6, all pass at 12.**

- Three are **stale expectations**, and re-deriving them is routine: §68's
  iteration-cap gates pin break times and residuals identified at the 0.1 s
  hold, and a different hold moves them. No conclusion of §68's changes — the
  finding was that fifteen times the iteration budget breaks on the *same*
  tick, and that is a statement about iterations, not about the hold.
  **AND ITEM 30 HAS SINCE MEASURED THE CURVE THEY SIT ON.** The frontal's
  symmetry break moves monotonically with the aerodynamic hold on the shipped
  quasi-steady wing — **1.083 s at 120 Hz through 1.350 s at the shipped
  interval 6 to 1.600 s at 5 Hz**. These three gates are not three independent
  facts that went stale; they are one curve read at two points. Re-deriving
  them means pinning them to the curve rather than to a hold.
- **The fourth is not stale.** It is §69's **control**:
  `!shipped.safetyEnvelopeEngaged` on the 4 m/s symmetric frontal. At 12 the
  shipped wing flies it with the envelope idle; **at 6 it engages.** Two
  consequences, and the second is worse than the first:
  - By guiding rule 12, a shipped wing that needs the numerical safety envelope
    in a **nominal** incident is a larger fact than the stability boundary
    moving 0.01. This is the wing getting harder to represent, not a number
    getting better.
  - It removes the control that makes §69's two drag rows mean anything. **Item
    12's "no route around Level 11" currently rests on a benchmark whose own
    control fails**, so that conclusion is suspended rather than wrong.
- **Not reverted.** §78/§79/§80's three measurements that the 0.1 s hold
  manufactures departures not in the aircraft are untouched by this, and the
  case for a finer hold stands on them. What does not stand is the claim the
  change was clean.
- **The decision has not been taken**, and it is a scoping call rather than a
  physics one: re-derive the three §68 gates and fix or accept the frontal; or
  hold the schedule at 12 until Level 11 and keep the finer hold as a
  measurement instrument only. The second is cheap and honest and costs the
  ~0.05 of boundary §78/§79 bought.
- **Consequence for item 24, which is why this was found now:** any stall
  recovery measured at interval 6 must print the envelope flag beside it. A
  recovery that passes through the safety envelope is not flight behaviour, and
  the surge-count measurement item 24 needs would be meaningless through it.
- Done when: the suite is green at the interval that ships, and which of the
  two routes was taken is written down with its cost.

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
- **AND THE MECHANISM IS NOW MEASURED RATHER THAN ASSERTED.** The sentence
  above has been carried since it was written. `aerodynamics_tests` sweeps
  incidence through the stall and locates two crossings *independently* — one
  from the polar alone with no solve in it, one from the iteration alone with
  no polar in it:

  | α | dCl/dα | residual @8 | residual @64 | 8× bought | converges |
  |---|---|---|---|---|---|
  | 2° | 7.033 | 7.98e-03 | 2.61e-06 | 3053× | yes |
  | 10° | 6.846 | 7.92e-03 | 2.70e-06 | 2931× | yes |
  | **12°** | **−3.073** | 1.83e+00 | 4.51e-01 | **4×** | **NO** |
  | 18° | −1.970 | 8.28e-01 | 1.03e+00 | 1× | NO |
  | 25° | −0.840 | 2.06e+00 | 1.15e+00 | 2× | NO |

  **The lift slope changes sign by 12.0°. The iteration stops converging by
  12.0°.** Item 6's sentence is the one the solver obeys.
- **The criterion is therefore available from the POLAR**, at a section's own
  incidence, before any iterating — a table lookup rather than a failed solve.
  That is what route 3 (item 30) needs to declare the separated regime on
  ENTRY instead of diagnosing it afterwards, and it costs one sample.
- **Two instruments had to be thrown away to get this, and both are the
  obvious one.** First, a contraction factor — the ratio of target residuals
  at k and k+1 passes, which is the textbook statistic for "does this iterate
  converge". It does not separate the regimes at all: at 25°, where the target
  provably wanders one to five times the step away, the 8-to-9 ratio is
  **0.429**, which reads as healthy contraction. A single pair of passes cannot
  tell contraction from an iterate part-way round a cycle. Second, a magnitude
  threshold on the residual — but the attached floor is 2.6e-06 against the
  solver's own 1e-06 tolerance, so reading that constant literally calls a
  converged solve non-convergent, and any looser number is chosen to make the
  answer come out. What needs no chosen constant is whether **eight times the
  budget bought anything**: 3000× attached, 1–4× separated, and in one row the
  residual is *larger* at 64 passes than at 8.
- The honest treatment is Level 11's unsteady wake. **But item 30 has since
  measured that strand 2 does not supply it** — the only configuration that
  ever held the symmetric frontal is one where the aerodynamic states run six
  times slow, which is a defect and not a scheme. **Item 6 is upstream of the
  wake, not downstream of it.**
- Note this is about the *cold* solve. Inside the coupled solver, with Level 4's
  separation state carried between steps, the wing walks into a fully separated
  46-degree stall at 4.65 m/s of sink without the solve failing at all.

## Integration debt

**23. Level 7 is reopened: its turn gate was passed on half of its text.** The
gate is *"weight shift and brake turns EMERGE without direct turn moments"*. The
absence of a direct turn moment was verified and is real. The emergence was
never checked, and §71 measures it at **0.014 rad/s** for full weight shift.

- Not a regression: it was never true. The level was closed on the half of the
  sentence that was testable at the time.
- **The lesson is about the gate's grammar rather than the wing.** "X emerges
  without Y" is two claims, and the negative one is much easier to check - no
  term named `weightShiftBank` appears anywhere, which is a grep. The positive
  one needs a manoeuvre and a number, and it was not written.
- **Swept, and the shape is common.** Sixteen exit-gate bullets across the
  ladder are two-claim sentences. Most are safe because the positive half is
  the easy half - "hands-up trim converges without a speed controller" (Level
  4) is verified by the trim existing. The ones worth re-checking are those
  where the *positive* half needs a manoeuvre:
  - **Level 8, "asymmetric separation produces spin/spiral behaviour, not
    barrel rolls."** Directly downstream of the same missing roll authority:
    with weight shift at 0.014 rad/s and brake turns 6x slow (item 0b), a
    spiral emerging properly is doubtful. Highest-risk of the remaining set.
  - **Level 2, "weight shift changes carabiner loads AND attachment
    geometry."** The load half is now gated at a 34% split (§71); the
    attachment-geometry half rides on the same anchor translation and is
    probably fine, but it has not been asserted.
  - This sweep checked the *grammar*, not each claim. Naming which halves are
    unverified is what it delivers.
- Closed by the geometric channel (item 21's design).

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

**11. REWRITTEN TWICE. This aircraft has a second, much slower pitch mode that
nobody had flown long enough to see.** Run `parapenting_pitch_axis_trace`.

Incidence spread over a ten-second window, still air, no input:

| after | hands-up | 25% brake |
|---|---|---|
| 30 s | 1.010° | 3.426° |
| 60 s | 0.597° | 2.260° |
| 120 s | 0.361° | 2.100° |
| 240 s | 0.074° | 1.017° |
| 480 s | 0.004° | 0.319° |
| 960 s | **0.000°** | **0.022°** |

It settles. It just takes **eight to sixteen minutes**, and every settle this
project has ever used was 20, 40 or 60 seconds.

**The slow mode, measured.** Period **16.3 s**, damping ratio **≈0.030**, taken
off 35 successive peaks of a 1200 s hands-up run. Incidence and airspeed move
in antiphase — 5.46° at 10.30 m/s, 4.63° at 10.78 — which is what a phugoid is.

| | measured | classical phugoid |
|---|---|---|
| period | 16.3 s | 4.80 s (πV√2/g at 10.6 m/s) |
| damping ratio | 0.030 | 0.062 (1/(√2·L/D) at glide 11.33) |

So the period is **3.4× longer than theory** and the damping about half.

**Both are now explained, by one mechanism, and the explanation is measured
rather than argued.** The classical formulas assume incidence is held fixed, so
lift and drag both go as V². Write the exponents as L ∝ Vⁿ, D ∝ V^d and the
phugoid is `ω = g√n/V`, `ζ = (d/2)/((L/D)√n)`. Measured off the flight path of
the same 1200 s run — `L = m(g cos γ + V γ̇)`, `D = −m(g sin γ + V̇)`, so the
aerodynamic loads under suspicion are not what is being trusted:

| | measured | classical | prediction from the measurement |
|---|---|---|---|
| lift exponent n | **0.171** | 2 | period **16.42 s** against 16.39 measured |
| drag exponent d | **0.313** | 2 | ζ **0.034** against 0.031 measured |

And 0.172 is the n the 3.4× period implies, arrived at from the other side.

**The pendulum does not hold incidence through the oscillation, it holds LIFT.**
Slowing down, the wing rotates nose-up on its lines at −1.69°/(m/s) and recovers
in incidence nearly everything it lost in dynamic pressure — lift varies by
**0.97% of weight** over the whole mode. The restoring force is the leftover,
and the leftover is almost nothing. That is the long period, and the same
flatness in drag is the low damping.

Taking n without d would have been worse than taking neither: it predicts
ζ = 0.221 against 0.031 measured, where untouched classical says 0.065. Lift
and drag are flat for the *same* reason, so they are one finding. See
`PHYSICS_LEARNINGS` §34.

**What this does not settle.** Whether −1.69°/(m/s) is *right*. Both roads come
off one trace, so what is established is that this is a phugoid whose restoring
gradient is nearly cancelled — not that the cancellation is the size a real
wing's would be. But the open question is now physical: that slope is the
pendulum tracking apparent gravity, which is exactly what `swingDampingRatio`
stands in for, and it is checkable against a real wing in a way "the period is
3.4× theory" never was.

- **What `calibration_tests` calls "Pitch: period 2.91 s, damping 0.28" is not
  this mode and never claimed to be.** It measures the wing oscillating
  against the pilot after a brake pulse — the pendulum mode — and gates it as
  "faster than a simple pendulum on the same lines". That is correct and
  healthy. The error was mine: I used the pendulum mode's damping to argue
  that a residual at 60 s "could not be a decaying mode", when the mode
  actually decaying has a fifth of that damping and five times the period.
  Two modes, and only the fast one had ever been characterised.
- **It is not the schedule — for the SLOW MODE, which is all this measured.**
  Solving the aerodynamics at 120 Hz instead of 10 Hz moves the 60 s spread
  from 0.597° to 0.528°. Consistent with a physical mode rather than a
  discretisation artefact. **Scope, added by §77: this was run on a settling
  wing at ratio 0.35 and contains no departure, so it says nothing about the
  stability boundary.** The boundary was tested separately and the answer is
  split — classes invariant, rate not. See the schedule sweep below.
- **It is not the section's stall hysteresis.** Hands-up the wing flies at
  4.9° against a section stall near 12°, so that loop is nowhere near active.
- Settled properly at 960 s: hands-up **4.925°**, 25% brake **5.724°**. Brake
  raises incidence by 0.80°, **which is the correct sign.**
**Settling time, flown to a 0.01° criterion rather than a clock:**

| brake | settles at | took | note |
|---|---|---|---|
| 0.00–0.15 | 4.925° | 410 s | inside the sewn-in slack, so all identical |
| 0.20 | 5.011° | 460 s | |
| 0.25 | 5.719° | 1080 s | |
| 0.30 | — | >1200 s | still moving, 0.52° spread |
| 0.35 | — | >1200 s | still moving, 2.62° spread |

- **Settling time grows far faster than the input.** Deep brake does not settle
  inside twenty minutes of simulated flight at all, so *no* deep-brake number
  anywhere in this project is a trim point. That includes the 40% departure
  this item has quoted for several levels.
- **Two things that do NOT change the settled answer.** The aerodynamic
  interval moves it by 0.01° (though it halves the settling *time* under
  brake, 1080 s to 560 s at 120 Hz). And `swingDampingRatio` from 0.35 to 0.90
  gives the same 5.72° and the same +0.79° brake response every time — only
  the settling time changes, 410 s down to 80 s.
- **That second one matters more than it looks.** The settled numbers do not
  depend on the one tuned coefficient in this axis. The ratio buys settling
  speed, not a trim. At 0.25 the aircraft departs, which is what actually pins
  it at 0.35 rather than the 0.06 that pilot and line drag imply.
- **DONE: the phugoid period and damping are explained rather than observed.**
  Both fall out of n = 0.171 and d = 0.313, measured off the flight path, and
  each predicts its number to within 2%. See the table above.
- **A consequence worth stating separately: the slow mode is NOT evidence about
  `swingDampingRatio`.** Its period and damping are accounted for by the wing's
  own lift and drag gradients and the glide ratio, with the tuned coefficient
  appearing nowhere in either formula. So the 0.031 damping ratio can no longer
  be read as "the pendulum damping is wrong" — it is what a wing with a nearly
  cancelled speed-lift gradient does regardless.
- **The departure is NOT the phugoid, and that was a prediction that failed.**
  `ω = g√n/V` diverges for n < 0, so if the departure were this mechanism the
  lift exponent would change sign exactly at the stability boundary. It does
  not. `parapenting_pitch_axis_trace --departure`:

| ratio | n | d | dα/dV | outcome | growing | per cycle |
|---|---|---|---|---|---|---|
| 0.20 | 0.190 | 0.514 | −1.504 | DEPARTED at 93 s | **5.68 s** | ×1.302 |
| 0.25 | 0.145 | 0.661 | −1.272 | DEPARTED at 348 s | **3.62 s** | ×1.113 |
| 0.30 | 0.181 | 0.341 | −1.674 | flying at 400 s | — | — |
| 0.35 | 0.170 | 0.306 | −1.686 | flying at 400 s | — | — |
| 0.50 | 0.142 | 0.252 | −1.715 | flying at 400 s | — | — |

  n stays between 0.14 and 0.19 across the boundary and does not even trend —
  0.20, which departs soonest, has the *highest* n in the table. Fed through
  `ζ = (d/2)/((L/D)√n)` the departing rows come out better damped (0.056 at
  0.20 against 0.034 at 0.35). The phugoid does not go unstable here.

- **What does: a mode in the pendulum band, measured by its period.** Off a
  10 Hz trace of the last 40 s before the wing lets go — 1 Hz cannot resolve a
  2.91 s mode, and undersampling a mode into invisibility is how this item lost
  two levels — the thing growing is **3.6–5.7 s**, not the phugoid's 16.4 s. Net
  damping ratio **−0.017 at 0.25 and −0.042 at 0.20**, so the stability boundary
  is between 0.25 and 0.30. The two departing runs disagree on period and the
  amplitude is large by then, so "pendulum band" is as strong a claim as this
  supports — but an order of magnitude is not a close call.
- **One thing did follow the ratio, and it is the phugoid mechanism:** dα/dV
  goes −1.27 to −1.72 as the ratio rises. `swingDampingRatio` really is buying
  pendulum tracking, exactly as the exponent work implied. Tracking is simply
  not what the departure is made of.
- **Measuring the fast mode's damping directly was attempted and did NOT
  succeed.** `parapenting_pitch_axis_trace --fast-mode` excites it with the same
  30% pulse `calibration_tests` uses, subtracts a control run to remove the slow
  mode exactly, and fits `C + A e^{−σt} cos(ωt+φ)` by grid search. The sweep is
  printed and marked not reportable, on three counts: the ratio-0.35 check row
  fits to R² 0.89 against this file's own 0.90 bar; damping comes out positive
  at every ratio including the two that depart, so it never crosses the zero
  `--departure` places between 0.25 and 0.30; and it is not monotonic in the
  ratio (0.21, 0.51, 0.61, 0.60, 0.51, 0.25, 0.15), which has more damper buying
  less damping over half its range. Two cycles at R² 0.9 is a fit trading decay
  against frequency, not a measurement.
- **What the attempt did establish, and it matters: the fast mode is dead by
  about 2.5 s.** `--fast-mode-dump` prints the swing trace; from 2.65 s to
  8.95 s it is monotonic, with no zero crossings at all. **`CalibrationManeuver`
  identifies its "period 2.91 s, damping 0.28" on a window that STARTS at 2 s**,
  so it is reading a mode that has largely ended. That number is gated in
  `calibration_tests` and is now in doubt. It was not changed, because doubt is
  not a measurement — but nothing should lean on it until this is settled.
- **BUILT, and it works: `parapenting_pitch_eigenmodes`.** Perturb the settled
  aircraft one state at a time, run each perturbation a fixed short time,
  difference against an unperturbed run — that is the state transition matrix,
  and its eigenvalues are every longitudinal mode at once. No excitation to
  design, no window, no filter, no superposition assumption. Six states: surge,
  heave, pitch attitude, pitch rate, link swing, link rate. Settle is paid once
  and the settled solver copied for all seven runs, which is what makes it
  affordable.

| transition time T | slow mode | fast mode |
|---|---|---|
| 0.10 s | 17.06 s, ζ 0.067 | 1.87 s, ζ 0.103 |
| 0.25 s | 16.60 s, ζ 0.054 | 1.86 s, ζ 0.092 |
| 0.50 s | 16.48 s, ζ 0.044 | 1.86 s, ζ 0.086 |
| 2.00 s | **16.40 s, ζ 0.033** | aliased to 4.00 s |
| *trace, independent* | *16.39 s, ζ 0.031* | — |

- **It passes its own check.** The slow mode converges onto the 16.39 s and
  0.031 measured off 27 peaks of a 1200 s run by completely different means.
  The convergence direction is right too: a slow mode's eigenvalue sits nearer
  1 the shorter T is, so short T resolves its damping worst. The two modes want
  different sampling intervals, which is why the band is reported rather than a
  favourite.
- **THE FAST MODE IS 1.86 s WITH ζ ≈ 0.09, NOT 2.91 s WITH ζ 0.28.** Stable
  across every T that does not alias it, and it agrees with the control-run
  trace measurement of the previous strand (≈1.85 s by hand off the dump). Two
  independent instruments, one in the time domain and one not, against the
  `calibration_tests` figure — which is identified on a window starting 2 s
  after release, by which time this mode has largely ended. **That gate should
  now be re-derived rather than trusted.** It has still not been edited: the
  right fix is to point `CalibrationManeuver` at a window that starts at the
  release, and that is a change to gated behaviour, so it wants its own commit.
- **The linearity check says which numbers to trust.** Halving every
  perturbation at matching T:

| | full step | half step | verdict |
|---|---|---|---|
| fast mode, T=0.25 | 1.86 s, ζ 0.0920 | 1.86 s, ζ 0.0922 | unmoved — solid |
| slow period, T=2 | 16.39 s | 16.40 s | unmoved — solid |
| slow damping, T=0.25 | ζ 0.0540 | ζ 0.0437 | **moves 19%** |
| slow damping, T=2 | ζ 0.0362 | ζ 0.0334 | moves 8% |

  So the fast mode and both periods are converged; **the slow mode's damping is
  not** — it moves with T *and* with step size, and is the one number here still
  to be pinned. It does bracket the trace's 0.031 from above throughout.
- **Aliasing, kept on the record because it cost a run to recognise.** Modes
  faster than 2T fold to exactly 2T: the 4.00 s row at T=2 is the fast mode
  aliased, not a mode. An early version reported 4.00 s at T=2 and 12.00 s at
  T=6 — two suspiciously round numbers, each exactly twice its own sampling
  interval.
- Spurious heavily-damped rows (26.96 s at ζ 0.77, half-life 2.5 s) appear only
  at T=2 and not below; treat anything with ζ > 0.5 from this tool as
  discretisation until it survives a change of T.
- **The sweep ran, and the prediction in this bullet was wrong.** `--sweep` takes
  the spectrum through the boundary. The fast mode's eigenvalue **does not cross
  into the right half plane** — it moves −0.357 to −0.291 /s from ratio 0.90 to
  0.10 and its period does not move at all (1.86 s throughout), so the pendulum
  is not what pins the ratio at 0.35. What crosses, between **0.28 and 0.25**, is
  the **16 s phugoid**, arriving by its damping while its frequency stays real —
  its period tracks 23.9 → 14.0 s. Same interval at T = 0.25 and T = 0.10.
- **A second criterion brackets the same interval, sharing no arithmetic with an
  eigenvalue:** settle each ratio from scratch and ask whether a trim exists at
  all. 0.30 settles, stable by a hair (−0.008/s); **0.25 departs during its own
  settle**, at 348 s and α 20.2°. That also disposes of the sweep's stated caveat
  — that it linearises about 0.35's trim rather than each ratio's — and the drift
  column at full settle (1.8 × 10⁻⁵ rad/s, 100× smaller than on a short settle)
  confirms the point is a trim.
- **This contradicts §35's acquittal of the phugoid, and §35's arithmetic was
  fine.** It predicted the lift exponent `n` crossing zero — a claim about the
  mode's *frequency*, `ω = g√n/V`. n holds at 0.14–0.19, which rules out the
  oscillation becoming a divergence and nothing more. The phugoid arrives by the
  half of the eigenvalue that prediction did not address. See `PHYSICS_LEARNINGS`
  §38.
- **Load-bearing caveat:** the quantity that crosses is the slow mode's damping —
  the one number the linearity check above says is *not* converged. The ordering
  is solid and monotone across three runs; the 0.28–0.25 interval is not, and
  should not be quoted tighter.
- **The time-domain confirmation ran (`--phugoid`), and it moved the answer.**
  Speed as the observable, 300 s, first 25 s discarded so the fast mode is gone
  rather than filtered. R² 0.98–1.000 on the clean rows, 30-plus extrema, and
  the fitted period tracks the eigenvalue's to ~1% — so both are watching the
  same mode. Halving the perturbation moves the rate 0.0115 → 0.0116.
- **Where they disagree, the eigenvalue is wrong, and an outside number says so.**
  At 0.35 the flown fit gives 16.38 s and ζ 0.0299 against `pitch_axis_trace
  --slow-mode`'s 16.39 s and 0.031 — 0.1% and 3%. The eigenvalue says ζ 0.0540,
  high by three quarters, in the one number §37's linearity check had already
  flagged as unconverged and bracketing the trace from above.
- **So the crossing is between ratio 0.35 and 0.30, not 0.28 and 0.25.** The
  own-trim settling behaviour agrees and fits nothing: 0.35 settles at 410 s,
  0.30 fails to settle in 420 s, 0.25 departs. A marginally growing phugoid is
  why 0.30 has no trim.
- **This explains the coefficient rather than bounding it: 0.35 is approximately
  the smallest value at which this wing's phugoid still damps**, not a margin
  chosen above a departure.
- Rows at 0.25 and 0.20 are excluded as evidence — R² 0.495 and 0.375, and the
  0.20 fit returns half the phugoid's period. Outside small-amplitude behaviour
  within the window, where this fit has no claim. The finding rests on 0.50–0.28.
- ~~**Next:** the missing mechanism has to act on **speed stability**, the flat
  lift curve of §34, not on the link.~~ **RETRACTED by the run below.** That was
  inferred from §34's damping formula, which `--phugoid`'s exponent table then
  showed is anti-correlated with the truth over exactly this parameter. The
  conclusion inherited the error of its premise.
- **§34's damping formula cannot contain this instability, and the test was one
  line of algebra nobody had run:** `ζ = (d/2)/((L/D)√n)` is positive whenever n
  and d are. The flown damping goes negative. So either d crosses zero or the
  model is structurally incapable — and §35 had only ever tested n.
- **d does not cross. It rises, 0.281 → 0.459 as the ratio falls**, so predicted
  ζ *rises* 0.0341 → 0.0510 over the interval where flown ζ *falls* through zero,
  0.1598 → −0.0167. Opposite directions.
- **The control holds, which is what makes that conclusive.** The same fit's
  period prediction lands within 1–4% at every ratio (18.07 vs 18.28, 16.44 vs
  16.38, 16.02 vs 15.82, 16.52 vs 15.88). The exponents are real; only the
  damping half of the model fails.
- **§34 is right at a point and wrong as a function.** At ratio 0.35 it still
  gives 0.0363 against 0.0299 flown — its original 0.034 against 0.031. A
  point-fit had been serving as a mechanism for three levels.
- **Next:** the dependence is not in the two-state theory at all — n and d move
  25% and 60% while flown damping moves 0.18 and changes sign. What the ratio
  changes is the **link**, the one state the two-state phugoid lacks. So the
  target is the **pendulum–phugoid coupling**: the six-state eigenproblem
  contains it and the two-state formula structurally cannot. The eigenvectors
  are the obvious next instrument — they are already computable from the
  transition matrix `--sweep` builds, and they would say how much link motion
  the 16 s mode actually carries and how that changes as the ratio falls.
- **The eigenvectors ran (`--shape`), and they kill the lag story specifically.**
  The link-to-surge phase in the 16 s mode is **constant within 1.9°** (−107.4 to
  −109.3) across the whole sweep while σ crosses zero — and −106.4 to −108.2 at
  T = 0.10, so it is the aircraft, not the discretisation. A tracking lag is
  exactly what would have moved that phase, and it is the mechanism the solver's
  own comment blames ("damped against the WORLD… a cost paid knowingly").
- **The prediction was badly posed and the amplitude column says so.** Work per
  cycle is amplitude × sin(phase); link/speed rises 0.319 → 0.510 (60%) over the
  same sweep. So a **gain** version of the coupling survives — but it was not
  predicted in advance and is not claimed, which is the §40 trap.
- **The energy integral ran, in a form with no model of energy in it** (`--shape`,
  `SplitCheck`): the left eigenvector maps a *measured* change in the matrix to
  the change it makes in the growth rate, entry by entry and additively, and the
  four block shares must add up to the measured Δσ. They do, to 7%, degrading to
  14% on a double-width step — first-order behaviour.
- **The answer is the adjoint: 0.985 of the 16 s mode's left eigenvector sits on
  the link's two rows.** The mode *looks* like a speed oscillation
  (articulation 0.29 vs the pendulum's 1.07) and *listens* almost entirely
  through the link. Conditioning |w^H v| = 0.10 — it is non-normal, and that gap
  is the mechanism.
- **So a coefficient living only in the link's equations can take the phugoid's
  damping through zero**, and §34's two-state theory cannot see it because it has
  no link row for the mode to listen through — not because its aerodynamics are
  wrong. `PHYSICS_LEARNINGS` §42.
- That 99% of the movement enters rows 4–5 is *near-tautological* and is labelled
  as such: `swingDampingRatio` appears only in the link's update.
- **Both follow-ups ran, and both came back negative** (`--shape`,
  `ReceptivityCheck`; `PHYSICS_LEARNINGS` §43).
- **The 0.985 was unit-flattered.** Rescaling the states is a similarity
  transform — eigenvalues untouched — and the share goes **0.9854 → 0.7786**.
  Still link-dominated, so §42's conclusion holds; the figure does not, and it
  had been quoted to three digits. §41's own lesson, not applied to §42.
  `cond` = 0.10 is invariant by construction, so the non-normality stands.
- **The receptivity is not fixed: 0.8898 → 0.7562 across the sweep**, monotone,
  while σ crosses zero. As the mode destabilises it listens *less* through the
  link — the same direction as §41's articulation result, from an independent
  column.
- **So "it destabilises because it couples to the link more" is wrong.** The link
  is the *channel* (adjoint link-dominated at every ratio, 0.76–0.89) and §42's
  split stands, but the trend in σ is not a trend in coupling strength — that
  has the wrong sign. What changes is the link's own dynamics, transmitted
  through a weakening channel.
- **The two terms cannot be separated by the coefficient, and the solver shows
  why.** All of `swingDampingRatio` enters as one scalar gain on the link's rate
  increment — `linkRate = (linkRate + linkAngularAccel*dt)/(1 + 2ζω dt)` — so it
  attenuates the wing's acceleration feeding the link at the same time. It is a
  damper *and* a coupling gain, which is why §42's split was spread across two
  blocks.
- **So the design question was asked of the matrix instead** (`--shape`,
  `DesignCheck`): `∂σ/∂Φ_ij` per unit relative change, finite-difference checked
  to ~1% on 2% perturbations.
- **A 1% change to `d(swing)/d(swing)` moves σ by +0.0133. The entire 0.35 → 0.30
  coefficient step moves it +0.0129.** One per cent of one entry is worth the
  whole step, and the top three entries are within a quarter of each other — all
  of them in the swing *angle* row.
- **Hypothesis this raises, and it may reframe the item:** the boundary's
  location is not robust. A 1% error anywhere in the link's rows moves it by the
  whole step, so 0.35 is arguably more a property of how the link is written than
  of a paraglider — and "find the missing stabilising mechanism" may be the wrong
  frame, with "a formulation whose stability is less sensitive" the real
  requirement. Not a conclusion; §40 is what happens when that step is skipped.
  `PHYSICS_LEARNINGS` §44.
- **The cheaper falsifier ran first, and it was the control §44 lacked:** the
  other twenty-four entries. **`d(surge)/d(surge)` = +1.631 is the largest
  sensitivity in the matrix**, above every link entry — worth +0.0163 per 1%, or
  about 1.25 coefficient steps, confirmed by finite difference to 0.3%.
- By block the link rows remain the more sensitive on average (rms 0.822 vs
  0.433) but the peak is on the wing side. **So fragility is a property of a
  non-normal mode, not of the link's formulation**, and §44's reframing is
  weakened by its own missing control. `PHYSICS_LEARNINGS` §45.
- **§39's retracted claim has independent support now, and the retraction still
  stands.** `d(surge)/d(surge)` *is* speed stability; §39 named it, §40 correctly
  retracted the *inference* that produced it (drawn from §34's anti-correlated
  damping formula). The claim is now supported by a measurement sharing no
  arithmetic with §34. The argument stays retracted; the claim does not.
- **That step ran and did not resolve** (`--shape`, `SurgeCheck`). `(Φ₀₀−1)/T` is
  not a constant — −0.082 at T = 0.10, −0.152 at 0.30, −0.19 at 0.50 — because
  `Φ = exp(AT)` carries `A²T/2` and it is not small here. There is no single A₀₀
  to compare against the closed form, and a first draft that read the widest T
  would have reported a factor of seven with a tidy story attached.
- **Extrapolated: −0.052 /s against −0.028 from the drag exponent.** Same order,
  factor 1.84 — the exponent accounts for a bit over half the surge decay. And
  the extrapolation reaches below the 0.1 s aerodynamic interval, i.e. outside
  the model, so it is not trustworthy either. `PHYSICS_LEARNINGS` §46.
- **Scope of that caveat, deliberately narrow:** every other result here is built
  from *eigenvalues* of Φ, exact for whatever T was used and cross-checked at
  several T. Comparing individual *entries* against continuous-time formulas is
  the fragile move and this is the only place it was made. §45's ranking compares
  entries of one matrix at one T and is unaffected.
- **The matrix logarithm ran** (`--shape`, `LogarithmCheck`), verified two ways:
  imaginary residual ≤1e-10, and `exp(AT)` rebuilt by scaling-and-squaring
  reproduces Φ to 1e-11.
- **There is no continuous A.** A₀₀ = −0.035 at T = 0.10, −0.021 at 0.25, **+0.006
  at 0.50** — it changes sign, spread 290% of its mean. Φ(T) is not an
  exponential family; the 0.1 s aerodynamic hold is the culprit. §46's comparison
  cannot be made this way, definitively. `PHYSICS_LEARNINGS` §47.
- **This closes the oldest loose end here: the slow mode's damping was never
  going to converge.** §37 flagged it as moving with T and step size, "the one
  number still to be pinned", and three levels treated that as a measurement
  needing improvement. A rate off Φ(T) has no T-independent value when Φ is not
  an exponential family. **Quote it with its T; do not chase it.**
- **Checked rather than assumed:** periods are T-stable (1.86 s everywhere);
  rates move ~20% with T, which is the band already documented; entries move by
  factors. §45's ranking is entry-based and was the one at risk — it holds, with
  the same top four at T = 0.10 and 0.25 and block ratios 1.87 vs 1.89.
- **The gate is done (§48).** `calibration_tests` no longer publishes 2.91 s /
  0.28 as the pitch mode. `IdentifyOscillation` is exported so the test can vary
  the window, and the window sweep shows the number is an artefact of it: nothing
  at 3.5 s, 1.42 at 5.0, 1.51 at 7.0, 2.91 at 9.0, nothing at 12 and 20, never
  more than one oscillation. A decay check fails too (excursion ratio 0.96 from
  2–5 s to 8–11 s) because by 8 s the signal is the 16 s mode. **The brake pulse
  cannot measure the fast mode.**
- The gate now asserts a bounded, measurable transient plus the simple-pendulum
  bound, and names the eigenmodes as the authority. Note the old bounds passed
  *both* 2.91/0.28 and 1.86/0.09 — they never distinguished them, so their
  passing was never evidence. All twelve suites green with the change.
- **§35's 3.6–5.7 s mode is retired as an artefact (§49).** Running §35's own
  peak counter on a synthetic signal containing **only** the two known modes —
  1.86 s decaying, 15.4 s growing — returns 3.64, 3.65, 3.69, 4.93 and 5.17 s,
  bracketing the reported band from both ends. At one mixture it returns 2.91 s,
  the number §48 retired from the calibration gate. Across the sweep the
  apparent period runs 2.46 to 9.87 s: it settles nowhere, which is what §35's
  *range across ratios* actually was.
- ~~Status is "available and strongly supported", not proven — the test shows
  the identifier *can* manufacture the band, not that it did. **To close it
  properly:** project the real departure trace onto the two known eigenvectors
  and measure the residual. A merge of two test binaries, not new physics.~~
  **DONE (§50), and it closed in the direction §49 expected.**
- **The projection ran (`pitch_eigenmodes --project`) and the residual is a few
  per cent.** Two runs at the same ratio from the same settled state, one kicked
  by 2° of pitch, differenced — the same construction Φ itself was built from —
  then resolved on the eigenvectors. What the flown deviation carries outside
  the span of 1.86 s and 15.4 s is **0.4–5%** of its norm while the motion is
  small, at T = 0.25 and T = 0.10 alike. A six-state linear system has nowhere
  else to put a third mode.
- **And the counter mis-reads the two-mode rebuild exactly as it mis-reads the
  trace:** 6.34 s on the flown signal, 7.80 s on that same signal rebuilt from
  the two modes alone. Both in the gap, neither a mode. §49 showed the band was
  *reachable* from two modes; this shows it comes out of the real trace and out
  of a two-mode reconstruction of the real trace, alike. **§35's mode is closed,
  not merely retired.**
- **What `--project` does NOT support, stated because the first version of it
  overclaimed:** the growth *rate*. About 0.35's shared trim the reference run
  drifts and ratio 0.30 came back +0.0017 /s against an eigenvalue of −0.0084 —
  a sign disagreement bought entirely by the reference not being stationary.
  About each ratio's own trim it is −0.0007 against −0.0084, right sign, tenth
  of the size, on 0.05–0.13° of deviation near the differencing floor. At 0.25
  there is **no own trim at all** — it departs through 20° at 348 s of its own
  settle — so the departing case must keep the drifting reference. The rate
  belongs to `--phugoid` and `--sweep`; the span is what this measures.
- The residual climbing to 13–29% past ~3° of deviation is the linearisation
  running out, not a missing mode, and the window is printed rather than
  trimmed. The t = 0 row near 1.00 is the kick sitting on the two fast *real*
  roots (−0.97, −5.97 /s), which die within seconds; the statistic starts at
  25 s for that reason and the table still prints from zero.
- **The pitch axis now has no unexplained observations.** What is left is an
  absence rather than a mystery: no *mechanism* for why 0.35 is needed, only a
  measured sensitivity (`d(surge)/d(surge)`, +1.63 per unit) and a list of
  eliminated candidates. That is a different kind of open than the one just
  closed.
- **Method lesson, paid for three times now (§36, §48, §49):** a crossing- or
  peak-counting identifier on a signal holding two modes an order of magnitude
  apart reports neither, and does not fail — it returns a confident intermediate
  number that moves with the mixture and the window. When a period lands between
  two known modes and moves with conditions, suspect the identifier.
- ~~**Still unclaimed either way:** the canopy-referenced link damper (the
  solver's own rejected alternative). Differencing its matrix would test whether
  the two failure modes are separable, but it needs a solver hook that does not
  exist and was not added for a test.~~ **CLAIMED (§51).** The hook exists
  (`SetLinkDampingReference`, default bit-identical to before) and
  `pitch_eigenmodes --damper` is the measurement.
- **The two failure modes are ONE failure, and it is the phugoid.** The canopy
  damper's fast mode does not go unstable — it gets *more* damped, −0.31 → −0.68
  at ratio 0.35 — and what diverges is the same 16 s mode, at σ **+0.156**
  against the world damper's −0.021. Control: articulation 0.22 vs 0.29 slow,
  1.08 vs 1.07 fast, so the shapes and roles are unchanged and this is not a
  relabelled mode. The stated prediction — that the fast mode would go, since
  "the pendulum is dragged by the wing" is a pendulum-mode claim — **failed**.
- **And the coefficient's sign of effect reverses in that frame:** σ rises with
  damping, +0.136 at ratio 0.10 to +0.194 at 0.90, unstable at every ratio.
  **No value of this coefficient stabilises the aircraft in the canopy frame.**
  Flown, cold: 0.90 departs at 17 s, 0.35 at 27 s, against a world-damped 0.35
  settling at 410 s — more damping departing sooner, which no world-referenced
  run does anywhere.
- **What that buys is a constraint on the missing mechanism, not a candidate
  for it:** whatever stabilises this wing cannot be link–canopy friction at any
  magnitude. What the world damper supplies is a rate against the *inertial*
  frame, and the phugoid needs that. The candidate the solver itself nominated
  is eliminated.
- Caveat, the same one §50 carries: the canopy spectra are linearised about the
  *world* solver's 0.35 trim, because the canopy solver has no trim — it departs
  cold at every ratio. The flown column is the part that stands alone, and the
  two agree in sign and ordering.
- **Open, recorded before it is explained:** as the ratio falls the link
  articulates *less* against the wing (0.383 → 0.266), not more. Less link
  damping does not mean a freer-swinging link inside this mode.
- **Method note (`PHYSICS_LEARNINGS` §41):** this level's stated control failed
  *undecidably* — "the code is broken" and "my expectation was wrong" both
  predicted the observed number. It was fixed by adding a physics-free residual
  (1e-16 to 1e-12: eigenvectors exact) and a scaling-free articulation ratio
  (fast 1.07, slow 0.29: control passes), not by reinterpreting the number after
  seeing it.
- ~~**Also open:** §35 measured the growing mode at 3.6–5.7 s on a departing
  wing, and no such mode is in this spectrum at any ratio. Large amplitude,
  outside what a linearisation claims — not a contradiction, not reconciled
  either.~~ **Closed by §49 and §50:** it was the peak counter, and the flown
  trace is in the span of the two known modes to a few per cent.
- **The departure is not a threshold crossing (§77).** `pitch_eigenmodes
  --trough` prints the CL trough of every phugoid cycle. Ratio 0.20, on clean
  14.8–17.8 s spacing: **0.481, 0.442, 0.380, 0.287, 0.014**. The wing flies
  *through* §75's static edge (0.425–0.461) and keeps flying, and the terminal
  cycle goes past §76's transient edge (0.18) without resolving near it.
  **Neither edge is the criterion.** The excursion grows ×1.6 per cycle for four
  cycles and then the fifth overshoots that geometry — the departure is where
  the growth stops being exponential.
- That was predicted in advance for §53's reason: amplification G = 13.9 under
  decaying modes says the behaviour is not a function of any scalar. §76's
  factor of 2.4 between the two edges was never two estimates of one quantity.
- **The close-spaced troughs are ripple, and the control says so.** Ratio 0.25
  shows 415 troughs spaced at 0.1 s — the aerodynamic hold exactly. Raising the
  hysteresis 0.002 → 0.030 takes them 415 → 91 → 23 while the phugoid count
  holds at 22–24 and the departure stays at 470 s to the second. Not a finding,
  and checked before the number was read rather than after (§36, §48, §49).
- **THE BULLET ABOVE SAYING "It is not the schedule" DOES NOT COVER THE
  BOUNDARY.** It was measured on a *settling* wing at ratio 0.35 — there is no
  departure in it. `--trough`'s schedule sweep, 1800 s, is the first time the
  stability boundary has been tested against the aerodynamic hold:

| ratio | hold 0.05 s | hold 0.10 s | hold 0.20 s |
|---|---|---|---|
| 0.35 | settled, CL 0.511 | settled, CL 0.507 | settled, CL 0.497 |
| 0.30 | moving, CL 0.508, 0.69° | moving, CL 0.386, 5.9° | moving, CL 0.303, 10.2° |
| 0.25 | moving, CL 0.224, 6.9° | DEPARTED 470 s | DEPARTED 163 s |
| 0.20 | DEPARTED 96 s | DEPARTED 92 s | DEPARTED 94 s |

- **The stability classes are schedule-invariant and the instability is real.**
  0.35 settles at every hold; **0.20 departs at 96 / 92 / 94 s across a fourfold
  change in the hold**, which is the cleanest control row here.
  `swingDampingRatio` = 0.35 is not an artefact.
- **The RATE near the boundary is not invariant.** Ratio 0.30's excursion at
  1800 s runs CL 0.508 → 0.386 → 0.303 as the hold coarsens — nearly settled to
  strongly growing. **§39 located the boundary between 0.35 and 0.30 on 0.30
  failing to settle inside a budget, and that is exactly the quantity that
  moves.** The location has been quoted to two digits for several levels and
  should not be, until it is re-derived at a stated hold.
- **A window statement of mine was corrected by its own instrument, and it is
  on the record because it nearly became the headline.** Over 900 s, ratio 0.25
  "did not depart" at the 0.05 s hold. At 1800 s it is at CL 0.224 with a 6.9°
  spread against a trim CL of 0.54 — not stable, slower. The lowest-CL column
  was put in the table before the run precisely so "still moving" could not be
  read as "settled". Fourth instance of §73/§74/§75's pattern; the only one that
  cost nothing.
- **DONE, and it moved the boundary (§78).** `pitch_eigenmodes --growth`
  measures the per-cycle growth factor `g` of peak-to-trough amplitude. No trim
  in it (0.30 and 0.25 have none), no window, no linearisation, no settle
  budget — and it exists at stable ratios, where it sits below 1.

| ratio | hold 0.025 s | 0.05 s | 0.10 s | 0.20 s |
|---|---|---|---|---|
| 0.35 | 0.788 | 0.802 | 0.830 | 0.887 |
| 0.30 | **0.977** | **0.994** | **1.029** | **1.100** |
| 0.25 | — | 1.26 | 1.30 | 1.39 |
| 0.20 | — | 1.61 | 1.66 | 1.78 |

- **It is a real quantity:** ratio 0.30 returns 0.99 fifty-four cycles running.
  The first continuous measurement on this axis.
- **The control it was built for failed:** `g` is *not* schedule-invariant. The
  intended use — carrying the boundary where the settle time cannot — is dead.
- **Where the movement lands is the finding: ratio 0.30 straddles 1.0.** Stable
  at the 0.025 s and 0.05 s holds, growing at 0.10 s and 0.20 s. §77 showed the
  settle *time* moves with the schedule; this shows the **sign of the growth
  rate** moves, at the ratio that decides the boundary. **Ratio 0.30 is stable
  at holds of 0.05 s and finer, measured directly, no extrapolation.**
- **The extrapolation is earned here where §46's was not.** First-cycle
  increments double for every doubling of the hold — first order in *h*,
  established rather than assumed — and Richardson from three independent pairs
  gives 0.774 (ratio 0.35) and 0.959 (0.30), agreeing to 0.002 across an
  eightfold range of *h*.
- **So part of why 0.35 is required is the 10 Hz hold, not the aircraft.** Not
  all of it: 0.20 departs at every hold and 0.35 is stable at every hold, so the
  instability is real. But **§39's boundary between 0.35 and 0.30 is not a
  property of the wing** — at a converged hold the boundary is below 0.30.
- **A method note that cost a wrong sentence.** The first reading used the
  *mean* `g` and concluded the sequence was not converging. At the 0.20 s hold
  the mean is dragged from a first-cycle 1.100 to 1.041 by nonlinear decline
  over 51 cycles. The small-amplitude growth factor is the FIRST cycles; where
  mean and first differ, the amplitude has grown enough for the nonlinearity to
  bite, which is §77's effect and a finding rather than a defect.
- **Recorded, NOT claimed:** the converged boundary sits near 0.28–0.29, which
  is where the *eigenvalue* put it (0.28–0.25) and where §39 ruled against it
  because the flown answer said 0.35–0.30. The flown answer was taken at the
  10 Hz hold. The eigenvalues were too, so there is no reason yet why they
  should agree — §40 is what stops this being written up as a result.
- **DONE (§79), and §39's verdict inverted.** `pitch_eigenmodes --holds` takes
  the spectrum through the boundary at four aerodynamic holds, converted to
  §78's per-cycle currency so the two instruments print in the same units.

| ratio | 0.025 s | 0.05 s | 0.10 s | 0.20 s | spread |
|---|---|---|---|---|---|
| 0.35 | 0.710 | 0.709 | 0.716 | 0.708 | 1.1% |
| 0.30 | 0.881 | 0.880 | 0.887 | 0.876 | 1.3% |
| 0.28 | 0.966 | 0.965 | 0.972 | 0.961 | 1.1% |
| 0.25 | 1.117 | 1.116 | 1.124 | 1.110 | 1.3% |

- **The eigenvalue does not move; the flown measurement does.** 1.1–1.3%
  non-monotone scatter against the flown `g`'s 12% monotone move at ratio 0.30.
  **The eigenvalue crossing is between 0.28 and 0.25 at every hold.**
- **So `swingDampingRatio` = 0.35 was pinned partly by a schedule-contaminated
  instrument.** §39's outside confirmation (`pitch_axis_trace --slow-mode`) was
  taken at the same 0.1 s hold as the flown fit it agreed with — two instruments
  sharing a defect agreed with each other, and it was read as confirmation.
- **What §39 got right:** the eigenvalue *is* biased stable — 0.711 vs the
  converged flown 0.774 at ratio 0.35, 0.881 vs 0.959 at 0.30. The
  characterisation was correct; the verdict about which instrument to trust was
  backwards. **The converged flown boundary is between 0.30 and 0.28**, which
  needs no correction factor: at a converged hold ratio 0.30 decays at 0.959.
- **Recorded, NOT claimed:** converged-flown-over-eigenvalue is 1.0890 at ratio
  0.35 and 1.0885 at 0.30 — the same factor to four digits, at two ratios whose
  `g` differ by 24%. Two points, not predicted in advance, and the only two
  ratios where a converged flown `g` exists. Test is more ratios.
- **§47's attribution is REFUTED, and it came free with this run.** §47 blamed
  the 0.1 s aerodynamic hold for Φ(T) not being an exponential family. That
  predicts the T-dependence shrinks at a finer hold. Spread between σ at
  T = 0.25 and T = 0.10, ratio 0.35: **0.0038 / 0.0041 / 0.0044 / 0.0043** over
  holds 0.025 → 0.20 s. An eightfold finer hold leaves it within 12% of where it
  started. §47's *observation* stands; its *diagnosis* does not.
- **THE LAST NAMED CANDIDATE IS ELIMINATED (§80), AND ITEM 11 NOW HAS NONE.**
  `pitch_eigenmodes --stiffness` and `--soft` test whether n's *value* sets the
  boundary's *location* — a question §35 and §38/§40 never asked, since both
  were about n as the crossing variable.

| spring | glide | g at 0.35 | 0.30 | boundary |
|---|---|---|---|---|
| ×1 | 10.97 | 0.788 | 0.977 | ~0.28 |
| ×2 | 10.49 | 0.985 | 1.225 dep. | ~0.32 |
| ×4 | 6.52 | 1.001 | departed | ≥0.35 |
| ×0.5 | **0.11** | departed | departed | not flying |
| ×0.25 | **0.71** | no amplitude | — | not flying |

- **Stiffening moves the boundary UP** — sign reversed against the prediction,
  §55's shape for the third time on this axis. **Softening destroys the trim**
  (glide 0.11 against a published 9.5) before it can inform on anything, so the
  lever is one-sided and §55's confound withheld the licence. Eliminated in
  both directions.
- **So the ~0.18 of unexplained coefficient has no candidate mechanism**, and
  item 11's exit criterion remains unreachable as written: at ratio 0.06 this
  wing departs. **That is now a scoping decision rather than a physics question**
  — see the note under "what it did not close".
- **Kept from the run:** the shipped 10 Hz hold makes the ×2/×4/×8 springs
  depart at 329/202/113 s reaching 20–25° incidence, and *none* of them departs
  at 0.025 s — the third distinct occasion (§78, §79, this) that the schedule
  manufactures a departure not in the aircraft. And a trim-free,
  schedule-converged boundary curve that did not exist before: ratio 0.35 →
  0.788, 0.30 → 0.977, 0.25 → 1.235, 0.20 → 1.586, 0.15 → 2.132.
- **Unresolved and now un-leverable:** §34 reads the flat lift curve as the
  defect; §35 found dα/dV going −1.27 → −1.72 as the ratio *rises*, i.e. more
  tracking with more stability. They point opposite ways, have done for dozens
  of sections, and the lever that would have separated them is the one just
  eliminated.
- **Next, two candidates and they are separable.** (1) The **120 Hz base
  timestep** — the one schedule quantity that has never moved, held fixed in
  every run here and in §47. Vary `timeStepS` and the T-spread either follows or
  it does not; `CoupledSchedule::timeStepS` already exists. (2) **Hidden
  states**, which would make this structural rather than numerical: the solver
  carries membrane, pressure, line-network and damping-probe state outside the
  six reduced coordinates, and a 6×6 Φ(T) read off a higher-dimensional system
  is a projection — generically *not* an exponential family, as a matter of
  mathematics rather than discretisation. If (2), no schedule refinement fixes
  it, §47's advice ("quote it with its T") was right for the wrong reason, and
  §50's residual outside the two-mode span is the same thing from another side.
- **THE OLD EXIT CRITERION WAS UNREACHABLE AND IS REPLACED (§80).** It read:
  *"the wing settles in a time a pilot would recognise with a damping ratio
  derived from pilot and line drag (~0.06) rather than chosen to keep the
  aircraft from departing."* At ratio 0.06 **this wing departs** — the
  schedule-converged boundary is between 0.30 and 0.28 (§79), and the trim-free
  curve at 0.15 is already growing at 2.132 per cycle. So the criterion asked
  the item to close at a ratio five times below where a trim exists, which no
  amount of narrowing could deliver: the item could only ever recede. It has
  receded through four sections, and that is the criterion's fault rather than
  the aircraft's.
- **What replaced it, and why it is weaker on purpose.** After §80 there is no
  named candidate mechanism for the ~0.18 of unexplained coefficient, so an
  exit that requires *deriving* the ratio is an exit conditioned on an
  unstarted discovery. What can honestly be asked is that the number be
  **bounded, attributed and gated** rather than derived — and if it is later
  derived, that is a bonus, not the gate.
- Done when, all three:
  1. **The boundary is quoted at a converged schedule with its instrument
     named.** Met as of §79/§80: 0.30–0.28 flown at the 0.05 s hold, with the
     eigenvalue's 0.28–0.25 recorded as biased stable and *not* averaged in.
  2. **`swingDampingRatio` is labelled for what it is** in the coefficient
     registry. Met as of §80 — the entry now says margin-above-a-boundary
     rather than smallest-value-that-damps, and carries the superseded 10 Hz
     reading as superseded — a stability margin above a measured departure boundary, not a
     derived damping — with the gap to the ~0.06 that pilot and line drag imply
     recorded as an open disagreement of known size (~0.18) and no candidate
     mechanism.
  3. **The margin itself is chosen against the boundary rather than inherited.**
     0.35 sits ~0.06 above a boundary of 0.29. Whether that is the right margin
     is a scoping call — state it, gate it in `calibration_tests` in the
     direction the model is wrong, and it can regress noticeably.
- **What this deliberately does NOT close.** The ~0.18 stays open and stays
  attributed to nothing. §34 and §35 still point opposite ways about the flat
  lift curve, and the lever that would have separated them is eliminated. The
  next instrument, if there is one, is the 120 Hz base timestep or the hidden
  states above — **and neither is worth a fifth run before the wing has been
  flown** (item 19). Real handling data is now more likely to name the missing
  mechanism than another sweep is.

**Two corrections on the record, because both were written into these docs and
acted on before being checked.**

*First:* that brake slows the wing while LOWERING its incidence — a sign error
blamed on the suspension. It was read from 60-second runs whose own spread was
0.6–2.3°, larger than the 0.5–1.8° differences it rested on. Settled, the sign
is right. See `PHYSICS_LEARNINGS` §33.

*Second:* that this is a **limit cycle** driven by the pendulum's tracking lag,
with `swingDampingRatio` suppressing it. Also wrong, and wrong in the same way
— it came from reading a fixed-time spread as an amplitude. The damping sweep
that "proved" it (2.68° at 0.25, 0.60 at 0.35, 0.04 at 0.90) is real, but it
measures **how fast the mode decays**, not how big a cycle it sustains. More
damping settles sooner; at a fixed 60-second sample that looks identical to a
smaller cycle, and it is not the same thing.

*Third, and this one stands:* the suspension's specific stiffness of 6.13 m is
not a lever. It is registered *Validated*, measured off the built graph at four
loads, and is not an input to the solver — `LineStiffnessAt` interpolates the
measured curve and 6.13 is that curve's slope written down afterwards.

### What the pitch-axis programme closed, and what it did not

> **Renamed to end a collision (§68).** This section used to be headed "What
> Level 11 closed", and Level 11 in the master plan is the **unsteady wake**,
> which is unstarted. A reader following "the fix is Level 11" from the
> collapse-symmetry gate landed here, found a level marked finished, and would
> reasonably conclude the fix had shipped. Level numbers are unchanged; only
> this heading and the sentence below it are.

The pitch-axis programme was the pitch axis by linearisation, run under the
name "Level 11" while it went. It is finished in the sense that
every *observation* on this axis is now accounted for; it is not finished in the
sense item 11 asks for, and the two are worth keeping apart.

**Built:** `parapenting_pitch_eigenmodes`, which perturbs the settled aircraft
state by state, differences against an unperturbed run, and takes the
eigenvalues of the transition matrix — every longitudinal mode at once, no
excitation, no window, no filter. Its sub-checks are `--sweep`, `--phugoid`,
`--shape`, `--project`, `--amplitude`, `--drag` and `--height`, and each is a
question the time
domain could not answer. `--amplitude` is the exception that proves the shape
of the level: it is the one sub-check that is *not* a linearisation about trim,
which is why it could retire a hypothesis none of the others could reach.

**Closed by it:**

- **The spectrum.** 1.86 s at ζ 0.09 (pendulum) and 16.4 s at ζ 0.033
  (phugoid), the latter checked against an independent 1200 s trace's 16.39 s
  and 0.031. Nothing else in the longitudinal plane.
- **Which mode departs.** Not the pendulum: its real part moves −0.357 to
  −0.291 /s from ratio 0.90 to 0.10 and never crosses. The **phugoid** crosses,
  by its damping, between 0.35 and 0.30 flown. §38–§39.
- **What 0.35 is.** Approximately the smallest ratio at which this wing's
  phugoid still damps — an explanation, not a margin above a departure. The
  own-trim runs agree without fitting: 0.35 settles at 410 s, 0.30 does not
  settle in 420 s, 0.25 departs during its own settle. §39.
- **How a link coefficient reaches a speed mode.** The phugoid's adjoint is
  0.78 on the link's two rows at conditioning 0.10 — it *looks* like a speed
  oscillation and *listens* through the link. Non-normality is the channel.
  §42–§43.
- **§35's 3.6–5.7 s third mode**, retired as a peak-counter artefact (§49) and
  then closed outright: the flown departure trace lies in the span of the two
  known modes to 0.4–5%, and §35's counter mis-reads a two-mode *rebuild* of
  that same trace into the gap exactly as it mis-reads the trace. §50.
- **The 2.91 s / 0.28 pitch mode the calibration gate published for eight
  levels**, shown to be an artefact of the identification window and replaced
  by a gate that asserts what the record supports. §48.

**Four things it retracted, which is the other half of the level:** §34's
damping formula as a mechanism (right at a point, anti-correlated as a function
— §40); the tracking-lag version of the coupling (the phase holds within 1.9°
while σ crosses zero — §41); the 0.985 adjoint share (unit-flattered, 0.78
rescaled — §43); and the "fragility is a property of the link's formulation"
reframing (the largest sensitivity in the matrix is a *wing* entry — §45).

- **The canopy-referenced damper**, rejected four levels ago on a departure and
  never measured. It fails through the *same* mode (the phugoid, σ +0.156), its
  pendulum is *more* damped, and the coefficient's sign of effect reverses so
  that no value of it stabilises the aircraft in that frame. Link–canopy
  friction is therefore eliminated as the missing mechanism at any magnitude.
  §51.

- **Transient growth is real and is not the mechanism (§53).** The first
  question on this axis that eigenvalues cannot answer: this aircraft amplifies
  a disturbance **tenfold in half a second while every mode decays** (G 9.0–13.9
  against an eigenvalue prediction of 1.00), and the control holds at two
  transition times so it is the aircraft rather than §47's sampling. But G
  *falls* as the ratio drops, 13.9 → 9.0, so it does not explain the boundary
  and §39's flown-versus-eigenvalue anomaly stands.
- **The pattern is three for three and it is the live lead:** articulation
  0.383 → 0.266 (§41), receptivity 0.89 → 0.76 (§43), amplification 13.9 → 9.0
  (§53). Every measure of link–wing interaction falls as the aircraft
  destabilises. The mechanism is in the link's own dynamics through a weakening
  channel, and no measurement of coupling strength will find it.

- **The basin does not shrink, it grows, and 0.30 has no trim at all (§54).**
  The last non-linear explanation available for §39's flown-versus-eigenvalue
  gap, and the one `SettleAt`'s own comment named as fork (b): the trim is
  stable to small disturbances and the wing leaves when something large enough
  happens. Tested twice, by instruments sharing no arithmetic, and both fail in
  the same direction. The phugoid's real part **falls** with displacement
  amplitude (−0.0201 to −0.0508 /s at ratio 0.35, ζ 0.053 to 0.25), and the
  smallest kick that departs **rises** as the ratio falls — 4.5 m/s of surge at
  0.90 and 0.50, 6.5 at 0.35, on a 10.5 m/s trim. Finite amplitude is
  eliminated. Two by-products worth more than the null result: the basin edge
  is **nose-down** (departure incidence −2.6, −2.5, −5.8° against a 5.1° trim),
  so the disturbance limit is the same low-CL loop-gain path as full bar and
  40% brake rather than a stall — the brake travel and turn rate listed below
  may share one cause with it. And ratio 0.30 does not settle in **3600 s** and
  does not depart, with its drift larger at 3600 s than at 900 (0.681 against
  0.148 m/s per second), which retires a guess made in the same pass: §39's
  flown boundary is not a settling-budget artefact.

- **The missing drag destabilises the wing (§55).** The first time items 11 and
  12 were run together. A phugoid is classically damped by `CD/(CL√2)`, so this
  wing's sixth-too-far glide should mean a sixth-too-little damping and
  `swingDampingRatio` could have been paying for it. Restored by bisection
  against the published glide — Δcd 0.01035, glide 10.97 → 9.49 — and **σ at
  ratio 0.35 falls from −0.0201 to −0.0159 where it was predicted to rise to
  −0.023**, while 0.30 goes from "does not settle in an hour" to departing
  during its own settle. The size estimate was right and the sign was not: drag
  is worth about 0.02 of coefficient against the 0.29 needed, and it costs
  rather than pays. **The forecast this leaves is worth more than the null
  result: closing item 12 should be expected to LOSE pitch stability and to
  need more artificial damping, not less.** The lead it opens is the next test
  on this item — the classical relation assumes drag at the centre of gravity,
  and here it acts at the canopy 6.6 m above the pilot, so put the same extra
  drag on the harness (`InstalledDragSpec.harnessDragCoefficient`) instead and
  see whether σ reverses. If it does, the destabilising quantity is the moment
  arm rather than the drag.

- **THE MOMENT ARM, NOT THE DRAG — and the boundary moves one full step (§56).**
  The first mechanism in three levels to move the boundary the right way. Same
  aircraft, same total drag (both bisected to the published 9.5 glide), applied
  at the two ends of the link: at the canopy σ at ratio 0.35 goes −0.0201 →
  −0.0159, at the **harness** it goes to **−0.0328**, 63% better damped. The
  harness wing then **settles at 0.30** with σ −0.0200 — the clean wing's own
  value at 0.35 — does not settle at 0.25 and departs at 0.20, so its boundary
  is **0.30–0.25 against the clean 0.35–0.30.** That is 0.05 of the 0.29 this
  item needs, about a sixth, and it is physics rather than a coefficient. It is
  emphatically not the whole answer.
  - The control: the harness figure bisected to 0.199 m² of Cd·A against an
    independent equal-force estimate of 0.279, and the harness wing trims at
    10.64 m/s and 4.84° against the clean wing's 10.60 and 4.92°, so the two
    differ in where the drag acts and hardly at all in where they trim.
  - **The mechanism sketch that came with it is RETRACTED (§57), and the
    measurement is not.** `--phase` ran it: both drag wings' link-swing phase
    moves the *same* way and by a few degrees, against a σ that moves ±60%, at
    two transition times with converged residuals. Fifth retraction on this
    axis.
  - **What replaced it is a sharper constraint.** The harness wing damps 63%
    harder with a phugoid mode shape *indistinguishable* from the clean wing's —
    link/speed 0.4609 against 0.4587, articulation 0.290 against 0.292 — while
    the canopy wing is the one that restructures the mode (link/speed +19%,
    articulation −14%) and damps less. Whatever the harness drag does, it does
    it **inside** the mode rather than to it, so eleven levels of looking for a
    link mechanism were looking at the wrong object. Articulation also tracks
    stability for a fourth time, now on a second axis (§53 had three, all on
    ratio).
  - **σ IS LINEAR IN THE HEIGHT THE DRAG ACTS AT, and that height is what this
    coefficient has been standing in for (§58).** The arm sweep as named was not
    available — `harnessBelowCanopyM` is the pendulum's own geometry and moving
    it moves three things — so the sweep is the *share* of the extra drag that
    pushes the bob, with the rest applied at the canopy where its arm is zero,
    every row re-bisected to the published glide. σ runs −0.0328 → −0.0280 →
    −0.0240 → −0.0196 → −0.0160 from all-at-pilot to none, monotone and nearly
    linear, a factor of 2.05 at constant glide. The zero end lands at −0.0160
    against §55's independently-constructed canopy wing at −0.0159.
  - **This item's own opening estimate is the mechanism.** It has said from the
    start that the ratio estimated from what physically damps the swing — the
    pilot's drag on an 8 m arm plus the lines sweeping — is nearer 0.06. §58
    measures that from the other end: drag at the bob damps the phugoid, drag at
    the canopy does not. The pilot's drag is not an analogue of
    `swingDampingRatio`, it is the physical quantity the coefficient replaces,
    and the model carries too little of it — which is item 12 at the harness
    (§56). The *form* of item 11's answer is now known; the *magnitude* is not.
    The glide-landing amount buys 0.05 of the 0.29.
  - **THE MAGNITUDE, MEASURED (§59): a sixth, and the rest is elsewhere.** Drag
    at the bob reaches this item's honest 0.06 — but at **3.2 m² of extra drag
    area, ten times the modelled pilot, on a wing gliding 2.86 against a
    published 9.5.** At a credible pilot drag (0.199 m², the only row that still
    glides 9.51) it buys 0.05 of the 0.29. Each doubling of pilot drag buys
    about another 0.05. So "the installed drag is too low" is eliminated as the
    *whole* answer, quantitatively.
  - **THE TERM THIS ITEM NAMES IS NOT IN THE MODEL (§59).** The estimate at the
    top of this item is *the pilot's drag on an 8 m arm, plus the lines
    sweeping*, and both are **swing dampers** — they oppose the pilot's motion
    relative to the air, which on a swinging pendulum includes L·q̇. The solver
    has neither. `harnessDragBody` is built from the **aircraft's** airspeed and
    so cannot oppose the swing; the lines' drag becomes a canopy moment and
    never reaches the bob. **Nothing aerodynamic in this solver is proportional
    to the swing rate — the only such term is `swingDampingRatio` itself**,
    which is precisely why it has been standing in since Level 3. What §§55–58
    measured is real and is a *different* mechanism: a speed-dependent force at
    the bob.
  - **THAT TERM IS NOW IN, AND IT SETTLED THE QUESTION AGAINST THIS ITEM
    (§60).** `SetHarnessDragReference(Pilot)` evaluates the pilot's drag against
    the pilot's own airflow — aircraft plus w×r, geometry the solver already
    had. It is worth 14% of σ, **0.01 of coefficient**, and no movement of the
    boundary, against a standing prediction of 0.29. The control column
    reproduced the eleven-level boundary exactly, so the measurement is sound.
    See the refutation under the estimate at the top of this item.
  - **THAT DEBT HAS BEEN PAID DOWN TO A SINGLE QUESTION (§61).** The flip was
    made and the full suite run: **green, zero failures**, and the numbers
    diffed against the pre-flip run. Glide 11.33 → 11.20 and sink 0.97 → 0.99
    move *toward* published; trim speed and incidence move slightly away; the
    brake-incidence disagreement **halves**, 0.73° → 0.49°; full bar left to
    settle ends at 45.9° instead of 83.4°. And the largest change of all is one
    no published number arbitrates: the **4 m/s symmetric collapse folds L
    0.999 R 1.000 where it folded 0.710/0.710** — the benchmark Level 8 and §13
    were tuned against. The term is small in trim and large in transients, and
    a collapse is nothing but transient.
  - **Not shipped, and deliberately.** Replacing a measured benchmark with an
    unmeasured one because the suite is green and the term is correct would be
    a judgement call made on the model's own authority. The flag stays,
    defaulted off, with both states measured — which is what makes deferring
    this honest rather than merely postponing it.
  - **One SIV answer decides it:** on a 4 m/s symmetric frontal, does an EN-B
    fold 70% or 100%? That belongs in `docs/PILOT_REVIEW_PROTOCOL.md` and in
    Level 9's outstanding external validation.
  - **THE PAIR IS NOW CLOSED — both named terms implemented and measured
    (§63).** The lines sweeping is a torque about the hinge with an s² weight,
    summed off the graph as `d·L·s²`. Measured, the lines are worth **10% of
    the pilot's term** against an estimated 7%: the arithmetic that dismissed
    them holds. Together the two terms move σ by 14%, **about 0.01 of
    coefficient against the 0.29**, so this item's founding estimate is now
    refuted on two measurements rather than one measurement and one argument.
  - **One refinement of §60, in the pair's favour:** at ratio 0.30 the two
    terms together *settle* where clean and either term alone do not, so on the
    0.35/0.30/0.25 grid the boundary does move one step. That flatters them —
    0.30 was already marginal (§54: neither settling in 3600 s nor departing, σ
    ≈ −0.008), so a 14% improvement tips a ratio sitting on the criterion and a
    coarse grid reports a whole step. Worth 0.01, not 0.05.
  - **Instrument limit, recorded because it bounds the smaller number:** the
    pair is not additive — pilot alone −0.0230, both −0.0229 — and the
    departure from additivity is *larger than the line term itself*. The order
    of the lines' contribution is solid; its second digit is not.

**What it did not close, and this is the whole of what remains:** there is no
*mechanism* for why 0.35 is needed. What exists is a well-measured sensitivity
— `d(surge)/d(surge)` at +1.63 per unit, worth 1.25 coefficient steps — and a
list of eliminated candidates. That is an absence rather than a mystery, and it
is a different kind of open than the ones above.

- **THE COEFFICIENT BUYS EXCITATION, NOT ROBUSTNESS (§76).** The one candidate
  §75 opened — that 0.35 is simply the least damping keeping the growing
  phugoid's CL trough inside the static envelope, so no mechanism need exist —
  is **refuted**, and what replaced it narrows the search.
  - Ratio 0.30 reaches **CL 0.425**, squarely at §75's static edge, and does
    not depart. Touching the edge is not what ends the flight.
  - Asked without a clock — from a settled trim, bisect the depth of a pure
    surge kick the wing comes back from — the transient recovery edge is
    **CL 0.18**, less than half the static edge. Static edge and point of no
    return are different numbers by a factor of 2.4.
  - **And the recovery edge does not move with the coefficient at all**:
    3.94 m/s of kick and a trough of 0.182 / 0.181 / 0.183 at ratios 0.90,
    0.50 and 0.35. A factor of 2.6 in the coefficient, no effect on the wing's
    tolerance.
  - **So a mechanism that made the wing more robust to low-CL transits is the
    wrong shape of answer.** `swingDampingRatio` does not change how much
    excursion the wing survives; it changes how much the phugoid produces.
    That is a constraint on the missing mechanism of the same kind as §51's,
    and it points at the phugoid's growth rather than the wing's margins.
  - **Owed to §54, with its limit stated:** §54 measured the basin along the
    *phugoid eigendirection* and found it growing (4.0, 4.0, 6.0 m/s). On a
    fixed direction it does not move. The eigendirection is itself a function
    of the ratio, so some of "the basin grows" is the direction rotating —
    though two directions were used rather than one direction read two ways,
    so this is a limit rather than a correction. §54's elimination of the
    finite-amplitude story stands either way.
  - Not claimed, recorded: the largest recovered kick places the wing at
    **CL 0.443** against §75's static edge of 0.425–0.461. One number, no
    prediction made in advance. §40's rule.

**Therefore the four loosened gates stay loosened** and the table above stands
as written: item 11 has not landed, and restoring the strict thresholds now
would make the suite green about a disagreement rather than honest about one.
The same applies to the brake-travel and turn-rate numbers listed with them.

**One candidate eliminated rather than left open,** which is the only change to
the shape of item 11 this level made: the missing mechanism cannot be
link–canopy friction. It has to act against the inertial frame. §51.

**One instrument limit worth carrying forward.** No rate measured through
`--project` is trustworthy at a departing ratio, because a ratio past the
stability boundary has no trim to linearise about — 0.25 leaves through 20° at
348 s of its own settle. Rates come from `--phugoid` and `--sweep`; `--project`
measures the span.

**11a. The pendulum damping ratio, as originally written.** Kept because the
reasoning below is still what has to be replaced, and only the diagnosis above
it changed.
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
  - **REFUTED BY MEASUREMENT (§60), and it is off by about twenty-five times.**
    The first of those two terms is now implemented — the pilot's drag
    evaluated against the pilot's own airflow, aircraft plus w×r — and it is
    worth **14% of σ and 0.01 of coefficient**, with the boundary not moving at
    all. The second cannot rescue it: line Cd·A is 0.098 against the harness's
    0.336, and the lines sweep at a velocity proportional to distance along the
    arm, so their damping torque carries an ∫s³ds/L⁴ = ¼ weighting — about 7%
    of a term already worth 0.01. **`swingDampingRatio` is not standing in for
    aerodynamic swing damping.** This sentence has framed the item since Level
    3 and every attempt on this axis inherited it.
  - What demonstrably moves the boundary is a *different* mechanism, found in
    §§56–58: the **magnitude of drag applied at the bob**, acting through the
    speed oscillation, σ linear in the height at which it acts. The item has
    been named after the wrong one of the two.
- Tried and rejected: damping the link against the CANOPY rather than the world,
  which is where the friction physically is. It leaves the pendulum free to be
  dragged by the wing and the aircraft left the envelope in twenty seconds.
- Done when: the wing holds full bar with the ratio at a value derived from
  pilot and line drag. The two levers on the loop gain were the section
  pitching moment (item 1) and the specific stiffness of 6.13 m.
- **One of the two has been measured and it is not the answer.** Item 1 is
  closed: the section moment is now solved rather than stated, it varies with
  incidence, and it is close to the old constant where it matters. Bar is
  better for it - the wing reaches **15.6 m/s, 56 km/h against a published 53**,
  before it lets go, where before it departed on the way there. But it still
  lets go, so what is left is the suspension side: the specific stiffness and
  the damping ratio.
- **This item also owns what is left of 40% brake.** An earlier reading of
  this said brake was making the wing FASTER over the first fifth of its
  travel. That was wrong and it is worth recording why: it was read off a
  ramp started before the wing had finished settling, so what it measured was
  the tail of the phugoid, not the brake. Settled properly at each setting,
  brake does what brake does - 10.48 m/s hands up, 10.24 at 20%, 9.76 at 25%,
  8.63 at 30% - and glide falls with it, and a firm input from trim climbs at
  1.15 m/s before settling slower. All three are now gated in `coupled_tests`.
  The rotation per unit brake was one of the two suspects here, and it has
  now been measured and fixed. It was **counted twice**: the line network
  shortened the brake main run by the whole 0.62 m of handle travel and
  rotated a rigid canopy with it, while the section polars spent that same
  travel again bending the trailing edge into camber. A brake line ends at 98%
  of chord, so the fabric it bends and the canopy it rotates are pulled
  through ONE length. Counted once - the take-up off the geometry, 2.315 m of
  mean chord at the four span stations the brake fan lands on - the 0.62 m
  divides as 0.120 slack, 0.298 fabric, 0.202 rotation, and full brake rotates
  the canopy 5.0 degrees where it used to rotate it 12.4.
- **That fix made the flying worse, and that is the finding.** The double
  count had been propping up a suspension that cannot otherwise produce the
  right SIGN. Brake now slows the wing while LOWERING its incidence - 4.4 deg
  at 25% against 5.14 at trim - and 40% departs nose-down through the same
  low-CL loop-gain path as full bar rather than by stalling. The section side
  is now measured on both counts: the pitching moment agrees with thin-airfoil
  flap theory to 10% (-0.61 per radian solved, about -0.55 from theory,
  against the analytic table's -0.34, which multiplied by the flap
  effectiveness a second time), and the take-up comes off the geometry. So
  this item is down to ONE unmeasured number: the specific stiffness of
  6.13 m.
- Registered Tuned/Unvalidated, superseded-by the pitch-axis programme (item
  11, not the master plan's Level 11), and bounded by the
  full-bar, deep-brake and brake-incidence gates in `calibration_tests` and
  the 25% brake and surge gates in `coupled_tests`.

**What to re-evaluate when item 11 lands.** These were loosened to keep the
suite honest about a disagreement rather than green about a fit. Each carries
its strict threshold in a comment beside it, so restoring it is a revert:

| where | now | restore to |
|---|---|---|
| `CalibrationTests.cpp`, 25% brake | incidence may drop up to 1.7 deg | `brake.settledIncidenceRad > trim.settledIncidenceRad` |
| `CoupledTests.cpp`, 25% brake | `clean.speed - 0.05` | `clean.speed - 0.3` |
| `CoupledTests.cpp`, surge endpoint | `trimLeadM + 0.3` | `trimLeadM + 0.5` |
| `CoupledTests.cpp`, surge rate | `< -0.02` | `< -0.05` |

Also re-evaluate then, because both are currently masked by the weak pitch
response rather than independently checked:

- the **brake travel at which the wing departs**, which is 40% against an
  EN-B's 65-80%. It is no longer a section stall, so the section polars are
  not what will move it.
- the **turn rate**, 0.031 rad/s at 0.4 deg of bank on 35% brake against an
  EN-B's 0.3 at 20-30 deg. Brake that cannot pitch the wing cannot bank it
  either, so this may be the same single cause and should not be chased
  separately until item 11 is closed.

One modelling refinement deliberately not taken, for whoever picks this up:
the take-up is subtracted from the brake line's REST LENGTH. Physically the
trailing edge itself moves down in the canopy frame, so the more faithful model
moves the attachment NODE and lets the line's direction and moment arm follow
it. The two agree to first order in the length budget, which is the dominant
term, and the node version is a larger change to the graph. It is worth doing
with item 11 rather than before it.

## Level 10 — performance and integration (in progress)

Strands 1 and 2 are done and are documented in `docs/SOLVER_PROFILE.md` and
`docs/SOLVER_LOD.md`. What follows is what those two left open. None of it is
blocking; all of it is the kind of thing that is obvious now and invisible in
six months.

**14. MOSTLY CLOSED. Construction was 1059 ms and is now 340 ms.** It was the
only measured cost in the solver a pilot would notice — per-step cost is 6.5%
of one core, against a second of stall to swap a wing.

- The section polar table is now cached: **723 ms to 4 ms**. See
  `docs/POLAR_CACHE.md`. The drift trap was the whole problem and it is solved
  by a WITNESS rather than by a version constant somebody has to remember to
  bump — one canonical cold solve stored in the file and re-solved on every
  load, so a changed viscous solver, boundary layer, profile geometry or
  panelling invalidates the cache even though every input is identical. Every
  failure path falls through to solving. All four refusals are gated in
  `aerodynamics_tests`.
- **What is left is 336 ms and it did not move.** That is the suspension
  network solving itself cold — trim load distribution, line stiffness curve,
  brake swing curve, about eleven 12000-iteration relaxations. It is not a
  table and cannot be cached the same way, because it depends on the line plan
  and the payload rather than on the section alone.
- ~~Next, and untried: warm-start each of those relaxations from the previous
  one. They are solves of the SAME network at neighbouring loads, and they are
  currently each started cold.~~ **Tried, and it is not the lever (§52).** The
  24 expensive solves are the stiffness probes, which IMPOSE an attitude
  0.02 rad from the hang pose — a warm start hands them the answer to a
  different question, and they converged no faster and less accurately. Warm
  starting the FREE brake sequence does help (1.44° → 0.16° of pose error at
  4000 iterations) but not enough to cut the count: matching the shipped
  accuracy still needs 12000, and a hundredth of a degree of incidence is worth
  0.83 against 0.30 of fold on this aircraft.
- **The lever is the damping, and the probe says so: the relaxation is RINGING,
  not converging.** Held at 0.02 rad the pitch probe passes +177%, −176%, +27%
  and −13% of its converged value at 500, 1000, 2000 and 4000 iterations. Fewer
  iterations at lower velocity retention is both faster and closer: held 8000 at
  0.997 retention lands every static output within **0.24%** of a
  48000-iteration reference where the shipped settings are **1.58%** out
  (roll spring, 8116 against 8254 Nm/rad), and costs 260 ms against 336.
- **It is measured, gated and NOT shipped, which is the finding.** Rebuilt on
  better-converged probes, two known-limitation gates change their verdict — and
  so does the 48000-iteration reference, which is what settles it:
  - the deep frontal's peak rotation, bounded at 2.4 rad/s: 2.06 shipped,
    **3.61 converged**, 3.90 and 71.0 at the two candidate settings. A factor
    of thirty-four across settings whose static outputs agree within 1.7%.
  - 40% brake departs nose-up at +91° shipped and converged, nose-DOWN at −90°
    at held 6000/0.995.
  So the suite is green partly because the mis-convergence damps an event the
  converged model does not damp. **A gate calibrated on a mis-converged model
  does not become wrong when the model improves; it becomes a decision nobody
  has taken** — and that decision is about two known-limitation events, not
  about load time, so it is not this item's to take.
- **What shipped:** the `ConstructionProbe` hook with defaults identical to the
  old behaviour, so the measurement is reproducible; a `suspension_tests` gate
  asserting the control (two relaxation paths converge on the same spring to
  0.16%, so the reference is a reference) and bounding the shipped error where
  it is so it cannot grow; and the numbers above written beside the constants.
- **Blocked, and precisely:** on deciding what the deep frontal's rotation bound
  means when the quantity it bounds is amplified numerical noise. `PHYSICS_TODO`
  item 6 and Level 11's unsteady wake are the real fix for that event.
- Done when: construction is under 100 ms. **Not met — it is 355 ms and the
  measured route to ~260 ms is the one above.** Note also that the free solves
  (the hang pose, the bar pose, the six brake stations) are bounded by an
  accuracy requirement rather than by effort: their iteration count buys
  hundredths of a degree of incidence, and this project has paid for those.

**15. The reduced tier converges a disturbance more slowly than the settled
numbers show.** Worst network residual over a run including the cold start is
28 N full against 83 N reduced — a 3× ratio where the settled ratio is 1.4×.

- Bounded, not unknown: the gust signature in `solver_lod` is measured through
  a real collapse and recovery, and `coupled_tests` gates the tier against the
  full solver on fold as well as trim.
- **Re-evaluate if the reduced tier is ever used for anything but frame rate.**
  It must not carry a published number, a Level 9 calibration, or a new
  gate — `FullFidelitySchedule` is the reference and a disagreement means a
  disagreement with it.
- The open question nobody has asked: whether `suspensionIterations` should be
  a function of how far the network moved last step rather than a constant.
  That would recover the transient without paying for it in cruise, and it is
  the only remaining idea here that is not just a smaller number.

**16. Research visualisation toggles are the unstarted Level 10 strand.** There
are two hard-coded bools in `ParagliderPawn` — airflow and geometry — each with
its own toggle and its own binding, and no general way to turn individual
solver outputs on and off. Item 5 (the collapse debug view) is a special case
and is separately blocked on item 7.

- **It cannot be the instrument for item 11, and an earlier note here claiming
  it could was wrong.** `ParagliderPawn` holds `ParagliderDynamics` and nothing
  else, so a view in the pawn shows the LEGACY model. The line network and the
  section moment live in the coupled solver, which the game does not run at
  all. Any in-engine instrument for the pitch axis is blocked behind item 17,
  which is itself blocked on item 11.
- The headless instrument that does work today is
  `parapenting_pitch_axis_trace`, and it is what found the limit cycle. Build
  the in-engine views for what they are actually for — inspecting a flight a
  pilot is having — not as a debugging route to item 11.

**17. Removing the legacy path is Level 10's exit gate and is BLOCKED.** See
item 7 for what it is and item 11 for why it cannot start. The exit gate is "no
legacy direct-control force remains active", and the geometry-driven stack
currently departs at 40% brake with brake lowering incidence. Swapping it under
`ParagliderPawn` today would be a regression a pilot would feel immediately.

- Note that the profile removed one excuse: the coupled solver is 15× faster
  than real time at full fidelity and 36× reduced, so performance was never
  the reason the game still flies `ParagliderDynamics`. Guiding rule 11 was,
  and now item 11 is.
- It is also not a switch flip: `ParagliderPawn` flies `ParagliderDynamics` and
  about twenty headers pull types from it. Measured since: the pawn makes **17
  calls** into `Dynamics`, and only **one** is the `Step`. The rest is
  telemetry reads and parameter setup, so the seam is narrower than the header
  count suggests and is not what blocks this.
- **THE BLOCKER HAS MOVED, and it is worse than it looked (§70).** Bisected,
  the geometry-driven stack flies to **37% of brake** (confirming the
  documented 40%), but only to **22% of speed bar** - half bar departs - and
  weight shift produces **0.01 rad/s**, which is nothing (items 21, 22). That
  is two of the four controls a paraglider has. A stated envelope could route
  around a deep-brake departure; it cannot describe an aircraft with no bar and
  no weight shift. **The stack can fly hands-up gliding to a third of brake -
  enough to compare against the legacy model, not enough to give a player.**

**18. Every calibration number was measured after too short a settle.** The
harness gives each manoeuvre **90 seconds** — already raised once from 15 for
exactly this reason — where hands-up needs 410 s and 25% brake needs 1080.

The arithmetic is the whole argument: the slow mode is 16.4 s at damping 0.031,
so 90 s is five and a half periods and leaves e^(−0.031·2π·5.5) ≈ **34% of the
opening transient still running**. And the `settled` flag does not catch it,
because holding airspeed to 1% over **two seconds** is a far looser test than
that mode is slow — two seconds inside a sixteen-second period is a chord of
the oscillation, not a measurement of it.

- **Credit where it is due, and a correction to my own earlier note here.** The
  slow mode was not undiscovered. `CalibrationManeuver.cpp` names it — "a slow
  speed-and-incidence mode near twenty" — records that fitting the whole
  45 s record locked onto it and reported "a 20.4 s pendulum with a damping
  ratio of 0.05, which is a true statement about the wrong mode", and windows
  the pitch identification to 2–9 s specifically to avoid it. What had not been
  done was connecting that mode to the *settle time*. The measurement in item
  11 (16.39 s, damping 0.031, off 27 peaks) sharpens a number that was already
  roughly right.

- The whole Level 9 table in `docs/CALIBRATION_REPORT.md` is therefore
  provisional, including the two published numbers that "land untuned" —
  trim speed and incidence. Those two are the hands-up case, which settles
  fastest, so they are the most likely to survive; that is a reason to expect
  them to hold, not a reason not to check.
- **Hands-up needs 410 s and the harness gives it 20.** Measured hands-up trim
  settles at 4.925° and 10.603 m/s. The brake rows need 460 s at 20%, 1080 s
  at 25%, and **more than 1200 s at 30% and beyond** — deep brake does not
  reach a trim point inside twenty minutes of flight, so every deep-brake
  number in the report is a snapshot of a transient, not a disagreement with
  the manufacturer.
- **Done: `CalibrationSettings::settleToCriterion`,** which flies windows until
  incidence holds 0.01° and airspeed 0.01 m/s over ten seconds, reports
  `actualSettleSeconds` and `preInputSettled`, and applies the same treatment
  to the record phase — a step input takes as long to settle as the trim does,
  so a fixed 45 s record averages an oscillation no matter how long the
  pre-input settle was. Off by default.
- **`parapenting_calibration_settled`** is the slow target that turns it on. It
  is deliberately NOT in `Tools/check-build.sh`: settling eight manoeuvres this
  way is the better part of an hour against minutes for the fast suite. The
  fast suite keeps its clock-based settle and its own gates; this says whether
  those gates point at trim points or at transients.
- **Measured. About a fifth of the drag disagreement was never drag.** Settled
  properly, hands-up glide is **10.96 against the clock's 11.33** and a
  published 9.5; sink is 1.015 against 0.97 and a published 1.14. Item 12 is
  smaller than it was written up as, and still real.
- **The two "landing untuned" numbers land slightly worse.** Trim speed goes
  from +2.1% over published to **+3.1%** (11.175 m/s, 40.2 km/h against 39),
  and incidence from 5.14° to **4.95°** against the 5.30° the published lift
  coefficient needs.
- **Two consistency checks now pass that could not before.** The brake pulse
  returns to exactly hands-up trim (11.173 against 11.175 m/s, 4.95° against
  4.95°) and weight shift leaves the speed alone (11.189). On the 90 s clock
  those read as different flight states, 11.56 and 11.09 — which was the
  transient, not the manoeuvre. That is the clearest evidence that the settled
  numbers are the real ones.
- Still open: 25% brake does not settle even given 1500 s after its input.
  Deep brake and full bar do reach a steady state, but a fully separated one at
  86–91° of incidence — a steady state, not a trim point.

## Level 11 — unsteady wake (STARTED, strand 1 landed, strand 2 landed and then re-scoped by item 30)

**Budget 36 hours, and it took four sessions to specify before any of it was
written.** The master plan's work list is four strands. They are not equally
blocked and they are not equally expensive, so this records the ordering and
the reason rather than leaving the next session to re-derive it.

**The entry criterion is a separated solve that is SINGLE-VALUED** (§68). Not a
stabler one and not a better-iterated one: 40, 200 and 600 iteration caps break
on the same tick, so the solve is not short of iterations — it has nothing to
converge to. `coupled_tests` already holds the gate that would show it fixed:
the symmetric frontal must not lose mirror symmetry on an aerodynamic tick, and
a drag correction landing the published glide must not engage the safety
envelope.

**26. Strand 1: unsteady circulation lag. DONE and validated.**

- `WagnerLag` in `VortexStepMethodSolver.h` — R. T. Jones' two-exponential
  approximation to Wagner's function, `Phi(s) = 1 - 0.165 e^(-0.0455 s) -
  0.335 e^(-0.30 s)`, in reduced time `s` = semichords travelled. Named source,
  per guiding rule 13; deviation from Wagner's exact function is under 1% and
  is the standard engineering form.
- Written as two first-order states rather than a convolution, which is what
  makes it affordable in a flight loop — the exponential form integrates
  exactly, so any `ds` is a closed-form step and there is no history to store.
- **Checked against published values rather than against itself**, marched
  rather than evaluated: 0.5000 / 0.5942 / 0.6655 / 0.7938 / 0.8786 / 0.9328 at
  s = 0, 1, 2, 5, 10, 20. Plus `Phi(0) = 0.5` exactly, monotonicity to s = 100,
  no transient from a settled start, and that reduced time counts semichords
  and not seconds. In `aerodynamics_tests`.
- **Two of those reference values were wrong when first written, and the code
  was right.** Worth recording: the failing check was the test's arithmetic,
  not the implementation, and the only reason that was visible in one step is
  that the reference was computed independently instead of captured from the
  code's own output. A golden-output test would have frozen the error.
- **Was not wired into the force assembly when this landed, and that was strand
  2 rather than an oversight — item 27 has since wired it.**
  The VSM builds section forces from the polar's lift coefficient and lets
  circulation enter only through the induced velocity. Lagging one without the
  other reports a wing whose lift and downwash disagree. The indicial response
  was landed alone because it is the part with a published answer to check
  against, and checking it inside a coupled solve would have been strictly
  harder for no benefit.

**27. Strand 2: wire the lag into the circulation solve. DONE, and the gate
passed.** Behind `VsmSettings::lagCirculation`, reached from the coupled solver
by `SetLagCirculation`. **Defaults OFF — the shipped aircraft does not fly on
this yet**, and turning it on is a decision that belongs with strand 3, not with
the measurement below.

- **The change is to the OUTER loop only.** The inner secant stays: a section's
  own trailing legs pass half a panel width from its control point, so the
  self-induced downwash has a gain of chord over panel width, and iterating it
  explicitly would make the answer depend on the mesh. That term is still
  implicit. What is dropped is the global fixed point ACROSS sections — the
  coupling the quasi-steady path already documents as the weak one — which
  becomes explicit in time, which is what a state is allowed to be.
- **The Γ↔Cl consistency is satisfied by construction rather than by
  agreement.** `Γ = ½ V c Cl`, so the force assembly reads the lift coefficient
  back OUT of the lagged circulation, `Cl = 2Γ/(cV)`, instead of sampling the
  polar a second time. The induced velocity uses the same `circulation` array,
  so lift and downwash cannot come from different instants.
- **Incidence is re-evaluated AT the lagged circulation**, not reported from the
  quasi-steady target. This is not cosmetic: the collapse model's
  `externalNoseCp` is a function of section incidence alone, so reporting the
  target's incidence hands the pressure margin a field belonging to a wing that
  does not exist — and that margin field is what the gate measures.
- **Kept Jacobi, deliberately.** Updating circulation in place would make the
  pass Gauss-Seidel, and a sweep running left to right gives one half-span an
  extra update — an order-dependent seed, in the one solve whose mirror
  symmetry is the gate.
- Drag and moment follow the lagged incidence but are **not themselves lagged**.
  Wagner's function is the indicial response of circulatory lift; there is no
  published indicial response for profile drag or the quarter-chord couple to
  check against, and inventing one would be a dial (guiding rule 13).
- `converged` reports false in this mode, because a state does not converge to
  anything on a tick. Safe: the coupled solver's rejection test reads
  finiteness and magnitude only, never `converged` — which is item 24 again.

**THE GATE, MEASURED.** The quasi-steady solve loses the margin field's mirror
symmetry at **t=1.350 s** and cannot be iterated out of it — caps of 40, 200 and
600 all break on that same tick, because the separated solve has nothing
single-valued to converge to. With the circulation carried as a state:

| | quasi-steady | lagged state |
|---|---|---|
| margin symmetry breaks | t=1.350 s | never |
| fold symmetry breaks | t=1.400 s | never |
| peak mirror residual | 7.23e-01 | **3.52e-14** |
| deepest fold | 1.000 | 0.998 |

It does not break at all, and it holds symmetry at round-off rather than merely
under the threshold. **This settles the question item 27 recorded rather than
assumed**: it was possible the lag would make the transient well posed while
branch selection stayed ambiguous, which would have shown as a break that moved
later or shrank without going away. It went away.

- **The wing still takes a real frontal**, and that guard is in the gate. A lag
  deep enough to stop the canopy folding would pass a symmetry test by deleting
  the event it is supposed to survive. It folds to 0.998 against the
  quasi-steady wing's 1.000 — the same collapse to three figures, measured in
  the same harness rather than compared against a number from a different one,
  which is why the fold-depth print was added to both blocks and not only the
  new one.
- **The seed is load-bearing and was wrong first.** A one-pass lagged solve
  cannot start from zero circulation: the quasi-steady path tolerates it because
  its outer loop iterates to the fixed point regardless of where it begins, but
  one pass from zero has every section computing its target with no other
  section's downwash, and `Settle` then adopts that over-lifted wing as the
  state rather than passing through it. Measured before the fix: symmetry lost
  at **t=0.000 s** with the canopy already fully collapsed. That is a seed
  transient and says nothing about the aerodynamics. The lagged path now seeds
  from a converged quasi-steady solve, and only the lagged path does — seeding
  unconditionally would turn the quasi-steady first solve from a cold start into
  a warm one, which converges to the same wing but not to the same bits.
- **The flag-off path is bit-identical to before the change**, checked rather
  than assumed: the whole coupled suite diffs to exactly the four new printed
  lines, with `1.87e-15` entering the fold and the 1.350/1.400 breaks unmoved.

**AND THE GATE ABOVE WAS MEASURED ON A LAG THAT IS NOT WAGNER'S. See item 30.**
Two separate reasons, both measured rather than argued: the aerodynamic states
were being advanced by the simulation step while the solve runs once per
`aerodynamicsInterval`, so at the shipped 6 this lag ran six times slow; and
even with that corrected, the composite one-pass-plus-Wagner response closes
12% of a circulation step where Jones' published Φ(0) is 0.5. The symmetry
result in this table is real and is not retracted — it was taken on this
solver, in this harness. What it does not establish is the claim the strand-3
note built on it, that the separated solve is single-valued under *Wagner's*
lag. Correcting only the arithmetic half of the excess takes the peak mirror
residual from **3.52e-14 to 4.04e-01**. **AND CORRECTING BOTH HALVES MOVES THE
BREAK TO t=0.050 s, twenty-seven times EARLIER than the quasi-steady wing's
1.350 s** — so the symmetry in this table came from the lag being too deep, and
item 27's headline is retracted. Item 30 carries the four-row measurement.

**What is NOT settled, and should not be read into the above.** The frozen and
still probes still call `SolveFrozen`, which passes no lag state and so stays
quasi-steady. The damping derivative is therefore measured on an unlagged wing
while the flying wing is lagged. That is deliberate for now — a probe is a
what-if about the same instant, and giving it its own marching lag state would
make it a second aircraft — but it is an inconsistency, and it is worth
measuring before it is worth fixing.

**28. Strand 3: shed and convect a free wake.** Trailing-edge vorticity shed and
convected, the canopy flying through its own wake, with adaptive near-field
lattice → far-field particle conversion so cost stays bounded. This is the
expensive strand and the one the 36-hour budget is mostly about.

**29. Strand 4: the exit gate's flight behaviours.** A spiral showing genuine
wake re-encounter on the inner wing; wingovers showing circulation lag rather
than instant response; a brake release with correct overshoot timing and no
tuned delay; wake cost inside the frame budget for 60 s of aggressive
manoeuvring. Strand 4 is mostly measurement once 2 and 3 are in.

- **Note for whoever takes strand 3, REWRITTEN BY ITEM 30 AND THE NEWS IS
  BAD.** This used to read: "strand 2 removed the entry criterion's blocker but
  did not turn itself on. The separated solve is single-valued under
  `lagCirculation` — that is what the symmetry result means — so §68's entry
  criterion is met by the lagged path and not by the shipped one." **That is
  not established.** The symmetry result was measured on a lag several times
  deeper than the one Wagner describes, for two separate reasons item 30
  measures, and with only the arithmetic half of that excess removed the
  frontal's mirror residual goes from `3.52e-14` back to `4.04e-01`.
  **Strand 3's entry criterion is NOT met — no longer "treat as open", it is
  measured.** With both corrections applied the frontal breaks at **0.050 s**
  against the quasi-steady wing's 1.350 s, so the correctly-lagged path is not
  merely failing to supply single-valuedness, it is worse than the path that
  ships. Strand 3 must not start on the strength of strand 2.
- *Historical, now closed:* this used to warn that item 25 was unresolved and
  the §69 control gate red at `aerodynamicsInterval` 6, so strand-2
  measurements would be read against a red benchmark. Both are settled, and the
  gate above was read against a green suite.

**30. THE SHIP QUESTION FOR STRAND 2, ASKED AND ANSWERED — AND IT FOUND TWO
DEFECTS ON THE WAY, ONE OF WHICH IS NOT STRAND 2'S.** Strand 2 landed the
lagged circulation defaulting OFF and left "whether it ships" to strand 3. The
gate it passed measures ONE collapse; turning the flag on ships it to every
second of every flight, and nothing had asked what it does in between.
`parapenting_pitch_eigenmodes --lag` is that question, in four parts. It costs
about twenty minutes and it does not assert, because two of its four answers
are disagreements rather than gates.

**1. TRIM IS THE CONTROL AND IT PASSES.** Wagner's function goes to 1, so a
wing holding a constant circulation forever has a lag state that has caught up
with it, and the two aircraft must trim in the same place. Settled to the
incidence-spread criterion at each one's own trim:

| | speed | sink | glide | incidence | settle |
|---|---|---|---|---|---|
| quasi-steady | 10.604 | 0.965 | 10.946 | 4.924° | 370 s |
| lagged state | 10.579 | 0.962 | 10.952 | 4.947° | **100 s** |

Speed differs by 0.24%, glide by 0.06%. The lagged wing settles in **100 s
against 370** — which is *not* evidence that it is better damped, and the block
says so where the number is printed: the criterion is ten seconds of incidence
spread under 0.01°, and a state that carries circulation forward smooths the
incidence trace whether or not it removes any energy. The spectrum is what
answers that.

**2. THE LAG IS NOT THE PHUGOID'S MISSING DAMPING. IT IS A LARGE PENDULUM
DAMPER.** Item 19's standing sentence is that "the phugoid's damping is not
available from any term currently in the solver". The lag is a term that was
not in the solver, and it is a candidate on physics rather than by elimination
— unsteady circulatory lag is the classical reason an aerofoil oscillating in
incidence is damped where a quasi-steady one is not. Measured at two transition
times, so a number that moves with T is visible as the sampling talking:

| wing | T | phugoid | ζ | σ /s | pendulum | ζ | σ /s |
|---|---|---|---|---|---|---|---|
| quasi-steady | 0.25 | 16.41 s | 0.0545 | -0.0209 | 1.84 s | 0.0933 | -0.320 |
| **lagged** | 0.25 | **10.16 s** | 0.0387 | -0.0240 | 1.94 s | **0.2475** | **-0.828** |
| quasi-steady | 0.10 | 16.63 s | 0.0660 | -0.0250 | 1.84 s | 0.0998 | -0.342 |
| **lagged** | 0.10 | **9.91 s** | 0.0312 | -0.0198 | 1.90 s | **0.2499** | **-0.852** |

- **The phugoid's ζ falls, and reading that as lost damping would be an error
  of arithmetic.** ζ = σ/ωₙ. σ — the actual energy removed per second, and the
  number that decides stability — is **-0.021 against -0.024 at T = 0.25 and
  -0.025 against -0.020 at T = 0.10**: unchanged, within a spread the sampling
  interval itself covers. What moved is ωₙ. The period drops from 16.4 s to
  **10.2 s, a 40% fall**, and ζ falls because its denominator rose.
- **So the answer to item 19's question is no.** The lag does not supply the
  phugoid damping that eight other routes failed to supply. It is worth saying
  plainly because the period moving 40% looks like a large effect on the mode,
  and on the stability of that mode it is not one.
- **What it does move is large and is in the wrong direction.** The pendulum's
  ζ goes from 0.093-0.100 to **0.248-0.250** — nearly tripled, consistent at
  both T. Item 19 established that the shipped structural damper is *right*
  precisely because it lands the pendulum at ζ 0.108 against a measured 0.09,
  and that reaching the phugoid with it costs "one visible swing where a real
  recovery shows several — exactly the ring the pilot asked to keep". **The lag
  spends that ring without being asked**, and by a similar amount to the
  structural damper the same item declined to use.
- **The honest caveat is structural.** The lagged aircraft carries a
  per-section circulation state the six-state reduction does not contain, so
  the matrix is a projection of a larger system. The columns are still honest —
  the lag state lives in `CoupledState` and is copied with each perturbed
  aircraft, which is what a real disturbance does to a real wing — but a mode
  living mostly in the circulation would not appear in that table at all.

**3. THE STEP RESPONSE FOUND AN ARITHMETIC DEFECT THAT IS NOT ABOUT THE FLAG AT
ALL, AND IT REACHES THE SHIPPED AIRCRAFT.** `SolveUnsteady` runs once every
`aerodynamicsInterval` steps and advances two states whose rates are per
SECOND: the separation state, which is what gives stall its memory, and strand
2's Wagner lag. **It was being handed the SIMULATION step.** At the shipped
interval 6 both have therefore been running at **one sixth of real time**, for
as long as the separation state has existed.

- **This is a defect, not a modelling choice.** A rate per second times the
  wrong number of seconds is arithmetic.
- **The test that says so is schedule-independence**, which is the property an
  aerodynamic state is supposed to have and did not. Fraction of a circulation
  step closed after 1.0 s: **0.280 at interval 6 against 0.544 at interval 1**,
  a factor of two apart on the same aircraft. Corrected, interval 6 gives
  **0.523** against interval 1's 0.544 — they agree.
- **The instrument self-checked without being asked to.** At interval 1 the
  elapsed time IS the simulation step, so the correction must be a no-op there,
  and the corrected and uncorrected interval-1 columns are identical to three
  decimals at every row.
- Behind `CoupledParagliderSolver::SetAerodynamicElapsedTime`, **defaulting OFF,
  and the default is the wrong one.** That is uncomfortable and it is
  deliberate: see what it costs, below.

**4. AND A SECOND DISCREPANCY SURVIVES THAT FIX: THE IMPLEMENTED LAG IS NOT
WAGNER'S.** Checked against R. T. Jones' published Φ(s) written out in the test
rather than reached for from the solver, reading circulation straight out of
the carried state instead of inferring it back through the aircraft's lift
coefficient. At **interval 1, where the elapsed-time defect does not apply at
all**:

| semichords | gap closed | Wagner Φ(s) |
|---|---|---|
| 0.08 | **0.119** | **0.508** |
| 0.45 | 0.214 | 0.546 |
| 2.27 | 0.322 | 0.682 |
| 9.09 | **0.544** | **0.869** |

- **Wagner's defining feature is that half the lift arrives immediately.** This
  wing delivers 12% of it. At nine semichords it has closed 54% of a gap the
  published function closes 87% of.
- **Strand 1 verified `WagnerLag` in isolation against Jones' values and that
  verification stands.** What was never checked is the COMPOSITE.

**IDENTIFIED, AND IT IS THE TARGET RATHER THAN THE RESPONSE.** Measured in
`aerodynamics_tests`, on the VSM alone — no coupled solver, no pressure,
membrane, collapse or suspension, and no settle. It reproduces the coupled
numbers to three decimals (0.119, 0.167, 0.215, 0.247, 0.327, 0.425, 0.552),
so **the defect is inside the VSM** and none of the five subsystems removed
had anything to do with it.

| semichords | quasi-steady | lagged | **what the pass aimed at** | Wagner Φ |
|---|---|---|---|---|
| 0.08 | **1.000** | 0.119 | **0.233** | 0.508 |
| 0.45 | 1.000 | 0.215 | 0.391 | 0.546 |
| 2.27 | 1.000 | 0.327 | 0.477 | 0.682 |
| 9.09 | 1.000 | 0.552 | 0.651 | 0.869 |
| 18.19 | 1.000 | 0.687 | 0.756 | 0.926 |

- **THE CONTROL IS THE COLUMN THAT MAKES THE REST READABLE, AND IT WAS MISSING
  FROM THE FIRST MEASUREMENT.** The quasi-steady wing closes **1.000** in one
  solve, at every row. That is what establishes that the target is 1.2× trim
  and that the denominator every other number is quoted against is the right
  one. Item 30's first pass through the coupled solver never printed it, and
  without it "the lagged wing closes 0.119" has an obvious alternative
  explanation — a wrong denominator — that could not be ruled out.
- **Wagner is doing exactly its job.** The shortfall is that it is applied to a
  target that has itself barely moved, and the two multiply:
  **Φ(0.076) = 0.508 × target 0.233 = 0.118 against a measured 0.119.** Checked
  as a product in the suite rather than told as a story, because a mechanism
  that reproduces the number to three decimals is identified and one that
  merely points the right way is a hypothesis.
- **AND THAT CONTRADICTS THE ASSUMPTION STRAND 2 WAS BUILT ON.** Its design
  note drops the global fixed point across sections on the stated grounds that
  it is *"the coupling the quasi-steady path already documents as the weak
  one"*. **It is not weak.** The quasi-steady column closes 1.000 in one solve
  *because its outer loop iterates*; one Jacobi pass of the same coupling
  closes **0.233**, leaving three quarters of a circulation step on the table.
  The self term is implicit and well posed, exactly as the note says — the
  cross-section coupling it was separated from is what carries the step.
- **So "explicit in time" is not a small change of scheme here.** A state is
  allowed to be explicit in time; what is not allowed is for that to be sold as
  dropping a weak coupling when it is carrying most of the answer. The wing is
  lagged twice, only one of the two is published, and **the unpublished one is
  the larger**.
**THE DESIGN QUESTION IS NOW MEASURED, AND THE ANSWER IS NO: ITERATING THE
TARGET IS NOT THE FIX.** The obvious repair is to build the target with more
than one Jacobi pass and apply Wagner to that, which is what
`VsmSettings::lagTargetPasses` now makes measurable (default 1, bit-identical
to what strand 2 shipped). Both regimes, each against its own trim, with the
denominator printed before any ratio built on it:

| passes | closed at trim | closed at 25° |
|---|---|---|
| 1 | 0.233 | -1.974 |
| 2 | 0.339 | -3.094 |
| 4 | 0.507 | -2.211 |
| 8 | 0.725 | -5.333 |
| 16 | 0.914 | -0.918 |
| 32 | **0.991** | -3.922 |
| 64 | **1.000** | -1.865 |
| *quasi-steady* | *1.000* | *1.076* |

- **ATTACHED, IT WORKS AND IT IS AFFORDABLE.** The target converges
  monotonically and reaches 0.991 in 32 passes. **The shipped flight solve
  already runs a 40-iteration cap**, and item 19 measured it converged there —
  so a target Wagner can honestly be applied to costs no more than the
  aerodynamics the aircraft flies on today, plus one Wagner step.
- **SEPARATED, IT FAILS, AND NOT BY MISSING.** Past the stall the target lands
  **one to five times the step away, on the wrong side, and where it lands
  depends on the budget** — non-monotone in pass count, which is not an
  unconverged solve but an iteration with nothing attracting it. This is item 6
  measured as a function of pass count for the first time rather than asserted.
- **The denominator was checked before the column was read**, because this item
  already made that mistake once: separated trim circulation is **843.5 against
  337.9 attached**, a gap of 168.7. The numbers are a measurement, not a
  division by something small.
- **So the scheme cannot be "more passes".** It works where the flow is
  attached and fails where it is separated, which is precisely the regime
  strand 2 exists for. The full 600-iteration adaptive solve *does* land
  (the quasi-steady row, 1.076) — that is why the quasi-steady path works at
  all — but it gets there by an amount of work with no bound, and **a
  fixed-cost state with an unbounded solve inside it is not a state.**

**AND THEN THE ATTACHED HALF WAS CLOSED: ON A CONVERGED TARGET THE COMPOSITE
IS WAGNER'S.** Everything above measures the TARGET, which is one factor of the
product — nobody had marched the wing on a converged target and asked the
original question again. Same sweep, same harness, `lagTargetPasses = 64`:

| semichords | closed (64 passes) | closed (1 pass) | Wagner Φ(s) |
|---|---|---|---|
| 0.08 | **0.510** | 0.119 | **0.508** |
| 0.15 | 0.519 | 0.167 | 0.516 |
| 0.45 | 0.554 | 0.215 | 0.546 |
| 0.91 | 0.600 | 0.247 | 0.587 |
| 2.27 | 0.701 | 0.327 | 0.682 |
| 4.55 | 0.799 | 0.425 | 0.780 |
| 9.09 | 0.882 | 0.552 | 0.869 |
| 18.19 | 0.935 | 0.687 | 0.926 |

Worst gap over the whole sweep **0.020**, against 0.389 at one pass. **Wagner's
defining feature is that half the lift arrives immediately, and this wing
delivers 0.510.**

- **Asserted against Jones' curve, not against the old number.** "Closer than
  one pass" would pass for a wing still several times slow. The gate is the
  published function at every row, and the second check — worst gap under half
  the old one — exists only to record that this is the same solver and the same
  harness, so the change is of scheme and not of instrument.
- **It was not a formality, and it could have failed.** The passes that build
  the target start FROM the lagged state, so their fixed point is the
  quasi-steady answer for a wing carrying the lagged downwash, not the final
  one. That it reproduces the indicial response of the published function is a
  measurement; had it not, "iterate the target" would have been wrong on the
  attached side too.
- **The composite now runs very slightly FAST, consistently: 0.510 against
  0.508, 0.935 against 0.926.** Under 2% and in one direction, which is the
  signature of the previous point — the target the state aims at each tick
  sits marginally above the true steady value while the circulation is still
  low. It is inside the gate and it is not noise; recorded so that if it grows
  it is already a known quantity.

**AND THE SEPARATED REGIME CAN NOW BE DECLARED RATHER THAN DISCOVERED**, which
is the precondition of route 3 below. `VsmSolution::targetResidual` reports the
pass-to-pass change in the TARGET, under `lagCirculation` only:

| passes | target residual, trim | target residual, 25° |
|---|---|---|
| 2 | 1.88e-02 | 2.42e+00 |
| 8 | 7.96e-03 | 9.86e-01 |
| 32 | **2.58e-04** | **3.01e-01** |
| 64 | **2.61e-06** | 1.01e+00 |

- **This is NOT `residual`, and the distinction is why nothing was reporting
  it.** Under lag `residual` is the distance the STATE still has to travel — a
  transient, large on a perfectly healthy solve. So the solver had no field
  that answered "is the thing I am aiming at a fixed point?", and item 30
  measured that past the stall it is not one.
- **Five to six orders of magnitude separate the regimes at the same fixed
  cost.** The separated column does not shrink with budget at all — 0.30 at 32
  passes and 1.01 at 64 — which is the non-monotone table above seen from
  inside the solve rather than from a test harness.
- **One pass reports −1, meaning NOT MEASURED, and that is deliberate.** One
  pass has no pass-to-pass change; reporting 0 there would claim convergence
  about a quantity nobody looked at, in exactly the setting that ships.
- **It changes no numbers.** It reads `nextCirculation`, enters no circulation,
  and is computed only under lag.

**AND THEN THE GATE WAS RUN ON THE REAL WAGNER, AND STRAND 2'S HEADLINE DOES
NOT SURVIVE IT.** Both corrections together for the first time — the elapsed
time AND a target iterated to 40 passes, reachable from the coupled solve by
`SetLagTargetPasses`. Same frontal, same gust, same 14 s, all four rows measured
in one harness rather than quoted across blocks:

| configuration | margin break | fold break | peak mirror resid | fold | worst target resid |
|---|---|---|---|---|---|
| strand 2 as shipped | never | never | **3.52e-14** | 0.998 | not measured |
| + elapsed time only | 1.050 s | 1.150 s | 4.04e-01 | 1.000 | not measured |
| + Wagner target only | 0.250 s | 0.250 s | **8.90e-01** | 1.000 | **9.70e+01** |
| **+ BOTH (the real Wagner)** | **0.050 s** | 0.100 s | 5.40e-01 | 1.000 | 2.90e+00 |

- **It does not merely fail to hold symmetry — it breaks EARLIER THAN THE
  QUASI-STEADY WING.** The shipped quasi-steady solve breaks at 1.350 s. The
  correctly-lagged wing breaks at **0.050 s**, twenty-seven times sooner, on
  the second aerodynamic tick after the gust. Every step toward the published
  function makes the gate worse, monotonically: never → 1.050 → 0.250 → 0.050.
- **So item 27's headline is now retracted rather than re-scoped.** The
  previous entry said the symmetry result "was real and is not retracted — it
  was taken on this solver, in this harness". That still holds as a statement
  about what was measured. What it means is now settled and it is the
  unwelcome reading: **the symmetry came from the depth of the lag, not from
  the circulation being a state.** A lag several times slower than Wagner's
  smooths the tick that picks a branch; Wagner's own lag does not.
- **The wing still folds in every row** — 1.000 against the shipped 0.998 — so
  none of these numbers is a symmetry bought by declining to collapse. That
  guard is in the gate, on all four rows, for the reason item 27 put it there.
- **STRAND 3's ENTRY CRITERION IS NOT MET, AND THIS IS THE MEASUREMENT THAT
  SAYS SO.** §68 asks for a separated solve that is SINGLE-VALUED. The lagged
  path was believed to supply it. It does not, once the lag is the one Jones
  published. Strand 3 should not start on the strength of strand 2.
- **The `+ Wagner target only` row is the worst of the four, and that is
  consistent rather than odd**: iterating the target without correcting the
  elapsed time drives a non-convergent iteration hard while the response that
  would smooth it still runs six times slow. Its target residual is **97**.

**AND THE SOLVE REPORTS THE REGIME ON THE TICK, IN FLIGHT.** The target
residual reaches 2.9 (both corrections) and 97 (target only) during the event,
through `CoupledDiagnostics::vsmTargetResidual`. This is item 6 measured
through the coupled solver for the first time rather than in a bare-VSM
harness: the collapse drives the target somewhere it does not converge, and the
solve now says so on the tick instead of leaving it to be inferred from a
symmetry break several ticks downstream. The one-pass rows report −1, NOT
MEASURED, which is the honest answer for a single pass and is what ships.

**AND THEN THE MECHANISM WAS CHASED DOWN, AND IT IS NOT UNSTEADINESS AT ALL.**
The four-row table is monotone, and its rows differ in how deep the lag is, so
the obvious reading is that strand 2's symmetry was bought with lag depth.
**That is a hypothesis with four points and two confounds, and it is wrong.**
`VsmSettings::lagDepthScale` divides the reduced time by a declared factor —
an instrument, 1.0 is Wagner's own depth and bit-identical — so depth is the
only thing that moves:

| × Wagner's lag | margin break, 40 passes | margin break, 1 pass |
|---|---|---|
| 1 | 0.050 | 1.050 |
| 4 | 0.050 | 1.100 |
| 16 | 0.050 | 1.100 |
| 64 | **0.050** | **1.100** |

- **Sixty-four times Wagner's lag does not delay the break by one tick.** With
  the target iterated it is not a function of depth at all; with one pass it
  buys a single aerodynamic tick and then stops, and it never holds.
- **Swept at BOTH target settings on purpose.** Sweeping only at 40 passes asks
  whether depth can rescue an iterated target; the answer matters, but one pass
  is the configuration strand 2's symmetry actually lived in, and a sweep run
  only where the symmetry was already gone would have looked conclusive while
  answering the wrong question.

**SO IT WAS THE OTHER STATE — AND THEN IT WASN'T THAT EITHER.** The elapsed-time
defect slowed TWO states by the same factor and one flag corrected both, so no
measurement to this point could separate them. `separationDepthScale` does the
same job for the stall memory, which makes the attribution a 2×2 on a wing whose
elapsed time is correct:

| circulation lag | stall memory | margin break | fold |
|---|---|---|---|
| 1× | 1× | 1.050 | 1.000 |
| 6× | 1× | 1.100 | 1.000 |
| 1× | 6× | 1.150 | 1.000 |
| **6×** | **6×** | **never** | **0.998** |

- **NEITHER STATE ALONE HOLDS IT.** Each buys 50–100 ms in the same direction.
  Both together buy the entire event. **The symmetry is not a property of
  either state**, so it is not a property of circulation being carried as a
  state — which is the mechanism strand 2's design note claims.
- **What it is a statement about is the COUPLING.** What holds the frontal
  symmetric is the aerodynamics *as a whole* running six times slower than the
  120 Hz structure it feeds. The collapse solver was being handed a field that
  could not move at its own speed, and that — not unsteadiness — is what
  removed the branch choice.
- **The control is the fourth row and it is exact.** 6×/6× reconstructs the
  elapsed-time defect out of two instruments on a wing whose flag says the
  elapsed time is correct, and reproduces item 27's printed numbers: never
  breaks, folds to 0.998 where every other cell folds to 1.000. The instruments
  measure the defect rather than something adjacent to it.
- **Every cell still folds**, so no row here is a symmetry bought by declining
  to collapse.

**AND THE SAME STATEMENT MADE WITH THE SCHEDULE — WHICH IS WHERE §68 AND ITEM
25 JOIN THIS.** If the symmetry needs the aerodynamics running slower than the
structure, there is a second and completely independent way to say it: leave
every state's physics alone and change how often the collapse solver is handed
a new field. That is `aerodynamicsInterval`, a shipped number rather than an
instrument, and it had never been swept against this gate. **On the shipped
quasi-steady wing — no lag, no instruments, no corrections:**

| interval | aero rate | margin break | fold |
|---|---|---|---|
| 1 | 120 Hz | **1.083 s** | 1.000 |
| 2 | 60 Hz | 1.150 s | 1.000 |
| 3 | 40 Hz | 1.225 s | 1.000 |
| **6 (ships)** | 20 Hz | **1.350 s** | 1.000 |
| 12 | 10 Hz | 1.400 s | 1.000 |
| 24 | 5 Hz | **1.600 s** | 1.000 |

- **§68's attribution is true and is not a fact about the aerodynamics.** "The
  margin field loses symmetry on a single aerodynamic tick" holds at every
  row — but *which* tick is the schedule's to choose, and it moves half a
  second across the range, monotonically, on the aircraft that ships.
- **Item 25 is the same curve seen from a different gate.** Four gates fail at
  interval 6 and pass at 12; three of them pin break times and residuals
  identified at the 0.1 s hold. This is why. They are not independent facts
  about the wing, they are one curve read at two points.
- **The control is the shipped number, exactly.** Interval 6 reproduces §68's
  1.350 s to the tick, so the other rows are the schedule moving and not a
  different aircraft.
- **BUT THE SCHEDULE ONLY POSTPONES IT — IT NEVER REMOVES IT**, and that
  distinction is the useful part. Slowing the two states held the frontal for
  the whole event; no hold from 120 Hz to 5 Hz does. They are not the same
  currency, and the 2×2 says why: it takes BOTH states slow, and a quasi-steady
  wing has no circulation state for the schedule to reach. **The symmetry is
  not bought by feeding the structure less often. It is bought by the
  aerodynamic field being unable to change, in both of the ways it can change**
  — which is a defect wearing two costumes rather than a scheme.

**WHAT THIS COSTS STRAND 3, STATED PLAINLY.** The entry criterion asks for a
separated solve that is single-valued. The only configuration this project has
ever seen hold the symmetric frontal is one where the aerodynamic states are
six times too slow, and that configuration is a defect rather than a scheme. A
free wake will not inherit single-valuedness from strand 2, because strand 2
never had it. **Item 6 is the open problem, and it is upstream of the wake.**

**WHAT IS LEFT, AND IT IS NOW A NARROW QUESTION.** Wagner's function describes
approach to a STEADY value, and in the separated regime this wing does not have
one that a bounded solve can find. So either:

- the lag is applied to something other than a converged fixed point, and then
  what it means has to be stated — it is no longer Jones' Φ and should not be
  reported as it; or
- the separated regime gets a different formulation entirely, which is item 6's
  own suggestion and a level rather than a fix — **and item 6's mechanism is
  now measured: the iteration stops converging at 12.0°, exactly where the
  section's lift slope changes sign**, so whatever that formulation is, what it
  has to replace is the branch a negative slope makes non-contracting; or
- the target is iterated where that converges and the scheme degrades
  deliberately where it does not, which is honest only if the degradation is
  declared and gated rather than discovered later. **This is now the route with
  both of its halves measured**: the converging side reproduces Jones' function
  to 0.020 over the whole sweep at a cost the shipped solve already pays, and
  the degradation is visible from inside the solve on the tick it happens,
  through `targetResidual`, at fixed cost. **And the signal no longer has to be
  a failed iteration**: item 6's criterion is the sign of dCl/dα, which is a
  polar lookup at a section's own incidence, so the degradation can be declared
  on ENTRY rather than detected after the passes are spent. What is still
  missing is the DECISION about what the scheme does when it fires — which is a
  modelling choice about separated flow, not a measurement.

**None of these is chosen here, and none should be chosen without the collapse
gates in front of it** — the frontal is a separated-flow event, so it is the
benchmark that would decide between them, and it is currently characterised
against aerodynamic states running six times slow.
- **This is the general form of the lesson item 19 keeps relearning**, one turn
  further out: a component verified against a published number, and then wired
  into something that changes it, with nothing re-checking the assembled
  answer against the same published number.

**WHAT THIS COSTS, AND WHY THE CORRECTION DEFAULTS OFF.** With the elapsed time
corrected, **seven gates fail** across `collapse_tests` and `coupled_tests` —
every one of them a collapse benchmark, which is exactly what these two states
are made of. Among them is **strand 2's own gate**: peak mirror residual
**4.04e-01 against the 3.52e-14 item 27 reports**, so the symmetry that closed
strand 2 does not survive its own aerodynamics running at the right speed.

- **The flag-off path is unchanged and all twelve suites are green**, which is
  the only reason this lands as a measurement rather than as a red tree.
- **Turning it on is a level's re-derivation, not a line**, and it should be
  taken as one: every collapse gate re-characterised against a wing whose stall
  memory runs six times faster than the one they were written against.
- **And it should probably wait for the Wagner discrepancy**, because fixing
  the elapsed time alone leaves the composite response still several times
  slower than the published function — so the gates would be re-derived once
  and then need it again. The two are one piece of work.
- **Item 27's headline is now RETRACTED, not re-scoped.** This entry used to
  say the re-scoping was enough. The four-row table above closes it: run on
  Wagner's actual lag the same frontal breaks at 0.050 s, so the round-off
  symmetry was a property of a lag several times too deep and not of carrying
  circulation as a state. The measurement was real; the mechanism it was
  attributed to is not the one that produced it.

## Data gaps

**19. The legacy pitch axis has no gravity-referenced pendulum, and a pilot
felt it.** Reported from flying the game: "after a stall the recovery is fast,
but the heading is still pretty much going down - it stabilises as if it was on
the moon's gravity. If I touch the brake or weight shift it works correctly."

Measured in `physics_tests`, full brake to a stall then hands off:

| | |
|---|---|
| trim glide before | 9.39 |
| stalled sink | 5.41 m/s |
| peak sink AFTER release | **11.75 m/s (x2.17)** |
| stall state cleared at | 1.91 s |
| flying forward again at | 4.39 s |
| glide back to trim at | **34.05 s** |

- **The stall STATE clears in under two seconds and the FLIGHT PATH takes half
  a minute.** The pre-existing deep-stall gate only checked the former, which
  is why this was invisible to the suite while being obvious to a pilot.
- **The cause is that `pitchStiffness` is referenced to incidence.** Its own
  comment calls it the "aerodynamic/pendular restoring moment toward the
  configured trim INCIDENCE" - the weathercock and the pendulum folded into one
  spring that measures incidence error. A wing diving vertically is already at
  trim incidence, so the spring reads zero and does nothing. Nothing else in
  the pitch axis references gravity. What recovery there is comes from the slow
  speed-for-height exchange.
- **Why brake and weight shift "work correctly":** both inject moments
  directly, so they bypass the missing term entirely. The pilot's own
  observation is the cleanest evidence for the diagnosis.
- **Scale.** The whole axis runs on 165 N.m/rad. The geometry-driven stack
  *measures* the line network's pitch spring at **6317 N.m/rad at 1 g** and
  carries the pendulum as a real degree of freedom. Different definitions, but
  the gap is roughly what "moon gravity" describes.
- **One latent bug found and fixed on the way, which was NOT the cause.** The
  hang-tilt pendulum input was gated to zero below 0.5 m/s of ground speed - a
  guard against dividing by an undefined track direction - and a stall descends
  at 0.39, so the pendulum switched off for the whole early recovery. Now
  referenced to the aircraft's heading, which is defined at zero airspeed, so
  there is nothing to guard. Measured effect on the numbers above: **none**
  (11.7547 to 11.7532). Kept because it is correct, reported because assuming
  it was the fix would have been wrong.
- **Not fixed, and the reason is a decision rather than difficulty.** Adding a
  gravity-referenced pendulum term changes an axis eleven calibration gates are
  written against, and the target numbers - how fast *should* a stall recovery
  convert descent into forward flight? - want a pilot's judgement rather than a
  plausible constant. Bounded at today's values in `physics_tests` so it cannot
  quietly worsen while that is decided.

**THE PILOT'S JUDGEMENT ARRIVED — *"I just want it to stall, and then I want
the shoot and pendulum after as realistic as possible"* — AND ACTING ON IT
KILLED TWO HYPOTHESES, INCLUDING THIS ITEM'S OWN PROPOSED FIX.** Three results,
all measured, none of them the plan above.

**1. THE PROPOSED FIX IS NOT PHYSICAL AND THE MODEL SAID SO IN ONE RUN.** This
item asks for "a gravity-referenced pendulum term" on the aircraft's pitch
axis. Built as specified — attitude against world vertical, stiffness from the
measured 1.86 s mode, referenced to an identified θ_trim of +0.353° — it
**destroys trim**: hands-up settles to 3.2 m/s forward at 12.9 m/s of sink with
67° of incidence. The aircraft stops flying and parachutes.

The reason is not a tuning error. **Gravity acts at the centre of mass and
therefore exerts no moment about the centre of mass.** A free rigid body has no
gravity pendulum in its own attitude, whatever its shape. What actually
restores a paraglider's pitch is the canopy's *aerodynamic* resultant acting
about 6.5 m above a CG that sits essentially at the pilot — and the model
already carries that, as `canopyRelativePitchRad`. **The term this item asks
for cannot be written, and the entry should not be read as a work order.** What
is real in the diagnosis is the observation about `pitchStiffness` reading zero
in a vertical dive; what is wrong is the proposed remedy.

**2. THE STATED BLOCKER IS FALSE FOR THE THREE CONSTANTS THAT MATTER.** "Changes
an axis eleven calibration gates are written against" is the reason this item
has sat. Swept directly — `pitchStiffness` over 165 → 2400, `canopyPitchPeriodS`
over 1.86 → 4.00, `swingIncidenceGain` over 0.15 → 0.50 — **hands-up trim does
not move at all**: 10.75 m/s, 1.143 sink, 9.40 glide in *every* cell of both
sweeps, and the brake-zoom gate passes in all but the corner noted below.

`pitchStiffness` **cannot** break trim, and it is structural rather than lucky:
it multiplies `alphaFromTrim`, which is zero at trim by construction, so its
magnitude sets how fast incidence errors are corrected and never where the
equilibrium is. The axis is far more open to tuning than this item claimed.

**3. THE 4.00 s COMPROMISE IS A 1D SWEEP OF A 2D PROBLEM — AND FIXING IT DOES
NOT HELP.** `canopyPitchPeriodS` is held at 4.00 instead of the measured 1.86
by one gate (>1.0 m of zoom climb), with the header recording "1.86 s gives
0.86 m and fails it". Swept against `swingIncidenceGain`:

| gain | 1.86 s | 2.40 | 3.00 | 3.50 | 4.00 |
|---|---|---|---|---|---|
| 0.15 | **1.03** | 1.04 | 1.04 | 1.04 | 1.05 |
| 0.35 | 0.93 | 0.95 | 1.01 | 1.01 | 1.02 |
| 0.50 | **0.86** | 0.93 | 0.94 | 0.95 | 0.95 |

The recorded 0.86 m reproduces exactly — **at gain 0.50**. The period was swept
there, the gain was later lowered to 0.35, and the period was never re-swept.
At gain 0.15 the measured 1.86 s passes the gate at 1.03 m. **The corner
exists.**

**It is not worth taking, because the constant does not do what it was assumed
to do.** The post-stall oscillation a pilot actually watches has a period of
**6.6 s at `canopyPitchPeriodS` 1.86 and 6.8 s at 4.00** — the parameter moves
it by 3%. Whatever sets the swing the pilot sees, it is not this.

- **The "moon gravity" arithmetic is seductive and should be resisted.** A
  pendulum's period goes as 1/√g, so lunar gravity stretches it by √6 = 2.45,
  and 1.86 × 2.45 = 4.56 s against a shipped 4.00. That is a good enough match
  to feel like a finding, and the sweep above shows it is a coincidence: moving
  that constant to 1.86 leaves the observed swing at 6.6 s.
- **`pitchStiffness` is not the lever either.** √(K/I) = √(165/154) predicts
  6.07 s against an observed 6.6, which looks like the answer, but swept the
  period goes 6.6, 10.7, 2.1, 8.7, 8.8, 8.9 at K = 165 … 2400 — non-monotone
  and not a second-order mode in this parameter. The post-stall pitch motion is
  not a simple oscillator in either constant.
- ~~**OPEN:** what sets the ~6.6 s post-stall pitch oscillation?~~
  **IDENTIFIED. IT IS THE PHUGOID, AND IT IS TEXTBOOK-CORRECT.**

**THE MODE WAS IDENTIFIED RATHER THAN GUESSED AT, AND THAT ENDED THE ITEM.**
The legacy path was linearised about hands-up trim the way `pitch_eigenmodes`
does the coupled one — perturb each longitudinal state, run 0.5 s, take the
state transition matrix's eigenvalues (Faddeev-LeVerrier, Durand-Kerner; worst
unperturbed drift 9.2e-06 per second, so the point is a trim). Three
oscillatory modes, and **ablation** — freeze one state, re-identify — says what
each is made of:

| mode | vanishes when frozen | identity |
|---|---|---|
| **5.79 s, ζ 0.10** | `vx` **or** `vz` | **phugoid** — speed/height exchange |
| 4.78 s, ζ 0.55 | `canopySwing` / `canopyRate` | canopy pendulum |
| 1.70 s, ζ 0.73 | `payloadPitch` / `payloadRate` | payload pendulum |

- **What a pilot calls "the pendulum after a stall" is the phugoid.** It is the
  only lightly damped mode in the aircraft, so it dominates the free response
  and buries both real pendulums.
- **And it is right.** Lanchester gives T = 2πV/(g√2) = **4.87 s** at this
  wing's 10.75 m/s against the identified 5.79, and classic phugoid damping
  1/(√2·L/D) at L/D 9.4 gives **ζ 0.075** against the identified 0.10. The
  legacy model's post-stall oscillation is a correct phugoid for a low-L/D
  aircraft, and a correct phugoid rings for about ten cycles. **There is no
  defect here to fix.**
- **`pitchStiffness` participates in no oscillatory mode at all** — 165, 700
  and 1758 give identical spectra (5.79 / 4.78 / 1.70). That retro-explains the
  non-monotone garbage from sweeping it: those numbers were large-amplitude
  transient shape, not a mode, and the metric was reading the shoot rather than
  the ringing.
- **`canopyPitchDampingRatio` is not a route to it either.** Taken from the
  shipped 0.55 down to the eigenvalue's own 0.09, the observed post-stall
  period stays 5.5–6.6 s and the visible swings go **up**, 11 to 17 — more
  phugoid, not more pendulum. Trim is 10.75 / 1.143 / 9.40 and the zoom gate
  passes at every value.

**CROSS-MODEL DISAGREEMENT, FOUND ON THE WAY AND WORTH MORE THAN THIS ITEM.**
`pitch_eigenmodes` measures the COUPLED solver's phugoid at **14–24 s**. The
legacy path's is 5.79 s and Lanchester says 4.87 s. The two models of this
aircraft disagree about the phugoid by a factor of three to five.

**AN EARLIER DRAFT OF THIS PARAGRAPH CALLED THE COUPLED SOLVER WRONG, AND THAT
IS NOT SUPPORTED.** Lanchester's T = π√2·V/g is derived for a rigid aircraft
holding constant incidence, and a paraglider is not one: it is a light wing on
a heavy pendular payload, and the pendulum is free to trade against the
speed/height exchange. The ablation above shows the legacy phugoid is
**decoupled** from both pendulums — it survives freezing `canopySwing`,
`canopyRate`, `payloadPitch` and `payloadRate`, and dies only on `vx`/`vz`.
That decoupling is a property of the lumped path, not of the aircraft. In the
coupled solver the swing is a real degree of freedom, so a pendulum-lengthened
phugoid is a physically available answer, and agreeing with the rigid-body
formula would be the thing needing explanation.

**RESOLVED BY ABLATION, AND THE ANSWER IS THAT NEITHER MODEL IS WRONG — THEY
MODEL DIFFERENT AIRCRAFT. THE CONSEQUENCE FOR ITEM 17 IS BAD.**

The same ablation was pointed at the coupled solver (settled 370 s on the
incidence-spread criterion, trim 10.56 m/s / 0.965 sink, convention self-check
clean, unperturbed drift 8.5e-04 m/s per second; identical answers at
T = 0.5 s and T = 1.0 s, so the result is not a window artefact):

| frozen | modes |
|---|---|
| *nothing (shipped)* | **16.4 s (ζ 0.035)**, 1.84 s (ζ 0.092) |
| surge `vx` | 1.87 s |
| heave `vz` | 6.70 s (ζ 0.362), 2.43 s |
| pitch attitude | 6.63 s (ζ 0.419) |
| pitch rate | 5.96 s (ζ 0.393) |
| **link swing** | 1.92 s — **long mode gone** |
| **link rate** | 1.92 s — **long mode gone** |

**The 16.4 s mode dies when the link is frozen.** It is a pendulum-coupled
phugoid: it needs the whole longitudinal chain — speed, heave, attitude, pitch
rate *and* the link — and freezing any one of them destroys it. Constrain any
single degree of freedom and what is left oscillates at 6.0–6.7 s, near
Lanchester's 4.80. So the pendulum coupling is exactly the mechanism that
stretches it, the coupled solver is self-consistent, and Lanchester simply does
not apply to it.

**The legacy path matches Lanchester because its phugoid is decoupled from its
pendulums**, which the earlier ablation showed directly. That decoupling is an
artefact of lumping, not a property of the wing. On the physics, **the coupled
solver is the more honest of the two**: a real pilot swings, and that swing
does trade against speed.

**AND THAT IS THE PROBLEM.** What item 17 proposes to migrate the game onto:

| | period | ζ | decay/cycle | time to half |
|---|---|---|---|---|
| legacy, what ships today | 5.79 s | 0.100 | 0.53 | **6.4 s** |
| coupled, item 17's target | 16.4 s | 0.035 | 0.80 | **51.8 s** |
| Lanchester rigid, theory | 4.80 s | 0.075 | 0.62 | 7.1 s |

**A longitudinal disturbance takes 52 seconds to half-amplitude on the coupled
solver against 6.4 on the legacy path — eight times worse — at a period three
times longer.** The pilot's standing complaint about the *legacy* model is that
it "stabilises as if it was on the moon's gravity". Item 17's swap would make
precisely that complaint three times slower and eight times more persistent,
and it would do so while being the more physically defensible model.

- **This is now a blocker on item 17, and a new one.** Item 17 was blocked on
  two departures and a dead weight-shift control — all envelope problems. This
  is a handling problem inside the envelope, at trim, and it would be
  discovered by the pilot on the first flight after the swap.
- **The swap cannot be a swap.** Either the coupled solver's link damping is
  wrong at trim (ζ 0.035 on the dominant mode is very light for an aircraft
  with a person hanging off it and lines that sweep), or the mode is right and
  the game needs a deliberate, documented handling departure to be flyable —
  the same call item 24's phugoid raises, but four times larger.
- **What this does NOT say:** that ζ 0.035 is wrong. It is measured, not
  asserted, and `swingDampingRatio` at 0.35 against item 11's ~0.18 and the
  ~0.06 that pilot and line drag imply is a live open question elsewhere in
  this file. The next move is to sweep the coupled link damping against
  **this** mode's ζ, which is a number nothing has been tuned against before.

**SWEPT. ONE DAMPER SERVES BOTH MODES, AND THE TWO REQUIREMENTS PULL OPPOSITE
WAYS.** Every row settled on the incidence-spread criterion with its own trim,
and the spectrum was taken with the same settings the trim was flown at:

| structural damping /s | phugoid | ζ | pendulum | ζ | glide |
|---|---|---|---|---|---|
| 0.8 | 16.04 s | 0.012 | 2.23 s | **0.003** | 10.86 (unsettled) |
| **1.6 — shipped** | 16.42 s | **0.035** | 2.19 s | **0.108** | 10.95 |
| 2.4 | 16.81 s | 0.058 | 2.11 s | 0.197 | 10.97 |
| 3.2 | 17.23 s | 0.081 | 2.02 s | 0.266 | 10.95 |
| 4.8 | 18.07 s | 0.126 | 2.26 s | 0.417 | 10.95 |
| 6.4 | 18.95 s | 0.169 | 2.74 s | 0.547 | 10.96 |

**Glide holds at 10.86–10.97 throughout, so none of this is bought by wrecking
the polar.** The damper moves both modes monotonically, and that is the
problem:

- **The shipped 1.6 is RIGHT for the pendulum.** It puts the pendulum mode at
  **ζ 0.108** against the **0.09 `pitch_eigenmodes` measures** for this wing.
  The constant item 24 calls out as having "no registry entry, no derivation,
  no test, no sweep" turns out to sit within 20% of the measured value for the
  mode it most directly controls. It should be registered and derived, but it
  is not arbitrary.
- **Fixing the phugoid with it costs the pendulum.** Reaching the legacy path's
  ζ 0.10 on the phugoid needs structural ≈ 4.0, which puts the pendulum at
  ζ ≈ 0.34 — three to four times its measured damping, and one visible swing
  where a real recovery shows several. **That is exactly the ring the pilot
  asked to keep.** One constant, two modes, opposite requirements.
- **The low-damping end is not flyable.** `swingDampingRatio` 0.18 and 0.06 —
  item 11's "known" value and the one pilot-and-line drag imply — both
  **depart** past 20 degrees of incidence, as does structural 0.0. The values
  item 11 derives as physically correct cannot be flown in this solver. That is
  a statement about the solver rather than about the wing, and it is the
  measured version of the commit note that the damping coefficient "buys
  excitation, not robustness".

**ITEM 11'S TWO NAMED MISSING TERMS ARE NOW CLOSED, AND THEY CONFIRM THE
ESTIMATE THAT DISMISSED THEM.** Both are implemented behind hooks and default
off "pending exactly this measurement":

| | phugoid ζ | change | glide |
|---|---|---|---|
| shipped | 0.0351 | — | 10.95 |
| pilot-referenced harness drag | 0.0419 | **+19%** | 10.97 |
| line sweep damping | 0.0358 | **+2%** | 10.97 |
| both | 0.0425 | +21% | 10.96 |

§60's arithmetic predicted the line sweep was worth about 7% of a term itself
worth 0.01 of coefficient; it measures at +2% of the phugoid's damping and
leaves the polar alone. **The estimate that dismissed these terms was right,
and this is the closure measurement item 11 asked for** — they are real, they
are small, and they do not reach the mode that blocks item 17.

**SO THE PHUGOID'S DAMPING IS NOT AVAILABLE FROM ANY TERM CURRENTLY IN THE
SOLVER**, and the honest next question is why it is as low as it is.

> **ONE MORE TERM HAS SINCE BEEN TRIED, AND THE SENTENCE ABOVE SURVIVES IT.**
> Level 11 strand 2's lagged circulation is a term that was NOT in the solver
> when this was written, and unsteady circulatory lag is the classical reason
> an aerofoil oscillating in incidence is damped where a quasi-steady one is
> not — so it was a candidate on physics rather than by elimination. Measured
> (item 30): the phugoid's **σ does not move** — -0.021 against -0.024 at
> T = 0.25, -0.025 against -0.020 at T = 0.10 — while its PERIOD falls 40%,
> from 16.4 s to 10.2 s. ζ falls from 0.055 to 0.039, and reading that as lost
> damping is an error of arithmetic: ζ = σ/ωₙ and what moved is the
> denominator. **The lag reaches the phugoid's frequency and not its damping.**
> What it does damp is the pendulum, from ζ 0.09 to 0.25 — the ring this file
> spends several paragraphs above declining to spend. At L/D
10.95 the classic 1/(√2·L/D) gives ζ 0.065 against a measured 0.035, and in
absolute terms the damping RATE is worse than that ratio suggests: σ = ζ·ωn is
0.0134/s here against 0.085/s for a rigid aircraft at the same L/D — **six
times less energy removed per second from the mode a pilot sits inside.** A
pendulum trading energy with the phugoid can lengthen the period without
dissipating, which is consistent, but whether it should cost this much damping
is the open question, and it is now a specific one.

**~~MEASURED, AND THE PENDULUM IS NOT THE ONLY MECHANISM: THIS WING'S CL IS NOT
A FUNCTION OF INCIDENCE ALONE.~~ RETRACTED — THE SOFTENING WAS THE
INSTRUMENT.** The block below is kept because the way it failed is the reusable
part; the correction is immediately after it.

A phugoid oscillates in speed at nearly constant
incidence, so the aerodynamic force should scale as V². Scaling the settled
trim velocity by k with attitude and flight direction untouched — which pins
incidence by construction, and the solver's own diagnostic confirms it at
**4.924° in every row** — gives:

| k | speed | alpha | CL | force (a.u.) |
|---|---|---|---|---|
| 0.80 | 8.48 | 4.924° | **0.5802** | 7.531 |
| 0.90 | 9.54 | 4.924° | 0.5578 | 8.606 |
| 1.00 | 10.60 | 4.924° | 0.5418 | 9.807 |
| 1.10 | 11.66 | 4.924° | 0.5299 | 11.134 |
| 1.20 | 12.73 | 4.924° | **0.5209** | 12.588 |

**CL falls 11% across a 50% speed change at identical incidence** — CL ∝
V^-0.27, so the aerodynamic force rises as **V^1.73 rather than V^2**.

- **This is aeroelasticity, and it is an effect rather than a bug.** The canopy
  is a pressurised membrane on lines: higher dynamic pressure changes its
  billow and its loaded shape, so the circulation solution moves even though
  the geometric incidence has not. **The legacy path cannot exhibit this at
  all** — its CL is a pure function of alpha — which is a second concrete
  reason the two models' longitudinal behaviour differs, independent of the
  pendulum.
- **And it points the same way as the shortfall it was chasing.** A wing whose
  lift stiffens more slowly than V² restores a speed excess more weakly, which
  lengthens the phugoid and weakens its damping. The 16.4 s period and ζ 0.035
  now have **two** identified contributing mechanisms — pendulum coupling, from
  the ablation, and lift softening with speed, from here — rather than one
  unexplained gap.
- **ONE NUMBER IN THIS BLOCK IS NOT CLEAN AND SHOULD NOT BE QUOTED.** The
  whole-body force exponent measures ~1.3, below the 1.73 the CL sweep implies.
  That reads the CANOPY's acceleration, and the canopy is a body on a link, so
  line tension is **not** internal to it and does not cancel; the gap between
  1.73 and 1.3 is most likely tension rather than aerodynamics. **The CL column
  is the trustworthy result**, the force column is reported so the discrepancy
  is visible rather than hidden, and closing it needs a whole-aircraft momentum
  balance instead of a one-body one.
- **Next:** whether CL ∝ V^-0.27 is the RIGHT amount of softening is a question
  for the membrane and pressure solvers rather than for the phugoid. It is now
  a measurable property with a number on it, and `membrane_tests` /
  `pressure_tests` are where it would be gated.

**THE CORRECTION: CL IS FLAT, THE FORCE DOES RISE AS V², AND THE SOFTENING WAS
A ONE-STEP TRANSIENT.** The sweep above scaled the velocity and stepped **once**.
The canopy's shape state — cell pressure, separation, the suspension warm start
— is carried between steps and cannot re-equilibrate to a new dynamic pressure
in 1/120 s, so a one-step CL is the force the **old shape** makes at the **new
speed**. That is an unsteady transient, not an aeroelastic equilibrium, and
reporting it as a property of the wing was an error of the instrument.

Re-measured with the aircraft **held** at each speed — velocity and attitude
re-imposed every step, so speed and incidence stay pinned by construction while
every shape state relaxes freely:

| k | speed | CL @ 1 step | @ 0.5 s | @ 2 s | @ 8 s |
|---|---|---|---|---|---|
| 0.80 | 8.48 | 0.5802 | 0.5415 | 0.5417 | **0.5419** |
| 0.90 | 9.54 | 0.5578 | 0.5417 | 0.5418 | **0.5419** |
| 1.00 | 10.60 | 0.5418 | 0.5418 | 0.5418 | **0.5418** |
| 1.10 | 11.66 | 0.5299 | 0.5416 | 0.5412 | **0.5411** |
| 1.20 | 12.73 | 0.5209 | 0.5413 | 0.5405 | **0.5404** |

**Equilibrated CL ∝ V^-0.007 — flat to seven parts in a thousand — so the
aerodynamic force rises as V^1.99.** The solver's steady aerodynamics are
correct on exactly the axis the phugoid cares about, and there is no aeroelastic
lift softening to explain anything with. It also retires the ~1.3 whole-body
force exponent above as doubly contaminated: unsteady transient *and* link
tension.

- **So the pendulum coupling stands as the SOLE identified mechanism** for the
  16.4 s period and ζ 0.035. The second mechanism claimed here did not exist.
- **What IS real, and is worth its own line:** the transient. A step change in
  speed moves CL by up to 7% and it relaxes back in **under half a second** —
  every row is home by the 0.5 s column. That is unsteady aerodynamic lag with
  a time constant well below the 16.4 s mode, so it cannot affect the phugoid,
  but it is the right order to matter for gust and collapse response, where
  nothing has yet looked for it.
- **The lesson is the one this file keeps relearning.** Two entries above,
  item 24's duration table was a metric artefact; item 19's "moon gravity"
  arithmetic was a coincidence; this was a settling artefact. **Every one was
  found by re-measuring a result rather than by reasoning about it**, and the
  cost of not re-measuring is a mechanism in the physics that was never there.

**AND THE TRANSIENT ITSELF IS THE SCHEDULE, WHICH MAKES IT EVIDENCE FOR THE
INTERVAL DECISION RATHER THAN A PROPERTY OF THE WING.** It cannot be unsteady
aerodynamics: `WagnerLag` is implemented and verified against R. T. Jones'
published two-exponential form, but its own header states it is **"NOT YET
WIRED INTO THE FORCE ASSEMBLY, deliberately"** — every solve reads the polar
instantaneously. Two candidates remained, and they separate cleanly because
iterations cure one and cannot touch the other:

| `aerodynamicsInterval` | iteration cap | CL at step 1 | peak error |
|---|---|---|---|
| 6 *(the struct default)* | 40 | 0.5209 | **3.6%** |
| 12 | 40 | **0.3762** | **30.4%** |
| 12 | 150 | 0.3762 | 30.4% |
| 12 | 400 | 0.3762 | 30.4% |

- **It is the schedule, and not an unconverged solve.** Raising the flight
  solve cap from 40 to 400 changes nothing at all; shortening the interval
  changes everything.
- **The mechanism is exact, not approximate.** Between ticks the aerodynamic
  force is HELD, so the reported CL is the old force over the new dynamic
  pressure: 0.5418 / 1.44 = **0.3763** against a measured **0.3762**. For the
  whole hold the wing's aerodynamics are not merely lagged, they are *frozen*.
- **So the interval is worth 30% of CL for 100 ms against 3.6% for 50 ms**, on
  a 20% speed step. That is the physical argument the interval-12-versus-6
  question has been missing: at 12 the aircraft spends a tenth of a second
  after any speed change with aerodynamics that are wrong by a third, which is
  precisely the window a gust or a collapse onset lives in. **This supports the
  finer schedule on physics, independently of which gates it reddens.**
- **A residual 3.6% survives at every interval down to 1** and is unaffected by
  iterations, so it is genuine carried state — separation and cell pressure
  relaxing — and it is gone within six steps. That part is arguably the stall
  memory the separation state exists to provide.

**THE FOUR RED COUPLED GATES ARE RE-DERIVED AGAINST THE SHIPPED SCHEDULE, AND
THREE OF THEM HAD BEEN MEASURING THE SCHEDULE RATHER THAN THE AIRCRAFT.** The
failures were bisected to the interval change, not to any physics change, and
the measurement above is what says which side of that bisection to trust: a
frontal collapse develops in tens of milliseconds, and interval 12 freezes the
aerodynamics for a hundred. **The finer schedule is the one entitled to
characterise these benchmarks.** Per gate:

1. **The symmetric frontal's two failure modes had SWAPPED.** The recorded
   characterisation was "symmetry breaks on the SECTION and saturates at the
   HARNESS" — section splitting at 0.255 of spread, harness saturating at
   1.000/1.000. Measured at interval 6 it is the other way round: section
   saturates (**L 1.000 R 1.000, spread 0.000**) and harness splits (**L 0.773
   R 0.977, spread 0.204**). Both gates re-derived and the narrative rewritten
   to match.
2. **The load-bearing conclusion survived untouched**, which is what makes the
   swap safe to absorb: both corrections still engage the numerical safety
   envelope and the shipped wing still does not, so *"every drag correction
   that lands the published glide takes this frontal outside what the solver
   can represent, and item 12 has no route around Level 11"* stands exactly as
   written. **The conclusion was schedule-independent; only the
   characterisation was not.**
3. **The iteration-sweep's supporting gate changed sides, and the new answer is
   stronger.** It asserted `cold.residualBeforeBreak < shipped` — "the extra
   iterations are doing real work everywhere the solve can converge at all". At
   interval 6 the residual before the break is **7.02e-07 at cap 40, 6.83e-07
   at 200, 7.14e-07 at 600**: flat, and not monotone. There is no work left for
   the extra budget to do because the solve is *already converged at 40*. The
   gate now asserts that directly, which states the sweep's actual point — the
   break is not an iteration shortage — more plainly than the old form did. The
   headline gate (fifteen times the budget breaks on the same tick) never
   failed.
4. **A signed threshold was standing in for a magnitude one.** The spiral
   wind-up leaves the wing parked outside the attached regime; the gate read
   `angleOfAttack > 45°` because at interval 12 it parked at +70. At 6 it ends
   at **-168.68°**. Both are far outside anything the attached-flow
   formulation represents, which is the entire claim, and nothing downstream
   reads which way an unrepresentable wing points. Now bounded on `fabs`.

**What this unblocks:** strand 27's gate lives in this suite, and a red control
made it unreadable. The suite is the control again.

**A CLAIM WAS MADE HERE AND WITHDRAWN WITHIN THE SESSION, AND THE WITHDRAWAL IS
THE USEFUL PART.** The first reading of this measurement was that `SetSchedule`
is not idempotent — that calling it with the schedule's existing value shifted
the tick phase and changed the aircraft. It does not. `aerodynamicsInterval`
**defaults to 6**, and the probe had read "every 12 steps" out of a comment in
`pitch_axis_trace` and written 12 back, changing the value it believed it was
preserving. There is no phase bug; there are two different intervals. The
general form of the mistake is worth keeping: **a comment in one file was
treated as the default in another, and the two had diverged.**

**WHAT IS ACTUALLY LEFT IS A DESIGN CHOICE, NOT A BUG.** SIV footage shows a
post-stall recovery dominated by a fast pendulum swing, because a real pilot
damps the phugoid on the brakes without thinking about it and the model's pilot
does not. Making the game *feel* like that footage means either damping the
phugoid deliberately — knowingly departing from theory for feel — or giving the
canopy pendulum enough amplitude to be seen under it. **Both are pilot
judgement, neither is a correctness fix, and nothing should be tuned here until
that call is made.**
- Note this is the legacy path, which is what the game flies (item 7). The
  geometry-driven stack has the right structure by construction, so this is one
  more argument for item 17 - but that stack departs at 40% brake today and
  cannot fly a stall recovery either, so it is not a swap-in fix.

**20. Wing loading: the square root law is an approximation here, now
measured.** The suite corrects between a 94.3 kg solver and a 105 kg published
number using V proportional to sqrt(W) in three places. Swept across the EPIC 2
ML's certified 90-110 kg range in `calibration_tests`:

| all-up | trim speed | vs sqrt law | sink | glide | incidence |
|---|---|---|---|---|---|
| 90 kg | 10.54 m/s | — | 0.934 | 11.25 | 5.03 deg |
| 97 kg | 10.68 | -2.4% | 0.949 | 11.22 | 5.05 |
| 105 kg | 11.06 | -2.9% | 0.973 | 11.32 | 5.14 |
| 110 kg | 11.27 | **-3.3%** | 0.967 | 11.61 | 5.23 |

- **The departure is monotonic in weight**, which is a systematic effect rather
  than scatter, and **the mechanism is in the last column**: trim incidence
  climbs 0.20 degrees across the range, because the line network's pitch spring
  is geometric and stiffens with load, so the pitch balance settles slightly
  nose-up as the wing is loaded. V goes as sqrt(W/CL) and CL is not constant,
  so the fixed-CL law over-predicts.
- **Consequence beyond this block:** the correction applied at three call sites
  is good to about 3%, not exact. Bounded rather than corrected, because the
  fix is to compare at the same weight rather than to scale between weights.
- Glide is nearly loading-invariant as it should be - 3.25% across the whole
  range - and what movement there is tracks the incidence rather than being
  loose.

**21. Weight shift does nothing in the geometry-driven stack.** 50% of weight
shift gives **0.01 rad/s** of turn and leaves speed, sink and glide identical to
hands up to three significant figures. The legacy model at the same input turns
at 0.20 rad/s. Found by `parapenting_model_agreement` (§70).

- **It sat in a gap between two kinds of gate and that is why it survived.** It
  is not a departure, so the envelope gates did not catch it; it is not a
  disagreement with a published number, so calibration did not either.
- **It is a control, not a coefficient.** A paraglider has four inputs and this
  is one of them - a pilot flies whole turns on weight shift alone. Item 0b
  records the wing turning "several times too slowly" on brake; on weight shift
  it does not turn at all.
- Related to item 0b and possibly the same cause, but not assumed to be:
  0b is a brake-driven turn-rate deficit, this is a weight-shift authority of
  approximately zero.
- **DIAGNOSED (§71), and it is a structural gap rather than a weak number.**
  Traced link by link: full weight shift transfers **34% of the load** between
  the carabiners, correctly signed and proportional to the input, so the pilot
  moves and the lines feel it. Then it stops. `VsmSolveInput` carries airspeed,
  angular velocity, density, per-cell pressure, left/right brake and a
  per-section gust - and **nothing from the suspension solve**, so a 34% riser
  asymmetry has no channel through which to change the spanwise lift.
- **The surviving path cannot do the job arithmetically.** Weight shift
  translates the CG **7.1 cm** (`hipTravelM` 0.075 m x a 0.95 strap factor) on a
  **6.6 m** hang: atan(0.071/6.6) = 0.6 degrees of bank, 1.5 measured once the
  line network responds. Even 20 cm of hip travel gives 1.7. A real wing banks
  ten to fifteen. **No value of `hipTravelM` fixes this** - the fix is a
  channel from the suspension into the aerodynamic solve, so that differential
  riser load changes local incidence across the span, which is the mechanism
  that actually banks a paraglider.
- **It explains the shape of item 0b too.** Brake reaches the wing
  (`aero.leftBrake`/`rightBrake` are in the input struct), so brake turns work
  and are merely too slow. Weight shift has no equivalent, so it does not work
  at all. Two symptoms, one asymmetry in what the aero solve is allowed to know.
- Bounded in `coupled_tests`: the split is gated as WORKING and proportional,
  the bank is bounded under 3 degrees and the turn under 0.05 rad/s.
- **THE FIX IS DESIGNED, and it reopens Level 7.** That level's exit gate
  reads *"weight shift and brake turns EMERGE without direct turn moments"*.
  Only the negative half was ever checked. The design is in the master plan
  under "the geometric channel"; the short form:
  - `SuspensionSolution` computes `nodePositionM` for every attachment and
    then publishes one scalar, `incidenceChangeRad`, whose own comment says it
    is "the only path bar and brake have to the canopy". Everything spanwise is
    computed and discarded.
  - Publish a **per-station pose** instead: chord direction from each station's
    front (A/A') to rear (C) attachment, hence a per-section incidence offset
    from the design pose, plus the deformed arc.
  - `VsmSolveInput` gains `sectionIncidenceOffsetRad`, on the same pattern as
    `internalPressureCoefficient` and `sectionGustBodyMps` - **empty means the
    design pose**, so the change is additive and no existing caller moves.
  - **Brake's scalar path then retires into it**, which is the part that
    matters beyond this item: passing `leftBrake`/`rightBrake` into an
    aerodynamic solve is the control-to-aero shortcut guiding rule 4 forbids,
    and it is currently the only reason brake turns at all.
  - Weight shift then works **without being given a term of its own**, and if
    it needs one the design is wrong.
- **BUILT: the aerodynamic half. BLOCKED: the structural half** (§72).
  - `sectionIncidenceOffsetRad` exists and is gated. Empty is the design pose
    bit for bit, so no caller moved. **Twist buys 8543 N·m/rad of roll, flat
    to 0.03% over sixteen times the range**, mirror-exact, and it is a couple:
    four degrees changes lift by 0.7%.
  - **The per-station pose is identically zero and always will be.** Every
    canopy attachment is placed as `canopyOrigin + canopyAttitude.Rotate(...)`
    — one rigid body — so the offsets differ from the design pose by a single
    global rotation. Measured at full weight shift: identical to eight
    decimals across the span and **bit-identical left to right**, while the
    same solve puts the A row at 51 N left against 349 N right.
  - **The missing ingredient is canopy torsional compliance**, and nothing in
    the stack has it: Level 2's canopy is rigid, Level 6's membrane is 1-D
    chordwise strips with no torsion.
  - Because the gain is linear the requirement divides out: **matching today's
    full-brake roll takes about 10° of antisymmetric twist**, and brake is
    itself several times slow. So the open question is now well posed and
    structural — *does a canopy on its lines twist several degrees under a
    300 N row-tension difference?* — and it settles the item either way.
- Brake's scalar path **cannot retire yet**, because the channel that was to
  replace it carries nothing. Guiding rule 4's violation stands, now with a
  measured reason rather than an unexamined one.
- **No longer the cheapest thing standing between the stack and a pilot**: its
  remaining half needs canopy torsion, which is a level and not an afternoon.
- **AND CANOPY TORSION IS NOT ENOUGH ON ITS OWN** (§73). Measured by imposing
  the twist and flying the aircraft, which is what `SetImposedSpanwiseTwistRad`
  exists for:
  - **0.0272 rad/s per degree of twist, linear**, so 0.20 rad/s wants about
    **seven degrees** — superseding the ten §72 got by dividing brake's roll
    moment by the channel's gain, which answered a different question.
  - **The aircraft spirals at four.** Incidence falls from 4.7° to below zero,
    speed climbs past 21 m/s, and at ~35 s it winds up to 3.48 rad/s. The
    safety envelope never engages; this is the model's own behaviour.
  - So a **stable turn tops out near 0.09 rad/s**, and a real wing's 0.2–0.3
    is on the far side of a departure. **Perfect canopy torsion would not
    reach it.** What is in front of this item is **item 11**: §75 measured
    that the departure is the pitch divergence reached through a turn, at the
    same CL the accelerator departs at. Not a new level, and not a new item.

**24. `vsmConverged` is false in ordinary straight flight, so it gates
nothing.** Counting over 40 s of settled hands-up flight, 4180 steps of 4180
report it false. The flight solve's 40-iteration cap does not reach the 1e-6
tolerance and is not trying to; the flag compares against it anyway.

- Found while checking whether §73's spiral departure was numerical — for
  which it was useless, because a diagnostic that is false in the nominal case
  cannot distinguish the abnormal one.
- **Cheap and worth doing**: report the residual it actually reached against
  what the cap allows, or compare against a tolerance the flight solve is
  aiming at. Either makes the flag mean something.
- Nothing currently gates on it, which is the only reason this is small.

**25. CLOSED — both halves, and neither was what it looked like.** §74 closed
the skid (it was never there); §75 closed the spiral (it is item 11's pitch
divergence, reached through a turn instead of through the accelerator). What
was a new unowned stability problem is one already-owned one plus a reading
error. **Item 21 is behind item 11 now, not behind a separate item 25.**

- **The turn envelope's edge and the accelerator's edge are the same CL.** Last
  stable: **0.461** at 3.8° of twist, **0.425** at 20% of bar. They differ by
  0.036, less than either sweep's own step in CL (0.019 and 0.053), so they
  cannot be told apart. The accelerator run has no turn in it at all, which is
  what makes it the control.
- **So the stable turn ceiling near 0.09 rad/s is a pitch limit**, not a roll
  or spiral one. Nothing about canopy torsion, roll damping or yaw would move
  it. Fixing item 11 moves it.
- **And CL 0.35 is optimistic by about 0.09.** The loop-gain analysis puts the
  divergence there; the flown aircraft departs near **CL 0.44** by either
  route. Anywhere the record uses 0.35 to say what is reachable — "full bar is
  a CL 0.31 condition" — the real envelope is narrower than that implies.
- The earlier framing, kept because it was measured: departure between 3° and
  4° of imposed twist, i.e. between 0.086 and 0.19 rad/s of turn, where a real
  EN-B holds 0.2–0.3 rad/s as an ordinary sustainable turn.
- **The skid is closed and was never there (§74).** Sideslip measured directly
  is under 0.1°, and the payload link — where 95 of the 105 kg hangs, so the
  angle that actually turns the aircraft — sits at the coordinated bank within
  2%. `bankRad` reports the CANOPY's bank, which sits inboard by exactly the
  deflection the line roll spring needs to carry the twist's steady roll
  moment (+433 against -443 N·m at 3°, closing to 2%). §73 compared the canopy
  angle against a formula about the link angle. Gated in `coupled_tests`.
- The lateral force budget closes on the aerodynamic side force plus the
  canopy's banked lift, to 10%. The side force is real and large — 71 N at 3°,
  two thirds of the turn — and it appears **at zero sideslip**, because an
  arched canopy loaded antisymmetrically produces side force without slip.
- The "one fault or two" hypothesis is closed twice over: there was no second
  fault, and there turned out to be no *first* one either.
- **What inherits item 25's priority is item 11.** The argument for putting a
  stability problem in front of the canopy-torsion level survives intact — it
  is upstream of the turn authority the geometric channel was meant to supply,
  and building torsion without it would deliver a channel whose output the
  aircraft cannot use. Only the name of the blocker changes, and item 11 was
  already the top of the list.
- Level 8's unverified gate — *"asymmetric separation produces spin/spiral
  behaviour"* (item 23) — loses its suspected cause with this. The aircraft
  has no spiral mode of its own to be behind it; what it has is a pitch
  divergence, which is a different thing to gate against.

**22. The geometry-driven stack departs at 22% of speed bar, not at full bar.**
The record says full bar is the pitch-divergent condition because
`calibration_tests` only ever applied `accelerator = 1.0`. Bisected by
`parapenting_model_agreement` (§70), the wing departs at **22% of travel** -
half bar, an ordinary cruise input, is already well outside what the stack can
fly.

- Brake, bisected on the same run, flies to **37%** against the documented
  "departs at 40%" - so that sentence is confirmed and now has a boundary
  rather than a bracket.
- **Together these move item 17's blocker.** "Departs at 40% brake" sounded
  like an edge case a stated envelope could route around. Having no speed bar
  beyond 22% and no weight shift at all is two of the four controls missing,
  and an envelope excluding both does not describe a flyable aircraft.

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
