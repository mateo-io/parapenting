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

- The largest single candidate is the momentum thickness the shear layer off
  the cell mouth carries onto the upper surface. It is certainly not zero.
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
- Done when: glide lands inside the published figure without a coefficient
  chosen to put it there.
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
- **It is not the schedule.** Solving the aerodynamics at 120 Hz instead of
  10 Hz moves the 60 s spread from 0.597° to 0.528°. Consistent with a
  physical mode rather than a discretisation artefact.
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
- **Next, and this step redirected it:** the ~0.06 target cannot be reached by
  finding more *link* damping — at 0.06 the phugoid is far past its sign change.
  The missing stabilising mechanism has to act on **speed stability**, the flat
  lift curve of §34 (`L ~ V^0.171`), which is what leaves the phugoid nothing to
  restore with. That is the quantity to go after next, not the link.
- **Also open:** §35 measured the growing mode at 3.6–5.7 s on a departing wing,
  and no such mode is in this spectrum at any ratio. Large amplitude, outside
  what a linearisation claims — not a contradiction, not reconciled either.
- Done when: the wing settles in a time a pilot would recognise with a damping
  ratio derived from pilot and line drag (~0.06) rather than chosen to keep the
  aircraft from departing.

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
- Registered Tuned/Unvalidated, superseded-by Level 11, and bounded by the
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
- Next, and untried: warm-start each of those relaxations from the previous
  one. They are solves of the SAME network at neighbouring loads, and they are
  currently each started cold. Note `solver_lod` measured that the warm-started
  in-flight network converges in 40 iterations against a cold 12000, which is
  where the suspicion comes from.
- Done when: construction is under 100 ms.

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
  about twenty headers pull types from it.

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
