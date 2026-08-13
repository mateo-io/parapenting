# What building this taught, and what it cost

Bugs that took real time to find, and the patterns behind them. Written so the
next person does not pay for them twice.

The through-line: **almost every serious bug in this project was a sign or a
convention, not an algorithm.** The maths was usually right the first time. What
was wrong was which way something pointed, or what a number meant, or which of
two descriptions of the same thing was authoritative.

---

## 1. The tests were compiled out

`Tests/` built with `-DCMAKE_BUILD_TYPE=Release`, which defines `NDEBUG`, which
turns every `assert()` into nothing. **605 assertions in `PhysicsTests.cpp` had
been inert.** The suite passed by doing nothing, and had done for a long time.

Turning them back on immediately surfaced: a wind-direction bug, six stale route
expectations, and two selectable routes that put the pilot off the map.

**The tell** was a compiler warning — `unused variable 'weightShift'` — for a
variable used only inside asserts. That warning had been visible the whole time.

**Rule:** a test suite that has never failed is not evidence of anything. Break
it deliberately and check it notices.

---

## 2. Conventions documented backwards, and the cost compounds

`ParagliderCoordinateSystem.h` stated all three rotation senses. All three were
wrong:

| documented | actual |
|---|---|
| roll (+X) right wing down | right wing **UP** |
| pitch (+Y) nose up | nose **DOWN** |
| yaw (+Z) nose left | nose **RIGHT** |

The flight frame is forward/right/up, which is left-handed, while the quaternion
algebra is right-handed. Nobody reconciled them; each author who found a sign
wrong fixed it *downstream* against the header instead of against the wing.

The result was a model where weight shift banked the wing right and flew it
left, and brake banked it left and flew it right — each control with one correct
half and one inverted, and the two compensating well enough that it shipped.

**Two of my own attempted fixes were also wrong**, because I reasoned from the
header. Only integrating the attitude directly and measuring settled it.

**Rule:** derive signs from a measurement, not from a comment. For anything
attitude-related, read it off the rotated basis vectors:

```cpp
incidence (nose-up +) = asin(attitude.Rotate({1,0,0}).z)
bank (right tip down +) = asin(-attitude.Rotate({0,1,0}).z)
```

---

## 3. One variable, two opposite physical effects

`lateralLoadImbalance` carried both:

- **carabiner tension** asymmetry — the loaded side is pulled **down**
- **aerodynamic lift** asymmetry — the lifted side goes **up**

One sign was applied to both. Whichever control the coefficient had been fitted
to worked; the other was inverted. No downstream sign fix could rescue both,
which is why the bug survived so long — every local fix broke the other control.

**Rule:** if two effects have opposite signs, they are two variables. Merging
them looks like simplification and is actually a permanent trap.

---

## 4. Counting the same force twice

Appeared three times, in three subsystems:

- **Level 3:** the payload roll moment was both a hang-angle target *and* an
  injected acceleration. It pinned the relative roll at its clamp, and the wing
  tipped over ten seconds into a turn.
- **Level 4:** the section's own bound vortex induced on its own control point,
  when the 2D polar *is* that circulation. It halves the lift-curve slope,
  exactly and quietly.
- **Level 4 again:** having excluded the self-panel to fix that, its trailing
  legs went too — and those must stay, because the wake carries only the
  spanwise *change* in circulation. Removing them made the cancellation
  one-sided and the induced velocity enormous.

**Rule:** when coupling a table-driven model to a field solver, ask what the
table already contains. A polar is not a boundary condition.

---

## 5. Convergence measures that measure the wrong thing

Two variants, both of which produce a solver that lies about itself.

**The residual measured the damped step**, not the equation error. Increasing
damping shrank the number, so a solve damped to a crawl reported itself
converged. Measure the residual *before* relaxation.

**A stalled iteration looks exactly like an equilibrium.** The membrane's
kinetic energy fell to zero and the shape stopped dead — at 91 mm where the
analytic answer was 26. Energy going to zero proves nothing. The tell was that
the answer moved with *iteration count* rather than with time.

**Rule:** the convergence test is "does the answer change when I change the
budget", never "has it stopped moving".

---

## 6. Solver devices that change the physics

Fictitious mass is a legitimate and very effective device: the per-substep push
is F·h²/2m, so mass reduces it exactly as substep count does, and mass is free.
A 10⁴ mass scale took the membrane from unconverged to exact.

Then it sagged the strip under 10⁴ times its own weight, because gravity was
applied to the scaled mass.

**Rule:** a numerical device may change how fast something settles. The moment
it changes *where* it settles, it is a physics change wearing a device's
clothes. Compute physical forces from physical quantities and only accelerate
against the fictitious ones — and assert the answer is unchanged when the device
is varied.

---

## 7. Stiff systems: the timestep is usually not the problem

Recurring pattern — a system that will not settle, where the instinct is more
iterations:

- **membrane:** 2.7×10⁶ N/m against 0.46 g per node puts fabric elastic waves at
  12 kHz. Nobody cares about 12 kHz. Resolving it was all the solve was doing.
- **Level 7 damping, twice.** First: roll damping has a 20 ms time constant and
  was being held across a 100 ms aerodynamic interval — damping applied at a
  rate five time constants stale, which is not damping but excitation. Diverged
  to NaN in five seconds. Evaluating it every step fixed that, and left the
  second one, which took another round of work to see: the term was still being
  integrated *explicitly*. Yaw damping of 80 Nm per rad/s on an inertia of 150
  needs `c·dt/I < 2` to be stable and sat at eleven times that, so it alternated
  sign and doubled every step. Asymmetric brake reached an infinite turn rate in
  fourteen seconds.

  The fix is one divide. Backward Euler on the damping term alone —
  `omega' = (omega + M/I·dt) / (1 + c/I·dt)` — is unconditionally stable at any
  coefficient, and leaves everything else explicit.

**Rule, sharpened:** a stiff term does not have to set the timestep. Ask which
*term* is stiff, and integrate that one implicitly. Making the rate live is
necessary and is not sufficient.

**Rule:** compare every subsystem's own time constant against the rate you
intend to run it at, *before* choosing that rate. Where they conflict, either
split the load into held and live parts, or make the fast dynamics artificially
slow, or solve it statically. Do not simply iterate harder.

---

## 8. Getting the object wrong

The membrane's first version was a chordwise loop — nodes around the section,
pinned at leading and trailing edges. It inflated to a circle regardless of the
profile it was cut to, because that is what a closed loop of fabric with
pressure inside *is*.

The fabric works spanwise. Ribs hold the profile; the skin bulges between them.

The same class of error, elsewhere: applying the aerodynamic resultant in **body
axes** at a level with no aerodynamic solver. A nose-up rotation carries the lift
round with it and the pitch equilibrium runs away instead of standing still.

**Rule:** before solving, state what physically holds the thing in place. If the
answer is "nothing", the topology is wrong and no amount of solver work fixes
it.

---

## 9. Six solvers means six initial conditions

The Level 7 coupled solve stalled and fell out of the sky, and the cause was in
none of the solvers.

The flight state started **flying** — 10.8 m/s, level, in trim. The cell
pressure state started **packed** — every cell empty, because that is the honest
default for a canopy that has not inflated yet. On the first aerodynamic
re-solve the VSM was told every cell was at zero internal pressure. Level 5's
coupling then correctly cut section lift to 15% and added a large drag penalty,
and the wing correctly stalled.

**Every subsystem did exactly the right thing.** The aggregate was a wing doing
10 m/s with no air in it, which is not a wing.

What makes this class hard is that it is invisible from inside any one solver.
Each had a defensible default. Only the combination was nonsense, and nothing
owned the combination.

**Rule:** when state lives in several subsystems, "start the simulation" is its
own operation with its own correctness condition — every piece of state
consistent with the same instant. Give it a name (`SeedInflated`), call it
explicitly, and be suspicious of any subsystem whose default is "the beginning
of its own story" when the caller is starting mid-story.

A second, cheaper instance of the same thing in the same commit: the coupled
solver was not applying installed drag, so it glided at 14 where the wing glides
at 9.5. Level 4 had computed it correctly and Level 7 simply never asked. An
integration layer can be wrong purely by omission, and omission does not throw.

---

## 10. Degenerate cases with no meaningful answer

A dead calm has no wind direction. `atan2(0, 0)` returns 0 or 180 depending on
the *sign of zero*, and the preflight briefing charged the launch a 48-point
cross-wind penalty for being cross-wind in still air.

**Rule:** where a quantity is undefined, say so explicitly and let callers check
the magnitude. Do not let a formula answer a question that has no answer.

---

## 11. Validate against something external

The checks that actually caught things were the ones with an outside reference:

- lifting-line theory for the VSM — **0.2% on CL_α**
- the circular-arc solution for the membrane — **26.32 mm vs 25.99**
- published best glide — **9.46 vs 9.5** for the wing alone, and **9.5** again
  from the fully coupled solve, which also lands trim speed at 38.5 km/h against
  a published 39
- published total line length — **254.8 m vs 254**
- rigid-body statics for carabiner loads — exact

Every one of these caught a real error. Checks written against the code's own
previous output caught none, and in several cases *encoded the bug*: the Level 1
arc had two sign errors that the tests passed because they had made the same
wrong assumption.

**Rule:** prefer a check against theory, published data or a closed form. A
golden-value test only pins behaviour; it never tells you the behaviour was
right.

---

## 12. What honest failure looks like

Three things were committed *not working*, deliberately:

- the membrane's first attempt, with the diagnosis
- deep stall in the VSM, as a locked known-failure check
- Level 7's unstable trim

In each case the alternative was a green suite asserting a wrong number — the
membrane would have pinned a 91 mm bulge where physics says 26. A test that
passes on a wrong answer is worse than no test, because it actively defends the
error.

The known-failure locks are worth the pattern: `Check(!converged, "KNOWN
FAILURE: ... delete this check when it does")`. It fails loudly when someone
fixes it, which is exactly when you want to be told.

Two of the three are now closed, and closing Level 7's told us something about
the third kind of honesty. The excluded suite carried a written suspicion — "not
reproducible by a direct probe, so the suspect is the test harness rather than
the solver, but that is unverified". It was wrong, and the word doing the damage
was *unverified*: a plausible diagnosis, recorded as a caveat, kept the bug
parked for a whole level. A direct probe at -O0 reproduced it on the first run.

**Rule:** an unverified diagnosis is not a finding, and writing it down next to
the defect makes it look like one. Either verify it or record only the symptom.

---

## 13. Measuring a derivative is its own numerical problem

Three separate defects in Level 7 were all one mistake: treating "measure how
the moment changes with rotation rate" as free.

- **Dividing by the state.** The estimator took the live moment difference over
  the live rotation rate, with a guard zeroing it below 10⁻³ rad/s. Near zero
  rate that is noise over nothing, and worse, the guard made the damping law
  *discontinuous in the state*. Two mirror-image flights took opposite branches
  of it in the fourth second and stopped being mirror images.
- **A one-sided difference.** Replacing it with a fixed ±0.3 rad/s probe fixed
  the conditioning but, probed on one side only, still measured a different
  coefficient for a left turn than for its mirrored right turn — a one-sided
  difference is not odd in the rate. Centring it restored mirror symmetry to
  2×10⁻⁸ rad after ten seconds.
- **Probing a different object.** The probes called the cold solve capped at 40
  iterations, where cold needs ninety, and got the equilibrium separation for
  whatever incidence they landed on rather than the wing's actual separation
  state. So the derivative described a wing that did not exist, and moved 10%
  between consecutive intervals. Holding the live separation state and
  continuing each probe's own circulation made it converge in a handful of
  iterations — the suite got about ten times faster as a side effect.

**Rule:** a finite difference needs a perturbation *you* choose, centred, taken
about the state you actually have, with both solves converged. If a solve is
warm-started or iteration-capped for speed, a derivative built from it inherits
that error amplified by one over the step.

---

## 14. Every accepted quantity needs a bound, not just the one that bit you

The coupled solver rejected an aerodynamic solve whose *force* exceeded fifty
times weight. It never bounded the moment. One step near stall returned a
converged solve carrying 34 kNm of yaw against a `q·S·b` of 14 kNm; it was
accepted, and then every subsequent step failed and the safety envelope
faithfully held that number for ten seconds. 26 kNm on an inertia of 150 is a
turn rate of 100 rad/s, and then infinity.

The envelope worked exactly as designed and made things worse, because "hold the
last value" is only safe if the last value was checked. And the check that
mattered was missing on the quantity nobody had seen fail yet.

**Rule:** a validity gate covers a *state*, not a symptom. List everything the
gate lets through and give each one a bound with physical units behind it — here
`q·S·b`, which was already available. And a fallback that holds a previous value
must be as suspicious of that value as of the new one.

---

## 15. A coordinate is a claim about the world

Three separate defects, one root: a position that meant something different to
two parts of the program.

- **The frame flip.** The terrain frame defined +Y as route-left while the
  flight frame defined it as route-right, and nothing converted. The whole
  surveyed landscape was mirrored about the route axis. It was invisible in
  play because the mirroring was *uniform* — pull left, bank left, watch the
  world turn left — and only the relationship to real geography was wrong,
  which is precisely what ridge lift, rotor, wind bearings and circuits are
  built on.
- **The invented lane.** Two routes were translated as a group onto `y = -8500`
  to keep them off the surveyed grid, preserving their intra-valley geometry
  while placing them 20 km from the valley. The terrain there was an analytic
  proxy that put a 950 m landing field at 4683 m.
- **The Interlaken-relative weather.** Thermal triggers and every authored
  weather volume were expressed relative to one launch, with a single lane
  offset standing in for "somewhere else". Somewhere else was dead air.

**Rule:** a coordinate system is not a convention to be recorded, it is a claim
that has to be *checked against something outside the program*. The check that
finally held this together tests both halves in one place: that left brake turns
the wing toward -Y, **and** that the lake which is really west reads its real
surface elevation on that side. Either half alone passes happily while the world
is mirrored.

**Corollary:** never special-case one route's or one site's coordinates. Project
from the anchor. Every special case here was a 20 km error wearing a comment
explaining why it was fine.

---

## 16. The datum is part of the measurement

Local z in this project is metres relative to a landing field at 565 m — not
MSL, not above ground. Two rotor samples at the same "altitude" of z = 260
compared open valley air against a point thirty metres *inside a hillside*. The
0.82-against-0.00 that produced was recorded as evidence that the airflow model
was sided against the geography, cited in three documents, and carried for 25
commits after the bug it described had been fixed.

Sampled at a common height above ground, the field is symmetric and always was.

**Rule:** before comparing two measurements, state what they are relative to. In
terrain, "same altitude" is almost never the comparison you want; "same height
above ground" usually is. And a measurement offered as evidence for a defect
should be re-taken when the defect is closed — otherwise the number outlives the
bug and keeps arguing for it.

---

## 17. Verify the constraints, not just the code

A handoff document said Unreal build/cook was "quota-blocked until roughly
2026-08-05". It was inherited, repeated across several commits, used to justify
committing engine changes unverified, and used to write a whole tracking section
for the resulting debt. It was false. Unreal has no build quota. The first time
anyone ran the build it succeeded in 35 seconds, with every file that had been
marked unverified compiling clean.

This is the third instance of the same failure in this project, and the other
two are §12 and §16: a rotor measurement taken thirty metres underground that
argued for an already-fixed bug for 25 commits, and a diagnosis recorded as
"unverified" that parked a defect for a whole level. But a false *constraint* is
worse than a false *number*, because of how each fails:

- a wrong number is load-bearing — something eventually computes with it and
  disagrees;
- a wrong constraint is never computed with at all. The response to "we can't
  do X" is to route around X. Routing around leaves no contradiction to trip
  over, so the claim is never re-derived. It just quietly changes what gets
  built and what gets checked.

**Rule:** treat an inherited statement about the *environment* — a quota, a
missing tool, a broken pipeline, an unavailable dataset — as an untested claim
with a short expiry, not as a fact. Test it before designing around it. It costs
one command; believing it costs everything downstream of the workaround.

**Corollary:** when you do record a real constraint, record how it was
established and when, so the next reader knows what re-testing it would take.
"Quota-blocked until 2026-08-05" has no method attached and no way to check
whether it was ever true.

---

## 18. A symmetric case is a test instrument

Level 8's symmetric benchmark - the same descending air over both halves of a
wing - was written to check that a frontal collapse is symmetric. What it
actually did was find two defects nothing else could see, because a wing with
nothing asymmetric done to it has exactly one correct answer and any deviation
has a cause.

**The sum over a symmetric object need not be symmetric.** The collapse
solver's two half-wing averages were split by the sign of each section's
midpoint span fraction. With an odd section count one section sits on the
centreline, its midpoint lands within a rounding error of zero, and it was
counted whole on whichever side the arithmetic put it. Per-section states
agreed to 1e-15; the halves differed in the third decimal, and the flight model
turned on the difference. The fix is to weight each section by how much of it
lies on each side, from its extent rather than its midpoint - the same
weighting the aerodynamics already uses for brake.

**A Gauss-Seidel sweep has a direction, and the direction is physics here.**
`CanopyPressureSolver` computed crossport flow from the array it was writing,
so every cell saw its left neighbour already advanced and its right neighbour
not. Air crossed the span more easily one way than the other. Nothing in the
symmetric flight tests could see it, because they never asked two mirror cells
to agree. Reading neighbours from the start-of-step state costs one vector and
makes the answer independent of loop order.

**Rule:** for any solver over a symmetric object, run the symmetric case and
check the two halves against each other rather than against a tolerance on the
whole. It is the cheapest defect detector available, and it finds the class of
bug - loop order, index parity, accumulation - that physical intuition never
flags.

The same benchmark also says where the model stops being trustworthy, which is
worth as much: the two halves agree to 1e-15 through the fold and the first
second of the recovery, then diverge to 0.1 within two aerodynamic intervals as
the wing passes through the partly separated branch where the VSM does not
converge. A non-converged nonlinear solve turns rounding into a real
difference. That is the deep-stall problem showing up somewhere new, not a new
problem.

---

## 19. The one place the rule was not being checked

Guiding rule 3 says lines carry tension only, and the suspension network has
enforced it since Level 2 - the 120 mm of slack sewn into the brake line is in
its rest lengths, so hands-up transmits nothing. The aerodynamics never got the
message. It took the brake *handle position* as a camber change directly, so
the first 19% of the handle's travel deflected a trailing edge that no line was
pulling on.

Nothing caught it because every brake test used enough brake for the line to be
taut anyway, and the number that was wrong - the trailing edge deflection at
15% travel - was not the number any test read. It surfaced only when Level 8
asked the plan's own exit gate: does a brake pump inside the slack do anything
to a collapse? It did, and it should not have.

**Rule:** a guiding rule enforced in one subsystem is not enforced. Check it
where the quantity crosses into every *other* subsystem that consumes it - the
crossings are where an invariant quietly stops applying.

---

## 20. Agreement with a published number is not validation

The coupled model matched the manufacturer's 39 km/h trim speed almost exactly,
and had done since Level 7. It was wrong. The canopy was pinned straight below
the payload in body axes, which forced it to fly at 4.5 degrees of incidence;
a paraglider flies at about 11. The wing's lift curve - analytic thin-airfoil,
never validated against anything - is correspondingly too high. The two errors
were the same size and opposite signs, so the headline number came out right
and both of them stayed hidden for two levels.

Giving the wing and the pilot their real degree of freedom broke the agreement
immediately: trim fell to 29.5 km/h and the incidence rose to 11.8. The model
got *more* correct and the validation number got worse, which is the shape this
always takes.

**And the first diagnosis of the resulting gap was also wrong**, which is the
second half of the lesson. It blamed the analytic lift curve, on the reasoning
that the polars were the least validated thing in the stack. Testing that
against the published envelope refuted it in one run: the curve makes the
published trim CL at 5.3 degrees and the published top-speed CL at 0.5, and the
riser geometry spans most of the incidence range between them. The real defects
were in the pitching moment - four times too small, and never applied to the
wing at all - and the residue is a doubled pitch stiffness. Suspecting the
least-validated component is a reasonable prior and not evidence; the test that
distinguishes them costs one afternoon and the assumption costs a level.

**Rule:** a single scalar matching a published value validates nothing on its
own, because any two compensating errors can produce it. What validates is
agreement across quantities that cannot compensate for each other - trim speed
AND incidence AND the bar-to-trim ratio AND sink - and a stated mechanism for
each. When one number is right and the states behind it are not physical, the
number is a coincidence being maintained by a bug.

**Corollary, and the reason this one lasted:** the missing degree of freedom
was invisible because nothing asked for it. There was no test for "does the
wing move fore and aft relative to the pilot", because it was not a quantity
the model had. A model cannot fail a test for a state it does not represent -
which is why the accelerator changing the airspeed by exactly nothing went
unnoticed through two levels of gates.

---

## 21. Removing a wrong term can expose a second wrong term that was hiding behind it

The rigid motion counted gravity's restoring torque twice: once as a lumped
body's weight moment, once in the payload swing degree of freedom on the same
hinge. The wing carried roughly 14000 N·m/rad of pitch stiffness where the
lines provide 6300. This was written up, understood, and correct.

Fixing it worked, exactly as predicted — trim went from 31.9 km/h to about 40
against a published 39. And the wing then stalled at 35 seconds and stayed
stalled, from cold, hands off, in still air.

Not because the fix was wrong. Because with the double count gone, incidence is
set by the line spring alone, and the first attempt froze that spring at its 1 g
value while the aerodynamic moment it answers scales with dynamic pressure. A
spring that does not scale with anything loses to one that does.

**Rule:** a term that is provably wrong can still be load-bearing, and its load
is not always the one you removed it for. Before deleting a compensating error,
work out what ELSE is leaning on it.

**And the corollary that actually closed it:** when the replacement misbehaves,
measure the replacement rather than tuning it. Three of the four things that
made this work were measurements taken *after* the first attempt failed, and
none of them was guessable:

- the spring is proportional to LOAD, not constant — 3306, 6317, 11512 and
  15393 N·m/rad at ½, 1, 2 and 4 g;
- the probe that measures it needs 12000 iterations, returning 19849 at 120 and
  6371 at 48000, which is why the "just ask the live network" idea returned
  noise;
- the canopy pivots about a virtual hinge 6.62 m below itself, which is where
  its rotational inertia comes from.

The first attempt was reverted and written up. The second used those four
numbers and worked. The revert was not wasted; it was the measurement.

---

## 22. A simulation that starts mid-flight has to start trimmed

The canopy's pitch equilibrium is not its hang pose. This wing carries a 327 N·m
nose-down camber couple, so it sits about 3.3 degrees below where the lines
alone would hold it — and starting it at the hang pose is a 3.3 degree step
input into a spring with a damping ratio near 0.14.

That rings to twice the offset, which takes incidence from 6 degrees to 0.3,
which takes the LOAD off the lines. And because the line spring is geometric
rather than elastic, an unloaded wing has almost no pitch stiffness, so it
pitches further. Measured: 976 N and 5727 N·m/rad at a tenth of a second, 207 N
and 989 N·m/rad two seconds later, and the wing never came back.

None of that was the trim being wrong — the wing settles within half a km/h of
the published number either side of the excursion. It was a startup transient
with enough energy to knock the aircraft out of its own envelope.

**Rule:** the same reasoning that made this solver seed an inflated canopy
applies to every other state. An initial condition of zeros is not a wing, and
on a stiff, lightly damped axis it is not a small error either — it is an
impulse. This file already recorded the pressure version of this lesson
("the solvers were all right; the initial condition was not a wing") and it
took a second, more expensive instance to notice it was the same lesson.

---

## 23. Compare against the configuration the number was published in

The model was 9 km/h short of a published 39. About 2 of those km/h were not in
the model at all: the EPIC 2 ML's envelope is quoted at 105 kg all-up against a
90–110 kg certified range, and the solver's unballasted payload comes to 94.3.
Trim speed goes as the square root of wing loading, so that is 5.5% of speed
built into the *comparison* rather than into the physics.

**Rule:** a published performance number is a statement about an aircraft in a
configuration, and the configuration is part of the number. Before treating a
gap as a modelling error, check that the model is flying the same aeroplane at
the same weight in the same air. The calibration runner now ballasts to the
published weight and says so in its own output.

---

## 24. Check the sign against the world, not against the convention

The turn tests asserted that turn rate and bank carry opposite signs, on the
stated grounds that "positive bank is right tip up, which is a left turn". That
is backwards. The code says so plainly — `bankRad` is `asin(-span.z)`, so a
right tip *below* the horizon reads positive — and the assertion had been
passing because the old model banked the wrong way.

Two errors agreeing again, in the one place this project has explicitly warned
itself about twice ("it is the trap that convention has set twice before").

What settled it was refusing to reason about it: fly the wing, read the ground
track and the tip height as world vectors, and print them. Right brake turns the
track +1.217 rad toward +Y with the right tip 0.030 below the horizon; left
brake mirrors it to four digits.

**Rule:** a handedness claim written in a comment is a hypothesis. The only
reliable check is a quantity with no convention in it — where did it actually
go, in the world — and it costs about twenty lines to ask.

---

## 25. A constant in a struct cannot be a consequence of anything

The section polars had `stallMarginRad = 0.244` — where the section stalls,
measured from its own zero-lift angle. Brake moved the zero-lift angle, and the
stall margin above it was a constant, so **maximum lift came out identical at
every brake setting**. Not approximately identical: exactly, by construction.

A real deflected trailing edge raises maximum lift. That is most of what a flap
is for. The model could not express it, and the consequence reached the pilot:
40% brake — an ordinary EN-B input — walked the wing off the top of a curve
that never rose, and the whole flight envelope was hands-up to a quarter brake.

Nobody had to be wrong about flaps for this to happen. The number was fine as a
number; 14 degrees is what a thick cambered section does. What it could not be
was a *consequence*, and stall onset is a consequence — of the nose radius, of
the pressure gradient, of where the boundary layer runs out. Thin-airfoil theory
has no nose, so there was nowhere for the consequence to come from and the
constant filled the hole.

The test for this: **can this quantity change when the wing changes?** If the
answer is no and the real one would, the model has a hole with a plausible
number in it, and the number is hiding the hole rather than filling it.

## 26. A parameter fitted to one model's error moves when the error does

The design incidence — where the risers are cut, which sets where the wing hangs
— was identified by fitting the coupled solver's hands-up trim to the published
39 km/h. It was the one number in the model fitted to a published measurement,
it was documented as such, and it worked: trim landed at 39.1 km/h and three
numbers it had not been fitted to landed with it.

Replacing the section polars broke it. At the same 4.4 degrees the wing trimmed
at 11.96 m/s where it needs about 10.3 — a 16% error, from a parameter that had
been within half a percent. The fit had absorbed some of the analytic polars'
error, and when the error left, the absorbed part stayed.

What replaced it is a **design rule** rather than a fit: risers are cut so that
hands up, with no brake and no bar, the wing sits at its own best glide. That is
what the rigging angle exists to do. It is identified against the wing's own
aerodynamics, so it moves with the model instead of against it — and the four
published numbers still land, none of them fitted: trim +1.5%, incidence −1%,
sink −5%, glide +7%.

A fitted parameter is a loan against a model you are still changing.

## 27. The coefficient you cannot derive is the one to leave out

The section's profile drag is optimistic, and the largest known cause is the
momentum thickness the shear layer off the cell mouth carries onto the upper
surface. It is certainly not zero. Adding it would have taken whole-aircraft
glide from 10.32 to very near the published 9.5.

It was left out, because its size is a shear-layer spreading coefficient rather
than a piece of geometry — and swept over the range the literature supports, it
moves the section's drag **by a factor of five**. Any value in that range can be
justified in a sentence, and one of them lands on the published glide. Choosing
that one and writing the derivation above it would have produced a model that
agreed with the brochure and knew nothing.

What went in instead was the part that *is* geometry: there is a hole in the
nose, so the surface the flow reaches by crossing it has no laminar run. That is
worth about half the section's profile drag, it has no free coefficient, and the
8.6% of glide it does not explain is written down as item 12 with its two
candidates named.

The rule: a coefficient with a wide plausible range and a strong effect is not a
model, it is a dial. Leave it out and report the gap.

## 28. Two ways to be wrong can cancel, and replacing one exposes the other

The analytic table's brake pitching moment was `-0.60 * tau * delta`, where tau
is the thin-airfoil flap effectiveness. The effectiveness belongs to the *lift*
increment; applying it to the moment as well halved it. Thin-airfoil flap theory
gives about −0.55 per radian for this 22% flap, the solved section gives −0.61,
and the analytic table had −0.34.

So the section's brake moment was 45% too small — and the wing's *response* to a
brake moment is too strong, which is item 11. The two had been cancelling. Fixing
the section made brake worse: over the first fifth of the travel this wing now
accelerates, because the nose-down rotation outruns the camber.

That is not a regression to undo. It is the second error becoming visible, which
is §21 again in a different axis. The measurement that matters is the one that
says *which* half was wrong: the section moment now agrees with theory, so the
suspension side owns the rest.

## 29. Seed a boundary layer where the flow can keep it

The cell opening was first modelled by starting the upper surface's boundary
layer at the lip with the momentum thickness of the shear layer across the
opening, `theta = h/6`. Quadrupling the opening height changed the section's
drag by 2%.

An input that large cannot be that cheap, and the reason was where it was being
put. The lip sits in the steepest favourable gradient on the section — the flow
accelerates by a factor of three round the nose — and in a favourable gradient
`theta` decays as `Ue^-(H+2)`. The seed was being thrown away within two percent
of chord.

Physically the shear layer does not reattach at the lip; it reattaches
downstream, past the suction peak, in the decelerating region. Seeding it there
made the same input change the drag by a factor of five instead — which is what
led to leaving the coefficient out entirely (§27).

The general form: when a large input produces a small effect, find out whether
the model is telling you it does not matter, or whether you put it somewhere the
model can discard it.

## 30. Measure a steady state from a steady state

Brake was reported as making this wing accelerate over the first fifth of its
travel - a striking result, written up as a pitch-axis defect, and wrong. The
measurement ramped the brake in starting twenty seconds after the wing was
initialised, and the wing was still settling: hands up and untouched, over the
same two seconds, its incidence moved 0.4 degrees and its airspeed 0.2 m/s in
exactly the pattern that was attributed to the brake.

Settled properly - forty seconds hands up, an eight-second ramp, forty seconds
held, at each brake setting separately - brake slows the wing monotonically and
costs glide, which is what brake does.

The phugoid on this aircraft has a four-second period and a damping ratio near
0.09, so it takes the better part of a minute to disappear. Any steady-state
number read inside that window is a phase of the oscillation wearing a label.
The three gates that now exist for this - brake slows the wing, brake costs
glide, a firm input climbs - each fly their own settle rather than sharing one.

## 31. A model that stalls in the wrong place stalls everything downstream

The section was stalling at its nose. A turbulent boundary layer separating in
the first 3% of chord was being read as the section letting go, so one degree
of incidence took the flow from separating at 94% of chord to separating at 3%,
and the whole upper surface went at once.

The visible symptom was a stall angle that jumped around across the brake axis
- 10, 11, 7, 12, 3, 13 degrees - which looked like solver jitter and was
recorded as a minor limitation. It was not minor. Three unrelated-looking
failures were the same thing:

- the wing stalled at 35% of brake travel;
- a 4 m/s asymmetric gust folded the wing and then would not clear, because a
  wing that loses incidence anywhere loses its whole upper surface and cannot
  rebuild the speed to re-pressurise;
- a symmetric frontal stopped being symmetric, because the two halves fell off
  the cliff on different steps.

One fix - let a leading-edge bubble reattach, which is the turbulent twin of
the laminar short bubble the code already had - closed all three, and made
maximum lift and stall angle monotone in brake at the same time.

The lesson is about triage rather than aerodynamics: a numerical oddity in a
quantity everything else reads is not a small bug in that quantity. It is a
large bug everywhere, wearing the disguise of several small ones.

## 32. One control input, one length: check the budget

A brake line ends at 98% of chord. Pulling it does two things — it bends the
fabric aft of the brake attachment, and it rotates the whole canopy on its
suspension — and both were modelled, in different files, each helping itself to
the full 0.62 m of handle travel. The line network shortened the brake run by
all of it and rotated a rigid canopy; the section polars spent all of it again
bending the trailing edge into camber. Nobody had written down that these are
the same length.

The budget is the check, and it is arithmetic rather than aerodynamics. Add up
what each consumer takes and compare it to what the input supplies:

| | metres |
|---|---|
| handle travel | 0.620 |
| sewn-in slack | 0.120 |
| trailing-edge bend at full brake | 0.298 |
| left to rotate the canopy | 0.202 |

Full brake had been rotating the canopy 12.4° on a budget that allows 5.0°.

Two things generalise. First, a shared conserved quantity — a length, a mass
flow, an energy — is worth a budget table wherever two subsystems read the same
input, because neither subsystem is wrong on its own and no unit test of either
will catch it. Second, and this is §21 and §28 for the third time: fixing it
made the flying *worse*. Brake now slows the wing while lowering its incidence,
which is the wrong sign. The double count had been silently compensating a
suspension pitch response that is too weak, and the compensation was what made
the model look calibrated. The measurement worth having is the one that says
which half was propping up the other.

## 33. Before comparing two numbers, measure the noise they sit in

The pitch axis had a symptom everybody could state: brake slows the wing while
lowering its incidence, which is the wrong sign. It was written into the TODO,
the calibration report, the handoff and two commit messages, and a plan was
built on it — the suspension must be too weak, so find the lever.

There was no lever, and there was no sign error. Sixty seconds of still air,
hands up, no input: incidence is still swinging **0.60°** over the last ten
seconds, and under 25% brake it swings **2.26°**. The differences being read as
a sign error were 0.5–1.8°. The signal was smaller than the oscillation it was
being read from, and had been the whole time.

The calibration harness had been printing `NOT SETTLED` next to that exact row
for as long as the row existed. Nobody read the column, because the number
beside it looked like an answer.

Three things generalise.

**A difference is not a measurement until you know the spread.** Two settled
values and a subtraction is only meaningful if "settled" was checked rather
than assumed. The fix in the instrument was four lines: track min and max over
the last ten seconds and print them beside the mean. Every table this project
produces should carry that column.

**And then the same mistake was made again, one level up.** The first
correction concluded the wing had a *limit cycle*, because the spread at sixty
seconds did not go away and grew with brake, and because more `swingDampingRatio`
made it smaller. All three observations were real. The conclusion was wrong: the
wing settles, it just takes eight to sixteen minutes, and a damping ratio that
makes a mode decay faster looks exactly like one that suppresses a cycle **if
you only ever sample at a fixed time**. The second fix was to stop sampling at a
fixed time at all — settle to a criterion and report how long it took. A spread
measured at one horizon cannot tell a slow decay from a sustained oscillation,
and no amount of care about the first mistake protects you from making it again
with a bigger unit.

**§30 was the same lesson and it did not take.** "Measure a steady state from a
steady state" was written after a brake ramp was started before the wing had
settled. That was read as a procedural fix — settle longer — when it was really
a claim about instrumentation: you cannot know you settled without watching.
Settling longer does nothing when the aircraft has a limit cycle, and sixty
seconds is not more settled than thirty.

**The direction of a wrong answer is not evidence about its cause.** The
"suspension is too weak" theory was consistent with the observation, survived
several passes, and sent the search toward a number — the specific stiffness of
6.13 m — that turned out not to be an input to the solver at all. A theory that
explains a measurement you cannot actually resolve explains nothing.

**And the specific trap that made all of it possible: the right number about
the wrong mode.** The argument for "this cannot be a decaying oscillation" was
that the phugoid's period is 2.91 s and its damping 0.28, so twenty periods
would have killed it. Both numbers were real, both were measured by
`calibration_tests`, and both belong to a *different mode* — the wing swinging
against the pilot after a brake pulse, which that test names and gates
correctly. The mode actually decaying is the slow one: **period 16.4 s, damping
ratio 0.031**, incidence and airspeed in antiphase, needing eight to sixteen
minutes.

When a number is used to rule something out, check that it describes the thing
being ruled out. "The phugoid is well damped" was true of a mode that was not
the phugoid.

**The codebase knew, and knowing was not enough.** `CalibrationManeuver.cpp`
already named "a slow speed-and-incidence mode near twenty", already recorded
that fitting the full record returned "a 20.4 s pendulum with a damping ratio
of 0.05, which is a true statement about the wrong mode", and already windowed
the pitch identification to avoid it. All correct. But the settle time was
raised to 90 s and left there, and nobody carried the mode's own numbers across
to ask whether 90 s was enough for it — five and a half periods at damping
0.031 leaves a third of the transient. A fact recorded in one function does not
propagate to the constant three functions away that depends on it. When a
measurement lands, the useful question is which *other* numbers it invalidates.

## 34. Turn the disagreement into an exponent, then measure the exponent

The slow pitch mode had a lead rather than a question: "the phugoid period is
3.4× the classical value, find out why." That is not answerable as stated,
because "the pendulum changes things" explains any discrepancy of any size.

What made it answerable was noticing that the classical period rests on exactly
one assumption. Incidence is held fixed, so lift goes as V², and that is the
entire restoring force. Write the exponent as `n` in L ∝ Vⁿ and the phugoid is

    ω = g √n / V          ζ = (d/2) / ((L/D) √n)          D ∝ V^d

with the textbook pair at n = d = 2. Now "3.4× too long" stops being a mood and
becomes arithmetic: it says **n = 2/3.4² ≈ 0.17**, and n is measurable off the
trace without going near the period.

Measured, on a 1200 s hands-up run:

| | measured | classical | note |
|---|---|---|---|
| lift exponent n | **0.171** | 2 | 0.172 is what the period implies |
| drag exponent d | **0.313** | 2 | |
| period | 16.39 s | 4.80 s | 16.42 s predicted from measured n |
| damping ratio | 0.031 | 0.065 | 0.034 predicted from measured n and d |

**The mechanism, in one sentence: the pendulum does not hold incidence through
the oscillation, it holds LIFT.** Slowing down, the wing rotates nose-up on its
lines at −1.69°/(m/s) and recovers in incidence very nearly all it lost in
dynamic pressure. Lift varies by 0.97% of weight across the whole mode. The
restoring force that drives a phugoid is only what is left over, and what is
left over is almost nothing — hence a period 3.4× long and a mode that needs
eight to sixteen minutes to stand still.

Three things generalise.

**Convert a ratio into a parameter of the theory before hunting for causes.**
The 3.4× had been sitting in the docs for a level as "the best lead this item
has ever had", and it was — but it only became actionable once it was rewritten
as a claim about a number the trace could produce on its own terms.

**Measure the force off the PATH, not off the force bookkeeping.** Lift here is
`m(g cos γ + V γ̇)` and drag is `−m(g sin γ + V̇)`, both read from the velocity
vector. That is deliberate: the aerodynamic loads are the thing under
suspicion, so a check that reads them proves less. The path does not know what
the solver believes.

**Half an explanation was worse than none, and the arithmetic said so before
the run did.** Substituting the measured n while leaving the drag term at its
classical d = 2 predicts ζ = 0.221 against 0.031 measured — three times worse
than the untouched classical 0.065 it was meant to improve on. Fixing the
period alone would have looked like progress in the only column anyone was
watching while silently wrecking the other. Lift and drag are both flat against
speed here **for the same reason** — the incidence excursion that holds lift up
when the wing slows holds drag up with it — so the two exponents are one
finding, not two, and taking either without the other is not a partial result.

What this does NOT settle: whether n = 0.17 is *right*. It explains the model's
period from the model's own trajectory, which rules out the alternatives — a
mode governed by pitch stiffness rather than path-energy exchange would not have
produced this agreement — but both roads come off one trace. The open question
is now specific and physical instead of numerological: −1.69°/(m/s) is the
pendulum's tracking of apparent gravity, which is exactly what
`swingDampingRatio` stands in for, and whether a real wing tracks that
completely is checkable against a real wing.

## 35. A good explanation earns one prediction; spend it somewhere it can fail

§34 explained the slow mode's period and damping from one measured exponent. The
next question was the departure at `swingDampingRatio` 0.25, and the exponent
made a prediction about it for free: `ω = g√n/V` is oscillatory for n > 0 and
divergent for n < 0, and the wing sits at n = 0.171 — a fifteenth of classical
and very close to the sign change. If the ratio buys tracking, and tracking
flattens the lift curve against speed, the departure should be **n crossing
zero**: one number whose size is the period and whose sign is the stability.

It is not. n stays between 0.14 and 0.19 straight across the boundary, and the
ratio that departs soonest has the highest n in the table. What actually grows,
identified by its period, is a 3.6–5.7 s mode — the pendulum band, not the
16.4 s phugoid.

**The failure was worth more than the confirmation would have been.** It cost
one cheap run, and it removed a whole class of theories: the phugoid mechanism,
now understood in detail, simply does not reach the departure. Item 11 had spent
two levels treating "the slow mode" and "the departure" as one problem. They are
two, and only one of them is solved.

**Design the test so it can lose.** The prediction was stated before the run, in
the file, with the failing outcome written out beside the passing one — "if the
departing ratios still measure n > 0, this explanation does not reach it". A
prediction recorded only after the answer is known is not a prediction, and an
explanation that survives because nothing was staked on it has not been tested.

**And name the culprit, not just the acquittal.** "Not the phugoid" would have
been a weak result. Measuring the period of what does grow costs the same run
and points at the next mode. That needed a 10 Hz trace: the 1 Hz sampling that
the phugoid work ran on cannot resolve a 2.91 s mode at all, and undersampling
a mode into invisibility is precisely how this item lost two levels the first
time. When moving to a new mode, check the sample rate against *that* mode.

One thing did follow the ratio, and it is the part of §34 that survives: dα/dV
moves −1.27 to −1.72 as the ratio rises. The coefficient does buy pendulum
tracking. Tracking is just not what the departure is made of.

## 36. Five instruments for one number, and the fifth said "not reportable"

Measuring the fast mode's damping against `swingDampingRatio` should have been
the easy half of item 11. It took five instruments and still did not land. The
sequence is worth keeping, because every step was a smaller assumption than the
one before and four of them produced **confident, wrong output**.

| zero line | assumes about the interfering slow mode | result |
|---|---|---|
| window mean | it is an offset | 0 crossings at every ratio |
| fitted line | it is a ramp | 1–3 crossings; its bow left over |
| band pass | only that it is slower | unchanged; the extra "peaks" were the 10 Hz aero staircase |
| control run | nothing at all | clean response — and the mode is already gone |
| damped-sinusoid fit | — | fits at R² 0.9, and the answers are incoherent |

**The first output was the most dangerous.** Seven dashes reading "no
oscillation to identify" — which is indistinguishable from *this aircraft has no
fast mode* and was in fact *my zero line is in the wrong place*. What made it
recoverable was printing crossings, peaks and signal amplitude on the failing
rows. An instrument that cannot say why it failed is not reporting a result, it
is declining to speak, and those look identical in a table.

**Three of the five failures were diagnosed by guessing from summary
statistics, and two of those guesses were wrong.** Printing the trace ended it
in one run: the fast mode decays ~83% per cycle, so it is dead by 2.5 s, and a
centred 2.5 s moving average cannot report anything before 2.5 s. The filter was
being asked to recover a mode from the part of the record it had already ended
in — no cutoff fixes that. **Look at the signal before designing the filter.**

**The control run is the right idea and worth reusing.** Two runs identical
except for the input; the solver is deterministic, so the interfering mode is
bit-identical in both and subtracts exactly, with no assumption about its shape.
Its own assumption — superposition — is *checkable*, and it visibly fails past
4 s where the two runs diverge instead of converging. A checkable assumption
beats a filter cutoff, which is only ever a guess wearing a number.

**Knowing when to stop.** The final fits explain 90% of the signal and say
damping is positive at every ratio including the ones that depart, and
non-monotonic in the coefficient — more damper buying less damping. That is a
fit trading decay against frequency across two cycles, not a wing. The bar was
set before the run (the ratio-0.35 row must reproduce the suite's number), the
bar was missed, and the table is published marked NOT REPORTABLE rather than
quietly polished until it agreed.

**And it found something anyway, pointed at a number that is gated.** The
`CalibrationManeuver` pitch identification runs on a window starting 2 s after
the release — after this mode has largely ended. Its "period 2.91 s, damping
0.28" is therefore suspect. It has not been changed: doubt is not a measurement.
But a gate resting on a suspect number is worse than no gate, and that now has
to be resolved.

The real lesson is structural. Every time-trace method here ran aground on the
same rock — two modes an order of magnitude apart sharing one signal, the fast
one gone before the slow one has moved. That is not a windowing problem to be
out-thought. **Linearise about trim and take the eigenvalues**: every mode at
once, no excitation, no window, no filter, no superposition. Four instruments
were built to avoid a harder one that would have answered the question outright.

## 37. Ask the solver for its modes instead of watching for them

Four instruments failed to measure the fast pitch mode from a time trace (§36).
The fifth did not look at a time trace at all: perturb the settled aircraft one
state at a time, run each perturbation a fixed short time, difference against an
unperturbed run, and take the eigenvalues of the resulting transition matrix.

It worked on the first properly-signed attempt, and it reproduced the slow
mode's 16.39 s and 0.031 — measured off 27 peaks of a 1200 s run by entirely
different means — while also giving the fast mode, **1.86 s at ζ ≈ 0.09**, which
five time-domain attempts could not.

**Why it works where windowing cannot.** The two modes are an order of magnitude
apart and share one signal; the fast one is dead before the slow one has moved.
No window separates them, because there is no time at which only one is present
and large. The eigenvalues do not need one: every mode comes out of the same
matrix regardless of how big or long-lived it happens to be. Four instruments
were built to avoid a harder one that answered the question outright — and the
harder one was about 200 lines.

**The bug is the lesson.** The first version put a positive rotation about world
+Y into the attitude perturbation, while the readback took pitch from the
forward axis's rise — and a +Y rotation *lowers* that. Two of six states went in
inverted. The matrix came back with −0.96 and −0.99 on those diagonals, which is
an eigenvalue at μ ≈ −1, which reports a period of exactly 2T. So every mode
printed at exactly twice the sampling interval, and there is a real phenomenon
called aliasing that produces exactly that signature. **I wrote three paragraphs
of correct aliasing theory around a sign error.** A convention disagreeing with
itself does not look like a bug; it looks like physics, and it will happily
supply a mechanism for its own symptom.

The guard costs four lines and is now permanent: apply each perturbation, read
it straight back before stepping anything, and require +1 on its own state.
Anything else prints "every number below is void". Any code that perturbs a
state and reads a different function of that state wants this check.

**Printing the matrix is what found it.** The eigenvalues were plausible and
wrong; the matrix was obviously wrong at a glance, because over a tenth of a
second it should be near the identity and two diagonals were negative. When a
derived quantity is suspicious, print the thing it was derived from — the same
lesson as §36's "look at the signal", one level up.

**A check that varies two things at once is not a check.** The linearity test
originally halved the perturbation *and* changed the transition time, so nothing
could be concluded from it either way. It now halves the step at the same T as
the reference rows — and once fixed it earned its place immediately, by
separating the numbers that are converged from the one that is not: the fast
mode is identical to three digits under a halved step (ζ 0.0920 against 0.0922)
and both periods hold, while the slow mode's damping moves 19%. Without that
check all six numbers would have been reported with equal confidence, and one of
them does not deserve it.

## 38. Refuting one part of an eigenvalue is not refuting the mode

`pitch_eigenmodes --sweep` takes §37's spectrum through the departure boundary:
the same settled state, `swingDampingRatio` changed underneath it, eigenvalues
at twelve ratios from 0.90 to 0.10.

**The prediction, written in the file before the run, failed.** It was that the
*fast* mode goes unstable — the ratio is a link damping, the departure is fast,
`--departure` had identified the growing mode in the pendulum band. The fast
mode does not cross. Its real part moves from −0.357 to −0.291 across a ninefold
change in the coefficient, its period does not move at all — 1.86 s at every
ratio — and it is still firmly damped at 0.10 where the aircraft certainly
leaves.

Something does cross, and it is the **16 s phugoid** arriving by its damping.
The eigenvalues put the crossing between **0.28 and 0.25**; §39 measured it in
the time domain at **0.35 to 0.30** and showed why the eigenvalue is the one to
distrust here. The mode and the mechanism below stand; the interval is §39's. Its period tracks 23.9 → 14.0 s as the ratio falls and
its real part goes through zero right at the documented boundary, at both
transition times.

**A second instrument brackets the same interval sharing no arithmetic with an
eigenvalue.** Settling each ratio from scratch asks only whether a trim
*exists*: at 0.30 the wing settles and its spectrum is stable by a hair
(−0.008/s), at 0.25 it departs during its own settle at 348 s and 20° incidence,
so there is nothing there to linearise about. Two criteria, one interval. It
also retires the sweep's stated caveat — that everything is linearised about
0.35's trim rather than each ratio's own — as far as that can be retired, and
the drift column at full settle (1.8 × 10⁻⁵ rad/s, a hundredfold smaller than on
a short settle) says the point really is a trim.

**§35 acquitted the phugoid, and this is how a careful test still concluded too
much.** §35 predicted the lift exponent `n` would cross zero, measured it
holding at 0.14–0.19, and inferred the phugoid does not reach the departure. But
`ω = g√n / V` is a claim about the mode's *frequency* — n crossing zero is the
oscillation becoming a divergence. n staying positive rules that out and nothing
else. The phugoid arrives by the part of the eigenvalue ω says nothing about,
and the period staying near 16 s across the boundary — which §35 itself measured
— is exactly what that looks like. **A prediction about the real part and a
prediction about the imaginary part are two predictions; refuting one is not
evidence about the other.**

**The instrument's own uncertainty landed on the answer.** §37's linearity check
separated the converged numbers from the one that was not, and the one that was
not is the slow mode's damping. The crossing is a sign change in precisely that
number. The *ordering* is solid — monotone in the ratio, three independent
runs — so the finding stands as "the phugoid's damping crosses near the
boundary"; the interval 0.28–0.25 should not be quoted tighter. Being able to
say which half of a result is load-bearing is what the check bought.

**The corroboration failed and it is reported as failed.** The sweep flies each
ratio and fits the growth of a control-subtracted trace, so the spectrum would
have a trajectory behind it. It fitted at two ratios out of twelve, at R² 0.24
and 0.42, and disagreed with the eigenvalue eightfold. The diagnosis is the
instrument: it watches the *link* over 40 s, and the mode that crosses is a 16 s
oscillation growing at 0.008/s on a signal the fast mode has already left.
Undersampling a mode into invisibility, for the third time in this item, on the
third different instrument. The eigenvalue crossing currently has **no
time-domain confirmation**, and the sweep says so in its own output.

**Still unreconciled:** §35 identified the growing mode on a departing wing at
3.6–5.7 s, and no such mode exists in this spectrum at any ratio. That reading
was taken at large amplitude on a wing already leaving, where a linearisation
has no claim — so the two are not in contradiction, and they are also not
reconciled.

## 39. The corroborating instrument won, and it was the eigenvalue that lost

§38 ended owing a trajectory: the phugoid's damping crossing zero was a claim
made by eigenvalues alone, and the sweep's own flown column had failed to check
it. `--phugoid` is that check built properly, and the diagnosis of the failure
was specific enough to fix in three moves — **watch speed, not the link**,
because a phugoid is a height-speed exchange the link barely participates in;
**300 s, not 40**, because at 0.008/s the e-folding time alone is 125 s; and
**discard the first 25 s**, after which the fast mode is down by 2¹¹ and the
signal is the slow mode alone.

That third move is the one worth keeping. Five instruments in §36 could not
separate these two modes, and here the separation is free — because the *fast*
mode is now the contaminant rather than the target. Waiting out a mode is
trivial when you want the slow one and impossible when you want the fast one.
Same signal, same overlap; the easy direction is the one nobody needed until
now.

**It works, and the period is what proves it.** The fitted period tracks the
eigenvalue's to about 1% at every ratio with a clean fit (18.28 vs 18.56, 16.38
vs 16.60, 15.82 vs 15.99), so both instruments are demonstrably watching the
same mode. R² is 0.98–1.000 on the good rows, off 30-plus extrema, and halving
the perturbation moves the rate from 0.0115 to 0.0116 with the period
unchanged.

**The rates disagree, and there is an outside measurement to break the tie.**
At ratio 0.35 `pitch_axis_trace --slow-mode` measured this mode at 16.39 s and
ζ 0.031 off 27 peaks of a 1200 s run, sharing no code with either instrument.
The flown fit returns **16.38 s and ζ 0.0299** — 0.1% and 3%. The eigenvalue
returns ζ 0.0540, high by three quarters.

So **the eigenvalue's slow damping is biased toward stability**, exactly as
§37's linearity check warned before any of this ran: it was the one number that
moved with both T and step size, and the TODO already recorded that it brackets
the trace from above throughout. A caveat that was written down honestly, one
level early, turned out to name the number that would later be wrong. That is
what the caveat was for.

**The crossing moves: between ratio 0.35 and 0.30, not 0.28 and 0.25.** A third
line agrees and it fits nothing — the own-trim table settles 0.35 at 410 s,
fails to settle 0.30 in 420 s, and departs at 0.25. A marginally *growing*
phugoid is precisely why 0.30 has no settled trim to find.

**This explains the tuned coefficient instead of bounding it.** 0.35 is not a
safety margin chosen above a departure; it is approximately the smallest value
at which this wing's phugoid still damps at all, and the registry's `Tuned` 0.35
sits on that edge. It also redirects what is left of item 11: the target of
~0.06 from pilot and line drag cannot be reached by finding more *link* damping,
because at 0.06 the phugoid is far past its sign change. The missing mechanism
has to act on **speed stability** — the flat lift curve of §34 — not on the
link.

**Two rows are not evidence and are excluded in the output, not just here.** At
0.20 the fit returns a 7.87 s period, half the phugoid, at R² 0.375 off 69
extrema; at 0.25 R² is 0.495 and the period misses by 12%. Both are ratios where
the motion stops being one small oscillation inside the window, which is the
regime this fit has no claim on. The finding rests on 0.50 through 0.28, where
R² is 0.89–1.000 and the periods match — and those rows contain the crossing.

## 40. Right at a point, anti-correlated as a function

§34's phugoid model has two inputs — `ω = g√n/V` and `ζ = (d/2)/((L/D)√n)`. §35
spent its one prediction asking whether **n** crosses zero at the departure and
concluded the phugoid was not involved. §38 showed that inference was too wide.
But there is a sharper point neither made, visible in the formula itself:

**ζ as written cannot be negative.** With n > 0, d > 0 and a positive glide
ratio it is positive, full stop. The measured damping crosses zero between ratio
0.35 and 0.30. So either d crosses with it, or the model cannot produce this
instability at all. Nobody had ever measured d against the ratio.

**It does not cross. It rises — 0.281 → 0.459 as the ratio falls.** So predicted
ζ rises, 0.0341 → 0.0510, over exactly the interval where flown ζ falls through
zero, 0.1598 → −0.0167. Prediction and measurement move in **opposite
directions** across the boundary.

**The control is what makes that safe to assert.** The same fit's period
prediction lands at every ratio — 18.07 vs 18.28 flown, 16.44 vs 16.38, 16.02 vs
15.82, 16.52 vs 15.88 — 1 to 4%. §34's frequency claim reproduces across the
whole sweep off the same measurement, so the exponents are real and n is doing
real work. Only the damping half fails. Building the control in was the
difference between "the model's damping is wrong" and "my exponent fit is
broken", and those are indistinguishable without it.

**Right at a point, wrong as a function.** At the one operating point where §34
validated it, the formula still works: 0.0363 predicted against 0.0299 flown at
ratio 0.35 — §34's own 0.034 against 0.031. It is accurate at a point and
anti-correlated as a function of the parameter. Two very different kinds of
correct, and only the second was ever needed here. **An explanation validated at
one operating point has not been tested as a function of anything**, and the
project had been treating a point-fit as a mechanism for three levels.

**What it leaves.** Across the sweep n and d move 25% and 60% while the flown
damping moves by 0.18 and changes sign — almost none of that dependence lives in
the two-state theory. What the ratio changes is the **link**, and the link is
precisely the state the two-state phugoid does not have. The destabilisation is
a pendulum–phugoid *coupling*, which is why the six-state eigenproblem sees a
sign change and the two-state formula structurally cannot.

**Retraction, one level old.** §39 concluded the missing stabilising mechanism
"has to act on speed stability, not on the link". That was inferred *from* §34's
damping formula — the half this run shows is anti-correlated with the truth over
the very parameter in question. The conclusion inherited the error of its
premise. The quantity to go after is the pendulum–phugoid coupling itself.

## 41. A control that cannot fail decidably is not a control

§40 put the mechanism in the pendulum–phugoid coupling. Coupling is a property
of the mode *shape*, and the shapes were already in the transition matrix the
eigenvalues came from — so this cost no new flying.

**The control failed, and it failed undecidably.** It was stated in advance: the
1.86 s mode is the pendulum, so it must come back link-dominated, and the 16 s
mode speed-dominated. It did not — link/speed 0.42 for the fast mode against
0.46 for the slow. But *"this code is broken"* and *"my expectation about the
number was wrong"* both predict that, and the column as written could not
separate them. **A control has to be able to fail in a way that names what
failed.**

The fix was two more columns rather than a reinterpretation of the first:

- `residual` = ‖(Φ − μI)v‖/‖v‖, the arithmetic's own verdict with no physics in
  it. It comes back **1e-16 to 1e-12**. The eigenvectors are exact; the code was
  never the problem.
- `articulation` = |swing − attitude| / |attitude|, both radians, so there is no
  scaling choice to argue about. **Fast 1.07, slow 0.29.** The control passes on
  the scaling-free measure: the fast mode articulates the link against the wing
  nearly four times as hard.

So `link/speed` was the wrong instrument, not a wrong answer — surge is not a
phugoid-only coordinate, because both modes move the canopy's velocity. The
original column is kept, and kept honest, by printing what it did.

**The real prediction also failed: the phase does not move.** It sits between
−107.4° and −109.3° across the whole sweep — 1.9° — while σ goes −0.077 to
+0.008 and changes sign. At T = 0.10 it is −106.4° to −108.2°: the same span in
the same place, so this is the aircraft and not the discretisation.

**And the prediction was badly posed, which is the more useful half.** It
treated phase as the only way a coupling can change the energy it moves. Work
per cycle goes as amplitude × sin(phase), and the amplitude is *not* fixed —
link/speed rises monotonically 0.319 → 0.510 over the same sweep, 60%, while the
phase holds. So what is refuted is the **lag** version of the coupling
hypothesis, and a **gain** version is left standing.

That distinction is worth the run, because the lag version is the one the solver
itself blames: the link is damped against the world, and the comment on that
line calls the tracking lag "a cost paid knowingly". It was the natural suspect,
the phase column is exactly where it would have shown, and it does not show.

**Not established:** the gain story was not predicted in advance and is not
claimed. It is consistent with one column, which is precisely the trap §40 caught
§34 in. The test it needs is the **energy integral** — the work the link term
does on the phugoid over a cycle, evaluated on the eigenvector, a number whose
*sign* is the answer, and computable from what is already built.

**Recorded before it is explained:** as the ratio falls the link articulates
*less* against the wing, 0.383 → 0.266, not more. Less link damping does not
mean a link swinging more freely inside this mode. Nothing here explains that.

## 42. The mode looks like speed and listens like link

§41 owed an energy integral. There is a version of that question with no model
of energy in it at all: for a simple eigenvalue with right eigenvector v and
left eigenvector w normalised to w^H v = 1, `∂μ/∂Φ_ij = conj(w_i) v_j`. So a
*measured* change in the matrix — just difference Φ at two ratios — maps to the
change it makes in the growth rate, entry by entry, additively. Split the
entries into the wing's block, the link's block and the two coupling blocks and
each share has a sign.

**The built-in check earned its place in one run.** The four shares must add up
to the measured change in σ. The first run produced shares around 10¹³ against a
measured 0.013 — because the left eigenvector satisfies `Φᵀw = conj(μ)w` and I
had passed μ, returning the *conjugate* mode's vector, nearly orthogonal to v,
so the normalisation divided by almost nothing. Exactly §37's failure mode: a
convention disagreeing with itself, producing confident nonsense. **A
decomposition with no total to check against would have been read as a result.**
Fixed, the shares reproduce the measured Δσ to 7%, degrading to 14% on the
double-width step — first-order behaviour, which is its own validation.

**What is nearly tautological, said first.** That 99% of the movement enters
through rows 4–5 is barely a finding: `swingDampingRatio` appears in the link's
own update equation and nowhere else, so those are the only rows whose entries
change.

**The finding is the adjoint, and it is 0.985.** Changing rows 4–5 moves the
*phugoid's* eigenvalue only if the phugoid's left eigenvector has weight there.
That is not automatic, it is measurable, and it is essentially all of it: 98.5%
of the 16 s mode's adjoint sits on the link's two rows.

**So the mode looks like a speed oscillation and listens almost entirely through
the link.** Its right eigenvector is speed-dominated — articulation 0.29 against
the pendulum's 1.07, which is why every trace of it reads as a phugoid — while
its receptivity is link. Those two being different *is* non-normality, and the
conditioning |w^H v| = 0.10 says so directly; for a normal mode it would be near
1.

**That is the mechanism §40 and §41 were circling.** A coefficient living only in
the link's equations can take the phugoid's damping through zero because the
phugoid's adjoint is link. And §34's two-state theory cannot see any of it — not
because its aerodynamics are wrong, since its period prediction is still good to
1–4%, but because **it has no link row for the mode to listen through**. A model
can be right about what a mode looks like and structurally unable to say what
changes its stability. That is a sharper statement of §40's "right at a point,
anti-correlated as a function", and it says *why*.

It also fits §41 without being fitted to it: a receptivity that is fixed while a
gain changes is exactly a constant phase with a moving amplitude, which is what
the mode shapes showed.

**Not established.** cond = 0.10 means the individual shares are amplified
relative to a well-conditioned mode; the sum check validates the total, not each
share's precision. So the split *within* rows 4–5 — 0.0067 from the link's own
2×2 against 0.0052 from wing→link — is reported as roughly even rather than as a
ratio worth quoting. And link→wing contributing 0.5% says only that the link's
influence *on* the wing is not what changes; it does not say that influence is
small, which is a different measurement and was not made.

## 43. The lesson from two levels ago, not applied to the level in between

§42 reported that 98.5% of the phugoid's adjoint sits on the link's rows. Two
things needed testing before anything was built on that, and the first was an
audit of §42 itself.

**The audit: 0.985 does not survive a change of units.** A left eigenvector's
components carry units dual to the states', so summing |w₄|²+|w₅|² against the
rest adds radians to metres per second. Rescaling the states is a similarity
transform — the eigenvalues are untouched, it is a change of units and not of
physics — and under it the share at ratio 0.35 goes **0.9854 → 0.7786**.

§42's qualitative claim stands: 78% is still link-dominated, and nothing that
was concluded from it needs three digits. **The number does not stand, and it
was quoted to three.** §41's entire lesson was that a comparison depending on a
scaling needs a scaling-free control beside it — and §42 failed to apply it to
itself, one level later, in the same file. Knowing a lesson and applying it to
the thing in front of you are different acts, and the gap between them is one
level wide here.

What *is* untouched is the conditioning: `(Dw)^H(D⁻¹v) = w^H v`, invariant by
construction. **cond = 0.10 at the operating point**, so the non-normality that
made the mechanism intelligible in the first place is a real property and not a
unit artefact. The part of §42 that mattered is the part that was safe.

**The prediction also failed: the receptivity does not sit still.** Scaled, the
link's share of the adjoint falls **0.8898 → 0.7562** across the sweep, 15%,
while σ goes −0.077 to +0.008. Monotone, and in the direction nobody would have
guessed: **as the mode destabilises it listens *less* through the link.**

That is now the second measurement to run this way. §41 found the link
articulating *less* against the wing as the ratio falls (0.383 → 0.266); now the
adjoint's link share falls too. Shape and receptivity both move away from the
link while the mode goes unstable. So *"the phugoid destabilises because it
becomes more coupled to the link"* is not what is happening, and two independent
columns say so.

**What survives, stated carefully.** The link is the *channel* — the adjoint is
link-dominated at every ratio, 0.76 to 0.89 — and §42's split stands: the
movement in σ enters through the link's rows. But the *trend* in σ is not
carried by a trend in coupling strength, because that trend has the wrong sign.
What changes is the link's own dynamics, transmitted through a channel that is
itself weakening, and winning anyway.

For what comes next: the place a stabilising mechanism must enter drifts about
15% over the interval of interest. A design requirement with a drift in it
rather than a fixed target — worth knowing before something is built to hit it,
and not worth more than that.

## 44. One per cent of one entry is worth the whole coefficient step

Reading the solver's link update sharpened the design question. All of
`swingDampingRatio` enters at exactly one place —

```
linkRate = (linkRate + linkAngularAccel * dt) / (1 + 2 ζ ω dt)
```

— a single scalar gain on the whole increment. So it attenuates
`linkAngularAccel` too, and that term carries the *wing's* acceleration. **The
coefficient is simultaneously a damper on the link and a gain on the
wing-to-link coupling, and moving it cannot separate them.** That is why §42's
split found the movement spread across the link's own block *and* wing→link
rather than confined to the damper.

Separating them needs a perturbation the coefficient cannot make, and the matrix
allows what the solver does not: `∂σ/∂Φ_ij = Re(conj(w_i)v_j/μ)/T`, every entry
at once. Reported per unit *relative* change (§43's lesson, applied this time),
and finite-difference checked — 2% changes predicted against the eigenvalues of
the changed matrix agree to about 1%.

**The result is a comparison, not a number.** A 1% change to `d(swing)/d(swing)`
moves σ by **+0.0133**. Moving the tuned coefficient the whole way from 0.35 to
0.30 — the step that carries this wing from settling to not settling — moves it
**+0.0129**. One per cent of one matrix entry is worth the entire coefficient
step, and the top three entries are within a quarter of each other.

So the phugoid's stability sits on a knife edge with respect to the link's rows.
That is consistent with everything above rather than new: cond = 0.10 said the
mode is non-normal, and a non-normal mode is exactly one whose eigenvalue moves
far for a small change in the right place. §42 explained *why* a link
coefficient can move a speed mode; this says *how hard*.

**What it suggests about the item, as a hypothesis and not a conclusion.** Item
11 has been framed as finding a missing stabilising mechanism that would let the
ratio fall to the ~0.06 pilot and line drag imply. This says the boundary's
*location* is not a robust property — a 1% error anywhere in the link's rows
moves it by the whole 0.35→0.30 step. A number that fragile is more a property
of how the link is written than of a paraglider, so **"find the missing
mechanism" may be the wrong frame; a formulation whose stability is less
sensitive would be the real requirement.** Stated as a hypothesis, because §40
is what happens when a quantity that moved the right way once gets promoted to a
mechanism.

**One connection worth a measurement rather than an assertion.** The top entries
are in the swing *angle* row, and two of the three — `d(swing)/d(attitude)` and
`d(swing)/d(heave)` — are the link taking its lean from the wing's attitude and
vertical motion. That is the apparent-gravity tracking §34 and §35 measured as
dα/dV = −1.69°/(m/s), now appearing as the thing stability is most sensitive to
rather than as a trim slope. Same physics from two sides; not shown here to be
the same quantity.

**The caveat that limits all of it:** these are single-entry changes with the
other thirty-five held fixed. A real modification would move several entries
coherently and they could cancel. The map says where the mode is sensitive, not
what any physical change would do — checking a candidate means differencing its
matrix, which is what §42's split already does.

## 45. Looking where the light was, and a retracted claim coming back another way

§44 mapped `∂σ/∂Φ` over the **link's** rows, found them fiercely sensitive, and
concluded the departure boundary's location is a property of how the link is
written. It never looked at the other twenty-four entries. That control costs
nothing — the left and right eigenvectors are already in hand — and it changes
the conclusion.

**The single largest lever in the whole matrix is a wing row.**
`d(surge)/d(surge)` = **+1.631**, above every link entry, worth +0.0163 per 1% —
about one and a quarter of the entire 0.35→0.30 coefficient step. Finite
difference confirms it to 0.3%.

By block the link's rows are still the more sensitive *on average* — rms 0.822
against 0.433, a factor 1.9 — so §44 was not wrong about the block. But the peak
ratio is 0.81: the biggest single lever sits in the wing's own rows. **Fragility
here is a property of a non-normal mode, which moves far for a small change
anywhere it is receptive, and not a peculiarity of the link's formulation** —
the same fragility lands on core aerodynamics. §44's hypothesis that 0.35 is
"more a property of how the link is written than of a paraglider" is weakened by
its own missing control.

**And a retracted claim comes back by a different road, while the retraction
still stands.** §39 concluded the missing stabilising mechanism "has to act on
speed stability". §40 retracted it: it had been inferred from §34's damping
formula, which §40 showed is anti-correlated with the truth over this very
parameter. That inference was invalid and remains invalid.

But `d(surge)/d(surge)` *is* speed persistence — it is speed stability — and it
is now the largest single lever in the matrix, measured by something sharing no
arithmetic with §34. **A conclusion can be correct while the argument for it is
worthless, and arriving at it again by a sound route is not the same act as
un-retracting it.** The claim has support now; the reasoning that first produced
it is still wrong. Both belong in the record, and collapsing them would lose the
part that was actually learned.

The caveat from §44 is unchanged and limits this too: single-entry changes with
the other thirty-five fixed, and a sensitivity is not a mechanism.

## 46. An entry of Φ is not a coefficient of A, and the column said so first

§45 made `d(surge)/d(surge)` the best-measured lever on this mode, and the
obvious next question — written into the TODO by me — was whether it *is* §34's
drag exponent seen from the matrix side. In steady glide `D = W/(L/D)`, so
`D ~ V^d` gives `A₀₀ = −d·g/(V·(L/D))` ≈ −0.028 /s, and `A₀₀ = (Φ₀₀−1)/T` for
small T. Clean, closed-form, falsifiable.

**It did not resolve, and the failure is in the column rather than the
conclusion.** `(Φ₀₀−1)/T` is not a constant: −0.082 at T = 0.10, −0.152 at 0.30,
−0.19 at 0.50. A factor of two over the usable range, because `Φ = exp(AT)`
carries `A²T/2` and on a matrix this coupled that term is not small. **There is
no single A₀₀ to read off.**

A first draft of this check read the widest T, called it A₀₀, and would have
reported a factor of seven against the drag prediction — with a tidy story
attached about constrained versus unconstrained derivatives. The pre-written
verdict said "steady across T"; the table said otherwise on the first run. The
prose was wrong and the instrument was wrong, and the only reason that surfaced
is that the check printed every T instead of the one it wanted.

**Extrapolated properly: −0.052 /s against −0.028.** Same order, factor 1.84 —
not seven, and not one. The drag exponent accounts for a bit over half the surge
decay rather than all or none of it. That is a weaker answer than either draft
and it is the one the data supports.

**And the extrapolation is itself suspect, which is the part worth carrying.**
The aerodynamic interval is 0.1 s — loads are held between solves — so T below
that measures the hold rather than the wing, and T → 0 is *outside the model*
rather than merely hard to reach. A continuous A does not cleanly exist for this
solver at the scale the extrapolation reaches for.

**What this does to everything above: nothing, and the reason matters.** Every
other result in this file is built from *eigenvalues* of Φ, which are exact
properties of the map over whatever T was used, and were cross-checked at
several T throughout. Individual *entries* of Φ compared against continuous-time
formulas are the fragile construction, and this is the only place that was
attempted. §45's sensitivity ranking is untouched: it compares entries of one
matrix with each other at one T, and needs no continuous limit at all.

**What would settle it:** a real matrix logarithm of Φ, giving A with no small-T
expansion, evaluated at or above the aerodynamic interval. Contained arithmetic,
and the honest next step rather than another reading of the same contaminated
column.

## 47. There is no continuous A, and that is why one number never converged

§46 could not read a continuous coefficient off `(Φ₀₀−1)/T` and named the fix: a
real matrix logarithm, `A = log(Φ)/T` by eigendecomposition, no expansion in T.
Built, with three checks — and it asks a bigger question on the way, because **A
describes the solver only if one A reproduces Φ at every T**, which is not
guaranteed when aerodynamic loads are held for 0.1 s between solves.

**The arithmetic is sound.** Imaginary residual 1e-10 or better, so the
conjugate pairs cancel as a real Φ demands; and `exp(AT)` rebuilt by
scaling-and-squaring — independent of the eigendecomposition, not run backwards
— reproduces Φ to 1e-11.

**And there is no single A.** A₀₀ = −0.035 at T = 0.10, −0.021 at 0.25, **+0.006
at 0.50**. It changes sign; the spread is 290% of its own mean. Φ(T) is not an
exponential family, this solver is not a sampled continuous linear system at
these scales, and §46's comparison cannot be made this way at all. The 0.1 s
hold is the obvious culprit and the sizes fit: a 0.5 s step contains five holds
and cannot look like a smooth flow.

**This explains the oldest loose end in the file.** §37's linearity check
separated the converged numbers from the one that wasn't, and the one that
wasn't is the slow mode's damping — moving with T *and* with step size, written
down as "the one number still to be pinned". **It was never going to be pinned.**
A rate extracted from Φ(T) depends on T when Φ is not an exponential family, so
that number has no T-independent value to converge to. Three levels treated it
as a measurement needing improvement. It was a category error, and the fix is to
quote it with its T rather than to chase it.

**What survives, checked rather than asserted.** Three things were at risk and
they fare differently:

- **Periods** are T-stable — 1.86 s at every T tried, throughout.
- **Rates** move ~20% with T. That is the uncertainty the docs already carry on
  the slow mode's damping, now explained rather than outstanding.
- **Entries** move by factors, as A₀₀'s sign change shows.

§45's ranking is built on entries, so it was in real danger. **It holds:** at
T = 0.10 the top four entries come back in the same order as at 0.25, the
link/wing rms ratio is 1.87 against 1.89, the peak ratio 0.82 against 0.81. All
magnitudes scale, because a per-step sensitivity carries a 1/T — but §45
compared entries of *one* matrix against each other at *one* T, and that is
exactly what a common scale factor leaves alone. The construction turned out to
be robust for a reason that was not the reason it was chosen.

**One observation, flagged as not predicted in advance.** A₀₀ at T = 0.10 is
nearest the drag exponent's −0.028, and there is an a priori argument for
preferring that T — 0.1 s is the model's own load interval, so sampling there is
least distorted. But that argument was available before the run and was not
made, so it is a hypothesis about which T to trust, not a result. Picking the T
whose answer one likes is how §46 nearly reported a factor of seven.

## 48. The gate was measuring its own window, and the fix was to assert less

`calibration_tests` published "Pitch: period 2.91 s, damping 0.28" as this
wing's pitch mode, gated on `0.5 < period < simplePendulum` and
`0.02 < ζ < 0.9`. §37's eigenvalues had said 1.86 s at ζ 0.09 for eight levels
and the two were never reconciled, because reconciling them meant changing
gated behaviour.

**First correction, to my own description of it.** I had been calling this a
conflict with consequences for the build. It is not: the bounds are wide enough
that 1.86 s and ζ 0.09 both pass. The gate was never red and would not have gone
red. What was wrong was narrower and worse — **a number produced by an arbitrary
window was being published as physics**.

**The evidence, which cost one run.** Exporting `IdentifyOscillation` so the test
can vary the window it was previously handed:

| window | period | damping | oscillations |
|---|---|---|---|
| 2.0–3.5 s | — | — | **0** |
| 2.0–5.0 s | 1.42 | 0.39 | 1 |
| 2.0–7.0 s | 1.51 | 0.23 | 1 |
| **2.0–9.0 s** | **2.91** | **0.28** | 1 |
| 2.0–12.0 s | — | — | **0** |
| 2.0–20.0 s | — | — | **0** |

Never more than one oscillation; nothing at all on either side; and the shipped
9.0 s window is the **outlier** among the ends that find anything. The published
number ranged over a factor of two and vanished twice, purely on where the
window stopped.

**The replacement I tried first also failed, and that is the useful part.** A
decay check needs no period: peak-to-peak excursion early versus late. It gives
0.948° over 2–5 s against 0.911° over 8–11 s — **ratio 0.96**, no decay at all —
because by 8 s the swing signal *is* the 16 s mode and the fast one is long
gone. That is §36's result arriving from a third direction: two modes an order
of magnitude apart share this signal and no window separates them. The brake
pulse cannot measure the fast mode, full stop.

**So the gate now asserts less.** A released pulse leaves a measurable, bounded
swing transient, and the identifier's period — whatever window produced it —
still sits under the simple-pendulum bound, which is the one external reference
here that does not require resolving the mode. The period and damping are
printed, labelled as window identification, with the sensitivity table beside
them and the modal answer named as the authority.

**Weakening a gate to what its data supports is the point, not the cost.** The
old bounds were wide enough to admit both the right answer and the wrong one —
so they never distinguished them, and their passing was never evidence. A test
that cannot fail differently for a wrong answer is decoration, and this one had
been decorating a published number for eight levels.

## 49. The third mode was two modes and a peak counter

§35 identified what grows on a departing wing by counting peaks of incidence
over the last 40 s before it lets go, and reported **3.6–5.7 s**. Nothing in the
spectrum is there — the pendulum is 1.86 s at every ratio, the phugoid runs 23.9
down to 14.0 — and ten levels of modal work never accounted for it. It was
carried as unreconciled.

§48 supplied the precedent without meaning to: a peak counter on a two-mode
signal returned 2.91 s, a number between 1.86 and 16.4 belonging to the mixture
rather than the aircraft. 3.6–5.7 sits in the same gap. So run §35's own
algorithm on a signal built from the modes that *are* known — 1.86 s decaying at
σ −0.32, 15.4 s growing at +0.008 — swept over the amplitude ratio and phase,
with **no third mode in the generator at all**.

**It produces the band, and not narrowly:** 3.64, 3.65, 3.69, 4.93, 5.17 s.
§35's range bracketed from both ends. And at one mixture it returns **2.91 s** —
the number the calibration gate published for eight levels. The same identifier,
the same two modes, the same spurious gap, arrived at twice from different
records by different levels of this project.

**The giveaway is what the apparent period does across the sweep:** 2.46, 2.69,
2.91, 3.64, 3.65, 3.69, 4.93, 5.17, 7.30, 7.38, 7.45, 7.70, 9.87 s. It settles
nowhere. It is whatever the amplitude ratio and phase happen to be — and §35
reported a *range across ratios*, 3.6 to 5.7, which is exactly what a
mixture-dependent artefact looks like and not at all what a mode looks like.
That range had been read as scatter in a measurement; it was the signature of
the thing being measured not existing.

**What is established and what is not.** This shows the identifier *can*
manufacture that band from modes already known to be present. It does not show
that is what happened in the real trace — §40's lesson is that a mechanism which
reproduces a number is not thereby the mechanism. What it does is make the
artefact explanation cheap and available, against a rival requiring a mode no
eigenvalue has found at any ratio, at any transition time, in ten levels of
looking.

**What would settle it:** project the real departure trace onto the two known
eigenvectors and measure the residual. Small residual means the trace *is* the
two modes and the third never existed. That is a merge of two test binaries
rather than new physics, and it is the honest way to close this instead of
leaving it at "available".

**The general lesson, which this project has now paid for three times.** A
peak-counting or crossing-counting identifier run on a signal containing two
modes an order of magnitude apart does not report either one, and does not fail
either — it reports a confident intermediate number that moves with the mixture
and the window. §36 lost four instruments to it, §48 found it in a shipped gate,
and §49 finds it behind the last unexplained result on this axis. **When a
period lands between two known modes and moves with the conditions, suspect the
identifier before the aircraft.**

## 50. The flown departure is in the span of the two modes, and the counter mis-reads that too

§49 left the third mode "retired as an artefact, available but not proven": the
peak counter had been shown to manufacture 3.6–5.7 s out of a *synthetic*
two-mode signal, where the modes went in by hand. The test it named was the
projection — take the real trace, resolve it on the eigenvectors, and see what
is left over. `pitch_eigenmodes --project` is that merge.

**The construction is the part that decides whether it means anything.** Φ was
built by differencing a perturbed run against an unperturbed one, so it is the
Jacobian about a *trajectory*, not about a fixed point. The trace has to be
built the same way or the projection is of the wrong object: two runs at the
same ratio from the same settled state, one kicked by 2° of pitch, differenced
sample by sample at 10 Hz. §35's own trace cannot be used — it is a single
cold-start run whose deviation is measured from nothing in particular. What is
shared with §35 is the identifier and the modes, which is what is on trial.

**Result: the residual is a few per cent.** After removing the two conjugate
pairs — 1.86 s and 15.4 s — what the flown deviation carries outside their span
is 0.4–5% of its norm through the small-motion part of the run, at both T = 0.25
and T = 0.10. A six-state linear system has nowhere else to put a third mode, so
if one were driving this it would appear here. It does not.

**And §35's counter mis-reads the rebuild exactly as it mis-reads the trace.**
Run on the flown signal it returns 6.34 s; run on that same signal *rebuilt from
the two modes alone* — 1.86 s and 15.4 s by construction, nothing else — it
returns 7.80 s (6.90 s at T = 0.10). Both land in the gap, neither is a mode.
This is the step §49 could not take: the gap period now comes out of flown data
and out of a two-mode reconstruction of that same flown data, alike. The band is
what this identifier does to these two modes.

**Two honest limits, both structural rather than tidyable.**

*The residual climbs late — 13–29% past about 3° of deviation.* That is where a
linearisation stops claiming anything, so it is not evidence of a missing mode;
it is also not evidence of anything else, and the window is printed rather than
trimmed to flatter the number. The t = 0 row is near 1.00 for a bookkeeping
reason worth stating: the kick lands mostly on the two fast *real* roots (−0.97
and −5.97 /s), which are not "the two modes" and are gone within seconds. The
residual statistic starts at 25 s for that reason and the table still prints
from zero.

*The rate is not tested by this, and the first version of it pretended
otherwise.* Linearising about 0.35's shared trim — as every other check here
does, for free — gave a flown +0.0017 /s at ratio 0.30 against an eigenvalue of
−0.0084: a sign disagreement bought entirely by the reference run drifting
toward its own trim. Re-run about each ratio's own trim it becomes −0.0007
against −0.0084, the right sign at a tenth of the size, on a deviation of
0.05–0.13° that is near the floor of differencing two nonlinear runs. **And at
0.25 there is no own trim to use** — the aircraft leaves through 20° at 348 s of
its own settle, which is what being past the stability boundary means. So the
departing case necessarily keeps the drifting reference. What `--project`
supports is the **span**; the rate belongs to `--phugoid` and `--sweep`, which
measure it without this construction.

**The item this closes and the one it does not.** §35's growing 3.6–5.7 s mode
is now accounted for end to end: the identifier manufactures the band from these
two modes (§49), and the flown trace contains nothing but these two modes (§50).
Item 11's pitch axis has no unexplained observations left. What remains is an
absence rather than a mystery — there is still no *mechanism* for why 0.35 is
needed, only a measured sensitivity (`d(surge)/d(surge)`, +1.63 per unit) and a
list of eliminated candidates.

## 51. The rejected damper fails through the same mode, and reverses the sign of the coefficient

The solver damps the link against the **world**. Its own comment says the
friction physically sits between the link and the **canopy**, records that the
canopy-referenced version was tried, and rejects it in one clause: the pendulum
was left free to be dragged by the wing and the aircraft left the envelope in
twenty seconds. Item 11 carried that for four levels as "still unclaimed either
way" — because a wing that departs tells you it departed and nothing about
*why*, and the instrument that would say why did not exist when the rejection
was made. The hook to point it at the alternative is three lines in the solver;
`--damper` is the measurement.

**The question was not which damper is better.** It was whether the two failure
modes are the *same* failure. The world-damped solver departs by the 16 s
phugoid's damping crossing zero, and its pendulum stays at 1.86 s and firmly
damped from ratio 0.90 to 0.10 (§38). If the canopy version departs by the fast
mode instead, item 11's missing mechanism has a second constraint nobody knew
about.

**The prediction was stated first and it was wrong.** "The fast mode goes
unstable" — a dragged pendulum is a pendulum-mode statement, and twenty seconds
is eleven pendulum periods against about one phugoid period. The fast mode does
not go unstable. It gets *more* damped, −0.31 → −0.68 /s at ratio 0.35, and
stays damped at every ratio. The rejection's wording described what a departure
looked like from outside, and the prediction inherited the description.

**What goes unstable is the 16 s mode — the same one — at σ +0.156 against the
world's −0.021** at ratio 0.35. The control makes that defensible rather than a
coincidence of periods: articulation 0.22 canopy against 0.29 world for the slow
mode, 1.08 against 1.07 for the fast. Same shapes, same roles. **The pitch axis
has one failure, and it is the phugoid**, under either damper.

**The finding nobody was looking for: the coefficient's sign of effect
reverses.** In the canopy frame σ *rises* with damping — +0.136 at ratio 0.10 to
+0.194 at 0.90 — and the mode is unstable at every ratio tried. There is no
value of this coefficient that stabilises the aircraft in the canopy frame. The
flown runs agree, and in the same order: 0.90 departs in 17 s, 0.35 in 27 s,
both cold, against a world-damped 0.35 that settles at 410 s. That ordering —
more damping departing sooner — is something no world-referenced run does at any
ratio.

**What it buys item 11, as a constraint rather than a candidate:** whatever
stabilises this wing **cannot be link–canopy friction, at any magnitude**. The
friction is real and it is where the solver's comment says it is; it does not do
this job. What the world-referenced damper supplies is a rate measured against
the *inertial* frame, and the phugoid needs that. The candidate list shortens by
one, and it is the one the solver itself had nominated.

**What is not established.** The canopy spectra are linearised about the *world*
solver's 0.35 trim, because the canopy solver has no trim to linearise about —
it departs from a cold start at every ratio tried. Those eigenvalues therefore
describe a point the canopy aircraft does not fly through, the same caveat §50
carries at a departing ratio. The flown column stands on its own, and the two
agree in sign, in ordering, and with the twenty seconds recorded four levels ago.

**A stale comment found by doing this.** The lines that damp the link carried a
paragraph asserting the damper acts on the rate *relative to the canopy* — the
opposite of what the line does, and of what the paragraph eight lines above it
says. The two were written a level apart and the comment was never brought back
when the change was reverted. Corrected rather than deleted: the argument in it
is the real case against the world frame, and §41 is what answers it.

## 52. The construction probes were ringing, and the gates were written against the ring

Item 14 wanted the 336 ms the suspension network spends solving itself at
construction. The stated idea was warm-starting: eleven 12000-iteration
relaxations of the *same* network at neighbouring loads, each started cold,
against a `solver_lod` measurement showing a warm-started in-flight network
converging in 40 iterations rather than 12000.

**The stated idea does not work, for a reason worth keeping.** The 24 expensive
solves are the stiffness probes, which *impose* a canopy attitude 0.02 rad from
the hang pose and read the moment. Warm-starting one from the free solve hands
it the answer to a different question — the displacement being measured is
exactly what the warm start does not have — and it converged no faster and less
accurately (0.37 N of node residual against 0.011). Warm-starting the *free*
brake sequence does help, 1.44° → 0.16° of pose error at 4000 iterations, but
not enough to cut the iteration count: matching the shipped accuracy still needs
12000, and this aircraft has already shown a hundredth of a degree of incidence
moving a held collapse from 0.83 to 0.30 of fold.

**What is actually wrong is the damping, and the probe says so in one column.**
Held at 0.02 rad the pitch probe reports +177%, −176%, +27% and −13% of its
converged value at 500, 1000, 2000 and 4000 iterations. That is not a solve
creeping up on an answer; it is one ringing about it, and 12000 iterations stops
it somewhere on the way down. Damping the ring — fewer iterations at lower
velocity retention — is both faster and closer:

| setting | pitch k / roll k at 1 g | worst error, 0.5–4 g | ms |
|---|---|---|---|
| reference, 48000 | 5749.8 / 8253.6 | — | 1385 |
| shipped, 12000 at 0.999 | 5739.3 / 8116.4 | 1.58% | 336 |
| held 6000 at 0.995 | 5750.4 / 8242.1 | 1.40% | 221 |
| held 8000 at 0.997 | 5750.4 / 8261.8 | 0.24% | 260 |

**The control that makes this a numerical fact rather than a tuning:** the
equilibrium cannot depend on the fictitious damping, and it does not — 48000
iterations at 0.999 and at 0.995 agree to 0.16% across every load, by two very
different paths. It is now gated in `suspension_tests`, along with the shipped
error, bounded where it is so it cannot grow.

**And none of it ships, which is the actual finding.** Rebuilt on the
better-converged probes, two known-limitation gates change their verdict — and
so does the *reference itself*, which is what settles it:

- the deep frontal's peak rotation is bounded at 2.4 rad/s. Shipped it is 2.06.
  Converged (48000) it is **3.61**. At held 6000/0.995, **3.90**. At held
  8000/0.997, **71.0**. Four settings whose static outputs agree within 1.7%,
  spanning a factor of thirty-four on this number.
- 40% brake departs nose-*up* at +91° shipped, at +91° converged, and nose-*down*
  at −90° at held 6000/0.995. The direction of a departure past loop-gain-one is
  decided below the accuracy of any of these numbers.

So the shipped settings are green partly because their mis-convergence damps an
event that the converged model does not damp. **A gate calibrated on a
mis-converged model does not become wrong when the model improves; it becomes a
decision nobody has taken.** Both defaults were left where they were, both
measurements were written into the code beside them, and the hook that produced
them (`ConstructionProbe`) ships with defaults identical to the old behaviour.

**The general lesson.** A fixed iteration count is a claim about convergence
that nothing checks. This one had a comment justifying it — "19849 at 120, 9228
at 2000, 6371 at 48000, converged to within 0.3% by 12000" — and the numbers in
that comment are the ring, read as a curve creeping up on a limit. Three
readings of a decaying oscillation look exactly like convergence if you never
ask what is between them.

## 53. The aircraft amplifies tenfold while every mode decays — and it does it less as it destabilises

Ten levels of this axis have asked eigenvalue questions: which mode crosses,
when, through what channel. Eigenvalues are the whole story only for a *normal*
system, where the modes are orthogonal and nothing can grow while all of them
decay. §42 measured this mode's conditioning at |wᴴv| = 0.10 and §43 showed that
figure is invariant under rescaling, so this system is not normal — and nobody
had asked the question that fact opens.

**G(t) — the largest factor the linear system can amplify *any* disturbance by
over a time t — is the largest singular value of Φⁿ, not an eigenvalue of it.**
The prediction was stated first: if the departure is an eigenvalue phenomenon, G
stays near e^(σt) with no hump where the eigenvalues call the aircraft stable;
if it is transient growth, G is well above 1 there and **rises as the ratio
falls** — which would at last explain §39's recorded anomaly, a flown boundary
at ratio 0.35–0.30 against an eigenvalue boundary at 0.28–0.25, the eigenvalue
biased stable against the flown wing.

> **§79: most of that anomaly was not the aircraft.** The flown boundary moves
> with the aerodynamic hold and the eigenvalue's does not, so the 0.35–0.30
> figure was schedule-contaminated and the converged flown boundary is
> 0.30–0.28. A gap remains — the eigenvalue is biased stable by a constant
> factor — but it is much smaller than the one this paragraph set out to
> explain. The G result below is measured directly and stands on its own.

**Half of that is confirmed and it is new.** This aircraft amplifies a
disturbance about **tenfold while every one of its modes is decaying** — G 9.0
to 13.9 across the sweep against an eigenvalue prediction of 0.96 to 1.00. Ten
levels of eigenvalue work could not have seen it, because an eigenvalue is a
statement about long times and this happens in **half a second**.

**The control matters here more than usual and it passes.** §47 established that
the *entries* of Φ move by factors with the transition time, and a singular
value is a statement about entries in a way an eigenvalue is not — so a growth
factor peaking on the first step of the horizon is exactly what a sampling
artefact would look like. Measured at two transition times it is the same number
in the same place: 13.94 at T = 0.25 against 14.53 at 0.10, peaking at 0.5 s
against 0.4. It is the aircraft.

**The other half fails, and in the direction that decides it.** G *falls*
monotonically as the ratio drops — 13.94 at 0.90 to 9.00 at 0.25 — while σ
climbs through zero. And its peak is at half a second where the departure takes
tens of seconds. The aircraft has *less* of this amplification as it becomes
less stable, so transient growth is not the mechanism and §39's anomaly is still
unexplained.

**The pattern is now three for three, and it is the most useful thing here.**
Every measure of how strongly the link and the wing interact falls as this
aircraft destabilises:

| quantity | ratio 0.90 → 0.25 | section |
|---|---|---|
| link articulation against the wing | 0.383 → 0.266 | §41 |
| adjoint receptivity through the link | 0.89 → 0.76 | §43 |
| transient amplification G | 13.9 → 9.0 | §53 |

Three independent quantities, three different instruments, one direction — and
the wrong one for any story in which the wing destabilises because it couples to
the link *more*. **The mechanism is inside the link's own dynamics, transmitted
through a channel that is getting weaker, and no measurement of coupling
strength will find it.** That is a constraint on where to look next, arrived at
by three failed predictions rather than by one successful one.

## 54. The basin does not shrink, it grows — and 0.30 has no trim to have a basin around

§53 closed the last *linear* escape route for §39's recorded anomaly: the flown
boundary sits at ratio 0.35–0.30 and every eigenvalue instrument puts it at
0.28–0.25, the linear picture optimistic against the wing. Transient growth is
real and tenfold and moves the wrong way with the ratio, so it is not the
explanation.

That left exactly one hypothesis, and it was written down in `SettleAt`'s own
comment several levels ago as fork (b) and never followed up: **the trim is
stable to small disturbances and the aircraft leaves when something large enough
happens.** A basin that shrinks as the ratio falls would produce precisely the
observed bias, because every instrument on this axis is local and a local
instrument cannot see the size of a basin.

`--amplitude` tests it twice, and the two share no arithmetic.

**1. σ at amplitude.** Linearise not about the trim but about states displaced
along the phugoid's own eigendirection, at growing size. The prediction: if the
mechanism is nonlinear, the phugoid's real part climbs toward zero with
amplitude. **It falls, monotonically, at every ratio.**

| displacement | σ at 0.90 | σ at 0.35 |
|---|---|---|
| 0 (the trim) | −0.0752 | −0.0201 |
| 0.5 m/s of surge | −0.0691 | −0.0242 |
| 1 m/s | −0.0618 | −0.0308 |
| 2 m/s | (drift-void) | −0.0508 |

At 0.35 the damping ratio climbs 0.053 → 0.25 across that span. A displaced
wing is *more* damped, not less. Note the 0.90 column moves the other way and
still never approaches zero — the effect is small there and large at 0.35, which
is the opposite of a basin closing in.

**2. The basin itself, with no linearisation in the answer at all.** Kick each
own trim along the phugoid direction and fly it out; the smallest kick that
departs is the basin radius, bisected three times.

| ratio | survived | departed | departure alpha |
|---|---|---|---|
| 0.90 | 4.0 m/s | 4.5 m/s | −2.6° |
| 0.50 | 4.0 m/s | 4.5 m/s | −2.5° |
| 0.35 | **6.0 m/s** | **6.5 m/s** | −5.8° |
| 0.30 | no trim in 3600 s | — | — |

**The basin grows as the aircraft destabilises**, and it is enormous throughout:
4.5–6.5 m/s of surge on a 10.5 m/s trim is a disturbance of 40–60% of flight
speed. The wing nearest the boundary tolerates the largest one. Two instruments,
one direction, and it is the wrong direction for the hypothesis — so the
finite-amplitude story is **eliminated** rather than left open, which is the
same kind of result as §51 and worth as much.

**Two things fell out that nobody went looking for.**

**The basin edge is nose-down.** The incidence one step after the departing
kick is −2.6°, −2.5° and −5.8° against a 5.1° trim. The large-disturbance limit
is the low-CL loop-gain path — the same one item 11 already blames for full bar
and for 40% brake — and not a stall. The brake limit and the disturbance limit
may be one boundary, which would mean the three symptoms item 11 tracks have two
causes rather than three.

**A guess about the flown boundary, made here and refuted here.** §39 read
"0.30 does not settle in 420 s" as the wing approaching its boundary. At the
eigenvalue's own −0.008/s the approach to trim has a 125 s time constant, so a
genuinely stable wing would *also* miss that budget — which would have made part
of §39's boundary a settling-time artefact and explained the bias with no new
physics. The budget was raised to 900 s, then to 3600 s. **0.30 still does not
settle, and it does not depart either, and the excursion is BIGGER at the longer
budget:** the unperturbed drift in surge is 0.148 m/s per second at 900 s and
0.681 at 3600 s. Reading those as phugoid amplitude gives roughly 0.4 m/s
growing to 1.7 over 2700 s, σ ≈ +0.0005/s — marginal, phase-noisy, and positive.
0.30 is not settling slowly; it is slowly not settling. The artefact explanation
is dead, and §39's anomaly survives its fourth attempt.

**Method note, because it is what made the 0.30 rows honest.** The instrument
prints the drift of the *unperturbed* run beside every σ. A state displaced from
trim is not a trim, so a Jacobian there is a frozen local one, and past the
point where the drift over the transition time is comparable to the perturbation
sizes the row describes the trajectory rather than the aircraft. At 0.30 the
drift is 0.68 m/s per second against a 0.05 m/s perturbation, so **that whole
column is void by the instrument's own criterion** and is not quoted above. The
same measurement returned a 20.8 s mode at the 900 s budget and a 15.3 s mode at
3600 s, which is the corroboration: there is no trim there, so there is no
spectrum there either.

## 55. The missing drag destabilises the wing, and the classical phugoid relation has the sign backwards here

§54 finished the search of the link: the basin is enormous and *grows* as the
aircraft destabilises, so nothing about the disturbance explains the boundary
either. Every pointer left over said **wing** — the largest sensitivity in the
matrix is `d(surge)/d(surge)`, which is speed stability, and §54's basin edge is
nose-down at the low-CL loop gain rather than at a stall.

The wing has exactly one known, named, quantified defect, and it had been
sitting in a different item for four levels. **Item 12:** the solved section
runs 0.0157 at trim against a published 0.018–0.025, glide is 10.96 against 9.5.
A phugoid's damping is classically `ζ = CD / (CL √2)` — one over root-two times
the glide ratio — so a wing that glides a sixth too far has a phugoid damped a
sixth too little, and `swingDampingRatio` could have been paying for it. Nobody
had put the two items in the same run, because they were two items.

**The prediction was stated with its size, which is what made it fail usefully.**
Restoring the published glide is a 15% increase in CD/CL, so: ζ 0.065 → 0.074, σ
at ratio 0.35 from −0.0201 to about −0.023, and since 0.024 of σ is worth 0.05
of ratio, that is **about 0.006 of coefficient** against the 0.29 item 11 needs.
The expected result was elimination by a factor of fifty.

**The magnitude estimate held. The sign did not.**

| ratio | σ clean | σ with drag restored | outcome clean | outcome with drag |
|---|---|---|---|---|
| 0.35 | −0.0201 | **−0.0159** | settled | settled |
| 0.30 | no trim | no trim | not settled | **DEPARTED settling** |
| 0.25 | no trim | no trim | departed | departed |

Restoring the missing drag makes the phugoid **less** damped — σ falls 21% in
magnitude where it was predicted to rise 15% — and ratio 0.30, which on the
clean wing merely failed to settle inside an hour (§54), now departs during its
own settle. The boundary moved, and it moved the **wrong way**, by about 0.02 of
coefficient. Drag is eliminated as the mechanism twice over: it is an order too
small, and it has the wrong sign.

**The useful half is a forecast rather than a null result.** Item 12 is not on
item 11's critical path in the direction anyone would have guessed — whoever
closes the drag deficit should expect to **lose** pitch stability and to need
*more* artificial swing damping, not less. That is now a written prediction
against a future change, which is the most a null result can be worth.

**Why the classical relation fails here, as a lead and not a measurement.**
`ζ = CD/(CL√2)` is derived for drag acting at the centre of gravity of a rigid
aircraft along its flight path. On this aircraft the section drag acts at the
**canopy**, about 6.6 m above the pilot on a pendulum, and extra drag up there
pitches the wing against the payload rather than simply opposing the speed
excursion. If that is what is happening, the destabilising quantity is not drag
but the **height at which it acts** — which is testable immediately and cheaply,
because installed drag is already modelled separately at the harness
(`InstalledDragSpec.harnessDragCoefficient`). Put the same total extra drag on
the *pilot* instead of the canopy: if σ moves the other way, the mechanism is
the moment arm, and that is a wing–link term nothing on this axis has measured.
**This is the next test on item 11 and it is the first new handle in three
levels.**

**A second finding, which belongs to item 12 rather than item 11.** The offset
was calibrated by bisection against the glide, not stated — and landing the
glide **overshoots everything else**:

| wing | glide | speed | sink | incidence |
|---|---|---|---|---|
| as it flies | 10.97 | 10.60 m/s | 0.962 m/s | 4.92° |
| drag restored, Δcd 0.01035 | 9.49 | 9.93 m/s | 1.040 m/s | 6.36° |
| published | 9.50 | 10.83 m/s | 1.140 m/s | — |

Glide lands and the trim speed leaves — 9.93 against a published 10.83, where
the clean wing's 10.60 was nearly right — while sink is *still* low at 1.040
against 1.140. So a uniform section-drag offset is the wrong **shape** for the
missing term, and item 12's "done when glide lands inside the published figure"
is not a sufficient criterion: this offset satisfies it while making two other
published numbers worse. The missing drag has to land glide, speed and sink at
once, and a flat Δcd provably cannot.

**Method note.** The claim that 0.30 got worse is entirely a difference between
"did not settle" and "departed while settling". The first version of the table
printed both wings' σ and only the drag wing's outcome, which hid exactly that
comparison — the numbers were right and the table was not entitled to the
conclusion drawn from them. §54 spent 3600 s of settle budget establishing that
distinction; a table that drops it is a table that has un-learned it.

## 56. The same drag stabilises from one end of the link and destabilises from the other

§55 found that restoring the wing's missing drag makes it *less* stable, which
is backwards from `ζ = CD/(CL√2)`, and named the suspect: that relation is
derived for drag acting at the centre of gravity of a **rigid** aircraft. This
one is two bodies on a link, and section drag acts at the canopy 6.6 m above the
pilot, where extra drag is also a pitching moment.

The experiment is the only one that separates a drag term from a moment-arm
term: the same aircraft, the same total drag, applied at the two ends of the
link. `InstalledDragSpec` already carries drag on the **pilot** — its own
comment warns that reaching the canopy through the lines is the whole point of
having a pendulum — so the counterpart knob is a Cd·A there. **Both wings are
bisected to the published glide of 9.5**, so they carry the same drag at trim
and differ only in where it acts.

**The prediction was confirmed, and by more than its own margin.**

| ratio | σ clean | σ canopy | σ harness | outcome harness |
|---|---|---|---|---|
| 0.35 | −0.0201 | −0.0159 | **−0.0328** | settled |
| 0.30 | no trim | no trim (departed) | **−0.0200** | **settled** |
| 0.25 | no trim | no trim | — | not settled |
| 0.20 | — | — | — | DEPARTED settling |

The same drag makes the phugoid **63% better damped** at the pilot and 21%
*worse* at the canopy. The destabilising quantity is the **moment arm, not the
drag**, and that is a wing–link term nothing on this axis had measured in eleven
levels of looking at coefficients.

**The boundary moves one full step, and this is the number item 11 wants.** The
harness wing settles at 0.30 with σ −0.0200 — indistinguishable from the clean
wing's −0.0201 at 0.35 — fails to settle at 0.25, and departs at 0.20 and below.
So the boundary goes from 0.35–0.30 to **0.30–0.25**: one 0.05 step, against the
0.29 item 11 needs explained. **That is about a sixth of it.** The first
mechanism in three levels to move the boundary the right way, and it is physics
rather than a coefficient — but it is a sixth, and calling it the answer would
be the same mistake §35 made with the phugoid.

**The control is what makes it a comparison.** The harness bisection landed
0.199 m² of Cd·A against an independent equal-force estimate of 0.279 m² —
same order, which says this is one drag moved rather than two different ones —
and the two wings trim in nearly the same place. The harness wing sits at
10.64 m/s and 4.84° against the clean wing's 10.60 and 4.92°, so §55's confound
(any drag moves the trim) is small here in a way it was not for the canopy wing
at 9.93 m/s and 6.36°.

**The item 12 finding in the same table may be the larger one.** Against the
published 9.50 glide, 10.83 m/s and 1.140 m/s sink:

| where the missing drag is put | glide | speed | sink |
|---|---|---|---|
| on the section (item 12's assumption) | 9.49 | 9.93 | 1.040 |
| on the harness | **9.51** | **10.64** | **1.113** |

Item 12 has assumed for four levels that the deficit is the shear layer off the
cell mouth — a **section** term. A wing with the same deficit at the **harness**
lands all three published numbers at once where the section version lands one
and breaks the trim speed. That is a strong hint about where the missing drag
lives, and the installed-drag inputs are stated dials of exactly the kind that
hide one: `harnessAreaM2` is 0.32 and `lineProjectedFraction` is 0.35, neither
solved.

**What this does not show, stated because the result is flattering enough to
over-read.** It does not prove the missing drag is at the harness. 0.199 m² is a
59% increase on the harness Cd·A, which is a lot to attribute to an
underestimate, and a combination of section and installed deficits remains open
and is frankly more likely than either alone. What it shows is that a
harness-side deficit is **consistent with three published numbers** and a
section-side one is not — which is a constraint on the answer, not the answer.

**A hook that defaults to off has to be bit-identical when it is off, and
reassociating a product is not.** The harness knob was first written as
`q * (A * Cd + extraArea)`, which with the extra area at zero is physically the
same line and numerically a different double from `q * A * Cd`. It changed no
physics and still **failed a coupled check**: the deep symmetric frontal is a
partly separated solve with no steady state, so a last-bit difference in harness
drag walks into a different fold path and the mirror-symmetry check goes. The
fix is to add the extra term in a separate guarded statement. This project now
has four hooks of this shape — `SetSwingDampingRatio`,
`SetLinkDampingReference`, `SetSectionDragOffset` and this one — and each claims
in its own comment to be bit-identical to having no hook at all; the other three
are right by how they happened to be written rather than by design. Only the
full suite caught it, which is the argument `Tools/check-build.sh` makes in its
own header for never skipping it. (`x + 0.0` *is* exact, which is why the
section-drag hook does not have the problem.)

**The mechanism is named but not yet measured**, and that is the next step
rather than a conclusion. Drag at the canopy and drag at the pilot pitch the
aircraft opposite ways about the link, so the incidence correction the phugoid's
speed oscillation receives has opposite sign in the two cases. That sketch
predicts something checkable with machinery that already exists: `--shape`
measures the phase of link swing against surge, and §41 framed the whole
coupling question in those terms. Run it on the harness wing. If the phase moves
the way the sketch says, the mechanism is closed; if it does not, this is a
measured effect with the wrong story attached to it.

> **RETRACTED by §57.** It ran, and it was the wrong story: both phases move the
> *same* way, by a few degrees, against a σ that moves ±60%, at two transition
> times and with converged residuals. Everything else measured in this section
> stands — which is why the paragraph above was labelled a sketch and kept
> separate from the numbers. §57 has what the phase table shows instead, and it
> is better than the sketch was.

## 57. The mechanism sketch is retracted, and the mode shape says something better

§56 measured a real effect and attached a story to it, and labelled the story a
sketch: canopy and pilot drag pitch the aircraft opposite ways about the link,
so the phugoid's incidence correction changes sign. The prediction that follows
has a sign in it — relative to the clean wing, the two drag wings' link-swing
phase against surge must move in **opposite directions**. §41 is why phase is
the right question: what decides whether the link's motion removes energy from
the phugoid or feeds it is *when* the lean happens, not how big it is.

**It fails.** Both phases move the *same* way, and both barely move:

| wing | period | σ | phase | link/speed | articulation | residual |
|---|---|---|---|---|---|---|
| as it flies | 16.60 s | −0.0201 | −107.8° | 0.4587 | 0.292 | 2e−14 |
| drag at canopy | 13.66 s | −0.0159 | −111.6° | 0.5447 | 0.250 | 3e−14 |
| drag at harness | 16.83 s | −0.0328 | −109.1° | 0.4609 | 0.290 | 2e−14 |

Canopy −3.8°, harness −1.3°, same direction, against a σ that moves ±60%. At
the second transition time it is −3.9° and −1.4°, so this is the aircraft and
not the sampling, and the residuals say the eigenvector solves converged. **The
sketch is retracted. The measurement it was attached to is untouched** — the
boundary really does move a full ratio step — and this is the fifth retraction
on this axis, the same shape as §40 retracting §34's damping formula.

**What the table shows instead is worth more than the sketch was.** Look at the
harness row against the clean row in every column *except* σ: link/speed 0.4609
against 0.4587, articulation 0.290 against 0.292, period 16.83 against 16.60.
**The harness wing damps 63% harder with a phugoid mode shape indistinguishable
from the clean wing's.** The canopy wing, meanwhile, is the one that restructures
the mode — link/speed up 19%, articulation down 14% — and it damps *less*.

So the stabilisation does not run through the link's participation in the mode
at all. Eleven levels have looked for a link mechanism, and the thing that
finally moved the boundary changes the link's role in the mode by half a
percent. **That is a strong constraint: whatever the harness drag is doing, it
is doing it inside the mode rather than to it.**

**And articulation tracks stability a fourth time.** §53 tabulated three
independent measures of link–wing interaction falling as the aircraft
destabilises with the ratio. Here the axis is different — drag location, not
ratio — and the pattern holds: the destabilised wing (canopy, σ −0.0159) has
articulation 0.250 and the stable one (harness, σ −0.0328) has 0.290. Four for
four, on two different axes, and still no mechanism that uses it.

**The next hypothesis, named and NOT asserted, because that is the mistake this
section exists to correct.** `ζ = CD/(CL√2)` assumes drag at the centre of
gravity — and on this aircraft the pilot essentially *is* the centre of gravity,
105 kg of system against 5.1 kg of canopy. So "harness drag stabilises because
it is the classical term applied where the classical term belongs" is the
obvious next story. **It does not fit the numbers as it stands:** the classical
relation predicts +15% of damping for this glide change and the harness wing
delivers +63%, four times over. Something is amplifying it.

That is testable rather than arguable, and the test turns two points into a
function: `InstalledDragSpec.harnessBelowCanopyM` is the arm the harness drag
acts on, 7.8 m. Hold the drag fixed and sweep the arm. If σ is monotone in the
arm and heads toward the canopy wing's value as the arm goes to zero, then
height is the whole story and the 63% is the arm doing work the classical
relation has no term for. If σ is flat in the arm, the harness and canopy cases
differ by something other than height — which would mean §56's headline is also
mis-explained, though still measured.

## 58. σ is linear in the height the drag acts at, and that height is what `swingDampingRatio` has been standing in for

§56 had two aircraft and a factor of two between them; §57 killed the story
attached to that and left the fact sharper. This turns the two points into a
function.

**The sweep that was named was not the one available.** `harnessBelowCanopyM`
looks like the arm to vary, but it is the pendulum's own geometry — moving it
moves the baseline harness drag and the line moment too, so the sweep would
change three things and credit one. And a pendulum's bob is at the end of the
link by construction; there is no height for it to be at. What *is* free is how
much of the extra force pushes the bob, with the rest applied at the canopy
where its arm is zero. The resultant's height moves continuously with that
fraction.

**Every row re-bisected to the published glide**, because the first pass held
the drag area fixed instead — on the argument that the total force does not
depend on the fraction, which is true of the force and false of the aircraft —
and the glide walked 9.51 to 9.72 across the sweep. The table's own control
caught it.

| share at pilot | Cd·A | glide | period | σ | speed | incidence | articulation |
|---|---|---|---|---|---|---|---|
| 1.00 | 0.199 | 9.51 | 16.83 s | −0.0328 | 10.64 | 4.84° | 0.290 |
| 0.75 | 0.213 | 9.51 | 15.74 s | −0.0280 | 10.42 | 5.28° | 0.273 |
| 0.50 | 0.227 | 9.49 | 14.79 s | −0.0240 | 10.20 | 5.72° | 0.260 |
| 0.25 | 0.237 | 9.50 | 14.07 s | −0.0196 | 10.02 | 6.13° | 0.250 |
| 0.00 | 0.246 | 9.49 | 13.46 s | −0.0160 | 9.85 | 6.54° | 0.242 |

**σ is monotone and very nearly linear in the fraction** — successive steps of
0.0048, 0.0040, 0.0044, 0.0036 — across a factor of 2.05 in damping at constant
glide. The application point is the whole story.

**The zero end is an independent check and it passes to 0.6%.** Fraction 0 puts
a bare force at the canopy reference; §55's canopy wing puts distributed drag
through the VSM at the section control points, changing the local flow and the
solve. They are different constructions of "drag at the canopy" and they land at
σ −0.0160 and −0.0159.

**The confound that cannot be removed, and why it does not explain the trend.**
Incidence is an output, not an input, and it drifts 4.84° → 6.54° across the
table. But the model's own pitch loop gain goes as 1/CL², so higher incidence is
higher CL is a *lower* loop gain is *more* stable — and the high-incidence end
of this sweep is the *least* stable. The trim drift works against the σ trend
rather than producing it. That is an argument and not a measurement, and it is
labelled as one in the output.

**Articulation tracks stability a fifth time**, 0.290 → 0.242 down the same
column. Five instances now, on three axes — ratio (§53), drag location (§57),
and application height here — and still nothing that uses it.

**What this means for item 11, which is the point.** Item 11 has said since it
was written that the honest ratio, *estimated from what physically damps the
swing — the pilot's drag on an 8 m arm, plus the lines sweeping — is nearer
0.06* against the 0.35 the model needs. This sweep measures that same mechanism
from the other end: drag applied at the bob damps the phugoid, drag applied at
the canopy does not, and σ is linear in how much of it acts at the bob. **The
pilot's own drag is not an analogue of `swingDampingRatio`; it is the physical
quantity the coefficient is standing in for**, and the model has been carrying
too little of it — which is item 12, at the harness rather than the section
(§56).

That is the first mechanism-shaped answer item 11 has had. It is worth being
precise about how much it explains: the glide-landing amount of harness drag
buys 0.05 of the 0.29 (§56). The *form* is now known and the *magnitude* is not,
and those are different claims.

**The next test follows directly and is the one item 11 has wanted for eleven
levels.** Sweep the harness drag well past the glide-landing value and find the
ratio boundary at each. If pilot drag is the mechanism, there is some amount of
it at which the aircraft is stable at a ratio near the honest 0.06 — and that
number, against a real pilot's measured drag, is the whole of item 11's
question. The wing stops being physical partway up that sweep, which is fine:
it is a mechanism probe, not a wing.

## 59. Pilot drag reaches 0.06 at ten times a real pilot — and the term item 11 names is not in the model at all

§58 gave the *form* of item 11's answer and not the magnitude. This asks the
magnitude directly: more and more drag on the pilot, and the lowest ratio the
aircraft still settles at with each.

| extra Cd·A | × base harness | glide | boundary ratio | incidence |
|---|---|---|---|---|
| 0.199 | 0.6× | 9.51 | 0.30 | 4.84° |
| 0.400 | 1.2× | 8.38 | 0.25 | 4.75° |
| 0.800 | 2.4× | 6.74 | 0.20 | 4.57° |
| 1.600 | 4.8× | 4.76 | 0.10 | 4.20° |
| 3.200 | 9.5× | 2.86 | **0.06** | 3.47° |

The base harness is 0.32 m² at Cd 1.05, so 0.336 m² of Cd·A before any of this
is added.

**The middle outcome of the three, and it bounds item 11 from both sides.** Drag
at the bob *can* pay for the whole coefficient — the boundary walks all the way
to item 11's honest 0.06 — but only at **3.2 m² of extra drag area, ten times
the modelled pilot, on a wing gliding at 2.86 against a published 9.5.** That is
not a paraglider. At a *credible* pilot drag — the 0.199 row, which is also the
only row that still glides 9.51 — it buys 0.05 of the 0.29. **A sixth**, exactly
as §56 measured, and now with the rest of the curve behind it. Roughly, each
doubling of pilot drag buys another 0.05 of ratio.

So "the installed drag is too low" is eliminated as the *whole* answer to item
11, quantitatively rather than by argument. Something else supplies five sixths.

*(The σ-at-boundary column of the raw output is not a smooth function and should
not be read as one: it is σ at whichever coarse grid point happened to be the
last one that settled, not at the boundary itself.)*

**And then the actual finding, which came from reading the solver rather than
the table.** Item 11's own estimate is *the pilot's drag on an 8 m arm, plus the
lines sweeping*. Both of those are **swing dampers**: they oppose the pilot's
motion *relative to the air*, which on a swinging pendulum includes L·q̇.

The model has neither. `harnessDragBody` is built from the **aircraft's**
airspeed vector and applied to the bob:

```
harnessDragBody = airspeedBody * (-harnessDragN / airspeed)
```

so it is a force along the flight path whose magnitude depends on flight speed —
and it **cannot oppose the swing**, because the bob's own velocity through the
air never enters it. The lines' drag never reaches the bob at all: it becomes a
moment on the canopy. There is no aerodynamic term anywhere in this solver
proportional to the swing rate. **The only thing proportional to the swing rate
is `swingDampingRatio` itself.**

That reframes the whole of §§55–58. What those measured is real but is *not* the
mechanism item 11 names: it is a speed-dependent force applied at the bob, which
damps the phugoid by modulating the pendulum's driving term through the speed
oscillation. The genuine swing damper — drag opposing L·q̇ — has never been in
the model, which is exactly why a coefficient has been standing in for it since
Level 3.

**The next iteration is therefore not another probe. It is the missing term.**
The pilot's velocity through the air is the aircraft's airspeed plus the link
rate crossed into the link arm; the drag on it follows from the same
`InstalledDragSpec` already in the file, and the lines sweeping is the same
integral over the line length. Neither is a dial — both are consequences of
geometry the solver already has. And item 11's 0.06 is a *prediction* of what
they are worth, made before any of this, which makes the next run a test rather
than a calibration.

## 60. Item 11's founding estimate is wrong: the pilot's swing drag is worth 0.01, not 0.29

§59 found that the two terms item 11's opening estimate names — *the pilot's own
drag on an 8 m arm, plus the lines sweeping* — were never in the solver. This
puts the first one in and measures it. It is a **test of a standing prediction**:
item 11 has said since Level 3 that the ratio implied by those terms is "nearer
0.06" than 0.35, derived by hand from the physics long before §§53–59 existed and
never fitted to anything since.

| ratio | clean | σ clean | pilot airflow | σ pilot |
|---|---|---|---|---|
| 0.35 | settled | −0.0201 | settled | **−0.0230** |
| 0.30 | not settled | — | not settled | — |
| 0.25 | DEPARTED | — | **not settled** | — |
| 0.20 and below | DEPARTED | — | DEPARTED | — |

**The control passes exactly** — σ −0.0201 at 0.35, 0.30 unsettled, 0.25
departing, which is what this axis has reported for eleven levels — so the
change moved only what it was supposed to.

**The term is real, stabilising, and small.** σ improves 14% at ratio 0.35 and
0.25 is upgraded from departing to merely not settling. **The boundary does not
move.** Calibrated against §56, where 63% of σ bought one 0.05 step of ratio,
14% is worth about **0.01 of coefficient against the 0.29 needed** — three per
cent of the problem.

**So item 11's founding estimate is refuted, and it is off by a factor of about
twenty-five.** `swingDampingRatio` is *not* standing in for the pilot's
aerodynamic swing damping. That sentence has been at the top of the item since
Level 3, it framed every attempt on this axis, and it is wrong.

**The lines will not rescue it, by the same arithmetic that should have been
done first.** Line drag area is 254 m × 0.00105 m × 0.35 projected = 0.093 m², so
Cd·A 0.098 against the harness's 0.336 — 29%. But the lines sweep at a velocity
proportional to distance along the arm, so their damping torque carries an
∫s³ds/L⁴ = ¼ weighting the bob does not. **The lines sweeping should therefore
be worth about 0.29 × ¼ ≈ 7% of the pilot's term**, which is itself worth 0.01.
That is an estimate with its derivation shown, not a measurement, and it is the
remaining check — but nothing about it can turn 0.01 into 0.29.

**What this does to the shape of item 11.** The mechanism that demonstrably
moves the boundary is the one §§56–58 found, and it is *not* the one the item
named: it is the **magnitude of drag applied at the bob**, acting through the
speed oscillation, with σ linear in the height at which it acts. The
swing-rate-proportional damping the item has always described is measured, at
last, and is a rounding error. Those are two different mechanisms, and eleven
levels of this item have been named after the wrong one.

**The change is physically right and is left off by default**, which is a debt
rather than a decision: the pilot's drag really does depend on the pilot's
airflow, and `Aircraft` is the wrong reference on any wing whose pilot swings.
Making `Pilot` the default is a flight-behaviour change that needs the
calibration gates re-evaluated, and item 11's gates are already loosened, so it
belongs in a deliberate pass rather than tacked onto a measurement.

## 61. The correct drag reference passes every gate and is still not shipped

§60 left a debt: `HarnessDragReference::Aircraft` is wrong physics — a swinging
pilot's drag depends on the pilot's airflow — and it defaulted off only because
flipping it inside a measurement would have been careless. This iteration
flipped it and ran the full suite.

**The suite is green.** Zero failures, eleven suites, module included. Green was
never the test, and a decision rule was written down before the output: the
published references arbitrate. Toward them, a failing gate encoded the old
wrong number; away, revert whatever σ said; barely moving, keep it on
correctness.

**What it actually did, diffed against the pre-flip run:**

| quantity | before | after | published | reading |
|---|---|---|---|---|
| glide at trim | 11.33 | 11.20 | 9.5 | **toward** |
| sink | 0.97 | 0.99 | 1.14 | **toward** |
| trim speed | 39.8 km/h | 39.9 | 39.0 | slightly away |
| trim incidence | 5.14° | 5.09° | 5.30 needed | slightly away |
| brake-incidence disagreement | 0.73° wrong way | **0.49°** | should be positive | **halved** |
| coordinated turn 35% | NOT SETTLED | **settled** | — | better |
| accelerator step | settled | **NOT SETTLED** | — | worse |
| full bar left to settle | 83.4° | **45.9°** | — | far less divergent |
| **4 m/s symmetric collapse** | **L 0.710 R 0.710** | **L 0.999 R 1.000** | — | **near-total** |
| worst L−R fold difference | 0.323 | 0.420 | — | worse |

**The prediction I recorded was half right and the half it missed is the
important one.** I predicted trim would barely move because the swing rate at a
settled trim is near zero and the new term is proportional to it. Trim did
barely move. What I did not predict is that the term is *large in transients* —
and the collapse benchmark is nothing but transient. A 4 m/s symmetric frontal
that used to fold 71% now folds essentially completely.

**So the flip is not shipped, and the rule is why.** The result is mixed rather
than favourable: two published numbers move toward, two move slightly away, and
the largest change of all — the collapse depth — is one **no published number
arbitrates.** Turning a 71% benchmark fold into a 100% one is a pilot-visible
behaviour change, on the number Level 8 and §13 were tuned against, and the
model has no external data that says which is right. Shipping it because the
suite is green and the physics term is correct would be replacing a measured
benchmark with an unmeasured one on my own authority.

**What that costs and what it buys.** The engine keeps a term that is
*definitely* wrong in a way that matters only in transients; the record gains an
exact experiment for whoever runs the SIV validation Level 9's exit gate already
requires. That validation now has a specific question to answer rather than a
general one: **on a 4 m/s symmetric frontal, does an EN-B fold 70% or 100%?**
One SIV answer decides the default.

The flag stays, defaulted off, with this section as its justification. Both
states of it are now measured, which is the only reason the choice can be
deferred honestly rather than merely postponed.

## 62. The line-drag dial is not inclination plus shielding — it is 99% shielding

`InstalledDragSpec::lineProjectedFraction` is 0.35, and its comment says what
the number bundles: *manufactured line length is not all normal to the flow:
cascades overlap, upper galleries are inclined, lower lines shield one another*.
Three effects, one stated number, never derived — on a term that is 47% of the
canopy's own drag. Item 12 asks for a glide that lands "without a coefficient
chosen to put it there", and this is one of the coefficients.

**Two of the three effects are geometry, and the solver has already built the
geometry.** A cylinder in crossflow presents `L·d·sin θ` to the flow, so the
inclination part is a sum over the cable elements `suspension_tests` already
constructs. Measured off the built graph, against the flow direction:

| quantity | measured | spec says |
|---|---|---|
| line length | 254.7 m | 254.0 |
| length-weighted diameter | 0.00094 m | 0.00105 |
| **projected fraction from inclination** | **0.993** | 0.350 |
| projected area | 0.237 m² | 0.093 m² |

**The lines are essentially all normal to the flow.** They hang from canopy to
pilot and fan out spanwise; both directions are perpendicular to a horizontal
wind, and only the fore-and-aft spread between the A and C rows tilts any of
them appreciably. Inclination is worth **0.7%**, not the large share the comment
implies.

**So the dial is not three comparable effects. It is one effect, unmeasured.**
Of the 0.35, geometry accounts for 0.993 and the residual — **0.352** — is
entirely shielding and overlap. The shipped model therefore carries **39% of the
line drag area its own geometry presents**, on the strength of an
interference allowance nobody has ever justified in writing.

That is not a claim that 0.35 is wrong. It is a claim that it is now *isolated*:
a single flow-physics question — how much does a paraglider's line cascade
shield itself? — instead of a lumped number carrying two effects that turn out
to be negligible and one that is doing all the work. A 65% reduction from
shielding alone is aggressive; published practice on line drag tends to use
modest interference allowances rather than a factor of three.

**And it lands next to §56, which is why it matters now.** §56 found that the
model's missing drag, applied at the *harness* end, lands glide, trim speed and
sink together — and needed **0.199 m² of Cd·A** to do it. Restoring the lines to
a defensible shielding allowance of, say, 0.9 would add
(0.993 − 0.35)/0.993 × 0.237 × 1.05 ≈ **0.16 m² of Cd·A**, at a moment arm half
way down the lines rather than at the canopy. Same order, same end of the
aircraft, arrived at from geometry rather than from the glide it needs to
produce.

**The geometry is now wired in, and one number is left stated.** The solver
computes the projected line area from its own graph at construction, so the
length, the diameter and the inclination are consequences of the wing rather
than statements about it. `lineShieldingFactor` is what remains — alone,
labelled, and carried over from the old lumped value at **0.394** so that
measuring the geometry changes no flight behaviour. This is a refactor of the
*justification*, not of the physics, and the suite confirms it: every headline
number identical at printed precision.

*The one thing that is not exact:* 0.394 reproduces the old drag area to **0.14%**,
not to the bit. The factor is stated to three digits rather than tuned to
however many it would take to preserve the product exactly, because a
five-decimal coefficient chosen to make a diff vanish is the same species of
number this section exists to remove. 0.14% is below the printed precision of
every gate, and it is written down here rather than discovered later as an
unexplained drift.

**Stated as a lead and not a result**, because it connects a geometric
measurement to a fitted requirement and those are different kinds of number: it
would be exactly the sort of coincidence this axis has produced and retracted
four times. What makes it testable rather than tempting is that the shielding
allowance is now a lone unknown with a literature behind it, and the line drag
it implies can be measured against the glide *without* being fitted to it.

## 63. The lines sweeping, measured — and one grid step that flatters the pair

§60 implemented the first of item 11's two named terms and dismissed the second
by arithmetic: the lines carry 0.098 m² of Cd·A against the harness's 0.336, and
their sweeping torque takes an extra ∫s³ds/L⁴ = ¼ weighting, so ~7% of a term
already worth 0.01. That was reasoning, not measurement, and four pieces of
reasoning have been retracted on this axis. This measures it.

A cable at distance *s* below the hinge sweeps at *s·q̇*, so its damping torque
carries an *s²* weight the drag area does not. Summing `d·L·s²` over the cascade
at construction gives an area-moment which, times ½ρ·Cd·V, is a torque per unit
swing rate. Four wings, because the split is the claim:

| ratio | σ clean | σ pilot | σ lines | σ both |
|---|---|---|---|---|
| 0.35 | −0.0201 | −0.0230 | −0.0204 | −0.0229 |
| 0.30 | not settled | not settled | not settled | **−0.0108** |
| 0.25 | DEPARTED | not settled | DEPARTED | not settled |

**The estimate holds.** The lines are worth 0.0003 of σ against the pilot term's
0.0029 — **10%, against an estimated 7%.** Right to the precision that decides
anything, and §60's conclusion is unchanged: the two terms together move σ by
14%, which is about **0.01 of coefficient against the 0.29** item 11 needs.

**Two honest limits on that number, and the second matters more.**

*The pair is not additive.* Pilot alone gives −0.0230 and both give −0.0229 —
adding a damper made the aircraft a shade less damped. The discrepancy from
additivity, 0.0004, is **larger than the line term itself**, so "the lines are
worth 10% of the pilot's term" sits at the edge of what this instrument
resolves. The order is solid; the second digit is not.

*The boundary moves one grid step, and that flatters the pair.* At ratio 0.30
the pair **settles** where clean, pilot-alone and lines-alone all fail to — so
on the 0.35/0.30/0.25 grid this axis has always used, the boundary goes from
0.35–0.30 to 0.30–0.25. Read alone that looks like 0.05 of coefficient, five
times what σ says. Both are true and the reconciliation is that **0.30 was
already marginal**: §54 recorded it neither settling in 3600 s nor departing, at
σ ≈ −0.008. A 14% improvement is enough to tip a ratio sitting on the settling
criterion across it, and a coarse grid then reports a whole step. This refines
§60's "the boundary does not move" — with both terms it does — without changing
what the terms are worth.

**Item 11's founding estimate is now fully tested rather than half tested.** Both
named terms are implemented, both measured, and together they are worth a
thirtieth of what the estimate claimed. The conclusion of §60 stands on two
measurements instead of one measurement and one argument.

Both terms default off, for the reason §61 gave: they change what the aircraft
does, and that is a decision to take on evidence rather than as a side effect of
a measurement.

## 64. The correct line drag passes every published gate and is still not shipped — this time the blocker is symmetry

§62 isolated `lineShieldingFactor` as the last stated number in line drag, at
0.394: the cascade is asserted to hide 60% of its own frontal area. That was
never checked against the one thing that decides it — **how far apart the lines
are.** Wake interference between cylinders is a function of spacing in
*diameters*, and paraglider lines are a millimetre thick.

**Measured off the graph at the design pose:**

| | spacing |
|---|---|
| closest pair | **158 d** |
| lower quartile | 233 d |
| median | 534 d |
| upper quartile | 628 d |

Cylinder wake deficits are near-field: real a few diameters behind, largely
recovered by twenty or thirty. **The closest pair on this wing is 158 diameters
apart.** No wake argument reaches 60% shielding at those spacings. The measure
uses midpoint-to-midpoint distance, which *overstates* separation for lines
converging at a cascade junction — a bias toward the model's defence, and it
still fails by an order of magnitude.

**Removing the coefficient moves every published number toward its reference:**

| | shipped 0.394 | 0.9 | 1.0 | published |
|---|---|---|---|---|
| trim speed | 11.06 | 10.99 | 10.955 | 10.83 |
| sink | 0.97 | 1.086 | 1.100 | 1.14 |
| glide | 11.33 | 10.07 | **9.91** | 9.5 |
| incidence | 5.14° | 5.29° | 5.35° | 5.30 needed |

At 1.0 — no chosen number anywhere, just geometry times a subcritical cylinder
Cd — **glide error falls from 19% to 4.3%, sink from 15% to 3.5%**, and the
incidence lands on what the published lift coefficient needs. This is item 12's
own criterion met by *deleting* a coefficient rather than fitting one, on the
largest disagreement in the model.

**And it cannot ship, because it breaks a symmetry.** The 4 m/s symmetric
frontal folds **L 0.779 R 0.903** where it folded 0.710/0.710 identical to three
decimals, and the gate that fails is *"symmetrically — the two halves are the
same wing"*. Turn rate while folded goes 2.06 to 4.04 rad/s.

**That is not a gate encoding an old number, and the distinction is the whole
decision.** Every other failure this session was a calibration disagreement,
where "toward published" settles it. This is a mirror-symmetric wing answering a
mirror-symmetric input differently on its two halves — a solver-integrity
failure. No amount of agreement with published glide licenses it.

**What it exposes was already written down.** The gate's own text says the deep
symmetric frontal has no steady state to find and that *the peak folds still
match; the path there does not*. Higher line drag drives the collapse deeper
into that regime until the peaks stop matching too. **The correction is blocked
by a known solver limitation rather than by evidence against it**, and the gate
names the fix: Level 11.

**Deliberately not tuned to fit.** A shielding value chosen to be the largest
that keeps the symmetry gate green would be exactly the coefficient-chosen-to-
put-it-there this section exists to remove. 0.394 stays, now labelled as
*measured wrong* rather than merely unjustified, with the correction and its
blocker both recorded.

**This is the second time in this session** that a well-justified change passed
its published references and was held back by a benchmark with no published
arbiter (§61 was the first). The two are not symmetric: §61's change had no
published improvement to point to, and this one has a large one. What they share
is that shipping either would mean overruling a measured invariant on the
strength of a correct-sounding argument.

## 65. The wing is symmetric to round-off, so the collapse asymmetry is amplification and Level 11 is a stability problem

> **Superseded in part by §67.** The elimination stands. "Amplification" and
> "Level 11 is a stability problem" do not: the divergence was later measured to
> be two single-step discontinuities with no growth rate, and the fix is branch
> selection in the separated solve rather than stabilising a symmetric solution.

§64 was blocked by the 4 m/s symmetric frontal folding L 0.779 against R 0.903.
The gate blames the partly separated solve having no steady state and names
Level 11 as the fix — an explanation that had never been tested, and which does
not distinguish two very different situations:

- **amplification** — the wing is symmetric to machine precision, the collapse
  regime is unstable, and round-off grows into a visible difference. Nothing is
  broken, and Level 11 needs a formulation whose symmetric solution is *stable*.
- **a seed** — something upstream is not mirror-symmetric and the collapse
  merely reveals it. Then there is a defect with an address, far cheaper to fix
  than Level 11.

The suspension graph is where a seed would most likely live: every line load,
fold criterion and attachment position comes from it. Measured — for every
off-centre node, the distance from its mirror image to the nearest node of the
same kind and row:

| | |
|---|---|
| nodes with no mirror partner | **0** |
| worst mirror residual | **6.2e-15 m** |

On a structure eight metres across that is a relative 1e-15: round-off, not
construction. **The graph is eliminated as the seed**, and with it the most
likely candidate.

**So the gate's explanation is supported, and it now has a number behind it.** A
1e-15 asymmetry growing to a 0.124 difference in fold depth is an amplification
of order **1e14** across the event — which is what an unstable regime does to
round-off, and is not something a formulation error would produce so cleanly.
Level 11 is therefore a **stability** problem and not a bug hunt, which is worth
knowing before anyone spends a level looking for a broken mirror.

**What this does not prove**, and the boundary matters: only the graph was
measured. The VSM's section ordering, the collapse solver and the pressure
model are all downstream and could each carry an asymmetric step — a
sequentially-updated loop that reads partially-updated neighbours would break
symmetry systematically rather than by round-off. Those remain untested. What is
established is that the *structure* is clean, so any seed is in a **solver**,
and that the amplification factor is large enough that no seed is needed to
explain the result.

Bounded in `suspension_tests` as a structural claim: every off-centre node has a
mirror partner, and the worst residual is under 1e-12.

## 66. The VSM's section loop is symmetric too, and the amplification case is now the strong one

> **Superseded in part by §67**, on the same point as §65: the 9.3e-15 measurement
> stands, the "amplification of order 1e14" reading of it does not.

§65 eliminated the suspension graph as a seed for the collapse asymmetry and
named three untested downstream candidates: the VSM's section ordering, the
collapse solver, the pressure model. This tests the first and most likely — the
circulation solve is iterative over sections, and a loop reading partially
updated neighbours breaks symmetry *systematically* rather than by round-off.

The two signatures are far apart, which is what makes the test cheap and
decisive. Round-off gives a relative mirror residual near 1e-16; a
Gauss-Seidel-style sweep over a symmetric wing at symmetric incidence gives
something orders larger, biased toward the end the sweep finishes at.

**Measured on a symmetric flat wing at symmetric incidence: worst relative
left-right circulation difference 9.3e-15.** Round-off. The VSM's section loop
preserves mirror symmetry, and is eliminated.

**Two eliminations in a row make the amplification case the strong one.** The
structure is symmetric to 1e-15 (§65) and the aerodynamic solve is symmetric to
1e-14 — so a symmetric frontal enters the collapse regime symmetric to
round-off, and leaves it with a 0.124 difference in fold depth. That is
amplification of order 1e14 with the two most likely seeds now ruled out
individually rather than by argument.

**What is left, and it is a shorter list than it was:** the collapse solver and
the pressure model. Both are worth the same cheap probe, and until one of them
fails it, the gate's own explanation — no steady state to find in the partly
separated regime — stands as the best account of §64's blocker, and Level 11
remains a stability problem rather than a defect hunt.

Bounded in `aerodynamics_tests` at 1e-9, which is six orders above the measured
value and far below any systematic asymmetry.

## 67. The candidate list is empty, and the divergence is two discontinuities rather than an amplification

§65 named three downstream candidates that could seed the collapse asymmetry
blocking §64's line-drag correction. §66 eliminated the first. This closes the
other two, and then measures the thing the whole chain had only ever inferred.

**The collapse solver was already eliminated, by a test nobody remembered
writing.** `collapse_tests` has carried a mirror gate since Level 8: mirrored
input, spanwise-varying so a uniform field cannot flatter it, 240 steps, worst
section mirror error **0.00e+00**. It needed running, not writing.

**The pressure model needed a real probe, and the first one was worthless.**
The crossport loop is the one candidate with a record — it *was* Gauss-Seidel
once, every cell seeing its left neighbour advanced and its right neighbour
not, and it was fixed when Level 8 closed. Since then the fix has been carried
by a comment. Nothing failed if someone read `updated` there again.

The first probe settled a symmetric wing to steady state and compared mirror
cells. It passed — **and it passed with the defect deliberately reintroduced.**
At equilibrium every neighbour difference has gone to zero, so the crossport
term is zero and a directional sweep has nothing left to be directional about.
The probe was measuring nothing, and would have reported the candidate
eliminated on no evidence.

The working probe lives in the transient: the canopy inflates from empty with
both tip groups' inlets dead, which holds a **65.8 Pa** gradient across the ribs
for the whole fill, and the residual is taken over every step rather than at
the end. Measured: **0.000e+00** relative pressure difference and **0.000e+00**
fill difference. Round-off, exactly.

**And the power is not where it looks like it should be.** With the defect put
back, the *pressure* residual stays at 0.000e+00 and only the *fill* residual
moves — to 1.018e-03, one whole step of fill between cells 5 and 39. A cell
that is still filling holds no pressure by construction, so the asymmetry lives
in the fill front where every pressure is still zero. A probe watching pressure
alone reports a clean mirror on a solver that has the defect in it.

**So all three candidates are closed** — suspension graph 1e-15, VSM 9.3e-15,
collapse solver 0.00e+00, pressure model 0.00e+00 — and the pressure probe
doubles as the regression gate that fix never had.

### The amplification picture had the magnitude right and the mechanism wrong

Elimination left "amplification of order 1e14" standing, but that was always an
inference from endpoints: a 1e-15 wing reaching a 0.124 fold difference implies
a large factor somewhere, without anyone watching it happen. Watching it happen
says there is **no growth rate**, because nothing grows. The event is two
single-step discontinuities a tenth of a second apart, and the first is
invisible:

| | |
|---|---|
| enters the fold at | **2.33e-15** |
| **t = 1.400 s** | the pressure **margin** field breaks: 1.95e-14 → **4.91e-01** in one 8.3 ms step |
| **t = 1.500 s** | the **fold** breaks: 2.29e-15 → **3.94e-02** in one step, 100 ms later |
| peak | **9.55e-01** |

The 100 ms in the middle is the whole finding. At t=1.40 the canopy is carrying
an O(1) left-right asymmetry **in the criterion** while remaining a measurably
symmetric wing folded to 1e-15 — because every margin involved is still
*positive*, and a positive margin holds the nose regardless of its size:
`target = clamp(-margin/0.6, 0, 1)` is zero for all of them. The asymmetry is
latent. At t=1.50 that field crosses zero on one side only, and the sign test
turns it into a fold difference in a single step.

**The amplifier is a sign test applied to a field that had already lost its
symmetry while every value in it had the same sign.** Not an unstable mode
growing round-off — there is no exponential to fit, and the step before each
break is still at round-off.

**This sharpens Level 11 rather than confirming the plan for it.** §65 and §66
concluded "a stability problem — Level 11 needs a formulation whose symmetric
solution is *stable*." That aims at a mechanism which is not present. Nothing
here is a symmetric solution being nudged off an unstable equilibrium; the
margin field jumps by O(1) in one step, which is **branch selection in the
separated aerodynamic solve**, the same no-steady-state regime `PHYSICS_TODO`
item 6 records. The fix is convergence and single-valuedness there, not damping.

**What is still not proven.** The upstream jump is *consistent with* the VSM's
separated branch and is not directly attributed to it: what is measured is that
the margin field breaks first and the fold follows, so the asymmetry arrives
from upstream rather than being made in the collapse solver. Which upstream
step produces the O(1) jump is the next question, and it is a much narrower one
than the list this chain started with — a single 8.3 ms step at a known time on
a reproducible run.

Gated in `coupled_tests` as the causal claim: the margin must lose symmetry no
later than the fold does. If the fold ever broke first, the collapse solver
would be manufacturing the asymmetry rather than revealing it, and this
section's elimination of it would be wrong.

## 68. The jump is a VSM tick with nothing to converge to, and fifteen times the iterations lands on the same step

§67 closed the seed list and left one question: which upstream step produces
the O(1) jump in the pressure-margin field at t=1.400. This answers it, and
then tries to fix it — because the answer on its own does not say which of two
very different fixes Level 11 needs.

### Attribution: one aerodynamic tick, both sides of the margin at once

The margin decomposes exactly,

```
margin = internalCp - externalNoseCp - 0.75(1-load) - 0.35 slack
```

and `externalNoseCp` is a function of the section's **incidence alone**. Tracing
the two sides separately across the break — no source change needed, both are
already in the diagnostics — says they move on the same step, and that step is
an **aerodynamic tick**. The margin field is piecewise-constant between ticks
because that is when it is recomputed.

| at the breaking tick | |
|---|---|
| incidence side | 1.6e-14 → **3.79e-01** |
| pressure side | 3.3e-15 → **1.12e-01** |
| VSM residual | 1.14e-06 → **2.22e-02** |

Both sides break on the *same step*, gated as such: the incidence field and the
pressure-and-line field do not lose symmetry independently, they lose it
together, because one solve feeds both.

**The VSM residual had been falling monotonically for a second and a half** —
2.40 at the gust, 9.3e-05 by t=0.7, 1.14e-06 at t=1.3 — with the wing
mirror-symmetric to 1e-14 the whole way. Then it jumps four orders on one tick,
and the incidence field stops being symmetric on that same tick.

**§67's 100 ms latency is no longer mysterious either.** The aerodynamic
interval is 12 steps at 120 Hz — exactly 0.1 s. The fold breaks one
aerodynamic tick after the margin does, because that is when the next solve
lands. Nothing drifts in between; both are single ticks.

### The experiment: iterations cannot buy it

That left two readings with different fixes, and an argument would not settle
them:

- the flight solve is **capped at 40 iterations** once warm, so it may simply
  have run out before it landed. Cure: iterate more.
- the separated branch has **nothing to converge to**, so the iterate wanders
  and which half it wanders toward is arbitrary. Cure: an unsteady
  formulation. Iterations cannot touch it.

Raising the cap separates them. `SetFlightSolveIterationCap` was added for
exactly this — an instrument, defaulted to the shipped 40, on the same pattern
as `SetSectionDragOffset`.

| cap | breaks at | on an aero tick | VSM residual before → at |
|---|---|---|---|
| 40 (shipped) | **t=1.400 s** | yes | 1.14e-06 → 2.22e-02 |
| 200 | **t=1.400 s** | yes | 9.44e-07 → 6.95e-02 |
| 600 (cold-start budget) | **t=1.400 s** | yes | 8.03e-07 → 2.05e-02 |

**Fifteen times the budget lands on the same tick.** And the control is in the
same table: the residual *before* the break does improve with iterations,
1.14e-06 → 8.03e-07, so the extra work is real and the solve genuinely
converges better everywhere it can converge at all. At the breaking tick it
buys nothing — 2.22e-02, 6.95e-02, 2.05e-02, no better and not even monotone in
the cap.

**So the separated solve is not short of iterations. It has nothing to converge
to.** That is the same negative-lift-slope, no-steady-state branch `PHYSICS_TODO`
item 6 records, now demonstrated on the aircraft rather than argued from the
section polar.

### What this settles for Level 11

**Level 11 cannot be bought with compute**, and that is worth a section on its
own, because "raise the iteration cap" is the first thing a reader meeting this
divergence will try. It has been tried, at fifteen times the budget, and it
does not move the break by one step.

The three readings this chain has passed through, in order: a **defect** with an
address (§64-§65, refuted — every candidate mirror-symmetric); an
**instability** amplifying round-off (§65-§66, refuted — no growth rate, two
single-step discontinuities); and now a **non-convergent branch**, which is the
one that survives its own test. The fix is an unsteady wake formulation whose
separated solve is single-valued, not a stabilised or better-iterated steady
one.

**What is still not proven**, and it is a narrower gap than it was: *why* the
solve loses single-valuedness at that particular tick rather than at any of the
fourteen before it.

### A naming collision worth fixing before someone trips on it

**"Level 11" names two different things in this repo**, and one of them is
finished. `PHYSICS_TODO` §"What Level 11 closed" is the **pitch axis by
linearisation** — the eigenmode programme, done. The collapse gates, this
section, `PHYSICS_ENGINE` item 1 and `PHYSICS_TODO` items 6 and 14 all say
"Level 11's unsteady wake", which is a *different and unstarted* piece of work.

A reader meeting "the fix is Level 11" in the symmetry gate and following it to
the summary section will find a level marked closed and reasonably conclude the
fix has shipped. It has not. Nothing in this section or §67 is blocked on the
ambiguity, but the next person to pick up the unsteady wake should rename it
rather than inherit the collision. What is established is that it does, that both sides of the
margin follow it within the same step, that the fold follows exactly one
aerodynamic interval later, and that iterations are not the lever.

Gated in `coupled_tests`: the asymmetry must enter **on an aerodynamic tick**
(the margin field is only written then, so a break between ticks would mean
something other than the solve is writing into it); the residual must be below
1e-4 on the tick before and above 1e-3 on the tick that breaks; and the three
caps must break on the *same* tick with the pre-break residual improving — the
last being what makes the unchanged break time mean anything.

## 69. Every way of landing the published glide leaves the solver, so item 12 has no route around Level 11

The master plan's step 2, run. It asked one question with a large consequence:
does a **harness-side** drag correction close item 12 without needing the
unsteady wake? §64's line-drag correction was blocked by the symmetric frontal
losing its mirror symmetry, and four sessions went into establishing that the
separated regime behind that gate is Level 11's problem. If the deficit could
be placed at the harness instead, the whole ladder reordered.

The reasoning for expecting yes was explicit and it was wrong: harness drag
acts at the pilot, six metres under the canopy, so it should not touch the nose
pressure balance. **It touches it through the pendulum.** Drag at the pilot
pitches the wing nose-down relative to him, and nose-down is the frontal
direction.

### Three wings at the same published glide, one frontal

Each correction bisected to the published 9.5 glide by an earlier section —
the section offset 0.01035 (§55), the harness 0.199 m² (§56) — and flown into
the same 4 m/s symmetric frontal:

| | peak fold L / R | spread | worst L−R through the event | turn | envelope |
|---|---|---|---|---|---|
| shipped (glide 10.96) | 0.710 / 0.710 | 0.000 | 0.303 | 1.98 rad/s | idle |
| section +0.01035 | **0.958 / 0.703** | **0.255** | 0.667 | 1.93 rad/s | **ENGAGED** |
| harness +0.199 m² | 1.000 / 1.000 | 0.000 | 0.433 | **6.53 rad/s** | **ENGAGED** |

**The section correction breaks the symmetry the same way the line-drag one
did** — 0.255 of spread against §64's 0.124. That much was the expected shape
of the answer for the wrong candidate.

**The harness correction looks like it passes and does not.** A peak spread of
zero between two halves both saturated at a full collapse is not evidence of
symmetry, it is evidence of saturation — and the path asymmetry through the
event is *worse* than shipped, 0.433 against 0.303, with three times the
rotation.

**And both engage the numerical safety envelope.** By guiding rule 12 that
means their numbers are not flight behaviour at all. The benchmark cannot
adjudicate between the candidates, because every way of landing the published
glide takes this event outside what the solver can represent.

### What this settles

**Item 12 has no route around Level 11**, and that is now measured for all
three candidate locations — lines (§64), section, harness — rather than argued
for one. The wake stays on the critical path, and the master plan's step 2 is
answered in the direction that costs the most.

**It also sharpens what the blocker is.** The three corrections fail
differently — one splits the halves, one saturates them, one was already known
to split them — but they fail at the same place: the partly separated regime
§68 showed has no single-valued solution. A wing that glides 9.5 instead of
10.96 arrives at that regime harder, whatever slowed it down. The blocker is
not a property of line drag; line drag was just the first correction to reach
it.

**The honest negative.** This was run hoping to take a research level off the
critical path. It put it back on, and the value is that four sessions of
downstream work no longer rest on an untested assumption about where the
missing drag lives.

Bounded in `coupled_tests` as three claims that can each fail independently:
the shipped wing flies the frontal with the envelope idle (the control), both
corrections engage it (the limitation), and the two failure modes are distinct
— the section splits, the harness saturates.

## 70. The geometry-driven envelope, measured — and weight shift is the finding nobody was looking for

Items 7 and 17 have been blocked for several levels on a sentence: the
geometry-driven stack "departs at 40% brake". That is one point on one axis. A
swap needs the whole boundary, and "fly the stack across its stated envelope"
had no stated envelope to fly across. `parapenting_model_agreement` is guiding
rule 11 taken literally — both models, same wing, same 105 kg, same still air,
same inputs, settled the same way, reported side by side.

### Where the two models agree

| station | legacy v / sink / glide | coupled v / sink / glide | worst gap |
|---|---|---|---|
| hands up | 10.81 / 1.14 / 9.46 | 11.18 / 1.02 / 10.99 | 13.9% |
| brake 10% | 10.81 / 1.14 / 9.46 | 11.18 / 1.02 / 10.99 | 13.9% |
| brake 20% | 10.43 / 1.13 / 9.21 | 11.05 / 1.00 / 11.01 | 16.4% |
| brake 25% | 10.26 / 1.13 / 9.09 | 10.16 / 0.94 / 10.80 | 16.6% |
| brake 30% | 10.09 / 1.12 / 8.97 | 9.94 / 0.94 / 10.57 | 16.4% |
| brake 40% | 9.67 / 1.37 / 7.07 | **7.81 / 7.81 / 1.00** | 85.9% |
| weight shift 50% | 11.39 / 3.11 / 3.66 | **11.18 / 1.02 / 10.99** | 67.3% |
| half bar | 11.69 / 1.46 / 8.01 | **7.82 / 7.82 / 1.00** | 87.5% |
| full bar | 12.81 / 2.09 / 6.11 | **32.81 / 29.71 / 1.10** | 93.0% |

Brake 10% reads identically to hands up **in both models**, which is not a bug
in either: 10% is inside the brake free play this wing has sewn in, so the
handle is moving through air. Two independent models agreeing on that is a
small piece of evidence that the free play is right.

**The 13.9% at hands up is item 12 and nothing else** — glide 9.46 against
10.99. The two models agree on speed to 3% and on sink to 11%, and disagree on
glide because one of them is missing a sixth of its drag. This comparison
cannot say which.

### The envelope, bisected

| axis | flies to |
|---|---|
| symmetric brake | **37% of travel** |
| speed bar | **22% of travel** |
| weight shift | no departure — and no authority either |

Departure is defined by the wing rather than by a threshold on the answer:
incidence more than 30° above its own hands-up trim is a separated wing
whatever speed it reports.

**Brake confirms the documented number** — 37% against a stated "departs at
40%", which is the first time that sentence has had a boundary rather than a
bracket.

**The speed bar is worse than documented and this is new.** The record says
full bar is the pitch-divergent condition, because `calibration_tests` only
ever applied `accelerator = 1.0`. Bisected, the wing departs at **22% of bar**.
Half bar — an ordinary cruise input a pilot uses constantly — is already well
outside what the stack can fly.

### And the one nobody was looking for

**Weight shift does essentially nothing.** 50% of weight shift produces
**0.01 rad/s** of turn and no change in speed, sink or glide to three
significant figures — the coupled row is identical to hands up. The legacy
model at the same input is turning at 0.20 rad/s and descending at 3.11 m/s.

This is not a departure, so no gate caught it, and it is not a disagreement
with a published number, so calibration did not either. It sat in the gap
between the two. Item 0b records that the wing "turns several times too slowly"
on brake; on weight shift it does not turn at all, and weight shift is a
primary control — a pilot flies a whole turn with it.

### What this does to item 17

**It moves the blocker.** The reason the geometry-driven stack cannot fly the
game today is no longer "it departs at 40% brake", which sounded like an edge
case a stated envelope could route around. It is that **the stack has no speed
bar beyond 22%, and no weight shift at all** — two of the four controls a
paraglider has. An envelope that excludes those does not describe a flyable
aircraft; it describes hands-up gliding.

So step 4 of the plan — fly the stack across a stated envelope — cannot proceed
as written either. What it can proceed as is narrower and honest: the stack can
fly *hands-up to a third of brake*, which is enough to compare against the
legacy model in ordinary glide and not enough to give a player.

**The order this suggests** is that weight shift is now the cheapest thing on
the list that stands between the stack and a pilot, and it is untouched: no
item owns it, no gate covers it, and it is a control rather than a coefficient.

## 71. Weight shift reaches the lines and stops there, because the aerodynamic solve has no input from the suspension

§70 found weight shift producing **0.01 rad/s** of turn where the legacy model
gives 0.20 and a real EN-B gives something like 0.2–0.3. "Weak" and "missing a
mechanism" want different fixes, so this traces the chain link by link.

### Everything up to the line network works

| shift | left carabiner | right carabiner | split | bank | turn |
|---|---|---|---|---|---|
| 0.00 | 437.4 N | 437.4 N | 0.0% | 0.00° | 0.0000 rad/s |
| 0.50 | 363.0 N | 511.8 N | 17.0% | 0.75° | 0.0070 rad/s |
| 1.00 | 288.6 N | 586.1 N | **34.0%** | 1.50° | 0.0140 rad/s |

**A 34% load transfer between the carabiners at full shift is a real
asymmetry**, correctly signed and exactly proportional to the input. The pilot
moves, the harness rolls, the line anchor translates, and the lines feel it.
None of that is broken, and gating it as working is half of what this section
adds.

### The chain stops at the lines

`VsmSolveInput` carries airspeed, angular velocity, air density, per-cell
internal pressure coefficient, left and right brake, and a per-section gust.
**It carries nothing from the suspension solve.** There is no channel through
which a 34% difference in riser load can change the wing's spanwise lift
distribution, so it does not.

What is left is the one path that does exist: weight shift translates the
pilot's centre of gravity, the anchor moves with it, and the aircraft banks
geometrically. **That path is intact and arithmetically incapable of the
authority a real wing has.**

Full shift moves the CG **7.1 cm** — `hipTravelM` 0.075 m times a 0.95 strap
factor for a seatboard harness — on a **6.6 m** hang. That is
atan(0.071/6.6) = **0.6°** of bank from geometry alone; the solver reaches 1.5°
once the line network's own response is included. Even at a generous 20 cm of
hip travel it would be 1.7°. A real wing banks ten to fifteen degrees on weight
shift.

**So the mechanism that does the work on a real paraglider is not CG
translation.** It is the differential riser load changing the local incidence
across the span — the loaded half flying at higher lift — and that is precisely
the path with no channel here.

### Why this matters more than its size

**It is a structural gap, not a coefficient**, and that is the useful part: no
value of `hipTravelM` closes it. Sweeping the harness geometry would produce a
better-looking number and the same missing physics.

**It also explains the shape of item 0b.** Brake reaches the wing —
`aero.leftBrake` and `aero.rightBrake` are in the input struct — so brake-driven
turns work, just too slowly. Weight shift has no equivalent, so it does not
work at all. Two symptoms, one asymmetry in what the aerodynamic solve is
allowed to know.

**And it is the same class of finding as §70's stall pendulum (item 19):** a
mechanism referenced to the wrong thing, surviving because it sat between two
kinds of gate. It is not a departure, so the envelope checks missed it; it is
not a disagreement with a published number, so calibration missed it.

Bounded in `coupled_tests` as two claims that fail independently: the carabiner
split is produced and proportional (the half that works), and the resulting
bank stays under 3° with turn under 0.05 rad/s (the half that does not, bounded
rather than fixed).

## 72. The geometric channel's aerodynamic half is exact and linear; its structural half does not exist, because the canopy is rigid

§71 diagnosed the missing channel and the master plan designed it: read a
per-station pose off the solved line geometry, publish it as a per-section
incidence offset, and let the roll emerge. Building it turned up that the
design has two halves with very different standing, and only one of them can
be built against the stack as it is.

### The aerodynamic half works, and it is a clean linear gain

`VsmSolveInput::sectionIncidenceOffsetRad` exists, empty means the design
pose, and no existing caller moved. Measured on the EPIC 2 canopy with an
antisymmetric twist linear in span:

| twist | roll moment | gain | lift |
|---|---|---|---|
| 0.1° | 14.9 N·m | 8545 N·m/rad | 463.8 N |
| 1.0° | 149.1 N·m | 8543 N·m/rad | 463.5 N |
| 4.0° | 596.6 N·m | 8546 N·m/rad | 460.5 N |

**8543 N·m/rad, flat to 0.03% over sixteen times the range.** Mirror-exact to
2e-12. Lift changes by 0.7% at four degrees, so the twist is a couple: it
rolls the wing without re-trimming it. Nothing here needed a coefficient — it
is the integral of the section forces, the same way roll damping is.

Being linear is the useful part, because it turns the twist into a
**requirement that can be divided out** rather than a number to sweep. Full
one-side brake makes ~1530 N·m of roll on this wing, so **matching today's
brake authority takes about 10° of antisymmetric twist** — and brake itself is
several times slower than a real wing (item 0b), so matching a real wing takes
considerably more.

### The structural half is identically zero, and that is provable rather than measured

The design says to read the chord direction from each station's front (A/A′)
attachment to its rear (C) attachment. Every canopy attachment in
`TensionCableSolver` is placed as

```cpp
position[i] = canopyOrigin + canopyAttitude.Rotate(node.canopyLocalM);
```

**One rigid body.** So station-to-station chord directions differ from the
design pose by a single global rotation, identical for every station, and the
per-station offset the design asks for is exactly `incidenceChangeRad` again
with a longer derivation. Measured to confirm, at full weight shift:

| side | span | offset from design |
|---|---|---|
| left | −0.72 | +0.00263863 |
| left | −0.45 | +0.00263827 |
| right | +0.45 | +0.00263827 |
| right | +0.72 | +0.00263863 |

Identical to eight decimals across the span, and **bit-identical between left
and right under full weight shift**. The asymmetry is real and it is nowhere
near the attachment positions — it is in the tensions, where the same solve
puts the A row at **51 N left against 349 N right**, and the right A′ row at
0.27 N, essentially slack.

### What this actually means

**The missing ingredient is canopy torsional compliance, and it exists nowhere
in the stack.** Level 2's canopy is a rigid body; Level 6's membrane is 1-D
chordwise strips with no torsion. A differential row tension of 51 against 349
newtons has nothing to twist.

So the design's step 2 is done and its step 1 is blocked on a level that has
not been built. That reorders the work rather than only delaying it: the
question in front of the channel is no longer "wire it up" but **"does a
canopy on its lines twist several degrees under a 300 N row-tension
difference?"** — a structural question with a number attached, which is a far
better-posed thing to go and answer than the one this started with.

And the gain being linear means that answer settles it either way. A canopy
that twists a degree cannot be the mechanism, whatever else is built on top;
one that twists ten degrees closes item 21 and item 0b together.

**The general lesson, and it is the same shape as §71's:** a design derived
from what a solver *publishes* can be blocked by what that solver *represents*.
The line network computes every node position, so reading a pose off it looks
like arithmetic on available numbers — and it is, and the arithmetic returns a
constant, because the rigidity is upstream of the publishing. Checking what a
proposed input can vary, before building the thing that consumes it, is the
cheap version of this whole section.

Gated in `aerodynamics_tests`: zero offset is bit-identical to the design
pose, the sign is right, the gain is linear to under a percent across
sixteen times the range, twist is a couple, left and right mirror to 1e-9, and
full brake is worth several degrees of it.

## 73. The twist the aircraft needs is seven degrees, and it spirals at four — so canopy torsion is not the only thing in front of the turn

§72 left one number deciding whether canopy torsion is worth building: how much
twist is enough. It answered by dividing the roll moment full brake makes by
the channel's aerodynamic gain, and got about ten degrees. **That answers a
different question.** A steady turn is not held up by a steady roll moment —
once the wing is banked and turning, the moment keeping it there is not the
moment that put it there — so the twist that matches an existing control and
the twist the aircraft needs to turn need not be close.

`SetImposedSpanwiseTwistRad` measures the one that matters. It is an
instrument, on the pattern `SetSectionDragOffset` set: zero by default,
bit-identical to having no hook, and it is the only thing that ever writes the
channel, because the channel's structural half does not exist.

### The stable envelope, settled at 240 s

| twist | bank | turn | coordinated bank | ratio |
|---|---|---|---|---|
| 0.5° | 0.32° | 0.0130 rad/s | 0.80° | 39% |
| 1.0° | 0.64° | 0.0264 rad/s | 1.63° | 39% |
| 2.0° | 1.30° | 0.0543 rad/s | 3.40° | 38% |
| 3.0° | 2.20° | 0.0864 rad/s | 5.53° | 40% |

**0.0272 rad/s per degree, linear**, so a real EN-B's 0.20 rad/s wants **about
seven degrees** of antisymmetric twist. That is the number the canopy-torsion
level has to be measured against, and it supersedes §72's ten.

### But the aircraft spirals at four

At four degrees it does not settle. Incidence falls steadily from 4.7° to
below zero, speed climbs from 10.7 m/s past 21, and at about 35 s it snaps
into a spiral peaking at **3.48 rad/s**. The aerodynamic safety envelope
**never engages**, and the solve is no less converged than in straight flight
— `vsmConverged` is false at essentially every step of ordinary trim flight
too, so it carries no signal here (see below). This is the model's own
behaviour.

**So the ceiling on this aircraft's turn is not the twist a canopy can find.**
A stable turn tops out near 0.09 rad/s. A real wing's 0.2–0.3 is on the far
side of a spiral departure, and **perfect canopy torsion would not reach it.**

That reorders item 21 again, and in a more useful direction than §72 did.
§72 put it behind a structural level. This puts a *stability* problem in front
of the structural level — one that needs no new level to measure, is unowned by
any existing item, and would have to be solved before torsion could pay for
itself.

### And every turn it does fly is skidding, by a constant fraction

**WRONG, and §74 measures why — kept because the reading error is the useful
part.** Each stable point banks 38–40% of the coordinated bank for its own turn
rate and speed, and this section read the constancy of that fraction as a
missing mechanism: the aircraft yawing round without the bank following,
holding a steady sideslip in every turn.

The fraction is real and reproduces. The inference does not. `bankRad` is the
**canopy's** bank; the aircraft's 95 kg hangs on the payload link, and the
link sits at the coordinated bank within 2%. The sideslip, measured directly
rather than inferred, is under a tenth of a degree. The canopy sits inboard of
the link by exactly the angle the line roll spring needs to carry the twist's
steady roll moment — and both that moment and the coordinated bank are linear
in twist, which is why their ratio could not have varied. **The constancy was
arithmetic, not evidence.** See §74.

The hypothesis below is therefore closed too: the skid and the spiral are not
one fault, because there is no skid.

### `vsmConverged` reads false in straight flight, so it gates nothing

Found while checking whether the departure was numerical. Counting steps over
40 s of settled hands-up flight: **4180 of them report `vsmConverged` false**,
which is every step. The flight solve's 40-iteration cap does not reach the
1e-6 tolerance and is not meant to; the flag compares against it anyway. A
diagnostic that is false in the nominal case cannot distinguish the abnormal
one, which is what made it useless for exactly the question it was consulted
for. `PHYSICS_TODO` item 24.

**The lesson, which is §72's twice over:** both sections' first answer came
from dividing numbers that were already available, and both were answering an
adjacent question rather than the one asked. §72 divided two moments when the
question was about a flight path; the flag above compares against a tolerance
nothing is trying to meet. Flying the aircraft cost one instrument and forty
seconds of simulation, and it changed the plan.

## 74. There is no skid: the wing hangs at the coordinated bank, and §73 was reading the wrong angle

§73 measured that every stable turn banks 38-40% of the coordinated bank for
its own rate and speed, found the fraction constant across a sixfold input,
and concluded from the constancy that a mechanism was missing — that the wing
"yaws round without the bank following, holding a steady sideslip in every
turn." The fraction reproduces exactly. **The conclusion is wrong**, and item
25's skid half closes as a false finding rather than as a fix.

Three measurements, none of which existed when §73 was written:

| twist | canopy bank | canopy-to-link offset | link bank | coordinated | sideslip |
|---|---|---|---|---|---|
| 1° | 0.59° | 1.00° | **1.59°** | 1.63° | -0.02° |
| 2° | 1.30° | 2.06° | **3.35°** | 3.41° | -0.04° |
| 3° | 2.18° | 3.26° | **5.44°** | 5.53° | -0.07° |

- **The sideslip is zero**, to under a tenth of a degree, measured directly
  rather than inferred from the bank. A wing flying down its own centreline is
  not skidding, and that settles it without reference to any bank at all.
  **This is the load-bearing measurement**; the two below explain it rather
  than establish it.
- **The payload link is at the coordinated bank, within 2%.** Ninety-five of
  the 105 kg hangs on that link, so the link's lean is the angle a pilot flies
  and feels; the canopy's own bank tilts 5 kg of canopy. `bankRad` reports the
  canopy's, and §73 compared it against a formula about the other one.
  **Weaker evidence than it looks**, and worth saying so: the link is a
  pendulum driven by `g - a`, so it seeks apparent gravity *by construction*
  and could hardly land anywhere else. What it establishes is that the pilot
  feels a coordinated turn, not independently that the turn is one.
- **The canopy's shortfall is a spring deflection, and it balances.** The
  imposed twist puts +433 N·m of steady roll on the canopy at 3°, and the line
  roll spring returns -443 — a two-term equilibrium closing to 2%. That spring
  acts on the canopy-to-link *angle*, so carrying a steady moment **requires** a
  steady offset. The shortfall equals the offset to within a tenth of a degree
  at every point.
- **The flight path really is turning at the body yaw rate** (100%, 100%,
  100%), measured in the world. Assuming that would have been assuming the
  coordination under test, so it is measured.

### The turn is pulled round by a side force that is not a slip force

The lateral budget closes on two terms, to about 10%:

| twist | needs | aero side force | banked lift | sum |
|---|---|---|---|---|
| 1° | 29.2 N | 21.6 | 10.6 | 32.1 |
| 2° | 61.3 N | 44.4 | 23.3 | 67.7 |
| 3° | 99.7 N | 70.8 | 39.1 | 109.9 |

So there **is** a substantial aerodynamic side force — 71 N at 3°, 7% of the
aircraft's weight, and about two thirds of what the turn needs. It arrives at a
sideslip of 0.07°.

That combination is only strange for a flat wing. **This canopy is arched**,
so its sections' lift vectors have lateral components that cancel by symmetry
and stop cancelling the moment the wing is loaded antisymmetrically. The twist
is producing side force directly, without any slip to produce it with. Which is
the trap §73 fell into stated in one line: *"there is a side force"* and *"the
wing is slipping"* are the same sentence about a flat wing and different
sentences about this one.

**Why the constant fraction was the clue, read backwards.** The aero roll
moment is linear in twist and so is the coordinated bank; a ratio between two
linear terms in the same input *cannot* vary. §73 took the constancy as
evidence of a mechanism when it was evidence of arithmetic — the signature of
differencing two linear terms, which is the opposite of what it was read as.

**The lesson is §72's and §73's own, one turn further round.** Both of those
sections' first answers came from dividing numbers already to hand, and both
answered an adjacent question. This one came from a *diagnostic* already to
hand — `bankRad` — and the failure is the same shape: a quantity that was
close enough to the one the question was about that nobody checked which it
was. §73's fix was to fly the aircraft; the fix here is to measure the
specific angle the claim is about. Three new diagnostics
(`sideslipRad`, `aeroRollMomentNm`, `aeroSideForceN`) and one world-frame
heading rate.

**What survives.** The spiral departure between 3° and 4° of twist is
untouched and remains item 25's whole content: a stable turn still tops out
near 0.09 rad/s, a real EN-B's 0.2-0.3 is still past a departure, and canopy
torsion still would not reach it. What is gone is the hypothesis that the
spiral and the skid might be one fault — there is no second fault to be the
same one.

## 75. The spiral is not a spiral: it is the pitch divergence, and the turn is just another way of reaching it

§73 found the aircraft departing between 3° and 4° of imposed twist, called it
a spiral departure on the strength of the turn rate reaching 3.48 rad/s, and
left it as item 25 — a stability problem in front of the canopy-torsion level.
The same run reported incidence falling from 4.7° to below zero and speed
climbing past 21 m/s, which is the signature of a mechanism this stack has
carried a name for since the item 10 rewrite.

**Both departure boundaries are the same lift coefficient.**

| twist | settled CL | | bar | settled CL |
|---|---|---|---|---|
| 3.0° | 0.508 | | 0% | 0.551 |
| 3.4° | 0.491 | | 10% | 0.486 |
| 3.6° | 0.480 | | 15% | 0.478 |
| 3.8° | **0.461** | | 20% | **0.425** |
| 4.0° | departs | | 25% | departs |

The accelerator drives the same aircraft down the same CL scale with **no turn
at all**, so its boundary is the pitch axis's own. The two agree to 0.036,
which is *smaller than either sweep's own last step in CL* — 0.019 on twist,
0.053 on bar. They cannot be distinguished, and the test is written against
that resolution rather than against the gap that came out.

So **item 25's remaining half folds into item 11.** The turn is not a separate
instability; it is a second route to the same pitch divergence. That is the
whole of item 25 closed across §74 and §75: the skid was never there, and the
spiral is not its own fault.

### The CL 0.35 is optimistic by about 0.09

Both boundaries sit near **CL 0.44**, and the loop-gain analysis
(`a·c·Cm/(k·CL²)`, passing one at **CL 0.35**) is where the record has put the
divergence since the calibration work. That number is a static estimate off
the wing's own polar. The flown aircraft lets go about 0.09 of CL earlier, by
either route. Worth keeping, because 0.35 has been used to explain which
conditions are reachable — full bar as "a CL 0.31 condition" — and it makes
the envelope sound wider than it is.

### The argument that nearly carried this, and should not have

The first version asked a question of *order*: does CL cross 0.35 before the
turn rate runs away? It does — at 25.2 s against 25.9 s. **0.7 s out of 25,
between two quantities that are both running away exponentially by then.**
Which threshold trips first is a property of where the thresholds were put,
not of the aircraft. It would have given the right answer here for a reason
that does not generalise, and one change of threshold would have reversed it.

That is the same failure as §73's constant ratio and §74's wrong angle, in a
third costume: a number that *points at* the conclusion, accepted because the
conclusion was already believed. The fix each time is the same — find the
version of the claim that can come out the other way. Here that was the
accelerator control, which had no turn in it at all, and which was allowed to
refute the tidy answer right up until it didn't.

## 76. The wing's tolerance for low CL is the same at every damping ratio — so the coefficient buys excitation, not robustness

§75 put the aircraft's *static* envelope edge at CL 0.425 (accelerator) and CL
0.461 (imposed twist) — one edge, two routes. §54 had independently found the
finite-amplitude basin edge to be **nose-down**, at −2.6°, −2.5° and −5.8°
against a 5.1° trim, and said in as many words that this is "the same low-CL
loop-gain path as full bar and 40% brake rather than a stall."

That suggested a tidy unification, and it is worth stating because it is
**wrong**: that `swingDampingRatio` = 0.35 is not a missing mechanism at all,
but the least damping that keeps the growing phugoid's CL trough *inside* the
static envelope. Under that story item 11's eleven-level search for a
stabilising mechanism has been looking for something that need not exist.

### It is refuted, twice, and neither refutation is subtle

**Flying each ratio cold and watching CL rather than incidence** (900 s):

| ratio | lowest CL | at | outcome |
|---|---|---|---|
| 0.90 | 0.509 | 1 s | settled |
| 0.50 | 0.508 | 1 s | settled |
| 0.35 | 0.507 | 1 s | settled |
| 0.30 | **0.425** | 885 s | still moving |
| 0.25 | 0.007 | 469 s | **DEPARTED** |
| 0.20 | 0.001 | 92 s | **DEPARTED** |

Ratio 0.30 reaches **CL 0.425** — squarely at the static edge, to three digits
— and does not depart. Touching the edge is not what ends the flight.

**And the version with no window in it.** A growing oscillation's trough is
confounded with how long it was flown, so the question is asked without a
clock: from a settled trim, push the wing down the CL scale on a pure surge
kick and bisect the depth it comes back from. Pure surge deliberately, *not*
the phugoid eigendirection — §54 used that, and reusing it would make this a
re-run rather than a control.

| ratio | last recovered | first lost | trough there | kick's own CL |
|---|---|---|---|---|
| 0.90 | 3.94 m/s | 3.97 | **0.182** | 0.443 |
| 0.50 | 3.94 m/s | 3.97 | **0.181** | 0.443 |
| 0.35 | 3.94 m/s | 3.97 | **0.183** | 0.443 |

The wing recovers from a transit down to **CL 0.18**, less than half the static
edge. So the static edge and the transient point of no return are different
numbers by a factor of 2.4, and the unification does not join up.

### What replaces it is worth more than the hypothesis was

**The recovery edge does not move with the coefficient at all.** Same kick to
three digits (3.94 / 3.97 m/s), same trough to 0.002, across a factor of 2.6 in
`swingDampingRatio`. It is a property of the **wing**, and the coefficient has
no purchase on it whatever.

That narrows item 11 in a direction eleven levels of searching had not: whatever
`swingDampingRatio` is standing in for, **it does not change how much CL
excursion this wing survives — it changes how much excursion the phugoid
produces.** The coefficient acts on the excitation, not on the tolerance. A
"missing stabilising mechanism" that made the wing more robust to low-CL
transits would be the wrong shape of answer.

### A correction owed to §54, and its limit

§54 measured the basin along the **phugoid eigendirection** and found it
*growing* as the aircraft destabilises — 4.0, 4.0, 6.0 m/s survived at ratios
0.90, 0.50, 0.35. On a **fixed** direction the basin does not move at all. The
phugoid eigendirection is itself a function of the ratio, so at least part of
"the basin grows" is the direction rotating rather than the aircraft becoming
more tolerant.

**Stated as a limit rather than a correction**, because it was not tested
directly: two kick directions were used, not one direction and two readings.
§54's conclusion — that the finite-amplitude story is eliminated because the
basin does not *shrink* — is untouched either way, and is if anything
strengthened by a basin that does not move.

### The coincidence that is not claimed

The largest recovered kick places the wing at **CL 0.443**, and §75's static
edge is CL 0.425–0.461. That is a striking landing, from a measurement sharing
no arithmetic with either of §75's routes.

**It is not claimed.** It was not predicted in advance, it rests on a single
number, and the kick CL is the same at all three ratios for the trivial reason
that the bisection converged on the same kick. Recording it as an observation
with the prediction absent is exactly what §40 exists to enforce.

### Method note: a budget inherited without asking

The first run of this took **forty minutes** instead of four, because the new
check took the 3600 s settle budget from the block above it in the same file.
That budget exists so that "ratio 0.30 does not settle" is a statement about
the aircraft rather than the clock — a question neither of these checks asks.
Small, and the same shape as every other error on this axis: a quantity
carried over from a neighbouring context where it meant something else.

## 77. The departure is not a threshold crossing, and the boundary's *location* is the one part of it the schedule can move

§76 measured two edges and inferred a third thing. The edges: the transient
recovery edge at CL 0.18, which does not move with `swingDampingRatio`, and
§75's static envelope edge at CL 0.425–0.461. The inference, reached by
subtraction rather than measured: since tolerance is fixed and the boundary
moves, the coefficient must act on the excursion the phugoid *produces*.

That inference is sound and it was never a measurement. The sharp version:
**a departing wing leaves when its trough crosses the transient edge, not the
static one.** Written with four outcomes, three of them against it, and the
surviving ratios put in the same table as a free falsifier.

### Ratio 0.20 answers it, and the answer is "neither edge"

Five troughs, spaced 14.8 / 15.0 / 15.7 / 17.8 s — the phugoid, not an
identifier artefact:

| trough | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|
| CL | 0.481 | 0.442 | 0.380 | 0.287 | **0.014** |
| at | 26 s | 41 s | 56 s | 72 s | 90 s |
| α | 3.80° | 3.09° | 2.01° | 0.40° | −5.40° |

The wing flies **through** CL 0.425, 0.380 and 0.287 and keeps flying — so the
static edge is not the criterion, now confirmed from a second direction after
§76's ratio-0.30 result. And the terminal cycle runs 0.287 → 0.014, past CL
0.18 without ever resolving near it. The transient edge is not the criterion
either.

**What the sequence actually shows is a growth rate, not a threshold.** The
excursion below trim runs 0.06, 0.10, 0.16, 0.25 — a clean ×1.6 per cycle —
and then the last cycle overshoots that geometry badly instead of landing at
the ×1.6 prediction of ≈0.40. **The departure is where the growth stops being
exponential**, which is a statement about the nonlinearity and not about any
scalar the wing crosses.

This was the outcome predicted in advance, and for a stated reason: §53 found
transient amplification G = 13.9 while every mode decays, which already says
this system's behaviour is not a function of any one number. A surge kick
reaches CL 0.18 carrying 3.94 m/s of excess speed; the phugoid reaches the same
CL slowly with a different pitch rate. §76's factor of 2.4 was never two
estimates of one quantity.

### The identifier's control caught two things, and one was me

The first version printed the **last twelve troughs**, which on a departing row
are spaced 0.1–0.7 s — the collapse being sampled, not cycles of a 16 s mode.
The spacing column, printed next to every trough for exactly this reason,
made it obvious on sight. Splitting the list by *spacing* rather than by
*position* is what produced the table above.

That exposed the second thing: ratio 0.25 shows **415 troughs spaced closer
than 8 s**, starting at 177 s, spaced at 0.1 s — which is the aerodynamic hold
exactly. Two readings fitted, saying opposite things, so both knobs were turned:

| hysteresis | phugoid-like | closer | departs |
|---|---|---|---|
| 0.002 | 22 | 415 | 470 s |
| 0.010 | 24 | 91 | 470 s |
| 0.030 | 24 | 23 | 470 s |

Close troughs fall away by a factor of 18 while the phugoid count holds and the
departure time does not move by a millisecond. **Small ripple on a good
phugoid. Not a finding.** §36, §48 and §49 are three prior occasions on this
axis where an identifier returned a confident number instead of failing; this
is the first time the check was built in before the number was read.

### The lead that came out of the control, and the correction it needed

The other knob was not tidy. Over a 900 s window, ratio 0.25 departed at 163 s
at a 0.20 s hold, 470 s at 0.10 s, and **not at all** at 0.05 s. Item 11 carries
a bullet saying "it is not the schedule" — true, and it does not cover this: it
was measured on a *settling* wing at ratio 0.35, so there was no departure in
it. The stability boundary had never been tested against the schedule at all.
§47 had independently named this same 0.1 s hold as why Φ(T) is not an
exponential family.

**The sweep that followed refuted the exciting reading, using a column put in
to make that possible.** 1800 s, four times what the 0.10 s hold needs, with
the lowest CL printed beside every row:

| ratio | hold 0.05 s | hold 0.10 s | hold 0.20 s |
|---|---|---|---|
| 0.35 | settled, CL 0.511 | settled, CL 0.507 | settled, CL 0.497 |
| 0.30 | moving, CL **0.508**, 0.69° | moving, CL **0.386**, 5.9° | moving, CL **0.303**, 10.2° |
| 0.25 | moving, CL 0.224, 6.9° | DEPARTED 470 s | DEPARTED 163 s |
| 0.20 | DEPARTED **96 s** | DEPARTED **92 s** | DEPARTED **94 s** |

Ratio 0.25 at the fine hold reads "still moving" and its lowest CL is **0.224
against a trim of 0.54, with a 6.9° spread**. That is not a stable wing; it is a
slower one that had not crossed 20° inside the window. **"It does not depart at
0.05 s" was a statement about 900 seconds**, and the lowest-CL column exists so
that it could not survive as anything else.

### What is true: the classes are invariant, the rates are not

- **Ratio 0.35 settles at every hold** (CL 0.497–0.511, spread ≤0.01°).
- **Ratio 0.20 departs at 96 / 92 / 94 s** — unchanged across a **fourfold**
  change in the hold. The strongly unstable case does not care about the
  schedule at all, and it is the cleanest control row in the table.

So the instability is real and `swingDampingRatio` = 0.35 is not an artefact.
What the hold changes is **how fast marginal ratios develop**, and there the
trend is monotone and large: ratio 0.30's excursion at 1800 s goes CL 0.508 →
0.386 → 0.303 as the hold coarsens, i.e. from nearly settled to strongly
growing.

**And that is not free, because one live claim rests on exactly that quantity.**
§39 located the boundary "flown, between 0.35 and 0.30" on the strength of 0.30
*failing to settle inside a budget* — and failing to settle inside a budget is
precisely what moves with the hold. The stability classes at 0.35 and 0.20 are
schedule-invariant; **the boundary's location between them is not**, and it has
been quoted to two digits for several levels. Item 11's "it is not the schedule"
bullet is correct about the slow mode and does not extend to the boundary.

### The pattern, for the fourth time on this axis

§73's constant ratio, §74's wrong angle, §75's ordering argument — a number
that *points at* the conclusion, accepted because the conclusion was already
believed. This time the number was mine and it was one window wide, and what
caught it was not judgement but a column printed beside the answer because the
failure was anticipated in writing before the run. That is the only one of the
four that cost nothing.

## 78. A growth factor per cycle, and the coefficient's required value is partly the 10 Hz hold

§77 left a lead: peak-to-trough amplitude grows by a constant factor per cycle,
which is a *rate* — what §76 established the coefficient acts on — and unlike a
departure it exists at ratios that never depart, where it should sit below 1.
Every instrument this item has used is either a yes/no departure read against a
budget (and §77 showed the budget moves with the schedule) or an eigenvalue
(and §47 showed the slow mode's has no *T*-independent value). A per-cycle
growth factor `g` needs none of that.

**Amplitude is peak-to-trough, with no trim in it.** The obvious construction,
excursion below trim CL, cannot be computed at the two ratios that decide the
answer: 0.30 never settles and 0.25 departs, so neither has a trim.

### It is a real quantity — the first continuous one on this axis

`g` per cycle at the 0.05 s hold: ratio 0.30 returns **0.99 fifty-four times
running**, 0.35 returns 0.80 three times, 0.20 returns 1.61 / 1.59 / 1.64. The
amplitude sequence genuinely has one growth factor in it, and it is measurable
at stable ratios where a departure test has nothing to read.

Small-amplitude `g`, by ratio and aerodynamic hold:

| ratio | 0.025 s | 0.05 s | 0.10 s | 0.20 s |
|---|---|---|---|---|
| 0.40 | — | below floor | 0.68 | 0.73 |
| 0.35 | 0.788 | 0.802 | 0.830 | 0.887 |
| 0.30 | **0.977** | **0.994** | **1.029** | **1.100** |
| 0.25 | — | 1.26 | 1.30 | 1.39 |
| 0.20 | — | 1.61 | 1.66 | 1.78 |

### The control it was built for failed, and that is the finding

The reason to build `g` was the hope it would be schedule-invariant where
"settles inside 1800 s" is not. **It is not.** `g` moves monotonically with the
hold at every ratio — coarser hold, larger `g`, less stable. The intended use
is dead.

**What replaces it is worth more, because of where the movement lands.** Ratio
0.30 *straddles 1.0*: `g` = 0.977 and 0.994 at the 0.025 s and 0.05 s holds —
decaying — against 1.029 and 1.100 at 0.10 s and 0.20 s — growing. §77 could
only say the settle *time* moved with the schedule. This says the **sign of the
growth rate** moves, at the one ratio that decides where the boundary sits, and
says it with a quantity that has no budget in it anywhere.

**Ratio 0.30 is stable at holds of 0.05 s and finer. That is measured directly,
at two holds, with no extrapolation in it.**

### The extrapolation is available here, and §46 is why it had to be earned

The first draft of this section said the increments were the wrong shape for a
converged sequence — +0.030 then +0.017 for ratio 0.30 — and therefore that
extrapolating would repeat §46's error of reaching below the model's own
aerodynamic interval. **That reading was wrong, and it was wrong by reading the
wrong column.** It used the *mean* `g`, and at the 0.20 s hold the mean is
dragged from a first-cycle 1.100 down to 1.041 by the nonlinear decline over 51
cycles — the same effect §77 found in the last cycle before a departure.

On first-cycle `g`, the only one that is a linear rate, the increments are:

| ratio | 0.025→0.05 | 0.05→0.10 | 0.10→0.20 |
|---|---|---|---|
| 0.35 | +0.014 | +0.028 | +0.057 |
| 0.30 | +0.017 | +0.035 | +0.071 |

They **double for every doubling of the hold**. First order in *h*, exactly. So
the order is established rather than assumed, and Richardson extrapolation to
*h* → 0 gives, from three independent pairs:

| ratio | from 0.025/0.05 | from 0.05/0.10 | from 0.10/0.20 |
|---|---|---|---|
| 0.35 | 0.774 | 0.774 | 0.773 |
| 0.30 | 0.960 | 0.959 | 0.958 |

Agreement to 0.002 across pairs spanning an eightfold range of *h*. **This is
what §46 lacked**: there, `(Φ₀₀−1)/T` had no constant to extrapolate and its
order was never established, so a tidy factor came out with a story attached.
Here the order is confirmed by the increments and the limit by three concordant
pairs — and the load-bearing claim (0.30 is stable) does not rest on the
extrapolation at all, only the limit's *value* does.

### What this costs item 11

**Part of why `swingDampingRatio` must be 0.35 is the 10 Hz aerodynamic hold,
not the aircraft.** In the converged limit ratio 0.30 decays at 4% per cycle.
Not all of it is schedule: 0.20 departs at every hold with `g` ≈ 1.6–1.8 and
0.35 is stable at every hold, so the instability is real and the classes at the
extremes are invariant, exactly as §77 found. But **§39's location of the
boundary between 0.35 and 0.30 is not a property of the wing** — it was measured
at one hold, and at a converged hold the boundary is below 0.30.

### Recorded, not claimed

Reading the 0.05 s column, `g` goes 0.99 at ratio 0.30 and 1.26 at 0.25, so the
converged boundary sits somewhere near **0.28–0.29** — which is where the
*eigenvalue* put it (0.28–0.25), and §39 ruled against the eigenvalue precisely
because the flown answer said 0.35–0.30. The flown answer was taken at the
shipped 10 Hz hold.

**This is not claimed.** The eigenvalues were themselves computed at that same
0.1 s hold, so there is no reason yet why they should agree with a converged
flown value, and one number landing near another is what §40 exists to stop
being written up. The test it names is specific: **recompute the spectrum at
0.025 and 0.05 s holds and find where the crossing goes.** If the eigenvalue
crossing is hold-invariant while the flown one moves onto it, §39's verdict
inverts and the eigenvalue was right all along.

## 79. The eigenvalue was the hold-robust instrument all along, and §47 blamed the wrong culprit

§78 named this step: recompute the spectrum at finer aerodynamic holds and see
where the eigenvalue crossing goes. §39 had put the eigenvalue crossing at ratio
0.28–0.25 and the flown one at 0.35–0.30 and **ruled against the eigenvalue** —
"biased stable; the eigenvalue loses" — a verdict three later sections lean on.
§78 then found the flown crossing is not a property of the wing: it moves below
0.30 as the hold converges.

Both numbers were taken at the same 0.1 s hold, so §78 refused to claim the
inversion. This decides it, in **§78's currency** — the eigenvalue converted to
a per-cycle growth factor `g = exp(σ·period)` — so the two instruments print in
the same units and the disagreement is a number rather than two overlapping
brackets.

### The eigenvalue does not move. The flown measurement does.

`g` from the eigenvalue, across an eightfold range of hold:

| ratio | 0.025 s | 0.05 s | 0.10 s | 0.20 s | spread |
|---|---|---|---|---|---|
| 0.35 | 0.710 | 0.709 | 0.716 | 0.708 | **1.1%** |
| 0.30 | 0.881 | 0.880 | 0.887 | 0.876 | **1.3%** |
| 0.28 | 0.966 | 0.965 | 0.972 | 0.961 | 1.1% |
| 0.25 | 1.117 | 1.116 | 1.124 | 1.110 | 1.3% |

Against §78's flown `g` at ratio 0.30: 0.977 → 0.994 → 1.029 → 1.100, a **12%**
move. And the character differs, not just the size — the eigenvalue's 1% is
non-monotone scatter (0.10 s is the high point in every row, 0.20 s comes back
down), while the flown series is monotone and first-order in *h*.

**The eigenvalue crossing is between 0.28 and 0.25 at every hold** — σ negative
at 0.28 and positive at 0.25 in all four blocks. It does not move at all.

**So §39's verdict inverts on the question it actually turned on.** The
eigenvalue was the hold-robust instrument; the flown trace was measuring the
aircraft *plus its schedule*, and the outside number §39 leaned on
(`pitch_axis_trace --slow-mode`) was taken at the same 0.1 s hold as the flown
fit it agreed with. Two instruments sharing a defect agreed with each other,
and their agreement was read as confirmation.

### What §39 got right, and it is not nothing

The eigenvalue is *still* biased stable. Against §78's converged flown `g`
(Richardson, three concordant pairs): 0.711 against 0.774 at ratio 0.35, and
0.881 against 0.959 at 0.30. So §39's **characterisation** was correct even
though its verdict about which instrument to trust was backwards. The boundary
is not at 0.28–0.25 either.

**The converged flown boundary is between 0.30 and 0.28**, and that needs no
correction factor to state: at a converged hold ratio 0.30 decays at `g` = 0.959.

### Recorded, not claimed: the bias is a constant factor

Converged flown over eigenvalue: **1.0890** at ratio 0.35 and **1.0885** at
ratio 0.30. The same factor to four digits, at two ratios whose `g` differ by
24%.

**Not claimed.** It was not predicted in advance, it rests on **two ratios**,
and those are the only two where a converged flown `g` exists — 0.40 sat below
the amplitude floor and 0.25 departs before converging. A constant multiplicative
offset between a linearisation and a flown trace would be a real and useful
thing to know, and it is exactly the shape of result §40 exists to stop being
written up on two points. The test is more ratios with converged flown `g`.

### §47's attribution is refuted, and that came free

§47 found Φ(T) is not an exponential family — A₀₀ changing sign between T = 0.10
and T = 0.50 — and **blamed the 0.1 s aerodynamic hold**. That blame was never
tested and it makes a prediction: at a finer hold the T-dependence must shrink.

Spread between σ at T = 0.25 and T = 0.10, at ratio 0.35:

| hold | 0.025 s | 0.05 s | 0.10 s | 0.20 s |
|---|---|---|---|---|
| spread | 0.0038 | 0.0041 | 0.0044 | 0.0043 |

**It does not shrink.** An eightfold finer hold leaves the T-dependence within
12% of where it started, and nowhere near zero. §47's *observation* stands and
is confirmed here — σ moves ~20% between the two transition times at every hold
— but **its diagnosis is wrong**. The aerodynamic hold is not what makes Φ
non-exponential.

Two candidates remain, and they are separable:

- **The 120 Hz base timestep**, which was held fixed in every run here and in
  §47. It is the one schedule quantity that has never moved. Vary `timeStepS`
  and the spread either follows or it does not.
- **Hidden states, which would make this structural rather than numerical.**
  The solver carries membrane, pressure, line-network and damping-probe state
  outside the six reduced coordinates. A 6×6 Φ(T) read off a higher-dimensional
  linear system is a *projection*, and a projected transition operator is
  generically not an exponential family — that is a mathematical fact, not a
  discretisation artefact, and no schedule refinement would fix it.

If the second is right, §47's operational advice — "quote it with its T; do not
chase it" — was correct for the wrong stated reason, and the residual §50 found
outside the two-mode span is the same thing seen from another side.

## 80. The pitch spring is eliminated in both directions, and item 11 has no candidate left

The ledger after §79. The gap between the shipped `swingDampingRatio` = 0.35 and
this item's physical estimate of ~0.06 is 0.29, of which schedule and
measurement artefact account for ~0.05 (§78, §79), pilot-referenced drag plus
the lines sweeping 0.01 (§60, §63), and a credible pilot drag magnitude 0.05
(§59). **About 0.18 is unexplained**, and item 11's stated exit is unreachable
as written: at ratio 0.06 this wing departs, because the converged boundary is
0.29–0.30. Both named physical terms together moved that boundary by 0.01, so
whatever is missing is roughly **twenty times** their combined size.

Four eliminations pointed at one quantity — §76 (the coefficient acts on what
the phugoid *produces*), §45 (`d(surge)/d(surge)`, which is speed stability),
§39/§40 (retracted then re-supported), and §51/§54/§76 clearing everything
link-side. The quantity is §34's lift exponent, n = 0.171 against a classical 2,
because the canopy rotates nose-up on its lines at −1.69°/(m/s).

**The question was new even though n is not.** §35 asked whether n *crosses
zero*; §38 and §40 whether n or d *trend with the ratio*. Both are about n as
the crossing **variable** and both were answered correctly. Whether n's
**value** sets the boundary's **location** — n as a parameter — had never been
asked.

### Refuted, with the sign reversed

`SetPitchStiffnessScale`, a §6 diagnostic device (never to ship), multiplies the
line network's pitch spring. Trim-free per-cycle growth (§78) at the converged
0.025 s hold:

| spring | glide | g at 0.35 | 0.30 | 0.25 | boundary |
|---|---|---|---|---|---|
| ×1 | 10.97 | 0.788 | 0.977 | 1.235 | ~0.28 |
| ×2 | 10.49 | **0.985** | **1.225** dep. | departed | **~0.32** |
| ×4 | 6.52 | **1.001** | departed | departed | **≥0.35** |

**Stiffening moves the boundary up.** A wing whose canopy rotates *less* on its
lines needs *more* artificial damping. The prediction's direction was wrong —
the third time on this axis after §55's drag (+15% predicted, −21% measured).

### And the other side is untestable, which closes the lever rather than leaving it open

If stiffer is worse, softer is the interesting direction, and a two-sided lever
tested on one side is a half-run experiment. It was run:

| spring | glide | every ratio 0.35 → 0.10 |
|---|---|---|
| ×0.5 | **0.11** | departed, no cycles |
| ×0.25 | **0.71** | no amplitude — not flying |

Against ×1's 10.97 and a published 9.5. **Softening the pitch spring destroys
the trim before it can inform on the boundary.** Those rows are the device
wrecking the aircraft, not a measurement of it — §55's confound, flagged before
the first run, swamping the soft side entirely. The glide column is the licence
and it withheld it.

**So the pitch spring is eliminated as item 11's mechanism in both directions:
stiffer is measurably worse, and softer is unmeasurable.** The 0.18 has no
candidate left.

### What was worth having anyway

**The 10 Hz hold manufactures departures, a third time.** At the shipped hold
the ×2, ×4 and ×8 springs departed at 329, 202 and 113 s, reaching incidence
20–25° and glide 0.89–2.93. At 0.025 s none of them departs at all. After §78
(ratio 0.25) and §79 (the flown boundary), this is the third distinct occasion
the shipped schedule has produced a departure that is not in the aircraft.

**The boundary curve, trim-free and schedule-converged**, which did not exist
before: ratio 0.35 → 0.788, 0.30 → 0.977, 0.25 → 1.235, 0.20 → 1.586,
0.15 → 2.132, 0.10 → departs before a cycle completes.

**A note on §34 and §35 that stands unresolved.** §34 reads the flat lift curve
as why the mode is weakly restored and lightly damped — a defect. §35 found
dα/dV going −1.27 to −1.72 as the ratio *rises*, i.e. more tracking alongside
more stability. Those two have sat in the same item for dozens of sections
pointing opposite ways, and this section noticed it without being able to
resolve it, because the lever that would have is the one just eliminated.

### Method note: two of four runs were my own instrument, and both lessons were already in this file

The first run returned six blocks of "no trim" and could not say why. Two causes,
both self-inflicted:

- It asked for **own trims at ratios 0.30 and below**, which §78 had already
  shown do not settle. §79 had solved this — settle once at 0.35 and sweep off
  that state — and the structure was not reused.
- It collapsed *departed*, *still moving* and *budget exhausted* into one
  boolean, which is exactly the distinction §77 built its lowest-CL column to
  preserve. Applied there, dropped here.

And the third run reached for the **eigenvalue**, which needs a trim, for rows
whose defining property is having none — when §78 had built a trim-free
instrument for that case two sections earlier. **The failure mode is not
ignorance of the lesson but failure to retrieve it**, three times in one
investigation, each time on a lesson written down in this same document.

## 81. A component verified against a published number, wired into something that changes it, and nothing re-checked the assembly

Level 11 strand 1 implemented R. T. Jones' two-exponential Wagner function and
verified it against his published values. That verification is correct and it
still is. Strand 2 then wired it into the circulation solve, gated the wiring
on a collapse's mirror symmetry, and passed. Between the two, **nothing asked
the assembled solver the question strand 1 had asked the component**: does the
wing's circulation now follow Φ(s)?

It does not. It closes **12% of a step where Φ(0) is 0.5**, and 54% at nine
semichords where Φ is 0.87. Two mechanisms, and they are independent:

1. **The aerodynamic states were advancing by the simulation step** while
   `SolveUnsteady` runs once every `aerodynamicsInterval` steps — so at the
   shipped 6, six times slow. This is arithmetic, not modelling, and it reaches
   further than the flag: the separation state, which is stall's memory, has
   had it since it was written.
2. **The one-pass solve carries lag of its own.** A single explicit pass across
   sections is itself a relaxation in time, so the wing is lagged twice and
   only one of the two is published. Not yet identified further, and it should
   not be guessed at.

**The reusable part is the shape of the gap, not the numbers.** A component
gate answers "is this function right". An integration gate answers "does the
assembly do something I recognise". Strand 2's gate was the second kind — a
collapse holding its symmetry — and a symmetry test cannot tell a correctly
lagged wing from an over-lagged one, because *more* lag passes it *better*. The
gate was not weak; it was measuring a different property, and its passing was
read as covering both.

- **The tell was available and was not read.** Strand 2's own write-up notes
  that `converged` reports false in the lagged mode and that the residual "is
  the distance still to travel". A distance still to travel, at a settled trim,
  with a published function saying exactly how far it should be — that is the
  check, one line from where it was written down.
- **What made it findable was asking a different question entirely.** The
  session's question was whether the lag should SHIP, which meant measuring the
  flight in between rather than the one event the gate covers. The step
  response was included because item 19 had noted the transient was "the right
  order to matter for gust and collapse response, where nothing has yet looked
  for it". Two open notes, in different items, pointing at the same unmeasured
  place.
- **And the correction's own test was schedule-independence**, which is worth
  keeping as the general form: an aerodynamic state's response should not
  depend on how often the aerodynamics are solved. Before, interval 6 and
  interval 1 disagreed by a factor of two at 1.0 s; after, 0.523 against 0.544.
  That test would have caught defect 1 at any point in the last several levels,
  and no instrument in this project was asking it.

**The cost is recorded rather than paid.** Correcting the elapsed time reddens
seven collapse gates including strand 2's own, so it landed behind a flag that
defaults to the *wrong* behaviour — the same pattern this project uses for the
geometric channel and for item 11's two terms, and for the same reason: a
correct change whose consequences have not been re-derived is not yet a shipped
change.

## Numbers worth remembering

| quantity | value | why it matters |
|---|---|---|
| trim dynamic pressure | ~65 Pa | cell pressure, membrane load |
| cell pressure coefficient at trim | 0.97 | inlets sit near stagnation |
| line stretch at trim | 0.5 cm | small but not negligible |
| membrane strain at trim | 0.06% | the skin is effectively inextensible |
| installed drag | 47% of canopy drag | not a correction term |
| apparent mass, normal | 33.6 kg | a third of the aircraft |
| pendulum stiffness | ~7000 N·m/rad | dominates pitch and roll |
| roll damping time constant | 20 ms | sets the maximum aero interval |
| coupled trim speed | 8.86 m/s | 31.9 km/h against a published 39 - see §20 |
| coupled trim incidence | 9.1 deg | it was 4.5 pinned, 11.8 before the Cm fixes |
| section Cm at 3.5% camber | -0.110 | -pi h/c; the code had -pi h/4c |
| incidence for the published trim CL | 5.30 deg | the lift curve is close to right |
| line pitch stiffness at 1 g | 6317 N·m/rad | and PROPORTIONAL TO LOAD - §21 |
| line stiffness per newton | 6.13 m | 3306/6317/11512/15393 at ½/1/2/4 g |
| pitch hinge arm | 6.62 m | the canopy pivots below itself, not about itself |
| line roll stiffness at 1 g | 8204 N·m/rad | replaced a `W L sin` term |
| trim speed, 105 kg | 11.06 m/s | 39.8 km/h against a published 39.0 |
| trim incidence | 5.14 deg | against the 5.30 the published CL needs |
| glide at trim | 11.33 | against a published 9.5 - the drag deficit |
| brake rotation on the lines | 3.6 deg | between 19% and 40% of travel |
| camber couple at trim | 327 N·m | scales with q; drives the pitch loop gain |
| pitch loop gain at trim | 0.32 | passes 1 at CL 0.35; full bar is CL 0.31 |
| section CLmax, analytic | 0.866 at every brake | a stated stall margin cannot move |
| section CLmax, computed | 1.81 hands up, 2.48 at 40% brake | it moves because the flap is real |
| leading-edge bubble limit | 3% of chord | a turbulent separation forward of this reattaches |
| section Cd at trim | 0.0157 | published paraglider sections: 0.018-0.025 |
| twist-to-roll gain | 8543 N·m/rad | linear to 0.03%; divides a roll requirement into a twist |
| twist worth of full brake | ~10 deg | superseded by the aircraft measurement below - §72 |
| turn rate per degree of twist | 0.0272 rad/s | linear to 3 deg; **0.20 rad/s wants ~7 deg** - §73 |
| stable turn ceiling | ~0.09 rad/s | 4 deg of twist departs - and it is a PITCH departure - §75 |
| departure CL, by twist | 0.461 last stable | the turn envelope's edge - §75 |
| departure CL, by accelerator | 0.425 last stable | the SAME edge, no turn involved - §75 |
| loop gain's CL 0.35 | optimistic by ~0.09 | the flown aircraft departs near CL 0.44 - §75 |
| transient recovery edge | CL 0.18 | the wing comes back from this; HALF the static edge - §76 |
| recovery edge vs damping ratio | does not move | 0.182/0.181/0.183 over a 2.6x range - §76 |
| ratio 0.20's phugoid troughs | 0.481/0.442/0.380/0.287/0.014 | flies THROUGH both edges - §77 |
| what ends the flight | growth ceasing to be exponential | not a CL threshold at all - §77 |
| excursion growth per cycle, ratio 0.20 | x1.6 for four cycles | the fifth overshoots it - §77 |
| ratio 0.20 departure vs aero hold | 96 / 92 / 94 s | INVARIANT over a 4x hold change - §77 |
| ratio 0.30 excursion at 1800 s, by hold | CL 0.508 / 0.386 / 0.303 | rate near the boundary is NOT - §77 |
| what the 0.1 s hold can move | the boundary's LOCATION | §39's 0.35-0.30 rests on it - §77 |
| per-cycle growth g, ratio 0.30 | 0.977/0.994/1.029/1.100 | STRADDLES 1 over holds 0.025-0.20s - §78 |
| the same, ratio 0.35 | 0.788/0.802/0.830/0.887 | stable at every hold - §78 |
| the same, ratio 0.20 | 1.61/1.66/1.78 | unstable at every hold - §78 |
| g's convergence order in the hold | first order, increments double | established, not assumed - §78 |
| g extrapolated to zero hold | 0.774 (r 0.35), 0.959 (r 0.30) | three Richardson pairs agree to 0.002 - §78 |
| ratio 0.30 at holds <= 0.05 s | STABLE, measured directly | no extrapolation in this one - §78 |
| converged boundary, by g | near 0.28-0.29 | where the EIGENVALUE put it - unclaimed - §78 |
| eigenvalue g vs aero hold | 1.1-1.3%, non-monotone | HOLD-INVARIANT over 8x - §79 |
| flown g vs aero hold, same ratio | 12%, monotone | the flown trace carries the schedule - §79 |
| eigenvalue crossing vs hold | 0.28-0.25 at every hold | does not move at all - §79 |
| §39's "the eigenvalue loses" | INVERTED | it was the hold-robust one - §79 |
| converged flown boundary | between 0.30 and 0.28 | needs no correction factor - §79 |
| converged flown / eigenvalue g | 1.0890 and 1.0885 | same factor, two ratios, UNCLAIMED - §79 |
| §47's blame on the 0.1 s hold | REFUTED | T-spread 0.0038 to 0.0044 over 8x - §79 |
| what makes Phi non-exponential | base timestep, or hidden states | both untested; separable - §79 |
| boundary vs pitch spring x1/x2/x4 | ~0.28 / ~0.32 / >=0.35 | STIFFER IS WORSE - refuted, sign reversed - §80 |
| pitch spring x0.5 / x0.25 | glide 0.11 / 0.71 | softer is UNMEASURABLE; lever is one-sided - §80 |
| flown g, x1, converged hold | 0.788/0.977/1.235/1.586/2.132 | ratios 0.35 to 0.15, trim-free - §80 |
| stiff springs at the 10 Hz hold | depart at 329/202/113 s | numerical; none departs at 0.025 s - §80 |
| item 11's unexplained remainder | ~0.18 of coefficient | NO candidate mechanism left - §80 |
| CANOPY bank as a fraction of coordinated | 38-40% | the roll spring's deflection, NOT a skid - §74 |
| link bank against coordinated | within 2% | but the link seeks apparent gravity by construction - §74 |
| sideslip in a steady turn | under 0.1 deg | measured, not inferred; there is no skid - §74 |
| aero side force at 3 deg twist | 71 N | 2/3 of the turn's centripetal force, AT ZERO SLIP - the arc - §74 |
| section Cm per rad of brake | -0.61 | theory -0.55; the analytic table had -0.34 |
| usable envelope | hands-up to 25% brake | now BOTH ends are the pitch axis |
| wing's free hang angle | 4.75 deg nose-up | what sets trim incidence |
| canopy lead at trim | 0.77 m | and 1.85 m at the top of a surge |
| brake slack take-up | 19% of handle travel | below it the trailing edge does not move |
| tip line gap, built graph | 0.178 m | how far a fold must reach to cravat |
| gust that folds a half wing | 4 m/s descending, 1 s | 0.70 fold, full recovery |
| gust the wing does not survive | 5 m/s and up | deep-stall attractor, 7.5 m/s vertical |
| phugoid lift exponent, L ~ V^n | 0.171 | classical 2; this is the long period - §34 |
| phugoid drag exponent, D ~ V^d | 0.313 | classical 2; this is the low damping |
| incidence against speed, slow mode | -1.69 deg/(m/s) | the pendulum holds lift, not incidence |
| lift swing over the slow mode | 0.97% of weight | what is left to restore with |
| fast pitch mode, by eigenvalue | 1.86 s, zeta 0.09 | the authority; the gate no longer claims otherwise - §48 |
| the old gated "2.91 s / 0.28" | a window artefact | 1.42 / 1.51 / 2.91 / nothing, by window end - §48 |
| slow mode, by eigenvalue | 16.40 s, zeta 0.033 | against 16.39 / 0.031 off a trace |
| mode that diverges at low swing damping | 3.6-5.7 s | RETIRED: a peak-counter artefact - §49 |
| two-mode mixture under §35's peak counter | 2.46 to 9.87 s apparent | includes 3.64-5.17 and 2.91 - §49 |
| residual of the flown trace off the two modes | 0.4-5% while the motion is small | no third direction carries it - §50 |
| §35's counter on the flown trace | 6.34 s | neither 1.86 nor 15.4 - §50 |
| the same counter on a two-mode REBUILD of it | 7.80 s (6.90 at T=0.10) | the gap period out of a signal with no gap mode - §50 |
| canopy-referenced damper, slow mode at 0.35 | sigma +0.156 | against the world damper's -0.021 - §51 |
| the same, ratio 0.10 to 0.90 | +0.136 rising to +0.194 | MORE damping is more unstable; no value works - §51 |
| its fast mode | -0.68 /s, still 1.89 s | the dragged-pendulum story named the wrong mode - §51 |
| canopy-damped cold start, ratio 0.90 / 0.35 | departs at 17 s / 27 s | the solver's remembered "twenty seconds" - §51 |
| construction cost, suspension probes | 336 ms of 355 | 35 relaxations, 12000 iterations each - item 14 |
| roll spring at 1 g, shipped vs converged | 8116 vs 8254 Nm/rad | the probes stop mid-ring, 1.7% out - §52 |
| the same, held 8000 at 0.997 retention | 8262, worst 0.24% over 0.5-4 g | faster AND closer, and not shipped - §52 |
| deep frontal peak rotation, four settings | 2.06 / 3.61 / 3.90 / 71.0 rad/s | static outputs agree to 1.7%; this does not - §52 |
| transient amplification G, ratio 0.90 | 13.9 while every mode decays | the eigenvalues predict 1.00 - §53 |
| the same, ratio 0.25 | 9.0 | it FALLS as the wing destabilises - §53 |
| when it peaks | 0.4-0.5 s | the departure takes tens of seconds - §53 |
| its damping at ratio 0.25 / 0.20 | -0.017 / -0.042 | boundary is between 0.25 and 0.30 |
| fast mode's real part, ratio 0.90 to 0.10 | -0.357 to -0.291 /s | it never crosses; not the departure - §38 |
| mode whose damping DOES cross | the 16 s phugoid | not the pendulum - §38 |
| where it crosses, FLOWN | ratio 0.35 to 0.30 | SUPERSEDED: taken at the 0.1 s hold - §79 |
| where it crosses, by eigenvalue | ratio 0.28 to 0.25 | hold-invariant; §39's verdict INVERTED - §79 |
| where it crosses, converged | between 0.30 and 0.28 | the number to use - §79 |
| phugoid period, ratio 0.90 to 0.10 | 23.9 s to 14.0 s | the frequency never goes imaginary |
| lowest ratio with a trim at all | 0.30 | 0.25 departs at 348 s of its own settle |
| slow mode at 0.35, flown | 16.38 s, zeta 0.0299 | against 16.39 / 0.031 from a 1200 s trace |
| the same, by eigenvalue at T=0.25 | zeta 0.0540 | high by 3/4; §37's linearity check said so |
| what `swingDampingRatio` 0.35 IS | the phugoid's stability edge | not a margin above one - §39 |
| drag exponent d, ratio 0.50 to 0.28 | 0.281 rising to 0.459 | it never crosses zero - §40 |
| zeta predicted by §34 over that sweep | 0.034 rising to 0.051 | flown zeta FALLS 0.160 to -0.017 |
| §34's period prediction over the sweep | within 1-4% at every ratio | the frequency half still holds |
| link/attitude articulation, fast vs slow | 1.07 vs 0.29 | the fast mode IS the pendulum - §41 |
| link-to-surge phase in the 16 s mode | -107 to -109 deg | constant across the sign change - §41 |
| link/speed in the 16 s mode, 0.90 to 0.25 | 0.319 to 0.510 | amplitude moves where phase does not |
| phugoid adjoint on the link's rows | 0.78 scaled (0.985 raw) | link-dominated; the raw figure was unit-flattered - §43 |
| the same, ratio 0.90 to 0.25 | 0.89 falling to 0.76 | it listens LESS as it destabilises - §43 |
| phugoid mode conditioning, \|w^H v\| | 0.10 | non-normal: looks unlike what it listens to |
| share of d(sigma) entering rows 4-5 | 99% | near-tautological; the coefficient lives there |
| d(sigma) per 1% on d(swing)/d(swing) | +0.0133 /s | the whole 0.35->0.30 step is +0.0129 - §44 |
| top three sensitivities, link rows | 1.33, 1.31, 1.26 | all in the swing ANGLE row - §44 |
| largest sensitivity in the whole matrix | d(surge)/d(surge), +1.63 | a WING row, worth 1.25 coefficient steps - §45 |
| sensitivity rms, link rows vs wing rows | 0.822 vs 0.433 | link block more sensitive on average, peak is not |
| (Phi00-1)/T, T = 0.10 to 0.50 | -0.082 to -0.19 | NOT a constant; there is no single A00 - §46 |
| A00 extrapolated vs drag-exponent value | -0.052 vs -0.028 /s | same order, factor 1.84 - §46 |
| A00 = log(Phi)/T at T = 0.10 / 0.25 / 0.50 | -0.035 / -0.021 / +0.006 | it CHANGES SIGN: no continuous A - §47 |
| why the slow mode's damping never converged | Phi is not an exponential family | it has no T-independent value - §47 |
| §45's ranking at T = 0.10 vs 0.25 | same top four; rms ratio 1.87 vs 1.89 | entry ranking survives; magnitudes scale |
