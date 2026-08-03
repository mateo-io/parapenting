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
| section Cm per rad of brake | -0.61 | theory -0.55; the analytic table had -0.34 |
| usable envelope | hands-up to 25% brake | now BOTH ends are the pitch axis |
| wing's free hang angle | 4.75 deg nose-up | what sets trim incidence |
| canopy lead at trim | 0.77 m | and 1.85 m at the top of a surge |
| brake slack take-up | 19% of handle travel | below it the trailing edge does not move |
| tip line gap, built graph | 0.178 m | how far a fold must reach to cravat |
| gust that folds a half wing | 4 m/s descending, 1 s | 0.70 fold, full recovery |
| gust the wing does not survive | 5 m/s and up | deep-stall attractor, 7.5 m/s vertical |
